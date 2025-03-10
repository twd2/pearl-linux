// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for TI TPS6598x USB Power Delivery controller family
 *
 * Copyright (C) 2017, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/usb/typec.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>
#include <linux/usb/typec_altmode.h>

#include "tps6598x.h"
#include "trace.h"

/* Register offsets */
#define TPS_REG_VID			0x00
#define TPS_REG_PID			0x01
#define TPS_REG_MODE			0x03
#define TPS_REG_CMD1			0x08
#define TPS_REG_DATA1			0x09
#define TPS_REG_INT_EVENT1		0x14
#define TPS_REG_INT_EVENT2		0x15
#define TPS_REG_INT_MASK1		0x16
#define TPS_REG_INT_MASK2		0x17
#define TPS_REG_INT_CLEAR1		0x18
#define TPS_REG_INT_CLEAR2		0x19
#define TPS_REG_STATUS			0x1a
#define TPS_REG_POWER_STATE		0x20
#define TPS_REG_SYSTEM_CONF		0x28
#define TPS_REG_CTRL_CONF		0x29
#define TPS_REG_POWER_STATUS		0x3f
#define TPS_REG_RX_IDENTITY_SOP		0x48
#define TPS_REG_DATA_STATUS		0x5f

/* devices used on M1 Macs for Thunderbolt */
#define PID_CD3217			0x00cd3217
#define PID_CD3218			0x00cd3218

/* TPS_REG_INT_* bits */
#define TPS_REG_INT_PLUG_EVENT		BIT(3)

#define TPS_CD321X_INT_MASK		0x04c000000000050full
#define TPS_CD321X_INT_STATUS_CHG	BIT(8)
#define TPS_CD321X_INT_DATA_STATUS_CHG	BIT(10)

#define CD321X_SETTLE_MSEC		1000

/* TPS_REG_STATUS bits */
#define TPS_STATUS_PLUG_PRESENT		BIT(0)
#define TPS_STATUS_ORIENTATION(s)	(!!((s) & BIT(4)))
#define TPS_STATUS_PORTROLE(s)		(!!((s) & BIT(5)))
#define TPS_STATUS_DATAROLE(s)		(!!((s) & BIT(6)))
#define TPS_STATUS_VCONN(s)		(!!((s) & BIT(7)))

/* TPS_REG_POWER_STATE values */
#define TPS_POWER_STATE_S0		0
#define TPS_POWER_STATE_BOOT		7

/* TPS_REG_SYSTEM_CONF bits */
#define TPS_SYSCONF_PORTINFO(c)		((c) & 7)

/* TPS_REG_DATA_STATUS bits */
#define TPS_DATA_STATUS_CABLE_GEN_M	GENMASK(29,28)
#define TPS_DATA_STATUS_CABLE_GEN_S	28
#define TPS_DATA_STATUS_CABLE_SPEED_M	GENMASK(27,25)
#define TPS_DATA_STATUS_CABLE_SPEED_S	25
#define TPS_DATA_STATUS_CABLE_LINKTRN	BIT(20)
#define TPS_DATA_STATUS_CABLE_OPTICAL	BIT(18)
#define TPS_DATA_STATUS_CABLE_LEGACY	BIT(17)
#define TPS_DATA_STATUS_TBT_CONNECTION	BIT(16)
#define TPS_DATA_STATUS_USB3_CONNECTION	BIT(5)
#define TPS_DATA_STATUS_USB2_CONNECTION	BIT(4)
#define TPS_DATA_STATUS_CABLE_ACTIVE	BIT(2)

enum {
	TPS_PORTINFO_SINK,
	TPS_PORTINFO_SINK_ACCESSORY,
	TPS_PORTINFO_DRP_UFP,
	TPS_PORTINFO_DRP_UFP_DRD,
	TPS_PORTINFO_DRP_DFP,
	TPS_PORTINFO_DRP_DFP_DRD,
	TPS_PORTINFO_SOURCE,
};

/* TPS_REG_RX_IDENTITY_SOP */
struct tps6598x_rx_identity_reg {
	u8 status;
	struct usb_pd_identity identity;
} __packed;

/* Standard Task return codes */
#define TPS_TASK_TIMEOUT		1
#define TPS_TASK_REJECTED		3

enum {
	TPS_MODE_APP,
	TPS_MODE_BOOT,
	TPS_MODE_BIST,
	TPS_MODE_DISC,
};

static const char *const modes[] = {
	[TPS_MODE_APP]	= "APP ",
	[TPS_MODE_BOOT]	= "BOOT",
	[TPS_MODE_BIST]	= "BIST",
	[TPS_MODE_DISC]	= "DISC",
};

/* Unrecognized commands will be replaced with "!CMD" */
#define INVALID_CMD(_cmd_)		(_cmd_ == 0x444d4321)

struct tps6598x {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock; /* device lock */
	u8 i2c_protocol:1;
	u8 i2c_no_long:1;
	u8 just_init:1;
	u8 cd321x_support:1;
	u32 product_id;

	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_pd_identity partner_identity;
	struct usb_role_switch *role_sw;
	struct typec_capability typec_cap;

	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum power_supply_usb_type usb_type;
	u32 pwr_status, data_status;

	struct typec_usb4_cable cable_info;

	struct delayed_work cd321x_status_work;
};

static enum power_supply_property tps6598x_psy_props[] = {
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_usb_type tps6598x_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
};

static const char *tps6598x_psy_name_prefix = "tps6598x-source-psy-";

/*
 * Max data bytes for Data1, Data2, and other registers. See ch 1.3.2:
 * https://www.ti.com/lit/ug/slvuan1a/slvuan1a.pdf
 */
#define TPS_MAX_LEN	64

static int
tps6598x_block_read(struct tps6598x *tps, u8 reg, void *val, size_t len)
{
	u8 data[TPS_MAX_LEN + 1];
	int ret;

	if (WARN_ON(len + 1 > sizeof(data)))
		return -EINVAL;

	if (!tps->i2c_protocol)
		return regmap_raw_read(tps->regmap, reg, val, len);

	ret = regmap_raw_read(tps->regmap, reg, data, tps->i2c_no_long ? len + 1 : sizeof(data));
	if (ret)
		return ret;

	if (data[0] < len)
		return -EIO;

	memcpy(val, &data[1], len);
	return 0;
}

static int tps6598x_block_write(struct tps6598x *tps, u8 reg,
				const void *val, size_t len)
{
	u8 data[TPS_MAX_LEN + 1];

	if (!tps->i2c_protocol)
		return regmap_raw_write(tps->regmap, reg, val, len);

	data[0] = len;
	memcpy(&data[1], val, len);

	return regmap_raw_write(tps->regmap, reg, data, tps->i2c_no_long ? len + 1 : sizeof(data));
}

static inline int tps6598x_read8(struct tps6598x *tps, u8 reg, u8 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u8));
}

static inline int tps6598x_read16(struct tps6598x *tps, u8 reg, u16 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u16));
}

static inline int tps6598x_read32(struct tps6598x *tps, u8 reg, u32 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u32));
}

static inline int tps6598x_read64(struct tps6598x *tps, u8 reg, u64 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u64));
}

static inline int tps6598x_write16(struct tps6598x *tps, u8 reg, u16 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u16));
}

static inline int tps6598x_write32(struct tps6598x *tps, u8 reg, u32 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u32));
}

static inline int tps6598x_write64(struct tps6598x *tps, u8 reg, u64 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u64));
}

static inline int
tps6598x_write_4cc(struct tps6598x *tps, u8 reg, const char *val)
{
	return tps6598x_block_write(tps, reg, val, 4);
}

static int tps6598x_read_partner_identity(struct tps6598x *tps)
{
	struct tps6598x_rx_identity_reg id;
	int ret;

	ret = tps6598x_block_read(tps, TPS_REG_RX_IDENTITY_SOP,
				  &id, sizeof(id));
	if (ret)
		return ret;

	tps->partner_identity = id.identity;

	return 0;
}

static void tps6598x_set_data_role(struct tps6598x *tps,
				   enum typec_data_role role, bool connected)
{
	enum usb_role role_val;

	if (role == TYPEC_HOST)
		role_val = USB_ROLE_HOST;
	else
		role_val = USB_ROLE_DEVICE;

	if (!connected)
		role_val = USB_ROLE_NONE;

	usb_role_switch_set_role(tps->role_sw, role_val);
	typec_set_data_role(tps->port, role);
}

static int tps6598x_connect(struct tps6598x *tps, u32 status)
{
	struct typec_partner_desc desc;
	enum typec_pwr_opmode mode;
	u16 pwr_status;
	u32 data_status;
	unsigned long data_mode;
	int ret;

	ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
	if (ret < 0)
		return ret;

	ret = tps6598x_read32(tps, TPS_REG_DATA_STATUS, &data_status);
	if (ret)
		return ret;

	if (tps->partner) {
		if (tps->pwr_status == pwr_status && tps->data_status == data_status)
			return 0;

		if (!IS_ERR(tps->partner))
			typec_unregister_partner(tps->partner);
		tps->partner = NULL;
	}

	mode = TPS_POWER_STATUS_PWROPMODE(pwr_status);

	desc.usb_pd = mode == TYPEC_PWR_MODE_PD;
	desc.accessory = TYPEC_ACCESSORY_NONE; /* XXX: handle accessories */
	desc.identity = NULL;

	if (desc.usb_pd) {
		ret = tps6598x_read_partner_identity(tps);
		if (ret)
			return ret;
		desc.identity = &tps->partner_identity;
	}

	if(tps->cd321x_support && (data_status & TPS_DATA_STATUS_TBT_CONNECTION)) {
		data_mode = TYPEC_MODE_USB4;
		tps->cable_info.gen = (data_status & TPS_DATA_STATUS_CABLE_GEN_M) >>
				      TPS_DATA_STATUS_CABLE_GEN_S;
		tps->cable_info.speed = (data_status & TPS_DATA_STATUS_CABLE_SPEED_M) >>
					TPS_DATA_STATUS_CABLE_SPEED_S;
		tps->cable_info.link_training =
				!!(data_status & TPS_DATA_STATUS_CABLE_LINKTRN);
		tps->cable_info.is_optical =
				!!(data_status & TPS_DATA_STATUS_CABLE_OPTICAL);
		tps->cable_info.is_active =
				!!(data_status & TPS_DATA_STATUS_CABLE_ACTIVE);
		tps->cable_info.is_legacy_adapter =
				!!(data_status & TPS_DATA_STATUS_CABLE_LEGACY);
	} else if(data_status & TPS_DATA_STATUS_USB3_CONNECTION)
		data_mode = TYPEC_MODE_USB3;
	else if(data_status & TPS_DATA_STATUS_USB2_CONNECTION)
		data_mode = TYPEC_MODE_USB2;
	else
		data_mode = 0;

	typec_set_pwr_opmode(tps->port, mode);
	typec_set_pwr_role(tps->port, TPS_STATUS_PORTROLE(status));
	typec_set_vconn_role(tps->port, TPS_STATUS_VCONN(status));
	typec_set_orientation(tps->port, TPS_STATUS_ORIENTATION(status) ?
		TYPEC_ORIENTATION_REVERSE : TYPEC_ORIENTATION_NORMAL);
	tps6598x_set_data_role(tps, TPS_STATUS_DATAROLE(status), true);
	if(data_mode == TYPEC_MODE_USB4)
		typec_set_mode_data(tps->port, data_mode, &tps->cable_info);
	else
		typec_set_mode(tps->port, data_mode);

	tps->partner = typec_register_partner(tps->port, &desc);
	if (IS_ERR(tps->partner))
		return PTR_ERR(tps->partner);

	if (desc.identity)
		typec_partner_set_identity(tps->partner);

	power_supply_changed(tps->psy);

	return 0;
}

static void tps6598x_disconnect(struct tps6598x *tps, u32 status)
{
	if (!IS_ERR(tps->partner))
		typec_unregister_partner(tps->partner);
	tps->partner = NULL;
	typec_set_pwr_opmode(tps->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(tps->port, TPS_STATUS_PORTROLE(status));
	typec_set_vconn_role(tps->port, TPS_STATUS_VCONN(status));
	tps6598x_set_data_role(tps, TPS_STATUS_DATAROLE(status), false);
	typec_set_mode(tps->port, 0);
	power_supply_changed(tps->psy);
}

static int tps6598x_exec_cmd(struct tps6598x *tps, const char *cmd,
			     size_t in_len, u8 *in_data,
			     size_t out_len, u8 *out_data)
{
	unsigned long timeout;
	u32 val;
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
	if (ret)
		return ret;
	if (val && !INVALID_CMD(val))
		return -EBUSY;

	if (in_len) {
		ret = tps6598x_block_write(tps, TPS_REG_DATA1,
					   in_data, in_len);
		if (ret)
			return ret;
	}

	ret = tps6598x_write_4cc(tps, TPS_REG_CMD1, cmd);
	if (ret < 0)
		return ret;

	/* XXX: Using 1s for now, but it may not be enough for every command. */
	timeout = jiffies + msecs_to_jiffies(1000);

	do {
		ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
		if (ret)
			return ret;
		if (INVALID_CMD(val))
			return -EINVAL;

		if (time_is_before_jiffies(timeout))
			return -ETIMEDOUT;
	} while (val);

	if (out_len) {
		ret = tps6598x_block_read(tps, TPS_REG_DATA1,
					  out_data, out_len);
		if (ret)
			return ret;
		val = out_data[0];
	} else {
		ret = tps6598x_block_read(tps, TPS_REG_DATA1, &val, sizeof(u8));
		if (ret)
			return ret;
	}

	switch (val) {
	case TPS_TASK_TIMEOUT:
		return -ETIMEDOUT;
	case TPS_TASK_REJECTED:
		return -EPERM;
	default:
		break;
	}

	return 0;
}

static int tps6598x_dr_set(struct typec_port *port, enum typec_data_role role)
{
	const char *cmd = (role == TYPEC_DEVICE) ? "SWUF" : "SWDF";
	struct tps6598x *tps = typec_get_drvdata(port);
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
	if (ret)
		goto out_unlock;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret)
		goto out_unlock;

	if (role != TPS_STATUS_DATAROLE(status)) {
		ret = -EPROTO;
		goto out_unlock;
	}

	tps6598x_set_data_role(tps, role, true);

out_unlock:
	mutex_unlock(&tps->lock);

	return ret;
}

static int tps6598x_pr_set(struct typec_port *port, enum typec_role role)
{
	const char *cmd = (role == TYPEC_SINK) ? "SWSk" : "SWSr";
	struct tps6598x *tps = typec_get_drvdata(port);
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
	if (ret)
		goto out_unlock;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret)
		goto out_unlock;

	if (role != TPS_STATUS_PORTROLE(status)) {
		ret = -EPROTO;
		goto out_unlock;
	}

	typec_set_pwr_role(tps->port, role);

out_unlock:
	mutex_unlock(&tps->lock);

	return ret;
}

static const struct typec_operations tps6598x_ops = {
	.dr_set = tps6598x_dr_set,
	.pr_set = tps6598x_pr_set,
};

static irqreturn_t tps6598x_interrupt(int irq, void *data)
{
	struct tps6598x *tps = data;
	u64 event1;
	u64 event2;
	u32 status, data_status;
	u16 pwr_status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_read64(tps, TPS_REG_INT_EVENT1, &event1);
	ret |= tps6598x_read64(tps, TPS_REG_INT_EVENT2, &event2);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read events\n", __func__);
		goto err_unlock;
	}
	trace_tps6598x_irq(event1, event2);
	if (tps->just_init)
		goto err_clear_ints;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read status\n", __func__);
		goto err_clear_ints;
	}
	trace_tps6598x_status(status);

	if ((event1 | event2) & TPS_REG_INT_POWER_STATUS_UPDATE) {
		ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
		if (ret < 0) {
			dev_err(tps->dev, "failed to read power status: %d\n", ret);
			goto err_clear_ints;
		}
		trace_tps6598x_power_status(pwr_status);
	}

	if ((event1 | event2) & TPS_REG_INT_DATA_STATUS_UPDATE) {
		ret = tps6598x_read32(tps, TPS_REG_DATA_STATUS, &data_status);
		if (ret < 0) {
			dev_err(tps->dev, "failed to read data status: %d\n", ret);
			goto err_clear_ints;
		}
		trace_tps6598x_data_status(data_status);
	}

	/* Handle plug insert or removal */
	if ((event1 | event2) & TPS_REG_INT_PLUG_EVENT) {
		if (status & TPS_STATUS_PLUG_PRESENT) {
			ret = tps6598x_connect(tps, status);
			if (ret)
				dev_err(tps->dev,
					"failed to register partner\n");
		} else {
			tps6598x_disconnect(tps, status);
		}
	}

	if (tps->cd321x_support &&
	    ((event1 | event2) & (TPS_CD321X_INT_STATUS_CHG | TPS_CD321X_INT_DATA_STATUS_CHG))) {
		mod_delayed_work(system_wq, &tps->cd321x_status_work,
				 msecs_to_jiffies(CD321X_SETTLE_MSEC));
	}

err_clear_ints:
	tps6598x_write64(tps, TPS_REG_INT_CLEAR1, event1);
	tps6598x_write64(tps, TPS_REG_INT_CLEAR2, event2);

err_unlock:
	mutex_unlock(&tps->lock);

	return (event1 || event2) ? IRQ_HANDLED : IRQ_NONE;
}

static void tps6598x_cd321x_status_work(struct work_struct *work)
{
	struct tps6598x *tps = container_of(to_delayed_work(work),
				struct tps6598x, cd321x_status_work);
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read status\n", __func__);
		goto err_unlock;
	}

	if (status & TPS_STATUS_PLUG_PRESENT) {
		ret = tps6598x_connect(tps, status);
		if (ret)
			dev_err(tps->dev,
				"failed to register partner: %d\n", ret);
	} else {
		tps6598x_disconnect(tps, status);
	}

err_unlock:
	mutex_unlock(&tps->lock);
}

static int tps6598x_check_mode(struct tps6598x *tps)
{
	char mode[5] = { };
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_MODE, (void *)mode);
	if (ret)
		return ret;

	switch (match_string(modes, ARRAY_SIZE(modes), mode)) {
	case TPS_MODE_APP:
		return 0;
	case TPS_MODE_BOOT:
		dev_warn(tps->dev, "dead-battery condition\n");
		return 0;
	case TPS_MODE_BIST:
	case TPS_MODE_DISC:
	default:
		dev_err(tps->dev, "controller in unsupported mode \"%s\"\n",
			mode);
		break;
	}

	return -ENODEV;
}

static const struct regmap_config tps6598x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
};

static int tps6598x_psy_get_online(struct tps6598x *tps,
				   union power_supply_propval *val)
{
	int ret;
	u16 pwr_status;

	ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
	if (ret < 0)
		return ret;

	if (TPS_POWER_STATUS_CONNECTION(pwr_status) &&
	    TPS_POWER_STATUS_SOURCESINK(pwr_status)) {
		val->intval = 1;
	} else {
		val->intval = 0;
	}
	return 0;
}

static int tps6598x_psy_get_prop(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct tps6598x *tps = power_supply_get_drvdata(psy);
	u16 pwr_status;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
		if (ret < 0)
			return ret;
		if (TPS_POWER_STATUS_PWROPMODE(pwr_status) == TYPEC_PWR_MODE_PD)
			val->intval = POWER_SUPPLY_USB_TYPE_PD;
		else
			val->intval = POWER_SUPPLY_USB_TYPE_C;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = tps6598x_psy_get_online(tps, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int devm_tps6598_psy_register(struct tps6598x *tps)
{
	struct power_supply_config psy_cfg = {};
	const char *port_dev_name = dev_name(tps->dev);
	char *psy_name;

	psy_cfg.drv_data = tps;
	psy_cfg.fwnode = dev_fwnode(tps->dev);

	psy_name = devm_kasprintf(tps->dev, GFP_KERNEL, "%s%s", tps6598x_psy_name_prefix,
				  port_dev_name);
	if (!psy_name)
		return -ENOMEM;

	tps->psy_desc.name = psy_name;
	tps->psy_desc.type = POWER_SUPPLY_TYPE_USB;
	tps->psy_desc.usb_types = tps6598x_psy_usb_types;
	tps->psy_desc.num_usb_types = ARRAY_SIZE(tps6598x_psy_usb_types);
	tps->psy_desc.properties = tps6598x_psy_props;
	tps->psy_desc.num_properties = ARRAY_SIZE(tps6598x_psy_props);
	tps->psy_desc.get_property = tps6598x_psy_get_prop;

	tps->usb_type = POWER_SUPPLY_USB_TYPE_C;

	tps->psy = devm_power_supply_register(tps->dev, &tps->psy_desc,
					       &psy_cfg);
	return PTR_ERR_OR_ZERO(tps->psy);
}

static int tps6598x_probe(struct i2c_client *client)
{
	struct typec_capability typec_cap = { };
	struct tps6598x *tps;
	struct fwnode_handle *fwnode;
	u32 status;
	u32 conf;
	u32 vid;
	u32 pid;
	u8 pstate;
	int ret;
	unsigned long flags;

	tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	mutex_init(&tps->lock);
	tps->dev = &client->dev;

	if (device_property_read_bool(&client->dev, "just-init"))
		tps->just_init = true;

	tps->regmap = devm_regmap_init_i2c(client, &tps6598x_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);

	/*
	 * Checking can the adapter handle SMBus protocol. If it cannot, the
	 * driver needs to take care of block reads separately.
	 *
	 * FIXME: Testing with I2C_FUNC_I2C. regmap-i2c uses I2C protocol
	 * unconditionally if the adapter has I2C_FUNC_I2C set.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		tps->i2c_protocol = true;

	/* Some drivers don't like 65-byte writes, either. */
	if (device_property_read_bool(&client->dev, "no-long-writes"))
		tps->i2c_no_long = true;

	if (tps->just_init) {
		printk("Just performing basic initialization!\n");
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tps6598x_interrupt,
					IRQF_SHARED | IRQF_ONESHOT,
					dev_name(&client->dev), tps);

		return ret = 0;
	}
	ret = tps6598x_write64(tps, TPS_REG_INT_MASK1, 0);
	if (ret)
		dev_err(&client->dev, "failed to set default interrupt mask %d\n", 1);

	ret = tps6598x_write64(tps, TPS_REG_INT_MASK2, 0);
	if (ret)
		dev_err(&client->dev, "failed to set default interrupt mask %d\n", 2);

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tps6598x_interrupt,
					IRQF_SHARED | IRQF_ONESHOT,
					dev_name(&client->dev), tps);

	ret = tps6598x_read32(tps, TPS_REG_VID, &vid);
	if (ret < 0 || !vid)
		return -ENODEV;

	ret = tps6598x_read32(tps, TPS_REG_PID, &pid);
	if (ret < 0)
		return -ENODEV;
	tps->product_id = pid;

	/* Make sure the controller has application firmware running */
	ret = tps6598x_check_mode(tps);
	if (ret)
		return ret;

	if(pid == PID_CD3217 || pid == PID_CD3218)
		tps->cd321x_support = true;

	ret = tps6598x_read8(tps, TPS_REG_POWER_STATE, &pstate);
	if (ret < 0)
		return ret;

	ret = tps6598x_write64(tps, TPS_REG_INT_MASK1, 0);
	if (ret)
		dev_err(&client->dev, "failed to set default interrupt mask %d\n", 1);

	ret = tps6598x_write64(tps, TPS_REG_INT_MASK2, 0);
	if (ret)
		dev_err(&client->dev, "failed to set default interrupt mask %d\n", 2);

	INIT_DELAYED_WORK(&tps->cd321x_status_work, tps6598x_cd321x_status_work);

	local_irq_save(flags);
	if (pstate == TPS_POWER_STATE_BOOT) {
		/* on Apple M1, this is how the CD3217/8 comes up; transition to S0 */
		u8 ssps_data[2] = { TPS_POWER_STATE_S0, 0 };

		ret = tps6598x_exec_cmd(tps, "SSPS", sizeof(ssps_data), ssps_data, 0, NULL);
		if (ret) {
			dev_err(&client->dev, "failed to set power state S0\n");
			return ret;
		}
		dev_err(&client->dev, "port is now in power state S0\n");
	}

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret < 0)
		return ret;
	trace_tps6598x_status(status);

	ret = tps6598x_read32(tps, TPS_REG_SYSTEM_CONF, &conf);
	if (ret < 0)
		return ret;

	fwnode = device_get_named_child_node(&client->dev, "connector");
	if (!fwnode)
		return -ENODEV;

	/*
	 * This fwnode has a "compatible" property, but is never populated as a
	 * struct device. Instead we simply parse it to read the properties.
	 * This breaks fw_devlink=on. To maintain backward compatibility
	 * with existing DT files, we work around this by deleting any
	 * fwnode_links to/from this fwnode.
	 */
	fw_devlink_purge_absent_suppliers(fwnode);

	tps->role_sw = fwnode_usb_role_switch_get(fwnode);
	if (IS_ERR(tps->role_sw)) {
		ret = PTR_ERR(tps->role_sw);
		goto err_fwnode_put;
	}

	typec_cap.revision = USB_TYPEC_REV_1_2;
	typec_cap.pd_revision = 0x200;
	typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	typec_cap.driver_data = tps;
	typec_cap.ops = &tps6598x_ops;
	typec_cap.fwnode = fwnode;

	switch (TPS_SYSCONF_PORTINFO(conf)) {
	case TPS_PORTINFO_SINK_ACCESSORY:
	case TPS_PORTINFO_SINK:
		typec_cap.type = TYPEC_PORT_SNK;
		typec_cap.data = TYPEC_PORT_UFP;
		break;
	case TPS_PORTINFO_DRP_UFP_DRD:
	case TPS_PORTINFO_DRP_DFP_DRD:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DRD;
		break;
	case TPS_PORTINFO_DRP_UFP:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_UFP;
		break;
	case TPS_PORTINFO_DRP_DFP:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DFP;
		break;
	case TPS_PORTINFO_SOURCE:
		typec_cap.type = TYPEC_PORT_SRC;
		typec_cap.data = TYPEC_PORT_DFP;
		break;
	default:
		ret = -ENODEV;
		goto err_role_put;
	}

	ret = devm_tps6598_psy_register(tps);
	if (ret)
		return ret;

	tps->port = typec_register_port(&client->dev, &typec_cap);
	if (IS_ERR(tps->port)) {
		ret = PTR_ERR(tps->port);
		goto err_role_put;
	}
	fwnode_handle_put(fwnode);

	if (status & TPS_STATUS_PLUG_PRESENT) {
		ret = tps6598x_connect(tps, status);
		if (ret)
			dev_err(&client->dev, "failed to register partner\n");
	}
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tps6598x_interrupt,
					IRQF_SHARED | IRQF_ONESHOT,
					dev_name(&client->dev), tps);
	if (ret) {
		tps6598x_disconnect(tps, 0);
		typec_unregister_port(tps->port);
		goto err_role_put;
	}

	i2c_set_clientdata(client, tps);

	ret = tps6598x_write64(tps, TPS_REG_INT_MASK1, TPS_CD321X_INT_MASK);
	if (ret)
		dev_err(&client->dev, "failed to set default interrupt mask %d\n", 1);

	local_irq_restore(flags);
	return 0;

err_role_put:
	usb_role_switch_put(tps->role_sw);
err_fwnode_put:
	fwnode_handle_put(fwnode);

	return ret;
}

static int tps6598x_remove(struct i2c_client *client)
{
	struct tps6598x *tps = i2c_get_clientdata(client);

	tps6598x_disconnect(tps, 0);
	typec_unregister_port(tps->port);
	usb_role_switch_put(tps->role_sw);

	return 0;
}

static const struct of_device_id tps6598x_of_match[] = {
	{ .compatible = "ti,tps6598x", },
	{}
};
MODULE_DEVICE_TABLE(of, tps6598x_of_match);

static const struct i2c_device_id tps6598x_id[] = {
	{ "tps6598x" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6598x_id);

static struct i2c_driver tps6598x_i2c_driver = {
	.driver = {
		.name = "tps6598x",
		.of_match_table = tps6598x_of_match,
	},
	.probe_new = tps6598x_probe,
	.remove = tps6598x_remove,
	.id_table = tps6598x_id,
};
module_i2c_driver(tps6598x_i2c_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI TPS6598x USB Power Delivery Controller Driver");
