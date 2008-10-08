/*
 * Apple USB BCM5974 (Macbook Air and Penryn Macbook Pro) multitouch driver
 *
 * Copyright (C) 2008	   Henrik Rydberg (rydberg@euromail.se)
 *
 * The USB initialization and package decoding was made by
 * Scott Shawcroft as part of the touchd user-space driver project:
 * Copyright (C) 2008	   Scott Shawcroft (scott.shawcroft@gmail.com)
 *
 * The BCM5974 driver is based on the appletouch driver:
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2005      Johannes Berg (johannes@sipsolutions.net)
 * Copyright (C) 2005	   Stelian Pop (stelian@popies.net)
 * Copyright (C) 2005	   Frank Arnold (frank@scirocco-5v-turbo.de)
 * Copyright (C) 2005	   Peter Osterlund (petero2@telia.com)
 * Copyright (C) 2005	   Michael Hanselmann (linux-kernel@hansmi.ch)
 * Copyright (C) 2006	   Nicolas Boichat (nicolas@boichat.ch)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb/input.h>
#include <linux/hid.h>

#define APPLE_VENDOR_ID		0x05AC

/* MacbookAir, aka wellspring */
#define ATP_WELLSPRING_ANSI	0x0223
#define ATP_WELLSPRING_ISO	0x0224
#define ATP_WELLSPRING_JIS	0x0225
/* MacbookProPenryn, aka wellspring2 */
#define ATP_WELLSPRING2_ANSI	0x0230
#define ATP_WELLSPRING2_ISO	0x0231
#define ATP_WELLSPRING2_JIS	0x0232

#define ATP_DEVICE(prod) {					\
	.match_flags = (USB_DEVICE_ID_MATCH_DEVICE |		\
			USB_DEVICE_ID_MATCH_INT_CLASS |		\
			USB_DEVICE_ID_MATCH_INT_PROTOCOL),	\
	.idVendor = APPLE_VENDOR_ID,				\
	.idProduct = (prod),					\
	.bInterfaceClass = USB_INTERFACE_CLASS_HID,		\
	.bInterfaceProtocol = USB_INTERFACE_PROTOCOL_MOUSE	\
}

/* table of devices that work with this driver */
static const struct usb_device_id atp_table [] = {
	/* MacbookAir1.1 */
	ATP_DEVICE(ATP_WELLSPRING_ANSI),
	ATP_DEVICE(ATP_WELLSPRING_ISO),
	ATP_DEVICE(ATP_WELLSPRING_JIS),
	/* MacbookProPenryn */
	ATP_DEVICE(ATP_WELLSPRING2_ANSI),
	ATP_DEVICE(ATP_WELLSPRING2_ISO),
	ATP_DEVICE(ATP_WELLSPRING2_JIS),
	/* Terminating entry */
	{}
};
MODULE_DEVICE_TABLE(usb, atp_table);

MODULE_AUTHOR("Henrik Rydberg");
MODULE_DESCRIPTION("Apple USB BCM5974 multitouch driver");
MODULE_LICENSE("GPL");

#define dprintk(level, format, a...)\
	{ if (debug >= level) printk(KERN_DEBUG format, ##a); }

static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

/* button data structure */
struct bt_data {
	u8 unknown1;		/* constant */
	u8 button;		/* left button */
	u8 rel_x;		/* relative x coordinate */
	u8 rel_y;		/* relative y coordinate */
};

/* trackpad header structure */
struct tp_header {
	u8 unknown1[16];	/* constants, timers, etc */
	u8 fingers;		/* number of fingers on trackpad */
	u8 unknown2[9];		/* constants, timers, etc */
};

/* trackpad finger structure */
struct tp_finger {
	__le16 origin;		/* left/right origin? */
	__le16 abs_x;		/* absolute x coodinate */
	__le16 abs_y;		/* absolute y coodinate */
	__le16 rel_x;		/* relative x coodinate */
	__le16 rel_y;		/* relative y coodinate */
	__le16 size_major;	/* finger size, major axis? */
	__le16 size_minor;	/* finger size, minor axis? */
	__le16 orientation;	/* 16384 when point, else 15 bit angle */
	__le16 force_major;	/* trackpad force, major axis? */
	__le16 force_minor;	/* trackpad force, minor axis? */
	__le16 unused[3];	/* zeros */
	__le16 multi;		/* one finger: varies, more fingers: constant */
};

/* trackpad data structure, empirically at least ten fingers */
struct tp_data {
	struct tp_header header;
	struct tp_finger finger[16];
};

/* device-specific parameters */
struct atp_params {
	int dim;		/* logical dimension */
	int fuzz;		/* logical noise value */
	int devmin;		/* device minimum reading */
	int devmax;		/* device maximum reading */
};

/* device-specific configuration */
struct atp_config {
	int ansi, iso, jis;	/* the product id of this device */
	int bt_ep;		/* the endpoint of the button interface */
	int bt_datalen;		/* data length of the button interface */
	int tp_ep;		/* the endpoint of the trackpad interface */
	int tp_datalen;		/* data length of the trackpad interface */
	struct atp_params p;	/* finger pressure limits */
	struct atp_params w;	/* finger width limits */
	struct atp_params x;	/* horizontal limits */
	struct atp_params y;	/* vertical limits */
};

/* logical device structure */
struct atp {
	char phys[64];
	struct usb_device *udev;	/* usb device */
	struct input_dev *input;	/* input dev */
	struct atp_config cfg;		/* device configuration */
	int open;			/* >0: open, else closed */
	int suspended;			/* >0: suspended, else open */
	struct urb *bt_urb;		/* button usb request block */
	struct bt_data *bt_data;	/* button transferred data */
	struct urb *tp_urb;		/* trackpad usb request block */
	struct tp_data *tp_data;	/* trackpad transferred data */
	unsigned tp_valid;		/* trackpad sensors valid */
};

/* logical dimensions */
#define DIM_PRESSURE	256		/* maximum finger pressure */
#define DIM_WIDTH	16		/* maximum finger width */
#define DIM_X		1280		/* maximum trackpad x value */
#define DIM_Y		800		/* maximum trackpad y value */

/* logical signal quality */
#define SN_PRESSURE	45		/* pressure signal-to-noise ratio */
#define SN_WIDTH	100		/* width signal-to-noise ratio */
#define SN_COORD	250		/* coordinate signal-to-noise ratio */

/* device constants */
static const struct atp_config atp_config_table[] = {
	{
		ATP_WELLSPRING_ANSI,
		ATP_WELLSPRING_ISO,
		ATP_WELLSPRING_JIS,
		0x84, sizeof(struct bt_data),
		0x81, sizeof(struct tp_data),
		{ DIM_PRESSURE, DIM_PRESSURE / SN_PRESSURE, 0, 256 },
		{ DIM_WIDTH, DIM_WIDTH / SN_WIDTH, 0, 2048 },
		{ DIM_X, DIM_X / SN_COORD, -4824, 5342 },
		{ DIM_Y, DIM_Y / SN_COORD, -172, 5820 }
	},
	{
		ATP_WELLSPRING2_ANSI,
		ATP_WELLSPRING2_ISO,
		ATP_WELLSPRING2_JIS,
		0x84, sizeof(struct bt_data),
		0x81, sizeof(struct tp_data),
		{ DIM_PRESSURE, DIM_PRESSURE / SN_PRESSURE, 0, 256 },
		{ DIM_WIDTH, DIM_WIDTH / SN_WIDTH, 0, 2048 },
		{ DIM_X, DIM_X / SN_COORD, -4824, 4824 },
		{ DIM_Y, DIM_Y / SN_COORD, -172, 4290 }
	},
	{}
};

/* return the device-specific configuration by device */
static const struct atp_config *atp_product_config(struct usb_device *udev)
{
	u16 id = le16_to_cpu(udev->descriptor.idProduct);
	const struct atp_config *config;
	for (config = atp_config_table; config->ansi; ++config)
		if (config->ansi == id || config->iso == id ||
				config->jis == id)
			return config;
	return atp_config_table;
}

/* convert 16-bit little endian to signed integer */
static inline int raw2int(__le16 x)
{
	return (signed short)le16_to_cpu(x);
}

/* scale device data to logical dimensions (asserts devmin < devmax) */
static inline int int2scale(const struct atp_params *p, int x)
{
	return x * p->dim / (p->devmax - p->devmin);
}

/* all logical value ranges are [0,dim). */
static inline int int2bound(const struct atp_params *p, int x)
{
	int s = int2scale(p, x);
	return s < 0 ? 0 : s >= p->dim ? p->dim - 1 : s;
}

/* report button data as logical button state */
static int report_bt_state(struct atp *dev, int size)
{
	if (size != sizeof(struct bt_data))
		return -EIO;

	input_report_key(dev->input, BTN_LEFT, dev->bt_data->button);

	return 0;
}

/* report trackpad data as logical trackpad state */
static int report_tp_state(struct atp *dev, int size)
{
	const struct atp_config *c = &dev->cfg;
	const struct tp_finger *f = dev->tp_data->finger;
	const int fingers = (size - 26) / 28;
	int p, w, x, y, n;

	if (size < 26 || (size - 26) % 28 != 0)
		return -EIO;

	if (!fingers) {
		input_report_abs(dev->input, ABS_PRESSURE, 0);
		input_report_key(dev->input, BTN_TOOL_FINGER, false);
		input_report_key(dev->input, BTN_TOOL_DOUBLETAP, false);
		input_report_key(dev->input, BTN_TOOL_TRIPLETAP, false);
		return 0;
	}

	p = raw2int(f->force_major);
	w = raw2int(f->size_major);
	x = raw2int(f->abs_x);
	y = raw2int(f->abs_y);
	n = p > 0 ? fingers : 0;

	dprintk(9, "bcm5974: p: %+05d w: %+05d x: %+05d y: %+05d n: %d\n",
		p, w, x, y, n);

	input_report_abs(dev->input, ABS_PRESSURE, int2bound(&c->p, p));
	input_report_abs(dev->input, ABS_TOOL_WIDTH, int2bound(&c->w, w));
	input_report_abs(dev->input, ABS_X, int2bound(&c->x, x - c->x.devmin));
	input_report_abs(dev->input, ABS_Y, int2bound(&c->y, c->y.devmax - y));
	input_report_key(dev->input, BTN_TOOL_FINGER, n == 1);
	input_report_key(dev->input, BTN_TOOL_DOUBLETAP, n == 2);
	input_report_key(dev->input, BTN_TOOL_TRIPLETAP, n > 2);
	return 0;
}

/* Wellspring initialization constants */
#define ATP_WELLSPRING_MODE_READ_REQUEST_ID	1
#define ATP_WELLSPRING_MODE_WRITE_REQUEST_ID	9
#define ATP_WELLSPRING_MODE_REQUEST_VALUE	0x300
#define ATP_WELLSPRING_MODE_REQUEST_INDEX	0
#define ATP_WELLSPRING_MODE_VENDOR_VALUE_1	0x01
#define ATP_WELLSPRING_MODE_VENDOR_VALUE_2	0x05

static int atp_wellspring_init(struct atp *dev)
{
	char *data = kmalloc(8, GFP_KERNEL);
	int error = 0, size;

	if (!data) {
		err("bcm5974: out of memory");
		error = -ENOMEM;
		goto error;
	}

	/* reset button endpoint */
	if (usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
			USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT,
			0, dev->cfg.bt_ep, NULL, 0, 5000)) {
		err("bcm5974: could not reset button endpoint");
		error = -EIO;
		goto error;
	}

	/* reset trackpad endpoint */
	if (usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
			USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT,
			0, dev->cfg.tp_ep, NULL, 0, 5000)) {
		err("bcm5974: could not reset trackpad endpoint");
		error = -EIO;
		goto error;
	}

	/* read configuration */
	size = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
		ATP_WELLSPRING_MODE_READ_REQUEST_ID,
		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		ATP_WELLSPRING_MODE_REQUEST_VALUE,
		ATP_WELLSPRING_MODE_REQUEST_INDEX, data, 8, 5000);

	if (size != 8) {
		err("bcm5974: could not read from device");
		error = -EIO;
		goto error;
	}

	/* apply the mode switch */
	data[0] = ATP_WELLSPRING_MODE_VENDOR_VALUE_1;
	data[1] = ATP_WELLSPRING_MODE_VENDOR_VALUE_2;

	/* write configuration */
	size = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
		ATP_WELLSPRING_MODE_WRITE_REQUEST_ID,
		USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		ATP_WELLSPRING_MODE_REQUEST_VALUE,
		ATP_WELLSPRING_MODE_REQUEST_INDEX, data, 8, 5000);

	if (size != 8) {
		err("bcm5974: could not write to device");
		error = -EIO;
		goto error;
	}

	dev->tp_valid = 0;

	printk(KERN_INFO "bcm5974: Wellspring mode initialized.\n");

	kfree(data);
	return 0;

error:
	kfree(data);
	return error;
}

static void irq_button(struct urb *urb)
{
	struct atp *dev = urb->context;
	int error;

	switch (urb->status) {
	case 0:
		break;
	case -EOVERFLOW:
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dbg("bcm5974: button urb shutting down: %d", urb->status);
		return;
	default:
		dbg("bcm5974: button urb status: %d", urb->status);
		goto exit;
	}

	if (report_bt_state(dev, dev->bt_urb->actual_length)) {
		dprintk(1, "bcm5974: bad button package, length: %d\n",
			dev->bt_urb->actual_length);
		goto exit;
	}

	input_sync(dev->input);

exit:
	error = usb_submit_urb(dev->bt_urb, GFP_ATOMIC);
	if (error)
		err("bcm5974: button urb failed: %d", error);
}

static void irq_trackpad(struct urb *urb)
{
	struct atp *dev = urb->context;
	int error;

	switch (urb->status) {
	case 0:
		break;
	case -EOVERFLOW:
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dbg("bcm5974: trackpad urb shutting down: %d", urb->status);
		return;
	default:
		dbg("bcm5974: trackpad urb status: %d", urb->status);
		goto exit;
	}

	/* first sample data ignored */
	if (!dev->tp_valid) {
		dev->tp_valid = 1;
		goto exit;
	}

	if (report_tp_state(dev, dev->tp_urb->actual_length)) {
		dprintk(1, "bcm5974: bad trackpad package, length: %d\n",
			dev->tp_urb->actual_length);
		goto exit;
	}

	input_sync(dev->input);

exit:
	error = usb_submit_urb(dev->tp_urb, GFP_ATOMIC);
	if (error)
		err("bcm5974: trackpad urb failed: %d", error);
}

static int atp_open(struct input_dev *input)
{
	struct atp *dev = input_get_drvdata(input);

	if (!dev->open) {
		if (usb_submit_urb(dev->bt_urb, GFP_KERNEL))
			goto error;
		if (usb_submit_urb(dev->tp_urb, GFP_KERNEL))
			goto err_free_bt_urb;
	}

	dev->open = 1;
	dev->suspended = 0;
	return 0;

err_free_bt_urb:
	usb_free_urb(dev->bt_urb);
error:
	return -EIO;
}

static void atp_close(struct input_dev *input)
{
	struct atp *dev = input_get_drvdata(input);

	usb_kill_urb(dev->tp_urb);
	usb_kill_urb(dev->bt_urb);

	dev->open = 0;
	dev->suspended = 0;
}

static int atp_probe(struct usb_interface *iface,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(iface);
	const struct atp_config *cfg;
	struct atp *dev;
	struct input_dev *input_dev;
	int error = 0;

	/* find the product index */
	cfg = atp_product_config(udev);

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(struct atp), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!dev || !input_dev) {
		err("bcm5974: out of memory");
		error = -ENOMEM;
		goto err_free_devs;
	}

	dev->udev = udev;
	dev->input = input_dev;
	dev->cfg = *cfg;

	/* switch to raw sensor mode */
	if (atp_wellspring_init(dev)) {
		error = -EIO;
		goto err_free_devs;
	}

	dev->bt_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bt_urb) {
		error = -ENOMEM;
		goto err_free_devs;
	}

	dev->tp_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->tp_urb) {
		error = -ENOMEM;
		goto err_free_bt_urb;
	}

	dev->bt_data = usb_buffer_alloc(dev->udev,
		dev->cfg.bt_datalen, GFP_KERNEL,
		&dev->bt_urb->transfer_dma);
	if (!dev->bt_data) {
		error = -ENOMEM;
		goto err_free_urb;
	}

	dev->tp_data = usb_buffer_alloc(dev->udev,
		dev->cfg.tp_datalen, GFP_KERNEL,
		&dev->tp_urb->transfer_dma);
	if (!dev->tp_data) {
		error = -ENOMEM;
		goto err_free_bt_buffer;
	}

	usb_fill_int_urb(dev->bt_urb, udev,
		usb_rcvintpipe(udev, cfg->bt_ep),
		dev->bt_data, dev->cfg.bt_datalen,
		irq_button, dev, 1);

	usb_fill_int_urb(dev->tp_urb, udev,
		usb_rcvintpipe(udev, cfg->tp_ep),
		dev->tp_data, dev->cfg.tp_datalen,
		irq_trackpad, dev, 1);

	usb_make_path(udev, dev->phys, sizeof(dev->phys));
	strlcat(dev->phys, "/input0", sizeof(dev->phys));

	input_dev->name = "bcm5974";
	input_dev->phys = dev->phys;
	usb_to_input_id(dev->udev, &input_dev->id);
	input_dev->dev.parent = &iface->dev;

	input_set_drvdata(input_dev, dev);

	input_dev->open = atp_open;
	input_dev->close = atp_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_PRESSURE,
		0, cfg->p.dim, cfg->p.fuzz, 0);
	input_set_abs_params(input_dev, ABS_TOOL_WIDTH,
		0, cfg->w.dim, cfg->w.fuzz, 0);
	input_set_abs_params(input_dev, ABS_X,
		0, cfg->x.dim, cfg->x.fuzz, 0);
	input_set_abs_params(input_dev, ABS_Y,
		0, cfg->y.dim, cfg->y.fuzz, 0);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
	set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
	set_bit(BTN_LEFT, input_dev->keybit);

	error = input_register_device(dev->input);
	if (error)
		goto err_free_buffer;

	/* save our data pointer in this interface device */
	usb_set_intfdata(iface, dev);

	return 0;

err_free_buffer:
	usb_buffer_free(dev->udev, dev->cfg.tp_datalen,
		dev->tp_data, dev->tp_urb->transfer_dma);
err_free_bt_buffer:
	usb_buffer_free(dev->udev, dev->cfg.bt_datalen,
		dev->bt_data, dev->bt_urb->transfer_dma);
err_free_urb:
	usb_free_urb(dev->tp_urb);
err_free_bt_urb:
	usb_free_urb(dev->bt_urb);
err_free_devs:
	usb_set_intfdata(iface, NULL);
	kfree(dev);
	input_free_device(input_dev);
	return error;
}

static void atp_disconnect(struct usb_interface *iface)
{
	struct atp *dev = usb_get_intfdata(iface);

	usb_set_intfdata(iface, NULL);

	if (dev) {
		input_unregister_device(dev->input);
		usb_buffer_free(dev->udev, dev->cfg.tp_datalen,
			dev->tp_data, dev->tp_urb->transfer_dma);
		usb_buffer_free(dev->udev, dev->cfg.bt_datalen,
			dev->bt_data, dev->bt_urb->transfer_dma);
		usb_free_urb(dev->tp_urb);
		usb_free_urb(dev->bt_urb);
		kfree(dev);
	}

	printk(KERN_INFO "bcm5974: disconnected\n");
}

static int atp_suspend(struct usb_interface *iface, pm_message_t message)
{
	struct atp *dev = usb_get_intfdata(iface);

	if (dev) {
		if (!dev->suspended)
			atp_close(dev->input);
		dev->suspended++;
	}

	return 0;
}

static int atp_resume(struct usb_interface *iface)
{
	struct atp *dev = usb_get_intfdata(iface);
	int error = 0;

	if (dev && dev->suspended) {
		if (!--dev->suspended)
			error = atp_open(dev->input);
	}

	return error;
}

static int atp_reset_resume(struct usb_interface *iface)
{
	struct atp *dev = usb_get_intfdata(iface);

	if (dev && atp_wellspring_init(dev))
		printk(KERN_INFO "bcm5974: warning: reset failed\n");

	return atp_resume(iface);
}

static int atp_pre_reset(struct usb_interface *iface)
{
	return atp_suspend(iface, PMSG_ON);
}

static int atp_post_reset(struct usb_interface *iface)
{
	return atp_reset_resume(iface);
}

static struct usb_driver atp_driver = {
	.name = "bcm5974",
	.probe = atp_probe,
	.disconnect = atp_disconnect,
	.suspend = atp_suspend,
	.resume = atp_resume,
	.reset_resume = atp_reset_resume,
	.pre_reset = atp_pre_reset,
	.post_reset = atp_post_reset,
	.id_table = atp_table,
};

static int __init atp_init(void)
{
	return usb_register(&atp_driver);
}

static void __exit atp_exit(void)
{
	usb_deregister(&atp_driver);
}

module_init(atp_init);
module_exit(atp_exit);

