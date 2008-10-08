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
#include <linux/mutex.h>

#define USB_VENDOR_ID_APPLE		0x05ac

/* MacbookAir, aka wellspring */
#define USB_DEVICE_ID_APPLE_WELLSPRING_ANSI	0x0223
#define USB_DEVICE_ID_APPLE_WELLSPRING_ISO	0x0224
#define USB_DEVICE_ID_APPLE_WELLSPRING_JIS	0x0225
/* MacbookProPenryn, aka wellspring2 */
#define USB_DEVICE_ID_APPLE_WELLSPRING2_ANSI	0x0230
#define USB_DEVICE_ID_APPLE_WELLSPRING2_ISO	0x0231
#define USB_DEVICE_ID_APPLE_WELLSPRING2_JIS	0x0232

#define BCM5974_DEVICE(prod) {					\
	.match_flags = (USB_DEVICE_ID_MATCH_DEVICE |		\
			USB_DEVICE_ID_MATCH_INT_CLASS |		\
			USB_DEVICE_ID_MATCH_INT_PROTOCOL),	\
	.idVendor = USB_VENDOR_ID_APPLE,			\
	.idProduct = (prod),					\
	.bInterfaceClass = USB_INTERFACE_CLASS_HID,		\
	.bInterfaceProtocol = USB_INTERFACE_PROTOCOL_MOUSE	\
}

/* table of devices that work with this driver */
static const struct usb_device_id bcm5974_table[] = {
	/* MacbookAir1.1 */
	BCM5974_DEVICE(USB_DEVICE_ID_APPLE_WELLSPRING_ANSI),
	BCM5974_DEVICE(USB_DEVICE_ID_APPLE_WELLSPRING_ISO),
	BCM5974_DEVICE(USB_DEVICE_ID_APPLE_WELLSPRING_JIS),
	/* MacbookProPenryn */
	BCM5974_DEVICE(USB_DEVICE_ID_APPLE_WELLSPRING2_ANSI),
	BCM5974_DEVICE(USB_DEVICE_ID_APPLE_WELLSPRING2_ISO),
	BCM5974_DEVICE(USB_DEVICE_ID_APPLE_WELLSPRING2_JIS),
	/* Terminating entry */
	{}
};
MODULE_DEVICE_TABLE(usb, bcm5974_table);

MODULE_AUTHOR("Henrik Rydberg");
MODULE_DESCRIPTION("Apple USB BCM5974 multitouch driver");
MODULE_LICENSE("GPL");

#define dprintk(level, format, a...)\
	{ if (debug >= level) printk(KERN_DEBUG format, ##a); }

#define MODE_MOUSE	1
#define MODE_TOUCHPAD	2

static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

static int driver_mode = MODE_MOUSE;
module_param(driver_mode, int, 0644);
MODULE_PARM_DESC(driver_mode, "Driver mode (1 - mouse; 2 - touchpad)");

static int mouse_motion_damping = 8;
module_param(mouse_motion_damping, int, 0644);
MODULE_PARM_DESC(mouse_motion_damping, "Mouse motion damping");

static int mouse_wheel_damping = 256;
module_param(mouse_wheel_damping, int, 0644);
MODULE_PARM_DESC(mouse_wheel_damping, "Vertical mouse wheel damping");

static int mouse_hwheel_damping = 256;
module_param(mouse_hwheel_damping, int, 0644);
MODULE_PARM_DESC(mouse_hwheel_damping, "Horizontal mouse wheel damping");

static int mouse_button_mode = 2;
module_param(mouse_button_mode, int, 0644);
MODULE_PARM_DESC(mouse_button_mode, "Mouse button mode (1 - unix; 2 - macos)");

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
struct bcm5974_param {
	int dim;		/* logical dimension */
	int fuzz;		/* logical noise value */
	int devmin;		/* device minimum reading */
	int devmax;		/* device maximum reading */
};

/* device-specific configuration */
struct bcm5974_config {
	int ansi, iso, jis;	/* the product id of this device */
	int bt_ep;		/* the endpoint of the button interface */
	int bt_datalen;		/* data length of the button interface */
	int tp_ep;		/* the endpoint of the trackpad interface */
	int tp_datalen;		/* data length of the trackpad interface */
	struct bcm5974_param p;	/* finger pressure limits */
	struct bcm5974_param w;	/* finger width limits */
	struct bcm5974_param x;	/* horizontal limits */
	struct bcm5974_param y;	/* vertical limits */
};

/* mouse driver state */
struct bcm5974_mouse_state {
	int fingers;			/* number of fingers on trackpad */
	int rel_x;			/* horizontal relative counter */
	int rel_y;			/* vertical relative counter */
	int wheel_x;			/* horizontal wheel counter */
	int wheel_y;			/* vertical wheel counter */
};

/* logical device structure */
struct bcm5974 {
	char phys[64];
	struct usb_device *udev;	/* usb device */
	struct usb_interface *intf;	/* our interface */
	struct input_dev *input;	/* input dev */
	struct bcm5974_config cfg;	/* device configuration */
	struct mutex pm_mutex;		/* serialize access to open/suspend */
	int opened;			/* 1: opened, 0: closed */
	struct urb *bt_urb;		/* button usb request block */
	struct bt_data *bt_data;	/* button transferred data */
	struct urb *tp_urb;		/* trackpad usb request block */
	struct tp_data *tp_data;	/* trackpad transferred data */
	struct bcm5974_mouse_state ms;	/* mouse state */
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
static const struct bcm5974_config bcm5974_config_table[] = {
	{
		USB_DEVICE_ID_APPLE_WELLSPRING_ANSI,
		USB_DEVICE_ID_APPLE_WELLSPRING_ISO,
		USB_DEVICE_ID_APPLE_WELLSPRING_JIS,
		0x84, sizeof(struct bt_data),
		0x81, sizeof(struct tp_data),
		{ DIM_PRESSURE, DIM_PRESSURE / SN_PRESSURE, 0, 256 },
		{ DIM_WIDTH, DIM_WIDTH / SN_WIDTH, 0, 2048 },
		{ DIM_X, DIM_X / SN_COORD, -4824, 5342 },
		{ DIM_Y, DIM_Y / SN_COORD, -172, 5820 }
	},
	{
		USB_DEVICE_ID_APPLE_WELLSPRING2_ANSI,
		USB_DEVICE_ID_APPLE_WELLSPRING2_ISO,
		USB_DEVICE_ID_APPLE_WELLSPRING2_JIS,
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
static const struct bcm5974_config *bcm5974_get_config(struct usb_device *udev)
{
	u16 id = le16_to_cpu(udev->descriptor.idProduct);
	const struct bcm5974_config *cfg;

	for (cfg = bcm5974_config_table; cfg->ansi; ++cfg)
		if (cfg->ansi == id || cfg->iso == id || cfg->jis == id)
			return cfg;

	return bcm5974_config_table;
}

/* convert 16-bit little endian to signed integer */
static inline int raw2int(__le16 x)
{
	return (signed short)le16_to_cpu(x);
}

/* scale device data to logical dimensions (asserts devmin < devmax) */
static inline int int2scale(const struct bcm5974_param *p, int x)
{
	return x * p->dim / (p->devmax - p->devmin);
}

/* all logical value ranges are [0,dim). */
static inline int int2bound(const struct bcm5974_param *p, int x)
{
	int s = int2scale(p, x);

//      return clamp_val(s, 0, p->dim - 1);
        return s < 0 ? 0 : s >= p->dim ? p->dim - 1 : s;
}

/* setup which logical events to report */
static void setup_events_to_report(struct input_dev *input_dev,
				   const struct bcm5974_config *cfg)
{
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_LEFT, input_dev->keybit);

	switch (driver_mode) {
	case MODE_MOUSE:
		__set_bit(EV_REL, input_dev->evbit);
		__set_bit(REL_X, input_dev->relbit);
		__set_bit(REL_Y, input_dev->relbit);
		__set_bit(REL_HWHEEL, input_dev->relbit);
		__set_bit(REL_WHEEL, input_dev->relbit);
		__set_bit(BTN_MIDDLE, input_dev->keybit);
		__set_bit(BTN_RIGHT, input_dev->keybit);
		break;
	case MODE_TOUCHPAD:
		__set_bit(EV_ABS, input_dev->evbit);
		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);

		input_set_abs_params(input_dev, ABS_PRESSURE,
					0, cfg->p.dim, cfg->p.fuzz, 0);
		input_set_abs_params(input_dev, ABS_TOOL_WIDTH,
					0, cfg->w.dim, cfg->w.fuzz, 0);
		input_set_abs_params(input_dev, ABS_X,
					0, cfg->x.dim, cfg->x.fuzz, 0);
		input_set_abs_params(input_dev, ABS_Y,
					0, cfg->y.dim, cfg->y.fuzz, 0);
		break;
	}
}

/* update logical mouse button state */
static void update_bt_mouse_state(struct input_dev *dev,
				const struct bcm5974_mouse_state *ms,
				const struct bt_data *bt)
{
	const int n = ms->fingers;

	bool left, middle, right;
	switch (mouse_button_mode) {
	case 1:
		left = n <= 1 && bt->button;
		middle = n == 2 && bt->button;
		right = n >= 3 && bt->button;
		break;
	case 2:
		left = n <= 1 && bt->button;
		middle = n >= 3 && bt->button;
		right = n == 2 && bt->button;
		break;
	default:
		left = bt->button;
		middle = false;
		right = false;
		break;
	};
	input_report_key(dev, BTN_LEFT, left);
	input_report_key(dev, BTN_MIDDLE, middle);
	input_report_key(dev, BTN_RIGHT, right);
}

/* update logical touchpad button state */
static void update_bt_touchpad_state(struct input_dev *dev,
				const struct bt_data *bt)
{
	input_report_key(dev, BTN_LEFT, bt->button);
}

/* report button data as logical mouse/touchpad button state */
static int report_bt_state(struct bcm5974 *dev, int size)
{
	if (size != sizeof(struct bt_data))
		return -EIO;

	switch (driver_mode) {
	case MODE_MOUSE:
		update_bt_mouse_state(dev->input, &dev->ms, dev->bt_data);
		break;
	case MODE_TOUCHPAD:
		update_bt_touchpad_state(dev->input, dev->bt_data);
		break;
	}

	input_sync(dev->input);

	return 0;
}

/* update logical mouse motion state */
static void update_tp_mouse_state(struct input_dev *dev,
				struct bcm5974_mouse_state *ms,
				const struct bcm5974_config *c,
				const struct tp_finger *f,
				int p, int n)
{
	int dx = 0, dy = 0, sx = 0, sy = 0, swx = 0, swy = 0;

	if (f) {
		dx = raw2int(f->rel_x);
		dy = raw2int(f->rel_y);

		dprintk(9,
			"bcm5974: p: %+05d dx: %+05d dy: %+05d n: %d\n",
			p, dx, dy, n);
	}

	if (n >= 3) {
		/* swipe */
		ms->rel_x = 0;
		ms->rel_y = 0;
		ms->wheel_x += int2scale(&c->x, dx);
		ms->wheel_y = 0;
		swx = ms->wheel_x / mouse_hwheel_damping;
		ms->wheel_x -= swx * mouse_hwheel_damping;
	} else if (n == 2) {
		/* scroll */
		ms->rel_x = 0;
		ms->rel_y = 0;
		ms->wheel_x = 0;
		ms->wheel_y += int2scale(&c->y, dy);
		swy = ms->wheel_y / mouse_wheel_damping;
		ms->wheel_y -= swy * mouse_wheel_damping;
	} else {
		/* pointer */
		ms->rel_x += int2scale(&c->x, dx);
		ms->rel_y += int2scale(&c->y, -dy);
		ms->wheel_x = 0;
		ms->wheel_y = 0;
		sx = ms->rel_x / mouse_motion_damping;
		sy = ms->rel_y / mouse_motion_damping;
		ms->rel_x -= sx * mouse_motion_damping;
		ms->rel_y -= sy * mouse_motion_damping;
	}

	ms->fingers = n;

	input_report_rel(dev, REL_X, sx);
	input_report_rel(dev, REL_Y, sy);
	input_report_rel(dev, REL_HWHEEL, swx);
	input_report_rel(dev, REL_WHEEL, swy);
}

/* update logical touchpad state */
static void update_tp_touchpad_state(struct input_dev *dev,
				const struct bcm5974_config *c,
				const struct tp_finger *f,
				int p, int n)
{
	int w, x, y;

	if (f) {
		w = raw2int(f->size_major);
		x = raw2int(f->abs_x);
		y = raw2int(f->abs_y);

		dprintk(9,
			"bcm5974: p: %+05d w: %+05d x: %+05d y: %+05d n: %d\n",
			p, w, x, y, n);

		input_report_abs(dev, ABS_TOOL_WIDTH, int2bound(&c->w, w));
		input_report_abs(dev, ABS_X, int2bound(&c->x, x - c->x.devmin));
		input_report_abs(dev, ABS_Y, int2bound(&c->y, c->y.devmax - y));
	}

	input_report_abs(dev, ABS_PRESSURE, int2bound(&c->p, p));

	input_report_key(dev, BTN_TOOL_FINGER, n == 1);
	input_report_key(dev, BTN_TOOL_DOUBLETAP, n == 2);
	input_report_key(dev, BTN_TOOL_TRIPLETAP, n > 2);
}

/* report trackpad data as logical mouse/touchpad state */
static int report_tp_state(struct bcm5974 *dev, int size)
{
	const struct bcm5974_config *c = &dev->cfg;
	const int fingers = (size - 26) / 28;
	const struct tp_finger *f = 0;
	int p = 0, n = 0;

	if (size < 26 || (size - 26) % 28 != 0)
		return -EIO;

	if (fingers) {
		f = dev->tp_data->finger;
		p = raw2int(f->force_major);
		n = p > 0 ? fingers : 0;
	}

	switch (driver_mode) {
	case MODE_MOUSE:
		update_tp_mouse_state(dev->input, &dev->ms, c, f, p, n);
		break;
	case MODE_TOUCHPAD:
		update_tp_touchpad_state(dev->input, c, f, p, n);
		break;
	}

	input_sync(dev->input);

	return 0;
}

/* Wellspring initialization constants */
#define BCM5974_WELLSPRING_MODE_READ_REQUEST_ID		1
#define BCM5974_WELLSPRING_MODE_WRITE_REQUEST_ID	9
#define BCM5974_WELLSPRING_MODE_REQUEST_VALUE		0x300
#define BCM5974_WELLSPRING_MODE_REQUEST_INDEX		0
#define BCM5974_WELLSPRING_MODE_VENDOR_VALUE		0x01

static int bcm5974_wellspring_mode(struct bcm5974 *dev)
{
	char *data = kmalloc(8, GFP_KERNEL);
	int retval = 0, size;

	if (!data) {
		err("bcm5974: out of memory");
		retval = -ENOMEM;
		goto out;
	}

	/* read configuration */
	size = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
			BCM5974_WELLSPRING_MODE_READ_REQUEST_ID,
			USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			BCM5974_WELLSPRING_MODE_REQUEST_VALUE,
			BCM5974_WELLSPRING_MODE_REQUEST_INDEX, data, 8, 5000);

	if (size != 8) {
		err("bcm5974: could not read from device");
		retval = -EIO;
		goto out;
	}

	/* apply the mode switch */
	data[0] = BCM5974_WELLSPRING_MODE_VENDOR_VALUE;

	/* write configuration */
	size = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
			BCM5974_WELLSPRING_MODE_WRITE_REQUEST_ID,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			BCM5974_WELLSPRING_MODE_REQUEST_VALUE,
			BCM5974_WELLSPRING_MODE_REQUEST_INDEX, data, 8, 5000);

	if (size != 8) {
		err("bcm5974: could not write to device");
		retval = -EIO;
		goto out;
	}

	dprintk(2, "bcm5974: switched to wellspring mode.\n");

 out:
	kfree(data);
	return retval;
}

static void bcm5974_irq_button(struct urb *urb)
{
	struct bcm5974 *dev = urb->context;
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

	if (report_bt_state(dev, dev->bt_urb->actual_length))
		dprintk(1, "bcm5974: bad button package, length: %d\n",
			dev->bt_urb->actual_length);

exit:
	error = usb_submit_urb(dev->bt_urb, GFP_ATOMIC);
	if (error)
		err("bcm5974: button urb failed: %d", error);
}

static void bcm5974_irq_trackpad(struct urb *urb)
{
	struct bcm5974 *dev = urb->context;
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

	/* control response ignored */
	if (dev->tp_urb->actual_length == 2)
		goto exit;

	if (report_tp_state(dev, dev->tp_urb->actual_length))
		dprintk(1, "bcm5974: bad trackpad package, length: %d\n",
			dev->tp_urb->actual_length);

exit:
	error = usb_submit_urb(dev->tp_urb, GFP_ATOMIC);
	if (error)
		err("bcm5974: trackpad urb failed: %d", error);
}

/*
 * The Wellspring trackpad, like many recent Apple trackpads, share
 * the usb device with the keyboard. Since keyboards are usually
 * handled by the HID system, the device ends up being handled by two
 * modules. Setting up the device therefore becomes slightly
 * complicated. To enable multitouch features, a mode switch is
 * required, which is usually applied via the control interface of the
 * device.  It can be argued where this switch should take place. In
 * some drivers, like appletouch, the switch is made during
 * probe. However, the hid module may also alter the state of the
 * device, resulting in trackpad malfunction under certain
 * circumstances. To get around this problem, there is at least one
 * example that utilizes the USB_QUIRK_RESET_RESUME quirk in order to
 * recieve a reset_resume request rather than the normal resume.
 * Since the implementation of reset_resume is equal to mode switch
 * plus start_traffic, it seems easier to always do the switch when
 * starting traffic on the device.
 */
static int bcm5974_start_traffic(struct bcm5974 *dev)
{
	if (bcm5974_wellspring_mode(dev)) {
		dprintk(1, "bcm5974: mode switch failed\n");
		goto error;
	}

	if (usb_submit_urb(dev->bt_urb, GFP_KERNEL))
		goto error;

	if (usb_submit_urb(dev->tp_urb, GFP_KERNEL))
		goto err_kill_bt;

	return 0;

err_kill_bt:
	usb_kill_urb(dev->bt_urb);
error:
	return -EIO;
}

static void bcm5974_pause_traffic(struct bcm5974 *dev)
{
	usb_kill_urb(dev->tp_urb);
	usb_kill_urb(dev->bt_urb);
}

/*
 * The code below implements open/close and manual suspend/resume.
 * All functions may be called in random order.
 *
 * Opening a suspended device fails with EACCES - permission denied.
 *
 * Failing a resume leaves the device resumed but closed.
 */
static int bcm5974_open(struct input_dev *input)
{
	struct bcm5974 *dev = input_get_drvdata(input);
	int error;

	error = usb_autopm_get_interface(dev->intf);
	if (error)
		return error;

	mutex_lock(&dev->pm_mutex);

	error = bcm5974_start_traffic(dev);
	if (!error)
		dev->opened = 1;

	mutex_unlock(&dev->pm_mutex);

	if (error)
		usb_autopm_put_interface(dev->intf);

	return error;
}

static void bcm5974_close(struct input_dev *input)
{
	struct bcm5974 *dev = input_get_drvdata(input);

	mutex_lock(&dev->pm_mutex);

	bcm5974_pause_traffic(dev);
	dev->opened = 0;

	mutex_unlock(&dev->pm_mutex);

	usb_autopm_put_interface(dev->intf);
}

static int bcm5974_suspend(struct usb_interface *iface, pm_message_t message)
{
	struct bcm5974 *dev = usb_get_intfdata(iface);

	mutex_lock(&dev->pm_mutex);

	if (dev->opened)
		bcm5974_pause_traffic(dev);

	mutex_unlock(&dev->pm_mutex);

	return 0;
}

static int bcm5974_resume(struct usb_interface *iface)
{
	struct bcm5974 *dev = usb_get_intfdata(iface);
	int error = 0;

	mutex_lock(&dev->pm_mutex);

	if (dev->opened)
		error = bcm5974_start_traffic(dev);

	mutex_unlock(&dev->pm_mutex);

	return error;
}

static int bcm5974_probe(struct usb_interface *iface,
			 const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(iface);
	const struct bcm5974_config *cfg;
	struct bcm5974 *dev;
	struct input_dev *input_dev;
	int error = -ENOMEM;

	/* find the product index */
	cfg = bcm5974_get_config(udev);

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(struct bcm5974), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!dev || !input_dev) {
		err("bcm5974: out of memory");
		goto err_free_devs;
	}

	dev->udev = udev;
	dev->intf = iface;
	dev->input = input_dev;
	dev->cfg = *cfg;
	mutex_init(&dev->pm_mutex);

	/* setup urbs */
	dev->bt_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bt_urb)
		goto err_free_devs;

	dev->tp_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->tp_urb)
		goto err_free_bt_urb;

	dev->bt_data = usb_buffer_alloc(dev->udev,
					dev->cfg.bt_datalen, GFP_KERNEL,
					&dev->bt_urb->transfer_dma);
	if (!dev->bt_data)
		goto err_free_urb;

	dev->tp_data = usb_buffer_alloc(dev->udev,
					dev->cfg.tp_datalen, GFP_KERNEL,
					&dev->tp_urb->transfer_dma);
	if (!dev->tp_data)
		goto err_free_bt_buffer;

	usb_fill_int_urb(dev->bt_urb, udev,
			 usb_rcvintpipe(udev, cfg->bt_ep),
			 dev->bt_data, dev->cfg.bt_datalen,
			 bcm5974_irq_button, dev, 1);

	usb_fill_int_urb(dev->tp_urb, udev,
			 usb_rcvintpipe(udev, cfg->tp_ep),
			 dev->tp_data, dev->cfg.tp_datalen,
			 bcm5974_irq_trackpad, dev, 1);

	/* create bcm5974 device */
	usb_make_path(udev, dev->phys, sizeof(dev->phys));
	strlcat(dev->phys, "/input0", sizeof(dev->phys));

	input_dev->name = "bcm5974";
	input_dev->phys = dev->phys;
	usb_to_input_id(dev->udev, &input_dev->id);
	input_dev->dev.parent = &iface->dev;

	input_set_drvdata(input_dev, dev);

	input_dev->open = bcm5974_open;
	input_dev->close = bcm5974_close;

	setup_events_to_report(input_dev, cfg);

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
	input_free_device(input_dev);
	kfree(dev);
	return error;
}

static void bcm5974_disconnect(struct usb_interface *iface)
{
	struct bcm5974 *dev = usb_get_intfdata(iface);

	usb_set_intfdata(iface, NULL);

	input_unregister_device(dev->input);
	usb_buffer_free(dev->udev, dev->cfg.tp_datalen,
			dev->tp_data, dev->tp_urb->transfer_dma);
	usb_buffer_free(dev->udev, dev->cfg.bt_datalen,
			dev->bt_data, dev->bt_urb->transfer_dma);
	usb_free_urb(dev->tp_urb);
	usb_free_urb(dev->bt_urb);
	kfree(dev);
}

static struct usb_driver bcm5974_driver = {
	.name			= "bcm5974",
	.probe			= bcm5974_probe,
	.disconnect		= bcm5974_disconnect,
	.suspend		= bcm5974_suspend,
	.resume			= bcm5974_resume,
	.reset_resume		= bcm5974_resume,
	.id_table		= bcm5974_table,
	.supports_autosuspend	= 1,
};

static int __init bcm5974_init(void)
{
	return usb_register(&bcm5974_driver);
}

static void __exit bcm5974_exit(void)
{
	usb_deregister(&bcm5974_driver);
}

module_init(bcm5974_init);
module_exit(bcm5974_exit);

