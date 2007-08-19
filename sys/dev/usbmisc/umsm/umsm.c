/*	$DragonFly: src/sys/dev/usbmisc/umsm/Attic/umsm.c,v 1.2 2007/08/19 17:16:43 hasso Exp $	*/
/*	$OpenBSD: umsm.c,v 1.15 2007/06/14 10:11:16 mbalmer Exp $	*/

/*
 * Copyright (c) 2006 Jonathan Gray <jsg@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Driver for Qualcomm MSM EVDO and UMTS communication devices */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/conf.h>
#include <sys/tty.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <bus/usb/usb.h>
#include <bus/usb/usbdi.h>
#include <bus/usb/usbdi_util.h>
#include <bus/usb/usbdevs.h>
#include <dev/usbmisc/ucom/ucomvar.h>

#ifdef UMSM_DEBUG
static int	umsmdebug = 1;
#define DPRINTFN(n, x)  do { if (umsmdebug > (n)) printf x; } while (0)
#else
#define DPRINTFN(n, x)
#endif
#define DPRINTF(x) DPRINTFN(0, x)

#define UMSMBUFSZ	2048
#define UMSM_CONFIG_NO	0
#define UMSM_IFACE_NO	0

struct umsm_softc {
	struct ucom_softc	 sc_ucom;
};

struct ucom_callback umsm_callback = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static const struct usb_devno umsm_devs[] = {
	{ USB_VENDOR_AIRPRIME,	USB_PRODUCT_AIRPRIME_PC5220 },
	{ USB_VENDOR_DELL,	USB_PRODUCT_DELL_W5500 },
	{ USB_VENDOR_KYOCERA2,	USB_PRODUCT_KYOCERA2_KPC650 },
	{ USB_VENDOR_NOVATEL,	USB_PRODUCT_NOVATEL_EXPRESSCARD },
	{ USB_VENDOR_NOVATEL,	USB_PRODUCT_NOVATEL_MERLINV620 },
	{ USB_VENDOR_NOVATEL,	USB_PRODUCT_NOVATEL_S720 },
	{ USB_VENDOR_NOVATEL,	USB_PRODUCT_NOVATEL_U720 },
	{ USB_VENDOR_NOVATEL,	USB_PRODUCT_NOVATEL_XU870 },
	{ USB_VENDOR_NOVATEL,	USB_PRODUCT_NOVATEL_ES620 },
	{ USB_VENDOR_QUALCOMM,	USB_PRODUCT_QUALCOMM_MSM_HSDPA },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_EM5625 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_AIRCARD_580 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_AIRCARD_595 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_AIRCARD_875 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC5720 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC5720_2 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC5725 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC8755 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC8755_2 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC8765 },
	{ USB_VENDOR_SIERRA,	USB_PRODUCT_SIERRA_MC8775 },
};

static device_probe_t umsm_match;
static device_attach_t umsm_attach;
static device_detach_t umsm_detach;

static device_method_t umsm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, umsm_match),
	DEVMETHOD(device_attach, umsm_attach),
	DEVMETHOD(device_detach, umsm_detach),
	{ 0, 0 }
};

static driver_t umsm_driver = { 
	"ucom",
	umsm_methods,
	sizeof (struct umsm_softc)
};

DRIVER_MODULE(umsm, uhub, umsm_driver, ucom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(umsm, usb, 1, 1, 1);
MODULE_DEPEND(umsm, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(umsm, 1);

static int
umsm_match(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->iface != NULL)
		return UMATCH_NONE;

	return (usb_lookup(umsm_devs, uaa->vendor, uaa->product) != NULL) ?
	    UMATCH_VENDOR_PRODUCT : UMATCH_NONE;
}

static int
umsm_attach(device_t self)
{
	struct umsm_softc *sc = device_get_softc(self);
	struct usb_attach_arg *uaa = device_get_ivars(self);
	struct ucom_softc *ucom;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	usbd_status error;
	char *devinfo;
	const char *devname;
	int i;

	devinfo = kmalloc(1024, M_USBDEV, M_INTWAIT);
	ucom = &sc->sc_ucom;

	bzero(sc, sizeof (struct umsm_softc));

	usbd_devinfo(uaa->device, 0, devinfo);
	ucom->sc_dev = self;
	device_set_desc_copy(self, devinfo);

	ucom->sc_udev = uaa->device;
	ucom->sc_iface = uaa->iface;

	devname = device_get_nameunit(ucom->sc_dev);
	kprintf("%s: %s\n", devname, devinfo);
	kfree(devinfo, M_USBDEV);

	if (usbd_set_config_index(ucom->sc_udev, UMSM_CONFIG_NO, 1) != 0) {
		device_printf(ucom->sc_dev, "could not set configuration no\n");
		goto error;
	}

	/* get the first interface handle */
	error = usbd_device2interface_handle(ucom->sc_udev, UMSM_IFACE_NO,
	    &ucom->sc_iface);
	if (error != 0) {
		device_printf(ucom->sc_dev, "could not get interface handle\n");
		goto error;
	}

	id = usbd_get_interface_descriptor(ucom->sc_iface);

	ucom->sc_bulkin_no = ucom->sc_bulkout_no = -1;
	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(ucom->sc_iface, i);
		if (ed == NULL) {
			device_printf(ucom->sc_dev, "no endpoint descriptor "
				      "found for %d\n", i);
			goto error;
		}

		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK)
			ucom->sc_bulkin_no = ed->bEndpointAddress;
		else if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK)
			ucom->sc_bulkout_no = ed->bEndpointAddress;
	}
	if (ucom->sc_bulkin_no == -1 || ucom->sc_bulkout_no == -1) {
		device_printf(ucom->sc_dev, "missing endpoint\n");
		goto error;
	}

	ucom->sc_parent = sc;
	ucom->sc_portno = UCOM_UNK_PORTNO;
	ucom->sc_ibufsize = UMSMBUFSZ;
	ucom->sc_obufsize = UMSMBUFSZ;
	ucom->sc_ibufsizepad = UMSMBUFSZ;
	ucom->sc_opkthdrlen = 0;
	ucom->sc_callback = &umsm_callback;

	usbd_add_drv_event(USB_EVENT_DRIVER_ATTACH, ucom->sc_udev,
			   ucom->sc_dev);

	DPRINTF(("umsm: in = 0x%x, out = 0x%x, intr = 0x%x\n",
		 ucom->sc_bulkin_no, ucom->sc_bulkout_no, sc->sc_intr_number));

	ucom_attach(&sc->sc_ucom);

	return 0;

error:
	ucom->sc_dying = 1;
	return ENXIO;
}

static int
umsm_detach(device_t self)
{
	struct umsm_softc *sc = device_get_softc(self);
	int rv = 0;

	DPRINTF(("umsm_detach: sc=%p\n", sc));
	sc->sc_ucom.sc_dying = 1;
	rv = ucom_detach(&sc->sc_ucom);
	usbd_add_drv_event(USB_EVENT_DRIVER_DETACH, sc->sc_ucom.sc_udev,
			   sc->sc_ucom.sc_dev);

	return (rv);
}

#if 0 /* not yet */
int
umsm_activate(struct device *self, enum devact act)
{
	struct umsm_softc *sc = (struct umsm_softc *)self;
	int rv = 0;

	switch (act) {
	case DVACT_ACTIVATE:
		break;

	case DVACT_DEACTIVATE:
		if (sc->sc_subdev != NULL)
			rv = config_deactivate(sc->sc_subdev);
		sc->sc_dying = 1;
		break;
	}
	return (rv);
}
#endif
