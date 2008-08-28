/*
 * Copyright (c) 2004 Takanori Watanabe
 * Copyright (c) 2005 Markus Brueffer <markus@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/acpi_support/acpi_ibm.c,v 1.15 2007/10/25 17:30:18 jhb Exp $
 * $DragonFly: src/sys/dev/acpica5/acpi_thinkpad/acpi_thinkpad.c,v 1.1 2008/08/28 07:10:05 hasso Exp $
 */

/*
 * Driver for extra ACPI-controlled gadgets found on IBM and Lenovo ThinkPad
 * laptops. Inspired by the ibm-acpi and tpb projects which implement these
 * features on Linux.
 *
 *   acpi-ibm: <http://ibm-acpi.sourceforge.net/>
 *        tpb: <http://www.nongnu.org/tpb/>
 */

#include "opt_acpi.h"
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <machine/cpufunc.h>
#include <sys/module.h>
#include <sys/sensors.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/thread2.h>
#include <machine/clock.h>

#include "acpi.h"
#include "acpivar.h"
#include "acpi_if.h"

#define _COMPONENT	ACPI_OEM
ACPI_MODULE_NAME("THINKPAD")

/* Internal methods */
#define ACPI_THINKPAD_METHOD_EVENTS		1
#define ACPI_THINKPAD_METHOD_EVENTMASK		2
#define ACPI_THINKPAD_METHOD_HOTKEY		3
#define ACPI_THINKPAD_METHOD_BRIGHTNESS		4
#define ACPI_THINKPAD_METHOD_VOLUME		5
#define ACPI_THINKPAD_METHOD_MUTE		6
#define ACPI_THINKPAD_METHOD_THINKLIGHT		7
#define ACPI_THINKPAD_METHOD_BLUETOOTH		8
#define ACPI_THINKPAD_METHOD_WLAN		9
#define ACPI_THINKPAD_METHOD_FANSPEED		10
#define ACPI_THINKPAD_METHOD_FANLEVEL		11
#define ACPI_THINKPAD_METHOD_FANSTATUS		12
#define ACPI_THINKPAD_METHOD_THERMAL		13

/* Hotkeys/Buttons */
#define THINKPAD_RTC_HOTKEY1			0x64
#define   THINKPAD_RTC_MASK_HOME		(1 << 0)
#define   THINKPAD_RTC_MASK_SEARCH		(1 << 1)
#define   THINKPAD_RTC_MASK_MAIL		(1 << 2)
#define   THINKPAD_RTC_MASK_WLAN		(1 << 5)
#define THINKPAD_RTC_HOTKEY2			0x65
#define   THINKPAD_RTC_MASK_THINKPAD		(1 << 3)
#define   THINKPAD_RTC_MASK_ZOOM		(1 << 5)
#define   THINKPAD_RTC_MASK_VIDEO		(1 << 6)
#define   THINKPAD_RTC_MASK_HIBERNATE		(1 << 7)
#define THINKPAD_RTC_THINKLIGHT			0x66
#define   THINKPAD_RTC_MASK_THINKLIGHT		(1 << 4)
#define THINKPAD_RTC_SCREENEXPAND		0x67
#define   THINKPAD_RTC_MASK_SCREENEXPAND	(1 << 5)
#define THINKPAD_RTC_BRIGHTNESS			0x6c
#define   THINKPAD_RTC_MASK_BRIGHTNESS		(1 << 5)
#define THINKPAD_RTC_VOLUME			0x6e
#define   THINKPAD_RTC_MASK_VOLUME		(1 << 7)

/* Embedded Controller registers */
#define THINKPAD_EC_BRIGHTNESS			0x31
#define   THINKPAD_EC_MASK_BRI			0x7
#define THINKPAD_EC_VOLUME			0x30
#define   THINKPAD_EC_MASK_VOL			0xf
#define   THINKPAD_EC_MASK_MUTE			(1 << 6)
#define THINKPAD_EC_FANSTATUS			0x2F
#define   THINKPAD_EC_MASK_FANLEVEL		0x3f
#define   THINKPAD_EC_MASK_FANDISENGAGED	(1 << 6)
#define   THINKPAD_EC_MASK_FANSTATUS		(1 << 7)
#define THINKPAD_EC_FANSPEED			0x84

/* CMOS Commands */
#define THINKPAD_CMOS_VOLUME_DOWN		0
#define THINKPAD_CMOS_VOLUME_UP			1
#define THINKPAD_CMOS_VOLUME_MUTE		2
#define THINKPAD_CMOS_BRIGHTNESS_UP		4
#define THINKPAD_CMOS_BRIGHTNESS_DOWN		5

/* ACPI methods */
#define THINKPAD_NAME_KEYLIGHT			"KBLT"
#define THINKPAD_NAME_WLAN_BT_GET		"GBDC"
#define THINKPAD_NAME_WLAN_BT_SET		"SBDC"
#define   THINKPAD_NAME_MASK_BT			(1 << 1)
#define   THINKPAD_NAME_MASK_WLAN		(1 << 2)
#define THINKPAD_NAME_THERMAL_GET		"TMP7"
#define THINKPAD_NAME_THERMAL_UPDT		"UPDT"

#define THINKPAD_NAME_EVENTS_STATUS_GET		"DHKC"
#define THINKPAD_NAME_EVENTS_MASK_GET		"DHKN"
#define THINKPAD_NAME_EVENTS_STATUS_SET		"MHKC"
#define THINKPAD_NAME_EVENTS_MASK_SET		"MHKM"
#define THINKPAD_NAME_EVENTS_GET		"MHKP"
#define THINKPAD_NAME_EVENTS_AVAILMASK		"MHKA"

#define	THINKPAD_NUM_SENSORS			9
#define	THINKPAD_TEMP_SENSORS			8

#define ABS(x) (((x) < 0)? -(x) : (x))

struct acpi_thinkpad_softc {
	device_t	dev;
	ACPI_HANDLE	handle;

	/* Embedded controller */
	device_t	ec_dev;
	ACPI_HANDLE	ec_handle;

	/* CMOS */
	ACPI_HANDLE	cmos_handle;

	/* Fan status */
	ACPI_HANDLE	fan_handle;
	int		fan_levels;

	/* Keylight commands and states */
	ACPI_HANDLE	light_handle;
	int		light_cmd_on;
	int		light_cmd_off;
	int		light_val;
	int		light_get_supported;
	int		light_set_supported;

	/* led(4) interface */
	struct cdev	*led_dev;
	int		led_busy;
	int		led_state;

	int		wlan_bt_flags;
	int		thermal_updt_supported;

	unsigned int	events_availmask;
	unsigned int	events_initialmask;
	int		events_mask_supported;
	int		events_enable;

	/* sensors(9) related */
	struct ksensordev sensordev;
	struct ksensor sensors[THINKPAD_NUM_SENSORS];

	struct sysctl_ctx_list	 sysctl_ctx;
	struct sysctl_oid	*sysctl_tree;
};

static struct {
	char	*name;
	int	method;
	char	*description;
	int	access;
} acpi_thinkpad_sysctls[] = {
	{
		.name		= "events",
		.method		= ACPI_THINKPAD_METHOD_EVENTS,
		.description	= "ACPI events enable",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "eventmask",
		.method		= ACPI_THINKPAD_METHOD_EVENTMASK,
		.description	= "ACPI eventmask",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "hotkey",
		.method		= ACPI_THINKPAD_METHOD_HOTKEY,
		.description	= "Key Status",
		.access		= CTLTYPE_INT | CTLFLAG_RD
	},
	{
		.name		= "lcd_brightness",
		.method		= ACPI_THINKPAD_METHOD_BRIGHTNESS,
		.description	= "LCD Brightness",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "volume",
		.method		= ACPI_THINKPAD_METHOD_VOLUME,
		.description	= "Volume",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "mute",
		.method		= ACPI_THINKPAD_METHOD_MUTE,
		.description	= "Mute",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "thinklight",
		.method		= ACPI_THINKPAD_METHOD_THINKLIGHT,
		.description	= "Thinklight enable",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "bluetooth",
		.method		= ACPI_THINKPAD_METHOD_BLUETOOTH,
		.description	= "Bluetooth enable",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "wlan",
		.method		= ACPI_THINKPAD_METHOD_WLAN,
		.description	= "WLAN enable",
		.access		= CTLTYPE_INT | CTLFLAG_RD
	},
	{
		.name		= "fan_level",
		.method		= ACPI_THINKPAD_METHOD_FANLEVEL,
		.description	= "Fan level",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},
	{
		.name		= "fan",
		.method		= ACPI_THINKPAD_METHOD_FANSTATUS,
		.description	= "Fan enable",
		.access		= CTLTYPE_INT | CTLFLAG_RW
	},

	{ NULL, 0, NULL, 0 }
};

static struct lock tplock;

static int	acpi_thinkpad_probe(device_t dev);
static int	acpi_thinkpad_attach(device_t dev);
static int	acpi_thinkpad_detach(device_t dev);

static int	acpi_thinkpad_sysctl(SYSCTL_HANDLER_ARGS);
static int	acpi_thinkpad_sysctl_init(struct acpi_thinkpad_softc *sc,
		int method);
static int	acpi_thinkpad_sysctl_get(struct acpi_thinkpad_softc *sc,
		int method);
static int	acpi_thinkpad_sysctl_set(struct acpi_thinkpad_softc *sc,
		int method, int val);

static int	acpi_thinkpad_eventmask_set(struct acpi_thinkpad_softc *sc,
		int val);
static void	acpi_thinkpad_notify(ACPI_HANDLE h, UINT32 notify,
		void *context);
static void	acpi_thinkpad_refresh(void *);

static device_method_t acpi_thinkpad_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, acpi_thinkpad_probe),
	DEVMETHOD(device_attach, acpi_thinkpad_attach),
	DEVMETHOD(device_detach, acpi_thinkpad_detach),
	{0, 0}
};

static driver_t	acpi_thinkpad_driver = {
	"acpi_thinkpad",
	acpi_thinkpad_methods,
	sizeof(struct acpi_thinkpad_softc),
};

static devclass_t acpi_thinkpad_devclass;

DRIVER_MODULE(acpi_thinkpad, acpi, acpi_thinkpad_driver,
    acpi_thinkpad_devclass, 0, 0);
MODULE_DEPEND(acpi_thinkpad, acpi, 1, 1, 1);
static char    *thinkpad_ids[] = {"IBM0068", NULL};

static int
acpi_thinkpad_probe(device_t dev)
{
	if (acpi_disabled("thinkpad") ||
	    ACPI_ID_PROBE(device_get_parent(dev), dev, thinkpad_ids) == NULL ||
	    device_get_unit(dev) != 0) 
		return (ENXIO);

	device_set_desc(dev, "IBM/Lenovo ThinkPad ACPI Extras");
	return (0);
}

static int
acpi_thinkpad_attach(device_t dev)
{
	struct acpi_thinkpad_softc	*sc;
	struct acpi_softc	*acpi_sc;
	devclass_t		ec_devclass;
	int			i;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);

	acpi_sc = acpi_device_get_parent_softc(dev);

	/* Look for the first embedded controller */
        if (!(ec_devclass = devclass_find ("acpi_ec"))) {
		if (bootverbose)
			device_printf(dev, "Couldn't find acpi_ec devclass\n");
		return (EINVAL);
	}
        if (!(sc->ec_dev = devclass_get_device(ec_devclass, 0))) {
		if (bootverbose)
			device_printf(dev, "Couldn't find acpi_ec device\n");
		return (EINVAL);
	}
	sc->ec_handle = acpi_get_handle(sc->ec_dev);
	
	lockinit(&tplock, "thinkpad", 0, 0);
	lockmgr(&tplock, LK_EXCLUSIVE);

	sysctl_ctx_init(&sc->sysctl_ctx);
	sc->sysctl_tree = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
	    SYSCTL_CHILDREN(acpi_sc->acpi_sysctl_tree), OID_AUTO,
	    "thinkpad", CTLFLAG_RD, 0, "");

	/* Look for event mask and hook up the nodes */
	sc->events_mask_supported = ACPI_SUCCESS(acpi_GetInteger(sc->handle,
	    THINKPAD_NAME_EVENTS_MASK_GET, &sc->events_initialmask));

	if (sc->events_mask_supported) {
		SYSCTL_ADD_INT(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "initialmask", CTLFLAG_RD,
		    &sc->events_initialmask, 0, "Initial eventmask");

		/* The availmask is the bitmask of supported events */
		if (ACPI_FAILURE(acpi_GetInteger(sc->handle,
		    THINKPAD_NAME_EVENTS_AVAILMASK, &sc->events_availmask)))
			sc->events_availmask = 0xffffffff;

		SYSCTL_ADD_INT(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "availmask", CTLFLAG_RD,
		    &sc->events_availmask, 0, "Mask of supported events");
	}

	/* Hook up proc nodes */
	for (i = 0; acpi_thinkpad_sysctls[i].name != NULL; i++) {
		if (!acpi_thinkpad_sysctl_init(sc,
		    acpi_thinkpad_sysctls[i].method))
			continue;

		SYSCTL_ADD_PROC(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    acpi_thinkpad_sysctls[i].name,
		    acpi_thinkpad_sysctls[i].access,
		    sc, i, acpi_thinkpad_sysctl, "I",
		    acpi_thinkpad_sysctls[i].description);
	}

	lockmgr(&tplock, LK_RELEASE);

	/* Handle notifies */
	AcpiInstallNotifyHandler(sc->handle, ACPI_DEVICE_NOTIFY,
	    acpi_thinkpad_notify, dev);

	/* Attach sensors(9). */
	if (sensor_task_register(sc, acpi_thinkpad_refresh, 5)) {
		device_printf(sc->dev, "unable to register update task\n");
		return 1;
	}

	strlcpy(sc->sensordev.xname, device_get_nameunit(sc->dev),
	    sizeof(sc->sensordev.xname));

	for (i = 0; i < THINKPAD_TEMP_SENSORS; i++) {
		sc->sensors[i].type = SENSOR_TEMP;
		sensor_attach(&sc->sensordev, &sc->sensors[i]);
	}
	
	sc->sensors[i].type = SENSOR_FANRPM;
	sensor_attach(&sc->sensordev, &sc->sensors[i]);

	sensordev_install(&sc->sensordev);

	return (0);
}

static int
acpi_thinkpad_detach(device_t dev)
{
	int i;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	struct acpi_thinkpad_softc *sc = device_get_softc(dev);

	/* Disable events and restore eventmask */
	lockmgr(&tplock, LK_EXCLUSIVE);
	acpi_thinkpad_sysctl_set(sc, ACPI_THINKPAD_METHOD_EVENTS, 0);
	acpi_thinkpad_sysctl_set(sc, ACPI_THINKPAD_METHOD_EVENTMASK,
	    sc->events_initialmask);
	lockmgr(&tplock, LK_RELEASE);

	AcpiRemoveNotifyHandler(sc->handle, ACPI_DEVICE_NOTIFY,
	    acpi_thinkpad_notify);

	if (sc->sysctl_tree != NULL)
		sysctl_ctx_free(&sc->sysctl_ctx);

	sensordev_deinstall(&sc->sensordev);
	for (i = 0; i < THINKPAD_NUM_SENSORS; i++)
		sensor_detach(&sc->sensordev, &sc->sensors[i]);
	sensor_task_unregister(sc);

	return (0);
}

static int
acpi_thinkpad_eventmask_set(struct acpi_thinkpad_softc *sc, int val)
{
	int i;
	ACPI_OBJECT		arg[2];
	ACPI_OBJECT_LIST	args;
	ACPI_STATUS		status;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	KKASSERT(lockstatus(&tplock, curthread) != 0);

	args.Count = 2;
	args.Pointer = arg;
	arg[0].Type = ACPI_TYPE_INTEGER;
	arg[1].Type = ACPI_TYPE_INTEGER;

	for (i = 0; i < 32; ++i) {
		arg[0].Integer.Value = i+1;
		arg[1].Integer.Value = (((1 << i) & val) != 0);
		status = AcpiEvaluateObject(sc->handle,
		    THINKPAD_NAME_EVENTS_MASK_SET, &args, NULL);

		if (ACPI_FAILURE(status))
			return (status);
	}

	return (0);
}

static int
acpi_thinkpad_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct acpi_thinkpad_softc	*sc;
	int			arg;
	int			error = 0;
	int			function;
	int			method;
	
	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	sc = (struct acpi_thinkpad_softc *)oidp->oid_arg1;
	function = oidp->oid_arg2;
	method = acpi_thinkpad_sysctls[function].method;

	lockmgr(&tplock, LK_EXCLUSIVE);
	arg = acpi_thinkpad_sysctl_get(sc, method);
	error = sysctl_handle_int(oidp, &arg, 0, req);

	/* Sanity check */
	if (error != 0 || req->newptr == NULL)
		goto out;

	/* Update */
	error = acpi_thinkpad_sysctl_set(sc, method, arg);

out:
	lockmgr(&tplock, LK_RELEASE);
	return (error);
}

static int
acpi_thinkpad_sysctl_get(struct acpi_thinkpad_softc *sc, int method)
{
	ACPI_INTEGER	val_ec;
	int 		val = 0, key;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	KKASSERT(lockstatus(&tplock, curthread) != 0);

	switch (method) {
	case ACPI_THINKPAD_METHOD_EVENTS:
		acpi_GetInteger(sc->handle, THINKPAD_NAME_EVENTS_STATUS_GET,
		    &val);
		break;

	case ACPI_THINKPAD_METHOD_EVENTMASK:
		if (sc->events_mask_supported)
			acpi_GetInteger(sc->handle,
			    THINKPAD_NAME_EVENTS_MASK_GET, &val);
		break;

	case ACPI_THINKPAD_METHOD_HOTKEY:
		/*
		 * Construct the hotkey as a bitmask as illustrated below.
		 * Note that whenever a key was pressed, the respecting bit
		 * toggles and nothing else changes.
		 * +--+--+-+-+-+-+-+-+-+-+-+-+
		 * |11|10|9|8|7|6|5|4|3|2|1|0|
		 * +--+--+-+-+-+-+-+-+-+-+-+-+
		 *   |  | | | | | | | | | | |
		 *   |  | | | | | | | | | | +- Home Button
		 *   |  | | | | | | | | | +--- Search Button
		 *   |  | | | | | | | | +----- Mail Button
		 *   |  | | | | | | | +------- Thinkpad Button
		 *   |  | | | | | | +--------- Zoom (Fn + Space)
		 *   |  | | | | | +----------- WLAN Button
		 *   |  | | | | +------------- Video Button
		 *   |  | | | +--------------- Hibernate Button
		 *   |  | | +----------------- Thinklight Button
		 *   |  | +------------------- Screen expand (Fn + F8)
		 *   |  +--------------------- Brightness
		 *   +------------------------ Volume/Mute
		 */
		key = rtcin(THINKPAD_RTC_HOTKEY1);
		val = (THINKPAD_RTC_MASK_HOME | THINKPAD_RTC_MASK_SEARCH |
		    THINKPAD_RTC_MASK_MAIL | THINKPAD_RTC_MASK_WLAN) & key;
		key = rtcin(THINKPAD_RTC_HOTKEY2);
		val |= (THINKPAD_RTC_MASK_THINKPAD | THINKPAD_RTC_MASK_VIDEO |
		    THINKPAD_RTC_MASK_HIBERNATE) & key;
		val |= (THINKPAD_RTC_MASK_ZOOM & key) >> 1;
		key = rtcin(THINKPAD_RTC_THINKLIGHT);
		val |= (THINKPAD_RTC_MASK_THINKLIGHT & key) << 4;
		key = rtcin(THINKPAD_RTC_SCREENEXPAND);
		val |= (THINKPAD_RTC_MASK_THINKLIGHT & key) << 4;
		key = rtcin(THINKPAD_RTC_BRIGHTNESS);
		val |= (THINKPAD_RTC_MASK_BRIGHTNESS & key) << 5;
		key = rtcin(THINKPAD_RTC_VOLUME);
		val |= (THINKPAD_RTC_MASK_VOLUME & key) << 4;
		break;

	case ACPI_THINKPAD_METHOD_BRIGHTNESS:
		ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_BRIGHTNESS, &val_ec, 1);
		val = val_ec & THINKPAD_EC_MASK_BRI;
		break;

	case ACPI_THINKPAD_METHOD_VOLUME:
		ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_VOLUME, &val_ec, 1);
		val = val_ec & THINKPAD_EC_MASK_VOL;
		break;

	case ACPI_THINKPAD_METHOD_MUTE:
		ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_VOLUME, &val_ec, 1);
		val = ((val_ec & THINKPAD_EC_MASK_MUTE) ==
		    THINKPAD_EC_MASK_MUTE);
		break;

	case ACPI_THINKPAD_METHOD_THINKLIGHT:
		if (sc->light_get_supported)
			acpi_GetInteger(sc->ec_handle, THINKPAD_NAME_KEYLIGHT,
			    &val);
		else
			val = sc->light_val;
		break;

	case ACPI_THINKPAD_METHOD_BLUETOOTH:
		acpi_GetInteger(sc->handle, THINKPAD_NAME_WLAN_BT_GET, &val);
		sc->wlan_bt_flags = val;
		val = ((val & THINKPAD_NAME_MASK_BT) != 0);
		break;

	case ACPI_THINKPAD_METHOD_WLAN:
		acpi_GetInteger(sc->handle, THINKPAD_NAME_WLAN_BT_GET, &val);
		sc->wlan_bt_flags = val;
		val = ((val & THINKPAD_NAME_MASK_WLAN) != 0);
		break;

	case ACPI_THINKPAD_METHOD_FANSPEED:
		if (sc->fan_handle) {
			if (ACPI_FAILURE(acpi_GetInteger(sc->fan_handle,
			    NULL, &val)))
				val = -1;
		}
		else {
			ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_FANSPEED,
			    &val_ec, 2);
			val = val_ec;
		}
		break;

	case ACPI_THINKPAD_METHOD_FANLEVEL:
		/*
		 * The THINKPAD_EC_FANSTATUS register works as follows:
		 * Bit 0-5 indicate the level at which the fan operates. Only
		 *       values between 0 and 7 have an effect. Everything
		 *       above 7 is treated the same as level 7
		 * Bit 6 overrides the fan speed limit if set to 1
		 * Bit 7 indicates at which mode the fan operates:
		 *       manual (0) or automatic (1)
		 */
		if (!sc->fan_handle) {
			ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_FANSTATUS,
			    &val_ec, 1);
			val = val_ec & THINKPAD_EC_MASK_FANLEVEL;
		}
		break;

	case ACPI_THINKPAD_METHOD_FANSTATUS:
		if (!sc->fan_handle) {
			ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_FANSTATUS,
			    &val_ec, 1);
			val = (val_ec & THINKPAD_EC_MASK_FANSTATUS) ==
			    THINKPAD_EC_MASK_FANSTATUS;
		}
		else
			val = -1;
		break;
	}

	return (val);
}

static int
acpi_thinkpad_sysctl_set(struct acpi_thinkpad_softc *sc, int method, int arg)
{
	int			val, step, i;
	ACPI_INTEGER		val_ec;
	ACPI_OBJECT		Arg;
	ACPI_OBJECT_LIST	Args;
	ACPI_STATUS		status;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	KKASSERT(lockstatus(&tplock, curthread) != 0);

	switch (method) {
	case ACPI_THINKPAD_METHOD_EVENTS:
		if (arg < 0 || arg > 1)
			return (EINVAL);

		status = acpi_SetInteger(sc->handle,
		    THINKPAD_NAME_EVENTS_STATUS_SET, arg);
		if (ACPI_FAILURE(status))
			return (status);
		if (sc->events_mask_supported)
			return acpi_thinkpad_eventmask_set(sc,
			    sc->events_availmask);
		break;

	case ACPI_THINKPAD_METHOD_EVENTMASK:
		if (sc->events_mask_supported)
			return acpi_thinkpad_eventmask_set(sc, arg);
		break;

	case ACPI_THINKPAD_METHOD_BRIGHTNESS:
		if (arg < 0 || arg > 7)
			return (EINVAL);

		if (sc->cmos_handle) {
			/* Read the current brightness */
			status = ACPI_EC_READ(sc->ec_dev,
			    THINKPAD_EC_BRIGHTNESS, &val_ec, 1);
			if (ACPI_FAILURE(status))
				return (status);
			val = val_ec & THINKPAD_EC_MASK_BRI;

			Args.Count = 1;
			Args.Pointer = &Arg;
			Arg.Type = ACPI_TYPE_INTEGER;
			Arg.Integer.Value = (arg > val) ?
			    THINKPAD_CMOS_BRIGHTNESS_UP :
			    THINKPAD_CMOS_BRIGHTNESS_DOWN;

			step = (arg > val) ? 1 : -1;
			for (i = val; i != arg; i += step) {
				status = AcpiEvaluateObject(sc->cmos_handle,
				    NULL, &Args, NULL);
				if (ACPI_FAILURE(status))
					break;
			}
		}
		return ACPI_EC_WRITE(sc->ec_dev, THINKPAD_EC_BRIGHTNESS,
		    arg, 1);
		break;

	case ACPI_THINKPAD_METHOD_VOLUME:
		if (arg < 0 || arg > 14)
			return (EINVAL);

		status = ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_VOLUME,
		    &val_ec, 1);
		if (ACPI_FAILURE(status))
			return (status);

		if (sc->cmos_handle) {
			val = val_ec & THINKPAD_EC_MASK_VOL;

			Args.Count = 1;
			Args.Pointer = &Arg;
			Arg.Type = ACPI_TYPE_INTEGER;
			Arg.Integer.Value = (arg > val) ?
			    THINKPAD_CMOS_VOLUME_UP : THINKPAD_CMOS_VOLUME_DOWN;

			step = (arg > val) ? 1 : -1;
			for (i = val; i != arg; i += step) {
				status = AcpiEvaluateObject(sc->cmos_handle,
				    NULL, &Args, NULL);
				if (ACPI_FAILURE(status))
					break;
			}
		}
		return ACPI_EC_WRITE(sc->ec_dev, THINKPAD_EC_VOLUME, arg +
		    (val_ec & (~THINKPAD_EC_MASK_VOL)), 1);
		break;

	case ACPI_THINKPAD_METHOD_MUTE:
		if (arg < 0 || arg > 1)
			return (EINVAL);

		status = ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_VOLUME,
		    &val_ec, 1);
		if (ACPI_FAILURE(status))
			return (status);

		if (sc->cmos_handle) {
			val = val_ec & THINKPAD_EC_MASK_VOL;

			Args.Count = 1;
			Args.Pointer = &Arg;
			Arg.Type = ACPI_TYPE_INTEGER;
			Arg.Integer.Value = THINKPAD_CMOS_VOLUME_MUTE;

			status = AcpiEvaluateObject(sc->cmos_handle, NULL,
			    &Args, NULL);
			if (ACPI_FAILURE(status))
				break;
		}
		return ACPI_EC_WRITE(sc->ec_dev, THINKPAD_EC_VOLUME, (arg==1) ?
		   val_ec | THINKPAD_EC_MASK_MUTE :
		   val_ec & (~THINKPAD_EC_MASK_MUTE), 1);
		break;

	case ACPI_THINKPAD_METHOD_THINKLIGHT:
		if (arg < 0 || arg > 1)
			return (EINVAL);

		if (sc->light_set_supported) {
			Args.Count = 1;
			Args.Pointer = &Arg;
			Arg.Type = ACPI_TYPE_INTEGER;
			Arg.Integer.Value = arg ?
			    sc->light_cmd_on : sc->light_cmd_off;

			status = AcpiEvaluateObject(sc->light_handle, NULL,
			    &Args, NULL);
			if (ACPI_SUCCESS(status))
				sc->light_val = arg;
			return (status);
		}
		break;

	case ACPI_THINKPAD_METHOD_BLUETOOTH:
		if (arg < 0 || arg > 1)
			return (EINVAL);

		val = (arg == 1) ? sc->wlan_bt_flags |
		    THINKPAD_NAME_MASK_BT :
		    sc->wlan_bt_flags & (~THINKPAD_NAME_MASK_BT);
		return acpi_SetInteger(sc->handle, THINKPAD_NAME_WLAN_BT_SET,
		    val);
		break;

	case ACPI_THINKPAD_METHOD_FANLEVEL:
		if (arg < 0 || arg > 7)
			return (EINVAL);

		if (!sc->fan_handle) {
			/* Read the current fanstatus */
			ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_FANSTATUS,
			    &val_ec, 1);
			val = val_ec & (~THINKPAD_EC_MASK_FANLEVEL);

			return ACPI_EC_WRITE(sc->ec_dev, THINKPAD_EC_FANSTATUS,
			    val | arg, 1);
		}
		break;

	case ACPI_THINKPAD_METHOD_FANSTATUS:
		if (arg < 0 || arg > 1)
			return (EINVAL);

		if (!sc->fan_handle) {
			/* Read the current fanstatus */
			ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_FANSTATUS,
			    &val_ec, 1);

			return ACPI_EC_WRITE(sc->ec_dev, THINKPAD_EC_FANSTATUS,
			    (arg == 1) ? (val_ec | THINKPAD_EC_MASK_FANSTATUS) :
			    (val_ec & (~THINKPAD_EC_MASK_FANSTATUS)), 1);
		}
		break;
	}

	return (0);
}

static int
acpi_thinkpad_sysctl_init(struct acpi_thinkpad_softc *sc, int method)
{
	int 			dummy;
	ACPI_OBJECT_TYPE 	cmos_t;
	ACPI_HANDLE		ledb_handle;

	switch (method) {
	case ACPI_THINKPAD_METHOD_EVENTS:
		/* Events are disabled by default */
		return (TRUE);

	case ACPI_THINKPAD_METHOD_EVENTMASK:
		return (sc->events_mask_supported);

	case ACPI_THINKPAD_METHOD_HOTKEY:
	case ACPI_THINKPAD_METHOD_BRIGHTNESS:
	case ACPI_THINKPAD_METHOD_VOLUME:
	case ACPI_THINKPAD_METHOD_MUTE:
		/* EC is required here, which was already checked before */
		return (TRUE);

	case ACPI_THINKPAD_METHOD_THINKLIGHT:
		sc->cmos_handle = NULL;
		sc->light_get_supported = ACPI_SUCCESS(acpi_GetInteger(
		    sc->ec_handle, THINKPAD_NAME_KEYLIGHT, &sc->light_val));

		if ((ACPI_SUCCESS(AcpiGetHandle(sc->handle, "\\UCMS",
		    &sc->light_handle)) ||
		    ACPI_SUCCESS(AcpiGetHandle(sc->handle, "\\CMOS",
		    &sc->light_handle)) ||
		    ACPI_SUCCESS(AcpiGetHandle(sc->handle, "\\CMS",
		    &sc->light_handle))) &&
		    ACPI_SUCCESS(AcpiGetType(sc->light_handle, &cmos_t)) &&
		    cmos_t == ACPI_TYPE_METHOD) {
			sc->light_cmd_on = 0x0c;
			sc->light_cmd_off = 0x0d;
			sc->cmos_handle = sc->light_handle;
		}
		else if (ACPI_SUCCESS(AcpiGetHandle(sc->handle, "\\LGHT",
		    &sc->light_handle))) {
			sc->light_cmd_on = 1;
			sc->light_cmd_off = 0;
		}
		else
			sc->light_handle = NULL;

		sc->light_set_supported = (sc->light_handle &&
		    ACPI_FAILURE(AcpiGetHandle(sc->ec_handle, "LEDB",
		    &ledb_handle)));

		if (sc->light_get_supported)
			return (TRUE);

		if (sc->light_set_supported) {
			sc->light_val = 0;
			return (TRUE);
		}

		return (FALSE);

	case ACPI_THINKPAD_METHOD_BLUETOOTH:
	case ACPI_THINKPAD_METHOD_WLAN:
		if (ACPI_SUCCESS(acpi_GetInteger(sc->handle,
		    THINKPAD_NAME_WLAN_BT_GET, &dummy)))
			return (TRUE);
		return (FALSE);

	case ACPI_THINKPAD_METHOD_FANSPEED:
		/* 
		 * Some models report the fan speed in levels from 0-7
		 * Newer models report it contiguously
		 */
		sc->fan_levels = (ACPI_SUCCESS(AcpiGetHandle(sc->handle, "GFAN",
		    &sc->fan_handle)) ||
		    ACPI_SUCCESS(AcpiGetHandle(sc->handle, "\\FSPD",
		    &sc->fan_handle)));
		return (TRUE);

	case ACPI_THINKPAD_METHOD_FANLEVEL:
	case ACPI_THINKPAD_METHOD_FANSTATUS:
		/* 
		 * Fan status is only supported on those models,
		 * which report fan RPM contiguously, not in levels
		 */
		if (sc->fan_levels)
			return (FALSE);
		return (TRUE);

	case ACPI_THINKPAD_METHOD_THERMAL:
		if (ACPI_SUCCESS(acpi_GetInteger(sc->ec_handle,
		    THINKPAD_NAME_THERMAL_GET, &dummy))) {
			sc->thermal_updt_supported =
			    ACPI_SUCCESS(acpi_GetInteger(sc->ec_handle,
			    THINKPAD_NAME_THERMAL_UPDT, &dummy));
			return (TRUE);
		}
		return (FALSE);
	}
	return (FALSE);
}

static void
acpi_thinkpad_notify(ACPI_HANDLE h, UINT32 notify, void *context)
{
	int		event, arg, type;
	device_t	dev = context;
	struct acpi_thinkpad_softc *sc = device_get_softc(dev);

	ACPI_FUNCTION_TRACE_U32((char *)(uintptr_t)__func__, notify);

	if (notify != 0x80)
		device_printf(dev, "Unknown notify\n");

	for (;;) {
		acpi_GetInteger(acpi_get_handle(dev), THINKPAD_NAME_EVENTS_GET,
		    &event);

		if (event == 0)
			break;

		type = (event >> 12) & 0xf;
		arg = event & 0xfff;
		switch (type) {
		case 1:
			if (!(sc->events_availmask & (1 << (arg - 1)))) {
				device_printf(dev, "Unknown key %d\n", arg);
				break;
			}

			/* Notify devd(8) */
			device_printf(dev, "THINKPAD event: %u\n",
			    (arg & 0xff));
			acpi_UserNotify("THINKPAD", h, (arg & 0xff));
			break;
		default:
			break;
		}
	}
}

static void
acpi_thinkpad_refresh(void *arg)
{
	struct acpi_thinkpad_softc *sc = (struct acpi_thinkpad_softc *)arg;
	char temp_cmd[] = "TMP0";
	int data, i;
	ACPI_INTEGER speed;

	for (i = 0; i < THINKPAD_TEMP_SENSORS; i++) {
		temp_cmd[3] = '0' + i;
		
                /*
                 * The TMPx methods seem to return +/- 128 or 0
                 * when the respecting sensor is not available
                 */
		sc->sensors[i].flags &= ~SENSOR_FINVALID;
                if (ACPI_FAILURE(acpi_GetInteger(sc->ec_handle, temp_cmd,
                    &data)) || ABS(data) == 128 || data == 0) {
			sc->sensors[i].flags |= SENSOR_FINVALID;
                        data = 0;
		}
                else if (sc->thermal_updt_supported) {
                        /* Temperature is reported in tenth of Kelvin */
                        sc->sensors[i].value = data * 100000;
		}
		sc->sensors[i].value = data * 1000000 + 273150000;
        }
	
	sc->sensors[i].flags &= ~SENSOR_FINVALID;
	if (sc->fan_handle) {
		if (ACPI_FAILURE(acpi_GetInteger(sc->fan_handle,
		    NULL, &data)))
			sc->sensors[i].flags |= SENSOR_FINVALID;
                        data = -1;
        }
        else {
                ACPI_EC_READ(sc->ec_dev, THINKPAD_EC_FANSPEED, &speed, 2);
                data = speed;
        }

	sc->sensors[i].value = data;
}
