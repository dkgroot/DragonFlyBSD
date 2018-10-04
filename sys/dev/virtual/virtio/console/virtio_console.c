/*-
 * Copyright (c) 2014, Bryan Venteicher <bryanv@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Driver for VirtIO console devices. */

#include <sys/cdefs.h>
//__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
//#include <sys/kdb.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sglist.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/queue.h>

#include <sys/conf.h>
#include <sys/cons.h>
#include <sys/tty.h>
#include <sys/ttydefaults.h> 	/* for TTYDEF_* */

/* New */
//#include <sys/stdint.h>
//#include <sys/queue.h>
//#include <sys/types.h>
//#include <sys/condvar.h>
//#include <sys/unistd.h>
//#include <sys/callout.h>
//#include <sys/priv.h>
//#include <sys/serial.h>
//#include <sys/thread2.h>
//#include <sys/conf.h>
/* End New */

#include <sys/bus.h>

#include <dev/virtual/virtio/virtio/virtio.h>
#include <dev/virtual/virtio/virtio/virtqueue.h>
#include <dev/virtual/virtio/console/virtio_console.h>

#define VTCON_MAX_PORTS 32
#define VTCON_TTY_PREFIX "V"
#define VTCON_BULK_BUFSZ 128

/*
 * The buffer cannot cross more than one page boundary due to the
 * size of the sglist segment array used.
 */
CTASSERT(VTCON_BULK_BUFSZ <= PAGE_SIZE);

struct vtcon_softc;
struct vtcon_softc_port;

struct vtcon_port {
	struct lock			 vtcport_lock;
	struct vtcon_softc		*vtcport_sc;
	struct vtcon_softc_port		*vtcport_scport;
	struct tty			*vtcport_tty;
	cdev_t	 			vtcport_cdev;
	struct virtqueue		*vtcport_invq;
	struct virtqueue		*vtcport_outvq;
	int				 vtcport_id;
	int				 vtcport_flags;
#define VTCON_PORT_FLAG_GONE	0x01
#define VTCON_PORT_FLAG_CONSOLE	0x02

#if defined(KDB)
	int				 vtcport_alt_break_state;
#endif
};


// ? -> LWKT
#define VTCON_PORT_MTX(_port)		&(_port)->vtcport_lock
#define VTCON_PORT_LOCK_INIT(_port, _name, _devname) \
					lockinit(VTCON_PORT_MTX(_port), \
						_name # _devname, 0, LK_CANRECURSE)
#define VTCON_PORT_LOCK(_port)		lockmgr(VTCON_PORT_MTX(_port), LK_EXCLUSIVE)
#define VTCON_PORT_UNLOCK(_port)	lockmgr(VTCON_PORT_MTX(_port), LK_RELEASE)
#define VTCON_PORT_DESTROY(_port)	lockuninit(VTCON_PORT_MTX(_port))

struct vtcon_softc_port {
	struct vtcon_softc	*vcsp_sc;
	struct vtcon_port	*vcsp_port;
	struct virtqueue	*vcsp_invq;
	struct virtqueue	*vcsp_outvq;
};

struct vtcon_softc {
	device_t		 vtcon_dev;
	struct lock		 vtcon_lock;
	uint64_t		 vtcon_features;
	uint32_t		 vtcon_max_ports;
	uint32_t		 vtcon_flags;
#define VTCON_FLAG_DETACHED	0x01
#define VTCON_FLAG_SIZE		0x02
#define VTCON_FLAG_MULTIPORT	0x04

	/*
	 * Ports can be added and removed during runtime, but we have
	 * to allocate all the virtqueues during attach. This array is
	 * indexed by the port ID.
	 */
	struct vtcon_softc_port	*vtcon_ports;

	struct task		 vtcon_ctrl_task;
	struct virtqueue	*vtcon_ctrl_rxvq;
	struct virtqueue	*vtcon_ctrl_txvq;
	struct lock		 vtcon_ctrl_tx_lock;
};

// ? -> LWKT
#define VTCON_MTX(_sc)			&(_sc)->vtcon_lock
#define VTCON_LOCK_INIT(_sc, _name, _devname) \
					lockinit(VTCON_MTX(_sc), \
						_name # _devname, 0, LK_CANRECURSE)
#define VTCON_LOCK(_sc)			lockmgr(VTCON_MTX(_sc), LK_EXCLUSIVE)
#define VTCON_UNLOCK(_sc)		lockmgr(VTCON_MTX(_sc), LK_RELEASE)
#define VTCON_LOCK_ASSERT(_sc)		KKASSERT(lockowned(VTCON_MTX(_sc)) != 0)
#define VTCON_LOCK_ASSERT_NOTOWNED(_sc)	KKASSERT(lockowned(VTCON_MTX(_sc)) == 0)
#define VTCON_LOCK_DESTROY(_sc)		lockuninit(VTCON_MTX(_sc))

// ? -> LWKT
#define VTCON_CTRL_TX_MTX(_sc)		&(_sc)->vtcon_ctrl_tx_lock
#define VTCON_CTRL_TX_LOCK_INIT(_sc, _name, _devname) \
					lockinit(VTCON_CTRL_TX_MTX(_sc), \
						_name # _devname, 0, LK_CANRECURSE)
#define VTCON_CTRL_TX_LOCK(_sc)		lockmgr(VTCON_CTRL_TX_MTX(_sc), LK_EXCLUSIVE)
#define VTCON_CTRL_TX_UNLOCK(_sc)	lockmgr(VTCON_CTRL_TX_MTX(_sc), LK_RELEASE)
#define VTCON_CTRL_TX_DESTROY(_sc)	lockuninit(VTCON_CTRL_TX_MTX(_sc))

#define VTCON_ASSERT_VALID_PORTID(_sc, _id)			\
    KASSERT((_id) >= 0 && (_id) < (_sc)->vtcon_max_ports,	\
        ("%s: port ID %d out of range", __func__, _id))

#define VTCON_FEATURES  VIRTIO_CONSOLE_F_MULTIPORT

static struct virtio_feature_desc vtcon_feature_desc[] = {
	{ VIRTIO_CONSOLE_F_SIZE,	"ConsoleSize"	},
	{ VIRTIO_CONSOLE_F_MULTIPORT,	"MultiplePorts"	},
	{ VIRTIO_CONSOLE_F_EMERG_WRITE,	"EmergencyWrite" },

	{ 0, NULL }
};

#define VTCON_DEBUG 1
#if VTCON_DEBUG
#define PDEBUG(a)       (kprintf("%s:%d: ", __func__, __LINE__), kprintf a, kprintf("\n"))
#else
#define PDEBUG(a)	/* nop */
#endif

static int	 vtcon_modevent(module_t, int, void *);
static void	 vtcon_drain_all(void);

static int	 vtcon_probe(device_t);
static int	 vtcon_attach(device_t);
static int	 vtcon_detach(device_t);
static int	 vtcon_config_change(device_t);

static void	 vtcon_setup_features(struct vtcon_softc *);
static void	 vtcon_negotiate_features(struct vtcon_softc *);
static int	 vtcon_alloc_scports(struct vtcon_softc *);
static int	 vtcon_alloc_virtqueues(struct vtcon_softc *);
static void	 vtcon_read_config(struct vtcon_softc *,
		     struct virtio_console_config *);

static void	 vtcon_determine_max_ports(struct vtcon_softc *,
		     struct virtio_console_config *);
static void	 vtcon_destroy_ports(struct vtcon_softc *);
static void	 vtcon_stop(struct vtcon_softc *);

static int	 vtcon_ctrl_event_enqueue(struct vtcon_softc *,
		     struct virtio_console_control *);
static int	 vtcon_ctrl_event_create(struct vtcon_softc *);
static void	 vtcon_ctrl_event_requeue(struct vtcon_softc *,
		     struct virtio_console_control *);
static int	 vtcon_ctrl_event_populate(struct vtcon_softc *);
static void	 vtcon_ctrl_event_drain(struct vtcon_softc *);
static int	 vtcon_ctrl_init(struct vtcon_softc *);
static void	 vtcon_ctrl_deinit(struct vtcon_softc *);
static void	 vtcon_ctrl_port_add_event(struct vtcon_softc *, int);
static void	 vtcon_ctrl_port_remove_event(struct vtcon_softc *, int);
static void	 vtcon_ctrl_port_console_event(struct vtcon_softc *, int);
static void	 vtcon_ctrl_port_open_event(struct vtcon_softc *, int);
static void	 vtcon_ctrl_port_name_event(struct vtcon_softc *, int,
		     const char *, size_t);
static void	 vtcon_ctrl_process_event(struct vtcon_softc *,
		     struct virtio_console_control *, void *, size_t);
static void	 vtcon_ctrl_task_cb(void *, int);
static void	 vtcon_ctrl_event_intr(void *);
static void	 vtcon_ctrl_poll(struct vtcon_softc *,
		     struct virtio_console_control *control);
static void	 vtcon_ctrl_send_control(struct vtcon_softc *, uint32_t,
		     uint16_t, uint16_t);

static int	 vtcon_port_enqueue_buf(struct vtcon_port *, void *, size_t);
static int	 vtcon_port_create_buf(struct vtcon_port *);
static void	 vtcon_port_requeue_buf(struct vtcon_port *, void *);
static int	 vtcon_port_populate(struct vtcon_port *);
static void	 vtcon_port_destroy(struct vtcon_port *);
static int	 vtcon_port_create(struct vtcon_softc *, int);
static void	 vtcon_port_drain_bufs(struct virtqueue *);
static void	 vtcon_port_drain(struct vtcon_port *);
static void	 vtcon_port_teardown(struct vtcon_port *);
static void	 vtcon_port_change_size(struct vtcon_port *, uint16_t,
		     uint16_t);
static void	 vtcon_port_update_console_size(struct vtcon_softc *);
static void	 vtcon_port_enable_intr(struct vtcon_port *);
static void	 vtcon_port_disable_intr(struct vtcon_port *);
static void	 vtcon_port_in(struct vtcon_port *);
static void	 vtcon_port_intr(void *);
static void	 vtcon_port_out(struct vtcon_port *, void *, int);
static void	 vtcon_port_submit_event(struct vtcon_port *, uint16_t,
		     uint16_t);

static void	 vtcon_get_console_size(struct vtcon_softc *, uint16_t *,
		     uint16_t *);

static void	 vtcon_enable_interrupts(struct vtcon_softc *);
static void	 vtcon_disable_interrupts(struct vtcon_softc *);

static int	 vtcon_pending_free;

// ? new
static void	 vtcon_tty_start(struct tty *tp);
static void 	 vtcon_tty_stop(struct tty *tp, int);
static int	 vtcon_tty_param(struct tty *tp, struct termios *t);

/* // ? fbsd
static int	 vtcon_tty_open(struct tty *);
static void	 vtcon_tty_close(struct tty *);
static void	 vtcon_tty_outwakeup(struct tty *);
static void	 vtcon_tty_free(void *);

static struct ttydevsw vtcon_tty_class = {
	.tsw_flags	= 0,
	.tsw_open	= vtcon_tty_open,
	.tsw_close	= vtcon_tty_close,
	.tsw_outwakeup	= vtcon_tty_outwakeup,
	.tsw_free	= vtcon_tty_free,
};
*/

#define CDEV_MAJOR      18
static  d_open_t        vtcon_tty_open;
static  d_close_t       vtcon_tty_close;
//static  d_read_t        vtcon_tty_read;
//static  d_write_t       vtcon_tty_write;
//static  d_ioctl_t       vtcon_tty_ioctl;

static struct dev_ops vtcon_ops = {
  { "vtcon", 0, D_MPSAFE | D_TTY },
  .d_open =       vtcon_tty_open,
  .d_close =      vtcon_tty_close,
  //.d_read =       vtcon_tty_read,
  //.d_write =      vtcon_tty_write,
  //.d_ioctl =      vtcon_tty_ioctl,
  .d_kqfilter =   ttykqfilter,
  .d_revoke =     ttyrevoke
};

/* end new */

static device_method_t vtcon_methods[] = {
	/* Device methods. */
	DEVMETHOD(device_probe,		vtcon_probe),
	DEVMETHOD(device_attach,	vtcon_attach),
	DEVMETHOD(device_detach,	vtcon_detach),
	/* VirtIO methods. */
	//DEVMETHOD(virtio_config_change,	vtcon_config_change),		// ? --> Implement using virtio interrupt (like in virtio_balloon)

	DEVMETHOD_END
};

static driver_t vtcon_driver = {
	"vtcon",
	vtcon_methods,
	sizeof(struct vtcon_softc)
};
static devclass_t vtcon_devclass;

DRIVER_MODULE(virtio_console, virtio_pci, vtcon_driver, vtcon_devclass,
    vtcon_modevent, 0);
MODULE_VERSION(virtio_console, 1);
MODULE_DEPEND(virtio_console, virtio, 1, 1, 1);

static int
vtcon_modevent(module_t mod, int type, void *unused)
{
	kprintf("-------------------------\n");
	PDEBUG(("modevent type:%d", type));
	int error;

	switch (type) {
	case MOD_LOAD:
		error = 0;
		break;
//	case MOD_QUIESCE:			// ? fbsd
//		error = 0;
//		break;
	case MOD_UNLOAD:
		vtcon_drain_all();
		error = 0;
		break;
	case MOD_SHUTDOWN:
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static void
vtcon_drain_all(void)
{
	PDEBUG(("%s", ""));
	int first;

	for (first = 1; vtcon_pending_free != 0; first = 0) {
		if (first != 0) {
			kprintf("virtio_console: Waiting for all detached TTY "
			    "devices to have open fds closed.\n");
		}
		//pause("vtcondra", hz);
	}
}

static int
vtcon_probe(device_t dev)
{
	PDEBUG(("dev:%s", device_get_nameunit(dev)));

	if (virtio_get_device_type(dev) != VIRTIO_ID_CONSOLE)
		return (ENXIO);

	device_set_desc(dev, "VirtIO Console Adapter");

	return (BUS_PROBE_DEFAULT);
}

static int
vtcon_attach(device_t dev)
{
	PDEBUG(("dev:%s", device_get_nameunit(dev)));
	struct vtcon_softc *sc;
	struct virtio_console_config concfg;
	int error;

	sc = device_get_softc(dev);
	sc->vtcon_dev = dev;

	VTCON_LOCK_INIT(sc, "vtconmtx:", device_get_nameunit(dev));
	VTCON_CTRL_TX_LOCK_INIT(sc, "vtconctrlmtx:", device_get_nameunit(dev));

	virtio_set_feature_desc(dev, vtcon_feature_desc);
	vtcon_setup_features(sc);

	vtcon_read_config(sc, &concfg);
	vtcon_determine_max_ports(sc, &concfg);

	error = vtcon_alloc_scports(sc);
	if (error) {
		device_printf(dev, "cannot allocate softc port structures (error:%d)\n", error);
		goto fail;
	}

	error = vtcon_alloc_virtqueues(sc);
	if (error) {
		device_printf(dev, "cannot allocate virtqueues, (error:%d)\n", error);
		goto fail;
	}

	if (sc->vtcon_flags & VTCON_FLAG_MULTIPORT) {
		TASK_INIT(&sc->vtcon_ctrl_task, 0, vtcon_ctrl_task_cb, sc);
		error = vtcon_ctrl_init(sc);
		if (error) {
			device_printf(dev, "multiport vtcon control init failed, (error:%d)\n", error);
			goto fail;
		}
	} else {
		error = vtcon_port_create(sc, 0);
		if (error) {
			device_printf(dev, "vtcon port create failed, (error:%d)\n", error);
			goto fail;
		}
		if (sc->vtcon_flags & VTCON_FLAG_SIZE)
			vtcon_port_update_console_size(sc);
	}

	//error = virtio_setup_intr(dev, INTR_TYPE_TTY);
	error = virtio_setup_intr(dev, 0, NULL);	// INTR_TYPE_TTY ?
	if (error) {
		device_printf(dev, "cannot setup virtqueue interrupts (error:%d)\n", error);
		goto fail;
	}

	vtcon_enable_interrupts(sc);

	vtcon_ctrl_send_control(sc, VIRTIO_CONSOLE_BAD_ID,
	    VIRTIO_CONSOLE_DEVICE_READY, 1);

fail:
	if (error)
		vtcon_detach(dev);

	return (error);
}

static int
vtcon_detach(device_t dev)
{
	PDEBUG(("dev:%s", device_get_nameunit(dev)));
	struct vtcon_softc *sc;

	sc = device_get_softc(dev);

	VTCON_LOCK(sc);
	sc->vtcon_flags |= VTCON_FLAG_DETACHED;
	if (device_is_attached(dev))
		vtcon_stop(sc);
	VTCON_UNLOCK(sc);

	if (sc->vtcon_flags & VTCON_FLAG_MULTIPORT) {
		//taskqueue_drain(taskqueue_thread, &sc->vtcon_ctrl_task);
		taskqueue_drain(*taskqueue_thread, &sc->vtcon_ctrl_task);
		vtcon_ctrl_deinit(sc);
	}

	vtcon_destroy_ports(sc);
	VTCON_LOCK_DESTROY(sc);
	VTCON_CTRL_TX_DESTROY(sc);

	return (0);
}

static int
vtcon_config_change(device_t dev)
{
	PDEBUG(("dev:%s", device_get_nameunit(dev)));
	struct vtcon_softc *sc;

	sc = device_get_softc(dev);

	/*
	 * When the multiport feature is negotiated, all configuration
	 * changes are done through control virtqueue events.
	 */
	if ((sc->vtcon_flags & VTCON_FLAG_MULTIPORT) == 0) {
		if (sc->vtcon_flags & VTCON_FLAG_SIZE)
			vtcon_port_update_console_size(sc);
	}

	return (0);
}

static void
vtcon_negotiate_features(struct vtcon_softc *sc)
{
	device_t dev;
	uint64_t features;

	dev = sc->vtcon_dev;
	features = VTCON_FEATURES;
	
	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	sc->vtcon_features = virtio_negotiate_features(dev, features);
}

static void
vtcon_setup_features(struct vtcon_softc *sc)
{
	device_t dev;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	vtcon_negotiate_features(sc);

	if (virtio_with_feature(dev, VIRTIO_CONSOLE_F_SIZE))
		sc->vtcon_flags |= VTCON_FLAG_SIZE;

//	temp remarked out -> causes issue in vtcon_alloc_virtqueues if enabled
//	if (virtio_with_feature(dev, VIRTIO_CONSOLE_F_MULTIPORT))
//		sc->vtcon_flags |= VTCON_FLAG_MULTIPORT;
}

#define VTCON_GET_CONFIG(_dev, _feature, _field, _cfg)			\
	if (virtio_with_feature(_dev, _feature)) {			\
		virtio_read_device_config(_dev,				\
		    offsetof(struct virtio_console_config, _field),	\
		    &(_cfg)->_field, sizeof((_cfg)->_field));		\
	}

static void
vtcon_read_config(struct vtcon_softc *sc, struct virtio_console_config *concfg)
{
	device_t dev;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	bzero(concfg, sizeof(struct virtio_console_config));

	VTCON_GET_CONFIG(dev, VIRTIO_CONSOLE_F_SIZE, cols, concfg);
	VTCON_GET_CONFIG(dev, VIRTIO_CONSOLE_F_SIZE, rows, concfg);
	VTCON_GET_CONFIG(dev, VIRTIO_CONSOLE_F_MULTIPORT, max_nr_ports, concfg);
}

#undef VTCON_GET_CONFIG

static int
vtcon_alloc_scports(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));
	struct vtcon_softc_port *scport;
	int max, i;

	max = sc->vtcon_max_ports;

	//sc->vtcon_ports = malloc(sizeof(struct vtcon_softc_port) * max,
	//    M_DEVBUF, M_NOWAIT | M_ZERO);
	sc->vtcon_ports = contigmalloc(sizeof(struct vtcon_softc_port) * max,
		M_DEVBUF, M_WAITOK | M_ZERO, 0, BUS_SPACE_MAXADDR, 16, 0);
	if (sc->vtcon_ports == NULL)
		return (ENOMEM);

	for (i = 0; i < max; i++) {
		scport = &sc->vtcon_ports[i];
		scport->vcsp_sc = sc;
	}

	return (0);
}

static int
vtcon_alloc_virtqueues(struct vtcon_softc *sc)
{
	device_t dev;
	//struct vq_alloc_info *info;		// fbsd
	struct vtcon_softc_port *scport;
	int i, idx, portidx, nvqs, error;

	dev = sc->vtcon_dev;
	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	nvqs = sc->vtcon_max_ports * 2;
	if (sc->vtcon_flags & VTCON_FLAG_MULTIPORT)
		nvqs += 2;

	//info = malloc(sizeof(struct vq_alloc_info) * nvqs, M_TEMP, M_NOWAIT);					// ? fbsd -> replaced using static info[]
	//info = contigmalloc(sizeof(struct vq_alloc_info) * nvqs, M_TEMP, M_NOWAIT, 0, BUS_SPACE_MAXADDR, 16, 0);
	//if (info == NULL)
	//	return (ENOMEM);
	struct vq_alloc_info info[nvqs];

	for (i = 0, idx = 0, portidx = 0; i < nvqs / 2; i++, idx += 2) {

		if (i == 1) {
			/* The control virtqueues are after the first port. */
			//VQ_ALLOC_INFO_INIT(&info[idx], 0,
			//    vtcon_ctrl_event_intr, sc, &sc->vtcon_ctrl_rxvq,
			//    "%s-control rx", device_get_nameunit(dev));
			//VQ_ALLOC_INFO_INIT(&info[idx+1], 0,
			//    NULL, sc, &sc->vtcon_ctrl_txvq,
			//   "%s-control tx", device_get_nameunit(dev));
			VQ_ALLOC_INFO_INIT(&info[idx], 0,
			    &sc->vtcon_ctrl_rxvq,
			    "%s-control rx", device_get_nameunit(dev));
			VQ_ALLOC_INFO_INIT(&info[idx+1], 0,
			    &sc->vtcon_ctrl_txvq,
			    "%s-control tx", device_get_nameunit(dev));
			continue;
		}

		scport = &sc->vtcon_ports[portidx];

		//VQ_ALLOC_INFO_INIT(&info[idx], 0, vtcon_port_intr,
		//    scport, &scport->vcsp_invq, "%s-port%d in",
		//    device_get_nameunit(dev), i);
		//VQ_ALLOC_INFO_INIT(&info[idx+1], 0, NULL,
		//    NULL, &scport->vcsp_outvq, "%s-port%d out",
		//    device_get_nameunit(dev), i);
		VQ_ALLOC_INFO_INIT(&info[idx], 0,
		    &scport->vcsp_invq, "%s-port%d in",
		    device_get_nameunit(dev), i);
		VQ_ALLOC_INFO_INIT(&info[idx+1], 0,
		    &scport->vcsp_outvq, "%s-port%d out",
		    device_get_nameunit(dev), i);

		portidx++;
	}

	PDEBUG(("call virtio_alloc_virtqueues with (dev:%s, nvqs:%d, info:%p) and portidx:%d", device_get_nameunit(dev), nvqs, info, portidx));
	//error = virtio_alloc_virtqueues(dev, 0, nvqs, info);
	error = virtio_alloc_virtqueues(dev, nvqs, info);
	//free(info, M_TEMP);
	//contigfree(info, sizeof(struct vq_alloc_info) * nvqs, M_TEMP);	//-> using info[nvqs] instead of *info

	return (error);
}

static void
vtcon_determine_max_ports(struct vtcon_softc *sc,
    struct virtio_console_config *concfg)
{
	PDEBUG(("sofc:%p", sc));

	if (sc->vtcon_flags & VTCON_FLAG_MULTIPORT) {
		sc->vtcon_max_ports =
		    min(concfg->max_nr_ports, VTCON_MAX_PORTS);
		if (sc->vtcon_max_ports == 0)
			sc->vtcon_max_ports = 1;
	} else
		sc->vtcon_max_ports = 1;

	PDEBUG(("maxports:%d", sc->vtcon_max_ports));
}

static void
vtcon_destroy_ports(struct vtcon_softc *sc)
{
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;
	struct virtqueue *vq;
	int i;
	PDEBUG(("sofc:%p", sc));

	if (sc->vtcon_ports == NULL)
		return;

	VTCON_LOCK(sc);
	for (i = 0; i < sc->vtcon_max_ports; i++) {
		scport = &sc->vtcon_ports[i];

		port = scport->vcsp_port;
		if (port != NULL) {
			scport->vcsp_port = NULL;
			VTCON_PORT_LOCK(port);
			VTCON_UNLOCK(sc);
			vtcon_port_teardown(port);
			VTCON_LOCK(sc);
		}

		vq = scport->vcsp_invq;
		if (vq != NULL)
			vtcon_port_drain_bufs(vq);
	}
	VTCON_UNLOCK(sc);

	//free(sc->vtcon_ports, M_DEVBUF);
	contigfree(sc->vtcon_ports, sizeof(struct vtcon_softc_port) * sc->vtcon_max_ports, M_DEVBUF);
	sc->vtcon_ports = NULL;
}

static void
vtcon_stop(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));

	vtcon_disable_interrupts(sc);
	virtio_stop(sc->vtcon_dev);
}

static int
vtcon_ctrl_event_enqueue(struct vtcon_softc *sc,
    struct virtio_console_control *control)
{
	PDEBUG(("sofc:%p", sc));

	struct sglist_seg segs[2];
	struct sglist sg;
	struct virtqueue *vq;
	int error;

	vq = sc->vtcon_ctrl_rxvq;

	sglist_init(&sg, 2, segs);
	error = sglist_append(&sg, control,
	    sizeof(struct virtio_console_control));
	KASSERT(error == 0, ("%s: error %d adding control to sglist",
	    __func__, error));

	return (virtqueue_enqueue(vq, control, &sg, 0, sg.sg_nseg));
}

static int
vtcon_ctrl_event_create(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));

	struct virtio_console_control *control;
	int error;

	//control = malloc(
	//    sizeof(struct virtio_console_control) + VTCON_BULK_BUFSZ,
	//    M_DEVBUF, M_ZERO | M_NOWAIT);
	control = contigmalloc(sizeof(struct virtio_console_control) + 
		VTCON_BULK_BUFSZ, M_DEVBUF, M_ZERO | M_NOWAIT, 0, BUS_SPACE_MAXADDR, 16, 0);

	if (control == NULL)
		return (ENOMEM);

	error = vtcon_ctrl_event_enqueue(sc, control);
	if (error) {
		//free(control, M_DEVBUF);
		contigfree(control, sizeof(struct virtio_console_control) + VTCON_BULK_BUFSZ, M_DEVBUF);
	}

	return (error);
}

static void
vtcon_ctrl_event_requeue(struct vtcon_softc *sc,
    struct virtio_console_control *control)
{
	PDEBUG(("sofc:%p", sc));

	int error;

	bzero(control, sizeof(struct virtio_console_control));

	error = vtcon_ctrl_event_enqueue(sc, control);
	KASSERT(error == 0,
	    ("%s: cannot requeue control buffer %d", __func__, error));
}

static int
vtcon_ctrl_event_populate(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));

	struct virtqueue *vq;
	int nbufs, error;

	vq = sc->vtcon_ctrl_rxvq;
	error = ENOSPC;

	for (nbufs = 0; !virtqueue_full(vq); nbufs++) {
		error = vtcon_ctrl_event_create(sc);
		if (error)
			break;
	}

	if (nbufs > 0) {
		//virtqueue_notify(vq);			// fbsd
		virtqueue_notify(vq, NULL);
		error = 0;
	}

	return (error);
}

static void
vtcon_ctrl_event_drain(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));

	struct virtio_console_control *control;
	struct virtqueue *vq;
	int last;

	vq = sc->vtcon_ctrl_rxvq;
	last = 0;

	if (vq == NULL)
		return;

	VTCON_LOCK(sc);
	while ((control = virtqueue_drain(vq, &last)) != NULL) {
		//free(control, M_DEVBUF);
		contigfree(control, sizeof(struct virtio_console_control) + VTCON_BULK_BUFSZ, M_DEVBUF);
	}
	VTCON_UNLOCK(sc);
}

static int
vtcon_ctrl_init(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));

	int error;

	error = vtcon_ctrl_event_populate(sc);

	return (error);
}

static void
vtcon_ctrl_deinit(struct vtcon_softc *sc)
{
	PDEBUG(("sofc:%p", sc));

	vtcon_ctrl_event_drain(sc);
}

static void
vtcon_ctrl_port_add_event(struct vtcon_softc *sc, int id)
{
	device_t dev;
	int error;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	/* This single thread only way for ports to be created. */
	if (sc->vtcon_ports[id].vcsp_port != NULL) {
		device_printf(dev, "%s: adding port %d, but already exists\n",
		    __func__, id);
		return;
	}

	error = vtcon_port_create(sc, id);
	if (error) {
		device_printf(dev, "%s: cannot create port %d: %d\n",
		    __func__, id, error);
		vtcon_ctrl_send_control(sc, id, VIRTIO_CONSOLE_PORT_READY, 0);
		return;
	}
}

static void
vtcon_ctrl_port_remove_event(struct vtcon_softc *sc, int id)
{
	device_t dev;
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	scport = &sc->vtcon_ports[id];

	VTCON_LOCK(sc);
	port = scport->vcsp_port;
	if (port == NULL) {
		VTCON_UNLOCK(sc);
		device_printf(dev, "%s: remove port %d, but does not exist\n",
		    __func__, id);
		return;
	}

	scport->vcsp_port = NULL;
	VTCON_PORT_LOCK(port);
	VTCON_UNLOCK(sc);
	vtcon_port_teardown(port);
}

static void
vtcon_ctrl_port_console_event(struct vtcon_softc *sc, int id)
{
	device_t dev;
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	scport = &sc->vtcon_ports[id];

	VTCON_LOCK(sc);
	port = scport->vcsp_port;
	if (port == NULL) {
		VTCON_UNLOCK(sc);
		device_printf(dev, "%s: console port %d, but does not exist\n",
		    __func__, id);
		return;
	}

	VTCON_PORT_LOCK(port);
	VTCON_UNLOCK(sc);
	port->vtcport_flags |= VTCON_PORT_FLAG_CONSOLE;
	vtcon_port_submit_event(port, VIRTIO_CONSOLE_PORT_OPEN, 1);
	VTCON_PORT_UNLOCK(port);
}

static void
vtcon_ctrl_port_open_event(struct vtcon_softc *sc, int id)
{
	device_t dev;
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	scport = &sc->vtcon_ports[id];

	VTCON_LOCK(sc);
	port = scport->vcsp_port;
	if (port == NULL) {
		VTCON_UNLOCK(sc);
		device_printf(dev, "%s: open port %d, but does not exist\n",
		    __func__, id);
		return;
	}

	VTCON_PORT_LOCK(port);
	VTCON_UNLOCK(sc);
	vtcon_port_enable_intr(port);
	VTCON_PORT_UNLOCK(port);
}

static void
vtcon_ctrl_port_name_event(struct vtcon_softc *sc, int id, const char *name,
    size_t len)
{
	device_t dev;
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	scport = &sc->vtcon_ports[id];

	port = scport->vcsp_port;
	if (port == NULL) {
		device_printf(dev, "%s: name port %d, but does not exist\n",
		    __func__, id);
		return;
	}

	//tty_makealias(port->vtcport_tty, "vtcon/%*s", (int)len, name);  // ?	
}

static void
vtcon_ctrl_process_event(struct vtcon_softc *sc,
    struct virtio_console_control *control, void *payload, size_t plen)
{
	device_t dev;
	int id;

	dev = sc->vtcon_dev;

	PDEBUG(("sofc:%p, dev:%s", sc, device_get_nameunit(dev)));

	id = control->id;

	if (id < 0 || id >= sc->vtcon_max_ports) {
		device_printf(dev, "%s: invalid port ID %d\n", __func__, id);
		return;
	}

	switch (control->event) {
	case VIRTIO_CONSOLE_PORT_ADD:
		vtcon_ctrl_port_add_event(sc, id);
		break;

	case VIRTIO_CONSOLE_PORT_REMOVE:
		vtcon_ctrl_port_remove_event(sc, id);
		break;

	case VIRTIO_CONSOLE_CONSOLE_PORT:
		vtcon_ctrl_port_console_event(sc, id);
		break;

	case VIRTIO_CONSOLE_RESIZE:
		break;

	case VIRTIO_CONSOLE_PORT_OPEN:
		vtcon_ctrl_port_open_event(sc, id);
		break;

	case VIRTIO_CONSOLE_PORT_NAME:
		if (payload != NULL && plen > 0)
			vtcon_ctrl_port_name_event(sc, id,
			    (const char *)payload, plen);
		break;
	}
}

static void
vtcon_ctrl_task_cb(void *xsc, int pending)
{
	PDEBUG(("xsc:%p", xsc));

	struct vtcon_softc *sc;
	struct virtqueue *vq;
	struct virtio_console_control *control;
	int detached;
	uint32_t len;
	size_t plen;
	void *payload;

	sc = xsc;
	vq = sc->vtcon_ctrl_rxvq;

	VTCON_LOCK(sc);

	while ((detached = (sc->vtcon_flags & VTCON_FLAG_DETACHED)) == 0) {
		control = virtqueue_dequeue(vq, &len);
		payload = NULL;
		plen = 0;

		if (control == NULL)
			break;

		if (len > sizeof(control)) {
			payload = (void *)(control + 1);
			plen = len - sizeof(control);
		}

		VTCON_UNLOCK(sc);
		vtcon_ctrl_process_event(sc, control, payload, plen);
		VTCON_LOCK(sc);
		vtcon_ctrl_event_requeue(sc, control);
	}

	if (!detached) {
		//virtqueue_notify(vq);
		virtqueue_notify(vq, NULL);
		if (virtqueue_enable_intr(vq) != 0) {
			//taskqueue_enqueue(taskqueue_thread,
			//    &sc->vtcon_ctrl_task);
			taskqueue_enqueue(*taskqueue_thread,
			    &sc->vtcon_ctrl_task);
		}
	}

	VTCON_UNLOCK(sc);
}

static void
vtcon_ctrl_event_intr(void *xsc)
{
	PDEBUG(("xsc:%p", xsc));
	struct vtcon_softc *sc;

	sc = xsc;

	/*
	 * Only some events require us to potentially block, but it
	 * easier to just defer all event handling to the taskqueue.
	 */
	//taskqueue_enqueue(taskqueue_thread, &sc->vtcon_ctrl_task);
	taskqueue_enqueue(*taskqueue_thread, &sc->vtcon_ctrl_task);
}

static void
vtcon_ctrl_poll(struct vtcon_softc *sc,
    struct virtio_console_control *control)
{
	struct sglist_seg segs[2];
	struct sglist sg;
	struct virtqueue *vq;
	int error;

	PDEBUG(("softc:%p", sc));

	vq = sc->vtcon_ctrl_txvq;

	sglist_init(&sg, 2, segs);
	error = sglist_append(&sg, control,
	    sizeof(struct virtio_console_control));
	KASSERT(error == 0, ("%s: error %d adding control to sglist",
	    __func__, error));

	/*
	 * We cannot use the softc lock to serialize access to this
	 * virtqueue since this is called from the tty layer with the
	 * port lock held. Acquiring the softc would violate our lock
	 * ordering.
	 */
	VTCON_CTRL_TX_LOCK(sc);
	KASSERT(virtqueue_empty(vq),
	    ("%s: virtqueue is not emtpy", __func__));
	error = virtqueue_enqueue(vq, control, &sg, sg.sg_nseg, 0);
	if (error == 0) {
		//virtqueue_notify(vq);
		virtqueue_notify(vq, NULL);
		virtqueue_poll(vq, NULL);
	}
	VTCON_CTRL_TX_UNLOCK(sc);
}

static void
vtcon_ctrl_send_control(struct vtcon_softc *sc, uint32_t portid,
    uint16_t event, uint16_t value)
{
	PDEBUG(("softc:%p", sc));

	struct virtio_console_control control;

	if ((sc->vtcon_flags & VTCON_FLAG_MULTIPORT) == 0)
		return;

	control.id = portid;
	control.event = event;
	control.value = value;

	vtcon_ctrl_poll(sc, &control);
}

static int
vtcon_port_enqueue_buf(struct vtcon_port *port, void *buf, size_t len)
{
	PDEBUG(("port:%p", port));
	struct sglist_seg segs[2];
	struct sglist sg;
	struct virtqueue *vq;
	int error;

	vq = port->vtcport_invq;

	sglist_init(&sg, 2, segs);
	error = sglist_append(&sg, buf, len);
	KASSERT(error == 0,
	    ("%s: error %d adding buffer to sglist", __func__, error));

	error = virtqueue_enqueue(vq, buf, &sg, 0, sg.sg_nseg);

	return (error);
}

static int
vtcon_port_create_buf(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));
	void *buf;
	int error;

	//buf = malloc(VTCON_BULK_BUFSZ, M_DEVBUF, M_ZERO | M_NOWAIT);
	buf = contigmalloc(VTCON_BULK_BUFSZ, M_DEVBUF, M_ZERO | M_NOWAIT, 
		0, BUS_SPACE_MAXADDR, 16, 0);
	if (buf == NULL)
		return (ENOMEM);

	error = vtcon_port_enqueue_buf(port, buf, VTCON_BULK_BUFSZ);
	if (error) {
		//free(buf, M_DEVBUF);
		contigfree(buf, VTCON_BULK_BUFSZ, M_DEVBUF);
	}

	return (error);
}

static void
vtcon_port_requeue_buf(struct vtcon_port *port, void *buf)
{
	PDEBUG(("port:%p", port));
	int error;

	error = vtcon_port_enqueue_buf(port, buf, VTCON_BULK_BUFSZ);
	KASSERT(error == 0,
	    ("%s: cannot requeue input buffer %d", __func__, error));
}

static int
vtcon_port_populate(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));
	struct virtqueue *vq;
	int nbufs, error;

	vq = port->vtcport_invq;
	error = ENOSPC;

	for (nbufs = 0; !virtqueue_full(vq); nbufs++) {
		error = vtcon_port_create_buf(port);
		if (error)
			break;
	}

	if (nbufs > 0) {
		//virtqueue_notify(vq);
		virtqueue_notify(vq, NULL);
		error = 0;
	}

	return (error);
}

static void
vtcon_port_destroy(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));

	port->vtcport_sc = NULL;
	port->vtcport_scport = NULL;
	port->vtcport_invq = NULL;
	port->vtcport_outvq = NULL;
	port->vtcport_id = -1;
	VTCON_PORT_DESTROY(port);
	//free(port, M_DEVBUF);
	contigfree(port, sizeof(struct vtcon_port), M_DEVBUF);
}

static int
vtcon_port_init_vqs(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));

	struct vtcon_softc_port *scport;
	int error;

	scport = port->vtcport_scport;

	port->vtcport_invq = scport->vcsp_invq;
	port->vtcport_outvq = scport->vcsp_outvq;

	/*
	 * Free any data left over from when this virtqueue was in use by a
	 * prior port. We have not yet notified the host that the port is
	 * ready, so assume nothing in the virtqueue can be for us.
	 */
	vtcon_port_drain(port);

	KASSERT(virtqueue_empty(port->vtcport_invq),
	    ("%s: in virtqueue is not empty", __func__));
	KASSERT(virtqueue_empty(port->vtcport_outvq),
	    ("%s: out virtqueue is not empty", __func__));

	error = vtcon_port_populate(port);
	if (error)
		return (error);

	return (0);
}

static int
vtcon_port_create(struct vtcon_softc *sc, int id)
{
	device_t dev;
	cdev_t dev1;
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;
	int error;

	dev = sc->vtcon_dev;
	PDEBUG(("softc:%p, dev:%s", sc, device_get_nameunit(dev)));
	scport = &sc->vtcon_ports[id];

	VTCON_ASSERT_VALID_PORTID(sc, id);
	//MPASS(scport->vcsp_port == NULL);
	KASSERT(scport->vcsp_port == NULL, ("%s:%d: vcsp_port:%p", __func__, __LINE__, scport->vcsp_port));

	//port = malloc(sizeof(struct vtcon_port), M_DEVBUF, M_NOWAIT | M_ZERO);
	port = contigmalloc(sizeof(struct vtcon_port), M_DEVBUF, M_NOWAIT | M_ZERO, 
		0, BUS_SPACE_MAXADDR, 16, 0);
	if (port == NULL)
		return (ENOMEM);

	port->vtcport_sc = sc;
	port->vtcport_scport = scport;
	port->vtcport_id = id;
	//mtx_init(VTCON_PORT_MTX(port), "vtcpmtx", NULL, MTX_DEF);
	VTCON_PORT_LOCK_INIT(port, "vtcpmtx:", device_get_nameunit(dev));
	
	// hookup port-lock to vtcon_ops ?
	//port->vtcport_tty = tty_alloc_mutex(&vtcon_tty_class, port,   // ? fbsd
	//    VTCON_PORT_MTX(port));					// ?

	error = vtcon_port_init_vqs(port);
	if (error) {
		VTCON_PORT_LOCK(port);
		vtcon_port_teardown(port);
		return (error);
	}

	VTCON_LOCK(sc);
	VTCON_PORT_LOCK(port);
	scport->vcsp_port = port;
	vtcon_port_enable_intr(port);
	vtcon_port_submit_event(port, VIRTIO_CONSOLE_PORT_READY, 1);
	VTCON_PORT_UNLOCK(port);
	VTCON_UNLOCK(sc);

	// fbsd
	//tty_makedev(port->vtcport_tty, NULL, "%s%r.%r", VTCON_TTY_PREFIX,
	//    device_get_unit(dev), id);				// ?

	// dfly replacement
	struct tty *tp;
	tp = ttymalloc(&port->vtcport_tty);
	lwkt_gettoken(&tp->t_token);
	
	//tp->t_sc = (void *)sc;
	tp->t_sc = (void *)port;	// ? Is this ok ?
					// Or does it have to be sc ?
					// Currently used like this in vtcon_tty_open to retrieve pointer to port
	tp->t_oproc = vtcon_tty_start;
	tp->t_param = vtcon_tty_param;
	tp->t_stop = vtcon_tty_stop;
	dev1 = make_dev(&vtcon_ops, id, UID_ROOT, GID_WHEEL, 0666, "%s%d.%d", VTCON_TTY_PREFIX, device_get_unit(dev), id);
	port->vtcport_cdev = dev1;
	port->vtcport_cdev->si_tty = tp;
	port->vtcport_cdev->si_drv1 = sc;
	//sc->sc_cdev_init->si_tty = tp;
	//sc->sc_cdev_lock->si_tty = tp;
	//port->vtcport_tty->t_dev = dev1;
	//ttyregister(port->vtcport_tty);
	port->vtcport_tty = tp;

	PDEBUG(("softc:%p, dev:%s, port:%p", sc, device_get_nameunit(dev), port));
	lwkt_reltoken(&tp->t_token);
	/* end new */

	return (0);
}

static void
vtcon_port_drain_bufs(struct virtqueue *vq)
{
	void *buf;
	int last;

	last = 0;

	while ((buf = virtqueue_drain(vq, &last)) != NULL) {
		//free(buf, M_DEVBUF);
		contigfree(buf, VTCON_BULK_BUFSZ, M_DEVBUF);
	}
}

static void
vtcon_port_drain(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));

	vtcon_port_drain_bufs(port->vtcport_invq);
}

static void
vtcon_port_teardown(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));
	struct tty *tp;

	tp = port->vtcport_tty;

	port->vtcport_flags |= VTCON_PORT_FLAG_GONE;

	if (tp != NULL) {
		atomic_add_int(&vtcon_pending_free, 1);
		//tty_rel_gone(tp);				// ?
	} else
		vtcon_port_destroy(port);
}

static void
vtcon_port_change_size(struct vtcon_port *port, uint16_t cols, uint16_t rows)
{
	PDEBUG(("port:%p", port));
	struct tty *tp;
	struct winsize sz;

	tp = port->vtcport_tty;

	if (tp == NULL)
		return;

	bzero(&sz, sizeof(struct winsize));
	sz.ws_col = cols;
	sz.ws_row = rows;

	//tty_set_winsize(tp, &sz);				// >
}

static void
vtcon_port_update_console_size(struct vtcon_softc *sc)
{
	PDEBUG(("softc:%p", sc));
	struct vtcon_port *port;
	struct vtcon_softc_port *scport;
	uint16_t cols, rows;

	vtcon_get_console_size(sc, &cols, &rows);

	/*
	 * For now, assume the first (only) port is the console. Note
	 * QEMU does not implement this feature yet.
	 */
	scport = &sc->vtcon_ports[0];

	VTCON_LOCK(sc);
	port = scport->vcsp_port;

	if (port != NULL) {
		VTCON_PORT_LOCK(port);
		VTCON_UNLOCK(sc);
		vtcon_port_change_size(port, cols, rows);
		VTCON_PORT_UNLOCK(port);
	} else
		VTCON_UNLOCK(sc);
}

static void
vtcon_port_enable_intr(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));

	/*
	 * NOTE: The out virtqueue is always polled, so its interrupt
	 * kept disabled.
	 */
	virtqueue_enable_intr(port->vtcport_invq);
}

static void
vtcon_port_disable_intr(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));

	if (port->vtcport_invq != NULL)
		virtqueue_disable_intr(port->vtcport_invq);
	if (port->vtcport_outvq != NULL)
		virtqueue_disable_intr(port->vtcport_outvq);
}

static void
vtcon_port_in(struct vtcon_port *port)
{
	PDEBUG(("port:%p", port));
	struct virtqueue *vq;
	struct tty *tp;
	char *buf;
	uint32_t len;
	int i, deq;

	tp = port->vtcport_tty;
	vq = port->vtcport_invq;

again:
	deq = 0;

	while ((buf = virtqueue_dequeue(vq, &len)) != NULL) {
		for (i = 0; i < len; i++) {
#if defined(KDB)
			if (port->vtcport_flags & VTCON_PORT_FLAG_CONSOLE)
				kdb_alt_break(buf[i],
				    &port->vtcport_alt_break_state);
#endif
			//ttydisc_rint(tp, buf[i], 0);			// ?
			(*linesw[tp->t_line].l_rint)(0, tp);
		}
		vtcon_port_requeue_buf(port, buf);
		deq++;
	}
	//ttydisc_rint_done(tp);					// ?

	if (deq > 0) {
		//virtqueue_notify(vq);
		virtqueue_notify(vq, NULL);
	}

	if (virtqueue_enable_intr(vq) != 0)
		goto again;
}

static void
vtcon_port_intr(void *scportx)
{
	PDEBUG(("scportx:%p", scportx));
	struct vtcon_softc_port *scport;
	struct vtcon_softc *sc;
	struct vtcon_port *port;

	scport = scportx;
	sc = scport->vcsp_sc;

	VTCON_LOCK(sc);
	port = scport->vcsp_port;
	if (port == NULL) {
		VTCON_UNLOCK(sc);
		return;
	}
	VTCON_PORT_LOCK(port);
	VTCON_UNLOCK(sc);
	if ((port->vtcport_flags & VTCON_PORT_FLAG_GONE) == 0)
		vtcon_port_in(port);
	VTCON_PORT_UNLOCK(port);
}

static void
vtcon_port_out(struct vtcon_port *port, void *buf, int bufsize)
{
	PDEBUG(("port:%p", port));
	struct sglist_seg segs[2];
	struct sglist sg;
	struct virtqueue *vq;
	int error;

	vq = port->vtcport_outvq;
	KASSERT(virtqueue_empty(vq),
	    ("%s: port %p out virtqueue not emtpy", __func__, port));

	sglist_init(&sg, 2, segs);
	error = sglist_append(&sg, buf, bufsize);
	KASSERT(error == 0, ("%s: error %d adding buffer to sglist",
	    __func__, error));

	error = virtqueue_enqueue(vq, buf, &sg, sg.sg_nseg, 0);
	if (error == 0) {
		//virtqueue_notify(vq);
		virtqueue_notify(vq, NULL);
		virtqueue_poll(vq, NULL);
	}
}

static void
vtcon_port_submit_event(struct vtcon_port *port, uint16_t event,
    uint16_t value)
{
	PDEBUG(("port:%p", port));
	struct vtcon_softc *sc;

	sc = port->vtcport_sc;

	vtcon_ctrl_send_control(sc, port->vtcport_id, event, value);
}

/* new (from sys/bus/u4b/serial/usb_serial.h)*/
static void	 vtcon_tty_start(struct tty *tp)
{
}
static void 	 vtcon_tty_stop(struct tty *tp, int flag)
{
}
static int	 vtcon_tty_param(struct tty *tp, struct termios *t)
{
	return 0;
}

//static int
//vtcon_tty_open(struct tty *tp)
static  int
vtcon_tty_open(struct dev_open_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	//struct vtcon_softc *sc;
	struct tty *tp;
	struct vtcon_port *port;
	PDEBUG(("tty:%p, dev:%p", ap, dev));
	
	lwkt_gettoken(&tty_token);
	//sc = (struct vtcon_softc *)dev->si_drv1;
	tp = dev->si_tty;
	port = (struct vtcon_port *)tp->t_sc;
	lwkt_reltoken(&tty_token);
	
	//port = tty_softc(tp);						// ?
	lwkt_gettoken(&tp->t_token);
	if (port->vtcport_flags & VTCON_PORT_FLAG_GONE)
		return (ENXIO);

	vtcon_port_submit_event(port, VIRTIO_CONSOLE_PORT_OPEN, 1);
	lwkt_reltoken(&tp->t_token);
	return (0);
}

//static void
//vtcon_tty_close(struct tty *tp)
static int
vtcon_tty_close(struct dev_close_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct vtcon_port *port;
	struct tty *tp;
	PDEBUG(("tty:%p, dev:%p", ap, dev));
	
	lwkt_gettoken(&tty_token);
	//sc = (struct vtcon_softc *)dev->si_drv1;
	tp = dev->si_tty;
	port = (struct vtcon_port *)tp->t_sc;
	lwkt_reltoken(&tty_token);
	
	//struct vtcon_port *port = NULL;
	//port = tty_softc(tp);						// ?

	lwkt_gettoken(&tp->t_token);
	if (port->vtcport_flags & VTCON_PORT_FLAG_GONE)
		return (ENXIO);						// ? not sure about right error no

	vtcon_port_submit_event(port, VIRTIO_CONSOLE_PORT_OPEN, 0);
	lwkt_reltoken(&tp->t_token);
	
	return (0);
}

static void
vtcon_tty_outwakeup(struct tty *tp)
{
	PDEBUG(("tty:%p", tp));
	struct vtcon_port *port;
	char buf[VTCON_BULK_BUFSZ];
	int len;

	port = tp->t_sc;
	//port = tty_softc(tp);						// ?

	if (port->vtcport_flags & VTCON_PORT_FLAG_GONE)
		return;

	//while ((len = ttydisc_getc(tp, buf, sizeof(buf))) != 0)	// ?
	//	vtcon_port_out(port, buf, len);
}

static void
vtcon_tty_free(void *xport)
{
	PDEBUG(("xport:%p", xport));
	struct vtcon_port *port;

	port = xport;

	vtcon_port_destroy(port);
	atomic_subtract_int(&vtcon_pending_free, 1);
}

static void
vtcon_get_console_size(struct vtcon_softc *sc, uint16_t *cols, uint16_t *rows)
{
	PDEBUG(("softc:%p", sc));
	struct virtio_console_config concfg;

	KASSERT(sc->vtcon_flags & VTCON_FLAG_SIZE,
	    ("%s: size feature not negotiated", __func__));

	vtcon_read_config(sc, &concfg);

	*cols = concfg.cols;
	*rows = concfg.rows;
	
	PDEBUG(("columns:%d, rows:%d", *cols, *rows));
	
}

static void
vtcon_enable_interrupts(struct vtcon_softc *sc)
{
	PDEBUG(("softc:%p", sc));
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;
	int i;

	VTCON_LOCK(sc);

	if (sc->vtcon_flags & VTCON_FLAG_MULTIPORT)
		virtqueue_enable_intr(sc->vtcon_ctrl_rxvq);

	for (i = 0; i < sc->vtcon_max_ports; i++) {
		scport = &sc->vtcon_ports[i];

		port = scport->vcsp_port;
		if (port == NULL)
			continue;

		VTCON_PORT_LOCK(port);
		vtcon_port_enable_intr(port);
		VTCON_PORT_UNLOCK(port);
	}

	VTCON_UNLOCK(sc);
}

static void
vtcon_disable_interrupts(struct vtcon_softc *sc)
{
	PDEBUG(("softc:%p", sc));
	struct vtcon_softc_port *scport;
	struct vtcon_port *port;
	int i;

	VTCON_LOCK_ASSERT(sc);

	if (sc->vtcon_flags & VTCON_FLAG_MULTIPORT)
		virtqueue_disable_intr(sc->vtcon_ctrl_rxvq);

	for (i = 0; i < sc->vtcon_max_ports; i++) {
		scport = &sc->vtcon_ports[i];

		port = scport->vcsp_port;
		if (port == NULL)
			continue;

		VTCON_PORT_LOCK(port);
		vtcon_port_disable_intr(port);
		VTCON_PORT_UNLOCK(port);
	}
}
