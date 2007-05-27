/*-
 * Copyright (c) 1999 Michael Smith
 * All rights reserved.
 * Copyright (c) 1999 Poul-Henning Kamp
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
 *	$FreeBSD: src/sys/kern/vfs_conf.c,v 1.49.2.5 2003/01/07 11:56:53 joerg Exp $
 *	$DragonFly: src/sys/kern/vfs_conf.c,v 1.31 2007/05/27 18:37:23 dillon Exp $
 */

/*
 * Locate and mount the root filesystem.
 *
 * The root filesystem is detailed in the kernel environment variable
 * vfs.root.mountfrom, which is expected to be in the general format
 *
 * <vfsname>:[<path>]
 * vfsname   := the name of a VFS known to the kernel and capable
 *              of being mounted as root
 * path      := disk device name or other data used by the filesystem
 *              to locate its physical store
 *
 */

#include "opt_rootdevname.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/vnode.h>
#include <sys/mount.h>
#include <sys/malloc.h>
#include <sys/reboot.h>
#include <sys/diskslice.h>
#include <sys/disklabel.h>
#include <sys/conf.h>
#include <sys/cons.h>
#include <sys/device.h>
#include <sys/namecache.h>
#include <sys/paths.h>
#include <sys/thread2.h>

#include "opt_ddb.h"
#ifdef DDB
#include <ddb/ddb.h>
#endif

MALLOC_DEFINE(M_MOUNT, "mount", "vfs mount structure");

#define ROOTNAME	"root_device"

struct vnode	*rootvnode;
struct nchandle rootnch;

/* 
 * The root specifiers we will try if RB_CDROM is specified.  Note that
 * the ATA driver will accept acd*a and acd*c, but the SCSI driver
 * will only accept cd*c, so use 'c'.
 *
 * XXX TGEN NATA and, presumably, 'old'ATA will also accept the device name
 * without any fake partition, since the major & minor are identical for all
 * three (acd*, acd*a and acd*c). However, due to an as-of-yet undiscovered
 * bug, acd0c ends up with minor 2 when using NATA and booting cold. Since
 * NATA's acd_open() is unable to fulfill mounts on such 'ghost' cdevs, acd0
 * and acd1 have been added to the list of CD-ROM root device names.
 */
static char *cdrom_rootdevnames[] = {
	"cd9660:cd0c",
	"cd9660:acd0c",
	"cd9660:cd1c",
	"cd9660:acd1c",
	"cd9660:acd0",
	"cd9660:acd1",
	NULL
};

static void	vfs_mountroot(void *junk);
static int	vfs_mountroot_try(const char *mountfrom);
static int	vfs_mountroot_ask(void);
static int	getline(char *cp, int limit);

/* legacy find-root code */
char		*rootdevnames[2] = {NULL, NULL};
static int	setrootbyname(char *name);

SYSINIT(mountroot, SI_SUB_MOUNT_ROOT, SI_ORDER_SECOND, vfs_mountroot, NULL);
	
/*
 * Find and mount the root filesystem
 */
static void
vfs_mountroot(void *junk)
{
	int	i;
	cdev_t	save_rootdev = rootdev;
	
	/* 
	 * The root filesystem information is compiled in, and we are
	 * booted with instructions to use it.
	 */
#ifdef ROOTDEVNAME
	if ((boothowto & RB_DFLTROOT) && 
	    !vfs_mountroot_try(ROOTDEVNAME))
		return;
#endif
	/* 
	 * We are booted with instructions to prompt for the root filesystem,
	 * or to use the compiled-in default when it doesn't exist.
	 */
	if (boothowto & (RB_DFLTROOT | RB_ASKNAME)) {
		if (!vfs_mountroot_ask())
			return;
	}

	/*
	 * We've been given the generic "use CDROM as root" flag.  This is
	 * necessary because one media may be used in many different
	 * devices, so we need to search for them.
	 */
	if (boothowto & RB_CDROM) {
		for (i = 0; cdrom_rootdevnames[i] != NULL; i++) {
			if (!vfs_mountroot_try(cdrom_rootdevnames[i]))
				return;
		}
	}

	/*
	 * Try to use the value read by the loader from /etc/fstab, or
	 * supplied via some other means.  This is the preferred 
	 * mechanism.
	 */
	if (!vfs_mountroot_try(kgetenv("vfs.root.mountfrom")))
		return;

	/*
	 * If a vfs set rootdev, try it (XXX VINUM HACK!)
	 */
	if (save_rootdev != NULL) {
		rootdev = save_rootdev;
		if (!vfs_mountroot_try(""))
			return;
	}

	/* 
	 * Try values that may have been computed by the machine-dependant
	 * legacy code.
	 */
	if (rootdevnames[0] && !vfs_mountroot_try(rootdevnames[0]))
		return;
	if (rootdevnames[1] && !vfs_mountroot_try(rootdevnames[1]))
		return;

	/*
	 * If we have a compiled-in default, and haven't already tried it, try
	 * it now.
	 */
#ifdef ROOTDEVNAME
	if (!(boothowto & RB_DFLTROOT))
		if (!vfs_mountroot_try(ROOTDEVNAME))
			return;
#endif

	/* 
	 * Everything so far has failed, prompt on the console if we haven't
	 * already tried that.
	 */
	if (!(boothowto & (RB_DFLTROOT | RB_ASKNAME)) && !vfs_mountroot_ask())
		return;
	panic("Root mount failed, startup aborted.");
}

/*
 * Mount (mountfrom) as the root filesystem.
 */
static int
vfs_mountroot_try(const char *mountfrom)
{
        struct mount	*mp;
	char		*vfsname, *devname;
	int		error;
	char		patt[32];

	vfsname = NULL;
	devname = NULL;
	mp      = NULL;
	error   = EINVAL;

	if (mountfrom == NULL)
		return(error);		/* don't complain */

	crit_enter();
	kprintf("Mounting root from %s\n", mountfrom);
	crit_exit();

	/* parse vfs name and devname */
	vfsname = kmalloc(MFSNAMELEN, M_MOUNT, M_WAITOK);
	devname = kmalloc(MNAMELEN, M_MOUNT, M_WAITOK);
	vfsname[0] = devname[0] = 0;
	ksprintf(patt, "%%%d[a-z0-9]:%%%ds", MFSNAMELEN, MNAMELEN);
	if (ksscanf(mountfrom, patt, vfsname, devname) < 1)
		goto done;

	/* allocate a root mount */
	error = vfs_rootmountalloc(vfsname, 
			devname[0] != 0 ? devname : ROOTNAME, &mp);
	if (error != 0) {
		kprintf("Can't allocate root mount for filesystem '%s': %d\n",
		       vfsname, error);
		goto done;
	}
	mp->mnt_flag |= MNT_ROOTFS;

	/* do our best to set rootdev */
	if ((devname[0] != 0) && setrootbyname(devname))
		kprintf("setrootbyname failed\n");

	/* If the root device is a type "memory disk", mount RW */
	if (rootdev != NULL && dev_is_good(rootdev) &&
	    (dev_dflags(rootdev) & D_MEMDISK)) {
		mp->mnt_flag &= ~MNT_RDONLY;
	}

	error = VFS_MOUNT(mp, NULL, NULL, proc0.p_ucred);

done:
	if (vfsname != NULL)
		kfree(vfsname, M_MOUNT);
	if (devname != NULL)
		kfree(devname, M_MOUNT);
	if (error != 0) {
		if (mp != NULL) {
			vfs_unbusy(mp);
			kfree(mp, M_MOUNT);
		}
		kprintf("Root mount failed: %d\n", error);
	} else {
		/* register with list of mounted filesystems */
		mountlist_insert(mp, MNTINS_FIRST);

		/* sanity check system clock against root fs timestamp */
		inittodr(mp->mnt_time);
		vfs_unbusy(mp);
	}
	return(error);
}

/*
 * Spin prompting on the console for a suitable root filesystem
 */
static int vfs_mountroot_ask_callback(struct dev_ops *ops, void *arg);

static int
vfs_mountroot_ask(void)
{
	char name[128];
	int llimit = 100;

	kprintf("\nManual root filesystem specification:\n");
	kprintf("  <fstype>:<device>  Specify root (e.g. ufs:da0s1a)\n");
	kprintf("  ?                  List valid disk boot devices\n");
	kprintf("  panic              Just panic\n");
	kprintf("  abort              Abort manual input\n");
	while (llimit--) {
		kprintf("\nmountroot> ");

		if (getline(name, 128) < 0)
			break;
		if (name[0] == 0) {
			;
		} else if (name[0] == '?') {
			kprintf("Possibly valid devices for 'ufs' root:\n");
			dev_ops_scan(vfs_mountroot_ask_callback, NULL);
			kprintf("\n");
			continue;
		} else if (strcmp(name, "panic") == 0) {
			panic("panic from console");
		} else if (strcmp(name, "abort") == 0) {
			break;
		} else if (vfs_mountroot_try(name) == 0) {
			return(0);
		}
	}
	return(1);
}

static
int
vfs_mountroot_ask_callback(struct dev_ops *ops, void *arg __unused)
{
	cdev_t dev;

	dev = get_dev(ops->head.maj, 0);
	if (dev_is_good(dev))
		kprintf(" \"%s\"", dev_dname(dev));
	return(0);
}

static int
getline(char *cp, int limit)
{
	char *lp;
	int c;

	lp = cp;
	for (;;) {
		c = cngetc();

		switch (c) {
		case -1:
			return(-1);
		case '\n':
		case '\r':
			kprintf("\n");
			*lp++ = '\0';
			return(0);
		case '\b':
		case '\177':
			if (lp > cp) {
				kprintf("\b \b");
				lp--;
			} else {
				kprintf("%c", 7);
			}
			continue;
		case '#':
			kprintf("#");
			lp--;
			if (lp < cp)
				lp = cp;
			continue;
		case '@':
		case 'u' & 037:
			lp = cp;
			kprintf("%c", '\n');
			continue;
		default:
			if (lp - cp >= limit - 1) {
				kprintf("%c", 7);
			} else {
				kprintf("%c", c);
				*lp++ = c;
			}
			continue;
		}
	}
}

/*
 * Convert a given name to the cdev_t of the disk-like device
 * it refers to.
 */
struct kdbn_info {
	const char *name;
	int nlen;
	int minor;
	cdev_t dev;
};

static int kgetdiskbyname_callback(struct dev_ops *ops, void *arg);

cdev_t
kgetdiskbyname(const char *name) 
{
	char *cp;
	int nlen;
	int unit, slice, part;
	cdev_t rdev;
	struct kdbn_info info;

	/*
	 * Get the base name of the device
	 */
	if (strncmp(name, __SYS_PATH_DEV, sizeof(__SYS_PATH_DEV) - 1) == 0)
		name += sizeof(__SYS_PATH_DEV) - 1;
	cp = __DECONST(char *, name);
	while (*cp == '/')
		++cp;
	while (*cp >= 'a' && *cp <= 'z')
		++cp;
	if (cp == name) {
		kprintf("missing device name\n");
		return (NULL);
	}
	nlen = cp - name;

	/*
	 * Get the unit.
	 */
	unit = strtol(cp, &cp, 10);
	if (name + nlen == (const char *)cp || unit < 0 || unit >= DKMAXUNITS) {
		kprintf("bad unit: %d\n", unit);
		return (NULL);
	}

	/*
	 * Get the slice.  Note that if no partition or partition 'a' is
	 * specified, and no slice is specified, we will try both 'ad0a'
	 * (which is what you get when slice is 0), and also 'ad0' (the
	 * whole-disk partition, slice == 1).
	 */
	if (*cp == 's') {
		slice = cp[1] - '0';
		if (slice >= 1)
			++slice;
		cp += 2;
	} else {
		slice = 0;
	}

	/*
	 * Get the partition.
	 */
	if (*cp >= 'a' && *cp <= 'p') {
		part = *cp - 'a';
		++cp;
	} else {
		part = 0;
	}

	if (*cp != '\0') {
		kprintf("junk after name\n");
		return (NULL);
	}

	/*
	 * Locate the device
	 */
	bzero(&info, sizeof(info));
	info.nlen = nlen;
	info.name = name;
	info.minor = dkmakeminor(unit, slice, part);
	dev_ops_scan(kgetdiskbyname_callback, &info);
	if (info.dev == NULL) {
		kprintf("no such device '%*.*s'\n", nlen, nlen, name);
		return (NULL);
	}

	/*
	 * FOUND DEVICE
	 */
	rdev = make_sub_dev(info.dev, info.minor);
	return(rdev);
}

static
int
kgetdiskbyname_callback(struct dev_ops *ops, void *arg)
{
	struct kdbn_info *info = arg;
	cdev_t dev;
	const char *dname;

	dev = get_dev(ops->head.maj, info->minor);
	if (dev_is_good(dev) && (dname = dev_dname(dev)) != NULL) {
		if (strlen(dname) == info->nlen &&
		    strncmp(dname, info->name, info->nlen) == 0) {
			info->dev = dev;
			return(-1);
		}
	}
	return(0);
}

/*
 * Set rootdev to match (name), given that we expect it to
 * refer to a disk-like device.
 */
static int
setrootbyname(char *name)
{
	cdev_t diskdev;

	diskdev = kgetdiskbyname(name);
	if (diskdev != NULL) {
		rootdev = diskdev;
		return (0);
	}

	return (1);
}

#ifdef DDB
DB_SHOW_COMMAND(disk, db_getdiskbyname)
{
	cdev_t dev;

	if (modif[0] == '\0') {
		db_error("usage: show disk/devicename");
		return;
	}
	dev = kgetdiskbyname(modif);
	if (dev != NULL)
		db_printf("cdev_t = %p\n", dev);
	else
		db_printf("No disk device matched.\n");
}
#endif
