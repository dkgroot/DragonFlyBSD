/*
 * Copyright (c) 2003,2004 The DragonFly Project.  All rights reserved.
 * 
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 * 
 * Copyright (c) 1989, 1993, 1995
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Poul-Henning Kamp of the FreeBSD Project.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)vfs_cache.c	8.5 (Berkeley) 3/22/95
 * $FreeBSD: src/sys/kern/vfs_cache.c,v 1.42.2.6 2001/10/05 20:07:03 dillon Exp $
 * $DragonFly: src/sys/kern/vfs_cache.c,v 1.29 2004/09/30 18:59:48 dillon Exp $
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <sys/mount.h>
#include <sys/vnode.h>
#include <sys/malloc.h>
#include <sys/sysproto.h>
#include <sys/proc.h>
#include <sys/namei.h>
#include <sys/nlookup.h>
#include <sys/filedesc.h>
#include <sys/fnv_hash.h>
#include <sys/globaldata.h>
#include <sys/kern_syscall.h>

/*
 * Random lookups in the cache are accomplished with a hash table using
 * a hash key of (nc_src_vp, name).
 *
 * Negative entries may exist and correspond to structures where nc_vp
 * is NULL.  In a negative entry, NCF_WHITEOUT will be set if the entry
 * corresponds to a whited-out directory entry (verses simply not finding the
 * entry at all).
 *
 * Upon reaching the last segment of a path, if the reference is for DELETE,
 * or NOCACHE is set (rewrite), and the name is located in the cache, it
 * will be dropped.
 */

/*
 * Structures associated with name cacheing.
 */
#define NCHHASH(hash)	(&nchashtbl[(hash) & nchash])
#define MINNEG		1024

MALLOC_DEFINE(M_VFSCACHE, "vfscache", "VFS name cache entries");

static LIST_HEAD(nchashhead, namecache) *nchashtbl;	/* Hash Table */
static struct namecache_list	ncneglist;		/* instead of vnode */

static u_long	nchash;			/* size of hash table */
SYSCTL_ULONG(_debug, OID_AUTO, nchash, CTLFLAG_RD, &nchash, 0, "");

static u_long	ncnegfactor = 16;	/* ratio of negative entries */
SYSCTL_ULONG(_debug, OID_AUTO, ncnegfactor, CTLFLAG_RW, &ncnegfactor, 0, "");

static u_long	numneg;		/* number of cache entries allocated */
SYSCTL_ULONG(_debug, OID_AUTO, numneg, CTLFLAG_RD, &numneg, 0, "");

static u_long	numcache;		/* number of cache entries allocated */
SYSCTL_ULONG(_debug, OID_AUTO, numcache, CTLFLAG_RD, &numcache, 0, "");

static u_long	numunres;		/* number of unresolved entries */
SYSCTL_ULONG(_debug, OID_AUTO, numunres, CTLFLAG_RD, &numunres, 0, "");

SYSCTL_INT(_debug, OID_AUTO, vnsize, CTLFLAG_RD, 0, sizeof(struct vnode), "");
SYSCTL_INT(_debug, OID_AUTO, ncsize, CTLFLAG_RD, 0, sizeof(struct namecache), "");

/*
 * The new name cache statistics
 */
SYSCTL_NODE(_vfs, OID_AUTO, cache, CTLFLAG_RW, 0, "Name cache statistics");
#define STATNODE(mode, name, var) \
	SYSCTL_ULONG(_vfs_cache, OID_AUTO, name, mode, var, 0, "");
STATNODE(CTLFLAG_RD, numneg, &numneg);
STATNODE(CTLFLAG_RD, numcache, &numcache);
static u_long numcalls; STATNODE(CTLFLAG_RD, numcalls, &numcalls);
static u_long dothits; STATNODE(CTLFLAG_RD, dothits, &dothits);
static u_long dotdothits; STATNODE(CTLFLAG_RD, dotdothits, &dotdothits);
static u_long numchecks; STATNODE(CTLFLAG_RD, numchecks, &numchecks);
static u_long nummiss; STATNODE(CTLFLAG_RD, nummiss, &nummiss);
static u_long nummisszap; STATNODE(CTLFLAG_RD, nummisszap, &nummisszap);
static u_long numposzaps; STATNODE(CTLFLAG_RD, numposzaps, &numposzaps);
static u_long numposhits; STATNODE(CTLFLAG_RD, numposhits, &numposhits);
static u_long numnegzaps; STATNODE(CTLFLAG_RD, numnegzaps, &numnegzaps);
static u_long numneghits; STATNODE(CTLFLAG_RD, numneghits, &numneghits);

struct nchstats nchstats[SMP_MAXCPU];
/*
 * Export VFS cache effectiveness statistics to user-land.
 *
 * The statistics are left for aggregation to user-land so
 * neat things can be achieved, like observing per-CPU cache
 * distribution.
 */
static int
sysctl_nchstats(SYSCTL_HANDLER_ARGS)
{
	struct globaldata *gd;
	int i, error;

	error = 0;
	for (i = 0; i < ncpus; ++i) {
		gd = globaldata_find(i);
		if ((error = SYSCTL_OUT(req, (void *)&(*gd->gd_nchstats),
			sizeof(struct nchstats))))
			break;
	}

	return (error);
}
SYSCTL_PROC(_vfs_cache, OID_AUTO, nchstats, CTLTYPE_OPAQUE|CTLFLAG_RD,
  0, 0, sysctl_nchstats, "S,nchstats", "VFS cache effectiveness statistics");

static void cache_zap(struct namecache *ncp);

/*
 * cache_hold() and cache_drop() prevent the premature deletion of a
 * namecache entry but do not prevent operations (such as zapping) on
 * that namecache entry.
 */
static __inline
struct namecache *
_cache_hold(struct namecache *ncp)
{
	++ncp->nc_refs;
	return(ncp);
}

static __inline
void
_cache_drop(struct namecache *ncp)
{
	KKASSERT(ncp->nc_refs > 0);
	if (ncp->nc_refs == 1 && 
	    (ncp->nc_flag & NCF_UNRESOLVED) && 
	    TAILQ_EMPTY(&ncp->nc_list)
	) {
		cache_zap(ncp);
	} else {
		--ncp->nc_refs;
	}
}

/*
 * Link a new namecache entry to its parent.  Be careful to avoid races
 * if vhold() blocks in the future.
 */
static void
cache_link_parent(struct namecache *ncp, struct namecache *par)
{
	KKASSERT(ncp->nc_parent == NULL);
	ncp->nc_parent = par;
	if (TAILQ_EMPTY(&par->nc_list)) {
		TAILQ_INSERT_HEAD(&par->nc_list, ncp, nc_entry);
		/*
		 * Any vp associated with an ncp which has children must
		 * be held.
		 */
		if (par->nc_vp)
			vhold(par->nc_vp);
	} else {
		TAILQ_INSERT_HEAD(&par->nc_list, ncp, nc_entry);
	}
}

/*
 * Remove the parent association from a namecache structure.
 */
static void
cache_unlink_parent(struct namecache *ncp)
{
	struct namecache *par;

	if ((par = ncp->nc_parent) != NULL) {
		ncp->nc_parent = NULL;
		par = cache_hold(par);
		TAILQ_REMOVE(&par->nc_list, ncp, nc_entry);
		if (par->nc_vp && TAILQ_EMPTY(&par->nc_list))
			vdrop(par->nc_vp);
		cache_drop(par);
	}
}

/*
 * Allocate a new namecache structure.
 */
static struct namecache *
cache_alloc(void)
{
	struct namecache *ncp;

	ncp = malloc(sizeof(*ncp), M_VFSCACHE, M_WAITOK|M_ZERO);
	ncp->nc_flag = NCF_UNRESOLVED;
	ncp->nc_error = ENOTCONN;	/* needs to be resolved */
	TAILQ_INIT(&ncp->nc_list);
	return(ncp);
}


/*
 * Ref and deref a namecache structure.
 */
struct namecache *
cache_hold(struct namecache *ncp)
{
	return(_cache_hold(ncp));
}

void
cache_drop(struct namecache *ncp)
{
	_cache_drop(ncp);
}

/*
 * Namespace locking.  The caller must already hold a reference to the
 * namecache structure in order to lock/unlock it.  This function prevents
 * the namespace from being created or destroyed by accessors other then
 * the lock holder.
 *
 * Note that holding a locked namecache structure does not prevent the
 * underlying vnode from being destroyed and the namecache state moving
 * to an unresolved state.  XXX MP
 */
void
cache_lock(struct namecache *ncp)
{
	thread_t td = curthread;
	int didwarn = 0;

	KKASSERT(ncp->nc_refs != 0);
	for (;;) {
		if (ncp->nc_exlocks == 0) {
			ncp->nc_exlocks = 1;
			ncp->nc_locktd = td;
			break;
		}
		if (ncp->nc_locktd == td) {
			++ncp->nc_exlocks;
			break;
		}
		ncp->nc_flag |= NCF_LOCKREQ;
		if (tsleep(ncp, 0, "clock", hz) == EWOULDBLOCK) {
			if (didwarn == 0) {
				didwarn = 1;
				printf("[diagnostic] cache_lock: blocked on %*.*s\n",
					ncp->nc_nlen, ncp->nc_nlen,
					ncp->nc_name);
			}
		}
	}
	if (didwarn == 1) {
		printf("[diagnostic] cache_lock: unblocked %*.*s\n",
			ncp->nc_nlen, ncp->nc_nlen, ncp->nc_name);
	}
}

void
cache_unlock(struct namecache *ncp)
{
	thread_t td = curthread;

	KKASSERT(ncp->nc_refs > 0);
	KKASSERT(ncp->nc_exlocks > 0);
	KKASSERT(ncp->nc_locktd == td);
	if (--ncp->nc_exlocks == 0) {
		ncp->nc_locktd = NULL;
		if (ncp->nc_flag & NCF_LOCKREQ) {
			ncp->nc_flag &= ~NCF_LOCKREQ;
			wakeup_one(ncp);
		}
	}
}

/*
 * ref-and-lock, unlock-and-deref functions.
 */
struct namecache *
cache_get(struct namecache *ncp)
{
	_cache_hold(ncp);
	cache_lock(ncp);
	return(ncp);
}

void
cache_put(struct namecache *ncp)
{
	cache_unlock(ncp);
	_cache_drop(ncp);
}

/*
 * Resolve an unresolved ncp by associating a vnode with it.  If the
 * vnode is NULL, a negative cache entry is created.
 *
 * The ncp should be locked on entry and will remain locked on return.
 */
void
cache_setvp(struct namecache *ncp, struct vnode *vp)
{
	KKASSERT(ncp->nc_flag & NCF_UNRESOLVED);
	ncp->nc_vp = vp;
	if (vp != NULL) {
		/*
		 * Any vp associated with an ncp which has children must
		 * be held.
		 */
		if (!TAILQ_EMPTY(&ncp->nc_list))
			vhold(vp);
		TAILQ_INSERT_HEAD(&vp->v_namecache, ncp, nc_vnode);

		/*
		 * Set auxillary flags
		 */
		switch(vp->v_type) {
		case VDIR:
			ncp->nc_flag |= NCF_ISDIR;
			break;
		case VLNK:
			ncp->nc_flag |= NCF_ISSYMLINK;
			/* XXX cache the contents of the symlink */
			break;
		default:
			break;
		}
		++numcache;
		ncp->nc_error = 0;
	} else {
		TAILQ_INSERT_TAIL(&ncneglist, ncp, nc_vnode);
		++numneg;
		ncp->nc_error = ENOENT;
	}
	ncp->nc_flag &= ~NCF_UNRESOLVED;
}

/*
 * Disassociate the vnode or negative-cache association and mark a
 * namecache entry as unresolved again.  Note that the ncp is still
 * left in the hash table and still linked to its parent.
 *
 * The ncp should be locked on entry and will remain locked on return.
 */
void
cache_setunresolved(struct namecache *ncp)
{
	struct vnode *vp;

	if ((ncp->nc_flag & NCF_UNRESOLVED) == 0) {
		ncp->nc_flag |= NCF_UNRESOLVED;
		ncp->nc_flag &= ~(NCF_WHITEOUT|NCF_ISDIR|NCF_ISSYMLINK);
		ncp->nc_error = ENOTCONN;
		++numunres;
		if ((vp = ncp->nc_vp) != NULL) {
			--numcache;
			ncp->nc_vp = NULL;	/* safety */
			TAILQ_REMOVE(&vp->v_namecache, ncp, nc_vnode);
			if (!TAILQ_EMPTY(&ncp->nc_list))
				vdrop(vp);
		} else {
			TAILQ_REMOVE(&ncneglist, ncp, nc_vnode);
			--numneg;
		}
	}
}

/*
 * vget the vnode associated with the namecache entry.  Resolve the namecache
 * entry if necessary and deal with namecache/vp races.  The passed ncp must
 * be referenced and may be locked.  The ncp's ref/locking state is not 
 * effected by this call.
 *
 * lk_type may be LK_SHARED, LK_EXCLUSIVE.  A ref'd, possibly locked
 * (depending on the passed lk_type) will be returned in *vpp with an error
 * of 0, or NULL will be returned in *vpp with a non-0 error code.  The
 * most typical error is ENOENT, meaning that the ncp represents a negative
 * cache hit and there is no vnode to retrieve, but other errors can occur
 * too.
 *
 * The main race we have to deal with are namecache zaps.  The ncp itself
 * will not disappear since it is referenced, and it turns out that the
 * validity of the vp pointer can be checked simply by rechecking the
 * contents of ncp->nc_vp.
 */
int
cache_vget(struct namecache *ncp, struct ucred *cred,
	   int lk_type, struct vnode **vpp)
{
	struct vnode *vp;
	int error;

again:
	vp = NULL;
	if (ncp->nc_flag & NCF_UNRESOLVED) {
		cache_lock(ncp);
		error = cache_resolve(ncp, cred);
		cache_unlock(ncp);
	} else {
		error = 0;
	}
	if (error == 0 && (vp = ncp->nc_vp) != NULL) {
		error = vget(vp, NULL, lk_type, curthread);
		if (error) {
			if (vp != ncp->nc_vp)	/* handle cache_zap race */
				goto again;
			vp = NULL;
		} else if (vp != ncp->nc_vp) {	/* handle cache_zap race */
			vput(vp);
			goto again;
		}
	}
	if (error == 0 && vp == NULL)
		error = ENOENT;
	*vpp = vp;
	return(error);
}

int
cache_vref(struct namecache *ncp, struct ucred *cred, struct vnode **vpp)
{
	struct vnode *vp;
	int error;

again:
	vp = NULL;
	if (ncp->nc_flag & NCF_UNRESOLVED) {
		cache_lock(ncp);
		error = cache_resolve(ncp, cred);
		cache_unlock(ncp);
	} else {
		error = 0;
	}
	if (error == 0 && (vp = ncp->nc_vp) != NULL) {
		vref(vp);
		if (vp != ncp->nc_vp) {		/* handle cache_zap race */
			vrele(vp);
			goto again;
		}
	}
	if (error == 0 && vp == NULL)
		error = ENOENT;
	*vpp = vp;
	return(error);
}

/*
 * Try to destroy a namecache entry.  The entry is disassociated from its
 * vnode or ncneglist and reverted to an UNRESOLVED state.
 *
 * Then, if there are no additional references to the ncp and we can
 * successfully delete the children, the entry is also removed from the
 * namecache hashlist / topology.
 *
 * References or undeletable children will prevent the entry from being
 * removed from the topology.  The entry may be revalidated (typically
 * by cache_enter()) at a later time.  Children remain because:
 *
 *	+ we have tried to delete a node rather then a leaf in the topology.
 *	+ the presence of negative entries (we try to scrap these).
 *	+ an entry or child has a non-zero ref count and cannot be scrapped.
 *
 * This function must be called with the ncp held and will drop the ref
 * count during zapping.
 */
static void
cache_zap(struct namecache *ncp)
{
	struct namecache *par;

	/*
	 * Disassociate the vnode or negative cache ref and set NCF_UNRESOLVED.
	 */
	cache_setunresolved(ncp);

	/*
	 * Try to scrap the entry and possibly tail-recurse on its parent.
	 * We only scrap unref'd (other then our ref) unresolved entries,
	 * we do not scrap 'live' entries.
	 */
	while (ncp->nc_flag & NCF_UNRESOLVED) {
		/*
		 * Someone other then us has a ref, stop.
		 */
		if (ncp->nc_refs > 1)
			goto done;

		/*
		 * We have children, stop.
		 */
		if (!TAILQ_EMPTY(&ncp->nc_list))
			goto done;

		if (ncp->nc_flag & NCF_HASHED) {
			ncp->nc_flag &= ~NCF_HASHED;
			LIST_REMOVE(ncp, nc_hash);
		}

		/*
		 * Unlink from its parent and free, then loop on the
		 * parent.  XXX temp hack, in stage-3 parent is never NULL
		 */
		if ((par = ncp->nc_parent) != NULL) {
			par = cache_hold(par);
			TAILQ_REMOVE(&par->nc_list, ncp, nc_entry);
			if (par->nc_vp && TAILQ_EMPTY(&par->nc_list))
				vdrop(par->nc_vp);
		}
		--numunres;
		ncp->nc_refs = -1;	/* safety */
		ncp->nc_parent = NULL;	/* safety */
		if (ncp->nc_name)
			free(ncp->nc_name, M_VFSCACHE);
		free(ncp, M_VFSCACHE);
		ncp = par;
		if (par == NULL)	/* temp hack */
			return;		/* temp hack */
	}
done:
	--ncp->nc_refs;
}

/*
 * NEW NAMECACHE LOOKUP API
 *
 * Lookup an entry in the cache.  A locked, referenced, non-NULL 
 * entry is *always* returned, even if the supplied component is illegal.
 * The returned namecache entry should be returned to the system with
 * cache_put() or cache_unlock() + cache_drop().
 *
 * namecache locks are recursive but care must be taken to avoid lock order
 * reversals.
 *
 * Nobody else will be able to manipulate the associated namespace (e.g.
 * create, delete, rename, rename-target) until the caller unlocks the
 * entry.
 *
 * The returned entry will be in one of three states:  positive hit (non-null
 * vnode), negative hit (null vnode), or unresolved (NCF_UNRESOLVED is set).
 * Unresolved entries must be resolved through the filesystem to associate the
 * vnode and/or determine whether a positive or negative hit has occured.
 *
 * It is not necessary to lock a directory in order to lock namespace under
 * that directory.  In fact, it is explicitly not allowed to do that.  A
 * directory is typically only locked when being created, renamed, or
 * destroyed.
 *
 * The directory (par) may be unresolved, in which case any returned child
 * will likely also be marked unresolved.  Likely but not guarenteed.  Since
 * the filesystem VOP_NEWLOOKUP() requires a resolved directory vnode the
 * caller is responsible for resolving the namecache chain top-down.  This API 
 * specifically allows whole chains to be created in an unresolved state.
 */
struct namecache *
cache_nlookup(struct namecache *par, struct nlcomponent *nlc)
{
	struct namecache *ncp;
	struct namecache *new_ncp;
	struct nchashhead *nchpp;
	u_int32_t hash;
	globaldata_t gd;

	numcalls++;
	gd = mycpu;

	/*
	 * Try to locate an existing entry
	 */
	hash = fnv_32_buf(nlc->nlc_nameptr, nlc->nlc_namelen, FNV1_32_INIT);
	hash = fnv_32_buf(&par, sizeof(par), hash);
	new_ncp = NULL;
restart:
	LIST_FOREACH(ncp, (NCHHASH(hash)), nc_hash) {
		numchecks++;

		/*
		 * Zap entries that have timed out.
		 */
		if (ncp->nc_timeout && 
		    (int)(ncp->nc_timeout - ticks) < 0
		) {
			cache_zap(cache_hold(ncp));
			goto restart;
		}

		/*
		 * Break out if we find a matching entry.  Note that
		 * UNRESOLVED entries may match.
		 */
		if (ncp->nc_parent == par &&
		    ncp->nc_nlen == nlc->nlc_namelen &&
		    bcmp(ncp->nc_name, nlc->nlc_nameptr, ncp->nc_nlen) == 0
		) {
			if (new_ncp) {
				free(new_ncp->nc_name, M_VFSCACHE);
				free(new_ncp, M_VFSCACHE);
			}
			goto found;
		}
	}

	/*
	 * We failed to locate an entry, create a new entry and add it to
	 * the cache.  We have to relookup after possibly blocking in
	 * malloc.
	 */
	if (new_ncp == NULL) {
		new_ncp = cache_alloc();
		new_ncp->nc_name = malloc(nlc->nlc_namelen, 
					    M_VFSCACHE, M_WAITOK);
		goto restart;
	}

	ncp = new_ncp;

	/*
	 * Initialize as a new UNRESOLVED entry, lock (non-blocking),
	 * and link to the parent.
	 */
	ncp->nc_nlen = nlc->nlc_namelen;
	bcopy(nlc->nlc_nameptr, ncp->nc_name, nlc->nlc_namelen);
	nchpp = NCHHASH(hash);
	LIST_INSERT_HEAD(nchpp, ncp, nc_hash);
	ncp->nc_flag |= NCF_HASHED;
	cache_get(ncp);
	cache_link_parent(ncp, par);
	return(ncp);

	/*
	 * Entry found.  Cleanup any dangling new_ncp, ref and lock
	 * the ncp.
	 */
found:
	cache_get(ncp);
	return(ncp);
}

/*
 * Resolve an unresolved namecache entry, generally by looking it up.
 * The passed ncp must be locked. 
 *
 * Theoretically since a vnode cannot be recycled while held, and since
 * the nc_parent chain holds its vnode as long as children exist, the
 * direct parent of the cache entry we are trying to resolve should
 * have a valid vnode.  If not then generate an error that we can 
 * determine is related to a resolver bug.
 */
int
cache_resolve(struct namecache *ncp, struct ucred *cred)
{
	struct namecache *par;

	if ((par = ncp->nc_parent) == NULL) {
		ncp->nc_error = EXDEV;
	} else if (par->nc_vp == NULL) {
		ncp->nc_error = EXDEV;
	} else {
		ncp->nc_error = vop_resolve(par->nc_vp->v_ops, ncp, cred);
	}
	return(ncp->nc_error);
}

/*
 * Lookup an entry in the cache.
 *
 * XXX OLD API ROUTINE!  WHEN ALL VFSs HAVE BEEN CLEANED UP THIS PROCEDURE
 * WILL BE REMOVED.
 *
 * Lookup is called with dvp pointing to the directory to search,
 * cnp pointing to the name of the entry being sought. 
 *
 * If the lookup succeeds, the vnode is returned in *vpp, and a
 * status of -1 is returned.
 *
 * If the lookup determines that the name does not exist (negative cacheing),
 * a status of ENOENT is returned. 
 *
 * If the lookup fails, a status of zero is returned.
 *
 * Matching UNRESOLVED entries are resolved.
 *
 * HACKS: we create dummy nodes for parents
 */
int
cache_lookup(struct vnode *dvp, struct vnode **vpp, struct componentname *cnp)
{
	struct namecache *ncp;
	struct namecache *par;
	struct namecache *bpar;
	u_int32_t hash;
	globaldata_t gd = mycpu;

	numcalls++;

	/*
	 * Obtain the namecache entry associated with dvp, creating one if
	 * necessary.  If we have to create one we have insufficient 
	 * information to hash it or even supply the name, but we still
	 * need one so we can link it in.
	 *
	 * NOTE: in this stage of development, the passed 'par' is
	 * almost always NULL.
	 */
	while ((par = TAILQ_FIRST(&dvp->v_namecache)) == NULL) {
		par = cache_alloc();
		if (TAILQ_FIRST(&dvp->v_namecache) != NULL)
			free(par, M_VFSCACHE);
		else
			cache_setvp(par, dvp); /* XXX par not locked */
	}

	/*
	 * Deal with "." and "..".  In this stage of code development we leave
	 * the returned ncpp NULL.  Note that if the namecache is disjoint,
	 * we won't find a vnode for "..".
	 */
	if (cnp->cn_nameptr[0] == '.') {
		if (cnp->cn_namelen == 1) {
			*vpp = dvp;
			dothits++;
			numposhits++;	/* include in total statistics */
			return (-1);
		}
		if (cnp->cn_namelen == 2 && cnp->cn_nameptr[1] == '.') {
			dotdothits++;
			numposhits++;	/* include in total statistics */
			if ((cnp->cn_flags & CNP_MAKEENTRY) == 0)
				return (0);
			if (par->nc_parent == NULL ||
			    par->nc_parent->nc_vp == NULL) {
				return (0);
			}
			*vpp = par->nc_parent->nc_vp;
			return (-1);
		}
	}

	/*
	 * Try to locate an existing entry
	 */
	hash = fnv_32_buf(cnp->cn_nameptr, cnp->cn_namelen, FNV1_32_INIT);
	bpar = par;
	hash = fnv_32_buf(&bpar, sizeof(bpar), hash);
restart:
	LIST_FOREACH(ncp, (NCHHASH(hash)), nc_hash) {
		numchecks++;

		/*
		 * Zap entries that have timed out.
		 */
		if (ncp->nc_timeout && 
		    (int)(ncp->nc_timeout - ticks) < 0
		) {
			cache_zap(cache_hold(ncp));
			goto restart;
		}

		/*
		 * Break out if we find a matching entry.
		 */
		if (ncp->nc_parent == par &&
		    ncp->nc_nlen == cnp->cn_namelen &&
		    bcmp(ncp->nc_name, cnp->cn_nameptr, ncp->nc_nlen) == 0
		) {
			cache_hold(ncp);
			break;
		}
	}

	/*
	 * We found an entry but it is unresolved, act the same as if we
	 * failed to locate the entry.  cache_enter() will do the right
	 * thing.
	 */
	if (ncp && (ncp->nc_flag & NCF_UNRESOLVED)) {
		cache_drop(ncp);
		ncp = NULL;
	}

	/*
	 * If we failed to locate an entry, return 0 (indicates failure).
	 */
	if (ncp == NULL) {
		if ((cnp->cn_flags & CNP_MAKEENTRY) == 0) {
			nummisszap++;
		} else {
			nummiss++;
		}
		gd->gd_nchstats->ncs_miss++;
		return (0);
	}

	/*
	 * If we found an entry, but we don't want to have one, we zap it.
	 */
	if ((cnp->cn_flags & CNP_MAKEENTRY) == 0) {
		numposzaps++;
		gd->gd_nchstats->ncs_badhits++;
		cache_zap(ncp);
		return (0);
	}

	/*
	 * If the vnode is not NULL then return the positive match.
	 */
	if (ncp->nc_vp) {
		numposhits++;
		gd->gd_nchstats->ncs_goodhits++;
		*vpp = ncp->nc_vp;
		cache_drop(ncp);
		return (-1);
	}

	/*
	 * If the vnode is NULL we found a negative match.  If we want to
	 * create it, purge the negative match and return failure (as if
	 * we hadn't found a match in the first place).
	 */
	if (cnp->cn_nameiop == NAMEI_CREATE) {
		numnegzaps++;
		gd->gd_nchstats->ncs_badhits++;
		cache_zap(ncp);
		return (0);
	}

	numneghits++;

	/*
	 * We found a "negative" match, ENOENT notifies client of this match.
	 * The nc_flag field records whether this is a whiteout.  Since there
	 * is no vnode we can use the vnode tailq link field with ncneglist.
	 */
	TAILQ_REMOVE(&ncneglist, ncp, nc_vnode);
	TAILQ_INSERT_TAIL(&ncneglist, ncp, nc_vnode);
	gd->gd_nchstats->ncs_neghits++;
	if (ncp->nc_flag & NCF_WHITEOUT)
		cnp->cn_flags |= CNP_ISWHITEOUT;
	cache_drop(ncp);
	return (ENOENT);
}

/*
 * Add an entry to the cache.  (OLD API)
 *
 * XXX OLD API ROUTINE!  WHEN ALL VFSs HAVE BEEN CLEANED UP THIS PROCEDURE
 * WILL BE REMOVED.
 */
void
cache_enter(struct vnode *dvp, struct vnode *vp, struct componentname *cnp)
{
	struct namecache *par;
	struct namecache *ncp;
	struct namecache *new_ncp;
	struct namecache *bpar;
	struct nchashhead *nchpp;
	u_int32_t hash;

	/*
	 * If the directory has no namecache entry we must associate one with
	 * it.  The name of the entry is not known so it isn't hashed.  This
	 * is a severe hack to support the old API.
	 */
	while ((par = TAILQ_FIRST(&dvp->v_namecache)) == NULL) {
		par = cache_alloc();
		if (TAILQ_FIRST(&dvp->v_namecache) != NULL)
			free(par, M_VFSCACHE);
		else
			cache_setvp(par, dvp);
	}
	cache_hold(par);

	/*
	 * This may be a bit confusing.  "." and ".." are 'virtual' entries.
	 * We do not actually create a namecache entry representing either.
	 * However, the ".." case is used to linkup a potentially disjoint
	 * directory with its parent, to disconnect a directory from its
	 * parent, or to change an existing linkage that may no longer be
	 * correct (as might occur when a subdirectory is renamed).
	 */

	if (cnp->cn_namelen == 1 && cnp->cn_nameptr[0] == '.') {
		cache_drop(par);
		return;
	}
	if (cnp->cn_namelen == 2 && cnp->cn_nameptr[0] == '.' &&
	    cnp->cn_nameptr[1] == '.'
	) {
		if (vp == NULL) {
			if (par->nc_parent)
				cache_unlink_parent(par);
		} else {
			while ((ncp = TAILQ_FIRST(&vp->v_namecache)) == NULL) {
				ncp = cache_alloc();
				if (TAILQ_FIRST(&vp->v_namecache) != NULL)
					free(ncp, M_VFSCACHE);
				else
					cache_setvp(ncp, vp);
			}
			/*
			 * ncp is the new parent of par
			 */
			cache_hold(ncp);
			if (par->nc_parent)
				cache_unlink_parent(par);
			cache_link_parent(par, ncp);
			cache_drop(ncp);
		}
		cache_drop(par);
		return;
	}

	/*
	 * Locate other entries associated with this vnode and zap them,
	 * because the purge code may not be able to find them due to
	 * the topology not yet being consistent.  This is a hack (this
	 * whole routine is a hack, actually, so that makes this a hack
	 * inside a hack).
	 */
	if (vp) {
again:
		TAILQ_FOREACH(ncp, &vp->v_namecache, nc_vnode) {
			if ((ncp->nc_flag & NCF_UNRESOLVED) == 0) {
				cache_zap(cache_hold(ncp));
				goto again;
			}
		}
	}

	/*
	 * Try to find a match in the hash table, allocate a new entry if
	 * we can't.  We have to retry the loop after any potential blocking
	 * situation.
	 */
	hash = fnv_32_buf(cnp->cn_nameptr, cnp->cn_namelen, FNV1_32_INIT);
	bpar = par;
	hash = fnv_32_buf(&bpar, sizeof(bpar), hash);

	new_ncp = NULL;
againagain:
	LIST_FOREACH(ncp, (NCHHASH(hash)), nc_hash) {
		numchecks++;

		/*
		 * Break out if we find a matching entry.
		 */
		if (ncp->nc_parent == par &&
		    ncp->nc_nlen == cnp->cn_namelen &&
		    bcmp(ncp->nc_name, cnp->cn_nameptr, ncp->nc_nlen) == 0
		) {
			cache_hold(ncp);
			break;
		}
	}
	if (ncp == NULL) {
		if (new_ncp == NULL) {
			new_ncp = cache_alloc();
			new_ncp->nc_name = malloc(cnp->cn_namelen, 
						M_VFSCACHE, M_WAITOK);
			goto againagain;
		}
		ncp = new_ncp;
		cache_hold(ncp);
		ncp->nc_nlen = cnp->cn_namelen;
		bcopy(cnp->cn_nameptr, ncp->nc_name, cnp->cn_namelen);
		nchpp = NCHHASH(hash);
		LIST_INSERT_HEAD(nchpp, ncp, nc_hash);
		ncp->nc_flag |= NCF_HASHED;
		cache_link_parent(ncp, par);
	} else if (new_ncp) {
		free(new_ncp->nc_name, M_VFSCACHE);
		free(new_ncp, M_VFSCACHE);
	}
	cache_drop(par);
	cache_setunresolved(ncp);
	cache_setvp(ncp, vp);

	/*
	 * Set a timeout
	 */
	if (cnp->cn_flags & CNP_CACHETIMEOUT) {
		if ((ncp->nc_timeout = ticks + cnp->cn_timeout) == 0)
			ncp->nc_timeout = 1;
	}

	/*
	 * If the target vnode is NULL if this is to be a negative cache
	 * entry.
	 */
	if (vp == NULL) {
		ncp->nc_flag &= ~NCF_WHITEOUT;
		if (cnp->cn_flags & CNP_ISWHITEOUT)
			ncp->nc_flag |= NCF_WHITEOUT;
	}
	cache_drop(ncp);

	/*
	 * Don't cache too many negative hits
	 */
	if (numneg > MINNEG && numneg * ncnegfactor > numcache) {
		ncp = TAILQ_FIRST(&ncneglist);
		KKASSERT(ncp != NULL);
		cache_zap(cache_hold(ncp));
	}
}

/*
 * Name cache initialization, from vfsinit() when we are booting
 */
void
nchinit(void)
{
	int i;
	globaldata_t gd;

	/* initialise per-cpu namecache effectiveness statistics. */
	for (i = 0; i < ncpus; ++i) {
		gd = globaldata_find(i);
		gd->gd_nchstats = &nchstats[i];
	}
	
	TAILQ_INIT(&ncneglist);
	nchashtbl = hashinit(desiredvnodes*2, M_VFSCACHE, &nchash);
}

/*
 * Called from start_init() to bootstrap the root filesystem.  Returns
 * a referenced, unlocked namecache record.
 */
struct namecache *
cache_allocroot(struct vnode *vp)
{
	struct namecache *ncp = cache_alloc();

	cache_setvp(ncp, vp);
	ncp->nc_flag |= NCF_MOUNTPT | NCF_ROOT;
	return(cache_hold(ncp));
}

/*
 * vfs_cache_setroot()
 *
 *	Create an association between the root of our namecache and
 *	the root vnode.  This routine may be called several times during
 *	booting.
 *
 *	If the caller intends to save the returned namecache pointer somewhere
 *	it must cache_hold() it.
 */
void
vfs_cache_setroot(struct vnode *nvp, struct namecache *ncp)
{
	struct vnode *ovp;
	struct namecache *oncp;

	ovp = rootvnode;
	oncp = rootncp;
	rootvnode = nvp;
	rootncp = ncp;

	if (ovp)
		vrele(ovp);
	if (oncp)
		cache_drop(oncp);
}

/*
 * Invalidate all namecache entries to a particular vnode as well as 
 * any direct children of that vnode in the namecache.  This is a 
 * 'catch all' purge used by filesystems that do not know any better.
 *
 * A new vnode v_id is generated.  Note that no vnode will ever have a
 * v_id of 0.
 *
 * Note that the linkage between the vnode and its namecache entries will
 * be removed, but the namecache entries themselves might stay put due to
 * active references from elsewhere in the system or due to the existance of
 * the children.   The namecache topology is left intact even if we do not
 * know what the vnode association is.  Such entries will be marked
 * NCF_UNRESOLVED.
 *
 * XXX: Only time and the size of v_id prevents this from failing:
 * XXX: In theory we should hunt down all (struct vnode*, v_id)
 * XXX: soft references and nuke them, at least on the global
 * XXX: v_id wraparound.  The period of resistance can be extended
 * XXX: by incrementing each vnodes v_id individually instead of
 * XXX: using the global v_id.
 */
void
cache_purge(struct vnode *vp)
{
	static u_long nextid;
	struct namecache *ncp;
	struct namecache *scan;

	/*
	 * Disassociate the vnode from its namecache entries along with
	 * (for historical reasons) any direct children.
	 */
	while ((ncp = TAILQ_FIRST(&vp->v_namecache)) != NULL) {
		cache_hold(ncp);

restart: /* YYY hack, fix me */
		TAILQ_FOREACH(scan, &ncp->nc_list, nc_entry) {
			if ((scan->nc_flag & NCF_UNRESOLVED) == 0) {
				cache_zap(cache_hold(scan));
				goto restart;
			}
		}
		cache_zap(ncp);
	}

	/*
	 * Calculate a new unique id for ".." handling
	 */
	do {
		nextid++;
	} while (nextid == vp->v_id || nextid == 0);
	vp->v_id = nextid;
}

/*
 * Flush all entries referencing a particular filesystem.
 *
 * Since we need to check it anyway, we will flush all the invalid
 * entries at the same time.
 */
void
cache_purgevfs(struct mount *mp)
{
	struct nchashhead *nchpp;
	struct namecache *ncp, *nnp;

	/*
	 * Scan hash tables for applicable entries.
	 */
	for (nchpp = &nchashtbl[nchash]; nchpp >= nchashtbl; nchpp--) {
		ncp = LIST_FIRST(nchpp);
		if (ncp)
			cache_hold(ncp);
		while (ncp) {
			nnp = LIST_NEXT(ncp, nc_hash);
			if (nnp)
				cache_hold(nnp);
			if (ncp->nc_vp && ncp->nc_vp->v_mount == mp)
				cache_zap(ncp);
			else
				cache_drop(ncp);
			ncp = nnp;
		}
	}
}

/*
 * cache_leaf_test()
 *
 *	Test whether the vnode is at a leaf in the nameicache tree.
 *
 *	Returns 0 if it is a leaf, -1 if it isn't.
 */
int
cache_leaf_test(struct vnode *vp)
{
	struct namecache *scan;
	struct namecache *ncp;

	TAILQ_FOREACH(scan, &vp->v_namecache, nc_vnode) {
		TAILQ_FOREACH(ncp, &scan->nc_list, nc_entry) {
			/* YYY && ncp->nc_vp->v_type == VDIR ? */
			if (ncp->nc_vp != NULL)
				return(-1);
		}
	}
	return(0);
}

/*
 * Perform canonical checks and cache lookup and pass on to filesystem
 * through the vop_cachedlookup only if needed.
 *
 * vop_lookup_args {
 *	struct vnode a_dvp;
 *	struct vnode **a_vpp;
 *	struct componentname *a_cnp;
 * }
 */
int
vfs_cache_lookup(struct vop_lookup_args *ap)
{
	struct vnode *dvp, *vp;
	int lockparent;
	int error;
	struct vnode **vpp = ap->a_vpp;
	struct componentname *cnp = ap->a_cnp;
	struct ucred *cred = cnp->cn_cred;
	int flags = cnp->cn_flags;
	struct thread *td = cnp->cn_td;
	u_long vpid;	/* capability number of vnode */

	*vpp = NULL;
	dvp = ap->a_dvp;
	lockparent = flags & CNP_LOCKPARENT;

	if (dvp->v_type != VDIR)
                return (ENOTDIR);

	if ((flags & CNP_ISLASTCN) && (dvp->v_mount->mnt_flag & MNT_RDONLY) &&
	    (cnp->cn_nameiop == NAMEI_DELETE || cnp->cn_nameiop == NAMEI_RENAME)) {
		return (EROFS);
	}

	error = VOP_ACCESS(dvp, VEXEC, cred, td);

	if (error)
		return (error);

	error = cache_lookup(dvp, vpp, cnp);

	if (!error) 
		return (VOP_CACHEDLOOKUP(dvp, vpp, cnp));

	if (error == ENOENT)
		return (error);

	vp = *vpp;
	vpid = vp->v_id;
	cnp->cn_flags &= ~CNP_PDIRUNLOCK;
	if (dvp == vp) {   /* lookup on "." */
		vref(vp);
		error = 0;
	} else if (flags & CNP_ISDOTDOT) {
		VOP_UNLOCK(dvp, NULL, 0, td);
		cnp->cn_flags |= CNP_PDIRUNLOCK;
		error = vget(vp, NULL, LK_EXCLUSIVE, td);
		if (!error && lockparent && (flags & CNP_ISLASTCN)) {
			if ((error = vn_lock(dvp, NULL, LK_EXCLUSIVE, td)) == 0)
				cnp->cn_flags &= ~CNP_PDIRUNLOCK;
		}
	} else {
		error = vget(vp, NULL, LK_EXCLUSIVE, td);
		if (!lockparent || error || !(flags & CNP_ISLASTCN)) {
			VOP_UNLOCK(dvp, NULL, 0, td);
			cnp->cn_flags |= CNP_PDIRUNLOCK;
		}
	}
	/*
	 * Check that the capability number did not change
	 * while we were waiting for the lock.
	 */
	if (!error) {
		if (vpid == vp->v_id)
			return (0);
		vput(vp);
		if (lockparent && dvp != vp && (flags & CNP_ISLASTCN)) {
			VOP_UNLOCK(dvp, NULL, 0, td);
			cnp->cn_flags |= CNP_PDIRUNLOCK;
		}
	}
	if (cnp->cn_flags & CNP_PDIRUNLOCK) {
		error = vn_lock(dvp, NULL, LK_EXCLUSIVE, td);
		if (error)
			return (error);
		cnp->cn_flags &= ~CNP_PDIRUNLOCK;
	}
	return (VOP_CACHEDLOOKUP(dvp, vpp, cnp));
}

static int disablecwd;
SYSCTL_INT(_debug, OID_AUTO, disablecwd, CTLFLAG_RW, &disablecwd, 0, "");

static u_long numcwdcalls; STATNODE(CTLFLAG_RD, numcwdcalls, &numcwdcalls);
static u_long numcwdfail1; STATNODE(CTLFLAG_RD, numcwdfail1, &numcwdfail1);
static u_long numcwdfail2; STATNODE(CTLFLAG_RD, numcwdfail2, &numcwdfail2);
static u_long numcwdfail3; STATNODE(CTLFLAG_RD, numcwdfail3, &numcwdfail3);
static u_long numcwdfail4; STATNODE(CTLFLAG_RD, numcwdfail4, &numcwdfail4);
static u_long numcwdfound; STATNODE(CTLFLAG_RD, numcwdfound, &numcwdfound);

int
__getcwd(struct __getcwd_args *uap)
{
	int buflen;
	int error;
	char *buf;
	char *bp;

	if (disablecwd)
		return (ENODEV);

	buflen = uap->buflen;
	if (buflen < 2)
		return (EINVAL);
	if (buflen > MAXPATHLEN)
		buflen = MAXPATHLEN;

	buf = malloc(buflen, M_TEMP, M_WAITOK);
	bp = kern_getcwd(buf, buflen, &error);
	if (error == 0)
		error = copyout(bp, uap->buf, strlen(bp) + 1);
	free(buf, M_TEMP);
	return (error);
}

char *
kern_getcwd(char *buf, size_t buflen, int *error)
{
	struct proc *p = curproc;
	char *bp;
	int i, slash_prefixed;
	struct filedesc *fdp;
	struct namecache *ncp;
	struct vnode *vp;

	numcwdcalls++;
	bp = buf;
	bp += buflen - 1;
	*bp = '\0';
	fdp = p->p_fd;
	slash_prefixed = 0;
	for (vp = fdp->fd_cdir; vp != fdp->fd_rdir && vp != rootvnode;) {
		if (vp->v_flag & VROOT) {
			if (vp->v_mount == NULL) {	/* forced unmount */
				*error = EBADF;
				return(NULL);
			}
			vp = vp->v_mount->mnt_vnodecovered;
			continue;
		}
		TAILQ_FOREACH(ncp, &vp->v_namecache, nc_vnode) {
			if (ncp->nc_parent && ncp->nc_parent->nc_vp &&
			    ncp->nc_nlen > 0) {
				break;
			}
		}
		if (ncp == NULL) {
			numcwdfail2++;
			*error = ENOENT;
			return(NULL);
		}
		for (i = ncp->nc_nlen - 1; i >= 0; i--) {
			if (bp == buf) {
				numcwdfail4++;
				*error = ENOMEM;
				return(NULL);
			}
			*--bp = ncp->nc_name[i];
		}
		if (bp == buf) {
			numcwdfail4++;
			*error = ENOMEM;
			return(NULL);
		}
		*--bp = '/';
		slash_prefixed = 1;
		vp = ncp->nc_parent->nc_vp;
	}
	if (!slash_prefixed) {
		if (bp == buf) {
			numcwdfail4++;
			*error = ENOMEM;
			return(NULL);
		}
		*--bp = '/';
	}
	numcwdfound++;
	*error = 0;
	return (bp);
}

/*
 * Thus begins the fullpath magic.
 */

#undef STATNODE
#define STATNODE(name)							\
	static u_int name;						\
	SYSCTL_UINT(_vfs_cache, OID_AUTO, name, CTLFLAG_RD, &name, 0, "")

static int disablefullpath;
SYSCTL_INT(_debug, OID_AUTO, disablefullpath, CTLFLAG_RW,
    &disablefullpath, 0, "");

STATNODE(numfullpathcalls);
STATNODE(numfullpathfail1);
STATNODE(numfullpathfail2);
STATNODE(numfullpathfail3);
STATNODE(numfullpathfail4);
STATNODE(numfullpathfound);

int
vn_fullpath(struct proc *p, struct vnode *vn, char **retbuf, char **freebuf) 
{
	char *bp, *buf;
	int i, slash_prefixed;
	struct filedesc *fdp;
	struct namecache *ncp;
	struct vnode *vp;

	numfullpathcalls++;
	if (disablefullpath)
		return (ENODEV);

	if (p == NULL)
		return (EINVAL);

	/* vn is NULL, client wants us to use p->p_textvp */
	if (vn == NULL) {
		if ((vn = p->p_textvp) == NULL)
			return (EINVAL);
	}

	buf = malloc(MAXPATHLEN, M_TEMP, M_WAITOK);
	bp = buf + MAXPATHLEN - 1;
	*bp = '\0';
	fdp = p->p_fd;
	slash_prefixed = 0;
	for (vp = vn; vp != fdp->fd_rdir && vp != rootvnode;) {
		if (vp->v_flag & VROOT) {
			if (vp->v_mount == NULL) {	/* forced unmount */
				free(buf, M_TEMP);
				return (EBADF);
			}
			vp = vp->v_mount->mnt_vnodecovered;
			continue;
		}
		TAILQ_FOREACH(ncp, &vp->v_namecache, nc_vnode) {
			if (ncp->nc_parent && ncp->nc_parent->nc_vp &&
			    ncp->nc_nlen > 0) {
				break;
			}
		}
		if (ncp == NULL) {
			numfullpathfail2++;
			free(buf, M_TEMP);
			return (ENOENT);
		}
		for (i = ncp->nc_nlen - 1; i >= 0; i--) {
			if (bp == buf) {
				numfullpathfail4++;
				free(buf, M_TEMP);
				return (ENOMEM);
			}
			*--bp = ncp->nc_name[i];
		}
		if (bp == buf) {
			numfullpathfail4++;
			free(buf, M_TEMP);
			return (ENOMEM);
		}
		*--bp = '/';
		slash_prefixed = 1;
		vp = ncp->nc_parent->nc_vp;
	}
	if (!slash_prefixed) {
		if (bp == buf) {
			numfullpathfail4++;
			free(buf, M_TEMP);
			return (ENOMEM);
		}
		*--bp = '/';
	}
	numfullpathfound++;
	*retbuf = bp; 
	*freebuf = buf;
	return (0);
}

