/*
 * random.h -- A strong random number generator
 *
 * $FreeBSD: src/sys/sys/random.h,v 1.19.2.2 2002/09/17 17:11:54 sam Exp $
 *
 * Version 0.95, last modified 18-Oct-95
 * 
 * Copyright Theodore Ts'o, 1994, 1995.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 * 
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU Public License, in which case the provisions of the GPL are
 * required INSTEAD OF the above restrictions.  (This clause is
 * necessary due to a potential bad interaction between the GPL and
 * the restrictions contained in a BSD-style copyright.)
 * 
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Many kernel routines will have a use for good random numbers,
 * for example, for truly random TCP sequence numbers, which prevent
 * certain forms of TCP spoofing attacks.
 * 
 */

#ifndef	_SYS_RANDOM_H_
#define	_SYS_RANDOM_H_

#ifndef _SYS_TYPES_H_
#include <sys/types.h>
#endif
#ifndef _SYS_IOCCOM_H_
#include <sys/ioccom.h>
#endif

#define	MEM_SETIRQ	_IOW('r', 1, u_int16_t)	/* set interrupt */
#define	MEM_CLEARIRQ	_IOW('r', 2, u_int16_t)	/* clear interrupt */
#define	MEM_RETURNIRQ	_IOR('r', 3, u_int16_t)	/* obsolete */
#define	MEM_FINDIRQ	_IOWR('r', 4, u_int16_t) /* next interrupt */

#ifdef _KERNEL

/*
 * XXX: consider only statically allocating some, and allocating
 *      most others dynamically.
 */
#define RAND_SRC_UNKNOWN	0x00
#define RAND_SRC_SEEDING	0x01
#define RAND_SRC_TIMING		0x02
#define RAND_SRC_INTR		0x03
#define RAND_SRC_RDRAND		0x04
#define RAND_SRC_PADLOCK	0x05
#define RAND_SRC_GLXSB		0x06
#define RAND_SRC_HIFN		0x07
#define RAND_SRC_UBSEC		0x08
#define RAND_SRC_SAFE		0x09
#define RAND_SRC_VIRTIO		0x0a

/* Type of the cookie passed to add_interrupt_randomness. */

struct random_softc {
	int		sc_intr;
	int		sc_enabled;
};

/* Exported functions */

void rand_initialize(void);
void add_keyboard_randomness(u_char scancode);
void add_interrupt_randomness(int intr);
int add_buffer_randomness(const char *, int);
int add_buffer_randomness_src(const char *, int, int srcid);

u_int read_random(void *buf, u_int size);
u_int read_random_unlimited(void *buf, u_int size);
struct knote;
int random_filter_read(struct knote *kn, long hint);

#endif /* _KERNEL */

#endif /* !_SYS_RANDOM_H_ */
