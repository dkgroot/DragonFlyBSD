/*-
 * Copyright (c) 1997 Doug Rabson
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
 * $FreeBSD: src/sbin/kldload/kldload.c,v 1.6.2.2 2002/12/07 08:44:02 jmallett Exp $
 * $DragonFly: src/sbin/kldload/kldload.c,v 1.3 2005/04/02 16:04:41 liamfoy Exp $
 */
#include <sys/param.h>
#include <sys/linker.h>

#include <err.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static void
usage(void)
{
    fprintf(stderr, "usage: kldload [-nv] file ...\n");
    exit(1);
}

int
main(int argc, char **argv)
{
    int c;
    int errors;
    int fileid;
    int verbose;
    int check_loaded;

    errors = 0;
    verbose = 0;
    check_loaded = 0;

    while ((c = getopt(argc, argv, "nv")) != -1)
	switch (c) {
	case 'n':
	    check_loaded = 1;
	    break;
	case 'v':
	    verbose = 1;
	    break;
	default:
	    usage();
	}
    argc -= optind;
    argv += optind;

    if (argc == 0)
	usage();

    while (argc-- != 0) {
	fileid = kldload(argv[0]);
	if (fileid < 0) {
	    if (check_loaded != 0 && errno == EEXIST) {
		if (verbose)
		    printf("%s is already loaded\n", argv[0]);
	    } else {
		switch (errno) {
		case EEXIST:
		    warnx("can't load %s: module already loaded or "
			"in kernel", argv[0]);
		    break;
		case ENOEXEC:
		    warnx("an error occurred while loading the module. "
			"Please check dmesg(8) for more details.");
		    break;
		default:
		    warn("can't load %s", argv[0]);
		    break;
		}
		errors++;
	    }
	} else {
	    if (verbose)
		printf("Loaded %s, id=%d\n", argv[0], fileid);
	}
	argv++;
    }

    return errors ? 1 : 0;
}
