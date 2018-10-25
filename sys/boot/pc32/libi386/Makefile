# $FreeBSD: src/sys/boot/i386/libi386/Makefile,v 1.32 2003/07/02 12:45:45 ru Exp $
#
.if ${MACHINE_ARCH} == "x86_64"
.MAKEFLAGS:  MACHINE_ARCH=i386 MACHINE_PLATFORM=pc32 REALLY_X86_64=true
.else
.MAKEFLAGS:  REALLY_X86_64=false
.endif

LIB=			i386
INTERNALLIB=		true
SYSDIR?=		${.CURDIR}/../../..
.include "${SYSDIR}/conf/acpi.mk"

SRCS=	biosacpi.c bioscd.c biosdisk.c biosmem.c biospnp.c \
	biospci.c biossmap.c bootinfo.c bootinfo32.c bootinfo64.c \
	comconsole.c devicename.c elf32_freebsd.c elf64_freebsd.c \
	i386_copy.c i386_module.c nullconsole.c pxe.c pxetramp.s \
	smbios.c time.c vidconsole.c x86_64_tramp.S \
	multiboot.c multiboot_tramp.S

CFLAGS+=	-ffreestanding
CFLAGS+=	-fno-strict-aliasing
BOOT_COMCONSOLE_PORT?= 0x3f8
CFLAGS+=	-DCOMPORT=${BOOT_COMCONSOLE_PORT}

BOOT_COMCONSOLE_SPEED?= 9600
CFLAGS+=	-DCOMSPEED=${BOOT_COMCONSOLE_SPEED}

.ifdef(BOOT_BIOSDISK_DEBUG)
# Make the disk code more talkative
CFLAGS+= -DDISK_DEBUG
.endif

# Include simple terminal emulation (cons25-compatible)
CFLAGS+= -DTERM_EMU

CFLAGS+=	-I${.CURDIR}/../../common -I${.CURDIR}/../btx/lib \
		-I"${SYSDIR}/${ACPICA_DIR}/include" \
		-I"${SYSDIR}/${OSACPI_MI_DIR}" \
		-I${.CURDIR}/../../.. -I.
# the location of libstand
CFLAGS+=	-I${.CURDIR}/../../../../lib/libstand/

.include <bsd.lib.mk>
