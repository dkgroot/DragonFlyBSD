
KMOD=	virtio_console
SRCS=	virtio_console.c
SRCS+=	virtio_bus_if.h
SRCS+=	bus_if.h device_if.h

.include <bsd.kmod.mk>
CFLAGS+= -Wno-unused-function -Wno-unused-variable
