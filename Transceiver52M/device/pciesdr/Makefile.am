include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CXXFLAGS = -lpthread -lsdr $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)

noinst_HEADERS = PCIESDRDevice.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = PCIESDRDevice.cpp
libdevice_la_LIBADD = $(top_builddir)/Transceiver52M/device/common/libdevice_common.la
