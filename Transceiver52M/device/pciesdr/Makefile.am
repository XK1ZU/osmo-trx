include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/..
AM_CXXFLAGS = -lpthread -lsdr $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)

noinst_HEADERS = PCIESDRDevice.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = PCIESDRDevice.cpp
