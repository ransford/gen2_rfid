#
# Copyright 2004,2005,2006,2008,2009,2010 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

include $(top_srcdir)/Makefile.common

# list of programs run by "make check" and "make distcheck"
TESTS = test_all

# ----------------------------------------------------------------
# rfid C++ library: libgnuradio-rfid.so
# ----------------------------------------------------------------

# C/C++ headers get installed in ${prefix}/include/$(modname)
modinclude_HEADERS = \
	rfid_center_ff.h \
	rfid_reader_decoder.h \
	rfid_tag_decoder_f.h \
	rfid_clock_recovery_zc_ff.h \
	rfid_command_gate_cc.h \
	rfid_global_vars.h \
	rfid_reader_f.h

lib_LTLIBRARIES = libgnuradio-rfid.la

libgnuradio_rfid_la_SOURCES = \
	rfid_center_ff.cc \
	rfid_reader_decoder.cc \
	rfid_tag_decoder_f.cc \
	rfid_clock_recovery_zc_ff.cc \
	rfid_command_gate_cc.cc \
	rfid_global_vars.cc \
	rfid_reader_f.cc

libgnuradio_rfid_la_LIBADD = \
	$(GNURADIO_CORE_LA)

libgnuradio_rfid_la_LDFLAGS = \
	$(NO_UNDEFINED)

# ----------------------------------------------------------------
# rfid C++ QA library: libgnuradio-rfid-qa.so (not installed)
# ----------------------------------------------------------------

noinst_LTLIBRARIES = libgnuradio-rfid-qa.la

libgnuradio_rfid_qa_la_SOURCES = \
	qa_rfid.cc 

libgnuradio_rfid_qa_la_LDFLAGS = $(NO_UNDEFINED) -version-info 0:0:0

libgnuradio_rfid_qa_la_LIBADD = \
	libgnuradio-rfid.la \
	$(CPPUNIT_LIBS)

# ----------------------------------------------------------------
# headers that don't get installed
# ----------------------------------------------------------------
noinst_HEADERS = 
# ----------------------------------------------------------------
# test program
# ----------------------------------------------------------------
noinst_PROGRAMS = \
	test_all

test_all_SOURCES = test_all.cc
test_all_LDADD   = libgnuradio-rfid-qa.la
