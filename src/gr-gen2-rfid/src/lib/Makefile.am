#
# Copyright 2004,2005,2006 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
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

ourpythondir = $(grpythondir)
ourlibdir    = $(grpyexecdir)

INCLUDES = $(STD_DEFINES_AND_INCLUDES) $(PYTHON_CPPFLAGS) $(CPPUNIT_INCLUDES)

SWIGCPPPYTHONARGS = -noruntime -c++ -fvirtual -python -modern $(PYTHON_CPPFLAGS) \
$(STD_DEFINES_AND_INCLUDES) -I$(swigincludedir) -I$(grincludedir)

#SWIGPYTHONARGS = $(SWIGPYTHONFLAGS) $(SWIGGRFLAGS)

ALL_IFILES = 				\
	$(LOCAL_IFILES)			\
	$(NON_LOCAL_IFILES)		

NON_LOCAL_IFILES =			\
	$(GNURADIO_I)	

LOCAL_IFILES = 				\
	$(top_srcdir)/gr-gen2-rfid/src/lib/rfid.i				

# These files are built by SWIG.  The first is the C++ glue.
# The second is the python wrapper that loads the _rfid shared library
# and knows how to call our extensions.

swig_built_sources = 			\
	rfid.cc				\
	rfid.py				

# This gets rfid.py installed in the right place
ourpython_PYTHON =			\
	rfid.py

ourlib_LTLIBRARIES = _rfid.la

# These are the source files that go into the shared library
_rfid_la_SOURCES = 			\
	rfid.cc				\
	rfid_clock_recovery_zc_ff.cc	\
	rfid_cmd_gate.cc		\
	rfid_gen2_reader.cc		\		
	rfid_reader_decoder.cc



# magic flags
_rfid_la_LDFLAGS = $(NO_UNDEFINED) -module -avoid-version

# link the library against some comon swig runtime code and the 
# c++ standard library
_rfid_la_LIBADD = 			\
	$(GNURADIO_CORE_LA)		\
	$(PYTHON_LDFLAGS)		\
	-lstdc++			

rfid.cc rfid.py: rfid.i $(ALL_IFILES)
	$(SWIG) $(SWIGCPPPYTHONARGS) -module rfid -o rfid.cc $(LOCAL_IFILES)

#rfid.cc rfid.py:  $(LOCAL_IFILES) $(ALL_IFILES)
#	$(SWIG) $(SWIGPYTHONARGS) -module rfid -o rfid.cc $(LOCAL_IFILES)

# These headers get installed in ${prefix}/include/gnuradio
grinclude_HEADERS =                     \
	rfid_clock_recovery_zc_ff.cc	\
	rfid_cmd_gate.cc		\
	rfid_gen2_reader.cc		\
	rfid_reader_decoder.cc			

# These swig headers get installed in ${prefix}/include/gnuradio/swig
#swiginclude_HEADERS = 			\
#	$(LOCAL_IFILES)			

# Don't distribute output of swig
dist-hook:
	@for file in $(BUILT_SOURCES); do echo $(RM) $(distdir)/$$file; done
	@for file in $(BUILT_SOURCES); do $(RM) $(distdir)/$$file; done

MOSTLYCLEANFILES = $(BUILT_SOURCES) *.pyc

