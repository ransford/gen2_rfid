dnl Copyright 2007,2008 Free Software Foundation, Inc.
dnl 
dnl This file is part of GNU Radio
dnl 
dnl GNU Radio is free software; you can redistribute it and/or modify
dnl it under the terms of the GNU General Public License as published by
dnl the Free Software Foundation; either version 3, or (at your option)
dnl any later version.
dnl 
dnl GNU Radio is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY; without even the implied warranty of
dnl MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
dnl GNU General Public License for more details.
dnl 
dnl You should have received a copy of the GNU General Public License
dnl along with GNU Radio; see the file COPYING.  If not, write to
dnl the Free Software Foundation, Inc., 51 Franklin Street,
dnl Boston, MA 02110-1301, USA.

AC_DEFUN([GRC_GR_GEN2_RFID],[
    GRC_ENABLE(gr-gen2-rfid)

    dnl Don't do gr-gen2-rfid if usrp or gnuradio-core skipped
    GRC_CHECK_DEPENDENCY(gr-gen2-rfid, usrp)
    GRC_CHECK_DEPENDENCY(gr-gen2-rfid, gnuradio-core)

    AC_CONFIG_FILES([ \
        gr-gen2-rfid/Makefile \
        gr-gen2-rfid/doc/Makefile \
        gr-gen2-rfid/src/Makefile \
       	gr-gen2-rfid/src/lib/Makefile \
        gr-gen2-rfid/src/python/Makefile \
        gr-gen2-rfid/src/python/run_tests
    ])

    GRC_BUILD_CONDITIONAL(gr-gen2-rfid,[
	dnl run_tests is created from run_tests.in.  Make it executable.
	AC_CONFIG_COMMANDS([run_tests_gen2_rfid], [chmod +x gr-gen2-rfid/src/python/run_tests])
    ])
])
