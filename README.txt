Tested on:
  Ubuntu 8.10 64-bit (2.6.27-3-rt)
  GNU Radio revision 11059
  Gen2 reader revision 3416


Hardware:
	USRP, two 900 MHZ daughterboards with the ISM filters removed (see
	endnote)
	Two antennas, preferably directional. Attach to TX/RX ports on
	the daughterboards

	
	
First:
	Download/build/install gnuradio dependencies
	Download gnuradio, do not build
	Make sure you are running a real-time kernel
	


1. Copy src/misc_files/grc_gr_gen2_rfid.m4 to gnuradio/config
2. Edit gnuradio/config/Makefile.am to include grc_gr_gen2_rfid.m4
3. Copy src/gr-gen2-rfid directory to gnuradio/
4. Edit gnuradio/configure.ac, add GRC_GR_GEN2_RFID
5. ./bootstrap;./configure
6. Make sure gen2_rfid was configured
7. make;sudo make install
8. Copy src/misc_files/gen2_reader.rbf to /usr/local/share/usrp/rev4
9. Run sudo GR_SCHEDULER=STS nice -n -20 ./gen2_reader.py
10. Press 'a' and enter to print out the log


Reliably meeting the timing requirements of the Gen 2 protocol is
difficult using the standard USRP/GNURadio configuration. This is
because the default behavior is to look at the signal processing
graph, determine the maximum amount of samples that it can consumed,
and then block on the USRP until that maximum is received before
scheduling the flowgraph.  With low data rates and low processing time
you end up blocking way too long, waiting for samples from the
USRP. To meet the timing requirements you need to modify the
scheduler behavior so that at every call to the USB subsystem whatever
samples are available are immediately passed to the application.  I have
included files that make this modification:

src/misc_files/usrp_source_base.cc needs to be
copied to gr-usrp/src/

src/misc_files/fusb_linux.cc needs to be copied to
usrp/host/lib/legacy/

This should result in gen2_reader.py having latency ~200-300 us. Check
out the files to see how it works. Note that you will be overwriting
the standard file of GNU Radio. In general, the modified files should
work as well as or better than the standard mechanism. 

Hardware Modifications:
The filter on the 900 MHz daughterboards will fail eventually
because of the high amplitude continuous wave. The daughterboard will
be non-functional at that point, with an output power of about 5-10
mW. Check out the gnuradio mailing list for instructions on bypassing
the filter. Bonus: you can  then transmit outside of the ISM band
(assuming that is legal in your area) and you get 500 mW output power
instead of 200 mW.  


Some installation tips at: https://www.noisebridge.net/wiki/RFID_Hacking/usrp/
Implementation details can be found at: http://www.cs.washington.edu/homes/buettner/docs/UW-CSE-09-10-02.PDF

-- Michael Buettner (buettner@cs.washington.edu)
