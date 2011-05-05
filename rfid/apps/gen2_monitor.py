#!/usr/bin/env python
#Developed by: Michael Buettner (buettner@cs.washington.edu)

from gnuradio import gr, gru
from gnuradio import usrp2

from gnuradio.eng_option import eng_option
from string import split
import math
from threading import Timer

import rfid

PRINT_TO_SCREEN = False
LOG_INTERVAL = 1

#If DEBUG = True the program will output two traces of the signal
#rm_signal.out shows the signal being fed into the block that decodes the 
# reader commands.
#rm_thresh.out shows the threshold that the block is using for detecting
# positive/negative edges in the reader signal. Use this for debugging.
#Plot them in Matlab using read_complex_binary.m (in gnuradio tree) 
DEBUG = True

class reader_monitor(gr.top_block):
    def __init__(self, u, center_freq, hw_dec_rate, downsample_rate, pulse_width, tari, debug_on):
        gr.top_block.__init__(self)

        self.u = u
        self.u.set_decim(hw_dec_rate)
        g = u.gain_range()
        self.u.set_gain(float(g[0] + g[1]) / 4)
        print "Using gain of", float(g[0] + g[1]) / 4 , "(", g[0], "-", g[1], ")"
        x = self.u.set_center_freq(center_freq)
        if not x:
            print "Couldn't set rx freq"
            
        sample_rate = self.u.adc_rate() / hw_dec_rate
        us_per_sample = 1e6 / sample_rate * downsample_rate
        print "USRP Sample Rate: "+ str(sample_rate) + " us Per Sample: " + str(us_per_sample)

        #25 Msps is too much to process. But we don't want to decimate at the USRP because we want the full band.
        # We can downsample, because we don't care about aliasing. 
        self.downsample = gr.keep_one_in_n(gr.sizeof_gr_complex, int(downsample_rate))

        self.to_mag = gr.complex_to_mag()
        
        #For TARI = 24, PW == DELIM. So we can't do a matched filter, or everything becomes a delimiter and a zero.
        # PW / 2 smooths reasonably well.
        self.smooth = gr.moving_average_ff(int(pulse_width / us_per_sample) /2  , int(pulse_width / us_per_sample) /2 ) 
        
        self.rd = rfid.reader_decoder(us_per_sample, tari)

        if(debug_on):
            self.sink = gr.file_sink(gr.sizeof_float, "rm_thresh.out")
            self.signal_sink = gr.file_sink(gr.sizeof_float, "rm_signal.out")
            self.connect(self.smooth, self.signal_sink)

        else:
            self.sink = gr.null_sink(gr.sizeof_float)
        
        self.connect(self.u, self.downsample, self.to_mag, self.smooth, self.rd, self.sink)

        

    
    def get_log(self):
        if self.rd != None:
            return self.rd.get_log()
        

def timed_log_write(rm, log_file):
    log = rm.get_log()
    print "Wrote log: " + str(log.count())
    
    i = log.count();
    for k in range(0, i):
        decode_log_msg(log.delete_head_nowait(), log_file, PRINT_TO_SCREEN)
        
        i = i + 1
    log_file.flush()
    
    t = Timer(LOG_INTERVAL, timed_log_write, [rm, log_file])
    t.setDaemon(True)
    t.start()

def main():
    center_freq = 915e6
    hw_dec_rate = 4       #USRP decimation rate. Lower -> more bandwidth
    downsample_rate = 10  #If you get 'S's, increase downsample_rate

    
    #tari = 7             # us. If you don't know, use 0. Then look at log to find actual TARI value.
    #pulse_width = 3.5      # us. 1/2 tari

    tari = 24             # us. If you don't know, use 0. Then look at log to find actual TARI value.
    pulse_width = 12      # us. 1/2 tari


    log_file = open("rm_log.log", 'w')

    u  = usrp2.source_32fc()
    
    
    rm = reader_monitor(u, center_freq, hw_dec_rate, downsample_rate, pulse_width, tari, DEBUG)

    x = gr.enable_realtime_scheduling()
    if x != gr.RT_OK:
        print "Warning: failed to enable realtime scheduling"
        
    rm.start()
    t = Timer(LOG_INTERVAL, timed_log_write, [rm, log_file])
    t.setDaemon(True)
    t.start()

    while 1:
        c = raw_input("Q to Quit")
        if c == "Q" or c == "q":
           
            break
            
    print "Shutting Down"
    rm.stop()
    log_file.close()


def decode_log_msg(msg, log_file, print_to_screen):
    READER_COMMAND, POWER_DOWN, START = range(3)
    
    if msg.type() == POWER_DOWN:
        if(print_to_screen):
            print msg.to_string()

        log_file.write(msg.to_string()+"\n")

    if msg.type() == START:
        if(print_to_screen):
            print msg.to_string()
        log_file.write(msg.to_string()+"\n")

    if msg.type() == READER_COMMAND:
        formatted_string = msg.to_string()
        fields = split(msg.to_string(), ",")
        cmd = fields[0].split(':')[1]

        if cmd[0:2] == "00" and len(cmd) == 4:
            formatted_string = "DECODED:QREP,%s" % (formatted_string)

        elif cmd[0:2] == "01" and len(cmd) == 18:
            RN16  = hex(int(bin_to_dec(cmd[2:])))[2:]
            while len(RN16) < 4:
                RN16 = "0" + RN16
            
            formatted_string = "DECODED:ACK,RN16:%s,%s" % (RN16, formatted_string)
            

        elif cmd[0:4] == "1000" and len(cmd) == 22:
            formatted_string = "DECODED:QUERY,%s" % (formatted_string)

        elif cmd[0:4] == "1001" and len(cmd) == 9:
            formatted_string = "DECODED:QADJ,%s" % (formatted_string)

        elif cmd[0:4] == "1010" and len(cmd) > 44:
            formatted_string = "DECODED:SELECT,%s" % (formatted_string)

        elif cmd[0:8] == "11000000" and len(cmd) == 8:
            formatted_string = "DECODED:NAK,%s" % (formatted_string)

        elif cmd[0:8] == "11000001" and len(cmd) == 40:
            formatted_string = "DECODED:REQ_RN,%s" % (formatted_string)

        elif cmd[0:8] == "11000010" and len(cmd) > 57 :
            formatted_string = "DECODED:READ,%s" % (formatted_string)

        elif cmd[0:8] == "11000011" and len(cmd) > 58:
            formatted_string = "DECODED:WRITE,%s" % (formatted_string)

        else:
            formatted_string = "DECODED:ERROR,%s" % (formatted_string)

        if(print_to_screen):
            print formatted_string
        log_file.write(formatted_string + "\n")
    

def bin_to_dec(bits):

    dec = int(0)
    for i in range(1, len(bits) + 1):
        dec = dec + int(bits[-i]) * math.pow(2, i - 1)
        

    return dec

if __name__ == '__main__':
    main()
    

        

