#!/usr/bin/env python

from gnuradio import gr, gru, rfid
from gnuradio import usrp
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from string import split
from string import strip
from string import atoi
import time
import os


# usrp decimation and filter decimation for different uplink freqs
# Uplink frequency set in reader_vars.h

#40 khz: 32 / 8

up_link_freq = 40  # kHz

dec_rate = 32
sw_dec = 8

interp = 512

samples_per_pulse = 3

class my_top_block(gr.top_block):
    def __init__(self, rx, reader, tx):
        gr.top_block.__init__(self)
               
        samp_freq = (64 / dec_rate) * 1e6
        amplitude = 33000

        num_taps = int(64000 / (dec_rate * up_link_freq * 2))  #Matched filter for 1/2 cycle

        taps = [complex(1,1)] * num_taps
        
        print num_taps

        filt = gr.fir_filter_ccc(sw_dec, taps)
        
        to_mag = gr.complex_to_mag()
        amp = gr.multiply_const_cc(amplitude)

        c_gate = rfid.cmd_gate(dec_rate * sw_dec, reader.STATE_PTR)
        zc = rfid.clock_recovery_zc_ff(samples_per_pulse, 2);
        
        dummy = gr.null_sink(gr.sizeof_float)
        to_complex = gr.float_to_complex()


        r = gr.enable_realtime_scheduling()
        if r != gr.RT_OK:
            print "Warning: failed to enable realtime scheduling"
    
        


        self.connect(rx, filt, to_mag, c_gate, zc,  reader, amp, tx);   

        #uncomment to output RX trace
        #com_out = gr.file_sink(gr.sizeof_gr_complex, "iq_out.dat")  
        #self.connect(rx, com_out)
  
        
       
        

def main():
    

    #TX
    which_usrp = 0
    fpga = "gen2_reader.rbf"

    freq = 915e6
    rx_gain = 20
    
    samp_freq = (64 / dec_rate) * 1e6

    

    tx = usrp.sink_c(which_usrp,fusb_block_size = 1024, fusb_nblocks=4, fpga_filename=fpga)
    tx.set_interp_rate(interp)
    tx_subdev = (0,0)
    tx.set_mux(usrp.determine_tx_mux_value(tx, tx_subdev))
    subdev = usrp.selected_subdev(tx, tx_subdev)
    subdev.set_enable(True)
    subdev.set_gain(subdev.gain_range()[2])
    
    t = tx.tune(subdev.which(), subdev, freq)
    if not t:
        print "Couldn't set tx freq"
#End TX

#RX
            

    rx = usrp.source_c(which_usrp, dec_rate, fusb_block_size = 512, fusb_nblocks = 16, fpga_filename=fpga)
    rx_subdev_spec = (1,0)
    rx.set_mux(usrp.determine_rx_mux_value(rx, rx_subdev_spec))
    rx_subdev = usrp.selected_subdev(rx, rx_subdev_spec)
    rx_subdev.set_gain(rx_gain)
    rx_subdev.set_auto_tr(False)
    rx_subdev.set_enable(True)
    
    us_per_sample = 1 / (64.0 / dec_rate / sw_dec)
    print "Sample Frequency: "+ str(samp_freq) + " us Per Sample: " + str(us_per_sample)
    r = usrp.tune(rx, 0, rx_subdev, freq)
    if not r:
        print "Couldn't set rx freq"

#End RX

    
    gen2_reader = rfid.gen2_reader(dec_rate * sw_dec * samples_per_pulse, interp)

    tb = my_top_block(rx, gen2_reader, tx)
    tb.start()
    

    log_file = open("log_out.log", "w")
    

    
    while 1:
            
        c = raw_input("'Q' to quit\n")
        if c == "q":
            break

        if c == "A" or c == "a":
            log_file.write("T,CMD,ERROR,BITS,SNR\n")
            log = gen2_reader.get_log()
            print "Log has %s Entries"% (str(log.count()))
            i = log.count();
                             
            
            for k in range(0, i):
                msg = log.delete_head_nowait()
                print_log_msg(msg, log_file)

    
    
        
    tb.stop()
   
    log_file.close()

 



def print_log_msg(msg, log_file):
    LOG_PWR_UP, LOG_QUERY, LOG_QREP, LOG_ACK, LOG_NAK, LOG_RN16, LOG_EPC, LOG_EMPTY, LOG_COLLISION, LOG_TIME_MISS, LOG_BAD_RN_MISS, LOG_ERROR, LOG_OKAY = range(13)


    fRed = chr(27) + '[31m'
    fBlue = chr(27) + '[34m'
    fReset = chr(27) + '[0m'


    if msg.type() == LOG_PWR_UP:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t Power Up" %(fields[-1]) 
        log_file.write(fields[-1] + ",PWR_UP,0,0,0\n");

    if msg.type() == LOG_QUERY:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t Query" %(fields[-1]) 
        log_file.write(fields[-1] + ",QUERY,0,0,0\n");

    if msg.type() == LOG_QREP:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t QRep" %(fields[-1]) 
        log_file.write(fields[-1] + ",QREP,0,0,0\n");

    if msg.type() == LOG_ACK:
        fields = split(strip(msg.to_string()), " ")
        rn16 = fields[0].split(",")[0]
        snr = strip(fields[0].split(",")[1])
        tmp = atoi(rn16,2)
        if msg.arg2() == LOG_ERROR:
            print "%s\t%s ACKED w/ Error: %04X%s" %(fields[-1], fRed, tmp, fReset)
            log_file.write(fields[-1] +",ACK,1," + str(hex(tmp))[2:] +"," + snr  +"\n")
        else:
            print "%s\t ACKED: %04X%s" %(fields[-1], tmp, fReset)
            log_file.write(fields[-1] +",ACK,0," + str(hex(tmp))[2:] + "," + snr +"\n");

    if msg.type() == LOG_NAK:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t NAK" %(fields[-1])
        log_file.write(fields[-1] + ",NAK,0,0,0\n");

    
    if msg.type() == LOG_RN16:
        print "LOG_RN16"
        
    if msg.type() == LOG_EPC:
        fields = split(strip(msg.to_string()), " ")
        print fields[0]
        epc = fields[0].split(",")[0]
        snr = strip(fields[0].split(",")[1])
        epc = epc[16:112]
        tmp = atoi(epc,2)
        if msg.arg2() == LOG_ERROR:
            print "%s\t    %s EPC w/ Error: %024X%s" %(fields[-1],fRed, tmp, fReset)
            log_file.write(fields[-1] + ",EPC,1," + str(hex(tmp))[2:-1] + ","+snr + "\n");
        else:
            print "%s\t    %s EPC: %024X%s" %(fields[-1],fBlue, tmp, fReset)
            log_file.write(fields[-1] +",EPC,0," + str(hex(tmp))[2:-1] + "," +snr + "\n");

    if msg.type() == LOG_EMPTY:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t    - Empty Slot - " %(fields[-1]) 
        log_file.write(fields[-1] + ",EMPTY,0,0,0\n");

    if msg.type() == LOG_COLLISION:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t    - Collision - " %(fields[-1]) 
        log_file.write(fields[-1] + ",COLLISION,0,0,0\n");

    if msg.type() == LOG_TIME_MISS:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t    %s Timing Miss%s" %(fields[-1],fRed, fReset)
        log_file.write(fields[-1] + ",TIME_MISS,0,0,0\n");
    
    if msg.type() == LOG_BAD_RN_MISS:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t    %s Bad RN16 Miss%s" %(fields[-1],fRed, fReset)
        log_file.write(fields[-1] + ",BAD_RN_MISS,0,0,0\n");
    

        


if __name__ == '__main__':
    main ()
