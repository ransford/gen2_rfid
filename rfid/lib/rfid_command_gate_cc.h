/* -*- c++ -*- */


#ifndef INCLUDED_RFID_COMMAND_GATE_CC_H
#define INCLUDED_RFID_COMMAND_GATE_CC_H

#include <gr_block.h>
#include <gr_message.h>
#include <gr_msg_queue.h>

#ifndef READER_VARS
#include "rfid_global_vars.h"
#endif


class rfid_command_gate_cc;
typedef boost::shared_ptr<rfid_command_gate_cc> rfid_command_gate_cc_sptr;


rfid_command_gate_cc_sptr 
rfid_make_command_gate_cc (int pw, int T1, int sample_rate);

class rfid_command_gate_cc : public gr_block 
{
 
 private:
  friend rfid_command_gate_cc_sptr
  rfid_make_command_gate_cc (int pw, int T1, int sample_rate);
 
  float                   d_us_per_rcv;
  int	                d_pw;  //Reader pulsewidth in us
  int                   d_T1;  //T1 value according to spec
  int			d_sample_rate;
  int                   d_pass_count;
  int                   d_pw_num_samples, d_T1_num_samples;
  double                d_max_rssi, d_min_rssi, d_avg_rssi, d_std_dev_rssi;
  int                   d_sample_count;
  int                   d_num_pulses;
  
 
  float static const AVG_WIN = 1500; // Window to average amplitude over, in us
  float static const THRESH_FRACTION = 0.75; //Percent of avg amplitude to detect edges
  double static const MIN_AMP_THRESH = 0;     //Eventually, expose as user parameter
  float * d_window_samples;   //Array to hold samples for averaging amplitude
  int d_window_length;        //Length of window
  int d_window_index;         //Index to oldest sample
  double d_avg_amp;           //Average amplitude over window
  bool neg_edge_found;        //True if found negative edge for bit
  double d_thresh;            //Amplitude threshold for detecing edges
  double d_min_amp_thresh;    //To filter out nearby readers

  gr_msg_queue_sptr	d_ctrl_out;  //Pipe control messages to reader block.

  rfid_command_gate_cc(int pw, int T1, int sample_rate);
  void forecast (int noutput_items, gr_vector_int &ninput_items_required);
  bool is_negative_edge(float sample);
  bool is_positive_edge(float sample);
  void calc_signal_stats(float * buffer, int len, double * max, double * min, double* avg, double * std_dev );
  


  
 
 public:
  ~rfid_command_gate_cc();
    
  int general_work(int noutput_items, 
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);

  void	set_ctrl_out(const gr_msg_queue_sptr msgq) { d_ctrl_out = msgq; }

};

#endif

