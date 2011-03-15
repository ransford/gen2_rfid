/* -*- c++ -*- */


#ifndef INCLUDED_RFID_READER_DECODER_H
#define INCLUDED_RFID_READER_DECODER_H

#include <gr_sync_block.h>
#include <gr_message.h>
#include <gr_msg_queue.h>


class rfid_reader_decoder;
typedef boost::shared_ptr<rfid_reader_decoder> rfid_reader_decoder_sptr;


rfid_reader_decoder_sptr 
rfid_make_reader_decoder (float us_per_sample, float tari);

class rfid_reader_decoder : public gr_sync_block 
{
 
 private:
  friend rfid_reader_decoder_sptr
  rfid_make_reader_decoder (float us_per_sample, float tari);
 
  enum {BEGIN, DELIM_FOUND, TARI_FOUND, RTCAL_FOUND, DATA};
  enum {READER_COMMAND, POWER_DOWN, START};
  float static const AVG_WIN = 750; // Window to average amplitude over, in us
  float static const THRESH_FRACTION = 0.9; //Percent of avg amplitude to detect edges
  int static const MAX_BITS = 256;  
  double static const MIN_AMP_THRESH = 0;     //Eventually, expose as user parameter

  double d_us_per_sample;
  int d_delim_width;          //Length of start delimiter, in samples
  int d_max_tari, d_min_tari, d_tari, d_rtcal, d_trcal; //Samples
  int d_state;                //Current state
  float * d_window_samples;   //Array to hold samples for averaging amplitude
  int d_window_length;        //Length of window
  int d_window_index;         //Index to oldest sample
  double d_avg_amp;           //Average amplitude over window
  double d_min_amp_thresh;    //To filter out nearby readers
  double d_thresh;            //Amplitude threshold for detecing edges
  int d_high_count, d_low_count, d_command_count, d_interarrival_count; //Sample counters
  bool neg_edge_found;        //True if found negative edge for bit
  int d_pivot;                // RTCal / 2. Determines data-0, data-1

  char d_bits[512];
  int d_len_bits;
  

 

 
  rfid_reader_decoder(float us_per_sample, float tari);
  void advance_decoder(int next_state);
  bool is_negative_edge(float sample);
  bool is_positive_edge(float sample);
  void log_event(int event, int lag_samples);
 
 public:
  ~rfid_reader_decoder();
  gr_msg_queue_sptr log_q;
  gr_msg_queue_sptr get_log() const {return log_q;}
  int work(int noutput_items, 
	   gr_vector_const_void_star &input_items,
	   gr_vector_void_star &output_items);

};

#endif

