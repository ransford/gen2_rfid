/* -*- c++ -*- */


#ifndef INCLUDED_RFID_TAG_DECODER_F_H
#define INCLUDED_RFID_TAG_DECODER_F_H

#include <gr_block.h>
#include "rfid_global_vars.h"
#include <gr_message.h>
#include <gr_msg_queue.h>

class rfid_tag_decoder_f;
typedef boost::shared_ptr<rfid_tag_decoder_f> rfid_tag_decoder_f_sptr;


rfid_tag_decoder_f_sptr 
rfid_make_tag_decoder_f ();

class rfid_tag_decoder_f : public gr_block 
{
 
 private:
  friend rfid_tag_decoder_f_sptr
  rfid_make_tag_decoder_f ();
 
  

  //std::vector<float>	d_preamble_cor_vec;
  int                   d_preamble_offset;
  int			d_one_vlen;
  char *                d_tag_bit_vector;
  int                   d_skip_count;
  int                   d_samples_since_reset;
  int                   d_preamble_miss_threshold;
  float                 d_last_score;
  int                   d_samples_processed;
 
  gr_msg_queue_sptr	d_ctrl_out;  //Pipe control messages to reader block.

  rfid_tag_decoder_f();
  void forecast (int noutput_items, gr_vector_int &ninput_items_required); 
 
 public:
  ~rfid_tag_decoder_f();
  
  int general_work(int noutput_items, 
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);

  void	set_ctrl_out(const gr_msg_queue_sptr msgq) { d_ctrl_out = msgq; }

};

#endif

