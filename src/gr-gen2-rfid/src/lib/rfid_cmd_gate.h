/* -*- c++ -*- */
/* 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RFID_CMD_GATE_H
#define INCLUDED_RFID_CMD_GATE_H

#include <gr_block.h>
#include <gr_message.h>
#include <gr_msg_queue.h>

#include "reader_vars.h"

class rfid_cmd_gate;
typedef boost::shared_ptr<rfid_cmd_gate> rfid_cmd_gate_sptr;

rfid_cmd_gate_sptr 
rfid_make_cmd_gate (int dec, state * reader_state );

class rfid_cmd_gate : public gr_block
{
private:
  friend rfid_cmd_gate_sptr rfid_make_cmd_gate(int dec, state * reader_state);
  
  rfid_cmd_gate(int dec,  state * reader_state);

  FILE * out_file;
  float *debug_out;

  timeval time;
  int d_dec;
  float d_us_per_sample;
  int d_us_per_xmit;
  int d_tag_pw;
  int d_t1;
  int d_gate_cnt;

  int d_sig_detect_number;
  int d_sig_detect_counter;
  double d_sig_detect_max;
  double d_sig_detect_min;
  
  int d_sample_count;

  state * d_reader_state;

  float * sig_detect_buffer;

  void forecast (int noutput_items, gr_vector_int &ninput_items_required); 
  int max_min(const float * buffer, int len, double * max, double * min, double* avg, double * std_dev );


public:
  ~rfid_cmd_gate(); 
  int general_work(int noutput_items, 
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);
  
};

#endif
