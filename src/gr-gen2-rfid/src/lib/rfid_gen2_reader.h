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

#ifndef INCLUDED_RFID_GEN2_READER_H
#define INCLUDED_RFID_GEN2_READER_H

#include <gr_block.h>
#include <gr_message.h>
#include <gr_msg_queue.h>

#include "reader_vars.h"

class rfid_gen2_reader;
typedef boost::shared_ptr<rfid_gen2_reader> rfid_gen2_reader_sptr;

rfid_gen2_reader_sptr 
rfid_make_gen2_reader (int dec, int interp );

class rfid_gen2_reader : public gr_block
{
private:
  friend rfid_gen2_reader_sptr rfid_make_gen2_reader(int dec, int interp);
  
  rfid_gen2_reader(int dec, int interp);

  FILE * out_file;
  float *debug_out;

  gr_complex * d_one;
  gr_complex * d_zero;
  gr_complex * d_query_cmd;
  
  int d_one_len, d_zero_len, d_query_len;
  gr_complex * cw_buffer;
  gr_complex * zero_buffer;

  int d_dec, d_interp;
  float d_us_per_sample;
  int d_us_per_xmit;
  int d_tag_pw;
  int d_num_input_items;
  int DEFAULT_NUM_INPUT_ITEMS;

  int d_t1;
  int d_gate_cnt;
  
  int d_items_copied;
  float * d_sample_buffer;
  int d_skip_cnt;
  float d_last_score;

  int d_find_preamble_count;


  gr_complex * d_reader_framesync;
  int d_reader_framesync_len;
  
  gr_complex * d_qrep_msg;
  gr_complex * d_nak_msg;
  int d_qrep_msg_len, d_nak_msg_len;
  

  char * d_tag_bit_vector;

  gr_msg_queue_sptr out_q;
  gr_message_sptr tx_msg;
  int d_msg_count;


  gr_msg_queue_sptr log_q;
  enum {LOG_PWR_UP, LOG_QUERY, LOG_QREP, LOG_ACK, LOG_NAK, LOG_RN16, LOG_EPC, LOG_EMPTY, LOG_COLLISION, LOG_TIME_MISS, LOG_BAD_RN_MISS, LOG_ERROR, LOG_OKAY, LOG_SNR};  

  void forecast (int noutput_items, gr_vector_int &ninput_items_required); 
  void gen_query_cmd();
  void gen_qrep_cmd();
  void gen_nak_cmd();
  void reset_receive_state();
  void max_min(const float * buffer, int len, double * max, double * min, double* avg );
  int send_ack();
  int check_crc(char * bits, int num_bits);
  void log_msg(int message, char * text, int error);

public:
  ~rfid_gen2_reader(); 
  int general_work(int noutput_items, 
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);

  state * STATE_PTR;
  state STATE;
  
  

  gr_msg_queue_sptr get_log() const {return log_q;}

};

#endif
