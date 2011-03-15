/* -*- c++ -*- */

#ifndef INCLUDED_RFID_READER_F_H
#define INCLUDED_RFID_READER_F_H

#include <gr_block.h>
#include <gr_message.h>
#include <gr_msg_queue.h>
#include "rfid_global_vars.h"

class rfid_reader_f;
typedef boost::shared_ptr<rfid_reader_f> rfid_reader_f_sptr;

rfid_reader_f_sptr
rfid_make_reader_f (int sample_rate);

class rfid_reader_f : public gr_block {
  friend rfid_reader_f_sptr 
  rfid_make_reader_f (int sample_rate);
  

  
  int d_sample_rate;

  float * d_one;
  float * d_zero;
  float * d_query_cmd;
  float * d_reader_framesync;
  float * d_qrep;
  float * d_nak;
  float * d_req_rn;
  float * d_read;
  int d_one_len, d_zero_len, d_query_len, d_reader_framesync_len, d_qrep_len, d_nak_len, d_req_rn_len, d_read_len;

  float * cw_buffer;
  float * zero_buffer;
  float d_us_per_xmit;
 
  gr_message_sptr tx_msg;
  gr_message_sptr d_ctrl_msg;
  gr_msg_queue_sptr out_q;
  gr_msg_queue_sptr d_ctrl_q;
  int d_msg_count;
  
  float collision_threshold;
  float Q_fp; 
  int d_num_empty_rounds; 
  int d_tags_read_in_cycle, d_slots_occupied;

  char last_handle[16];
  int which_handle ;

  //Gen 2 parameters
  char d_CMD[5];
  char d_DR[2];
  char d_M[3];
  char d_tr_ext[2];
  char d_sel[3];
  char d_session[3];
  char d_target[2];
  char d_Q[5];
  char d_CRC[6]; 


  gr_msg_queue_sptr log_q;
  enum {LOG_START_CYCLE, LOG_QUERY, LOG_ACK, LOG_QREP, LOG_NAK, LOG_REQ_RN, LOG_READ, LOG_RN16, LOG_EPC, LOG_HANDLE, LOG_DATA, LOG_EMPTY, LOG_COLLISION, LOG_OKAY, LOG_ERROR};

  public:
  
  int general_work(int noutput_items, 
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);
  
  gr_msg_queue_sptr    ctrl_q() const {return d_ctrl_q;}
  gr_msg_queue_sptr get_log() const {return log_q;}

  private: 
  rfid_reader_f (int sample_rate);
  void gen_query_cmd();
  void gen_qrep_cmd();
  void gen_nak_cmd();
  void gen_req_rn_cmd();
  void gen_read_cmd(char * handle);

  void send_ack();
  void start_cycle();
  void send_query();
  void send_qrep();
  void send_nak();
  void send_req_rn();
  void send_read();

  bool send_another_query();
  void update_q(int slot_occupancy);

  int check_crc(char * bits, int num_bits);
  void set_num_samples_to_ungate();

  void forecast (int noutput_items, gr_vector_int &ninput_items_required); 
  void log_msg(int message, char * text, int error);
  
  
};




#endif
