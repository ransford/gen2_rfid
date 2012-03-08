/* -*- c++ -*- */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include <rfid_reader_f.h>
#include <gr_io_signature.h>
#include <cstdlib>
#include <cstdio>
#include <signal.h>
#include <sys/time.h>
#include <string.h>


rfid_reader_f_sptr
rfid_make_reader_f (int sample_rate)
{
  return rfid_reader_f_sptr(new rfid_reader_f (sample_rate));
} 




rfid_reader_f::rfid_reader_f (int sample_rate)
  : gr_block ("reader_f",
		   gr_make_io_signature (1, 1, sizeof(float)),
		   gr_make_io_signature (1, 1, sizeof (float))),
    d_sample_rate (sample_rate), d_ctrl_q(gr_make_msg_queue(100))
{
  init_global_reader_state();

  log_q = gr_make_msg_queue(500000);//Holds log messages, drained by python app
  out_q = gr_make_msg_queue(1000);  //Holds messages for transmission at the end of work
  
  d_us_per_xmit = 1000000 / (float)sample_rate;
  


  //Create data-1 array
  d_one_len = ((pulse_width / d_us_per_xmit) * 4);
  d_one = (float *)malloc(d_one_len * sizeof(float));
  for(int i = 0 ; i < (pulse_width / d_us_per_xmit) * 3; i++){
    d_one[i] = 1;
   }
  for(int i = (pulse_width / d_us_per_xmit) * 3; i < ((pulse_width / d_us_per_xmit) * 4); i++){
    d_one[i] = 0;
  }

  //Create data-0 array
  d_zero_len = ((pulse_width / d_us_per_xmit) * 2);
  d_zero = (float *)malloc(d_zero_len * sizeof(float));
  for(int i = 0 ; i < (pulse_width / d_us_per_xmit); i++){
    d_zero[i] = 1;
  }
  for(int i = (pulse_width / d_us_per_xmit); i < ((pulse_width / d_us_per_xmit) * 2); i++){
    d_zero[i] = 0;
  }

   //Set up cw and zero buffers 
  cw_buffer = (float *)malloc(8196 * sizeof(float));
  for(int i = 0; i < 8196; i++){
    cw_buffer[i] = 1;
  }
  zero_buffer = (float *)malloc(8196 * sizeof(float));
  for(int i = 0; i < 8196; i++){
   zero_buffer[i] = 0;
  }

  //Set up reader framesync
  int len = 0;
  float tmp_framesync[8196];
  //Delim
  for(int i = 0; i < delim_width / d_us_per_xmit; i++){ //Delim
    tmp_framesync[len++] = 0;
  }
  //Data-0
  memcpy((float *)&tmp_framesync[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  //RTCal
  for(int i = 0; i < (rtcal_width  / d_us_per_xmit) - (pulse_width / d_us_per_xmit); i++){
    tmp_framesync[len++] = 1;
  }
  //PW
  for(int i = 0; i < (pulse_width / d_us_per_xmit); i++){
    tmp_framesync[len++] = 0;
  }
  d_reader_framesync_len = len;
  d_reader_framesync = (float *)malloc(d_reader_framesync_len * sizeof(float));
  memcpy(d_reader_framesync, tmp_framesync, d_reader_framesync_len * sizeof(float));

 

   char CMD[5] = "1000";
   memcpy(d_CMD, CMD, 5);
   char DR[2] = "0";
   memcpy(d_DR, DR, 2);
   char M[3] = "10";
   memcpy(d_M, M, 3);
   char tr_ext[2] = "1";
   memcpy(d_tr_ext, tr_ext, 2);
   char sel[3] = "00";
   memcpy(d_sel, sel, 3);
   char session[3] = "00";
   memcpy(d_session, session, 3);
   char target[2] = "0";
   memcpy(d_target, target, 2);
   char Q[5] = "0000";
   memcpy(d_Q, Q, 5);



   gen_query_cmd();
   gen_qrep_cmd();
   gen_nak_cmd();
   gen_req_rn_cmd();
   //   gen_read_cmd();


   global_reader_state->num_rounds = READER_NUM_ROUNDS;
   
   global_reader_state->num_cycles = READER_NUM_CYCLES; 
   global_reader_state->cur_cycle = 0;

   Q_fp = INIT_QFP;
   memcpy(d_Q, q_strings[(int)Q_fp], 5);
   collision_threshold = COLLISION_THRESHOLD;
   d_tags_read_in_cycle = 0;
   d_msg_count = 0;

   if(READ_DATA){
     printf("Warning: Data read functions are stubs.\n");
   }


 }



 int
 rfid_reader_f::general_work(int noutput_items,
					 gr_vector_int &ninput_items,
					 gr_vector_const_void_star &input_items,
					 gr_vector_void_star &output_items)
 {
   const float * in = (const float *)input_items[0];
   float *out = (float *) output_items[0];
   int written = 0;
   int consumed = ninput_items[0];




   d_ctrl_msg = d_ctrl_q->delete_head_nowait();
   if(!d_ctrl_msg){
     return 0;
   }

   int ctrl = d_ctrl_msg->msg()[0];

   global_reader_state->command_gate_status = GATE_RESET;
   if(ctrl == TIMER_FIRED){
    
     start_cycle();
     send_query();
    
     
   }
   if(ctrl == BITS_DECODED){
    
     if(global_reader_state->last_cmd_sent == QUERY || global_reader_state->last_cmd_sent == QREP){
        
       
       //Tag responses must end in 1. Don't bother sending the ACK
       if (global_reader_state->tag_bit_vector[global_reader_state->num_bits_to_decode - 1] == '0'){
	 
	 global_reader_state->tag_bit_vector[global_reader_state->num_bits_to_decode - 1] = '\0';
	 char tmp[500];
	 char tmp2[500];
	 strcpy(tmp, global_reader_state->tag_bit_vector);
	 sprintf(tmp2, ",%f\n", global_reader_state->std_dev_signal / global_reader_state->std_dev_noise);
	 strcat(tmp, tmp2);
	 
	 update_q(2);  //We assume this is a collision, because collided preamble correlate often.

	 log_msg(LOG_RN16, tmp, LOG_ERROR);
	 if(global_reader_state->cur_slot < global_reader_state->num_slots){
	   send_qrep();
	 }
	 else if(send_another_query()){
	   send_query();
	 }
	 
       }
       //RN16 decoded. ACK it
       else{

	 global_reader_state->tag_bit_vector[global_reader_state->num_bits_to_decode - 1] = '\0';
	 char tmp[500];
	 char tmp2[500];
	 strcpy(tmp, global_reader_state->tag_bit_vector);
	 sprintf(tmp2, ",%f\n", global_reader_state->std_dev_signal / global_reader_state->std_dev_noise);
	 strcat(tmp, tmp2);
	 
	 log_msg(LOG_RN16, tmp, LOG_OKAY);
	 send_ack();
       }

     }
     //Decoded EPC
     else if(global_reader_state->last_cmd_sent == ACK){

       update_q(1);  //We at least found a preamble, so there must be a tag there. 

       global_reader_state->tag_bit_vector[global_reader_state->num_bits_to_decode - 1] = '\0';
       char tmp[500];
       char tmp2[500];
       strcpy(tmp, global_reader_state->tag_bit_vector);
       sprintf(tmp2, ",%f\n", global_reader_state->std_dev_signal / global_reader_state->std_dev_noise);
       strcat(tmp, tmp2);
       int pass = check_crc(global_reader_state->tag_bit_vector, global_reader_state->num_bits_to_decode);
       
       //No Errors, send Qrep
       if(pass == 1){

	 d_tags_read_in_cycle++;
	 log_msg(LOG_EPC, tmp, LOG_OKAY);
	 if(READ_DATA){  //Flag for sending REQ_RN. 
	   send_req_rn();
	 }
	 else{
	   send_qrep();  //Always send QRep so tag knows it was read
	 }
	 
       }
       else{

	 log_msg(LOG_EPC, tmp, LOG_ERROR);
	 send_nak();
	 //If there are more slots, send QREP. Otherwise, send another query. 
	 if(global_reader_state->cur_slot < global_reader_state->num_slots){

	   send_qrep();
       
	 }
	 else if(send_another_query()){
	   send_query();
	 }
	 
       }
       
       
     }
     //Decoded handle, send READ command
     else if(global_reader_state->last_cmd_sent == REQ_RN){
       
       global_reader_state->tag_bit_vector[global_reader_state->num_bits_to_decode - 1] = '\0';
       char tmp[500];
       char tmp2[500];
       strcpy(tmp, global_reader_state->tag_bit_vector);
       sprintf(tmp2, ",%f\n", global_reader_state->std_dev_signal / global_reader_state->std_dev_noise);
       strcat(tmp, tmp2);    
       log_msg(LOG_HANDLE, tmp, LOG_OKAY);
       char * handle;
       memcpy(last_handle, global_reader_state->tag_bit_vector, 16 * sizeof(char));
       handle = last_handle;
       gen_read_cmd(handle);
       send_read();
       

     }
     //Decoded data. DOES NOT CHECK CRC.
     else if(global_reader_state->last_cmd_sent == READ){
      
       global_reader_state->tag_bit_vector[global_reader_state->num_bits_to_decode - 1] = '\0';
       char tmp[500];
       char tmp2[500];
       strcpy(tmp, global_reader_state->tag_bit_vector);
       sprintf(tmp2, ",%f\n", global_reader_state->std_dev_signal / global_reader_state->std_dev_noise);
       strcat(tmp, tmp2);    
       log_msg(LOG_DATA, tmp, LOG_OKAY);


       //Send another qrep or query
       if(global_reader_state->cur_slot < global_reader_state->num_slots){
	 send_qrep();
	 
       }
       else if (send_another_query()){
	 
	 send_query();
       }

     }
   }
   
   if(ctrl == NO_PREAMBLE){
     char tmp[500];
     
   

     if(global_reader_state->std_dev_signal / global_reader_state->std_dev_noise > collision_threshold){//Detected a signal
       if(global_reader_state->last_cmd_sent == ACK){  //Maybe we missed the preamble. But there was a signal
	 update_q(1);  
       }
       else if(global_reader_state->last_cmd_sent == QUERY || global_reader_state->last_cmd_sent == QREP){// Collision
	 update_q(2); 
       }
     }
     else{
       if(global_reader_state->last_cmd_sent == ACK){  //We probably correlated on a collision. Maybe we got the bits wrong, but we err this direction.
	 update_q(2);  
       }
       else if(global_reader_state->last_cmd_sent == QUERY || global_reader_state->last_cmd_sent == QREP){
	 update_q(0);
       }

     }

     sprintf(tmp, "%f\n", global_reader_state->std_dev_signal / global_reader_state->std_dev_noise);
     log_msg(LOG_EMPTY, tmp, LOG_OKAY);


     if(global_reader_state->cur_slot < global_reader_state->num_slots){

       send_qrep();
       
     }
     else if (send_another_query()){

       send_query();
     }
   }





   //Transmit commands, if there are any
   if(!tx_msg){
     tx_msg = out_q->delete_head_nowait();
   }

   //Transmit noutout_items worth of command, if there is more reschedule this block. 
   while(tx_msg){


     int mm = std::min((long unsigned int)(tx_msg->length() - d_msg_count) / sizeof(float), (long unsigned int) noutput_items - written);

     memcpy(&out[written], &tx_msg->msg()[d_msg_count], mm * sizeof(float));
     written += mm;
     d_msg_count += mm * sizeof(float);
     if(d_msg_count == (int)tx_msg->length()){
       tx_msg.reset();
       d_msg_count = 0;
       tx_msg = out_q->delete_head_nowait(); 
     

     }

     if(written == noutput_items && tx_msg){ 
       break;
     }



   }

   consume_each(consumed);
   return written;
 }

void 
rfid_reader_f::start_cycle(){

  Q_fp = INIT_QFP;
  int num_pkts = 0;
  global_reader_state->cur_cycle++;
  if(global_reader_state->cur_cycle %10 == 0){
    printf("Starting cycle %d\n", global_reader_state->cur_cycle);
  }
  d_tags_read_in_cycle = 0;

  log_msg(LOG_START_CYCLE, NULL, LOG_OKAY);

  global_reader_state->cur_round = 0;
  
  int min_pwr_dwn = 1000;  //Spec says 1000.

  num_pkts = ((min_pwr_dwn / d_us_per_xmit) / usrp_pkt_size); //Round to the nearest usrp_pkt_size samples
  
  for(int i = 0; i < num_pkts; i++){
    gr_message_sptr pwr_dwn_msg = gr_make_message(0,
						  sizeof(float),
						  0,
						  (usrp_pkt_size) * sizeof(float));
    memcpy(pwr_dwn_msg->msg(), zero_buffer, (usrp_pkt_size) * sizeof(float));
    out_q->insert_tail(pwr_dwn_msg);
  }
  
  
}


 void rfid_reader_f::send_query(){
   //Generate query to update Q etc.
   gen_query_cmd();
   
   //Sending a query.
   global_reader_state->last_cmd_sent = QUERY;
   global_reader_state->cur_slot = 1;
   global_reader_state->cur_round++;

   global_reader_state->num_bits_decoded = 0;
   global_reader_state->num_bits_to_decode = no_RN16_bits;
   set_num_samples_to_ungate();

   int num_pkts = 0;



   log_msg(LOG_QUERY, NULL, LOG_OKAY);
   
   int min_cw = 1500;
   
  
   num_pkts = ((min_cw / d_us_per_xmit) / usrp_pkt_size);
   
   for(int i = 0; i < num_pkts; i++){
     gr_message_sptr cw_msg = gr_make_message(0,
					      sizeof(float),
					      0,
					      (usrp_pkt_size) * sizeof(float));
     memcpy(cw_msg->msg(), cw_buffer, usrp_pkt_size * sizeof(float));
     out_q->insert_tail(cw_msg);
   }
   gr_message_sptr query_msg = gr_make_message(0,
					       sizeof(float),
					       0,
					       (d_query_len) * sizeof(float));
   memcpy(query_msg->msg(), d_query_cmd, d_query_len * sizeof(float));
   out_q->insert_tail(query_msg);

   
   int tail_cw;
   if(TRANSMIT_CW){
     tail_cw =  global_reader_state->T1_value + (global_reader_state->num_samples_to_ungate * global_reader_state->us_per_rcv);
   }
   else{
     tail_cw = 250;
   }



   for(int i = 0; i < ((tail_cw / d_us_per_xmit) / usrp_pkt_size) ; i++){

     gr_message_sptr cw_msg = gr_make_message(0,
					      sizeof(float),
					      0,
					      (usrp_pkt_size) * sizeof(float));
     memcpy(cw_msg->msg(), cw_buffer, usrp_pkt_size * sizeof(float));
     out_q->insert_tail(cw_msg);
   }



   // for(int i = 0; i < 1; i++){
   //   gr_message_sptr pwr_dwn_msg = gr_make_message(0,
   // 						  sizeof(float),
   // 						  0,
   // 						  (usrp_pkt_size) * sizeof(float));
   //   memcpy(pwr_dwn_msg->msg(), zero_buffer, (usrp_pkt_size) * sizeof(float));

   //   out_q->insert_tail(pwr_dwn_msg);
   // }




   char q_str[1000];
   sprintf(q_str, "CMD: %s DR: %s M: %s TR: %s SEL: %s Sess:%s Targ:%s Q: %s CRC: %s", d_CMD, d_DR, d_M, d_tr_ext, d_sel, d_session, d_target, d_Q, d_CRC);



 


 }

 void 
 rfid_reader_f::send_qrep(){

   log_msg(LOG_QREP, NULL, LOG_OKAY);
   
   global_reader_state->cur_slot++;
   global_reader_state->last_cmd_sent = QREP;

   // gr_message_sptr cw_msg1 = gr_make_message(0,
   // 					    sizeof(float),
   // 					    0,
   // 					    (usrp_pkt_size) * sizeof(float));
   // memcpy(cw_msg1->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   // out_q->insert_tail(cw_msg1);

   gr_message_sptr qrep_msg = gr_make_message(0,
					      sizeof(float),
					      0,
					      d_qrep_len  * sizeof(float));
   memcpy(qrep_msg->msg(), d_qrep, d_qrep_len * sizeof(float));
   out_q->insert_tail(qrep_msg);

   global_reader_state->num_bits_to_decode = no_RN16_bits;
   global_reader_state->num_bits_decoded = 0;
   set_num_samples_to_ungate();

   int tail_cw;
   if(TRANSMIT_CW){
     tail_cw =  global_reader_state->T1_value + (global_reader_state->num_samples_to_ungate * global_reader_state->us_per_rcv);
   }
   else{
     tail_cw = 250;
   }


   
   int qrep_pad_len = 26;  //This is a hack. It is the right padding for the Qrep, but is hardcoded here.
   for(int i = 0; i < (((tail_cw / d_us_per_xmit) - qrep_pad_len)  / usrp_pkt_size) ; i++){

     gr_message_sptr cw_msg = gr_make_message(0,
					      sizeof(float),
					      0,
					      (usrp_pkt_size) * sizeof(float));
     memcpy(cw_msg->msg(), cw_buffer, usrp_pkt_size * sizeof(float));
     out_q->insert_tail(cw_msg);
   }

   //   for(int i = 0; i < 1; i++){
   //   gr_message_sptr pwr_dwn_msg = gr_make_message(0,
   // 						  sizeof(float),
   // 						  0,
   // 						  (usrp_pkt_size) * sizeof(float));
   //   memcpy(pwr_dwn_msg->msg(), zero_buffer, (usrp_pkt_size) * sizeof(float));

   //   out_q->insert_tail(pwr_dwn_msg);
   // }

 }

void
rfid_reader_f::send_req_rn(){
   log_msg(LOG_REQ_RN, NULL, LOG_OKAY);

   global_reader_state->last_cmd_sent = REQ_RN;

   gr_message_sptr cw_msg1 = gr_make_message(0,
					    sizeof(float),
					    0,
					    (usrp_pkt_size) * sizeof(float));
   memcpy(cw_msg1->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   out_q->insert_tail(cw_msg1);

   gr_message_sptr req_rn_msg = gr_make_message(0,
					     sizeof(float),
					     0,
					     d_req_rn_len * sizeof(float));
   memcpy(req_rn_msg->msg(), d_req_rn, d_req_rn_len * sizeof(float));
   out_q->insert_tail(req_rn_msg);

   gr_message_sptr cw_msg = gr_make_message(0,
					    sizeof(float),
					    0,
					    (usrp_pkt_size) * sizeof(float));
   memcpy(cw_msg->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   out_q->insert_tail(cw_msg);

   global_reader_state->num_bits_to_decode = no_RN16_bits;
   global_reader_state->num_bits_decoded = 0;
   set_num_samples_to_ungate();

}

void
rfid_reader_f::send_read(){
   log_msg(LOG_READ, NULL, LOG_OKAY);

   global_reader_state->last_cmd_sent = READ;

   gr_message_sptr cw_msg1 = gr_make_message(0,
					    sizeof(float),
					    0,
					    (usrp_pkt_size) * sizeof(float));
   memcpy(cw_msg1->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   out_q->insert_tail(cw_msg1);

   cw_msg1 = gr_make_message(0,
					    sizeof(float),
					    0,
					    (usrp_pkt_size) * sizeof(float));
   memcpy(cw_msg1->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   out_q->insert_tail(cw_msg1);


   cw_msg1 = gr_make_message(0,
					    sizeof(float),
					    0,
					    (usrp_pkt_size) * sizeof(float));
   memcpy(cw_msg1->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   out_q->insert_tail(cw_msg1);


   gr_message_sptr read_msg = gr_make_message(0,
					     sizeof(float),
					     0,
					     d_read_len * sizeof(float));
   memcpy(read_msg->msg(), d_read, d_read_len * sizeof(float));
   out_q->insert_tail(read_msg);

   gr_message_sptr cw_msg = gr_make_message(0,
					    sizeof(float),
					    0,
					    (usrp_pkt_size) * sizeof(float));
   memcpy(cw_msg->msg(), cw_buffer, (usrp_pkt_size) * sizeof(float));
   out_q->insert_tail(cw_msg);

   global_reader_state->num_bits_to_decode = no_DATA_bits;
   global_reader_state->num_bits_decoded = 0;
   set_num_samples_to_ungate();

}


 void
 rfid_reader_f::send_nak(){
   global_reader_state->nak_sent = true;
   log_msg(LOG_NAK, NULL, LOG_OKAY);

   global_reader_state->last_cmd_sent = NAK;
   gr_message_sptr nak_msg = gr_make_message(0,
					     sizeof(float),
					     0,
					     d_nak_len * sizeof(float));
   memcpy(nak_msg->msg(), d_nak, d_nak_len * sizeof(float));
   out_q->insert_tail(nak_msg);

   int spacing = 128;  
   int num_pkts;
   num_pkts = ((spacing / d_us_per_xmit) / usrp_pkt_size); //Round to the nearest usrp_pkt_size samples

   for(int i = 0; i < num_pkts; i++){
     gr_message_sptr cw_msg = gr_make_message(0,
					      sizeof(float),
					      0,
					      (usrp_pkt_size) * sizeof(float));
     memcpy(cw_msg->msg(), cw_buffer, usrp_pkt_size * sizeof(float));
     out_q->insert_tail(cw_msg);
   }


   
 }


 void 
 rfid_reader_f::send_ack(){
   float ack_msg[8196];

   log_msg(LOG_ACK, NULL, LOG_OKAY);
   
   global_reader_state->last_cmd_sent = ACK;




   int pad = 0;
   int len = 0;

   //Add header
   memcpy(ack_msg, d_reader_framesync, d_reader_framesync_len * sizeof(float));
   memcpy((float *)&ack_msg[d_reader_framesync_len], d_zero, d_zero_len * sizeof(float));
   memcpy((float *)&ack_msg[d_reader_framesync_len + d_zero_len], d_one, d_one_len * sizeof(float));

   len = d_reader_framesync_len + d_zero_len + d_one_len;

   for(int i = 0; i < global_reader_state->num_bits_to_decode - 1; i++){
     if(global_reader_state->tag_bit_vector[i] == '0'){
       memcpy((float *)&ack_msg[len], d_zero, d_zero_len * sizeof(float));
       len += d_zero_len;
     }
     else{
       memcpy((float *)&ack_msg[len], d_one, d_one_len * sizeof(float));
       len += d_one_len; 
     }

   }

   pad = usrp_pkt_size - (len % usrp_pkt_size);




   float tmp[pad+len];
   memcpy(tmp, ack_msg, len * sizeof(float));
   memcpy(&tmp[len], cw_buffer, pad *sizeof(float));


   gr_message_sptr new_tx_msg = gr_make_message(0,
						sizeof(float),
						0,
						(len + pad) * sizeof(float));

   memcpy(new_tx_msg->msg(), tmp, (pad + len) *sizeof(float));
   out_q->insert_tail(new_tx_msg);

   
   global_reader_state->num_bits_to_decode = no_EPC_bits;
   global_reader_state->num_bits_decoded = 0;
   set_num_samples_to_ungate();

   int tail_cw;
   if(TRANSMIT_CW){
     tail_cw =  global_reader_state->T1_value + (global_reader_state->num_samples_to_ungate * global_reader_state->us_per_rcv);
   }
   else{
     tail_cw = 250;
   }

   // printf("ACKTail:%d\n",  tail_cw);

   for(int i = 0; i < (((tail_cw / d_us_per_xmit) - pad) / usrp_pkt_size) ; i++){

     gr_message_sptr cw_msg = gr_make_message(0,
   					      sizeof(float),
   					      0,
   					      (usrp_pkt_size) * sizeof(float));
     memcpy(cw_msg->msg(), cw_buffer, usrp_pkt_size * sizeof(float));
     out_q->insert_tail(cw_msg);
   }



   // for(int i = 0; i < 1; i++){
   //   gr_message_sptr pwr_dwn_msg = gr_make_message(0,
   // 						  sizeof(float),
   // 						  0,
   // 						  (usrp_pkt_size) * sizeof(float));
   //   memcpy(pwr_dwn_msg->msg(), zero_buffer, (usrp_pkt_size) * sizeof(float));

   //   out_q->insert_tail(pwr_dwn_msg);
   // }



 }


 void
 rfid_reader_f::gen_query_cmd(){

   int len_query = 22;

   char * q_bits; 


   if(CHANGE_Q){
     if(OPTIMAL_Q){
       if(NUM_TAGS - d_tags_read_in_cycle > 0){
	 memcpy(d_Q, opt_q_strings[NUM_TAGS - d_tags_read_in_cycle - 1 ], 5);
	 //printf("%d left, Q = :%s\n", NUM_TAGS - d_tags_read_in_cycle,opt_q_strings[NUM_TAGS - d_tags_read_in_cycle - 1]); 
       }
       else{
	  int Q = (int)Q_fp;
	  
	  if(Q > 8){  //I'm lazy and only filled in the table to 8
	    printf("Q > 8. Not changing. Q:%d\n", Q);
	    printf("NUmTags: %d Read: %d\n", NUM_TAGS, d_tags_read_in_cycle);
	  }
	  else{
	    //printf("WTF\n");
	    //printf("NUmTags: %d Read: %d\n", NUM_TAGS, d_tags_read_in_cycle);
	    //assume zero tags left.
	    memcpy(d_Q, q_strings[0], 5);
	  }
       }
       
       
     }
     else{

       int Q = (int)Q_fp;
       
       if(Q > 8){  //I'm lazy and only filled in the table to 8
	 printf("Q > 8. Not changing. Q:%d\n", Q);
       }
       else{
	 memcpy(d_Q, q_strings[Q], 5);
       }
     }
     

   }

   q_bits = (char * )malloc(len_query);
   q_bits[0] = '\0';
   strcat(q_bits, d_CMD);
   strcat(q_bits, d_DR);
   strcat(q_bits, d_M);
   strcat(q_bits, d_tr_ext);
   strcat(q_bits, d_sel);
   strcat(q_bits, d_session);
   strcat(q_bits, d_target);
   strcat(q_bits, d_Q);


   //Calculate number of slots
   int q = 0;
   int factor = 8;
   for(int i = 0; i < 4; i++){
     if(d_Q[i] == '1'){
       q += factor;
     }
     factor = factor / 2;
   }

   global_reader_state->num_slots = (int)pow(2, q);

   if(d_tr_ext[0] == '0'){
     global_reader_state->num_bits_in_preamble = 10; // 4 LF + 6
   }
   else{
     global_reader_state->num_bits_in_preamble = 22; // 16 LF + 6
   }


   //Calculate CRC, add to end of message
   char crc[] = {'1','0','0','1','0'};
   for(int i = 0; i < 17; i++){
     char tmp[] = {'0','0','0','0','0'};
     tmp[4] = crc[3];
     if(crc[4] == '1'){
       if (q_bits[i] == '1'){
	 tmp[0] = '0';
	 tmp[1] = crc[0];
	 tmp[2] = crc[1];
	 tmp[3] = crc[2];
       }
       else{
	 tmp[0] = '1';
	 tmp[1] = crc[0];
	 tmp[2] = crc[1];
	 if(crc[2] == '1'){
	   tmp[3] = '0';
	 }
	 else{
	   tmp[3] = '1';
	 }
       }
     }
     else{
       if (q_bits[i] == '1'){
	 tmp[0] = '1';
	 tmp[1] = crc[0];
	 tmp[2] = crc[1];
	 if(crc[2] == '1'){
	   tmp[3] = '0';
	 }
	 else{
	   tmp[3] = '1';
	 }
       }
       else{
	 tmp[0] = '0';
	 tmp[1] = crc[0];
	 tmp[2] = crc[1];
	 tmp[3] = crc[2];
       }
     }
     memcpy(crc, tmp, 5);
   }

   int cnt = 0;
   for(int i = 4; i > -1; i--){
     q_bits[17 + cnt] = crc[i];
     d_CRC[cnt] = crc[i];
     cnt++;
   }

   //Setup d_query_cmd
   int num_0 = 0;
   int num_1 = 0;

   for(int i = 0; i < len_query; i++){
     if(q_bits[i] == '1'){
       num_1++;
     }
     else{
       num_0++;
     }
   }

   d_query_len = ((delim_width + pulse_width + pulse_width + rtcal_width + trcal_width) / d_us_per_xmit) + (num_1 * d_one_len) + (num_0 * d_zero_len);



   int pad = usrp_pkt_size - (d_query_len % usrp_pkt_size);

   d_query_len += pad;
   d_query_cmd = (float * )malloc((d_query_len) * sizeof(float)); 

   int j = 0;
   //Pad for USB buffer size
   for(int i = 0; i < pad; i++){
     d_query_cmd[j++] = 1;
   } 

   memcpy(&d_query_cmd[j], d_reader_framesync, d_reader_framesync_len * sizeof(float));
   j += d_reader_framesync_len;

   //TRCal
   for(int i = 0; i < (trcal_width  / d_us_per_xmit) - (pulse_width / d_us_per_xmit); i++){

     d_query_cmd[j++] = 1;
   }

   //PW
   for(int i = 0; i < (pulse_width / d_us_per_xmit); i++){

     d_query_cmd[j++] = 0;
   }

   for(int i = 0; i < len_query; i++){

     if(q_bits[i] == '0'){
       memcpy((float *)&d_query_cmd[j], d_zero, d_zero_len * sizeof(float));
       j += d_zero_len;
     }
     else if(q_bits[i] == '1'){
       memcpy((float *)&d_query_cmd[j], d_one, d_one_len * sizeof(float));
       j += d_one_len;
     }
   }

  
   //if(strcmp(global_reader_state->M, "00") == 0){
   // global_reader_state->tag_preamble_cor_vec = fm0_preamble;
   // global_reader_state->tag_preamble_cor_vec_len = len_fm0_preamble;
   // global_reader_state->tag_one_cor_vec = fm0_one_vec;
   // global_reader_state->tag_one_cor_vec_len = len_fm0_one;
   //}
   if(strcmp(d_M, "11") == 0){
     global_reader_state->tag_preamble_cor_vec = m8_preamble_vec;
     global_reader_state->tag_preamble_cor_vec_len = m8_preamble_len;
     global_reader_state->tag_one_cor_vec = m8_data_one_vec;
     global_reader_state->tag_one_cor_vec_len = m8_one_len;
     global_reader_state->num_pulses_per_bit = 16;
     global_reader_state->num_samples_per_bit = global_reader_state->num_pulses_per_bit * global_reader_state->num_samples_per_pulse; 

   }
   if(strcmp(d_M, "10") == 0){
     global_reader_state->tag_preamble_cor_vec = m4_preamble_vec;
     global_reader_state->tag_preamble_cor_vec_len = m4_preamble_len;
     global_reader_state->tag_one_cor_vec = m4_data_one_vec;
     global_reader_state->tag_one_cor_vec_len = m4_one_len;
     global_reader_state->num_pulses_per_bit = 8;
     global_reader_state->num_samples_per_bit = global_reader_state->num_pulses_per_bit * global_reader_state->num_samples_per_pulse; 

   }
   if(strcmp(d_M, "01") == 0){
     global_reader_state->tag_preamble_cor_vec = m2_preamble_vec;
     global_reader_state->tag_preamble_cor_vec_len = m2_preamble_len;
     global_reader_state->tag_one_cor_vec = m2_data_one_vec;
     global_reader_state->tag_one_cor_vec_len = m2_one_len;
     global_reader_state->num_pulses_per_bit = 4;
     global_reader_state->num_samples_per_bit = global_reader_state->num_pulses_per_bit * global_reader_state->num_samples_per_pulse; 
   }

 }

 void
 rfid_reader_f::gen_nak_cmd(){
  //Set up NAK message
   int len = 0;
   float tmp_nak[8196];
   memcpy(tmp_nak, d_reader_framesync, d_reader_framesync_len * sizeof(float));
   len += d_reader_framesync_len;
   memcpy(&tmp_nak[len], d_one, d_one_len * sizeof(float));
   len += d_one_len;
   memcpy(&tmp_nak[len], d_one, d_one_len * sizeof(float));
   len += d_one_len;
   memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;
   memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;
   memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;
   memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;
   memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;
   memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;

   int pad = usrp_pkt_size - (len % usrp_pkt_size);
   d_nak = (float *)malloc((len + pad) * sizeof(float));
   memcpy(d_nak, tmp_nak, len * sizeof(float));
   memcpy(&d_nak[len], cw_buffer, pad * sizeof(float));

   d_nak_len = len + pad;

 }

//This is a dummy function. It works for the WISP, as it just checks opcode and length.
void 
rfid_reader_f::gen_req_rn_cmd(){
  int len = 0;
  float tmp_req_rn[8196];
  memcpy(tmp_req_rn, d_reader_framesync, d_reader_framesync_len * sizeof(float));
  len += d_reader_framesync_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_req_rn[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_req_rn[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_req_rn[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_req_rn[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;

  //Dummy bits. WISP ignores these bits anyway.
   memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;

 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_req_rn[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;

   int pad = (usrp_pkt_size ) - (len % usrp_pkt_size);
   d_req_rn = (float *)malloc((len + pad) * sizeof(float));
   memcpy(d_req_rn, tmp_req_rn, len * sizeof(float));
   memcpy(&d_req_rn[len], cw_buffer, pad * sizeof(float));

   d_req_rn_len = len + pad;

}


//This is a dummy function. It works for the WISP, as it just checks opcode and length.
void
rfid_reader_f::gen_read_cmd(char * handle){

  int len = 0;
  float tmp_read[8196];
  memcpy(tmp_read, d_reader_framesync, d_reader_framesync_len * sizeof(float));
  len += d_reader_framesync_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_read[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_read[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_read[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_zero, d_zero_len * sizeof(float));
  len += d_zero_len;

  

  for (int i = 0; i < 16; i++){
    if(handle[i] == '0'){
      memcpy(&tmp_read[len], d_zero, d_zero_len * sizeof(float));
      len += d_zero_len;
    }
    else{
      memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
      len += d_one_len;
    }

  }

  
  //32 bits
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  //48
   memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
 memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
 len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
 len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
 len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  //50
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  memcpy(&tmp_read[len], d_one, d_one_len * sizeof(float));
  len += d_one_len;
  
  int pad = (usrp_pkt_size ) - (len % usrp_pkt_size);
  d_read = (float *)malloc((len + pad) * sizeof(float));
   memcpy(d_read, tmp_read, len * sizeof(float));
   memcpy(&d_read[len], cw_buffer, pad * sizeof(float));
   
   d_read_len = len + pad;


}

 void 
 rfid_reader_f::gen_qrep_cmd(){

   int len = 0;
   float tmp_qrep[8196];
   memcpy(tmp_qrep, d_reader_framesync, d_reader_framesync_len * sizeof(float));
   len += d_reader_framesync_len;
   memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;
   memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(float));
   len += d_zero_len;

   if(d_session[0] == '0'){ 
     memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(float));
     len += d_zero_len;
   }
   else{
     memcpy(&tmp_qrep[len], d_one, d_one_len * sizeof(float));
     len += d_one_len;
   }

   if(d_session[1] == '0'){ 
     memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(float));
     len += d_zero_len;
   }
   else{
     memcpy(&tmp_qrep[len], d_one, d_one_len * sizeof(float));
     len += d_one_len;
   }
   int pad = (usrp_pkt_size ) - (len % usrp_pkt_size);
   d_qrep = (float *)malloc((len + pad) * sizeof(float));
   memcpy(d_qrep, tmp_qrep, len * sizeof(float));
   memcpy(&d_qrep[len], cw_buffer, pad * sizeof(float));

   d_qrep_len = len + pad;

 }

//0: 0 tags
//1: 1 tag
//2: 2 or more tags
void 
rfid_reader_f::update_q(int slot_occupancy){
  static float C = 0.5;

  switch (slot_occupancy){
       case 0:
	 Q_fp = std::max(float(0), Q_fp - C);
	 break;
       case 1:
	 d_slots_occupied++;
	 break;
       default:
	 d_slots_occupied++;
	 Q_fp = std::min(float(15), Q_fp + C);
	 break;
  }
  
}

bool
rfid_reader_f::send_another_query(){
  

  if(CHANGE_Q){

    if(d_slots_occupied == 0){
	d_num_empty_rounds++;
    }
    else{//If there was an occupied slot.. keep sending queries
      d_num_empty_rounds = 0;
    }
    d_slots_occupied = 0;
  
    if(d_num_empty_rounds == 2){

      if(!TIMED_CYCLE_MODE && global_reader_state->cur_cycle < global_reader_state->num_cycles){
	start_cycle();
	return true;
      }

      printf("Finished %d cycles.\n", global_reader_state->cur_cycle);
      return false;
    }
    else{     
      return true;
    }
  }
  else{
    if(global_reader_state->cur_round < global_reader_state->num_rounds){
      return true;
    }
    else if(!TIMED_CYCLE_MODE && global_reader_state->cur_cycle < global_reader_state->num_cycles){
      start_cycle();

      return true;
    }
    else{
      printf("Finished %d cycles.\n", global_reader_state->cur_cycle);
      return false;
    }
  }
  printf("send_another_query: should not get here\n");
  return false;
}


 void
 rfid_reader_f::forecast (int noutput_items, gr_vector_int &ninput_items_required)
 {
   unsigned ninputs = ninput_items_required.size ();
   for (unsigned i = 0; i < ninputs; i++){
     ninput_items_required[i] = 0;  //Always want to be scheduled to check for message
   }   

 }


 int
 rfid_reader_f::check_crc(char * bits, int num_bits){
   register unsigned short i, j;
   register unsigned short crc_16, rcvd_crc;
   unsigned char * data;
   int num_bytes = num_bits / 8;
   data = (unsigned char* )malloc(num_bytes );
   int mask;

  
   for(i = 0; i < num_bytes; i++){
     mask = 0x80;
     data[i] = 0;
     for(j = 0; j < 8; j++){
       if (bits[(i * 8) + j] == '1'){
	 data[i] = data[i] | mask;
       }
       mask = mask >> 1;
     }

   }
   rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

   crc_16 = 0xFFFF; 
   for (i=0; i < num_bytes - 2; i++) {
     crc_16^=data[i] << 8;
     for (j=0;j<8;j++) {
       if (crc_16&0x8000) {
	 crc_16 <<= 1;
	 crc_16 ^= 0x1021; // (CCITT) x16 + x12 + x5 + 1
       }
       else {
	 crc_16 <<= 1;
       }
     }
   }
   crc_16 = ~crc_16;

   if(rcvd_crc != crc_16){
     //    printf("Failed CRC\n");
     return -1;
   }
   else{
    return 1;
  }
}

void
rfid_reader_f::set_num_samples_to_ungate(){
  global_reader_state->num_samples_to_ungate = 100 + global_reader_state->num_samples_per_bit * 
    (global_reader_state->num_bits_to_decode + global_reader_state->num_bits_in_preamble) + (int)(250 / global_reader_state->us_per_rcv);
}
void
rfid_reader_f::log_msg(int message, char * text, int error){
 if(LOGGING){
      char msg[1000];
      timeval time;
      gettimeofday(&time, NULL);
      tm * t_info = gmtime(&time.tv_sec);
      int len = 0;
      if(text != NULL){
	len = sprintf(msg, "%s Time: %d.%03ld\n", text, (t_info->tm_hour * 3600) +  (t_info->tm_min * 60) + t_info->tm_sec, time.tv_usec / 1000 );
      }
      else{
	len = sprintf(msg,"Time: %d.%03ld\n", (t_info->tm_hour * 3600) +  (t_info->tm_min * 60) + t_info->tm_sec, time.tv_usec / 1000 );
      }
      gr_message_sptr log_msg = gr_make_message(message, 
						0,
						error,
						len);
      memcpy(log_msg->msg(), msg, len);
      
      
      log_q->insert_tail(log_msg);
    }
}
