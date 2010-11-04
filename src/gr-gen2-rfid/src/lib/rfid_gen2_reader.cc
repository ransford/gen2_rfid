/* -*- c++ -*- */
/*
 * Copyright 2004,2006 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif



#include <rfid_gen2_reader.h>
#include <gr_io_signature.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>

#ifndef READER_VARS
#include "reader_vars.h"
#endif

#include <sys/time.h>
#include <float.h>
#include <string.h>


rfid_gen2_reader_sptr
rfid_make_gen2_reader(int dec, int interp)
{
  return rfid_gen2_reader_sptr(new rfid_gen2_reader(dec, interp));
}
rfid_gen2_reader::rfid_gen2_reader(int dec, int interp): gr_block("rfid_gen2_reader", gr_make_io_signature (1, 1, sizeof(float)), gr_make_io_signature (1,1,sizeof(gr_complex))) 

{

  if(DEBUG_ON){
    debug_out = (float *)malloc(MAX_INPUT_ITEMS * sizeof(float) * 100); 
    out_file = fopen("Output.reader", "w");
  }

  out_q = gr_make_msg_queue(100);  //Holds messages for transmission at the end general_work
  log_q = gr_make_msg_queue(500000);//Holds log messages, drained by python app
   

  d_interp = interp;
  d_dec = dec;
  d_us_per_sample = 1 / ((64.0) / dec);  //64 MHz ADC
  d_us_per_xmit = interp / 128;          //128 MHz DAC

  

  d_sample_buffer = (float *)malloc(MAX_INPUT_ITEMS * 4 * sizeof(float));

  //Create data-1 array
  d_one_len = ((reader_pw / d_us_per_xmit) * 4);
  d_one = (gr_complex *)malloc(d_one_len * sizeof(gr_complex));
  for(int i = 0 ; i < (reader_pw / d_us_per_xmit) * 3; i++){
    d_one[i] = 1;
   }
  for(int i = (reader_pw / d_us_per_xmit) * 3; i < ((reader_pw / d_us_per_xmit) * 4); i++){
    d_one[i] = 0;
  }
  
  //Create data-0 array
  d_zero_len = ((reader_pw / d_us_per_xmit) * 2);
  d_zero = (gr_complex *)malloc(d_zero_len * sizeof(gr_complex));
  for(int i = 0 ; i < (reader_pw / d_us_per_xmit); i++){
    d_zero[i] = 1;
  }
  for(int i = (reader_pw / d_us_per_xmit); i < ((reader_pw / d_us_per_xmit) * 2); i++){
    d_zero[i] = 0;
  }

   //Set up cw and zero buffers 
  cw_buffer = (gr_complex *)malloc(8196 * sizeof(gr_complex));
  for(int i = 0; i < 8196; i++){
    cw_buffer[i] = 1;
  }
  zero_buffer = (gr_complex *)malloc(8196 * sizeof(gr_complex));
  for(int i = 0; i < 8196; i++){
   zero_buffer[i] = 0;
  }

  //Set up reader framesync
  int len = 0;
  gr_complex tmp_framesync[8196];
  //Delim
  for(int i = 0; i < reader_delim / d_us_per_xmit; i++){ //Delim
    tmp_framesync[len++] = 0;
  }
  //Data-0
  memcpy((gr_complex *)&tmp_framesync[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  //RTCal
  for(int i = 0; i < (reader_rtcal  / d_us_per_xmit) - (reader_pw / d_us_per_xmit); i++){
    tmp_framesync[len++] = 1;
  }
  //RTCal PW
  for(int i = 0; i < (reader_pw / d_us_per_xmit); i++){
    tmp_framesync[len++] = 0;
  }
  d_reader_framesync_len = len;
  d_reader_framesync = (gr_complex *)malloc(d_reader_framesync_len * sizeof(gr_complex));
  memcpy(d_reader_framesync, tmp_framesync, d_reader_framesync_len * sizeof(gr_complex));


  //Initialize STATE
  STATE_PTR = &STATE;

  char CMD[5] = "1000";
  memcpy(STATE.CMD, CMD, 5);
  char DR[2] = "0";
  memcpy(STATE.DR, DR, 2);
  char M[3] = "11";
  memcpy(STATE.M, M, 3);
  char tr_ext[2] = "1";
  memcpy(STATE.tr_ext, tr_ext, 2);
  char sel[3] = "00";
  memcpy(STATE.sel, sel, 3);
  char session[3] = "00";
  memcpy(STATE.session, session, 3);
  char target[2] = "0";
  memcpy(STATE.target, target, 2);
  char Q[5] = "0000";
  memcpy(STATE.Q, Q, 5);

  STATE.round = 0;
  STATE.cycle = 0;

  STATE.num_tags_found = 0;
 
  gen_query_cmd();
  gen_qrep_cmd();
  gen_nak_cmd();

  STATE.last_cmd = IDLE;
  STATE.gated = true;
  STATE.send_query = false;
  
  d_tag_bit_vector = (char *)malloc(max_tag_response_len * sizeof(char));
  set_history(STATE.tag_preamble_len);
  d_skip_cnt = 0;
  d_last_score = FLT_MIN;
  d_msg_count = 0;

  reset_receive_state();

  DEFAULT_NUM_INPUT_ITEMS = STATE.tag_preamble_len;//A reasonable number of input items to request
  d_num_input_items = DEFAULT_NUM_INPUT_ITEMS;  

}


rfid_gen2_reader::~rfid_gen2_reader()
{
  if(DEBUG_ON){
    printf("closing outfile\n");
    fclose(out_file);
  }
}


int
rfid_gen2_reader::general_work(int noutput_items,
			  gr_vector_int &ninput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
 
{
  const float * in = (const float *)input_items[0];
  gr_complex * out = (gr_complex * )output_items[0];
  int nout = 0;
  int debug_out_cnt = 0;
  int consumed = d_num_input_items;  //Only consume the number of samples we asked for
  int num_samples = 0;  //Number of samples to iterate through. 
                        // Will be set to d_num_input_items + samples we copied last call
  int i = 0;

 

 
  
  //If we are starting a new cycle, or have samples that need to be transmitted, skip main loop
  if(!STATE.start_cycle && !tx_msg){
    
    
    num_samples = d_items_copied + d_num_input_items;
    memcpy(&d_sample_buffer[d_items_copied], in, ninput_items[0] * sizeof(float));  //Just copy everything
    
     
    while(i < int(num_samples - history())){  //We will copy history() samples for next call

      if(DEBUG_ON){
	if(d_skip_cnt-- > 1){
	  if(DEBUG_ON){
	    debug_out[debug_out_cnt++] = d_sample_buffer[i];
	  
	  } 
	  i++;
	  continue;
	}
      }
    
      //Correlate for preamble
      if(!STATE.found_preamble){
	double sum = 0;
	double total_pwr = 0;
	float score = 0;
	for(int j = 0; j < STATE.tag_preamble_len; j++){
	  total_pwr += fabs(d_sample_buffer[i + j]);
	  sum += STATE.tag_preamble[j] * (d_sample_buffer[i + j]);
	}
      
	score = fabs(sum) / total_pwr;    

	if(d_last_score != 0 && score < d_last_score){//We found max correlation at the last sample
	  if(DEBUG_ON){
	  
	    debug_out[debug_out_cnt++] = 10000;
	    debug_out[debug_out_cnt++] = d_sample_buffer[i];
	  }
	
	  set_history(STATE.tag_one_len);
	
	  STATE.found_preamble = true;
	  d_skip_cnt = STATE.tag_preamble_len - 1;
	  	  
	  STATE.SNR = (total_pwr / STATE.tag_preamble_len) / STATE.std_dev_rssi;
		
	}
	else{
	  if(score > .9){  //Store last score. Keep correlating, next score might be better yet.
	    double max, min, avg;
	    max_min(&d_sample_buffer[i], STATE.tag_preamble_len, &max, &min, &avg);
	   
	    if(fabs(max + min) < max){  //Hack to ignore USRP TX glitch (max min should be centered)
	      d_last_score = score;
	   
	    }

	  }
	  if(DEBUG_ON){
	    debug_out[debug_out_cnt++] = d_sample_buffer[i];
	  
	  }
	}
	//Preamble not detected 
	if(d_find_preamble_count++ > STATE.tag_preamble_len * 6){
	  
	  STATE.SNR = (total_pwr / STATE.tag_preamble_len) / STATE.std_dev_rssi;

	  //If we see a signal, probably collided tags as we missed the preamble
	  if(STATE.last_cmd == QUERY || STATE.last_cmd == QREP){
	    if(STATE.SNR > SIG_DETECT_THRESH){
	      char tmp[500];
	      sprintf(tmp, "%f", STATE.SNR);
	      log_msg(LOG_COLLISION, tmp, LOG_OKAY);
	    }
	    else{
	      char tmp[500];
	      sprintf(tmp, "%f", STATE.SNR);
	      log_msg(LOG_EMPTY, tmp, LOG_OKAY);
	    }
	   

	  }
	  //We missed the EPC, send NAK just in case
	  if(STATE.last_cmd == ACK){
	  
	      gr_message_sptr nak_msg = gr_make_message(0,
							sizeof(gr_complex),
							0,
							d_nak_msg_len * sizeof(gr_complex));
	      memcpy(nak_msg->msg(), d_nak_msg, d_nak_msg_len * sizeof(gr_complex));
	      out_q->insert_tail(nak_msg);
	      
	      gr_message_sptr cw_msg = gr_make_message(0,
						       sizeof(gr_complex),
						       0,
						       (128) * sizeof(gr_complex));
	      memcpy(cw_msg->msg(), cw_buffer, (128) * sizeof(gr_complex));
	      out_q->insert_tail(cw_msg);
	      
	  

	   
	    if(STATE.SNR < SIG_DETECT_THRESH){ 
	      //This is just an approximation, maybe we sent a bad ACK
	      if(STATE.bit_error_on_previous){
		log_msg(LOG_BAD_RN_MISS, NULL, LOG_OKAY);
	
	      }
	      else{
		log_msg(LOG_TIME_MISS, NULL, LOG_OKAY);
	
	      }
	   
	    }
	    else{
	      //Missed EPC preamble, but found signal. Not much we can do about that
	     

	    }
	    log_msg(LOG_NAK, NULL, LOG_OKAY);
	  }

	  //We have sent QREPs for each slot, send another Query
	  if(STATE.num_slots - STATE.cur_slot == 0){
	    STATE.send_query = true;
	    
	  }

	  else if(STATE.num_slots - STATE.cur_slot > 0){ //If we are not done with slots
	    STATE.cur_slot++;
	    //Send padding and QREP
	    
	    gr_message_sptr qrep_msg = gr_make_message(0,
						       sizeof(gr_complex),
						       0,
						       d_qrep_msg_len  * sizeof(gr_complex));
	    memcpy(qrep_msg->msg(), d_qrep_msg, d_qrep_msg_len * sizeof(gr_complex));
	    out_q->insert_tail(qrep_msg);
	    

	    log_msg(LOG_QREP, NULL, LOG_OKAY);
	  
	  }
	  
	  if(STATE.last_cmd == ACK){
	    STATE.last_cmd = NAK_QREP;
	  }
	  else{
	    STATE.last_cmd = QREP;
	  }

	  reset_receive_state();
	  STATE.gated = true;
	  consumed = ninput_items[0];
	  
	  break;
	
	}
      
      }//End finding preamble
      else{//Decode bits
      
	//Correlate for bits, using Miller encoded 1

	double sum = 0;
	double total_pwr = 0;
	float score = 0;
	for(int j = 0; j < STATE.tag_one_len; j++){
	  total_pwr += fabs(d_sample_buffer[i + j]);
	  sum += STATE.tag_one[j] * (d_sample_buffer[i + j]);
	}
	
	score = fabs(sum) / total_pwr; 

	//A high score means the bit is a 1
	if(score  > .5){
	  
	  if(DEBUG_ON){
	    debug_out[debug_out_cnt++] = d_sample_buffer[i] + 5;
	    
	  }
	  d_tag_bit_vector[STATE.num_bits_decoded++] = '1';
	}
	else{
	  if(DEBUG_ON){
	    debug_out[debug_out_cnt++] = d_sample_buffer[i] - 5;
	    
	  }
	  d_tag_bit_vector[STATE.num_bits_decoded++] = '0';
	}
	
	
	
	if(score > .45 && score < .55){//Mark as error, but still send out bits. Just for debugging
	  
	  STATE.bit_error = true;
	  
	}
	
	//Skip over the bit, and set d_num_input_items so that we request exactly the # of samples we need
	d_skip_cnt = STATE.tag_one_len;	
	
	//Commented out for latency without optimizing request size
	//d_num_input_items = DEFAULT_NUM_INPUT_ITEMS;
	d_num_input_items = STATE.tag_one_len * (STATE.num_bits_to_decode - STATE.num_bits_decoded);
	

	if(STATE.num_bits_decoded == STATE.num_bits_to_decode){
	 

	  if(STATE.last_cmd == QUERY || STATE.last_cmd == QREP){
	    send_ack();
	    STATE.last_cmd = ACK;

	    // for(int i = 0; i < STATE.num_bits_decoded;i++){
	    // 		printf("%c", d_tag_bit_vector[i]);
	    // 	      }
	    // 	      printf("\n");

	    
	    //Set bit error for ACK command. Used to determine missed due to timing/bad RN16 
	    if(STATE.bit_error){
	      STATE.bit_error_on_previous = true;
	      d_tag_bit_vector[STATE.num_bits_decoded] = '\0';
	      char tmp[500];
	      char tmp2[500];
	      strcpy(tmp, d_tag_bit_vector);
	      sprintf(tmp2, ",%f\n", STATE.SNR);
	      strcat(tmp, tmp2);
	      log_msg(LOG_ACK, tmp, LOG_ERROR);

	    }
	    else{
	    
	      d_tag_bit_vector[STATE.num_bits_decoded] = '\0';
	      char tmp[500];
	      char tmp2[500];
	      strcpy(tmp, d_tag_bit_vector);
	      sprintf(tmp2, ",%f\n", STATE.SNR);
	      strcat(tmp, tmp2);

	      log_msg(LOG_ACK, tmp, LOG_OKAY);
	      STATE.bit_error_on_previous = false;
	    }
	  }
	
	  else if(STATE.last_cmd == ACK){
	    // for(int i = 0; i < STATE.num_bits_decoded;i++){
// 	      printf("%c", d_tag_bit_vector[i]);
// 	    }
// 	    printf("\n");
	    int pass = check_crc(d_tag_bit_vector, STATE.num_bits_decoded);
	    
	   
	    STATE.cur_slot++;

	    if(pass == 1){
	      
	      gr_message_sptr qrep_msg = gr_make_message(0,
							 sizeof(gr_complex),
							 0,
							 d_qrep_msg_len * sizeof(gr_complex));
	      memcpy(qrep_msg->msg(), d_qrep_msg, d_qrep_msg_len * sizeof(gr_complex));
	      out_q->insert_tail(qrep_msg);


	      STATE.num_tags_found++;
	      STATE.last_cmd = QREP;
	      d_tag_bit_vector[STATE.num_bits_decoded] = '\0';
	      char tmp[500];
	      char tmp2[500];
	      strcpy(tmp, d_tag_bit_vector);
	      sprintf(tmp2, ",%f\n", STATE.SNR);
	      strcat(tmp, tmp2);
	      
	      log_msg(LOG_EPC, tmp, LOG_OKAY);
	      log_msg(LOG_QREP, NULL, LOG_OKAY);
	    
	    
	    }
	    else{//Failed CRC
	      
		gr_message_sptr nak_msg = gr_make_message(0,
							  sizeof(gr_complex),
							  0,
							  d_nak_msg_len * sizeof(gr_complex));
		memcpy(nak_msg->msg(), d_nak_msg, d_nak_msg_len * sizeof(gr_complex));
		out_q->insert_tail(nak_msg);
	      
		gr_message_sptr cw_msg = gr_make_message(0,
							 sizeof(gr_complex),
							 0,
							 (128) * sizeof(gr_complex));
		memcpy(cw_msg->msg(), cw_buffer, (128) * sizeof(gr_complex));
		out_q->insert_tail(cw_msg);
	      
		gr_message_sptr qrep_msg = gr_make_message(0,
							   sizeof(gr_complex),
							   0,
							   d_qrep_msg_len * sizeof(gr_complex));
		memcpy(qrep_msg->msg(), d_qrep_msg, d_qrep_msg_len * sizeof(gr_complex));
		out_q->insert_tail(qrep_msg);
	

		STATE.last_cmd = NAK_QREP;
		d_tag_bit_vector[STATE.num_bits_decoded] = '\0';
		char tmp[500];
		char tmp2[500];
		strcpy(tmp, d_tag_bit_vector);
		sprintf(tmp2, ",%f\n", STATE.SNR);
		strcat(tmp, tmp2);

		log_msg(LOG_EPC, tmp, LOG_ERROR);
		log_msg(LOG_NAK, NULL, LOG_OKAY);
		log_msg(LOG_QREP, NULL, LOG_OKAY);
		
	    

	    }
	  
	    

	    
	  }
	
	  if(DEBUG_ON){
	    debug_out[debug_out_cnt++] = -10;
	  
	  }
	
	  STATE.gated = true;
	  consumed = ninput_items[0];
	  reset_receive_state();
	  break;
	
	}
      
      }
      if(DEBUG_ON){
	i++;
      }
      else{ //Skip instead of iteration
      
	if(d_skip_cnt > 0){
	  i = i + d_skip_cnt;
	}
	else{
	  i++;
	}
      
      }
    }//end main loop
  }   
  if(STATE.start_cycle){


    STATE.start_cycle = false;
    STATE.num_tags_found = 0;

    int min_pwr_dwn = 2000;  //Spec says 1000, more is fine.
    int num_pkts = ((min_pwr_dwn / d_us_per_xmit) / 128); //Round to the nearest 128 samples

    for(int i = 0; i < num_pkts; i++){
      gr_message_sptr pwr_dwn_msg = gr_make_message(0,
						     sizeof(gr_complex),
						     0,
						     (128) * sizeof(gr_complex));
      memcpy(pwr_dwn_msg->msg(), zero_buffer, (128) * sizeof(gr_complex));
      out_q->insert_tail(pwr_dwn_msg);
    }
    
    
    log_msg(LOG_PWR_UP, NULL, LOG_OKAY);

    STATE.round = 0;
    STATE.send_query = true;
    STATE.cycle++;
   
  }


  if(STATE.send_query && STATE.round == num_rounds && STATE.cycle == num_cycles){
    printf("Finished all cycles/rounds: %d\n", STATE.cycle);
      //exit(0);    //Should end reader.  But, this currently screws up gen2_reader.py logging
  }


  if(STATE.send_query && STATE.round < num_rounds){
    reset_receive_state();
    
    STATE.send_query = false;
    
    gen_query_cmd();
    STATE.gated = true;
    STATE.last_cmd = QUERY;
    
    if(FIND_ALL_TAGS){
      if(NUM_TAGS - STATE.num_tags_found == 0){
	printf("Found all tags!: %d\n", STATE.round);
	STATE.round = num_rounds;  //We win. No more rounds
      }
    }
    
    STATE.round++;
    
    int min_cw = 2000;
    int num_pkts = ((min_cw / d_us_per_xmit) / 128);
    
    for(int i = 0; i < num_pkts; i++){
      gr_message_sptr cw_msg = gr_make_message(0,
					       sizeof(gr_complex),
					       0,
					       (128) * sizeof(gr_complex));
      memcpy(cw_msg->msg(), cw_buffer, 128 * sizeof(gr_complex));
      out_q->insert_tail(cw_msg);
    }
    
    gr_message_sptr query_msg = gr_make_message(0,
						sizeof(gr_complex),
						0,
						(d_query_len) * sizeof(gr_complex));
    memcpy(query_msg->msg(), d_query_cmd, d_query_len * sizeof(gr_complex));
    out_q->insert_tail(query_msg);
    
    char q_str[1000];
    sprintf(q_str, "CMD: %s DR: %s M: %s TR: %s SEL: %s Sess:%s Targ:%s Q: %s CRC: %s", STATE.CMD, STATE.DR, STATE.M, STATE.tr_ext, STATE.sel, STATE.session, STATE.target, STATE.Q, STATE.CRC);
    
    log_msg(QUERY, q_str, LOG_OKAY);
  

   
  }//End Send Query
 
  
  //Transmit commands, if there are any
  if(!tx_msg){
    tx_msg = out_q->delete_head_nowait();
  }

  //Transmit noutout_items worth of command, if there is more reschedule this block. 
  while(tx_msg){
    
    int mm = std::min((tx_msg->length() - d_msg_count) / sizeof(gr_complex), (long unsigned int) noutput_items - nout);
    memcpy(&out[nout], &tx_msg->msg()[d_msg_count], mm * sizeof(gr_complex));
    nout += mm;
    d_msg_count += mm * sizeof(gr_complex);
    if(d_msg_count == (int)tx_msg->length()){
      tx_msg.reset();
      d_msg_count = 0;
      tx_msg = out_q->delete_head_nowait(); 
      
	      
    }
    if(nout == noutput_items && tx_msg){  //We have to transmit more, setting d_num_input_items == 0 reschedules this block
      d_num_input_items = 0;
      break;      
    }
    d_num_input_items = DEFAULT_NUM_INPUT_ITEMS;
    
    
  }



  if(DEBUG_ON){
    fwrite(debug_out, sizeof(float), debug_out_cnt, out_file);
  }

  //Copy end of buffer to head of sample_buffer if we are in the middle of processing
  if(!STATE.gated){
    d_items_copied = num_samples - i;
  }
  else{
    d_items_copied = 0;
  }
  
  memcpy(d_sample_buffer, &d_sample_buffer[num_samples - d_items_copied], d_items_copied * sizeof(float));
  consume_each(consumed);
  

  return nout;
  
}

void
rfid_gen2_reader::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++){
    ninput_items_required[i] = d_num_input_items;

  }   
}

void
rfid_gen2_reader::gen_nak_cmd(){
 //Set up NAK message
  int len = 0;
  gr_complex tmp_nak[8196];
  memcpy(tmp_nak, d_reader_framesync, d_reader_framesync_len * sizeof(gr_complex));
  len += d_reader_framesync_len;
  memcpy(&tmp_nak[len], d_one, d_one_len * sizeof(gr_complex));
  len += d_one_len;
  memcpy(&tmp_nak[len], d_one, d_one_len * sizeof(gr_complex));
  len += d_one_len;
  memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  memcpy(&tmp_nak[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;

  int pad = 128 - (len % 128);
  d_nak_msg = (gr_complex *)malloc((len + pad) * sizeof(gr_complex));
  memcpy(d_nak_msg, tmp_nak, len * sizeof(gr_complex));
  memcpy(&d_nak_msg[len], cw_buffer, pad * sizeof(gr_complex));
  
  d_nak_msg_len = len + pad;

}
void 
rfid_gen2_reader::gen_qrep_cmd(){
  
  int len = 0;
  gr_complex tmp_qrep[8196];
  memcpy(tmp_qrep, d_reader_framesync, d_reader_framesync_len * sizeof(gr_complex));
  len += d_reader_framesync_len;
  memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;
  memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(gr_complex));
  len += d_zero_len;

  if(STATE.session[0] == '0'){ 
    memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(gr_complex));
    len += d_zero_len;
  }
  else{
    memcpy(&tmp_qrep[len], d_one, d_one_len * sizeof(gr_complex));
    len += d_one_len;
  }

  if(STATE.session[1] == '0'){ 
    memcpy(&tmp_qrep[len], d_zero, d_zero_len * sizeof(gr_complex));
    len += d_zero_len;
  }
  else{
    memcpy(&tmp_qrep[len], d_one, d_one_len * sizeof(gr_complex));
    len += d_one_len;
  }
  int pad = (128 ) - (len % 128);
  d_qrep_msg = (gr_complex *)malloc((len + pad) * sizeof(gr_complex));
  memcpy(d_qrep_msg, tmp_qrep, len * sizeof(gr_complex));
  memcpy(&d_qrep_msg[len], cw_buffer, pad * sizeof(gr_complex));
  //memcpy(d_qrep_msg, cw_buffer, pad * sizeof(gr_complex));
  //memcpy(&d_qrep_msg[pad], tmp_qrep, len * sizeof(gr_complex));
  
  d_qrep_msg_len = len + pad;

}

void
rfid_gen2_reader::gen_query_cmd(){
 
  int len_query = 22;
  
  char * q_bits; 


  //If we are adapting Q, set it based on the number of tags left
  if(CHANGE_Q){
    if(NUM_TAGS - STATE.num_tags_found > 0){
    
      memcpy(STATE.Q, q_params[NUM_TAGS - STATE.num_tags_found - 1], 5);

    }
    
  }

  q_bits = (char * )malloc(len_query);
  q_bits[0] = '\0';
  strcat(q_bits, STATE.CMD);
  strcat(q_bits, STATE.DR);
  strcat(q_bits, STATE.M);
  strcat(q_bits, STATE.tr_ext);
  strcat(q_bits, STATE.sel);
  strcat(q_bits, STATE.session);
  strcat(q_bits, STATE.target);
  strcat(q_bits, STATE.Q);


  //Calculate number of slots
  int q = 0;
  int factor = 8;
  for(int i = 0; i < 4; i++){
    if(STATE.Q[i] == '1'){
      q += factor;
    }
    factor = factor / 2;
  }
  STATE.cur_slot = 0;
  STATE.num_slots = (int)pow(2, q);



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
    STATE.CRC[cnt] = crc[i];
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
  
  d_query_len = ((reader_delim + reader_pw + reader_pw + reader_rtcal + reader_trcal) / d_us_per_xmit) + (num_1 * d_one_len) + (num_0 * d_zero_len);
  
  

  int pad = 128 - (d_query_len % 128);
 
  d_query_len += pad;
  d_query_cmd = (gr_complex * )malloc((d_query_len) * sizeof(gr_complex)); 
    
  int j = 0;
  //Pad for USB buffer size
  for(int i = 0; i < pad; i++){
    d_query_cmd[j++] = 1;
  } 

  memcpy(&d_query_cmd[j], d_reader_framesync, d_reader_framesync_len * sizeof(gr_complex));
  j += d_reader_framesync_len;

  //TRCal
  for(int i = 0; i < (reader_trcal  / d_us_per_xmit) - (reader_pw / d_us_per_xmit); i++){
    
    d_query_cmd[j++] = 1;
  }

  //TRCal PW
  for(int i = 0; i < (reader_pw / d_us_per_xmit); i++){
    
    d_query_cmd[j++] = 0;
  }

  for(int i = 0; i < len_query; i++){
    
    if(q_bits[i] == '0'){
      memcpy((gr_complex *)&d_query_cmd[j], d_zero, d_zero_len * sizeof(gr_complex));
      j += d_zero_len;
    }
    else if(q_bits[i] == '1'){
      memcpy((gr_complex *)&d_query_cmd[j], d_one, d_one_len * sizeof(gr_complex));
      j += d_one_len;
    }
  }
  
  if(strcmp(STATE.M, "00") == 0){
    STATE.tag_preamble = fm0_preamble;
    STATE.tag_preamble_len = len_fm0_preamble;
    STATE.tag_one = fm0_one_vec;
    STATE.tag_one_len = len_fm0_one;
  }
  if(strcmp(STATE.M, "11") == 0){
    STATE.tag_preamble = m8_preamble;
    STATE.tag_preamble_len = len_m8_preamble;
    STATE.tag_one = m8_one_vec;
    STATE.tag_one_len = len_m8_one;

  }
  if(strcmp(STATE.M, "10") == 0){
    STATE.tag_preamble = m4_preamble;
    STATE.tag_preamble_len = len_m4_preamble;
    STATE.tag_one = m4_one_vec;
    STATE.tag_one_len = len_m4_one;

  }
  if(strcmp(STATE.M, "01") == 0){
    STATE.tag_preamble = m2_preamble;
    STATE.tag_preamble_len = len_m2_preamble;
    STATE.tag_one = m2_one_vec;
    STATE.tag_one_len = len_m2_one;

  }

}

void 
rfid_gen2_reader::max_min(const float * buffer, int len, double * max, double * min, double * avg )
{

  *max = DBL_MIN;
  *min = DBL_MAX;
  double tmp = 0;

   for (int i = 0; i < len; i++){
     tmp += buffer[i];
     if(buffer[i] > * max){
       *max = buffer[i];
     }
     if(buffer[i] < * min){
       *min = buffer[i];
     }
   }
  
  *avg = tmp / len;
  
}

void
rfid_gen2_reader::reset_receive_state(){
 
  set_history(STATE.tag_preamble_len);
  d_num_input_items = DEFAULT_NUM_INPUT_ITEMS;
  
  d_find_preamble_count = 0;
  STATE.found_preamble = false;
  STATE.bit_error = false;
  STATE.num_bits_decoded = 0;
  d_skip_cnt = 0;
  d_last_score = 0;

  
  
  if(STATE.last_cmd == ACK){
    STATE.num_bits_to_decode = num_EPC_bits;
  }
  else{
    STATE.num_bits_to_decode = num_RN16_bits;
  }
  
}


int 
rfid_gen2_reader::send_ack(){
  gr_complex ack_msg[8196];
  

  int pad = 0;
  int len = 0;
  
  //Add header
  memcpy(ack_msg, d_reader_framesync, d_reader_framesync_len * sizeof(gr_complex));
  memcpy((gr_complex *)&ack_msg[d_reader_framesync_len], d_zero, d_zero_len * sizeof(gr_complex));
  memcpy((gr_complex *)&ack_msg[d_reader_framesync_len + d_zero_len], d_one, d_one_len * sizeof(gr_complex));
  
  len = d_reader_framesync_len + d_zero_len + d_one_len;
  
  for(int i = 0; i < STATE.num_bits_decoded; i++){
    if(d_tag_bit_vector[i] == '0'){
      memcpy((gr_complex *)&ack_msg[len], d_zero, d_zero_len * sizeof(gr_complex));
      len += d_zero_len;
    }
    else{
      memcpy((gr_complex *)&ack_msg[len], d_one, d_one_len * sizeof(gr_complex));
      len += d_one_len; 
    }
    
  }

  pad = 128 - (len % 128);
  



  gr_complex tmp[pad+len];
  memcpy(tmp, ack_msg, len * sizeof(gr_complex));
  memcpy(&tmp[len], cw_buffer, pad *sizeof(gr_complex));
	 

  gr_message_sptr new_tx_msg = gr_make_message(0,
					       sizeof(gr_complex),
					       0,
					       (len + pad) * sizeof(gr_complex));
  
  memcpy(new_tx_msg->msg(), tmp, (pad + len) *sizeof(gr_complex));
  out_q->insert_tail(new_tx_msg);


  return 1;
}




int
rfid_gen2_reader::check_crc(char * bits, int num_bits){
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
rfid_gen2_reader::log_msg(int message, char * text, int error){
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
