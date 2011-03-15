/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <rfid_tag_decoder_f.h>
#include <gr_io_signature.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <float.h>
#include <cstdio>


//int 	tag_one_cor_vec[] = {1,-1,-1,1};
//int tag_one_cor_vec[] = {1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1}; 
rfid_tag_decoder_f_sptr
rfid_make_tag_decoder_f ()
{
  return rfid_tag_decoder_f_sptr (new rfid_tag_decoder_f ());
}

rfid_tag_decoder_f::rfid_tag_decoder_f()
  : gr_block("rfid_tag_decoder_f",
	     gr_make_io_signature (1, 1, sizeof(float)),
	     gr_make_io_signature (1, 1, sizeof(float)))
  
        
{
  
  init_global_reader_state();
  //  d_tag_bit_vector = (char*) malloc(512 * sizeof(char));
  set_history(m8_preamble_len);
  d_last_score = 0;
  d_preamble_offset = 0;
  global_reader_state->decoder_status = DECODER_SEEK_PREAMBLE;

  d_skip_count = 0;

}


rfid_tag_decoder_f::~rfid_tag_decoder_f()
{}



int rfid_tag_decoder_f::general_work(int noutput_items,
					gr_vector_int &ninput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items)
{
  const float * in = (const float *)input_items[0];
  float * out = (float * )output_items[0];

  int consumed = 0;
  int written = 0;

    
  
 
  for(int i = 0; i < std::min(ninput_items[0]  - (int)history(), noutput_items); i++){
    
    out[written] = in[i];
    consumed++;
    if(d_skip_count-- > 1){
      written++;
      continue;
    }
    if(global_reader_state->decoder_status == DECODER_SEEK_PREAMBLE){
      if(d_samples_processed == 0){
	out[written] = 20;
      }

      set_history(global_reader_state->tag_preamble_cor_vec_len);
      d_samples_processed++;
      //printf("%d,", d_samples_processed);
      double sum = 0;
      double total_pwr = 0;
      float score = 0;
      

      for(int j = 0; j < global_reader_state->tag_preamble_cor_vec_len; j++){
	total_pwr += fabs(in[i + j]);
	sum += global_reader_state->tag_preamble_cor_vec[j] * (in[i + j]);
	
      }
      score = fabs(sum) / total_pwr; 
      


      if(score < d_last_score && d_last_score > 0){
	//printf("preamble at offset:%d\n", d_preamble_offset);
	global_reader_state->decoder_status = DECODER_PREAMBLE_FOUND;
	d_skip_count = global_reader_state->tag_preamble_cor_vec_len - 1;
	set_history(global_reader_state->tag_one_cor_vec_len);
	d_last_score = 0;
	d_samples_processed = 0;
	d_preamble_offset = 0;
	
	out[written] = 10;
	
	
      }

      if(score > 0.9){
	d_last_score = score;
	d_preamble_offset++;
      }


      //2 * because MM outputs two samples per symbol, as it is configured. 
      if(d_samples_processed > 2 * (global_reader_state->num_pulses_per_bit * 
	                      (global_reader_state->num_bits_in_preamble + 
                               (global_reader_state->num_bits_to_decode / 2 )))){
	d_samples_processed = 0;
	global_reader_state->decoder_status = DECODER_CLEAR_PIPE;
	gr_message_sptr ctrl_msg = gr_make_message(0,
						   sizeof(int),
						   0,
						   (1) * sizeof(int));
	int command[] = {NO_PREAMBLE};
	memcpy(ctrl_msg->msg(), &command, 1 * sizeof(int));
	d_ctrl_out->insert_tail(ctrl_msg);
	d_samples_processed = 0;
	out[written] = 5;

      }
        
    }
    else if(global_reader_state->decoder_status == DECODER_PREAMBLE_FOUND){
      double sum = 0;
      double total_pwr = 0;
      float score = 0;
      
      for(int j = 0; j < global_reader_state->tag_one_cor_vec_len; j++){
	total_pwr += fabs(in[i + j]);
	sum += global_reader_state->tag_one_cor_vec[j] * (in[i + j]);
	
      }
      score = fabs(sum) / total_pwr;    
      

      if(score > 0.6){
	out[written] = 2;
	//printf("1\n");
	global_reader_state->tag_bit_vector[global_reader_state->num_bits_decoded++] = '1';
      }
      else{
	//printf("0\n");
	out[written] =  -2;
	global_reader_state->tag_bit_vector[global_reader_state->num_bits_decoded++] = '0';
      }
      
      if(global_reader_state->num_bits_decoded == global_reader_state->num_bits_to_decode ){


	global_reader_state->decoder_status = DECODER_CLEAR_PIPE;
	
	
	
	
	// for(int j = 0; j < global_reader_state->num_bits_to_decode; j++){
	//   printf("%c", global_reader_state->tag_bit_vector[j]);
	// }
	// printf("\n");


	global_reader_state->num_bits_decoded = 0;
	
	gr_message_sptr ctrl_msg = gr_make_message(0,
						   sizeof(int),
						   0,
						   (1) * sizeof(int));
	int command[] = {BITS_DECODED};
	memcpy(ctrl_msg->msg(), &command, 1 * sizeof(int));
	d_ctrl_out->insert_tail(ctrl_msg);
	d_samples_processed = 0;
	out[written++] = 15;
	
	break;
      }
     
      d_skip_count = global_reader_state->tag_one_cor_vec_len ;
    }
    written++;
  }
  
  if(global_reader_state->decoder_status == DECODER_CLEAR_PIPE){
    set_history(0);
        
    consumed = ninput_items[0];
    if(ninput_items[0] == 0 && global_reader_state->command_gate_status == GATE_CLOSED ){
      global_reader_state->decoder_status = DECODER_SEEK_PREAMBLE;
      set_history(global_reader_state->tag_preamble_cor_vec_len);

      //out[written++] = 30;
      // printf("BACK TO SEEK:%d\n",      2 * (global_reader_state->num_pulses_per_bit * 
      // 	                      (global_reader_state->num_bits_in_preamble + 
      //                          (global_reader_state->num_bits_to_decode / 2 ))));
    }
  }
  //written = 0;
  consume_each(consumed);
  return written;
}

void
rfid_tag_decoder_f::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++){
    //    ninput_items_required[i] = noutput_items + history();
    ninput_items_required[i] = history();  //Note, this may overschedule this block. Lots of overhead.
  }   
}
