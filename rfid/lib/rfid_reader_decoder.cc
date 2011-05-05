/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <rfid_reader_decoder.h>
#include <gr_io_signature.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <float.h>
#include <cstdio>


rfid_reader_decoder_sptr
rfid_make_reader_decoder (float us_per_sample, float tari)
{
  return gnuradio::get_initial_sptr (new rfid_reader_decoder (us_per_sample, tari));
}

rfid_reader_decoder::rfid_reader_decoder(float us_per_sample, float tari)
  : gr_sync_block("rfid_reader_decoder",
	     gr_make_io_signature (1, 1, sizeof(float)),
	     gr_make_io_signature (1,1,sizeof(float))),
    d_us_per_sample(us_per_sample)
  
        
{
  log_q = gr_make_msg_queue(100000);
  d_delim_width = (int)((1 / d_us_per_sample) * 12.5);
  d_max_tari = (int)((1 / d_us_per_sample) * 30);
  d_min_amp_thresh = MIN_AMP_THRESH;
  
  //Valid TARIs are [6.25, 25]us. Specifying your reader's TARI helps filter mistakes in decoding, as 
  // both the delim and TARI will need to match.
  // If you don't know your reader setting, specify 0 and look at the trace to see what was detected. 
  // Then rerun with TARI specified.
  
  if(tari > 6.25 && tari < 25){
    d_min_tari = (int) ((1 / d_us_per_sample) * (tari * 0.8));  
  }
  else{
    d_min_tari = (int)((1 / d_us_per_sample) * 6.25);   //Valid TARIs [6.25, 25]us. 
  }

  //Setup structure to hold samples. Used to track avg signal amplitude.
  d_window_length = (int)((1 / d_us_per_sample) * AVG_WIN);
  d_window_samples = (float *)malloc(d_window_length * sizeof(float));
  for (int i = 0; i < d_window_length; i++){
    d_window_samples[i] = 0;
  }
  d_window_index = 0;
  d_avg_amp = 0;

  advance_decoder(BEGIN);

  //Write start time to log file
  char tmp_str[10000];
  timeval time;
  gettimeofday(&time, NULL);
  char * s = ctime(&time.tv_sec);
      
  int str_len = sprintf(tmp_str, "###%s", s);
      
  gr_message_sptr log_msg =  gr_make_message(START,
					       0,
					       0,
					       str_len); 
  memcpy(log_msg->msg(), tmp_str, str_len);
  log_q->insert_tail(log_msg);


}


rfid_reader_decoder::~rfid_reader_decoder()
{}

inline bool 
rfid_reader_decoder::is_positive_edge(float sample){
  return sample > d_thresh;
  
}
inline bool 
rfid_reader_decoder::is_negative_edge(float sample){
  return sample < d_thresh;
  
}


int rfid_reader_decoder::work(int noutput_items, 
	   gr_vector_const_void_star &input_items,
	   gr_vector_void_star &output_items)
{
  const float * in = (const float *)input_items[0];
  float * out = (float * )output_items[0];



  for(int i = 0; i < noutput_items; i++){

    //Track average amplitude
    d_avg_amp = ((d_avg_amp * (d_window_length - 1)) + 
		 (d_avg_amp - d_window_samples[d_window_index]) + 
		 in[i]) / d_window_length;       //Calculate avg by factoring out oldest value, adding newest
    d_window_samples[d_window_index] = in[i];    //Replace oldest value
    d_window_index = (d_window_index + 1) % d_window_length; //Increment point to oldest value

    d_thresh = d_avg_amp * THRESH_FRACTION;  //Threshold for detecting negative/positive edges

    out[i] = d_thresh;

    d_command_count++;         //Tracks length of this command
    d_interarrival_count++;    //Tracks time from end of last command to start of this command

    switch(d_state){

    case BEGIN:
      
      if (is_negative_edge(in[i]) && !neg_edge_found && d_avg_amp > d_min_amp_thresh ){
	d_command_count = 0;               //Begin tracking command duration
	neg_edge_found = true;
	
      }
      if(neg_edge_found){
	d_low_count++;
	if (d_low_count > d_delim_width * 3){  //Reader powered down
	  log_event(POWER_DOWN, d_low_count);
	  advance_decoder(BEGIN);
	}
	if(is_positive_edge(in[i])){
	  if(d_low_count < d_delim_width / 2){ //Too short, not delim
	    advance_decoder(BEGIN);
	  }
	  else{
	    advance_decoder(DELIM_FOUND);  

	  }
	}
      }
      
      break;

    case DELIM_FOUND:
      
      if(!neg_edge_found){
	d_high_count++;
	if(d_high_count > d_max_tari){     //Too long, not TARI
	  advance_decoder(BEGIN);
	}
	if(is_negative_edge(in[i])){
	  neg_edge_found = true;
       	}
      }
      if(neg_edge_found){
	d_low_count++;
	if(d_high_count + d_low_count > d_max_tari){  //Too long, not TARI
	  advance_decoder(BEGIN);
	}
	if(is_positive_edge(in[i])){
	  if(d_high_count + d_low_count < d_min_tari){//Too short
	    advance_decoder(BEGIN);
	  }
	  else{  // Just right...
	    d_tari = d_high_count + d_low_count;
	    advance_decoder(TARI_FOUND);

	  }
	}
      }
     
      break;

    case TARI_FOUND:

      if(!neg_edge_found){
	d_high_count++;
	if(d_high_count > 3.5 * d_tari){    //Too long, not RTCal
	  advance_decoder(BEGIN);
	}
	if(is_negative_edge(in[i])){
	  neg_edge_found = true;
       	}
      }
      if(neg_edge_found){
	d_low_count++;
      	if(d_high_count + d_low_count > 3.5 * d_tari){ //Too long, not RTCal
	  
	  advance_decoder(BEGIN);
	}
	if(is_positive_edge(in[i])){
	  if(d_high_count + d_low_count < 2 * d_tari){ //Too short, not RTCal
	    advance_decoder(BEGIN);
	  }
	  else{

	    d_rtcal = d_high_count + d_low_count;
	    d_pivot = d_rtcal / 2;
	    advance_decoder(RTCAL_FOUND);

	  }
	}
      }
      
      break;

    case RTCAL_FOUND:
      if(!neg_edge_found){
	d_high_count++;
	if(d_high_count > 4 * d_rtcal){  //Too long, not TRCal
	  advance_decoder(BEGIN);
	}
	if(is_negative_edge(in[i])){
	  neg_edge_found = true;
       	}
      }
      if(neg_edge_found){
	d_low_count++;

	if(d_high_count + d_low_count > 4 * d_rtcal){  //Too long, not TRCal
	  
	  advance_decoder(BEGIN);
	}
	if(is_positive_edge(in[i])){
	  if(d_high_count + d_low_count < d_tari / 2){  //Too short for even a data-0
	    advance_decoder(BEGIN);
	  }
	  else{
	    if(d_high_count + d_low_count > 1 * d_rtcal){  //TRCal
	      d_trcal = d_high_count + d_low_count;

	    }
	    else if(d_high_count + d_low_count  > d_pivot){  //Data-1
	      d_bits[d_len_bits++] = '1';

	    }
	    else{  //Data-0
	      d_bits[d_len_bits++] = '0';

	    }
	    advance_decoder(DATA);
	  }
	}
      }
      
      break;
    case DATA:
      if(!neg_edge_found){
	d_high_count++;
	if(d_high_count > 4 * d_tari){  //No more bits, decode command
	  log_event(READER_COMMAND, d_high_count);
 	  advance_decoder(BEGIN);  

	}
	if(is_negative_edge(in[i])){
	  neg_edge_found = true;
       	}
      }
      if(neg_edge_found){
	d_low_count++;
	if(d_low_count > 4 * d_tari){    //Probably a quick power down. 
	                                 //Generally end of command caught in the above block
     
	  log_event(READER_COMMAND, d_low_count + d_high_count);
	  advance_decoder(BEGIN);
	}
      	
	if(is_positive_edge(in[i])){
	  if(d_high_count + d_low_count > d_pivot){// Data-1
	    d_bits[d_len_bits++] = '1';

	  }
	  else{             //data-0
	    d_bits[d_len_bits++] = '0';

	  }
	  advance_decoder(DATA);
	}
      }
      if(d_len_bits == MAX_BITS){  //If reader is off, and state machine gets to DATA, problems.. 
	advance_decoder(BEGIN);
      }

      break;
    default:
      printf("Invalid State\n");
    
    }

    
  }


  return noutput_items;
}

void
rfid_reader_decoder::advance_decoder(int next_state)
{

  d_high_count = 0;
  d_low_count = 0;
  d_state = next_state;
  neg_edge_found = false;
  if(next_state == BEGIN){
    d_len_bits = 0;
  }
  
}

//Lag samples is to factor out the high/low count samples used to detect end of command
void
rfid_reader_decoder::log_event(int event, int lag_samples){
  
  static int last_event = event;    //To avoid logging multiple power down events. 

  if(event == POWER_DOWN && last_event != POWER_DOWN){
    float interarrival = (d_interarrival_count - lag_samples) * d_us_per_sample;
    timeval time;
    gettimeofday(&time, NULL);
    tm * now = localtime(&time.tv_sec);

    char tmp_str[10000];
    int str_len = sprintf(tmp_str, "DECODED:PWR_DWN,INTERARRIVAL:%f,TIME:%d:%d:%d.%.6ld", interarrival, now->tm_hour, now->tm_min, now->tm_sec, time.tv_usec);

    gr_message_sptr log_msg =  gr_make_message(POWER_DOWN,
					       0,
					       0,
					       str_len); 

    memcpy(log_msg->msg(), tmp_str, str_len);
    log_q->insert_tail(log_msg);
  
    d_interarrival_count = lag_samples;  //For next command, interrarrival time is from beginning of power down.
  }

  if(event == READER_COMMAND){
  
    float interarrival = (d_interarrival_count - d_command_count) * d_us_per_sample;
    float command_duration = (d_command_count - lag_samples) * d_us_per_sample;
    
    d_bits[d_len_bits++] = '\0';
    
    timeval time;
    gettimeofday(&time, NULL);
    tm * now = localtime(&time.tv_sec);

    char tmp_str[10000];
    int str_len = sprintf(tmp_str, "CMD:%s,DURATION:%f,INTERARRIVAL:%f,TIME:%d:%d:%d.%.6ld,TARI:%f,RTCAL:%f,TRCAL:%f",d_bits, command_duration, interarrival, now->tm_hour, now->tm_min, now->tm_sec, time.tv_usec, d_tari * d_us_per_sample, d_rtcal * d_us_per_sample, d_trcal * d_us_per_sample);
    
    
    
    
    d_interarrival_count = lag_samples;  //Start tracking interarrival time
    
    gr_message_sptr log_msg =  gr_make_message(READER_COMMAND,
					       0,
					       0,
					       str_len); 
    memcpy(log_msg->msg(), tmp_str, str_len);
    if(log_q->full_p()){
      printf("Message log is full. Blocking\n");
    }
    log_q->insert_tail(log_msg);
  }
  
  last_event = event;

}

