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

#include <rfid_cmd_gate.h>
#include <gr_io_signature.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>

#ifndef READER_VARS
#include "reader_vars.h"
#endif

#include <sys/time.h>
#include <float.h>
#include <signal.h>
#include <math.h>



bool trigger_next_cycle = false;


static itimerval timer;


//Alarm handler that will trigger the start of a new cycle. Duration is set in reader_vars
void 
catch_cycle_alarm (int sig){
  trigger_next_cycle = true;
  signal(sig, catch_cycle_alarm);
  setitimer(ITIMER_REAL, &timer, NULL);


}

rfid_cmd_gate_sptr
rfid_make_cmd_gate(int dec, state * reader_state)
{
  return rfid_cmd_gate_sptr(new rfid_cmd_gate(dec, reader_state));
}

rfid_cmd_gate::rfid_cmd_gate(int dec, state * reader_state): 
  gr_block("rfid_cmd_gate", gr_make_io_signature (1, 1, sizeof(float)), gr_make_io_signature (1,1,sizeof(float))) 
{

  if(DEBUG_ON){
    debug_out = (float *)malloc(MAX_INPUT_ITEMS * sizeof(float) * 100); 
    out_file = fopen("Output.cg", "w");
  }
  d_reader_state = reader_state;
  d_dec = dec;
  d_us_per_sample = 1 / ((64.0) / dec);


  printf("%d %d %d\n", CYCLE_RATE, CYCLE_RATE / 1000, (CYCLE_RATE %1000) * 1000);
  timeval t = {CYCLE_RATE / 1000, (CYCLE_RATE % 1000) * 1000};

  timer.it_interval = t;
  timer.it_value = t;

  signal(SIGALRM, catch_cycle_alarm);
  setitimer(ITIMER_REAL, &timer, NULL);

}

rfid_cmd_gate::~rfid_cmd_gate()
{
  if(DEBUG_ON){
   printf("closing outfile\n");
   fclose(out_file);
  }
}

int
rfid_cmd_gate::general_work(int noutput_items,
			  gr_vector_int &ninput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
 
{
  const float * in = (const float *)input_items[0];
  float * out = (float * )output_items[0];
  int nout = 0;
  int debug_out_cnt = 0;



  //Set up the number of pulses for detecting the end of the last command transmitted
  switch(d_reader_state->last_cmd){

  case QUERY:
    d_reader_state->num_pulses_in_cmd = 26;
    break;
  case QREP:
    d_reader_state->num_pulses_in_cmd = 7;
    break;
  case ACK:
    d_reader_state->num_pulses_in_cmd = 21;
    break;
  case NAK_QREP:
    d_reader_state->num_pulses_in_cmd = 11;
    break;
  case IDLE:
    d_reader_state->num_pulses_in_cmd = 1000;
    break;
  default:
    printf("Last CMD not set\n");
  }

 
  static int num_pulses = 0;  
  for(int i = 0; i < noutput_items; i++){
        
    //If we are gated, detect the last command transmitted and ungate after that
    if(d_reader_state->gated){
      static bool edge_fell = false;
      if(DEBUG_ON){
	debug_out[debug_out_cnt++] = in[i];
      }

      
      if(!edge_fell){

	if(in[i] < d_reader_state->max_rssi * .7){
	  edge_fell = true;
	  
	}
	
      }
      else{

	if((in[i] > d_reader_state->max_rssi * .7) && d_reader_state->avg_rssi > 5000){  //avg_rssi > 5000 avoids triggering when powered down
	    num_pulses++;
	    edge_fell = false;
	    d_sample_count = 0;
	    if(DEBUG_ON){
	      debug_out[debug_out_cnt++] = in[i] + 100000;
	    }

	    
	}
      }
      if(num_pulses == d_reader_state->num_pulses_in_cmd){
	//	printf("num_pulse:%d\n", num_pulses);
	
	d_reader_state->gated = false;
			
	//Skip NAK, look for following QREP 
	if(d_reader_state->last_cmd == NAK_QREP){
	  d_reader_state->gated = true;
	  d_reader_state->last_cmd = QREP;
	  d_reader_state->num_pulses_in_cmd = 7;
	  
	}

	num_pulses = 0;

      }

      //If we saw pulse(s) but this wasn't a command, reset pulse counter
      if(d_sample_count > (int)(reader_trcal / d_us_per_sample) * 1.5 && num_pulses > 0){
	if(num_pulses > 1){//We will always trigger on power up, but we don't want to hear about it
	  printf("Missed Command #Pulses:%d\n", num_pulses);
	}
	num_pulses = 0;
	d_sample_count = 0;
	edge_fell = false;
		
      }
      d_sample_count++;
      
    }
  
    //We are ungated, pass through the samples
    //gen2_reader.cc resets state->gated after it is done
    else if(!d_reader_state->gated){
      out[nout++] = in[i];
      if(DEBUG_ON){
	debug_out[debug_out_cnt++] = in[i] + 5000;
      }
    }
  }
  
  //If the alarm went off, start a new cycle
  // Also calculates the STD_DEV for determining SNR
  if(trigger_next_cycle && d_reader_state->cycle < num_cycles){
    trigger_next_cycle = false;
    d_reader_state->start_cycle = true;
    max_min(in, ninput_items[0], &d_reader_state->max_rssi, &d_reader_state->min_rssi, &d_reader_state->avg_rssi, &d_reader_state->std_dev_rssi);
    memcpy(out, in, noutput_items * sizeof(float));  //Send out samples so downstream blocks get scheduled
    nout = noutput_items;
    d_reader_state->gated = false;

  }

  consume_each(noutput_items);
  if(DEBUG_ON){
    fwrite(debug_out, sizeof(float), debug_out_cnt, out_file);
  }

  
  return nout;
}

int 
rfid_cmd_gate::max_min(const float * buffer, int len, double * max, double * min, double * avg, double * std_dev)
{

  *max = DBL_MIN;
  *min = DBL_MAX;
  double tmp_avg = 0;
  double tmp_std_dev = 0;

  for (int i = 0; i < len; i++){
    tmp_avg += buffer[i];
    if(buffer[i] > * max){
      *max = buffer[i];
    }
    if(buffer[i] < * min){
      *min = buffer[i];
    }
  }
  tmp_avg = tmp_avg / len;
  //Calculate STD_DEV
  for (int i = 0; i < len; i++){
   
    tmp_std_dev += std::pow((buffer[i] - tmp_avg)  ,2);
  }
  

  tmp_std_dev = tmp_std_dev / len;
  tmp_std_dev = sqrt(tmp_std_dev);
 
  
  *avg = tmp_avg;
  *std_dev = tmp_std_dev;

  
 
  return 1;
}

void
rfid_cmd_gate::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++){
    ninput_items_required[i] = noutput_items;
  }   
}

