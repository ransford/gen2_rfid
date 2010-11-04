/* -*- c++ -*- */
/*
 * 1. Interpolate the signal by interp_factor
 * 2. Center the signal at 0 amplitude
 * 3. Find the zero crossings
 * 4. Sample at the appropriate distance from the zero crossing
 *
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <rfid_clock_recovery_zc_ff.h>
#include <gr_io_signature.h>
#include <gri_mmse_fir_interpolator.h>
#include <stdexcept>
#include <float.h>
#include <string.h>
#include <cstdio>

#ifndef READER_VARS
#include "reader_vars.h"
#endif





rfid_clock_recovery_zc_ff_sptr
rfid_make_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor)
{
  return rfid_clock_recovery_zc_ff_sptr(new rfid_clock_recovery_zc_ff(samples_per_pulse, interp_factor));
}

rfid_clock_recovery_zc_ff::rfid_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor)
  : gr_block("rfid_clock_recovery_zc_ff", 
		      gr_make_io_signature(1,1,sizeof(float)),
		      gr_make_io_signature(1,1,sizeof(float))),
    d_samples_per_pulse(samples_per_pulse), d_interp_factor(interp_factor), 
    d_interp(new gri_mmse_fir_interpolator())
{

  set_history(d_interp->ntaps());

  d_interp_buffer = (float * )malloc(8196 * sizeof(float) * d_interp_factor);  //buffer for storing interpolated signal
  
  for(int i = 0; i < 8196 * d_interp_factor; i++){
    d_interp_buffer[i] = 0;  

  }

  d_last_zc_count = 0;  //samples since last zero crossing
  d_pwr = 0;


  int num_pulses = 16;  // Should be a large enough averaging window

  d_avg_window_size = d_samples_per_pulse * num_pulses; 
  d_last_was_pos = true;  


  d_avg_vec_index = 0;
  d_avg_vec = (float*)malloc(d_avg_window_size * sizeof(float));

  
  
  if(DEBUG_ON){
    debug_out1 = (float *)malloc(8196 * sizeof(float) * d_interp_factor);
    for(int i = 0; i < 8196 * d_interp_factor; i++){
      debug_out1[i] = 0;
    }
    out_file1 = fopen("Output.zc.orig", "w");

    debug_out2 = (float *)malloc(8196 * sizeof(float) * d_interp_factor);
    for(int i = 0; i < 8196 * d_interp_factor; i++){
      debug_out2[i] = 0;
    }
    out_file2 = fopen("Output.zc.mod", "w");
  }
  

}

void 
rfid_clock_recovery_zc_ff::forecast(int noutput_items, gr_vector_int &ninput_items_required){
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++){
    ninput_items_required[i] = noutput_items + history();

  }   
}

rfid_clock_recovery_zc_ff::~rfid_clock_recovery_zc_ff()
{
  delete d_interp;
  if(DEBUG_ON){
    fclose(out_file1);
    fclose(out_file2);
  }

}

static inline bool
is_positive(float x){
  return x < 0 ? false : true;
}


int
rfid_clock_recovery_zc_ff::general_work(int noutput_items,
					gr_vector_int &ninput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items)
{
  const float *in = (const float *) input_items[0];
  float* out = (float *) output_items[0];
  int debug_out_cnt1 = 0;
  int debug_out_cnt2 = 0;
  int nout = 0;
  int num_past_samples = d_samples_per_pulse * d_interp_factor;  //This is so we can "look back" if the zero crossing
                                                                 //  is right at the start of the buffer
       
  int num_interp_samples = 0;
  



  for(int i = 0; i < noutput_items; i++){  // Interpolate and center signal 
                                           // Note: Beginning of buffer contains "num_past_samples" samples
    
    //Calculate average
    d_pwr -= d_avg_vec[d_avg_vec_index];
    d_pwr += in[i];
    d_avg_vec[d_avg_vec_index++] = in[i];
    
    if(d_avg_vec_index == d_avg_window_size){
      d_avg_vec_index = 0;
    }


    for(int j = 0; j < d_interp_factor; j++){
      d_interp_buffer[(i * d_interp_factor) + j + num_past_samples] = (d_interp->interpolate(&in[i], ((1 / (float)d_interp_factor) * (float)j))) - (d_pwr / (float)d_avg_window_size);
      num_interp_samples++;
  
    }
    
  }

  
  //Find zero crossings, reduce sample rate by taking only the samples we need
  // Start after the num_past_samples worth of padding
  for(int i = num_past_samples; i < num_interp_samples + num_past_samples; i++){
    if(DEBUG_ON){
      debug_out2[debug_out_cnt2++] =  d_interp_buffer[i];
    }

    if((d_last_was_pos && ! is_positive(d_interp_buffer[i])) || (!d_last_was_pos && is_positive(d_interp_buffer[i]))){
     
      //We found a zero crossing, "look back" and take the sample from the middle of the last pulse. 
      // A long period between zero crossings indicates the long pulse of the miller encoding, 
      // so take two samples from center of pulse
      if(d_last_zc_count > (d_samples_per_pulse * d_interp_factor) * 1.25){

	out[nout++] = d_interp_buffer[i - (d_last_zc_count / 2)];
	out[nout++] = d_interp_buffer[i - (d_last_zc_count / 2)];
      }
      else{

	out[nout++] = d_interp_buffer[i - (d_last_zc_count / 2)];
	
      }

      d_last_zc_count = 0;
    }
    else{
      d_last_zc_count++;
    }

    d_last_was_pos = is_positive(d_interp_buffer[i]);

  }

  
  if(DEBUG_ON){
    for(int i = 0; i < nout; i++){
     debug_out1[debug_out_cnt1++] =  out[i];
    }
  }

  //Copy num_past_samples to head of buffer so we can "look back" during the next general_work call
  memcpy(d_interp_buffer, &d_interp_buffer[num_interp_samples], num_past_samples * sizeof(float));

  if(DEBUG_ON){
    fwrite(debug_out1, sizeof(float), debug_out_cnt1, out_file1);
    fwrite(debug_out2, sizeof(float), debug_out_cnt2, out_file2);
  }
  consume_each(noutput_items);



  return nout;
}
		
      
