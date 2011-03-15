/* -*- c++ -*- */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <rfid_center_ff.h>
#include <gr_io_signature.h>
#include <gri_mmse_fir_interpolator.h>
#include <stdexcept>
#include <float.h>
#include <string.h>
#include <cstdio>


rfid_center_ff_sptr
rfid_make_center_ff(int samples_per_pulse)
{
  return rfid_center_ff_sptr(new rfid_center_ff(samples_per_pulse));
}

rfid_center_ff::rfid_center_ff(int samples_per_pulse)
  : gr_sync_block("rfid_center_ff", 
		      gr_make_io_signature(1,1,sizeof(float)),
		      gr_make_io_signature(1,1,sizeof(float))),
    d_samples_per_pulse(samples_per_pulse) 
    
{

 
  int num_pulses = 32;  // Should be a large enough averaging window

  //Setup structure to hold samples. Used to track avg signal amplitude.

  d_window_length = num_pulses * samples_per_pulse;
  d_window_samples = (float *)malloc(d_window_length * sizeof(float));
  for (int i = 0; i < d_window_length; i++){
    d_window_samples[i] = 0;
  }
  d_window_index = 0;
  d_avg_amp = 0;

}



rfid_center_ff::~rfid_center_ff()
{
  
  

}

static inline bool
is_positive(float x){
  return x < 0 ? false : true;
}


int
rfid_center_ff::work(int noutput_items,
		     gr_vector_const_void_star &input_items,
		     gr_vector_void_star &output_items)
{
  const float *in = (const float *) input_items[0];
  float* out = (float *) output_items[0];


  for(int i = 0; i < noutput_items; i++){
    

     //Track average amplitude
    d_avg_amp = ((d_avg_amp * (d_window_length - 1)) + 
		 (d_avg_amp - d_window_samples[d_window_index]) + 
		 std::abs(in[i])) / d_window_length;       //Calculate avg by factoring out oldest value, adding newest
    d_window_samples[d_window_index] = std::abs(in[i]);    //Replace oldest value
    d_window_index = (d_window_index + 1) % d_window_length; //Increment point to oldest value
    
    if(in[i] - d_avg_amp > 10){
      out[i] = 10;

    }
    else if(in[i] - d_avg_amp < -10){
      out[i] = -10;

    }
    else{

      out[i] = in[i] - d_avg_amp;

      }
    
  }
  

  return noutput_items;
}
		
     
