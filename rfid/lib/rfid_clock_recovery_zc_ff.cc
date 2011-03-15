
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





rfid_clock_recovery_zc_ff_sptr
rfid_make_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor)
{
  return rfid_clock_recovery_zc_ff_sptr(new rfid_clock_recovery_zc_ff(samples_per_pulse, interp_factor));
}

rfid_clock_recovery_zc_ff::~rfid_clock_recovery_zc_ff()
{
  

}

static inline bool
is_positive(float x){
  return x < 0 ? false : true;
}



rfid_clock_recovery_zc_ff::rfid_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor)
  : gr_block("rfid_clock_recovery_zc_ff", 
		      gr_make_io_signature(1,1,sizeof(float)),
		      gr_make_io_signature(1,1,sizeof(float))),
    d_samples_per_pulse(samples_per_pulse), d_interp_factor(interp_factor)
{

  set_history(d_samples_per_pulse);  //Initial estimate
  d_last_zc_count = 0;  //samples since last zero crossing
  d_max_drift = 0.22;  // Gen 2 spec
  d_alpha = 0.9;
  d_nominal_sp_pulse = d_samples_per_pulse;
}


int
rfid_clock_recovery_zc_ff::general_work(int noutput_items,
				gr_vector_int &ninput_items,
				gr_vector_const_void_star &input_items,
				gr_vector_void_star &output_items)
{
  const float *in = (const float *) input_items[0];
  float* out = (float *) output_items[0];
  int nout = 0;
  int consumed = 0;

  //printf("in%d out%d\n", ninput_items[0], noutput_items);
  //Find zero crossings, reduce sample rate by taking only the samples we need
  // Start after the num_past_samples worth of padding
  for(int i = history(); i < ninput_items[0] - history() ; i++){
    consumed++;
    d_last_zc_count++;

    if((d_last_was_pos && ! is_positive(in[i])) || (!d_last_was_pos && is_positive(in[i]))){
      
      //We found a zero crossing, "look back" and take the sample from the middle of the last pulse. 
      // A long period between zero crossings indicates the long pulse of the miller encoding, 
      // so take two samples from center of pulse
      
      if(d_last_zc_count > d_nominal_sp_pulse * (1 - d_max_drift) && d_last_zc_count < d_nominal_sp_pulse * (1 + d_max_drift)){
      	//printf("update\n");
      	d_samples_per_pulse = (d_samples_per_pulse * d_alpha) + (d_last_zc_count * (1 - d_alpha));
      }
      
      int num_pulses = (int) floor((d_last_zc_count / d_samples_per_pulse) + 0.5);
      //printf("ZC:%f Samples_per_pulse:%f num_pulses:%d\n", d_last_zc_count, d_samples_per_pulse, num_pulses);
      for(int j = 0; j < num_pulses; j++){
	out[nout++] = in[i - std::min(((int) floor(d_samples_per_pulse + 0.5) / 2) * (j + 1), i)];
	out[nout++] = in[i - std::min(((int) floor(d_samples_per_pulse + 0.5) / 2) * (j + 1), i)];
      }
      if(nout == noutput_items){
	break;
      }

      
       d_last_zc_count = 0;
     }
   

    d_last_was_pos = is_positive(in[i]);

    //out[nout++] = in[i];
  }
 
  consume_each(consumed);
  //set_history((int)floor(d_samples_per_pulse + 0.5));
  //printf("nout:%d\n", nout);
  return nout;
}
		
      
void
rfid_clock_recovery_zc_ff::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++){
    ninput_items_required[i] = noutput_items + history();
    
  }   
}
