/* -*- c++ -*- */

#ifndef INCLUDED_rfid_clock_recovery_zc_ff_H
#define INCLUDED_rfid_clock_recovery_zc_ff_H

#include <gr_block.h>

class gri_mmse_fir_interpolator;

class rfid_clock_recovery_zc_ff;
typedef boost::shared_ptr<rfid_clock_recovery_zc_ff> rfid_clock_recovery_zc_ff_sptr;

rfid_clock_recovery_zc_ff_sptr
rfid_make_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);

class rfid_clock_recovery_zc_ff : public gr_block
{  

  friend rfid_clock_recovery_zc_ff_sptr
  rfid_make_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);

  public:
  ~rfid_clock_recovery_zc_ff();
  int general_work(int noutput_items,
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);
protected:

  rfid_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);
  

private:
  int d_samples_per_pulse;
  int d_interp_factor;
  gri_mmse_fir_interpolator 	*d_interp;
  float * d_interp_buffer;
  int d_last_zc_count;
  float d_pwr;
  int d_avg_window_size;
  bool d_last_was_pos;

  float * d_avg_vec;
  int d_avg_vec_index;

  //Debug vars
  float * debug_out1;
  FILE * out_file1;

  float * debug_out2;
  FILE * out_file2;

  void forecast (int noutput_items, gr_vector_int &ninput_items_required);
  

};

#endif /* INCLUDED_rfid_clock_recovery_zc_ff_H*/
