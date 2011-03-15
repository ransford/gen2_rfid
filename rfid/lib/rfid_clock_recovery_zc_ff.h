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
  void forecast (int noutput_items, gr_vector_int &ninput_items_required); 

private:
  float d_nominal_sp_pulse;
  float d_samples_per_pulse;
  float d_last_zc_count;
  bool d_last_was_pos;
  float d_max_drift;
  float d_alpha;
  int d_interp_factor; //Kill this.
 

};

#endif /* INCLUDED_rfid_clock_recovery_zc_ff_H*/
