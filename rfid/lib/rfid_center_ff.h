/* -*- c++ -*- */

#ifndef INCLUDED_rfid_center_ff_H
#define INCLUDED_rfid_center_ff_H

#include <gr_sync_block.h>


class rfid_center_ff;
typedef boost::shared_ptr<rfid_center_ff> rfid_center_ff_sptr;

rfid_center_ff_sptr
rfid_make_center_ff(int samples_per_pulse);

class rfid_center_ff : public gr_sync_block
{  

  friend rfid_center_ff_sptr
  rfid_make_center_ff(int samples_per_pulse);

  public:
  ~rfid_center_ff();
  int work(int noutput_items,
	   gr_vector_const_void_star &input_items,
	   gr_vector_void_star &output_items);
protected:

  rfid_center_ff(int samples_per_pulse);
  

private:
  float * d_window_samples;   //Array to hold samples for averaging amplitude
  int d_window_length;        //Length of window
  int d_window_index;         //Index to oldest sample
  double d_avg_amp;           //Average amplitude over window
  int d_samples_per_pulse;
 

 
  

};

#endif /* INCLUDED_rfid_center_ff_H*/
