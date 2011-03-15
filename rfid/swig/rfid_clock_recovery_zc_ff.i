GR_SWIG_BLOCK_MAGIC(rfid, clock_recovery_zc_ff);

rfid_clock_recovery_zc_ff_sptr 
rfid_make_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);

class rfid_clock_recovery_zc_ff: public gr_block{
  rfid_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);

public:
  ~rfid_clock_recovery_zc_ff();
  
};
