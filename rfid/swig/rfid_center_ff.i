GR_SWIG_BLOCK_MAGIC(rfid, center_ff);

rfid_center_ff_sptr 
rfid_make_center_ff(int samples_per_pulse);

class rfid_center_ff: public gr_block{
  rfid_center_ff(int samples_per_pulse);

public:
  ~rfid_center_ff();
  
};
