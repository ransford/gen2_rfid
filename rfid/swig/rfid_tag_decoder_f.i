GR_SWIG_BLOCK_MAGIC(rfid, tag_decoder_f);

rfid_tag_decoder_f_sptr
rfid_make_tag_decoder_f();

class rfid_tag_decoder_f: public gr_block{
  rfid_tag_decoder_f();

public:
  ~rfid_tag_decoder_f();
  void set_ctrl_out(gr_msg_queue_sptr msgq) const;
};
