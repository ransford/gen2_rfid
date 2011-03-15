GR_SWIG_BLOCK_MAGIC(rfid, reader_decoder);

rfid_reader_decoder_sptr 
rfid_make_reader_decoder (float us_per_sample, float tari);


class rfid_reader_decoder: public gr_sync_block{
 
  rfid_reader_decoder (float us_per_sample, float tari);

public: 
  ~rfid_reader_decoder();
  gr_msg_queue_sptr get_log() const;

  
};
