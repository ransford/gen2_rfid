/* -*- c++ -*- */
 
%include "exception.i"
%include "gnuradio.i"                            // the common stuff
  
%{
#include "gnuradio_swig_bug_workaround.h"       // mandatory bug fix
#include "rfid_clock_recovery_zc_ff.h"
#include "rfid_cmd_gate.h"
#include "rfid_gen2_reader.h"
%}
  

//-----------------------------------------------------------------
GR_SWIG_BLOCK_MAGIC(rfid, cmd_gate);

rfid_cmd_gate_sptr rfid_make_cmd_gate (int dec, state * reader_state);


class rfid_cmd_gate: public gr_block{
 
  rfid_cmd_gate (int dec, state * reader_state);

public: 
  ~rfid_cmd_gate();

 
  
};


//-----------------------------------------------------------------
GR_SWIG_BLOCK_MAGIC(rfid, gen2_reader);

rfid_gen2_reader_sptr rfid_make_gen2_reader (int dec, int interp);


class rfid_gen2_reader: public gr_block{
 
  rfid_gen2_reader (int dec, int interp);

public: 
  ~rfid_gen2_reader();
  state * STATE_PTR;
  gr_msg_queue_sptr get_log() const;

 
  
};

//-----------------------------------------------------------------
GR_SWIG_BLOCK_MAGIC(rfid, clock_recovery_zc_ff);

rfid_clock_recovery_zc_ff_sptr 
rfid_make_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);

class rfid_clock_recovery_zc_ff: public gr_block{
  rfid_clock_recovery_zc_ff(int samples_per_pulse, int interp_factor);

public:
  ~rfid_clock_recovery_zc_ff();
  
};


