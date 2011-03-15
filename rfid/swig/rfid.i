/* -*- c++ -*- */

%include "gnuradio.i"			// the common stuff

%{
#include "rfid_clock_recovery_zc_ff.h"
#include "rfid_reader_decoder.h"
#include "rfid_center_ff.h"
#include "rfid_tag_decoder_f.h"
#include "rfid_command_gate_cc.h"
#include "rfid_reader_f.h"
%}

%include "rfid_clock_recovery_zc_ff.i"
%include "rfid_reader_decoder.i"
%include "rfid_center_ff.i"
%include "rfid_tag_decoder_f.i"
%include "rfid_command_gate_cc.i"
%include "rfid_reader_f.i"
