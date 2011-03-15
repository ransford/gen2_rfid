#ifndef READER_VARS
#define READER_VARS

#include <string.h>

//This configures the reader to start a cycle every XXX ms. 
// If false, the reader will start a new cycle immediately after one ends
const bool TIMED_CYCLE_MODE = true;  
const int READER_CYCLE_TIMER_RATE = 500; //XXX ms
////////

//These specify how many cycles, and how many rounds in each cycle, before the reader stops
// Note that if CHANGE_Q is true, a cycle ends when all tags are read. 
const int READER_NUM_CYCLES = 5;  //Number of Cycles, i.e. Power downs
const int READER_NUM_ROUNDS = 1;    //How many Queries per cycle. Only used if !CHANGE_Q. 
////////

//These specify how Q is changed
// If CHANGE_Q, then change Q based on the number of tags. If False, just look at READER_NUM_ROUNDS
// Optimal_Q chooses the optimal frame size based on tags read - NUM_TAGS
// !Optimal_Q, then use the EPC Q algorithm
const bool CHANGE_Q = false;  //If false, use a fixed number of rounds. Else, round ends when all tags are read
const bool OPTIMAL_Q = false; 
const int NUM_TAGS = 2;       //The number of tags in the population
/////////



const bool LOGGING = true;  //Generally, you want to log things.
const bool READ_DATA = false; //READ data after singulation. Currently, not fully implemented. 

const float COLLISION_THRESHOLD = 2;   //SNR to detect collisions, NOT in dB
const float INIT_QFP = 0;              //Initial Q value

//USRP1 automatically sends the CW, at least with the current GNURadio.
// USRP2, or if GNUradio changes, this will send out the CW manually. 
// If a tag is near the edge of the spec, the timing of the manual CW may be off.
const bool TRANSMIT_CW = true;  

enum {QUERY, ACK, QREP, NAK, REQ_RN, READ, IDLE};  //Gen 2 state machine
enum {GATE_RESET, GATE_OPEN, GATE_CLOSED, GATE_DISABLED, GATE_FINISHED};  //State of command_gate
enum {TIMER_FIRED, BITS_DECODED, NO_PREAMBLE};  //Messages from tag_decoder to reader_f
enum {DECODER_SEEK_PREAMBLE, DECODER_PREAMBLE_FOUND, DECODER_FINISHED, DECODER_STARTED, DECODER_CLEAR_PIPE};//State of tag_decoder

//This is for passing info, triggering changes, etc between blocks. 
struct reader_state{
  int last_cmd_sent;

  int command_gate_status;
  int decoder_status;
  int num_bits_to_decode;
  int num_bits_in_preamble;
  int num_bits_decoded;
  int num_pulses_per_bit;
  int num_samples_per_pulse;
  int num_samples_per_bit;
  int num_samples_to_ungate;
  float T1_value;
  float us_per_rcv;

  bool nak_sent;

  int num_cycles; 
  int cur_cycle;

  int num_rounds;
  int cur_round;

  int num_slots;
  int cur_slot;

  float * tag_preamble_cor_vec;
  float * tag_one_cor_vec;
  int tag_preamble_cor_vec_len, tag_one_cor_vec_len;

  char * tag_bit_vector;

  double min_pwr, max_pwr, std_dev_noise, std_dev_signal, avg_pwr; 

  
  

};

extern reader_state * global_reader_state;

extern void init_global_reader_state();



const static char * q_strings[] = {"0000","0001","0010","0011","0100","0101","0110","0111", "1000"};

const static char * opt_q_strings[] = {"0000","0001","0001","0010","0010","0010","0011","0011",
                                       "0011","0011","0011","0011","0100","0100","0100","0100"};

const int delim_width = 12; // usec
const int pulse_width = 12;
const int tari_width = 12;

//For 40 kHz uplink
const int trcal_width = 200;  //for 40 kHz
const int rtcal_width = 72;  //28 or 40
const int NUM_SAMPLES_PER_PULSE = 10;     


//For WISP. Remember to use M4 only
//const int rtcal_width = 60; //WISP
//const int trcal_width = 75; //WISP
//const int NUM_SAMPLES_PER_PULSE = 4;



/* static float m8_preamble_vec[] = {1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1, */
/* 			             1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1, */
/* 			             -1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1, */
/* 			             -1,1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1, */
/* 			             1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1, */
/* 			             -1,1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1}; */

/* static float m4_preamble_vec[] = {1,-1,1,-1,1,-1,1,-1, */
/* 			      1,-1,1,-1,-1,1,-1,1, */
/* 			      -1,1,-1,1,-1,1,-1,1, */
/* 			      -1,1,-1,1,1,-1,1,-1, */
/* 			      1,-1,1,-1,-1,1,-1,1, */
/* 			      -1,1,-1,1,1,-1,1,-1}; */

/* static float m2_preamble_vec[] = {1,-1,1,-1, */
/* 			      1,-1,-1,1, */
/* 			      -1,1,-1,1, */
/* 			      -1,1,1,-1, */
/* 			      1,-1,-1,1, */
/* 			      -1,1,1,-1}; */


static float m8_preamble_vec[] = {1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,
				  1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,
				  1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1};



static float m4_preamble_vec[] = {1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,
				  1,1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,1,1,-1,-1,1,1,-1,-1,
				  1,1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,1,1,-1,-1,1,1,-1,-1};



static float m2_preamble_vec[] = {1,1,-1,-1,1,1,-1,-1,
				  1,1,-1,-1,-1,-1,1,1,
				  -1,-1,1,1,-1,-1,1,1,
				  -1,-1,1,1,1,1,-1,-1,
				  1,1,-1,-1,-1,-1,1,1,
				  -1,-1,1,1,1,1,-1,-1};


//static float m2_data_one_vec[] = {1,-1,-1,1};
//static float m4_data_one_vec[] = {1,-1,1,-1,-1,1,-1,1};
//static float m8_data_one_vec[] = {1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1};
static float m2_data_one_vec[] = {1,1,-1,-1,-1,-1,1,1};
static float m4_data_one_vec[] = {1,1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1};
static float m8_data_one_vec[] = {1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,1};

//lens in pulses (i.e., 1/2 cycle)
const int m2_one_len = 8;
const int m4_one_len = 16;
//const int m4_one_len = 8;
const int m8_one_len = 32;
const int m2_preamble_len = 48; 
const int m4_preamble_len = 96; 
//const int m4_preamble_len = 48; 
const int m8_preamble_len = 192; 
const int max_tag_response = 512;
const int no_RN16_bits = 17 ;  
const int no_EPC_bits = 129 ;
const int no_DATA_bits = 48;
const int usrp_pkt_size = 128;  //In samples. 512 / size of complex. 

#endif
