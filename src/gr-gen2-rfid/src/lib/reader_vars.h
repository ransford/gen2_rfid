#ifndef READER_VARS
#define READER_VARS

const bool DEBUG_ON = false;
const bool LOGGING = true;

const bool CHANGE_Q = false;
const bool FIND_ALL_TAGS = false;
const int NUM_TAGS = 1;


const int CYCLE_RATE = 100; //in milliseconds

const int num_rounds = 1;
const int num_cycles = 1000;

const int MAX_INPUT_ITEMS = 3072 / 2;
const int SIG_DETECT_THRESH = 2; //SNR thresh for detecting signal



const int tag_freq = 41; //kHZ
//const int tag_freq = 28; //kHZ
//const int tag_freq = 256; //kHZ WISP

const int reader_delim = 12;  //usec
const int reader_pw = 12;
const int reader_tari = 24;

const int reader_rtcal = 72;  //28 or 40
//const int reader_rtcal = 60; //WISP

//const int reader_trcal = 287; //28 kHz
//const int reader_trcal = 75; //WISP
const int reader_trcal = 192;  //40 kHz



static float fm0_preamble[] = {1,1,-1,1,-1,-1,1,-1,-1,-1,1,1};
static float m8_preamble[] = {1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,
			      1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1,
			      -1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,
			      -1,1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1,
			      1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1,
			      -1,1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1};
static float m4_preamble[] = {1,-1,1,-1,1,-1,1,-1,
			      1,-1,1,-1,-1,1,-1,1,
			      -1,1,-1,1,-1,1,-1,1,
			      -1,1,-1,1,1,-1,1,-1,
			      1,-1,1,-1,-1,1,-1,1,
			      -1,1,-1,1,1,-1,1,-1};
static float m2_preamble[] = {1,-1,1,-1,
			      1,-1,-1,1,
			      -1,1,-1,1,
			      -1,1,1,-1,
			      1,-1,-1,1,
			      -1,1,1,-1};
static float fm0_one_vec[] = {1,1};
static float m2_one_vec[] = {1,-1,-1,1};
static float m4_one_vec[] = {1,-1,1,-1,-1,1,-1,1};
static float m8_one_vec[] = {1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1};

//lens in pulses (i.e., 1/2 cycle)
const int len_fm0_preamble = 12;
const int len_fm0_one = 2;
const int len_m2_one = 4;
const int len_m4_one = 8;
const int len_m8_one = 16;
const int len_m2_preamble = 24; 
const int len_m4_preamble = 48; 
const int len_m8_preamble = 96; 
const int max_tag_response_len = 512;
const int num_RN16_bits = 16 ;
const int num_EPC_bits = 128 ;

static char * q_params[] = {"0000","0001","0001","0010","0010","0010","0011","0011"};


enum command {IDLE, QUERY, ACK, QREP, NAK_QREP};

struct state{
  command last_cmd;
  bool gated;
  double avg_rssi;
  double max_rssi;
  double min_rssi;
  double std_dev_rssi;
  double SNR;
  
  //This should be used to tell cmd_gate how many pulses to look for
  //  This is currently hardcoded. 
  int num_pulses_in_cmd;

  int tag_one_len;
  int tag_preamble_len;
  float * tag_preamble;
  float * tag_one;

  char CMD[5];
  char DR[2];
  char M[3];
  char tr_ext[2];
  char sel[3];
  char session[3];
  char target[2];
  char Q[5];
  char CRC[6];

  bool found_preamble;
  int num_bits_to_decode;
  int num_bits_decoded;
  bool send_query;
  bool start_cycle;
  
  bool bit_error;
  bool bit_error_on_previous;
  
  int cur_slot;
  int num_slots;

  int round;
  int cycle;

  int num_tags_found;

  //These are deprecated but used by rfid_command_gate.cc
  bool reset_detector;
  bool signal_detected;


};



#endif
