#include "rfid_global_vars.h"
#include <cstdio>
#include <stdlib.h>


reader_state * global_reader_state;

void init_global_reader_state(){
  
 
  if(!global_reader_state){
    global_reader_state = (reader_state *)malloc(sizeof(reader_state));
    global_reader_state->tag_bit_vector = (char*) malloc(512 * sizeof(char));
    global_reader_state->command_gate_status = GATE_RESET;
    global_reader_state->num_bits_to_decode = 17;
    global_reader_state->num_bits_decoded = 0;
    global_reader_state->num_samples_per_pulse = NUM_SAMPLES_PER_PULSE;
    global_reader_state->num_samples_to_ungate = 0;

    
  }
 
}
