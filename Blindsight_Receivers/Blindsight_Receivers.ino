
#define MOTOR_CENTER        6       // Motor 2 output
#define MOTOR_LEFT          9       // Motor 1 output
#define MOTOR_RIGHT         5       // Motor 3 output

char status_bit = 1;

motor_list[] = {MOTOR_RIGHT, MOTOR_CENTER, MOTOR_LEFT};

void setup() {
  
}

void loop() {
// if there is data ready
  uint8_t pipe_num;
  if ( radio.available(&pipe_num) )
  {
    // Dump the payloads until we've gotten everything
    int[3] intensity;
    bool done = false;
    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      done = radio.read( &intensity, sizeof(int[3]) );

      // Spew it
      printf("Got payload %lu from node %i...",intensity,pipe_num+1);
    }
    
    // First, stop listening so we can talk
    radio.stopListening();

    /* *** Manage the intensity of the motors *** */
    int i;
    for(i = 0; i<3; i++){
      digitalWrite(motor_list[i], intensity[i]);
    }
    
    /* *** Reply to base with status bit 1 *** */
    // Open the correct pipe for writing
    radio.openWritingPipe(listening_pipes[pipe_num-1]);

    // Retain the low 2 bytes to identify the pipe for the spew
    uint16_t pipe_id = listening_pipes[pipe_num-1] & 0xffff;

    // Send the final one back.
    radio.write( &status_bit, sizeof(char) );
    printf("Sent response to %04x.\n\r",pipe_id);

    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }
}
