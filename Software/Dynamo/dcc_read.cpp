#ifndef DCC_READ_H
  #include "dcc_read.h"
#endif

/* NMRA allows up to 32 bytes per packet, the max length would be 301 bits transmitted and need 38 bytes (3 bits extra). 
 * DCC_EX is limited to only 11 bytes max. Much easier to account for and uses a smaller buffer. 
  */

dccrx dcc; //Define DCC handler object
bool dcc_ok = false; 
extern uint64_t time_us;


extern volatile uint8_t edge_stream;
extern volatile uint8_t period_stream; 
uint8_t last_edge_stream; 
extern volatile uint8_t edge_count;

//extern volatile uint16_t edge_time[DCC_RX_BUF]; 
//extern volatile uint8_t edge_time_int; 

void dccrx::loop_process(){
  uint8_t i = 0; 
  if (edge_count > 1) {
    edge_count = 0;
    last_edge_stream = edge_stream; 

    
    if (((last_edge_stream & 0x0F) == 0x0D) || ((last_edge_stream & 0x0F) == 0x07)) {
      last_rx_time = TIME_US;
      rx_bit_processor(1);
    }
    if (((last_edge_stream & 0x0F) == 0x08) || ((last_edge_stream & 0x0F) == 0x02)) {
      last_rx_time = TIME_US;
      rx_bit_processor(0);
    }
/*    if ((last_edge_stream & 0x01) == ((last_edge_stream >> 2) & 0x01)) {
      rx_bit_processor((last_edge_stream & 0x01));
    }*/

/*    
  if (edge_time_int > 4) {
  while (i < edge_time_int) {
    Serial.printf(" %u ", edge_time[i]);
    i++;  
  }
  Serial.printf(" \n");
  edge_time_int = 0; 
  }
*/        
  }
  if ((TIME_US - last_rx_time) < 100000) {
    if (dcc_ok != true) {
      Serial.printf("DCC OK \n");
     dcc_ok = true;
    }
  } else {
    if (dcc_ok != false) {
      Serial.printf("DCC NOT OK\n");
      dcc_ok = false;
    }
  }
  rx_queue(); 
  return;
}

uint8_t dccrx::rx_bit_processor(bool input){ //Append the input bit onto the data in memory, assembling it into packets. 
  //Serial.printf("DCCRX Bit processor %u, rx_pending %i \n", uint8_t(input), rx_pending);
  //Preamble detect
  if (input == 1) {
    consecutive_ones++; 
  }
  if ((input == 0) && (consecutive_ones >= 12) && (rx_pending < 0)) { //12 1 bits including last stop + 0 bit + no pending = pending
    //Found start bit. Reset counters to sort what follows into a new empty packet. 
    last_preamble = TIME_US; //Store the time of the last preamble
    rx_pending = rx_packet_getempty();
    rx_packets[rx_pending]->state = 1;
    rx_num_bits = 0; //Start counting new byte
    rx_num_bytes = 0; //packet byte 0
    rx_byteout = 0; // empty output byte
    consecutive_ones = 0; //Preamble count
    //Serial.printf("New Packet %u started \n", rx_pending); 

    return 1; //Packet was created and will start populating on the next call    
  }
  //Serial.printf("DCCRX Bit processor %u \n", uint8_t(input));
  if (rx_pending < 0) { 
    if (input == 0) {
      consecutive_ones = 0; 
      //Serial.printf("Got a 0, resetting consecutive 1s. rx_pending %i \n", rx_pending); 
    }
    //Serial.printf("DCCRX rx_bit_processor: No packet, so no need to continue this cycle. \n"); 
    return 0; //No packet, so no need to continue. Return to bit detector. 
  }
  //Bit Counting
  //Serial.printf("dcc: rx_num_bits %u, rx_num_bytes %u \n", rx_num_bits, rx_num_bytes); 
  if (rx_num_bits < 8) { //Valid data in bits 0-7, bit 8 indicates next byte (0) or end of data (1)
    rx_byteout = rx_byteout << 1; //Shift right to make room. 
    rx_byteout = rx_byteout | input; //OR the new bit onto the byte, since the bit shift added a zero at the end.
    
  } else { //rx_num_bits >= 8, copy the finished byte into the packet and check if the packet is complete. 
    if (rx_num_bytes > 48) {
      rx_num_bytes = 48; //truncate so it doesn't crash. 
    }
    rx_packets[rx_pending]->data_len = rx_num_bytes;         
    rx_packets[rx_pending]->packet_data[rx_packets[rx_pending]->data_len] = rx_byteout;
    rx_packets[rx_pending]->packet_time = TIME_US; //Update time this packet last received a bit
    //Serial.printf("dccrx: Byte %x complete, %u in packet %u \n", rx_byteout, rx_num_bytes, rx_pending); 
    if (input == 1) { //Bit 8 is 1, mark packet complete. Checksum it and reset rx_pending. 
      if (rx_packets[rx_pending]->Read_Checksum()) { //Read checksum, true if valid false if bad.
        Serial.printf("dccrx: Completed packet %u, checksum ok \n", rx_pending);
        rx_packets[rx_pending]->state = 3; //packet rx complete
        rx_packets[rx_pending]->packet_time = TIME_US;
      } else { //Checksum was invalid, discard packet. 
        Serial.printf("dccrx: Completed packet %u, checksum bad \n", rx_pending);
        rx_packets[rx_pending]->state = 4; //packet rx failed, mark for deletion.
      }
      //rx_packets[rx_pending]->reset_packet(); //Dump it for now. 
      rx_pending = -1;
    }
    rx_num_bytes++; //Increment byte counter
    rx_num_bits = 0; //Reset rx_num_bits
  }
  rx_num_bits++; 
  return 1;
}
void dccrx::rx_queue() { //Process queue of packets checking expirations, processing complete, and clearing failed/done
  uint8_t i = 0; 
  for(i = 0; i < DCC_RX_Q; i++) {
    if (!rx_packets[i]) { //No packet defined, skip this slot. 
      continue;
    }
    if (rx_packets[i]->state == 0) { //Empty slot
      continue; 
    }
    if ((rx_packets[i]->state == 1) || (rx_packets[i]->state == 2)) { //Pending packet or receiving packet, check last update timetime
      time_us = TIME_US;
      if ((time_us - rx_packets[i]->packet_time) > 100000) { //32 bytes * 9 bits per * 106uS. Should cover most situations. 
        rx_packets[i]->state = 4; //Mark as failed packet so it gets pruned. 
      }
    }
    if (rx_packets[i]->state == 3) { //Complete packet, process it. 
      rx_decode(i); //Decode the packet. 
    }   
    
    if ((rx_packets[i]->state == 4) || (rx_packets[DCC_RX_Q]->state == 5)) { //Packet failed or processed, remove.
      if (i == rx_pending) {
        rx_pending = -1; 
      }
      rx_packets[i]->reset_packet();
    }
  }

  return;
}

void dccrx::rx_decode(uint8_t rx_pkt) {
  if (!(rx_packets[rx_pkt])) { //Was called on an invalid index. Leave.    
    return; 
  }
  if (rx_packets[rx_pkt]->state != 3) { //Was called on a packet that isn't complete. Leave it.  
    return; 
  }
  rx_packets[rx_pkt]->state = 5; //mark complete. 
  return;
}

uint8_t dccrx::rx_packet_getempty(){ //Scan rx_packets and return 1st empty slot
  uint8_t count = 0; 
  while (count < DCC_RX_Q){  
    if (!rx_packets[rx_next_new]){ //Isn't initialized yet, fix this
      rx_packets[rx_next_new] = new DCC_packet();
    }
    if (rx_packets[rx_next_new]->state == 0) { //Exists and is marked empty, claim it.  
      break; //break out of the while loop and use this result. 
    }
    rx_next_new++; 
    if (rx_next_new > DCC_RX_Q) {
      rx_next_new = 0;
    }
  }
  if (count == DCC_RX_Q) { //Checked all slots without a result
    Serial.printf("WARNING: rx_packets out of space. RX packet %d will be overwritten, consider increasing LN_RX_Q \n", rx_next_new);
  }
  if (rx_next_new == rx_pending) {
    rx_pending = -1;
  }
  rx_packets[rx_next_new]->reset_packet(); //Sets it to defaults again   
  return rx_next_new;
}

void dccrx::dccrx_init(){
  rx_pending = -1; 
  return;
}

void dccrx_init() {
  dcc.dccrx_init(); 
  return;
}

void dccrx_loop() { //Reflector into dccrx::loop_process();
  dcc.loop_process(); 
  return;
}

/*************************
 * dccpacket definitions *
 *************************/
void DCC_packet::Make_Checksum(){ //Populate last byte with valid checksum

  return;
}
bool DCC_packet::Read_Checksum(){ //Verify checksum, returns true if valid, false if invalid.
  char xsum = 0x00;
  uint8_t i = 0;
  i = 0;
  Serial.printf("dccrx Checksum: %x ", xsum); 
    for (i = 0; i < (data_len); i++){
    xsum = xsum ^ packet_data[i];
    Serial.printf("%x ", xsum);
  }
  Serial.printf(" \n"); 
  if (xsum == 0) {
    return true;
  }
  return false; 
}
uint8_t DCC_packet::packet_size_check(){ //Check that a packet has a valid size. 
  uint8_t packet_size = data_len;

  return packet_size;
}
void DCC_packet::reset_packet(){ //Reset packet slot to defaults
  state = 0;
  data_len = 2;
  packet_time = 0;
  return;
}
