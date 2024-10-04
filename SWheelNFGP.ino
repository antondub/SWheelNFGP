const unsigned long bound_rate = 9600;
const unsigned int period = 232;

uint8_t _receivePin;
uint8_t _receiveBitMask;
volatile uint8_t *_receivePortRegister;
uint8_t _transmitBitMask;
volatile uint8_t *_transmitPortRegister;

void setTX(uint8_t tx);
int cWrite(uint16_t data[], byte data_size);
int serial_pause(int no_bits);
size_t write(uint16_t b);
inline void tunedDelay(uint16_t delay);

//SoftwareSerial lin_serial(5,7,false);
//lin_stack LIN1(&lin_serial, ident);

uint8_t tX_Pin = 7;

uint8_t u_Pin = 11;
uint8_t d_Pin = 10;
uint8_t ok_Pin = 12;

uint8_t r_Pin = 8;
uint8_t l_Pin = 9;

void setup() {
  // Configure Serial communication
  setTX(tX_Pin);
  pinMode(tX_Pin, OUTPUT);
  digitalWrite(tX_Pin,HIGH);

  pinMode(l_Pin, INPUT_PULLUP);
  pinMode(r_Pin, INPUT_PULLUP);
  pinMode(u_Pin, INPUT_PULLUP);
  pinMode(d_Pin, INPUT_PULLUP);
  pinMode(ok_Pin, INPUT_PULLUP);


  Serial.begin(115200); // Configure Serial for Serial Monitor
  //LIN1.setSerial(); // Configure Serial for receiving
}

uint16_t s_data_q[] = {0x401,0x401,0x401,0x81,0xA9,0x129};
uint16_t s_data_r[] = {0x401,0x101,0x401,0x81,0xA9,0x229};
uint16_t s_data_u[] = {0x401,0x401,0x401,0x485,0xA9,0x52D};
uint16_t s_data_ok[] = {0x401,0x041,0x401,0x81,0xA9,0x569};
uint16_t s_data_d[] = {0x401,0x401,0x401,0x5,0xA9,0x4AD};
uint16_t s_data_l[] = {0x401,0x81,0x401,0x81,0xA9,0x5A9};

void loop() {
  if(digitalRead(r_Pin)==LOW)
  {
    while(digitalRead(r_Pin)==LOW) tunedDelay(20);
    cWrite(s_data_r,6);  
  }
  if(digitalRead(l_Pin)==LOW)
  {
    while(digitalRead(l_Pin)==LOW) tunedDelay(20);
    cWrite(s_data_l,6);  
  }
  if(digitalRead(ok_Pin)==LOW)
  {
    while(digitalRead(ok_Pin)==LOW) tunedDelay(20);
    cWrite(s_data_ok,6);  
  }
  if(digitalRead(u_Pin)==LOW)
  {
    while(digitalRead(u_Pin)==LOW) tunedDelay(20);
    cWrite(s_data_u,6);  
  }
  if(digitalRead(d_Pin)==LOW)
  {
    while(digitalRead(d_Pin)==LOW) tunedDelay(20);
    cWrite(s_data_d,6);  
  }

  // cWrite(s_data_q,6);
  // delay(1);
  // cWrite(s_data_r,6);
  // delay(1);
  // cWrite(s_data_ok,6);
  // delay(1);
  // cWrite(s_data_l,6);
  // delay(41);
  // Checking LIN Bus periodicly
  //byte a = LIN1.read(data, data_size);
  // if(a == 1){ // If there was an event on LIN Bus, Traffic was detected. Print data to serial monitor
  //   Serial.println("Request Received!");
  //   Serial.print("Data : ");
  //   Serial.print(data[0], HEX);
  //   Serial.print(data[1], HEX);
  //   Serial.print(data[2], HEX);
  //   Serial.print(data[3], HEX);
  //   Serial.println(" ");

  // }else if(a == -1){ // Ident and Checksum validation Failed
  //   Serial.println("Corrupt Request Received!");
  // }
}

int cWrite(uint16_t data[], byte data_size){

  //serial_pause(9);

  //begin(bound_rate);

  for(int i=0;i<data_size;i++) write(data[i]); // write data to serial
  //write(checksum); // write Checksum Byte to serial
  //end(); // clear Serial config

  return 1;
}

int serial_pause(int no_bits){
  
  unsigned int del = period*no_bits;
  digitalWrite(tX_Pin,LOW);
  tunedDelay(del); 
  digitalWrite(tX_Pin,HIGH);
  return 1;
}

inline void tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}
void tx_pin_write(uint8_t pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}
void setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

// void setRX(uint8_t rx)
// {
//   pinMode(rx, INPUT);
//   if (!_inverse_logic)
//     digitalWrite(rx, HIGH);  // pullup for normal logic!
//   _receivePin = rx;
//   _receiveBitMask = digitalPinToBitMask(rx);
//   uint8_t port = digitalPinToPort(rx);
//   _receivePortRegister = portInputRegister(port);
// }

size_t write(uint16_t b)
{
  uint8_t oldSREG = SREG;
  cli();
  uint16_t mask = 0x01;
  // tx_pin_write(HIGH);
  // tunedDelay(period+15);

  for ( int i = 0 ; i < 11; i++ )
  {
      if (b & mask)
        tx_pin_write(HIGH);
      else
        tx_pin_write(LOW); 
      mask <<= 1;
      if(i==0)tunedDelay(period+10);
      else tunedDelay(period);
  }

  tx_pin_write(HIGH);
  SREG = oldSREG;
  //tunedDelay(period);
  
  return 1;
}
