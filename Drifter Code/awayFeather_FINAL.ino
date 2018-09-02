#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>


/* for Feather32u4 RFM9x */
#define RFM95_CS 8 
#define RFM95_RST 4
#define RFM95_INT 7


#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"
 
#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A
 
#elif defined(NRF52)  
  /* nRF52832 feather w/wing */
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif
 
 
#define RF95_FREQ 915.0 
 
/* Instance of the radio driver */
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
/* Blinky on receipt */
#define LED 13

/* To read LiPoly voltage and check if feather needs recharging */
#define VBATPIN A9

//Function protoypes:
void isLowBattery(float nowVbat);
void sendLocation();
void receiveRequest();
void radioPreparation();

  int k = 0; // used as index for sensor arrays
    char pHval[20] = {0};
//    char EC[30] = {0};
//    char DO[30] = {0};
    char RTDval[20] = {0};
    char gpsFeather[30] = {0};
 
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);           // start serial 

  while (!Serial) { // debugging! Comment out when not needed!
  }
  delay(100);
 
  radioPreparation();
  
} // void setup()


void loop() {       
  
  if(gpsFeather[1] != 0){ // To make sure we get msg from arduino first, before we do anything

    sendLocation();
         
    delay(1000);


    for (int x=0; x < 30000; x++) {         
      delay(1);
      Serial.print(x);
      receiveRequest();
   }
  }

  float Vbat = analogRead(VBATPIN);
  isLowBattery(Vbat);

} // void loop()



/* Initializes, sets up LoRa*/
void radioPreparation(){

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
 
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  
}



/* function that executes whenever data is received from master
 this function is registered as an event, see setup() -- Code accordingly for EC and
 DO when they are plugged in */
void receiveEvent(int howMany) {
              
   while (Wire.available() > 1){
    char c = Wire.read();
                   
     while(c != 'b') {
       Serial.print(c);
       RTDval[k] = c; 
       k+=1;
       c = Wire.read();
     }
     if(c == 'b') {
         Serial.print(c);
         RTDval[k] = '\0';
         k=0;
         pHval[k] = c;
         k+=1;
         c = Wire.read();
     }
    while(c != 'e'){
      Serial.print(c);
      pHval[k] = c;
      k+=1;
      c = Wire.read();
    }
    
    if(c == 'e'){
        Serial.print(c);
        pHval[k] = '\0';
        k=0;
        gpsFeather[k] = c;
        k+=1;
        c = Wire.read();  
    }
     while(Wire.available() > 1){
        Serial.print(c);
        gpsFeather[k] = c;
        k+=1;
        c = Wire.read();
      }  
   }
     int x = Wire.read();    // receive byte as an integer
     gpsFeather[k] = x;
     Serial.println((char)x);         // print the integer
     k+=1;
     gpsFeather[k] = '\0';

}


/** 
 *  Writes all data gathered from all sensors to serial monitor 
 *  @param float nowVbat  Voltage of battery at present time
**/
void isLowBattery(float nowVbat){

    nowVbat *= 2;    // we divided by 2, so multiply back 
    nowVbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    nowVbat /= 1024; // convert to voltage
  
  if(nowVbat < 3.7) { 
    // Send msg when feaher's battery gives less than 3.7 V
    Serial.print("VBat: " ); Serial.println(nowVbat);
    uint8_t data[] = "Feather has low battery (Less than 3.7 V)!";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
    digitalWrite(LED, LOW);
  }
}


/* Sends the location to the pther Feather */
void sendLocation(){

    // Send the location in degrees (latitude, longitude) no matter what -- works
    //  with Google Maps 
    uint8_t data[sizeof(gpsFeather)];
    for(int i=0; i<sizeof(gpsFeather); i++){
      data[i] = gpsFeather[i+1]; // 6 -- temporary fix!
    }
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
    Serial.println("Sent the location");
    digitalWrite(LED, LOW);

}

/* Responds with a sensor value to a request of the "home" Feather
  The requests are: RTD
                    pH 
                    EC
                    DO
                    LOC 
*/
void receiveRequest(){
  if (rf95.available()){
        
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
     
        if (rf95.recv(buf, &len)) {
          
          digitalWrite(LED, HIGH);
          RH_RF95::printBuffer("Received: ", buf, len);
          Serial.print("Got: ");
          Serial.println((char*)buf);
          if ((buf[0] == 'R') && (buf[1] == 'T') && (buf[2] == 'D')){
    
              // Send the RTD sensor value
              uint8_t data[sizeof(RTDval)];
              for(int i=0; i<sizeof(RTDval); i++){
                data[i] = RTDval[i+1]; // 6 -- temporary fix!
              }
              rf95.send(data, sizeof(data));
              rf95.waitPacketSent();
              Serial.println("Sent a reply");
              digitalWrite(LED, LOW);
          } else if ((buf[0] == 'p') && (buf[1] == 'H')){
            
              // Send the pH sensor value
              uint8_t data[sizeof(pHval)];
              for(int i=0; i<sizeof(pHval); i++){
                data[i] = pHval[i+1];
              }
              rf95.send(data, sizeof(data));
              rf95.waitPacketSent();
              Serial.println("Sent a reply");
              digitalWrite(LED, LOW);
            
          } else if ((buf[0] == 'E') && (buf[1] == 'C')){
    
              // Send the EC sensor value
              uint8_t data[] = "EC SENSED!";
              rf95.send(data, sizeof(data));
              rf95.waitPacketSent();
              Serial.println("Sent a reply");
              digitalWrite(LED, LOW);
             
          } else if ((buf[0] == 'D') && (buf[1] == 'O')){
    
             // Send the DO sensor value
              uint8_t data[] = "DO SENSED!";
              rf95.send(data, sizeof(data));
              rf95.waitPacketSent();
              Serial.println("Sent a reply");
              digitalWrite(LED, LOW);
            
          } else if ((buf[0] == 'L') && (buf[1] == 'O') && (buf[2] == 'C')){
    
              // Send the location in degrees (latitude, longitude) -- works with
              // Google Maps 
              uint8_t data[sizeof(gpsFeather)];
              for(int i=0; i<sizeof(gpsFeather); i++){
                data[i] = gpsFeather[i+1]; // 6 -- temporary fix!
              }
              rf95.send(data, sizeof(data));
              rf95.waitPacketSent();
              Serial.println("Sent a reply");
              digitalWrite(LED, LOW);
            
          } else {
    
             // Send a reminder to the user about the appropriate syntax 
             // of the questions
              uint8_t data[] = "For each sensor, type whatever is in each bracket to get a value: [RTD], [pH], [EC], [DO], [LOC]";
              rf95.send(data, sizeof(data));
              rf95.waitPacketSent();
              Serial.println("Sent a reply");
              digitalWrite(LED, LOW);
            
          }
            
         
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
  
        }
        else
        {
          Serial.println("Receive failed or nothing is sent");
        }
      }
}        

