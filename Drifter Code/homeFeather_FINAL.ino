#include <SPI.h>
#include <RH_RF95.h>
 
/* for feather32u4 */
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
 
/* Singleton instance of the radio driver */
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define LED 13 

// Function prototypes:
void radioPreparation();
void receiveLocation();
void sendRequest(char dataPacket[20]);
 
void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  Serial.begin(115200);
  while (!Serial) {  // for debugging
    delay(1);
  }
 
  delay(100);
  radioPreparation();
  
} // void setup()
 


void loop() {
  
    delay(1000); 
    
    receiveLocation();
        
     
    delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
    int availableBytes = Serial.available();
    
    if(availableBytes > 0) { 
      Serial.println("Transmitting..."); // Send a message to rf95_server
    
      char radiopacket[20] = "                   ";
      for(int i=0; i<availableBytes; i++) {
          radiopacket[i] = Serial.read();
      }
      Serial.print("Sending "); Serial.println(radiopacket);
      radiopacket[19] = 0;
      sendRequest(radiopacket);
    }
} // void loop()

/* Initializes, sets up LoRa*/
void radioPreparation() {

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

/* receives location from "away" Feather */
void receiveLocation(){

    if (rf95.available()){
    
    // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
              
              digitalWrite(LED, HIGH);
    //          RH_RF95::printBuffer("Received: ", buf, len);
    //          Serial.print("Got: ");
              Serial.println((char*)buf); // try String, if it works, fits with processing code...
        } 
    }
}

/** 
 *  Sends message with request of a sensor value (RTD, pH, EC, DO, LOC)
 *  @param char dataPacket[20]  message to be sent to "away" Feather
**/
void sendRequest(char dataPacket[20]) {

    
    Serial.println("Sending...");
    delay(10);
    rf95.send((uint8_t *)dataPacket, 20);
   
    Serial.println("Waiting for packet to complete..."); 
    delay(10);
    rf95.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    Serial.println("Waiting for reply...");
    if (rf95.waitAvailableTimeout(1000))
    { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
     {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reply, is there a listener around?");
    }
  
}
  
