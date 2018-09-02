#include <IridiumSBD.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



#define BNO055_SAMPLERATE_DELAY_MS (1000) // Set the delay between fresh samples 
#define GPSSerial Serial1 // UART between Arduino and GPS 
#define IridiumSerial Serial3 // UART between Arduino and RockBLOCK 
#define DIAGNOSTICS true // true if we want to see diagnostics 
#define IDLE_TIME 120000 // 2 minute time period between messages  
#define GPSECHO  false  // Set to true in order to see the raw GPS sentences in the Serial console 
#define oneWireBus 51 // pin of temperature sensor

// Declare the GPS and IridiumSBD objects
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_GPS GPS(&GPSSerial);
IridiumSBD modem(IridiumSerial);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

const int chipSelect = 10; 

boolean usingInterrupt = false; // initializing as false. Checks if interrupt is used (Program manipulates it accordingly in the useInterrupt function)


/* For water quality sensors */
char sensordata[30];                  // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0;       // We need to know how many characters bytes have been received

byte code = 0;                        // used to hold the I2C response code.
byte in_char = 0;                     // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

#define TOTAL_CIRCUITS 2              // CHANGE THIS TO 4 (RTD, pH, EC, DO) when everything is calibrated (Set the 
                                      //number of I2C circuits that are attached to the Tentacle shield(s))

int channel_ids[] = {99, 102};        // 100 is EC, 97 is DO (order matters!). It 
                                      // is a list of I2C ids that you set your circuits to.


char *channel_names[] = {"PH", "RTD"}; // ADD "EC" AND "DO"! List of channel names to give a name to each sensor ID (must be the same order as in channel_ids[]) 


// Function prototypes: 
void useInterrupt(boolean); 
void initializeSD();
void initializeIMU();
void initializeGPS();
void initializeRB();
void NOTuseInterrupt();
void writeToSerial(imu::Vector<3> accelerometer, imu::Vector<3> magnetometer, imu::Vector<3> gyroscope, imu::Vector<3> euler, imu::Vector<3> linearaccel, imu::Vector<3> gravity, imu::Quaternion quat, char forRTD[30], char forPH[30]); // add other 2 (EC and DO)...
void writeToSD(imu::Vector<3> accelerometer, imu::Vector<3> magnetometer, imu::Vector<3> gyroscope, imu::Vector<3> euler, imu::Vector<3> linearaccel, imu::Vector<3> gravity, imu::Quaternion quat, char forRTD[30], char forPH[30]); // add other 2 (EC and DO)...
String msgContent(imu::Vector<3> accelerometer, imu::Vector<3> magnetometer, imu::Vector<3> gyroscope, imu::Vector<3> euler, imu::Vector<3> linearaccel, imu::Vector<3> gravity, imu::Quaternion quat, char forRTD[30], char forPH[30]);
void sendMessage(const char * txt);
void transmitToFeather(char forRTD[30], char forPH[30]);
void parseNMEA();


void setup() {


  Wire.begin();         // enable I2C port.                          
  Serial.begin(115200); // Start the console serial port
   
  while (!Serial); // for debugging -- Comment out when not needed!


  // Initialize modules
  initializeSD();
  initializeIMU();
  initializeGPS();
  initializeRB();  
  
} // void setup()




// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
 // (for debugging purposes)
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  // writing direct to UDR0 instead of Serial.print... 
#endif
}


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() -- just interrupt somewhere in
    // the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {   // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
} // void useInterrupt(boolean v)


uint32_t timer = millis();



void loop() {

  char pH[30] = {0};
  char EC[30] = {0};
  char RTD[30] = {0};
  char DO[30] = {0};

   
 /* For IMU */
 Adafruit_BNO055 bno = Adafruit_BNO055(55);
  
   /* Unified Sensor System output */
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);  
  imu::Quaternion quat = bno.getQuat();// easier and more accurate to work with than Euler angles when doing sensor fusion or data manipulation with raw sensor data


/* Gathers water quality sensor values */
  for (int channel = 0; channel < TOTAL_CIRCUITS; channel++) {       // loop through all the sensors
  
    Wire.beginTransmission(channel_ids[channel]);     // call the circuit by its ID number.
    Wire.write('r');                          // request a reading by sending 'r'
    Wire.endTransmission();                         // end the I2C data transmission.
    
    delay(1000);  // AS circuits need a 1 second before the reading is ready

    sensor_bytes_received = 0;                        // reset data counter
    memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;

    Wire.requestFrom(channel_ids[channel], 48, 1);    // call the circuit and request 48 bytes (this is more then we need).
    code = Wire.read();

    while (Wire.available()) {          // are there bytes to receive?
      in_char = Wire.read();            // receive a byte.

      if (in_char == 0) {               // null character indicates end of command
        Wire.endTransmission();         // end the I2C data transmission.
        break;                          // exit the while loop, we're done here
      }
      else {
        sensordata[sensor_bytes_received] = in_char;      // append this byte to the sensor data array.
        sensor_bytes_received++;
      }
    }
    
    Serial.print(channel_names[channel]);   // print channel name
    Serial.print(':');

    switch (code) {                       // switch case based on what the response code is.
      case 1:                             // decimal 1  means the command was successful.
        Serial.println(sensordata);       // print the actual reading
        break;                              // exits the switch case.

      case 2:                             // decimal 2 means the command has failed.
        Serial.println("command failed");   // print the error
        break;                              // exits the switch case.

      case 254:                           // decimal 254  means the command has not yet been finished calculating.
        Serial.println("circuit not ready"); // print the error
        break;                              // exits the switch case.

      case 255:                           // decimal 255 means there is no further data to send.
        Serial.println("no data");          // print the error
        break;                              // exits the switch case.
    }

    // Stores water sensor values in char arrays (will be used in msg, sd, serial)
    
      if(channel==0){
        strncpy(pH, sensordata, 30);
      } else if(channel==1){
        strncpy(RTD, sensordata, 30);
      } 
      // ADD EC and DO, to replace it with: 
      // if(channel==0){
      //  strncpy(pH, sensordata, 30);
      // } else if(channel==1){
      //  strncpy(RTD, sensordata, 30);
      // } else if(channel==2){
      //  strncpy(EC, sensordata, 30);
      // } else if(channel==3){
      //  strncpy(DO, sensordata, 30);
      // } 

    //-------------------------------------------------------------------------------

  } // for loop 


  NOTuseInterrupt(); // if not using interrupt
  parseNMEA();       // parse NMEA sentences from GPS
 

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approx every 2 seconds, print out the current stats
  if (millis() - timer > 2000) { 
      timer = millis(); // reset timer

    transmitToFeather(RTD,pH); // Send water quality sensor values -- ADD EC and DO arrays when sensors are plugged in!          
              
    /* Write to Serial Monitor and SD card */ 
    if (GPS.fix) { // if we get a fix from the GPS
        writeToSerial(accelerometer,magnetometer,gyroscope,euler,linearaccel,gravity,quat,RTD,pH);
        writeToSD(accelerometer,magnetometer,gyroscope,euler,linearaccel,gravity,quat,RTD,pH);
    }  
  }  
 
  /* Sending the message */
  String DATA = msgContent(accelerometer,magnetometer,gyroscope,euler,linearaccel,gravity,quat,RTD,pH); // Create the message from sensor values gathered 
  const char * text = DATA.c_str();
  Serial.print("This is the message: ");  
  Serial.print(DATA);  
  sendMessage(text);

  delay(IDLE_TIME); // Idle for a constant time 
  
  // HERE USE TRANSISTOR TO TURN OFF SYSTEM WHEN WE GET CONVERTER

} // void loop()


/* See if the card is present and can be initialized */
void initializeSD() {
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything else:
    return;
  }
  Serial.println(" SD card is initialized.");
}  


/* Initializes IMU and check that its plugged n correctly */
void initializeIMU() {

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecing the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
} 


/* Initializes GPS' baud rate, rate for each command read */
void initializeGPS(){

  GPS.begin(9600); // GPS default baud rate

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);    // turns on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // turns on ONLY the "minimum recommended" data
  // Parser uses only for RMC only or RMC+GGA 

  // Set update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate -- good rate for parser...

  GPS.sendCommand(PGCMD_ANTENNA); // Requests updates on antenna status

  // Timer0 interrupt goes off every 1 millisecond, and reads data from the GPS 
  useInterrupt(true);

  delay(1000);
  
  GPSSerial.println(PMTK_Q_RELEASE);   // Ask for firmware version

}

/* Initializes RockBLOCK, checks signal quality */
void initializeRB(){

  int signalQuality = -1;
  int err;

  IridiumSerial.begin(19200);   // Start the serial port connected to the modem


  // Begin satellite modem process
  Serial.println("Starting modem...");
  
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  }


  // Prints the firmware revision
  char version[12];
  err = modem.getFirmwareVersion(version, sizeof(version));
  if (err != ISBD_SUCCESS)
  {
     Serial.print("FirmwareVersion failed: error ");
     Serial.println(err);
     return;
  }
  Serial.print("Firmware Version is ");
  Serial.print(version);
  Serial.println(".");

  // Tests the signal quality -- Range between (including) 0, 5
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is currently ");
  Serial.print(signalQuality);
  Serial.println(".");

} 

/*In case not using the interrupt above,
it is needed to 'hand query' the GPS */ 
void NOTuseInterrupt() {
 
  if (! usingInterrupt) {
    // read data from the GPS
    char c = GPS.read();
    // (for debugging purposes)
    if (GPSECHO)
      if (c) Serial.print(c);
  }
}

/** 
 *  Writes all data gathered from all sensors to serial monitor 
 *  @param imu::Vector<3> accelerometer   accelerometer vector reading
 *  @param imu::Vector<3> magnetometer  magnetometer vector reading
 *  @param imu::Vector<3> gyroscope  gyroscope vecor reading
 *  @param imu::Vector<3> euler  euler vector reading
 *  @param imu::Vector<3> linearaccel  linear accelaration vector reading
 *  @param imu::Vector<3> gravity  gravity vector reading
 *  @param imu::Quaternion quat  quat reading
 *  @param char forRTD[30] RTD sensor reading
 *  @param char forpH[30] pH sensor reading
 *  @param char forEC[30] EC sensor reading
 *  @param char forDO[30] DO sensor reading
**/
void writeToSerial(imu::Vector<3> accelerometer, imu::Vector<3> magnetometer, imu::Vector<3> gyroscope, imu::Vector<3> euler, imu::Vector<3> linearaccel, imu::Vector<3> gravity, imu::Quaternion quat, char forRTD[30], char forPH[30]) {
        
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    
      //SERIAL PRINT MAIN SENSOR VALUES HERE
      Serial.print("pH: "); Serial.println(forPH);
      //Serial.print("EC: "); Serial.println(EC);
      Serial.print("Tempetature: "); Serial.print(forRTD); Serial.println(" C");
      //Serial.print("DO: "); Serial.println(DO);
      
      // accelerometer
      Serial.print("Accelerometer (m/s^2): ");  
      Serial.print("X: ");
      Serial.print(accelerometer.x(), 4);
      Serial.print(" Y: ");
      Serial.print(accelerometer.y(), 4);
      Serial.print(" Z: ");
      Serial.println(accelerometer.z(), 4);

      // magnetometer
      Serial.print("Magnetometer (uT): ");  
      Serial.print("X: ");
      Serial.print(magnetometer.x(), 4);
      Serial.print(" Y: ");
      Serial.print(magnetometer.y(), 4);
      Serial.print(" Z: ");
      Serial.println(magnetometer.z(), 4);

      // gyroscope
      Serial.print("Gyroscope (rps): ");
      Serial.print("X: ");
      Serial.print(gyroscope.x(), 4);
      Serial.print(" Y: ");
      Serial.print(gyroscope.y(), 4);
      Serial.print(" Z: ");
      Serial.println(gyroscope.z(), 4);
  
      // euler
      Serial.print("Euler orientation (degrees -- 0-359): ");
      Serial.print("X: ");
      Serial.print(euler.x(), 4);
      Serial.print(" Y: ");
      Serial.print(euler.y(), 4);
      Serial.print(" Z: ");
      Serial.println(euler.z(), 4);
    
      // linearaccel
      Serial.print("Linearaccel (m/s^2): ");
      Serial.print("X: ");
      Serial.print(linearaccel.x(), 4);
      Serial.print(" Y: ");
      Serial.print(linearaccel.y(), 4);
      Serial.print(" Z: ");
      Serial.println(linearaccel.z(), 4);
    
      // gravity
      Serial.print("Gravity (m/s^2): ");
      Serial.print("X: ");
      Serial.print(gravity.x(), 4);
      Serial.print(" Y: ");
      Serial.print(gravity.y(), 4);
      Serial.print(" Z: ");
      Serial.println(gravity.z(), 4);
    
    
      
      // Quaternion data
      Serial.print("Quaternion orientation: ");
      Serial.print("qW: ");
      Serial.print(quat.w(), 4);
      Serial.print(" qX: ");
      Serial.print(quat.x(), 4);
      Serial.print(" qY: ");
      Serial.print(quat.y(), 4);
      Serial.print(" qZ: ");
      Serial.print(quat.z(), 4);
      Serial.println("\n\n");
      


//      Serial.print(Celcius); Serial.print(" C or "); Serial.print(Fahrenheit); Serial.println(" F");

}

/** 
 *  Writes all data gathered from all sensors to SD card
 *  @param imu::Vector<3> accelerometer   accelerometer vector reading
 *  @param imu::Vector<3> magnetometer  magnetometer vector reading
 *  @param imu::Vector<3> gyroscope  gyroscope vecor reading
 *  @param imu::Vector<3> euler  euler vector reading
 *  @param imu::Vector<3> linearaccel  linear accelaration vector reading
 *  @param imu::Vector<3> gravity  gravity vector reading
 *  @param imu::Quaternion quat  quat reading
 *  @param char forRTD[30] RTD sensor reading
 *  @param char forpH[30] pH sensor reading
 *  @param char forEC[30] EC sensor reading
 *  @param char forDO[30] DO sensor reading
**/
void writeToSD(imu::Vector<3> accelerometer, imu::Vector<3> magnetometer, imu::Vector<3> gyroscope, imu::Vector<3> euler, imu::Vector<3> linearaccel, imu::Vector<3> gravity, imu::Quaternion quat, char forRTD[30], char forPH[30]) {

     /* Open the csv file to save this data */
      File sDFile = SD.open("potetgps.csv", FILE_WRITE); 
 
      if (sDFile) {
        
          sDFile.print("\nTime: ");
          sDFile.print(GPS.hour, DEC); sDFile.print(':');
          sDFile.print(GPS.minute, DEC); sDFile.print(':');
          sDFile.print(GPS.seconds, DEC); sDFile.print('.');
          sDFile.println(GPS.milliseconds);
          sDFile.print("Date: ");
          sDFile.print(GPS.day, DEC); sDFile.print('/');
          sDFile.print(GPS.month, DEC); sDFile.print("/20");
          sDFile.println(GPS.year, DEC);
          sDFile.print("Fix: "); sDFile.print((int)GPS.fix);
          sDFile.print(" quality: "); sDFile.println((int)GPS.fixquality);


 
          sDFile.print("Location: ");
          sDFile.print(GPS.latitude, 4); sDFile.print(GPS.lat);
          sDFile.print(", "); 
          sDFile.print(GPS.longitude, 4); sDFile.println(GPS.lon);
          sDFile.print("Location (in degrees, works with Google Maps): ");
          sDFile.print(GPS.latitudeDegrees, 4);
          sDFile.print(", "); 
          sDFile.println(GPS.longitudeDegrees, 4);
        
          sDFile.print("Speed (knots): "); sDFile.println(GPS.speed);
          sDFile.print("Angle: "); sDFile.println(GPS.angle);
          sDFile.print("Altitude: "); sDFile.println(GPS.altitude);
          sDFile.print("Satellites: "); sDFile.println((int)GPS.satellites);
          //SD PRINT MAIN SENSOR VALUES HERE
          sDFile.print("pH: "); sDFile.println(forPH);
          //sDFile.print("EC: "); sDFile.println(EC);
          sDFile.print("Tempetature: "); sDFile.print(forRTD); sDFile.println(" C");         
          //sDFile.print("DO: "); Serial.println(DO);

          // accelerometer
          sDFile.print("Accelerometer (m/s^2): ");  
          sDFile.print("X: ");
          sDFile.print(accelerometer.x(), 4);
          sDFile.print(" Y: ");
          sDFile.print(accelerometer.y(), 4);
          sDFile.print(" Z: ");
          sDFile.println(accelerometer.z(), 4);
    
          // magnetometer
          sDFile.print("Magnetometer (uT): ");  
          sDFile.print("X: ");
          sDFile.print(magnetometer.x(), 4);
          sDFile.print(" Y: ");
          sDFile.print(magnetometer.y(), 4);
          sDFile.print(" Z: ");
          sDFile.println(magnetometer.z(), 4);
    
          // gyroscope
          sDFile.print("Gyroscope (rps): ");
          sDFile.print("X: ");
          sDFile.print(gyroscope.x(), 4);
          sDFile.print(" Y: ");
          sDFile.print(gyroscope.y(), 4);
          sDFile.print(" Z: ");
          sDFile.println(gyroscope.z(), 4);
      
          // euler
          sDFile.print("Euler orientation (degrees -- 0-359): ");
          sDFile.print("X: ");
          sDFile.print(euler.x(), 4);
          sDFile.print(" Y: ");
          sDFile.print(euler.y(), 4);
          sDFile.print(" Z: ");
          sDFile.println(euler.z(), 4);
        
          // linearaccel
          sDFile.print("Linearaccel (m/s^2): ");
          sDFile.print("X: ");
          sDFile.print(linearaccel.x(), 4);
          sDFile.print(" Y: ");
          sDFile.print(linearaccel.y(), 4);
          sDFile.print(" Z: ");
          sDFile.println(linearaccel.z(), 4);
        
          // gravity
          sDFile.print("Gravity (m/s^2): ");
          sDFile.print("X: ");
          sDFile.print(gravity.x(), 4);
          sDFile.print(" Y: ");
          sDFile.print(gravity.y(), 4);
          sDFile.print(" Z: ");
          sDFile.println(gravity.z(), 4);
        
        
          
          // Quaternion data
          sDFile.print("Quaternion orientation: ");
          sDFile.print("qW: ");
          sDFile.print(quat.w(), 4);
          sDFile.print(" qX: ");
          sDFile.print(quat.x(), 4);
          sDFile.print(" qY: ");
          sDFile.print(quat.y(), 4);
          sDFile.print(" qZ: ");
          sDFile.print(quat.z(), 4);
          sDFile.println("\n\n");
              
          sDFile.close();
      } else {
          Serial.println("error opening potetgps.csv");
        }
   

}    

/** 
 *  Creates message to be sent through satellite, including gps, water quality data and some IMU data.
 *  (Add EC and DO arrays to parameters of function and to the string returned) Message must be max 340 bytes!
 *  (Add EC and DO arrays to parameters of function and to the string returned) Message must be max 340 bytes!
 *  @param imu::Vector<3> accelerometer   accelerometer vector reading
 *  @param imu::Vector<3> magnetometer  magnetometer vector reading
 *  @param imu::Vector<3> gyroscope  gyroscope vecor reading
 *  @param imu::Vector<3> euler  euler vector reading
 *  @param imu::Vector<3> linearaccel  linear accelaration vector reading
 *  @param imu::Vector<3> gravity  gravity vector reading
 *  @param imu::Quaternion quat  quat reading
 *  @param char forRTD[30] RTD sensor reading
 *  @param char forpH[30] pH sensor reading
 *  @param char forEC[30] EC sensor reading
 *  @param char forDO[30] DO sensor reading
 *  @return String  message to be sent through satellite
**/
String msgContent(imu::Vector<3> accelerometer, imu::Vector<3> magnetometer, imu::Vector<3> gyroscope, imu::Vector<3> euler, imu::Vector<3> linearaccel, imu::Vector<3> gravity, imu::Quaternion quat, char forRTD[30], char forPH[30]){
    
    String dataString = "pH:";  
    dataString += String(forPH);
    //dataString += "EC:";
    //dataString += String(EC);
    //dataString += "DO:";
    //dataString += String(DO);
    dataString += "RTD:";
    dataString += String(forRTD);
    dataString += "C";
  
    dataString += "Location(in deg->G.Maps):";
    dataString += String(GPS.latitudeDegrees);
    dataString += ",";
    dataString += String(GPS.longitudeDegrees); 
  
    
    dataString += String("Time:");
    dataString += String(GPS.hour);
    dataString += ":";
    dataString += String(GPS.minute);
    dataString += ":";
    dataString += String(GPS.seconds); 
    dataString += "UTC "; 
  
    // accelerometer
    dataString += "Accelerometer (m/s^2)=>";  
    dataString += "X:";
    dataString += String(accelerometer.x());
    dataString += "Y:";
    dataString += String(accelerometer.y()); 
    dataString += "Z:";
    dataString += String(accelerometer.z());
  
    // magnetometer
    dataString += "Magnetometer (uT)=>";  
    dataString += "X:";
    dataString += String(magnetometer.x());
    dataString += "Y:";
    dataString += String(magnetometer.y()); 
    dataString += "Z:";
    dataString += String(magnetometer.z());
  
    // gyroscope
    dataString += "Gyroscope (rps)=>";  
    dataString += "X:";
    dataString += String(gyroscope.x());
    dataString += "Y:";
    dataString += String(gyroscope.y()); 
    dataString += "Z:";
    dataString += String(gyroscope.z());
  
    // euler
    dataString += "Euler orientation (degrees -- 0-359)=>";
    dataString += "X:";
    dataString += String(euler.x());
    dataString += "Y:";
    dataString += String(euler.y());
    dataString += "Z:";
    dataString += String(euler.z());
    
    // commented out because msg was too long, but info is still saved in the sd card
  //  // linearaccel
  //  dataString += "Linearaccel (m/s^2)=> ";  
  //  dataString += "X: ";
  //  dataString += String(linearaccel.x());
  //  dataString += " Y: ";
  //  dataString += String(linearaccel.y()); 
  //  dataString += " Z: ";
  //  dataString += String(linearaccel.z());
  
      // commented out because msg was too long, but info is still saved in the sd card
  //  // gravity
  //  dataString += "Gravity (m/s^2)=> ";  
  //  dataString += "X: ";
  //  dataString += String(gravity.x());
  //  dataString += " Y: ";
  //  dataString += String(gravity.y()); 
  //  dataString += " Z: ";
  //  dataString += String(gravity.z());
  
    // commented out because msg was too long, but info is still saved in the sd card
  //  // Quaternion data
  //  dataString += "Quaternion orientation=> ";  
  //  dataString += "qW: ";
  //  dataString += String(quat.w());
  //  dataString +=" qX: ";
  //  dataString += String(quat.x()); 
  //  dataString += " qY: ";
  //  dataString += " qZ: ";
  //  dataString += String(quat.z());
  return dataString;
    
}


/** 
 *  Sends message through satellite 
 *  @param const char * txt pointer to start of message to be sent
**/
void sendMessage(const char * txt){
  int err = ISBD_SUCCESS;
  Serial.print("Trying to send this message. This might take several minutes.\r\n");
  Serial.print(txt);

  // create a pointer to the output string (sendSBDText function takes in a pointer...)
  err = modem.sendSBDText(txt);

  //Inform if sending the message was a failure or a success
  if (err != ISBD_SUCCESS)
  {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println("Try again. Probably a better view of the sky is needed...");
  } else {
    Serial.println("It worked!");
  }
}

/** 
 *  Sends location and water quality sensor values to drifter's feather 
 *  @param char forRTD[30] RTD sensor reading
 *  @param char forpH[30] pH sensor reading
 *  @param char forEC[30] EC sensor reading
 *  @param char forDO[30] DO sensor reading
**/
void transmitToFeather(char forRTD[30], char forPH[30]) {
    char latitudeBuff[20] = {0};
    char longitudeBuff[20] = {0};
    char buff[40] = {0};

    dtostrf(GPS.latitudeDegrees, 0, 4, latitudeBuff);
    dtostrf(GPS.longitudeDegrees, 0, 4, longitudeBuff);
    strncpy(buff, latitudeBuff, 7);
    for(int j=0; j<sizeof(longitudeBuff); j++) {
      buff[7+j] = longitudeBuff[j]; 
    }
    
    Wire.beginTransmission(8); // transmit to device #8
            Wire.write("a");        // sends 13 bytes
            Wire.write(forRTD);              // sends 
            Wire.write("b");        // sends 13 bytes
            Wire.write(forPH);              // sends  
            // add EC, DO when plugged in
        //      Wire.write("c");        
        //      Wire.write(EC);
        //      Wire.write("d");        
        //      Wire.write(DO);
            Wire.write("e");        // sends 13 bytes
            Wire.write(buff);      // sends   
            Wire.endTransmission();    // stop transmitting
}            

/* parse NMEA sentence */
void parseNMEA(){
  
   // if a sentence is received, we can check the checksum and parse it...
  if (GPS.newNMEAreceived()) {
    // If we print the NMEA sentence, or data
    // we end up not listening and catching other sentences, so might cause a problem when  
    // using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}


//------------------DIAGNOSTICS-----------------------
#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
