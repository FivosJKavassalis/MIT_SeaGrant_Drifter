# MIT Sea Grant Drifter Electronics

Student: Fivos Kavassalis --
Supervised by: Dr. Tom Consi

# Abstract

MIT Sea Grant is developing a Lagrangian drifting vehicle equipped with
sensors to measure some water parameters to support the lab’s research in ocean
acidification. Key to the operation of this vehicle is a reliable communications link
between the drifter and the lab. This will primarily involve the vehicle transmitting
its water quality data (pH, RTD, EC, DO), its position (GPS) within
Massachusetts Bay (the operating arena) and its motion and orientation data
(IMU). Therefore, a satellite and wireless communications system was developed
for the drifter. The former is established through the Iridium satellite constellation.
More specifically, the module implemented was the “RockBLOCK 9603” which
uses Iridium Short-Burst Data (SBD). The latter, uses the LoRaWAN protocol
(unlicensed radio spectrum to enable low power, wide area communication) and is
the “Feather 32u4 with LoRa module” microcontroller (915 MHz). Through
satellite, the drifter posts a message containing the water quality data collected
from the drifter’s sensors, the location and some information from the IMU to the
selected email accounts and to a web page. Through wireless communications, the
drifter is able to post a link to Google Maps to a web page with its location and is
able to interact with the user. Consequently, we can ask the drifter about its
location and/or any of its water quality data at any time and we will get a response
with the value that was last recorded from the drifter’s corresponding sensor. The
drifter also stores all the data gathered in an SD card.


# Introduction

The main focus of my internship was to build a communications system for
the drifter. Initially, my aim was to establish communication through satellite.
Therefore, I researched satellite communications, its fundamentals and the types
of satellite communications services available. I familiarized myself with the
“RockBLOCK 9603”, which is a satellite communications transceiver designed to
interface to small low-cost single board computers such as the Arduino. With this
knowledge, a system was created, including the RockBLOCK, a GPS and a
potentiometer imitating a sensor. The RockBLOCK was able to upload a message
with the pot value and the location to the selected email account(s) and web
page(s). Afterwards, a data logging shield was added in order to store all the data
added from the Arduino. Then, the potentiometer was replaced with the tentacle
shield, which is the base of the four water quality sensors that we want to measure.
The sensors are connected through I2C with the “master” Arduino. Later, I added a
“Feather 32u4 with LoRa module” to the drifter’s electronics in order to have more
of an immediate communication with it. The Feather was also connected through
I2C as a “slave”. In the next step we added an IMU and we stored motion and
orientation data. Some were also sent through the RockBLOCK. Finally, I did
research on microcontrollers that could possibly “fit” with our application and
learned about pH sensor circuits that work along pH Electrodes.

![a](https://user-images.githubusercontent.com/34138314/47153275-cdfd2d80-d2e7-11e8-84b1-1b77910695e4.PNG)
Figure 1. Implementation of Project


# Hardware Implementation

The circuit consists of an Arduino Mega 2560, along with a Data Logging
shield and the Tentacle Shield (hosts and isolates the 4 EZO sensor circuits from
Atlas Scientific, refer to Figures 2, 3 & 4), the RockBLOCK 9603 (external
antenna used through SMA connector , refer to Figure 6), two 32u4 Feathers (with
915 MHz LoRa modules, refer to Figure 8 for remote Feather), the GPS breakout
V3 (connected external antenna) and an IMU from Adafruit (Figure 1 & 2).
Eventually, the Traco Power “THN 15-1211” will be used, with probably a 12V,
5Ah battery. The Traco Power “THN 15-1211” is a DC/DC converter that was
chosen after considering my DMM measurements and each module’s datasheet (15
W, nominally 12 VDC Vin, 5V@3A). This converter was also chosen because of
its remote on/off control feature (e.g. Through transistor). This will power all of the
system except for the drifter’s Feather in order to have at least one form of
communication at all times.

The Arduino, the 4 water quality sensors, the IMU and the remote Feather
are connected through I2C (Figure 1 & 2) instead of UART, since it is faster, more
reliable, provides many privileges when coding, and needs only 2 pins from the
Arduino (instead of 2 for each module in UART). Furthermore, due to the use of
byte addressing in I2C, we can connect up to 127 devices simultaneously, which
may prove helpful for an improved version of this project. This connection has the
Arduino as the “master” and the rest of the modules as “slaves” (Figure 1). The
GPS and the RockBLOCK are connected through UART, since only this interface
has been available (Figure 1 & 2). Finally, the other Feather will be connected to
one of the lab’s computers so we can “talk” to the drifter through the LoRaWAN
protocol.

![bpng](https://user-images.githubusercontent.com/34138314/47154481-1f5aec00-d2eb-11e8-9976-1d9875d1a3dc.PNG)
Figure 2. Circuit without battery and converter

![c](https://user-images.githubusercontent.com/34138314/47154494-2a158100-d2eb-11e8-913b-15df0dcc84f1.PNG)
Figure 3 & 4. Arduino Mega with its two shields

# Software Implementation

For the programs developed, the Arduino and Processing IDEs were used.
The role of the Arduino in this project (Figure 5) was to gather all the data
accumulated from the sensors, meaning the 4 low fidelity sensor data (RTD, pH,
EC, DO), the location and time from the GPS, and motion/orientation data from the
IMU. Then, it sends the location (longitude, latitude) and the water quality sensor
values to the remote Feather. Moreover, the Arduino asks the RockBLOCK to send
all the data, except for a few vectors collected from the IMU due to the limitation
of Iridium’s Short-Burst Data (it can send/receive limited amount of bytes; see
RockBLOCK guide for more information). However, the Arduino Data Logging
Shield stores everything to a micro SD card.
The RockBLOCK 9603 is single-handedly responsible for the drifter’s
satellite communications, since it has a built-in Iridium 9603N module which
enables it to use Short-Burst Data, which is a bandwidth-limited messaging system
capable of transmitting packets up to 340 bytes and receiving up to 270 bytes. Its
purpose, from a software standpoint, is to post a message periodically, containing
water quality data, location, time and some IMU data to the selected web page(s)
and e-mail account(s). If the RockBlock is not available to send the desired
message at that moment due to lack of signal strength, it will eventually manage to
send it successfully along with any other message that was created after it. The
messages will be sent with the time that they collected their sensor values, and not
with the time they were sent. Please refer to the following flow chart for
clarification.

 ![d](https://user-images.githubusercontent.com/34138314/47154509-34377f80-d2eb-11e8-8840-296394308d8d.PNG)
![e](https://user-images.githubusercontent.com/34138314/47154525-3d285100-d2eb-11e8-969f-5c756b9cfe0a.PNG)
FIgure 5. Flow Diagram of Arduino Mega

![f](https://user-images.githubusercontent.com/34138314/47154534-44e7f580-d2eb-11e8-9ec8-8b138655d7f2.PNG)
Figure 6. Rockblock 9603 with external antenna


On the other hand, the Feather 32u4 with the 915 MHz radio module is great
for small data packet transmission when longer range is needed. It may be wise to
replace the wire antenna that we have now, by taking advantage of its uFL
connector, in order to obtain a longer line-of-sight. As stated earlier, there are two
Feathers; one at “home” and one “away”.
The “away” Feather’s role (Figure 7) is to receive the water sensor data and
location from Arduino periodically. To be more precise, it is sent once for each
message sent. Afterwards, it transmits the location to the “home” Feather and waits
for “home” Feather to request any water sensor datum or location, in order to reply
with its corresponding value. This Feather, also “refreshes” the location of the
drifter to the “home Feather” periodically. It will be powered independently from
the rest of the system, with a 3.7 V Lithium Polymer battery. This independence,
creates a window of communication between the lab and the drifter. Finally, it
informs the “home” Feather when it has low battery and informs the user if there
are connectivity issues.

![g](https://user-images.githubusercontent.com/34138314/47154555-4fa28a80-d2eb-11e8-93bd-df16fea1e553.PNG)
Figure 7. Flow Diagram for “away” Feather

![h](https://user-images.githubusercontent.com/34138314/47154576-5a5d1f80-d2eb-11e8-8ac4-65be713ccdb8.PNG)
Figure 8. “Away” (Remote) Feather 32u4 w/ LoRa module

The “home” Feather (Figure 9) periodically receives the drifter’s location
from the “away” Feather. Consequently, through the Processing IDE (Figure 10)
the serial port is read, and a link to Google Maps with the corresponding
coordinates is sent to a webpage (right now: drifterlocation.com). Furthermore, the
“home” Feather receives any water quality sensor value or location, depending on
its request. The “home” Feather also points out connectivity problems.

![apng](https://user-images.githubusercontent.com/34138314/47154904-46fe8400-d2ec-11e8-9b67-233b7df7db73.PNG)
Figure 9. Flow Diagram for “home” Feather (above)

![j](https://user-images.githubusercontent.com/34138314/47154612-719c0d00-d2eb-11e8-9dfe-9f3bb0987f39.PNG)

Figure 10. Flow Diagram for lab computer that will be used in order to see the drifter’s location on
Google Maps (above)


# Summary

In this project, a communications system was established with the drifter,
the lab and the internet (Figure 11). More specifically, the Arduino gathers all the
sensor data and stores them in an SD card. Furthermore, the location and four
water quality sensor values are transmitted to the “away” Feather and the Arduino
commands the RockBLOCK to send all the data gathered from the sensors (except
for a few vectors from the IMU) through the Iridium satellite constellation. The
“away” Feather’s functionality (after receiving the location and the water quality
sensor data) is that it sends the drifter’s location to the “home” Feather. Also, it is
waiting for a message from the “home” Feather to respond with either a sensor
value or with a reminder of the messages that enable the “away” Feather to return a
sensor value. Finally, it sends a message to the “home” Feather if it runs low on
battery. The “home” Feather receives the drifter’s location which is then uploaded
to a web page with a link to Google Maps along with the corresponding
coordinates. Moreover, it waits for the user to type in a request to send to the
“away” Feather in order to get a reply with a sensor value.

![a](https://user-images.githubusercontent.com/34138314/47153275-cdfd2d80-d2e7-11e8-84b1-1b77910695e4.PNG)
Figure 11. Hardware and Software Implementation Diagram


# Conclusion

This setup was tested many times (i.e. I have sent around 60 satellite
messages thus far) to make sure it works. Hopefully, it will also be tested in the
Massachusetts Bay when the other projects involved with the drifter are finalized,
to check its functionality in the actual operation arena. The next steps for this
project may be to add the Traco DC/DC converter along with the 12V battery and
to plug in the EC and DO stamps so we are able to receive values from those
sensors as well. Finally, one of the greatest takeaways from this project was
familiarizing myself more in depth with satellite communications and learning
about the LoRaWAN protocol.

# Documentation/Datasheets:

1. Arduino Mega 2560:
https://www.arduino.cc/en/Guide/ArduinoMega2560

2. Arduino Data Logger Shield:
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf

3. Whitebox Labs Arduino Tentacle Shield:
https://www.whiteboxes.ch/tentacle/?v=f214a7d42e0d#tentacle-t1

4. Adafruit Feather 32u4 with 915MHz Lora Radio Module:
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-feather-32u4-radio-with-lor
a-radio-module.pdf

5. RockBLOCK 9603:
https://docs.rockblock.rock7.com/docs

6. Adafruit Ultimate GPS Breakout v3:
https://learn.adafruit.com/adafruit-ultimate-gps/overview

7. Adafruit BNO055 Absolute Orientation Sensor:
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview

8. TRACO POWER THN 15-1211:
1.https://assets.tracopower.com/20180713155438/THN15/documents/thn15-datas
heet.pdf
2.https://assets.tracopower.com/20180713155438/THN15/documents/thn15-appli
cation.pdf
