import processing.serial.*;

Serial myPort;  // The serial port

char [] gpsCoords;
PrintWriter output;

void setup() {
  gpsCoords = new char [14];
  // Lists all the available serial ports
  printArray(Serial.list());
  // Opens the port to be used at the rate we want  
  myPort = new Serial(this, Serial.list()[2], 115200); // Change accordingly! 
  
}
       
void draw() {
  
  int i=0;
  while (myPort.available() > 0) {
    int inByte = myPort.read();
    println((char)inByte);
    // build array of gps coordinates
   if(i < gpsCoords.length){ 
     gpsCoords[i] = (char)inByte; 
       i++; 
   } 
  }
  //println(gpsCoords);
  
  // when we receive coordianates from the "away" feather:
  while(gpsCoords[2] == '.'){
    // Create a new file in the sketch directory
      output = createWriter("DATA/index.html"); 
      output.println("<h1> MIT Sea Grant Drifter Location </h1>");   
      output.println("<br><br>");
      output.println("<a href=\"https://www.google.com/maps/place/");
    
      //Turns characters to strings for the html file
      String string = "";
      String gps1 = String.valueOf(gpsCoords[0]);
      String gps2 = String.valueOf(gpsCoords[1]);
      String gps3 = String.valueOf(gpsCoords[2]);
      String gps4 = String.valueOf(gpsCoords[3]);
      String gps5 = String.valueOf(gpsCoords[4]);
      String gps6 = String.valueOf(gpsCoords[5]);
      String gps7 = String.valueOf(gpsCoords[6]);
      String gps8 = String.valueOf(gpsCoords[7]);
      String gps9 = String.valueOf(gpsCoords[8]);
      String gps10 = String.valueOf(gpsCoords[9]);
      String gps11= String.valueOf(gpsCoords[10]);
      String gps12 = String.valueOf(gpsCoords[11]);
      String gps13 = String.valueOf(gpsCoords[12]);
      String gps14 = String.valueOf(gpsCoords[13]);  
    
    
      string += gps1;
      string += gps2;
      string += gps3;
      string += gps4;
      string += gps5;
      string += gps6;
      string += gps7;
      string += ","; // or single apostrophess
      string += gps8;
      string += gps9;
      string += gps10;
      string += gps11;
      string += gps12;
      string += gps13;
      string += gps14;
    
      output.println(string);
      output.println("\">Google Maps</a>");
      output.flush(); // Writes the remaining data to the file
      output.close(); // Finishes the file
      exit(); // Stops the program
  }  
  
  println("not ready yet !!!"); // "home" feather has not received coordinates from
                                // "away" feather yet  
}
  
