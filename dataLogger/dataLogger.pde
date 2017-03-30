// data logging for master makers autonomous car
// Team Beetle Bot
// Press a key to save the data and close the program

import processing.serial.*;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.util.Date;

Serial myPort;
String val;
PrintWriter output;

void setup()
{
  size(200, 200);
  background(100);
  
  // On Windows machines, this generally opens COM1.
  String portName = Serial.list()[1]; //0 = COM3, 1 = COM6, 2, ...
  System.out.println(portName);
  myPort = new Serial(this, portName, 115200);
  
  DateFormat dateFormat = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss");
  Date date = new Date();
  String filename = dateFormat.format(date);
  System.out.println(filename);
  
  output = createWriter(filename + ".txt");
}

void draw()
{
  if ( myPort.available() > 0) {
    val = myPort.readStringUntil('\n');
    if(val != null && !val.isEmpty()) {
      output.print(val);
      print(val);
    }
  } 
}

void keyPressed() 
{
  if (key == 'q') {
    output.flush();
    output.close();
    exit();
  }
}