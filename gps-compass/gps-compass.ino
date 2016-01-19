#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <Wire.h>
#include "compass.h"

#define NUMPIXELS      16
#define PIXPIN         6
#define FIRSTPIX       6
#define Task_t 10          // Task Time in milli seconds

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXPIN, NEO_GRB + NEO_KHZ800);
// Connect the DIN for the neopixels to the pin specified as PIXPIN above
// Connect the compass to I2C 
// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;


Adafruit_GPS GPS(&mySerial);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  pinMode(13, OUTPUT);

  Wire.begin();
  pixels.begin(); 
  compass_x_offset = 122.17;
  compass_y_offset = 230.08;
  compass_z_offset = 389.85;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;
  
  compass_init(2);
  compass_debug = 1;
  compass_offset_calibration(3);
  
  theaterChaseRainbow(5);  
  digitalWrite(13,HIGH);  
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();



void loop()                    
{    
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

    if (GPS.fix) {

      //***************Below is the calculator for dist && bearing***************
      float R = 6371000; //radius of earth in meters
      float lat1 = 30.4018; //put your target's latitude here
      float lon1 = -97.8913; //put your target's longitude here
      float lat2 = GPS.latitudeDegrees;
      float lon2 = GPS.longitudeDegrees;

      //calculate bearing and remove negative degrees
      float brng = degrees(atan2((sin(radians(lon2-lon1)) * cos(radians(lat2))), (cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(lon2-lon1)))));
      while (brng < 0) 
        brng += 360;

      //calculate distance to target in meters
      float a = sin(radians(lat2-lat1)/2) * sin(radians(lat2-lat1)/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(radians(lon2-lon1)/2) * sin(radians(lon2-lon1)/2);
      float d = R * 2 * atan2(sqrt(a), sqrt(1-a));

      
      compass_heading();
      bearing = bearing + brng;
      while (bearing < 0) 
        bearing += 360;
      
      if((bearing >= 0 && bearing <= 11.25) || (bearing > 348.75 && bearing <= 360))
      {
        lightWay(0);
      }
      if(bearing > 11.25 && bearing <= 33.75)
      {
        lightWay(1);
      }
      if(bearing > 33.75 && bearing <= 56.25)
      {
        lightWay(2);
      }
      if(bearing > 56.25 && bearing <= 78.75)
      {
        lightWay(3);
      }
      if(bearing > 78.75 && bearing <= 101.25)
      {
        lightWay(4);
      }
      if(bearing > 101.25 && bearing <= 123.75)
      {
        lightWay(5);
      }
      if(bearing > 123.75 && bearing <= 146.25)
      {
        lightWay(6);
      }
      if(bearing > 146.25 && bearing <= 168.75)
      {
        lightWay(7);
      }
      if(bearing > 168.75 && bearing <= 191.25)
      {
        lightWay(8);
      }
      if(bearing > 191.25 && bearing <= 213.75)
      {
        lightWay(9);
      }
      if(bearing > 213.75 && bearing <= 236.25)
      {
        lightWay(10);
      }
      if(bearing > 236.25 && bearing <= 258.75)
      {
        lightWay(11);
      }
      if(bearing > 258.75 && bearing <= 281.25)
      {
        lightWay(12);
      }
      if(bearing > 281.25 && bearing <= 303.75)
      {
        lightWay(13);
      }
      if(bearing > 303.75 && bearing <= 326.25)
      {
        lightWay(14);
      }
      if(bearing > 326.25 && bearing <= 348.75)
      { 
        lightWay(15);
      }

    
  }
}

void lightWay(int offset) {
  int shift = FIRSTPIX + offset;
  int n = shift;
    while (n > 15) {n = n - 16;}
  int w = shift + 4;
    while (w > 15) {w = w - 16;}
  int s = shift + 8;
    while (s > 15) {s = s - 16;}
  int e = shift + 12;
    while (e > 15) {e = e - 16;}

 for (int i = 0; i < NUMPIXELS; i++)
 {
  pixels.setPixelColor(i, pixels.Color(0,0,0)); // Black/off
 }
    
 pixels.setPixelColor(n, pixels.Color(255,0,0)); // Bright red color.
 pixels.setPixelColor(s, pixels.Color(0,0,255)); // Moderately bright blue color.  
 pixels.setPixelColor(e, pixels.Color(255,255,255)); // Bright white color. 
 pixels.setPixelColor(w, pixels.Color(255,255,255)); // Bright white.  
 
 pixels.show(); // This sends the updated pixel color to the hardware.
}

//Below are the classes for the beginning light show and do not need to be changed (probably)
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixels.show();

      delay(wait);

      for (int i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
