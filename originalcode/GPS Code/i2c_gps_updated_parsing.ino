// Test code for Adafruit GPS That Support Using I2C
//
// This code shows how to parse data from the I2C GPS
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(0x10);  // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loop() // run over and over again
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.println("**************************");
    if (GPS.fix) {
      
      float latitude = GPS.latitude;
      int degrees = (latitude / 100);
      float minutes = latitude - (degrees*100);
      minutes = minutes / 60;
      float degreeval = degrees + minutes;
      char direction = GPS.lat;
      if (direction == 'N')
      {
        degreeval = degreeval;
      }
      if (direction == 'S')
      {
        degreeval = degreeval * -1;
      }
      Serial.print("latitude: "); Serial.println(degreeval, 6);

      //---------------------------------------

      float longitude = GPS.longitude;
      degrees = (longitude / 100);
      minutes = longitude - (degrees*100);
      minutes = minutes / 60;
      degreeval = degrees + minutes;
      direction = GPS.lon;
      if (direction == 'W')
      {
        degreeval = degreeval * -1;
      }
      if (direction == 'E')
      {
        degreeval = degreeval;
      }
      Serial.print("longitude: "); Serial.println(degreeval, 6);
      //Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);
      Serial.println("**************************");
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}