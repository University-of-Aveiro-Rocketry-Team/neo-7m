#include <VMA430_GPS.h>     // Include the GPS module library
#include <HardwareSerial.h> // Include the software serial library

HardwareSerial ss(2); // GPS RX to ESP 17, GPS TX to ESP 16
VMA430_GPS gps(&ss);     // Pass the softwareserial connection info the the GPS module library

void setup()
{
  Serial.begin(115200);
  Serial.println("hello");
  gps.begin(9600); // Sets up the GPS module to communicate with the Arduino over serial at 9600 baud
  gps.setUBXNav(); // Enable the UBX mavigation messages to be sent from the GPS module
}

void loop()
{
  if (gps.getUBX_packet()) // If a valid GPS UBX data packet is received...
  {
    gps.parse_ubx_data(); // Parse the new data
    if (gps.utc_time.valid) // If the utc_time passed from the GPS is valid...
    {
      // Print UTC time hh:mm:ss
      Serial.println();
      Serial.print("UTC time: ");
      Serial.print(gps.utc_time.hour);
      Serial.print(":");
      Serial.print(gps.utc_time.minute);
      Serial.print(":");
      Serial.print(gps.utc_time.second);
      Serial.println();
    }
    // Print location (latitude/longitude)
    Serial.println();
    Serial.print("location: ");
    Serial.print("Lat: ");
    Serial.print(gps.location.latitude, 8); // to 8 decimals
    Serial.print(" Long: ");
    Serial.print(gps.location.longitude, 8); // to 8 decimals
  }

  delay(10);
}
