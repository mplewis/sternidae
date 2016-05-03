#include "ILI9341_t3.h"
#include "SPI.h"
#include <math.h>

// Hardware
// TFT LCD
const int TFT_DC = 9;
const int TFT_CS = 10;
ILI9341_t3 lcd = ILI9341_t3(TFT_CS, TFT_DC);

// A physical location
typedef struct Location {
  double lat;
  double lng;
} Location;

// A name attached to a physical location
typedef struct Destination {
  char name[16];
  Location loc;
} Destination;

// Waypoints to be used for navigation
Destination dests[] = {
  {"Test", {0.0, 10.0}},
  {"Home", {45.022884, -93.244983}},
  {"Work", {44.989136, -93.250454}},
};

// For calculating distance
// Earth's radius, in mi
const double EARTH_RADIUS = 3959;
// Radians per degree
const double RADS_PER_DEG = 2 * 3.14159 / 360;

// Updated by on_new_gps
// Location
Location current_loc = {0};
// Time in milliseconds
unsigned long current_time = 0;
unsigned long last_time = 0;
double delta_time = 0;
// Distance in mi
double current_dist = 0;
double last_dist = 0;
double delta_dist = 0;
// Bearing in degrees
double current_bearing = 0;
// ETA in milliseconds. If negative, we're traveling away from current_dest
double eta_time = 0;

// For averaging ETA
const int avg_count = 10;
double avg_buf[avg_count] = {0};
int avg_buf_pos = 0;

// Change current_dest to change the distance and eta
Location current_dest = dests[0].loc;

// Uses Haversine formula
// http://www.movable-type.co.uk/scripts/gis-faq-5.1.html
// http://andrew.hedges.name/experiments/haversine/
double distance(Location from, Location to) {
  double dlat = (to.lat - from.lat) * RADS_PER_DEG;
  double dlng = (to.lng - from.lng) * RADS_PER_DEG;
  double a = pow(sin(dlat/2), 2) + cos(from.lat) * cos(to.lat) * pow(sin(dlng/2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return EARTH_RADIUS * c;
}

// http://www.movable-type.co.uk/scripts/latlong.html
// http://forum.arduino.cc/index.php?topic=45760.0
// returns bearing in degrees
double bearing(Location from, Location to) {
  double lat1 = from.lat * RADS_PER_DEG;
  double lng1 = from.lng * RADS_PER_DEG;
  double lat2 = to.lat * RADS_PER_DEG;
  double lng2 = to.lng * RADS_PER_DEG;

  double bearing = atan2(
    sin(lng2-lng1) * cos(lat2),
    (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(lng2-lng1))
  ) / RADS_PER_DEG;

  // use mod to turn -90 = 270
  return fmod((bearing + 360.0), 360);
}

// Stores a value for the sliding window average
void put_avg(double val) {
  avg_buf[avg_buf_pos] = val;
  avg_buf_pos = (avg_buf_pos + 1) % avg_count;
}

// Gets the average of all stored values
double get_avg() {
  double avg = 0;
  for (int i = 0; i < avg_count; i++) {
    avg += avg_buf[i] / avg_count;
  }
  return avg;
}

// Called when the GPS has a new location
void on_new_gps(double lat, double lng) {
  current_loc.lat = lat;
  current_loc.lng = lng;

  last_time = current_time;
  current_time = millis();
  delta_time = current_time - last_time;

  last_dist = current_dist;
  current_dist = distance(current_loc, current_dest);
  delta_dist = current_dist - last_dist;

  current_bearing = bearing(current_loc, current_dest);

  double inst_eta = -delta_time * current_dist / delta_dist;
  put_avg(inst_eta);
  eta_time = get_avg();
}

// Used for simulating GPS data
unsigned long update_at = 0;

// Start serial
void setup() {
  Serial.begin(9600);

  lcd.begin();
  lcd.fillScreen(ILI9341_BLACK);
  lcd.setTextColor(ILI9341_YELLOW);
  lcd.setTextSize(2);
  lcd.println("Hello world!");
}

// Fake GPS data and print calculated values
void loop() {
  if (millis() >= update_at) {
    update_at = millis() + 250;
    int ticks = millis() / 250 % 96;
    on_new_gps(
      (double) cos(ticks * (2 * 3.14159 / 96)),
      (double) sin(ticks * (2 * 3.14159 / 96))
    );
    Serial.print(current_loc.lat, 6);
    Serial.print(", ");
    Serial.println(current_loc.lng, 6);
    Serial.println(current_dist);
    Serial.println(current_bearing);
    Serial.println();
  }
}
