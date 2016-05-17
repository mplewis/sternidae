#include "ILI9341_t3.h"
#include "SPI.h"
#include "Average.h"
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
  {"Test", {0.0, 2.0}},
  {"Home", {45.022884, -93.244983}},
  {"Work", {44.989136, -93.250454}},
};

// For calculating distance
// Earth's radius, in mi
const double EARTH_RADIUS = 3959;
// Radians per degree
const double RADS_PER_DEG = 2 * PI / 360;

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
// Averaged ETA in milliseconds
Average<double> avg_eta_time(10);

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
  avg_eta_time.push(inst_eta);
  eta_time = avg_eta_time.mean();
}

void draw_dist_eta() {
  const int LINE_LEN = 14;
  lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  // Arduino doesn't support snprintf with %f
  char out[LINE_LEN];
  double dist = min(current_dist, 999.9);
  dtostrf(dist, 0, 1, out);
  strlcat(out, "mi", LINE_LEN);
  lcd.setCursor(0, 24);
  lcd.print(out);

  lcd.setCursor(8 * 18, 24);
  if (eta_time >= 0) {
    long secs = eta_time / 1000;
    secs = min(99*60*60 + 59*60, secs);  // max at 99:59
    int hrs = secs / 60 / 60;
    int mins = secs / 60 % 60;
    snprintf(out, LINE_LEN, "%2d:%02d", hrs, mins);
    lcd.print(out);
  } else {
    lcd.print(" -:--");
  }
}

/*
Draw the BCEF rectangle as shown below:

       e__d__f
      /  /  /
     /  /  /
    /  /  /
   /  /  /
  b__a__c
    /
   /
  /
 / angle
*________
^
^--- origin

<CBE = <BEF = <EFC = <FCB = 90°
start_len: origin -> a
end_len: origin -> d

angle is in degrees
origin_x, origin_y, start_len, end_len, thickness are in pixels
*/
void draw_line(double origin_x, double origin_y, double angle,
               double start_len, double end_len, double thickness,
               uint16_t color) {
  angle -= 90;  // align needle 90° left to N
  angle = angle * RADS_PER_DEG;  // convert to radians
  double a_x = cos(angle) * start_len + origin_x;
  double a_y = sin(angle) * start_len + origin_y;
  double d_x = cos(angle) * end_len + origin_x;
  double d_y = sin(angle) * end_len + origin_y;
  double ab_x = cos(angle + PI/2) * thickness / 2;
  double ab_y = sin(angle + PI/2) * thickness / 2;
  double b_x = a_x + ab_x;
  double b_y = a_y + ab_y;
  double c_x = a_x - ab_x;
  double c_y = a_y - ab_y;
  double e_x = d_x + ab_x;
  double e_y = d_y + ab_y;
  double f_x = d_x - ab_x;
  double f_y = d_y - ab_y;
  lcd.fillTriangle(b_x, b_y, c_x, c_y, e_x, e_y, color);
  lcd.fillTriangle(c_x, c_y, e_x, e_y, f_x, f_y, color);
}

void draw_lines() {
  lcd.fillCircle(120, 172, 103 - 4, ILI9341_BLACK);
  draw_line(120, 172, current_bearing, 30, 95, 6, ILI9341_RED);
}

void draw() {
  draw_dist_eta();
  draw_lines();
}

// Used for simulating GPS data
unsigned long update_at = 0;

// Start serial
void setup() {
  Serial.begin(9600);
  lcd.begin();
  lcd.setRotation(2);
  lcd.fillScreen(ILI9341_BLACK);
  lcd.setCursor(0, 0);
  lcd.setTextSize(3);
  lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  lcd.println("Lake Harriet");
  lcd.setCursor(0, 296);
  lcd.println("12:30   14.8V");
  lcd.fillCircle(120, 172, 103, ILI9341_WHITE);
}

// Fake GPS data and print calculated values
void loop() {
  if (millis() >= update_at) {
    update_at = millis() + 250;
    int ticks = millis() / 250 % 96;
    on_new_gps(
      (double) cos(ticks * (2 * PI / 96)),
      (double) sin(ticks * (2 * PI / 96))
    );
    draw();
  }
}
