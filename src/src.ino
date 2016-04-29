#include <math.h>

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

// Updated by on_new_gps
// Location
Location current_loc = {0};
// Time in milliseconds
unsigned long current_time = 0;
unsigned long last_time = 0;
double delta_time = 0;
// Distance in degrees
double current_dist = 0;
double last_dist = 0;
double delta_dist = 0;
// ETA in milliseconds. If negative, we're traveling away from current_dest
double eta_time = 0;

// For averaging ETA
const int avg_count = 10;
double avg_buf[avg_count] = {0};
int avg_buf_pos = 0;

// Change current_dest to change the distance and eta
Location current_dest = dests[0].loc;

// Assumes flat world - error: ~1km @ 1000km
double distance(Location from, Location to) {
  return sqrt(pow(from.lat - to.lat, 2) + pow(from.lng - to.lng, 2));
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

  eta_time = -delta_time * current_dist / delta_dist;
  put_avg(eta_time / 1000);
}

// Used for simulating GPS data
unsigned long update_at = 0;

// Start serial
void setup() {
  Serial.begin(9600);
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
    Serial.println(delta_dist);
    Serial.println(get_avg());
    Serial.println();
  }
}
