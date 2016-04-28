#include <math.h>

typedef struct Location {
  char name[16];
  double lat;
  double lng;
} Location;

unsigned long updateAt = 0;

Location currentLoc = {0};
Location lastLoc = {0};
double currentMph = 0;
double currentHeading = 0;

Location locations[] = {
  {"Home", 45.022884, -93.244983},
  {"Work", 44.989136, -93.250454},
};

double distance(Location from, Location to) {
  return sqrt(pow(from.lat - to.lat, 2) + pow(from.lng - to.lng, 2));
}

void onNewGps(double lat, double lng, double mph, double heading) {
  currentLoc.lat = lat;
  currentLoc.lng = lng;
  currentMph = mph;
  Serial.println(currentLoc.lat, 6);
  Serial.println(currentLoc.lng, 6);
  Serial.println(distance(lastLoc, currentLoc));
  lastLoc = currentLoc;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (millis() >= updateAt) {
    updateAt = millis() + 1000;
    int ticks = (millis() / 1000) % 24;
    onNewGps(
      (double) cos(ticks * (2 * 3.14159 / 24)),
      (double) sin(ticks * (2 * 3.14159 / 24)),
      (double) random(0, 8000) / 100,
      (double) random(0, 360)
    );
  }
}
