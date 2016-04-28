#include <math.h>

typedef struct Location {
  double lat;
  double lng;
} Location;

typedef struct Destination {
  char name[16];
  Location loc;
} Destination;

// Updated by onNewGps
// Location
Location currentLoc = {0};
// Time in milliseconds
unsigned long currentTime = 0;
unsigned long lastTime = 0;
double deltaTime = 0;
// Distance in degrees
double currentDist = 0;
double lastDist = 0;
double deltaDist = 0;
// ETA in milliseconds. If negative, we're traveling away from currentDest
double etaTime = 0;

Destination dests[] = {
  {"Test", {0.0, 10.0}},
  {"Home", {45.022884, -93.244983}},
  {"Work", {44.989136, -93.250454}},
};

// Change currentDest to change the
Location currentDest = dests[0].loc;

double distance(Location from, Location to) {
  return sqrt(pow(from.lat - to.lat, 2) + pow(from.lng - to.lng, 2));
}

void onNewGps(double lat, double lng) {
  currentLoc.lat = lat;
  currentLoc.lng = lng;

  lastTime = currentTime;
  currentTime = millis();
  deltaTime = currentTime - lastTime;

  lastDist = currentDist;
  currentDist = distance(currentLoc, currentDest);
  deltaDist = currentDist - lastDist;

  etaTime = -deltaTime * currentDist / deltaDist;
}

void setup() {
  Serial.begin(9600);
}

unsigned long updateAt = 0;

void loop() {
  if (millis() >= updateAt) {
    updateAt = millis() + 1000;
    int ticks = millis() / 1000 % 24;
    onNewGps(
      (double) cos(ticks * (2 * 3.14159 / 24)),
      (double) sin(ticks * (2 * 3.14159 / 24))
    );
    Serial.print(currentLoc.lat, 6);
    Serial.print(", ");
    Serial.println(currentLoc.lng, 6);
    Serial.println(currentDist);
    Serial.println(deltaDist);
    Serial.println(etaTime / 1000);
    Serial.println();
  }
}
