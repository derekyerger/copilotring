#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_VL6180X.h"

#define PIN        6  // NeoPixel ring connected to pin 6 
#define NUMPIXELS 16 // Number of LEDs in the ring

Adafruit_BNO055 bno = Adafruit_BNO055(55); Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);
Adafruit_VL6180X vl = Adafruit_VL6180X();

float ball_angle = 0.0;  // Ball position in degrees 
float ball_velocity = 0.0;  // Ball velocity 
float damping = 0.95;  // To prevent infinite motion 
float accel_weight = 0.2;  // Scaling factor for acceleration influence
long hue = 0;
int brightness = 50; 
  
void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  } 
  if (!vl.begin()) {
    Serial.println("VL6180X not detected");
    while (1);
  } 
pixels.begin(); pixels.clear(); pixels.setBrightness(20);  pixels.show(); }


uint8_t lastvl;
unsigned long lastvlread;

void loop() { imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

// Compute acceleration influence (Y as forward/backward, X as lateral)
float accel_x = accel.x() * accel_weight;
float accel_y = accel.z() * accel_weight;

// Gravity naturally pulls the ball downward (towards 180 degrees)
float gravity_effect = 0.001; // Adjust as needed
if (ball_angle < 180) ball_velocity += gravity_effect;
else ball_velocity -= gravity_effect;

// Update velocity based on acceleration
ball_velocity += accel_x * cos(ball_angle * DEG_TO_RAD) + accel_y * sin(ball_angle * DEG_TO_RAD);
ball_velocity *= damping;

// Update ball position
ball_angle += ball_velocity;
if (ball_angle < 0) ball_angle += 360;
if (ball_angle >= 360) ball_angle -= 360;

// Map angle to NeoPixel ring (16 LEDs, 360 degrees)
int led_index = round((ball_angle / 360.0) * NUMPIXELS) % NUMPIXELS;

  if ((millis() - lastvlread) > 1000) {
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();
    lastvlread = millis();
  
    if (status == VL6180X_ERROR_NONE) {
      Serial.print("Range: "); Serial.println(range);
      int nv = 100-range;
      if (nv < 0) nv = 0;
    if (range != lastvl) pixels.setBrightness(nv);
    lastvl = range;
    }
  }
// Display the ball position
if (abs(ball_velocity) < 3) pixels.clear(); 
else {
    hue += 256;
  hue %= 5*65536;
}
pixels.setPixelColor(led_index, pixels.ColorHSV(hue));
pixels.show();

delay(5); // Small delay to smooth animation

}
