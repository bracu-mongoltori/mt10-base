#include <Servo.h>

Servo sg1, sg2;

const int LIMIT1[] = {30, 135};
const int LIMIT2[] = {10, 160};

void setup() {
  Serial.begin(9600);

  sg1.attach(3);
  sg2.attach(9);

}

void loop() {
  sg1.write(LIMIT1[1]);
  sg2.write(LIMIT2[1]);
}
