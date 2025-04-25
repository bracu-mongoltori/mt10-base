#include <ESP32Servo.h>

Servo sg1, sg2;

const float spdArm = 1;

const float spdLin = 2;
const unsigned int moveDelay = 15;

const double len1 = 15.5, len2 = 14;
const int LIMIT1[] = {30, 135};
const int LIMIT2[] = {10, 135};
const double a2Offs = 10, a1Offs = -60;

double ang1, ang2;
double cAng1, cAng2;

double targetPX = 21.4, targetPY = 13.8;

long timer;

void setup() {
  Serial.begin(9600);

  sg1.attach(38);
  sg2.attach(39);
}

void loop() {
  getUserInput();

  setIK(targetPX, targetPY);

  ang1 = constrain(ang1, LIMIT1[0], LIMIT1[1]);
  ang2 = constrain(ang2, LIMIT2[0], LIMIT2[1]);

  smoothing();
  //if(millis()-timer > 200){
    sg1.write(int(cAng1));
    sg2.write(int(cAng2));
    //timer = millis();
  //}
}

//void smoothing(){
//  cAng1 = cAng1 + (ang1 - cAng1)*spdArm;
//  cAng2 = cAng2 + (ang2 - cAng2)*spdArm;
//}

void smoothing(){
  if(abs(cAng1 - ang1) > spdLin){
    cAng1 += (ang1 > cAng1)? spdLin : -spdLin;
  }
  else{
    cAng1 = ang1;
  }

  if(abs(cAng2 - ang2) > spdLin){
    cAng2 += (ang2 > cAng2)? spdLin : -spdLin;
  }
  else{
    cAng2 = ang2;
  }
  delay(moveDelay);
}

void getUserInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Invalid input. Please enter X and Y separated by a space.");
      Serial.print(">> ");
      return;
    }

    String xString = input.substring(0, spaceIndex);
    String yString = input.substring(spaceIndex + 1);
    float x = xString.toFloat();
    float y = yString.toFloat();

    targetPX = x;
    targetPY = y;
    
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
  }
}

void setIK(double x, double y) {
  double sqd = x * x + y * y;
  double d = pow(sqd, 0.5);
  double ang = degrees(atan2(y, -x));
  if (ang < 0) {
    ang += 360;
  }
  if (d > len1 + len2) {
    ang2 = a2Offs;
    ang1 = ang + a1Offs;
    return;
  }

  double angBs = degrees(acos( (len1 * len1 + sqd - len2 * len2) / (2 * len1 * d)));

  double angTs = degrees(acos( (len1 * len1 + len2 * len2 - sqd) / (2 * len1 * len2) ));
  ang1 = ang - angBs + a1Offs;
  ang2 = a2Offs + (180 - angTs);
}
