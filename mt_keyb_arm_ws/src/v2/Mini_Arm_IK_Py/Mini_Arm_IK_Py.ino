//#include <Servo.h>
#include <ESP32Servo.h>

Servo sg1, sg2;

const float spdLin = 1;
const unsigned int moveDelay = 15;
const int LIMIT1[] = {30, 135};
const int LIMIT2[] = {10, 160};

double ang1, ang2;
double cAng1, cAng2;

void setup() {
  Serial.begin(9600);

  sg1.attach(38);
  sg2.attach(39);
//  sg1.attach(3);
//  sg2.attach(9);
}

void loop() {
  getUserInput();

  ang1 = constrain(ang1, LIMIT1[0], LIMIT1[1]);
  ang2 = constrain(ang2, LIMIT2[0], LIMIT2[1]);

  smoothing();
}

void smoothing(){
  while(cAng1 != ang1 || cAng2 != ang2){
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
    Serial.print(cAng1);
    Serial.print(",");
    Serial.println(cAng2);
    sg1.write(int(cAng1));
    sg2.write(int(cAng2));
    delay(moveDelay);
  }
}

void getUserInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Invalid input. Please enter Angle1 and Angle2 separated by a space.");
      Serial.print(">> ");
      return;
    }

    String angStr1 = input.substring(0, spaceIndex);
    String angStr2 = input.substring(spaceIndex + 1);
    ang1 = angStr1.toFloat();
    ang2 = angStr2.toFloat();

    Serial.print(ang1);
    Serial.print(",");
    Serial.println(ang2);
  }
}
