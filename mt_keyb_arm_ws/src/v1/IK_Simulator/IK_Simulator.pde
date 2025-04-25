import processing.serial.*;
Serial myPort;

float scl = 10;

Segment jnt;
Segment sg1, sg2;
Segment _sg1, _sg2;

float len1 = 15.5, len2 = 14;
//float []LIMIT1 = {30, 135};
//float []LIMIT2 = {10, 160};
float []LIMIT1 = {0, 180};
float []LIMIT2 = {0, 270};

int ik_pole_direction = 1;

float ang1, ang2;
float _ang1, _ang2;
//float a1Offs = -60;
//float a2Offs = 10;
float a1Offs = 0;
float a2Offs = 135;

float targetPx = 0, targetPy = 30;

float []rngY = {0, 17};
float []rngX = {14, 24};

long timer;

void setup() {
  size(800, 500);
  jnt = new Segment(400, 400);

  sg1 = new Segment(jnt, len1, a1Offs);
  sg1.limit = LIMIT1;
  sg2 = new Segment(sg1, len2, a2Offs);
  sg2.limit = LIMIT2;
  sg2.vOff = 180;
  
  _sg1 = new Segment(jnt, len1, a1Offs);
  _sg1.limit = LIMIT1;
  _sg2 = new Segment(_sg1, len2, a2Offs);
  _sg2.limit = LIMIT2;
  _sg2.vOff = 180;
  timer = millis();

  try {
    String portName = Serial.list()[0];
    print(portName);
    myPort = new Serial(this, portName, 9600);
  }
  catch(Exception e) {
    println(e);
  }
}

void setIK(float x, float y) {
  float sqd = x*x + y*y;
  float d = pow(sqd, 0.5);
  float ang = degrees(atan2(y, -x));
  if (ang<0) {
    ang+=360;
  }
  if (d > len1+len2) {
    ang1 = ang + a1Offs;
    ang2 = a2Offs;
    _ang1 = ang1;
    _ang2 = ang2;
    return;
  }

  float angBs = degrees(acos( (len1*len1 + sqd - len2*len2)/ (2*len1*d)));

  float angTs = degrees(acos( (len1*len1 + len2*len2 - sqd)/(2*len1*len2) ));
  ang1 = ang - angBs*ik_pole_direction + a1Offs;
  ang2 = a2Offs + (180-angTs)*ik_pole_direction;
  _ang1 = ang + angBs*ik_pole_direction + a1Offs;
  _ang2 = a2Offs - (180-angTs)*ik_pole_direction;
}

void Loop() {
  setIK(targetPx, targetPy);
  ang1 = constrain(ang1, LIMIT1[0], LIMIT1[1]);
  ang2 = constrain(ang2, LIMIT2[0], LIMIT2[1]);
  _ang1 = constrain(_ang1, LIMIT1[0], LIMIT1[1]);
  _ang2 = constrain(_ang2, LIMIT2[0], LIMIT2[1]);

  sg1.write(ang1);
  sg2.write(ang2);
  
  _sg1.write(_ang1);
  _sg2.write(_ang2);
}

void _readLine() {
  String line = myPort.readStringUntil('\n');
  if (line != null) {
    println(line);
  }
}

void draw() {
  background(255);
  _readLine();
  Loop();

  float []p = headPos(ang1, ang2);
  float []pc = headPos(_ang1, _ang2);

  for (int i = int(-width/scl); i < (width/scl); i++) {
    if (i%5 == 0) {
      stroke(0, 50);
    } else {
      stroke(200, 50);
    }
    line(i*scl+jnt.x, 0, i*scl+jnt.x, height);
  }
  for (int i = int(-height/scl); i < (height/scl); i++) {
    if (i%5 == 0) {
      stroke(0, 50);
    } else {
      stroke(200, 50);
    }
    line(0, jnt.y-i*scl, width, jnt.y-i*scl);
  }

  noFill();
  stroke(255, 100, 0);
  quad(rngX[0]*scl+jnt.x, jnt.y-rngY[0]*scl, rngX[1]*scl+jnt.x, jnt.y-rngY[0]*scl, rngX[1]*scl+jnt.x, jnt.y-rngY[1]*scl, rngX[0]*scl+jnt.x, jnt.y-rngY[1]*scl);

  stroke(0);
  line(jnt.x + targetPx*scl, jnt.y - targetPy*scl-15, jnt.x + targetPx*scl, jnt.y - targetPy*scl+15);
  line(jnt.x + targetPx*scl-15, jnt.y - targetPy*scl, jnt.x + targetPx*scl+15, jnt.y - targetPy*scl);

  float d = dist(p[0], p[1], targetPx, targetPy);
  float dc = dist(pc[0], pc[1], targetPx, targetPy);
  if (d > 0.01) {
    stroke(255, 0, 0);
    line(jnt.x+p[0]*scl, jnt.y-p[1]*scl, jnt.x + targetPx*scl, jnt.y - targetPy*scl);
    fill(255,0,0);
    text(String.format("%.2fcm", d), jnt.x+((p[0]+targetPx)/2)*scl, jnt.y-((p[1]+targetPy)/2)*scl);
  }
  if (dc > 0.01) {
    stroke(150,0,0, 150);
    line(jnt.x+pc[0]*scl, jnt.y-pc[1]*scl, jnt.x + targetPx*scl, jnt.y - targetPy*scl);
    fill(150,0,0, 150);
    text(String.format("%.2fcm", dc), jnt.x+((pc[0]+targetPx)/2)*scl, jnt.y-((pc[1]+targetPy)/2)*scl);
  }
  jnt.show(true);
  _sg1.show(false);
  _sg2.show(false);
  sg1.show(true);
  sg2.show(true);


  fill(255, 0, 0);
  textSize(20);
  text(String.format("x: %.2f cm \ny: %.2f cm", targetPx, targetPy), 10, 20);
  if (millis()-timer > 1000) {
    send();
    timer = millis();
  }
}

float []headPos(float ang1, float ang2) {
  float a1 = ang1 - a1Offs;
  float a2 = a1 + ang2 - a2Offs;

  float rx = -(cos(radians(a1))*len1 + cos(radians(a2))*len2);
  float ry = sin(radians(a1))*len1 + sin(radians(a2))*len2;

  float []pos = {rx, ry};
  return pos;
}

void send() {
  try {
    myPort.write(String.format("%.2f %.2f\n", targetPx, targetPy));
  }
  catch(Exception e) {
    println(e);
  }
}

void mouseDragged() {
  setTarget();
}

void mousePressed() {
  setTarget();
}

void mouseReleased() {
  setTarget();
  timer = millis();
}

void setTarget() {
  targetPx = (mouseX - jnt.x)/scl;
  targetPy = (jnt.y - mouseY)/scl;

  //targetPx = constrain(targetPx, rngX[0], rngX[1]);
  //targetPy = constrain(targetPy, rngY[0], rngY[1]);
}
