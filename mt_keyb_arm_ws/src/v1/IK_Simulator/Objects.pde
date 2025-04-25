class Segment {
  Segment prev;
  float angle, len, target, offset;
  float x, y;
  boolean jnt = false;

  float vOff;
  float []limit = {0, 180};

  Segment(float x, float y) {
    this.x = x;
    this.y = y;
    this.angle = radians(a1Offs+90);
    jnt = true;
  }
  Segment(Segment prev, float len, float offs) {
    this.prev = prev;
    this.angle = 0;
    this.len = len*scl;
    this.target = 0;
    this.offset = offs;

    this.vOff = 0;
  }

  void show(boolean main) {
    if (jnt) {
      noFill();
      stroke(0);
      circle(this.x, this.y, 15);
      return;
    }
    //this.angle = lerp(this.angle, radians(this.target-180+this.offset), spd);
    this.angle = radians(this.target);
    float []p1 = this.prev.getEnd();
    float []p2 = this.getEnd();
    noFill();
    stroke(0);
    if (!main) {
      stroke(128);
    }
    line(p1[0], p1[1], p2[0], p2[1]);
    circle(p2[0], p2[1], 15);

    stroke(0, 150);
    if (main) {
      arc(p1[0], p1[1], 100, 100, this.prev.getAng()+radians(180-this.offset + this.limit[0]), this.prev.getAng()+radians(180-this.offset + this.limit[1]));

      fill(255, 0, 0);
      textSize(15);
      text(String.format("%.0f°", this.target), p1[0] + 10, p1[1] - 10);
    }
  }

  float []getEnd() {
    if (jnt) {
      float[] p = {this.x, this.y};
      return p;
    }
    //float a = this.getAng() + radians(this.vOff);
    float a = this.getAng();
    float []p = this.prev.getEnd();
    float []np = {p[0] - this.len * cos(a),
      p[1] - this.len * sin(a)
    };
    return np;
  }

  float getAng() {
    if (jnt) {
      return 0;
    }

    float ap = this.prev.getAng();
    return ap + this.angle - radians(this.offset);
  }

  void write(float t) {
    this.target = t;
  }
}
