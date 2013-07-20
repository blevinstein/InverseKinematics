import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class InverseKinematics extends PApplet {

// tolerance allowed when checking for floating point equality
float TOL = .001f;
boolean RANDOMIZE = false;

public PVector[] inverseKinematics(float len[], PVector disp) {
  float prefs[] = new float[len.length - 1];
  for(int i=0; i<prefs.length; i++) prefs[i] = 0;
  return inverseKinematics(len, disp, prefs);
}

/*
 * calculates a set of vectors r,
 * such that r[i].mag() == len[i],
 * the sum of all r[i] == disp, and
 * preferences are respected
 */
public PVector[] inverseKinematics(float len[], PVector disp, float prefs[]) {
  assert(len.length > 0);
  // split disp into magnitude m and unit vector
  float m = disp.mag();
  PVector disp_unit = disp.get(); disp_unit.normalize();
  // N=1 base case
  if(len.length == 1) {
    assert(abs(m-len[0]) < TOL);
    PVector r[] = new PVector[1];
    r[0] = disp.get();
    return r;
  }
  // calculate perpendicular vector to disp_unit
  PVector disp_perp = prefs[0] < 0 ?
                      new PVector(-disp_unit.y,disp_unit.x) :
                      new PVector(disp_unit.y,-disp_unit.x);
  if(RANDOMIZE && floor(random(2))==0) disp_perp.mult(-1);
  // N>1 recursive case
  // create recursive arrays
  float rLen[] = new float[len.length-1];
  arrayCopy(len, 1, rLen, 0, len.length-1);
  float rPrefs[] = new float[prefs.length-1];
  arrayCopy(prefs, 1, rPrefs, 0, prefs.length-1);
  // check that a solution is possible
  PVector total_range = radius_range(len);
  assert(total_range.x <= m+TOL && m-TOL <= total_range.y);
  // get possible range for recursive ase
  PVector range = radius_range(rLen);
  // limit range based on len[0]
  float tMax = len[0] + m;
  float tMin = abs(len[0] - m);
  if(range.x < tMin) range.x = tMin;
  if(range.y > tMax) range.y = tMax;
  // choose distance
  float radius = lerp(range.y, range.x, abs(constrain(prefs[0], -1, 1)));
  if(RANDOMIZE) radius = range.x + random(range.y - range.x);
  // calculate triangle parameters
  float cosr = (sq(len[0]) + sq(m) - sq(radius)) / (2*len[0]*m);
  assert(abs(cosr) <= 1 + TOL);
  if(cosr > 1) cosr = 1;
  if(cosr < -1) cosr = -1;
  float d = len[0] * cosr;
  float h = len[0] * sqrt(1 - sq(cosr));
  assert(!Float.isNaN(h));
  // calculate vectors
  PVector first[] = new PVector[1];
  first[0] = PVector.add(PVector.mult(disp_unit,d), PVector.mult(disp_perp,h));
  PVector tail[] = inverseKinematics(rLen, PVector.sub(disp,first[0]), rPrefs);
  return (PVector[])concat(first, tail);
}

/*
 * calcuates the range of possible
 * distances which can be covered by
 * vectors of the given lengths
 */
// NOTE: uses PVector to represent a range
public PVector radius_range(float len[]) {
  PVector r = new PVector(len[0],len[0]);
  for(int i=1; i<len.length; i++) {
    if(len[i] > r.y) {
      r = new PVector(len[i] - r.y,len[i] + r.y);
    } else {
      r = new PVector(max(r.x - len[i],0), r.y + len[i]);
    }
  }
  return r;
}
int keys[] = new int[256];
int KEY_NUM = 256, KEY_UP = 0, KEY_DOWN = 1;
int KEY_W = 87, KEY_A = 65, KEY_S = 83, KEY_D = 68;
int KEY_R = 82;

public void keyPressed() {
  keys[keyCode] = KEY_DOWN;
  //println(keyCode);
}

public void keyReleased() {
  keys[keyCode] = KEY_UP;
}

float len[];
int N = 4;
PVector from;
PVector to;
float pref;

public void start() {
  size(800,600);
  background(255);
  len = new float[N];
  pref = 0;
  // equal lengths
  //for(int i=0; i<N; i++) len[i] = dist(0,0,width,height)*1f/N;
  // exponential decreasing lengths
  //for(int i=0; i<N; i++) len[i] = dist(0,0,width,height)*pow(2,-(i+1)); len[N-1] = len[N-2];
  // exponential increasing lengths
  for(int i=0; i<N; i++) len[i] = dist(0,0,width,height)*pow(2,i-N); len[0] = len[1];
  from = new PVector(width/4f,height/4f);
  to = new PVector(width*3/4f,height*3/4f);
}

public void draw() {
  
  if(keys[KEY_W] == KEY_DOWN) pref += .005f + random(TOL);
  if(keys[KEY_S] == KEY_DOWN) pref -= .005f + random(TOL);
  if(pref < -1) pref = -1;
  if(pref > 1) pref = 1;

  noStroke();  
  String s = str(pref);
  fill(255);
  rect(width-100,height-textAscent(),width,height);
  fill(0);
  text(str(pref), width-textWidth(str(pref)), height);

  noFill();
  stroke(0);
  if(RANDOMIZE) stroke(random(256),random(256),random(256));
  float prefs[] = new float[len.length-1];
  for(int i=0; i<prefs.length; i++) prefs[i] = pref;
  PVector[] lines = inverseKinematics(len, PVector.sub(to,from), prefs);
  PVector pen = from.get();
  for(int i=0; i<lines.length; i++) {
    PVector next = PVector.add(pen,lines[i]);
    drawCircle(pen, 5);
    drawLine(pen, next);
    pen = next;
  }
  drawCircle(pen, 5);
}

public void mousePressed() {
  PVector p = new PVector(mouseX, mouseY);
  if(mouseButton == LEFT) {
    from = p;
  } else if(mouseButton == RIGHT) {
    to = p;
  }
  background(255);
}

public void drawCircle(PVector a,float r) {
  ellipseMode(CENTER);
  ellipse(a.x, a.y, r, r);
}

public void drawLine(PVector a,PVector b) {
  line(a.x, a.y, b.x, b.y);
}

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "InverseKinematics" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
