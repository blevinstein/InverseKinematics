int keys[] = new int[256];
int KEY_NUM = 256, KEY_UP = 0, KEY_DOWN = 1;
int KEY_W = 87, KEY_A = 65, KEY_S = 83, KEY_D = 68;
int KEY_R = 82;

void keyPressed() {
  keys[keyCode] = KEY_DOWN;
  //println(keyCode);
}

void keyReleased() {
  keys[keyCode] = KEY_UP;
}

float len[];
int N = 4;
PVector from;
PVector to;
float pref;

void settings() {
  size(800,600);
}

void start() {
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

void draw() {
  background(255);
  
  if(keys[KEY_W] == KEY_DOWN) pref += .005 + random(TOL);
  if(keys[KEY_S] == KEY_DOWN) pref -= .005 + random(TOL);
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

void mouseEvent(int x, int y, int button) {
  PVector p = new PVector(x, y);
  if(button == LEFT) {
    from = p;
  } else if(button == RIGHT) {
    to = p;
  }
}

void mousePressed() {
  mouseEvent(mouseX, mouseY, mouseButton);
}

void mouseDragged() {
  mouseEvent(mouseX, mouseY, mouseButton);
}

void drawCircle(PVector a,float r) {
  ellipseMode(CENTER);
  ellipse(a.x, a.y, r, r);
}

void drawLine(PVector a,PVector b) {
  line(a.x, a.y, b.x, b.y);
}