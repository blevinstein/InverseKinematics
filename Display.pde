float len[];
int N = 8;
PVector from;
PVector to;

void start() {
  size(800,600);
  len = new float[N];
  // equal lengths
  //for(int i=0; i<N; i++) len[i] = dist(0,0,width,height)*1f/N;
  // exponential lengths
  for(int i=0; i<N; i++) len[i] = dist(0,0,width,height)*pow(2,-(i+1)); len[N-1] = len[N-2];
  from = new PVector(width/4f,height/4f);
  to = new PVector(width*3/4f,height*3/4f);
}

void draw() {
  background(255);
  PVector[] lines = inverseKinematics(len, PVector.sub(to,from));
  PVector pen = from.get();
  for(int i=0; i<lines.length; i++) {
    PVector next = PVector.add(pen,lines[i]);
    drawLine(pen, next);
    pen = next;
  }
}

void mousePressed() {
  PVector p = new PVector(mouseX, mouseY);
  if(mouseButton == LEFT) {
    from = p;
  } else if(mouseButton == RIGHT) {
    to = p;
  }
}

void drawLine(PVector a,PVector b) {
  line(a.x, a.y, b.x, b.y);
}

