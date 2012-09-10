// tolerance allowed when checking for floating point equality
float TOL = .001;
boolean RANDOMIZE = false;

/*
 * calculates a set of vectors r,
 * such that r[i].mag() == len[i]
 * and the sum of all r[i] == disp
 */
PVector[] inverseKinematics(float len[],PVector disp) {
  assert(len.length > 0);
  // split disp into magnitude m and unit vector
  float m = disp.mag();
  PVector disp_unit = disp.get(); disp_unit.normalize();
  // calculate perpendicular vector to disp_unit
  // TODO: choose perpendicular direction based on preference
  PVector disp_perp = new PVector(disp_unit.y,-disp_unit.x);
  if(RANDOMIZE && floor(random(2))==0) disp_perp.mult(-1);
  // NOTE: with better floating point equality checking, N=1 should be base
  // N=1 trivial case
  if(len.length == 1) {
    assert(abs(m-len[0]) < TOL);
    PVector r[] = new PVector[1];
    r[0] = disp.get();
    return r;
  }
  // N>1 recursive case
  float rLen[] = new float[len.length-1];
  arrayCopy(len, 1, rLen, 0, len.length-1);
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
  // TODO: choose a radius between range min and max based on preference
  float radius = (range.x + range.y)/2;
  if(RANDOMIZE) radius = range.x + random(range.y - range.x);
  // calculate triangle parameters
  float cosr = (sq(len[0]) + sq(m) - sq(radius)) / (2*len[0]*m);
  assert(cosr <= 1 + TOL);
  if(cosr > 1) cosr = 1;
  float d = len[0] * cosr;
  float h = len[0] * sqrt(1 - sq(cosr));
  // calculate vectors
  PVector first[] = new PVector[1];
  first[0] = PVector.add(PVector.mult(disp_unit,d), PVector.mult(disp_perp,h));
  PVector tail[] = inverseKinematics(rLen,PVector.sub(disp,first[0]));
  return (PVector[])concat(first, tail);
}

/*
 * calcuates the range of possible
 * distances which can be covered by
 * vectors of the given lengths
 */
// NOTE: uses PVector to represent a range
PVector radius_range(float len[]) {
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
