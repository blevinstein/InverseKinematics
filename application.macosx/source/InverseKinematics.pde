// tolerance allowed when checking for floating point equality
float TOL = .001;
boolean RANDOMIZE = false;

PVector[] inverseKinematics(float len[], PVector disp) {
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
PVector[] inverseKinematics(float len[], PVector disp, float prefs[]) {
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
