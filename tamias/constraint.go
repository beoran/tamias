package tamias

var CONSTRAINT_BIAS_COEF = Float(0.1); 

type Constraint interface {
  A() (*Body)
  B() (*Body)
  Data() (interface{})
  
  MaxForce() (Float)
  BiasCoef() (Float)
  MaxBias() (Float)

  PreStep(dt, dt_inv int)
  ApplyImpulse()
  GetImpulse() (Float)
}



type constraint struct {
  a, b *Body;
  maxForce, biasCoef, maxBias Float;  
  data interface{}
}

type BasicConstraint constraint



func (c *  constraint) Bodies() (*Body, *Body) {
  return c.a, c.b
}


func (c *  constraint) MaxForce() (Float) {
  return c.maxForce
}
  
func (c *  constraint) BiasCoef() (Float) {
  return c.biasCoef
}

func (c *  constraint) MaxBias() (Float) {
  return c.maxBias
}

func (c * constraint) Init(a, b *Body) (* constraint) {  
  c.a        = a
  c.b        = b  
  c.maxForce = INFINITY
  c.biasCoef = CONSTRAINT_BIAS_COEF
  c.maxBias  = INFINITY
  return c
}

type SpringConstraint interface {
  Constraint
  SpringTorque(Vect) (Float) 
}

type DampedRotarySpring struct {
  constraint
  restAngle , stiffness , damping   Float
  dt        , target_wrn, iSum      Float
}

type DampedSpring struct {
  constraint
  anchr1, anchr2 Vect
  restLength, stiffness, damping Float;  
  dt, target_vrn Float  
  r1, r2 Vect
  nMass Float;
  n Vect;
}

type GearJoint struct {
  constraint
  phase, ratio, ratio_inv, iSum Float
  bias, jAcc, jMax Float
} 

type GrooveJoint struct {
  constraint
  grv_n, grv_a, grv_b Vect
  anchr2 Vect  
  grv_tn Vect
  clamp Float
  r1, r2 Vect
  k1, k2 Vect
  
  jAcc Vect
  jMaxLen Vect
  bias Vect
}


type PinJoint struct {
  constraint
  anchr1, anchr2 Vect
  dist Float   
  r1, r2 Vect
  n Vect
  nMass Float  
  jnAcc, jnMax Float
  bias Float
}

type PivotJoint struct {
  constraint
  anchr1, anchr2 Vect  
  r1, r2 Vect
  k1, k2 Vect
  
  jAcc Vect
  jMaxLen Float
  bias Vect
}


type RatchetJoint struct {
  constraint
  angle, phase, ratchet Float  
  iSum Float
  bias Float
  jAcc, jMax Float 
} 

type RotaryLimitJoint struct {
  constraint
  min, max Float;
  iSum Float;
  bias Float;
  jAcc, jMax Float;
} 

type SimpleMotor struct {
  constraint
  rate, iSum, jAcc, jMax Float;
}

type SlideJoint struct {
  constraint
  anchr1, anchr2 Vect;
  min, max Float;
  
  r1, r2 Vect;
  n Vect;
  nMass Float;
  
  jnAcc, jnMax Float;
  bias Float;
} 


func (spring * DampedRotarySpring) SpringTorque(relativeAngle Float) (Float) {
  return (relativeAngle - spring.restAngle)*spring.stiffness;
}

func (spring * DampedRotarySpring) PreStep(dt, dt_inv Float) {
  a, b             := spring.Bodies()
  spring.iSum       = Float(1.0)/(a.i_inv + b.i_inv);
  spring.dt         = dt;
  spring.target_wrn = Float(0.0);  
  // apply spring torque
  da                := a.a - b.a
  j_spring          := spring.SpringTorque(da) * dt
  a.w               -= j_spring * a.i_inv;
  b.w               += j_spring * b.i_inv;
}


func (spring * DampedRotarySpring) ApplyImpulse() { 

  a, b := spring.Bodies()    
  // compute relative velocity
  wrn  := a.w - b.w;
  
  //normal_relative_velocity(a, b, r1, r2, n) - spring.target_vrn;
  
  // compute velocity loss from drag
  // not 100% certain this is derived correctly, though it makes sense
  w_aid  := (-spring.damping*spring.dt/spring.iSum).Exp()
  w_damp := wrn*(Float(1.0) - w_aid);
  spring.target_wrn = wrn - w_damp;
  
  //apply_impulses(a, b, spring.r1, spring.r2, cpvmult(spring.n, v_damp*spring.nMass));
  j_damp := w_damp*spring.iSum;
  a.w   -= j_damp*a.i_inv;
  b.w   += j_damp*b.i_inv;
}

func (spring * DampedRotarySpring) GetImpulse() (Float) {
  return Float(0.0)
}


func DampedRotarySpringAlloc() (* DampedRotarySpring) {
  return &DampedRotarySpring{}
}

func (spring * DampedRotarySpring) Init(a, b *Body, restAngle, stiffness,
      damping Float) (* DampedRotarySpring) {
       
  spring.constraint.Init(a, b)  
  spring.restAngle  = restAngle
  spring.stiffness  = stiffness
  spring.damping    = damping  
  return spring
}

func DampedRotarySpringNew(a, b *Body, restAngle, stiffness,
      damping Float) (* DampedRotarySpring) {
  return DampedRotarySpringAlloc().Init(a, b, restAngle, stiffness, damping)      
}

/*
static cpFloat
defaultSpringForce(cpDampedSpring *spring, cpFloat dist){
  return (spring->restLength - dist)*spring->stiffness;
}

static void
preStep(cpDampedSpring *spring, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = spring->constraint.a;
  cpBody *b = spring->constraint.b;
  
  spring->r1 = cpvrotate(spring->anchr1, a->rot);
  spring->r2 = cpvrotate(spring->anchr2, b->rot);
  
  cpVect delta = cpvsub(cpvadd(b->p, spring->r2), cpvadd(a->p, spring->r1));
  cpFloat dist = cpvlength(delta);
  spring->n = cpvmult(delta, 1.0f/(dist ? dist : INFINITY));
  
  // calculate mass normal
  spring->nMass = 1.0f/k_scalar(a, b, spring->r1, spring->r2, spring->n);

  spring->dt = dt;
  spring->target_vrn = 0.0f;

  // apply spring force
  cpFloat f_spring = spring->springForceFunc((cpConstraint *)spring, dist);
  apply_impulses(a, b, spring->r1, spring->r2, cpvmult(spring->n, f_spring*dt));
}

static void
applyImpulse(cpDampedSpring *spring)
{
  cpBody *a = spring->constraint.a;
  cpBody *b = spring->constraint.b;
  
  cpVect n = spring->n;
  cpVect r1 = spring->r1;
  cpVect r2 = spring->r2;

  // compute relative velocity
  cpFloat vrn = normal_relative_velocity(a, b, r1, r2, n) - spring->target_vrn;
  
  // compute velocity loss from drag
  // not 100% certain this is derived correctly, though it makes sense
  cpFloat v_damp = -vrn*(1.0f - cpfexp(-spring->damping*spring->dt/spring->nMass));
  spring->target_vrn = vrn + v_damp;
  
  apply_impulses(a, b, spring->r1, spring->r2, cpvmult(spring->n, v_damp*spring->nMass));
}

static cpFloat
getImpulse(cpConstraint *constraint)
{
  return 0.0f;
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpDampedSpring)

cpDampedSpring *
cpDampedSpringAlloc(void)
{
  return (cpDampedSpring *)cpmalloc(sizeof(cpDampedSpring));
}

cpDampedSpring *
cpDampedSpringInit(cpDampedSpring *spring, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
{
  cpConstraintInit((cpConstraint *)spring, cpDampedSpringGetClass(), a, b);
  
  spring->anchr1 = anchr1;
  spring->anchr2 = anchr2;
  
  spring->restLength = restLength;
  spring->stiffness = stiffness;
  spring->damping = damping;
  spring->springForceFunc = (cpDampedSpringForceFunc)defaultSpringForce;
  
  return spring;
}

cpConstraint *
cpDampedSpringNew(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
{
  return (cpConstraint *)cpDampedSpringInit(cpDampedSpringAlloc(), a, b, anchr1, anchr2, restLength, stiffness, damping);
}

static void
preStep(cpGearJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // calculate moment of inertia coefficient.
  joint->iSum = 1.0f/(a->i_inv*joint->ratio_inv + joint->ratio*b->i_inv);
  
  // calculate bias velocity
  cpFloat maxBias = joint->constraint.maxBias;
  joint->bias = cpfclamp(-joint->constraint.biasCoef*dt_inv*(b->a*joint->ratio - a->a - joint->phase), -maxBias, maxBias);
  
  // compute max impulse
  joint->jMax = J_MAX(joint, dt);

  // apply joint torque
  cpFloat j = joint->jAcc;
  a->w -= j*a->i_inv*joint->ratio_inv;
  b->w += j*b->i_inv;
}

static void
applyImpulse(cpGearJoint *joint)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // compute relative rotational velocity
  cpFloat wr = b->w*joint->ratio - a->w;
  
  // compute normal impulse 
  cpFloat j = (joint->bias - wr)*joint->iSum;
  cpFloat jOld = joint->jAcc;
  joint->jAcc = cpfclamp(jOld + j, -joint->jMax, joint->jMax);
  j = joint->jAcc - jOld;
  
  // apply impulse
  a->w -= j*a->i_inv*joint->ratio_inv;
  b->w += j*b->i_inv;
}

static cpFloat
getImpulse(cpGearJoint *joint)
{
  return cpfabs(joint->jAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpGearJoint)

cpGearJoint *
cpGearJointAlloc(void)
{
  return (cpGearJoint *)cpmalloc(sizeof(cpGearJoint));
}

cpGearJoint *
cpGearJointInit(cpGearJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->phase = phase;
  joint->ratio = ratio;
  joint->ratio_inv = 1.0f/ratio;
  
  joint->jAcc = 0.0f;
  
  return joint;
}

cpConstraint *
cpGearJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio)
{
  return (cpConstraint *)cpGearJointInit(cpGearJointAlloc(), a, b, phase, ratio);
}

void
cpGearJointSetRatio(cpConstraint *constraint, cpFloat value)
{
  cpConstraintCheckCast(constraint, cpGearJoint);
  ((cpGearJoint *)constraint)->ratio = value;
  ((cpGearJoint *)constraint)->ratio_inv = 1.0f/value;
}


static void
preStep(cpGrooveJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // calculate endpoints in worldspace
  cpVect ta = cpBodyLocal2World(a, joint->grv_a);
  cpVect tb = cpBodyLocal2World(a, joint->grv_b);

  // calculate axis
  cpVect n = cpvrotate(joint->grv_n, a->rot);
  cpFloat d = cpvdot(ta, n);
  
  joint->grv_tn = n;
  joint->r2 = cpvrotate(joint->anchr2, b->rot);
  
  // calculate tangential distance along the axis of r2
  cpFloat td = cpvcross(cpvadd(b->p, joint->r2), n);
  // calculate clamping factor and r2
  if(td <= cpvcross(ta, n)){
    joint->clamp = 1.0f;
    joint->r1 = cpvsub(ta, a->p);
  } else if(td >= cpvcross(tb, n)){
    joint->clamp = -1.0f;
    joint->r1 = cpvsub(tb, a->p);
  } else {
    joint->clamp = 0.0f;
    joint->r1 = cpvsub(cpvadd(cpvmult(cpvperp(n), -td), cpvmult(n, d)), a->p);
  }
  
  // Calculate mass tensor
  k_tensor(a, b, joint->r1, joint->r2, &joint->k1, &joint->k2); 
  
  // compute max impulse
  joint->jMaxLen = J_MAX(joint, dt);
  
  // calculate bias velocity
  cpVect delta = cpvsub(cpvadd(b->p, joint->r2), cpvadd(a->p, joint->r1));
  joint->bias = cpvclamp(cpvmult(delta, -joint->constraint.biasCoef*dt_inv), joint->constraint.maxBias);
  
  // apply accumulated impulse
  apply_impulses(a, b, joint->r1, joint->r2, joint->jAcc);
}

static inline cpVect
grooveConstrain(cpGrooveJoint *joint, cpVect j){
  cpVect n = joint->grv_tn;
  cpVect jClamp = (joint->clamp*cpvcross(j, n) > 0.0f) ? j : cpvproject(j, n);
  return cpvclamp(jClamp, joint->jMaxLen);
}

static void
applyImpulse(cpGrooveJoint *joint)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  cpVect r1 = joint->r1;
  cpVect r2 = joint->r2;
  
  // compute impulse
  cpVect vr = relative_velocity(a, b, r1, r2);

  cpVect j = mult_k(cpvsub(joint->bias, vr), joint->k1, joint->k2);
  cpVect jOld = joint->jAcc;
  joint->jAcc = grooveConstrain(joint, cpvadd(jOld, j));
  j = cpvsub(joint->jAcc, jOld);
  
  // apply impulse
  apply_impulses(a, b, joint->r1, joint->r2, j);
}

static cpFloat
getImpulse(cpGrooveJoint *joint)
{
  return cpvlength(joint->jAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpGrooveJoint)

cpGrooveJoint *
cpGrooveJointAlloc(void)
{
  return (cpGrooveJoint *)cpmalloc(sizeof(cpGrooveJoint));
}

cpGrooveJoint *
cpGrooveJointInit(cpGrooveJoint *joint, cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchr2)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->grv_a = groove_a;
  joint->grv_b = groove_b;
  joint->grv_n = cpvperp(cpvnormalize(cpvsub(groove_b, groove_a)));
  joint->anchr2 = anchr2;
  
  joint->jAcc = cpvzero;
  
  return joint;
}

cpConstraint *
cpGrooveJointNew(cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchr2)
{
  return (cpConstraint *)cpGrooveJointInit(cpGrooveJointAlloc(), a, b, groove_a, groove_b, anchr2);
}

static void
preStep(cpPinJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  joint->r1 = cpvrotate(joint->anchr1, a->rot);
  joint->r2 = cpvrotate(joint->anchr2, b->rot);
  
  cpVect delta = cpvsub(cpvadd(b->p, joint->r2), cpvadd(a->p, joint->r1));
  cpFloat dist = cpvlength(delta);
  joint->n = cpvmult(delta, 1.0f/(dist ? dist : (cpFloat)INFINITY));
  
  // calculate mass normal
  joint->nMass = 1.0f/k_scalar(a, b, joint->r1, joint->r2, joint->n);
  
  // calculate bias velocity
  cpFloat maxBias = joint->constraint.maxBias;
  joint->bias = cpfclamp(-joint->constraint.biasCoef*dt_inv*(dist - joint->dist), -maxBias, maxBias);
  
  // compute max impulse
  joint->jnMax = J_MAX(joint, dt);
  
  // apply accumulated impulse
  cpVect j = cpvmult(joint->n, joint->jnAcc);
  apply_impulses(a, b, joint->r1, joint->r2, j);
}

static void
applyImpulse(cpPinJoint *joint)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  cpVect n = joint->n;

  // compute relative velocity
  cpFloat vrn = normal_relative_velocity(a, b, joint->r1, joint->r2, n);
  
  // compute normal impulse
  cpFloat jn = (joint->bias - vrn)*joint->nMass;
  cpFloat jnOld = joint->jnAcc;
  joint->jnAcc = cpfclamp(jnOld + jn, -joint->jnMax, joint->jnMax);
  jn = joint->jnAcc - jnOld;
  
  // apply impulse
  apply_impulses(a, b, joint->r1, joint->r2, cpvmult(n, jn));
}

static cpFloat
getImpulse(cpPinJoint *joint)
{
  return cpfabs(joint->jnAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpPinJoint);


cpPinJoint *
cpPinJointAlloc(void)
{
  return (cpPinJoint *)cpmalloc(sizeof(cpPinJoint));
}

cpPinJoint *
cpPinJointInit(cpPinJoint *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->anchr1 = anchr1;
  joint->anchr2 = anchr2;
  
  cpVect p1 = cpvadd(a->p, cpvrotate(anchr1, a->rot));
  cpVect p2 = cpvadd(b->p, cpvrotate(anchr2, b->rot));
  joint->dist = cpvlength(cpvsub(p2, p1));

  joint->jnAcc = 0.0f;
  
  return joint;
}

cpConstraint *
cpPinJointNew(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
{
  return (cpConstraint *)cpPinJointInit(cpPinJointAlloc(), a, b, anchr1, anchr2);
}

static void
preStep(cpPivotJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  joint->r1 = cpvrotate(joint->anchr1, a->rot);
  joint->r2 = cpvrotate(joint->anchr2, b->rot);
  
  // Calculate mass tensor
  k_tensor(a, b, joint->r1, joint->r2, &joint->k1, &joint->k2);
  
  // compute max impulse
  joint->jMaxLen = J_MAX(joint, dt);
  
  // calculate bias velocity
  cpVect delta = cpvsub(cpvadd(b->p, joint->r2), cpvadd(a->p, joint->r1));
  joint->bias = cpvclamp(cpvmult(delta, -joint->constraint.biasCoef*dt_inv), joint->constraint.maxBias);
  
  // apply accumulated impulse
  apply_impulses(a, b, joint->r1, joint->r2, joint->jAcc);
}

static void
applyImpulse(cpPivotJoint *joint)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  cpVect r1 = joint->r1;
  cpVect r2 = joint->r2;
    
  // compute relative velocity
  cpVect vr = relative_velocity(a, b, r1, r2);
  
  // compute normal impulse
  cpVect j = mult_k(cpvsub(joint->bias, vr), joint->k1, joint->k2);
  cpVect jOld = joint->jAcc;
  joint->jAcc = cpvclamp(cpvadd(joint->jAcc, j), joint->jMaxLen);
  j = cpvsub(joint->jAcc, jOld);
  
  // apply impulse
  apply_impulses(a, b, joint->r1, joint->r2, j);
}

static cpFloat
getImpulse(cpConstraint *joint)
{
  return cpvlength(((cpPivotJoint *)joint)->jAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpPivotJoint)

cpPivotJoint *
cpPivotJointAlloc(void)
{
  return (cpPivotJoint *)cpmalloc(sizeof(cpPivotJoint));
}

cpPivotJoint *
cpPivotJointInit(cpPivotJoint *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->anchr1 = anchr1;
  joint->anchr2 = anchr2;
  
  joint->jAcc = cpvzero;
  
  return joint;
}

cpConstraint *
cpPivotJointNew2(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
{
  return (cpConstraint *)cpPivotJointInit(cpPivotJointAlloc(), a, b, anchr1, anchr2);
}

cpConstraint *
cpPivotJointNew(cpBody *a, cpBody *b, cpVect pivot)
{
  return cpPivotJointNew2(a, b, cpBodyWorld2Local(a, pivot), cpBodyWorld2Local(b, pivot));
}

static void
preStep(cpRatchetJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  cpFloat angle = joint->angle;
  cpFloat phase = joint->phase;
  cpFloat ratchet = joint->ratchet;
  
  cpFloat delta = b->a - a->a;
  cpFloat diff = angle - delta;
  cpFloat pdist = 0.0f;
  
  if(diff*ratchet > 0.0f){
    pdist = diff;
  } else {
    joint->angle = cpffloor((delta - phase)/ratchet)*ratchet + phase;
  }
  
  // calculate moment of inertia coefficient.
  joint->iSum = 1.0f/(a->i_inv + b->i_inv);
  
  // calculate bias velocity
  cpFloat maxBias = joint->constraint.maxBias;
  joint->bias = cpfclamp(-joint->constraint.biasCoef*dt_inv*pdist, -maxBias, maxBias);
  
  // compute max impulse
  joint->jMax = J_MAX(joint, dt);

  // If the bias is 0, the joint is not at a limit. Reset the impulse.
  if(!joint->bias)
    joint->jAcc = 0.0f;

  // apply joint torque
  a->w -= joint->jAcc*a->i_inv;
  b->w += joint->jAcc*b->i_inv;
}

static void
applyImpulse(cpRatchetJoint *joint)
{
  if(!joint->bias) return; // early exit

  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // compute relative rotational velocity
  cpFloat wr = b->w - a->w;
  cpFloat ratchet = joint->ratchet;
  
  // compute normal impulse 
  cpFloat j = -(joint->bias + wr)*joint->iSum;
  cpFloat jOld = joint->jAcc;
  joint->jAcc = cpfclamp((jOld + j)*ratchet, 0.0f, joint->jMax*cpfabs(ratchet))/ratchet;
  j = joint->jAcc - jOld;
  
  // apply impulse
  a->w -= j*a->i_inv;
  b->w += j*b->i_inv;
}

static cpFloat
getImpulse(cpRatchetJoint *joint)
{
  return cpfabs(joint->jAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpRatchetJoint)

cpRatchetJoint *
cpRatchetJointAlloc(void)
{
  return (cpRatchetJoint *)cpmalloc(sizeof(cpRatchetJoint));
}

cpRatchetJoint *
cpRatchetJointInit(cpRatchetJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->angle = 0.0f;
  joint->phase = phase;
  joint->ratchet = ratchet;
  
  joint->angle = b->a - a->a;
  
  return joint;
}

cpConstraint *
cpRatchetJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet)
{
  return (cpConstraint *)cpRatchetJointInit(cpRatchetJointAlloc(), a, b, phase, ratchet);
}

static void
preStep(cpRotaryLimitJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  cpFloat dist = b->a - a->a;
  cpFloat pdist = 0.0f;
  if(dist > joint->max) {
    pdist = joint->max - dist;
  } else if(dist < joint->min) {
    pdist = joint->min - dist;
  }
  
  // calculate moment of inertia coefficient.
  joint->iSum = 1.0f/(a->i_inv + b->i_inv);
  
  // calculate bias velocity
  cpFloat maxBias = joint->constraint.maxBias;
  joint->bias = cpfclamp(-joint->constraint.biasCoef*dt_inv*(pdist), -maxBias, maxBias);
  
  // compute max impulse
  joint->jMax = J_MAX(joint, dt);

  // If the bias is 0, the joint is not at a limit. Reset the impulse.
  if(!joint->bias)
    joint->jAcc = 0.0f;

  // apply joint torque
  a->w -= joint->jAcc*a->i_inv;
  b->w += joint->jAcc*b->i_inv;
}

static void
applyImpulse(cpRotaryLimitJoint *joint)
{
  if(!joint->bias) return; // early exit

  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // compute relative rotational velocity
  cpFloat wr = b->w - a->w;
  
  // compute normal impulse 
  cpFloat j = -(joint->bias + wr)*joint->iSum;
  cpFloat jOld = joint->jAcc;
  if(joint->bias < 0.0f){
    joint->jAcc = cpfclamp(jOld + j, 0.0f, joint->jMax);
  } else {
    joint->jAcc = cpfclamp(jOld + j, -joint->jMax, 0.0f);
  }
  j = joint->jAcc - jOld;
  
  // apply impulse
  a->w -= j*a->i_inv;
  b->w += j*b->i_inv;
}

static cpFloat
getImpulse(cpRotaryLimitJoint *joint)
{
  return cpfabs(joint->jAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpRotaryLimitJoint)

cpRotaryLimitJoint *
cpRotaryLimitJointAlloc(void)
{
  return (cpRotaryLimitJoint *)cpmalloc(sizeof(cpRotaryLimitJoint));
}

cpRotaryLimitJoint *
cpRotaryLimitJointInit(cpRotaryLimitJoint *joint, cpBody *a, cpBody *b, cpFloat min, cpFloat max)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->min = min;
  joint->max  = max;
  
  joint->jAcc = 0.0f;
  
  return joint;
}

cpConstraint *
cpRotaryLimitJointNew(cpBody *a, cpBody *b, cpFloat min, cpFloat max)
{
  return (cpConstraint *)cpRotaryLimitJointInit(cpRotaryLimitJointAlloc(), a, b, min, max);
}

static void
preStep(cpSimpleMotor *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // calculate moment of inertia coefficient.
  joint->iSum = 1.0f/(a->i_inv + b->i_inv);
  
  // compute max impulse
  joint->jMax = J_MAX(joint, dt);

  // apply joint torque
  a->w -= joint->jAcc*a->i_inv;
  b->w += joint->jAcc*b->i_inv;
}

static void
applyImpulse(cpSimpleMotor *joint)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  // compute relative rotational velocity
  cpFloat wr = b->w - a->w + joint->rate;
  
  // compute normal impulse 
  cpFloat j = -wr*joint->iSum;
  cpFloat jOld = joint->jAcc;
  joint->jAcc = cpfclamp(jOld + j, -joint->jMax, joint->jMax);
  j = joint->jAcc - jOld;
  
  // apply impulse
  a->w -= j*a->i_inv;
  b->w += j*b->i_inv;
}

static cpFloat
getImpulse(cpSimpleMotor *joint)
{
  return cpfabs(joint->jAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpSimpleMotor)

cpSimpleMotor *
cpSimpleMotorAlloc(void)
{
  return (cpSimpleMotor *)cpmalloc(sizeof(cpSimpleMotor));
}

cpSimpleMotor *
cpSimpleMotorInit(cpSimpleMotor *joint, cpBody *a, cpBody *b, cpFloat rate)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->rate = rate;
  
  joint->jAcc = 0.0f;
  
  return joint;
}

cpConstraint *
cpSimpleMotorNew(cpBody *a, cpBody *b, cpFloat rate)
{
  return (cpConstraint *)cpSimpleMotorInit(cpSimpleMotorAlloc(), a, b, rate);
}

static void
preStep(cpSlideJoint *joint, cpFloat dt, cpFloat dt_inv)
{
  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  joint->r1 = cpvrotate(joint->anchr1, a->rot);
  joint->r2 = cpvrotate(joint->anchr2, b->rot);
  
  cpVect delta = cpvsub(cpvadd(b->p, joint->r2), cpvadd(a->p, joint->r1));
  cpFloat dist = cpvlength(delta);
  cpFloat pdist = 0.0f;
  if(dist > joint->max) {
    pdist = dist - joint->max;
  } else if(dist < joint->min) {
    pdist = joint->min - dist;
    dist = -dist;
  }
  joint->n = cpvmult(delta, 1.0f/(dist ? dist : (cpFloat)INFINITY));
  
  // calculate mass normal
  joint->nMass = 1.0f/k_scalar(a, b, joint->r1, joint->r2, joint->n);
  
  // calculate bias velocity
  cpFloat maxBias = joint->constraint.maxBias;
  joint->bias = cpfclamp(-joint->constraint.biasCoef*dt_inv*(pdist), -maxBias, maxBias);
  
  // compute max impulse
  joint->jnMax = J_MAX(joint, dt);

  // apply accumulated impulse
  if(!joint->bias) //{
    // if bias is 0, then the joint is not at a limit.
    joint->jnAcc = 0.0f;
//  } else {
    cpVect j = cpvmult(joint->n, joint->jnAcc);
    apply_impulses(a, b, joint->r1, joint->r2, j);
//  }
}

static void
applyImpulse(cpSlideJoint *joint)
{
  if(!joint->bias) return;  // early exit

  cpBody *a = joint->constraint.a;
  cpBody *b = joint->constraint.b;
  
  cpVect n = joint->n;
  cpVect r1 = joint->r1;
  cpVect r2 = joint->r2;
    
  // compute relative velocity
  cpVect vr = relative_velocity(a, b, r1, r2);
  cpFloat vrn = cpvdot(vr, n);
  
  // compute normal impulse
  cpFloat jn = (joint->bias - vrn)*joint->nMass;
  cpFloat jnOld = joint->jnAcc;
  joint->jnAcc = cpfclamp(jnOld + jn, -joint->jnMax, 0.0f);
  jn = joint->jnAcc - jnOld;
  
  // apply impulse
  apply_impulses(a, b, joint->r1, joint->r2, cpvmult(n, jn));
}

static cpFloat
getImpulse(cpConstraint *joint)
{
  return cpfabs(((cpSlideJoint *)joint)->jnAcc);
}

static const cpConstraintClass klass = {
  (cpConstraintPreStepFunction)preStep,
  (cpConstraintApplyImpulseFunction)applyImpulse,
  (cpConstraintGetImpulseFunction)getImpulse,
};
CP_DefineClassGetter(cpSlideJoint)

cpSlideJoint *
cpSlideJointAlloc(void)
{
  return (cpSlideJoint *)cpmalloc(sizeof(cpSlideJoint));
}

cpSlideJoint *
cpSlideJointInit(cpSlideJoint *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat min, cpFloat max)
{
  cpConstraintInit((cpConstraint *)joint, &klass, a, b);
  
  joint->anchr1 = anchr1;
  joint->anchr2 = anchr2;
  joint->min = min;
  joint->max = max;
  
  joint->jnAcc = 0.0f;
  
  return joint;
}

cpConstraint *
cpSlideJointNew(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat min, cpFloat max)
{
  return (cpConstraint *)cpSlideJointInit(cpSlideJointAlloc(), a, b, anchr1, anchr2, min, max);
}



*/
