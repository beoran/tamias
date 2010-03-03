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
