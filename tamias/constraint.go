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


func (c *  constraint) MaxForce() (Float) {
  return c.maxForce
}
  
func (c *  constraint) BiasCoef() (Float) {
  return c.biasCoef
}

func (c *  constraint) MaxBias() (Float) {
  return c.maxBias
}

func (c *  constraint) PreStep(dt, dt_inv int) {
  
}

func (c *  constraint) ApplyImpulse() {
  
}
  
func (c *  constraint) GetImpulse() (Float) {
  return Float(0.0)
}

func (c *  constraint) Init(a, b *Body) {  
  c.a        = a
  c.b        = b  
  c.maxForce = INFINITY
  c.biasCoef = CONSTRAINT_BIAS_COEF
  c.maxBias  = INFINITY
}

type SpringConstraint interface {
  Constraint
  SpringTorque() (Float) 
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


type cpRatchetJoint struct {
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


