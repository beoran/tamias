package tamias

type BodyVelocityFunc func (body Body, gravity Vect, damping, dt Float)
type BodyPositionFunc func (body Body, dt Float)
type DataPointer * interface{}

/*
var (
  BodyUpdateVelocityDefault BodyVelocityFunc = Body.UpdateVelocity
  BodyUpdatePositionDefault BodyPositionFunc = Body.UpdatePosition
) 
*/
 
type Body struct  {
	// *** Integration Functions.ntoehu

	// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
	velocityFunc BodyVelocityFunc;
	
	// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
	positionFunc BodyPositionFunc;
	
	// *** Mass Properties
	
	// Mass and it's inverse.
	// Always use cpBodySetMass() whenever changing the mass as these values must agree.
	m, m_inv Float;
	
	// Moment of inertia and it's inverse.
	// Always use cpBodySetMoment() whenever changing the moment as these values must agree.
	i, i_inv Float;
	
	// *** Positional Properties
	
	// Linear components of motion (position, velocity, and force)
	p, v, f Vect;
	
	// Angular components of motion (angle, angular velocity, and torque)
	// Always use cpBodySetAngle() to set the angle of the body as a and rot must agree.
	a, w, t Float;
	
	// Cached unit length vector representing the angle of the body.
	// Used for fast vector rotation using cpvrotate().
	rot Vect;
	
	// *** User Definable Fields
	
	// User defined data pointer.
	data DataPointer;
	
	// Maximum velocities this body can move at after integrating velocity
	v_limit, w_limit Float;
	
	// *** Internally Used Fields
	
	// Velocity bias values used when solving penetrations and correcting constraints.
	v_bias Vect;
	w_bias Float;
	
	//	int active;
}	

func (body * Body) Mass() (Float)  {
  return body.m
}

func (body * Body) SetMass(m Float) (Float)  {
  body.m 	= m
  body.m_inv	= Float(1.0) / m
  return body.m
}

func (body * Body) Moment() (Float)  {
  return body.i
}

func (body * Body) SetMoment(i Float) (Float)  {
  body.i 	= i	
  body.i_inv 	= Float(1.0) / i
  return body.i
}

func (body * Body) Pos() (Vect)  {
  return body.p
}

func (body * Body) Vel() (Vect)  {
  return body.v
}

func (body * Body) Force() (Vect)  {
  return body.f
}

func (body * Body) Angle() (Float)  {
  return body.a
}

func (body * Body) SetAngle(a Float) (Float)  {
  body.a 	= a
  body.rot	= a.VectForAngle()
  return body.a
}

func (body * Body) AngVel() (Float)  {
  return body.w
}

func (body * Body) Torque() (Float)  {
  return body.t
}

func (body * Body) Rot() (Vect)  {
  return body.rot
}

func (body * Body) VelLimit() (Float)  {
  return body.v_limit
}

func (body * Body) AngVelLimit() (Float)  {
  return body.w_limit
}

func (body * Body) SetVelLimit(v Float) (Float)  {
  body.v_limit = v
  return body.v_limit
}

func (body * Body) SetAngVelLimit(v Float) (Float)  {
  body.w_limit = v
  return body.w_limit
}

// Convert body local to world coordinates
func (body *Body) Local2World(v Vect) (Vect) {
  return body.p.Add(v.Rotate(body.rot))
}

// Convert world to body local coordinates
func (body *Body) World2Local(v Vect) (Vect) {
  return v.Sub(body.p).Unrotate(body.rot)
}

// Apply an impulse (in world coordinates) to the body at a point relative to 
// the center of gravity (also in world coordinates).
func (body *Body) ApplyImpulse(j, r Vect) {
	body.v  = body.v.Add(j.Mult(body.m_inv))
	body.w += body.i_inv * r.Cross(j)
}

// Not intended for external use. Used by cpArbiter.c and cpConstraint.c.

func (body *Body) ApplyBiasImpulse(j, r Vect) {
	body.v_bias  = body.v_bias.Add(j.Mult(body.m_inv))
	body.w_bias += body.i_inv * r.Cross(j)
}

func BodyAlloc() (* Body) {
  return &Body{}
}


func (body *Body) Init(m Float, i Float) (*Body) {
	/* Compiler doesnt' seem to support these yet.
	body.velocityFunc = Body.UpdateVelocity
	body.positionFunc = Body.UpdatePosition
	*/
	
	body.SetMass(m)
	body.SetMoment(i)

	body.p = VZERO
	body.v = VZERO
	body.f = VZERO
	
	body.SetAngle(Float(0.0))
	body.w = Float(0.0)
	body.t = Float(0.0)
	
	body.v_bias = VZERO
	body.w_bias = Float(0.0)
	
	body.data    = nil
	body.v_limit = INFINITY
	body.w_limit = INFINITY
//	body.active = 1;
	return body
}

func BodyNew(m, i Float) (*Body) {
	return BodyAlloc().Init(m, i)
}

func (body * Body) Destroy() {
}

func (body * Body) Free() {
  if body != nil {	
    body.Destroy()
    body = nil
  }
}


func (body * Body) Slew(pos Vect, dt Float)  {
  delta := pos.Sub(body.p)
  body.v = delta.Mult(Float(1.0) / dt)
}

func (body * Body) UpdateVelocity(gravity Vect, damping, dt Float)  {
  vdamp := body.v.Mult(damping)  
  vforc := gravity.Add(body.f.Mult(body.m_inv)).Mult(dt)
  body.v = vdamp.Add(vforc).Clamp(body.v_limit)
  w_lim := body.w_limit
  waid  := body.w*damping + body.t*body.i_inv*dt
  body.w = waid.Clamp(-w_lim, w_lim)
}

func (body * Body) UpdatePosition(dt Float)  {
	body.p = body.p.Add(body.v.Add(body.v_bias).Mult(dt))
	body.SetAngle(body.a + (body.w + body.w_bias)*dt)
	body.v_bias = VZERO;
	body.w_bias = Float(0.0)
}

func (body * Body) ResetForces() {
  body.f = VZERO
  body.t = Float(0.0)
} 

func (body * Body) ApplyForce(force, r Vect) {
	body.f  = body.f.Add(force)
	body.t += r.Cross(force)
}

func ApplyDampedSpring(a, b *Body, anchr1, anchr2 Vect, rlen, k, dmp, dt Float) {
	// Calculate the world space anchor coordinates.
	r1 	:= anchr1.Rotate(a.rot)
	r2 	:= anchr2.Rotate(b.rot)
	
	delta 	:= b.p.Add(r2).Sub(a.p.Add(r1))
	dist  	:= delta.Length()	
	n     	:= VZERO
	if (dist > 0.0) { 
	  n    = delta.Mult(Float(1.0)/dist) 
	}  
	
	f_spring := (dist - rlen)*k;

	// Calculate the world relative velocities of the anchor points.
	// This really should be in the impulse solver and can produce problems when using large damping values.
	v1 	:= a.v.Add(r1.Perp().Mult(a.w))
	v2 	:= b.v.Add(r2.Perp().Mult(b.w))
	vrn	:= v2.Sub(v1).Dot(n)
	f_damp  := vrn * dmp.Min(Float(1.0) / (dt * (a.m_inv + b.m_inv)))
	
	// Apply!
	f 	:= n.Mult(f_spring + f_damp)
	a.ApplyForce(f		, r1);
	b.ApplyForce(f.Neg()	, r2);
}


//int
//cpBodyMarkLowEnergy(cpBody *body, cpFloat dvsq, int max)
//{
//	cpFloat ke = body->m*cpvdot(body->v, body->v);
//	cpFloat re = body->i*body->w*body->w;
//	
//	if(ke + re > body->m*dvsq)
//		body->active = 1;
//	else if(body->active)
//		body->active = (body->active + 1)%(max + 1);
//	else {
//		body->v = cpvzero;
//		body->v_bias = cpvzero;
//		body->w = 0.0f;
//		body->w_bias = 0.0f;
//	}
//	
//	return body->active;
//}
