package tamias


/*
func J_MAX(constraint Constraint, dt Float) (Vect) {
  return Constraint.maxForce * dt
} 
*/

func RelativeVelocity(a, b *Body, r1, r2 Vect) (Vect){
	v1_sum := a.v.Add(r1.Perp().Mult(a.w))
	v2_sum := b.v.Add(r2.Perp().Mult(b.w))
	return v2_sum.Sub(v1_sum)
}

func NormalRelativeVelocity(a, b *Body, r1, r2, n  Vect) (Float) {
	return RelativeVelocity(a, b, r1, r2).Dot(n);
}

func ApplyImpulses (a, b *Body, r1, r2, j Vect) {
	a.ApplyImpulse(j.Neg()	, r1)
	b.ApplyImpulse(j	, r2)
}

func ApplyBiasImpulses(a, b *Body, r1, r2, j Vect) {
	a.ApplyBiasImpulse(j.Neg()	, r1)
	b.ApplyBiasImpulse(j		, r2)
}

func KScalar(a, b *Body, r1, r2, n Vect) (Float) {
	mass_sum := a.m_inv + b.m_inv;
	r1cn 	 := r1.Cross(n) 
	r2cn 	 := r2.Cross(n) 
	value    := mass_sum + a.i_inv*r1cn*r1cn + b.i_inv*r2cn*r2cn
	Assert(value != 0.0, "Unsolvable collision or constraint.")	
	return value
}

func KTensor(a, b *Body, r1, r2 Vect) (k1, k2 Vect) { 
	// calculate mass matrix
	// If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...	
	m_sum := a.m_inv + b.m_inv
	
	// start with I*m_sum
	k11 := m_sum	      ; k12 := Float(0.0)
	k21 := Float(0.0)    ;	k22 := m_sum
	
	// add the influence from r1
	a_i_inv := a.i_inv;
	r1xsq 	:=  r1.X * r1.X * a_i_inv
	r1ysq 	:=  r1.Y * r1.Y * a_i_inv
	r1nxy 	:= -r1.X * r1.Y * a_i_inv
	k11 += r1ysq; k12 += r1nxy
	k21 += r1nxy; k22 += r1xsq
	
	// add the influence from r2
	b_i_inv := b.i_inv;
	r2xsq 	:= r2.X * r2.X * b_i_inv
	r2ysq 	:= r2.Y * r2.Y * b_i_inv
	r2nxy 	:= -r2.X * r2.Y * b_i_inv
	k11 += r2ysq; k12 += r2nxy
	k21 += r2nxy; k22 += r2xsq
	
	// invert
	determinant := k11*k22 - k12*k21
	Assert(determinant != 0.0, "Unsolvable constraint.")
	
	det_inv := 1.0 / determinant;
	k1 	= V( k22*det_inv, -k12*det_inv)
	k2 	= V(-k21*det_inv,  k11*det_inv)
	return k1, k2
}

func (vr Vect) MultK(k1, k2 Vect) (Vect) {
  return V(vr.Dot(k1), vr.Dot(k2));
}


/*
#define CP_DefineClassGetter(t) const cpConstraintClass * t##GetClass(){return (cpConstraintClass *)&klass;}

#define J_MAX(constraint, dt) (((cpConstraint *)constraint)->maxForce*(dt))
*/