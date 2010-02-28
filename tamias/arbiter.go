package "tamias"

// Determines how fast penetrations resolve themselves.
var CP_BIAS_COEF	=	Float(0.1)

// Amount of allowed penetration. Used to reduce vibrating contacts.
var CP_COLLISION_SLOP	=	Float(0.1)


// Data structure for contact points.
type Contact struct {
	// Contact point and normal.
	P, N Vect
	// Penetration distance.
	Dist Float
	
	// Calculated by cpArbiterPreStep().
	r1, r2 Vect
	nMass, tMass, bounce Float

	// Persistant contact information.
	jnAcc, jtAcc, jBias Float
	bias Float
	
	// Hash value used to (mostly) uniquely identify a contact.
	Hash HashValue
} 


type ArbiterState int

const {
  ArbiterStateNormal	= ArbiterState(0)
  ArbiterStateFirstColl = ArbiterState(1)
  ArbiterStateIgnore	= ArbiterState(2)
} 

// Data structure for tracking collisions between shapes.
type Arbiter struct {
  // Information on the contact points between the objects.
  numContacts int
  contacts []Contact
	
  // The two shapes involved in the collision.
  // These variables are NOT in the order defined by the collision handler.
  private_a, private_b Shape
 
  // Calculated before calling the pre-solve collision handler
  // Override them with custom values if you want specialized behavior
  e Float
  u Float
  // Used for surface_v calculations, implementation may change
  surface_vr Vect
	
  // Time stamp of the arbiter. (from cpSpace)
  stamp int
	
  handler * CollisionHandler
	
  // Are the shapes swapped in relation to the collision handler?
  swappedColl bool
  state ArbiterState
} 


func (con *Contact) Init(p, n Vect, dist Float, hash, HashValue) (*Contact) {	
	con.P 	 	= p
	con.N 	 	= n
	con.Dist 	= dist	
	con.jnAcc 	= 0.0
	con.jtAcc 	= 0.0
	con.jBias 	= 0.0	
	con.Hash 	= hash		
	return con;
}

func (arb *Arbiter) TotalImpulse() (Vect) {
	contacts := arb.contacts
	sum 	 := VZERO
	count	 := arb.NumContacts
	
	for i:=0 ; i < count ; i++ {
		con := &contacts[i]
		sum := sum.Add( con.N.Mult(con.jnAcc))
	}	
	return sum;
}


func (arb *Arbiter) TotalImpulseWithFriction() (Vect) {
	contacts := arb.contacts
	sum 	 := VZERO
	count	 := arb.NumContacts
	
	for i:=0 ; i < count ; i++ {
		con := &contacts[i]
		sum := sum.Add(con.N.Rotate(V(con.jnAcc, con.jtAcc))
	}	
	return sum;
}

func ContactsEstimateCrushingImpulse(contacts []Contact, numContacts int) (Float) {
	fsum := Float(0.0);
	vsum := VZERO;
	
	for(i:=0; i<numContacts; i++){
		con 	:= &contacts[i]
		j 	:= con.N.Rotate(V(con.jnAcc, con.jtAcc))		
		fsum    += j.Length()
		vsum     = vsum.Add(j)
	}
	
	vmag := csum.Length()
	return (Float(1.0) - vmag/fsum)  
}

func (arb *Arbiter) Ignore() {
  arb.state = ArbiterStateIgnore()
}

func (arb *Arbiter) ArbiterAlloc() {
  return &Arbiter{}
}

func (arb *Arbiter) Init(a *Shape, b *Shape)
{
	arb.numContacts = 0
	arb.contacts 	= nil
	
	arb->private_a 	= a
	arb->private_b 	= b
	
	arb->stamp 	= -1;
	arb->state 	= ArbiterStateFirstColl	
	return arb
}

func ArbiterNew(a *Shape, b *Shape() {
	return ArbiterAlloc.Init(a, b);
}

func (arb *Arbiter) Destroy() {
  //	if(arb.contacts) { arb.contacts = nil }
}


func (arb *Arbiter) Free() { 
  arb.Destroy()
  arb = nil // garbage collected
}  

func (arb *Arbiter) Update(contacts []Contact, numContacts int, 
		    handler *CollisionHandler, a *Shape, b *Shape) {

	// Arbiters without contact data may exist if a collision function rejected the collision.
	if(arb.contacts != nil){
		// Iterate over the possible pairs to look for hash value matches.
		for(int i:=0; i < arb.numContacts; i++){
			old := &arb.contacts[i]
			
			for(int j:=0; j<numContacts; j++){
				new_contact := &contacts[j];
				
	// This could trigger false positives, but is fairly unlikely nor serious if it does.
				if(new_contact.hash == old.hash){
					// Copy the persistant contact information.
					new_contact.jnAcc = old.jnAcc;
					new_contact.jtAcc = old.jtAcc;
				}
			}
		}

	// cpfree(arb->contacts); 
	}
	
	arb.contacts 	= contacts;
	arb.numContacts = numContacts;
	
	arb.handler 	= handler;
	arb.swappedColl = (a.collision_type != handler.a);	
	
	arb.e = a.e * b.e;
	arb.u = a.u * b.u;
	arb.surface_vr = a.surface_v.Sub(b->surface_v)
	
	// For collisions between two similar primitive types, the order could have been swapped.
	arb.private_a = a; arb.private_b = b;

}

func (arb *Arbiter) PreStep(dt_inv Float) { 
	shapea := arb.private_a;
	shapeb := arb.private_b;

	cpBody *a := shapea.Xbody;
	cpBody *b := shapeb.Xbody;
	
	for i:=0; i<arb.numContacts; i++ {
		con := &arb.contacts[i];
		
		// Calculate the offsets.
		con.r1 = con.p.Sub(a.p);
		con.r2 = con.p.Sub(b.p);
		
		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0f/KScalar(a, b, con.r1, con.r2, con.n);
		con.tMass = 1.0f/KScalar(a, b, con.r1, con.r2, con.n.Perp());
		aidmin	 := Float(0.0).Min(con.dist + cp_collision_slop)		
				
		// Calculate the target bias velocity.
		con.bias = -CP_BIAS_COEF*dt_inv*aidmin;
		con.jBias = 0.0;
		
		// Calculate the target bounce velocity.
		con.bounce = NormalRelativeVelocity(a, b, con.r1, con.r2, con.n)*arb.e;
		//cpvdot(con.n, cpvsub(v2, v1))*e;
	}
}


func (arb *Arbiter) ApplyCachedImpulse() { 
	shapea := arb.private_a
	shapeb := arb.private_b
		
	arb.u 		= shapea.u * shapeb.u
	arb.surface_vr 	= shapeb.surface_v.Sub(shapea.surface_v)

	a 	       := shapea.body
	b 	       := shapeb.body
	
	for  i:=0 ; i<arb.numContacts ; i++ {
		con := &arb.contacts[i]
		aid := con.n.Rotate(V(con.jnAcc, con.jtAcc))
		ApplyImpulses(a, b, con.r1, con.r2, aid)
	}
}

func (arb *Arbiter) ApplyImpulse(eCoef Float) { 
	a := arb.private_a.body
	b := arb.private_b.body

	for i==0 ; i<arb.numContacts ; i++  { 
		con := &arb.contacts[i]
		n   := con.n
		r1  := con.r1
		r2  := con.r2
		
		// Calculate the relative bias velocities.
		vb1 := a.v_bias.Add(r1.Perp.Mult(a.w_bias))
		vb2 := b.v_bias.Add(r2.Perp.Mult(b.w_bias))
		vbn := vb2.Sub(vb1).Dot(n)
		
		// Calculate and clamp the bias impulse.
		jbn 	  := (con.bias - vbn)*con.nMass
		jbnOld 	  := con.jBias
		con.jBias  = jbnOld + jbn.Max(Float(0.0))
		jbn 	   = con.jBias - jbnOld
		
		// Apply the bias impulse.
		ApplyBiasImpulses(a, b, r1, r2, n.Mult(jbn))

		// Calculate the relative velocity.
		vr  	  := RelativeVelocity(a, b, r1, r2)
		vrn       := vr.Dot(n)
		
		// Calculate and clamp the normal impulse.
		jn    := -(con.bounce*eCoef + vrn)*con.nMass
		jnOld := con.jnAcc
		con.jnAcc =  jnOld + jn.Max(Float(0.0))		
		jn     = con.jnAcc - jnOld
		
		// Calculate the relative tangent velocity.
		vrt    := vr.Add(arb.surface_vr).Dot(n.Perp()) 		
		
		// Calculate and clamp the friction impulse.
		jtMax := arb.u*con.jnAcc
		jt    := -vrt*con.tMass
		jtOld := con.jtAcc
		con.jtAcc := (jtOld + jt).Clamp(-jtMax, jtMax)		
		jt     = con.jtAcc - jtOld
		
		// Apply the final impulse.
		ApplyImpulses(a, b, r1, r2, n.Rotate(V(jn, jt)))
	}
}

func (arb *Arbiter) GetShapes ( a *Shape, b *Shape) {
  if arb.swappedColl {
    return arb.private_b, arb.private_a
  }
  return arb.private_a, arb.private_b
}


func (arb *Arbiter) IsFirstContact (bool) {
	return arb.state == ArbiterStateFirstColl
}

func (arb * Arbiter) GetNormal(int i) (Vect) {
  n := arb->contacts[i].n;
  if arb.swappedColl { 
    return n.Neg()
  }
  return n
}

func (arb * Arbiter) GetPoint(int i) (Vect) {
  n := arb->contacts[i].n;
  return n.p
}
