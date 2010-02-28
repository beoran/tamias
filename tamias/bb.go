package tamias

// Bounding box type
type BB struct {
  L, B, R, T Float
}

func BBNew(l, b, r, t Float) (BB) { 
  return BB{L: l, B: b, R: r, T: t}
}

func (a BB) Intersects(b BB) bool {
  return (a.L<=b.R && b.L<=a.R && a.B<=b.T && b.B<=a.T)
}

func (bb BB) Contains(other BB) bool {
  return (bb.L < other.L && bb.R > other.R && bb.B < other.B && bb.T > other.T)
}

func (bb BB) ContainsVect(v Vect) bool {
  return (bb.L < v.X && bb.R > v.X && bb.B < v.Y && bb.T > v.Y)
}

func (am BB) Merge(bm BB) (BB) {
  l := am.L.Min(bm.L)
  b := am.B.Min(bm.B)
  r := am.R.Max(bm.R)
  t := am.T.Max(bm.T)
  return BBNew(l, b, r, t)
}

func (bb BB) Expand(v Vect) (BB) {
  l := bb.L.Min(v.X)
  b := bb.B.Min(v.Y)
  r := bb.R.Max(v.X)
  t := bb.T.Max(v.Y)
  return BBNew(l, b, r, t)
}

// clamps the vector to lie within the bbox
func (bb BB) ClampVect(v Vect) (Vect) {
  x :=  bb.L.Max(v.X).Min(bb.R) 
  y :=  bb.B.Max(v.Y).Min(bb.T) 
  return V(x, y)
}

// wrap a vector to a bbox
func (bb BB) WrapVect(v Vect) (Vect) {
	ix 	:= (bb.T - bb.L).Abs();
	modx 	:= (v.X  - bb.L).Mod(ix);
	x 	:= modx
	if (modx <= 0.0) { 
	  x 	 = modx + ix
	}  
	
	iy 	:= (bb.T - bb.B).Abs();
	mody 	:= (v.Y - bb.B).Mod(iy);
	y 	:= mody
	if (mody <= 0.0) { 
	  y 	 = mody + iy
	}  
	return V(x + bb.L, y + bb.B);
}

