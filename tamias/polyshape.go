package tamias

// Axis structure used by cpPolyShape.
type PolyShapeAxis struct {
  // normal
  n Vect
  // distance from origin
  d Float
} 

// Convex polygon shape structure.
type PolyShape struct {
  // A PolyShape is a shape
  * Shape   
  // Vertex and axis lists.
  numVerts  int;
  verts   []Vect;
  axes    []PolyShapeAxis;
  
  // Transformed vertex and axis lists.
  tVerts  []Vect;
  tAxes   []PolyShapeAxis;
}


// Returns the minimum distance of the polygon to the axis.
func (poly * PolyShape) ValueOnAxis(n Vect, d Float) (Float) {
  verts := poly.tVerts
  min   := n.Dot(verts[0])  
 
  for  i:=1 ; i< poly.numVerts; i++ { 
    min = min.Min(n.Dot(verts[i])) 
  }  
  
  return min - d;
}

func (poly * PolyShape) ContainsVert(v Vect) (bool) {
  axes := poly.tAxes;
  for i:=0 ; i<poly.numVerts; i++ {
    dist := axes[i].n.Dot(v) - axes[i].d;
    if dist > Float(0.0) { return false; }
  }  
  return true
}

// Same as cpPolyShapeContainsVert() but ignores faces pointing 
// away from the normal.
func (poly * PolyShape) ContainsVertPartial(v Vect) (bool) {
  axes := poly.tAxes;
  for i:=0 ; i<poly.numVerts; i++ {
    if axes[i].n.Dot(v) < Float(0.0) { continue; } 
    dist := axes[i].n.Dot(v) - axes[i].d;
    if dist > Float(0.0) { return false; } 
  }  
  return true
}


func PolyShapeAlloc() (*PolyShape) {
  return &PolyShape{}
}

func (poly * PolyShape) TransformVerts(p Vect, rot Vect) {
  src := poly.verts;
  dst := poly.tVerts;
  for i:= 0; i < poly.numVerts; i++ { 
    dst[i] = p.Add(src[i].Rotate(rot))
  }  
}

func (poly * PolyShape) TransformAxes(p Vect, rot Vect) {
  src := poly.axes
  dst := poly.tAxes
  var n Vect
  for i:= 0; i < poly.numVerts; i++ {
    n         = src[i].n.Rotate(rot)
    dst[i].n  = n
    dst[i].d  = p.Dot(n) + src[i].d
  }
}

func (poly * PolyShape) CacheBB(p, rot Vect) (BB) {    
  var l, b, r, t Float
  
  poly.TransformAxes(p, rot)  
  poly.TransformVerts(p, rot)
  
  verts:= poly.tVerts
  r = verts[0].X
  l = verts[0].X
  t = verts[0].Y
  b = verts[0].Y
    
  // TODO do as part of cpPolyShapeTransformVerts?
  for i:=1; i < poly.numVerts; i++ {
    v := verts[i]    
    l = l.Min(v.X)
    r = r.Max(v.X)    
    b = b.Min(v.Y)
    t = t.Max(v.Y)
  }
  
  return BBNew(l, b, r, t)
}

func (poly * PolyShape) Destroy() {
  poly.axes   = nil
  poly.tAxes  = nil
  poly.verts  = nil
  poly.tVerts = nil
}  

func (poly * PolyShape) PointQuery(p Vect) (bool) {
  return poly.Shape.BB.ContainsVect(p) && poly.ContainsVert(p)
}    

func (poly * PolyShape) SegmentQuery (a, b Vect) (info * SegmentQueryInfo) {
        
  axes      := poly.tAxes
  verts     := poly.tVerts
  numVerts  := poly.numVerts
  info       = &SegmentQueryInfo{}
  
  var axis PolyShapeAxis  
  var n, point Vect
  var an, bn, t, dt, dtMin, dtMax Float  
    
  for i:=0 ; i < numVerts; i++ {
    axis = axes[i]
    n   = axis.n;
    an  = a.Dot(n) 
    if axis.d > an { continue; }
     
    bn  = b.Dot(n)
    t   = axis.d - an / (bn - an)     
    if t < Float(0.0) || Float(1.0) < t { continue; }
    
    point = a.Lerp(b, t) 
    dt    = - n.Cross(point)
    dtMin = - n.Cross(verts[i])
    dtMax = - n.Cross(verts[(i+1) % numVerts])
    if dtMin <= dt && dt <= dtMax {
      info.shape = poly.Shape;
      info.t = t;
      info.n = n;
      // ??? do we not need return info here ???
    }
  }
  return info
}


var PolyClass = &ShapeClass{ POLY_SHAPE };

// Validate checks if the polygon is winding correcty.
func PolyShapeValidate(numVerts int, verts []Vect) (bool) {
  for i:=0; i < numVerts; i++ {
    a := verts[i];
    b := verts[(i+1)%numVerts];
    c := verts[(i+2)%numVerts];
    if b.Sub(a).Cross(c.Sub(b)) > Float(0.0) { 
      return false 
    }   
  }  
  return true;
}

func (poly * PolyShape) NumVerts() (int) {
  return poly.numVerts
}  

func (poly * PolyShape) GetVert(idx int) (Vect) {
  Assert(0 <= idx && idx < poly.NumVerts(), "Index out of range.")
  return poly.verts[idx]
}

func (poly * PolyShape) setUpVerts(numVerts int, verts []Vect, offset Vect) {
  poly.numVerts = numVerts;

  poly.verts    = make([]Vect, numVerts)
  poly.tVerts   = make([]Vect, numVerts)
  poly.axes     = make([]PolyShapeAxis, numVerts)
  poly.tAxes    = make([]PolyShapeAxis, numVerts)
  for i:=0 ; i < numVerts ; i++ {
    a := offset.Add(verts[i])
    b := offset.Add(verts[(i+1)%numVerts])
    n := b.Sub(a).Perp().Normalize() 
    poly.verts[i]   = a;
    poly.axes[i].n  = n;
    poly.axes[i].d  = n.Dot(a)
  }
}

func (poly * PolyShape) Init(body * Body, numVerts int, 
      verts []Vect, offset Vect) (* PolyShape) { 
  // Fail if the user attempts to pass a concave poly, or a bad winding.
  Assert(PolyShapeValidate(numVerts, verts), 
    "Polygon is concave or has a reversed winding.")
  
  poly.setUpVerts(numVerts, verts, offset);
  poly.Shape.Init(PolyClass, body);

  return poly;
}

func PolyShapeNew(body * Body, numVerts int, verts []Vect, offset Vect) (
  poly * PolyShape) {
  return PolyShapeAlloc().Init(body, numVerts, verts, offset) 
}  

func (poly * PolyShape) BoxInit(body * Body, width, height Float) (
      * PolyShape) {
  hw     := width / Float(2.0)
  hh     := width / Float(2.0)
  verts  := [4]Vect { V(-hw, -hh), V(-hw, hh), V(hw, hh), V(hw, -hh) }
  poly.Init(body, 4, verts[0:3], VZERO)
  return poly
}  

func BoxShapeNew(body * Body, width, height Float) (poly * PolyShape) {
  return PolyShapeAlloc().BoxInit(body, width, height) 
}  

// Unsafe API (chipmunk_unsafe.h)
func (poly * PolyShape) SetVerts(numVerts int, verts []Vect, offset Vect) {
  poly.Destroy()
  poly.setUpVerts(numVerts, verts, offset);
}

