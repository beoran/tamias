package tamias

type SegmentQueryInfo struct {
  // shape that was hit, nil if no collision
  shape * Shape
  // Distance along query segment, will always be in the range [0, 1].
  t Float
  // normal of hit surface
  n Vect 
} 

// Collision type, etc
type CollisionType int
type GroupType int
type LayerType int

// Enumeration of shape types.
type ShapeType int

const (
  CIRCLE_SHAPE	= ShapeType(0)
  SEGMENT_SHAPE	= ShapeType(1)
  POLY_SHAPE		= ShapeType(2)
  NUM_SHAPES		= ShapeType(3)
  NO_GROUP		  = GroupType(0)  
)  

const (
  LAYER_0		  = 1 << iota
  LAYER_1		  = 1 << iota
  LAYER_2		  = 1 << iota
  LAYER_3		  = 1 << iota
  LAYER_4		  = 1 << iota
  LAYER_5		  = 1 << iota
  LAYER_6		  = 1 << iota
  LAYER_7		  = 1 << iota
  LAYER_8		  = 1 << iota
  LAYER_9		  = 1 << iota
  LAYER_10		= 1 << iota
  LAYER_11		= 1 << iota
  LAYER_12		= 1 << iota
  LAYER_13		= 1 << iota
  LAYER_14    = 1 << iota
  LAYER_15    = 1 << iota
  LAYER_16    = 1 << iota 
  ALL_LAYERS	= -1
)


// Shape class is probably not needed, but used for now for storng 
// typeinfo
type ShapeClass struct {	
  Type ShapeType
}

// Basic shape struct that the others inherit from.
type Shape struct {
  // The "class" of a shape as defined above 
  * ShapeClass
  // cpBody that the shape is attached to.
  * Body
  // Cached BBox for the shape.
  * BB
  // Sensors invoke callbacks, but do not generate collisions
  sensor bool
  // *** Surface properties.
  // Coefficient of restitution. (elasticity)
  e Float;
  // Coefficient of friction.
  u Float;
  // Surface velocity used when solving for friction.
  surface_v Vect;
  // *** User Definable Fields
  // User defined data pointer for the shape.
  data DataPointer
  // User defined collision type for the shape.
  collision_type CollisionType
  // User defined collision group for the shape.
  group GroupType
  // User defined layer bitmask for the shape.
  layers LayerType
  // *** Internally Used Fields
  // Unique id used as the hash value.
  hashid HashValue
}

// Circle shape structure.
type CircleShape struct {
  * Shape
  // Center in body space coordinates
  c Vect
  // Radius.
  r Float
  // Transformed center. (world space coordinates)
  tc Vect;
} 

// Segment shape structure.
type SegmentShape struct {
  * Shape
  // Endpoints and normal of the segment. (body space coordinates)
  a, b, n Vect
  // Radius of the segment. (Thickness)
  r Float
  // Transformed endpoints and normal. (world space coordinates)
  ta, tb, tn Vect
} 

// Returns true if a shape was hit, false if not
func (info *SegmentQueryInfo) Hit() (bool) {
  return info.shape != nil 
}


func (info *SegmentQueryInfo) HitPoint(start, end Vect) (Vect) {
	return start.Lerp(end, info.t) 
}

func (info *SegmentQueryInfo) HitDist(start Vect, end Vect) (Float) {
	return start.Dist(end) * info.t
}

var ( 
  SHAPE_ID_COUNTER HashValue = HashValue(0)
)  

func ResetShapeIdCounter() {
	SHAPE_ID_COUNTER = HashValue(0)
}

func (shape * Shape) Init(klass *ShapeClass, body *Body) (*Shape){
	shape.ShapeClass = klass	
	shape.hashid 	   = SHAPE_ID_COUNTER
	SHAPE_ID_COUNTER++	
	
	shape.Body 	     = body
	shape.sensor 	   = false
	
	shape.e 	       = Float(0.0)
	shape.u 	       = Float(0.0)
	shape.surface_v  = VZERO
	
	shape.collision_type 	= 0
	shape.group 		= NO_GROUP
	shape.layers 		= ALL_LAYERS
	shape.data 		  = nil
	return shape;
}

func ShapeNew(klass *ShapeClass, body *Body) (*Shape) {
  return new(Shape).Init(klass, body)
}

func (shape * Shape) CacheBB(p Vect, rot Vect) (BB) {
  return BBMake(p.X, p.Y, p.X, p.Y)
}

func (shape * Shape) GetBB() (*BB) {
  return shape.BB
}

func (shape * Shape) Destroy() {
}

func (shape * Shape) Free() {
}

/* These are done differently in Go language
cpBB
cpShapeCacheBB(cpShape *shape)
{
	cpBody *body = shape->body;
	
	shape->bb = shape->klass->cacheData(shape, body->p, body->rot);
	return shape->bb;
}

int
cpShapePointQuery(cpShape *shape, cpVect p){
	return shape->klass->pointQuery(shape, p);
}

int
cpShapeSegmentQuery(cpShape *shape, cpVect a, cpVect b, cpSegmentQueryInfo *info){
	cpSegmentQueryInfo blank = {NULL, 0.0f, cpvzero};
	(*info) = blank;
	
	shape->klass->segmentQuery(shape, a, b, info);
	return (info->shape != NULL);
}



void
cpSegmentQueryInfoPrint(cpSegmentQueryInfo *info)
{
	printf("Segment Query:\n");
	printf("\tt: %f\n", info->t);
//	printf("\tdist: %f\n", info->dist);
//	printf("\tpoint: %s\n", cpvstr(info->point));
	printf("\tn: %s\n", cpvstr(info->n));
}
*/




func CircleShapeAlloc() (* CircleShape) {
	return &CircleShape{};
}

 
func bbFromCircle(c Vect, r Float) (BB) {
	return BBMake(c.X-r, c.Y-r, c.X+r, c.Y+r);
}

func (circle * CircleShape) CacheBB(p Vect, rot Vect) (BB) {
  circle.tc	=	p.Add(circle.c.Rotate(rot))
  return bbFromCircle(circle.tc, circle.r)
}

func (circle * CircleShape) PointQuery(p Vect) (bool) {	
	return circle.tc.Near(p, circle.r)
}



func CircleSegmentQuery(shape *Shape, center Vect, r Float, a, b Vect) (info * SegmentQueryInfo) {
	// umm... gross I normally frown upon such things (sic)
	aa := a.Sub(center);
	bb := b.Sub(center);
	
	qa := aa.Dot(aa) - Float(2.0)*aa.Dot(bb) + bb.Dot(bb)
	qb := Float(-2.0)*aa.Dot(aa) + Float(2.0)*aa.Dot(bb)
	qc := aa.Dot(aa) - r*r;
	
	det:= qb*qb - Float(4.0)*qa*qc
	info= &SegmentQueryInfo{}	
	if det >= Float(0.0) {
		t := (-qb - det.Sqrt())/(Float(2.0)*qa);
		if Float(0.0) <= t && t <= Float(1.0) { 
			info.shape = shape
			info.t = t
			info.n = a.Lerp(b, t).Normalize()
		}
	}
  return info 
}


func (circle * CircleShape) SegmentQuery(a, b Vect) (info * SegmentQueryInfo) {
  return CircleSegmentQuery(circle.Shape, circle.c, circle.r, a, b)
}

var CircleShapeClass *ShapeClass = &ShapeClass{ CIRCLE_SHAPE }


func (circle * CircleShape) Init(body * Body, radius Float, offset Vect) (* CircleShape) {
	circle.c = offset;
	circle.r = radius;	
	circle.Shape.Init(CircleShapeClass, body);	
	return circle;
}


func CircleShapeNew(body * Body, radius Float, offset Vect) (*CircleShape) {
  return CircleShapeAlloc().Init(body, radius, offset)
}

func (circle * CircleShape) Radius() (Float) {
  return circle.r
}

func (circle * CircleShape) Offset() (Vect) {
  return circle.c
}

func SegmentShapeAlloc() (*SegmentShape) {
  return &SegmentShape{}
}

func (seg * SegmentShape) CacheBB(p, rot Vect) (BB) {
  var l, r, s, t Float
  
  seg.ta = p.Add(seg.a.Rotate(rot));
  seg.tb = p.Add(seg.b.Rotate(rot));
  seg.tn = p.Add(seg.n.Rotate(rot));
    
  if(seg.ta.X < seg.tb.X){
    l = seg.ta.X
    r = seg.tb.X
  } else {
    l = seg.tb.X
    r = seg.ta.X
  }
  
  if(seg.ta.Y < seg.tb.Y){
    s = seg.ta.Y
    t = seg.tb.Y
  } else {
    s = seg.tb.Y
    t = seg.ta.Y
  }
  
  rad := seg.r
  return BBMake(l - rad, s - rad, r + rad, t + rad)
}

func (seg * SegmentShape) PointQuery(p Vect) (bool) {
  if !seg.BB.ContainsVect(p) { 
    return false 
  } 
   
  // Calculate normal distance from segment.
  dn    := seg.tn.Dot(p) - seg.ta.Dot(seg.tn)
  dist  := dn.Abs()      - seg.r
  if dist > 0.0 { 
    return false 
  }
  
  // Calculate tangential distance along segment.
  dt    := - seg.tn.Cross(p)
  dtMin := - seg.tn.Cross(seg.ta)     
  dtMax := - seg.tn.Cross(seg.tb)
  
  // Decision tree to decide which feature of the segment to collide with.
  if dt <= dtMin {
    if dt < (dtMin - seg.r) {
      return false
    } else {
      return seg.ta.Sub(p).Lengthsq() < (seg.r * seg.r) 
    }
  } else {
    if dt < dtMax {
      return true;
    } else {
      if dt < (dtMax + seg.r) {
        return seg.tb.Sub(p).Lengthsq() < (seg.r * seg.r)
      } else {
        return false
      }
    }
  }  
  return true
}


func (seg * SegmentShape) SegmentQuery(center Vect, a, b Vect) (info * SegmentQueryInfo) {
  
  n := seg.tn;
  // flip n if a is behind the axis
  if a.Dot(n) < seg.ta.Dot(n) { 
    n = n.Neg()
  }  
  
  info = &SegmentQueryInfo{}
  an  := a.Dot(n)
  bn  := b.Dot(n)
  d   := seg.ta.Dot(n) + seg.r
  t   := (d - an)/(bn - an)
  
  if Float(0.0) < t && t < Float(1.0) {    
    point := a.Lerp(b,t)
    dt    := - seg.tn.Cross(point)
    dtMin := - seg.tn.Cross(seg.ta)
    dtMax := - seg.tn.Cross(seg.tb)
        
    if(dtMin < dt && dt < dtMax){
      info.shape = seg.Shape
      info.t = t;
      info.n = n;
      
      return info; // don't continue on and check endcaps
    }
  }
  
  if seg.r > 0.0 {
    info1 := CircleSegmentQuery(seg.Shape, seg.ta, seg.r, a, b)
    info2 := CircleSegmentQuery(seg.Shape, seg.tb, seg.r, a, b)
    
    if info1.Hit() && !info2.Hit() {
      info = info1
    } else if info2.Hit() && !info1.Hit() {
      info = info2
    } else if info1.Hit() && info2.Hit() {
      if info1.t < info2.t {
        info = info1;
      } else {
        info = info2;
      }
    }
  }
  // return empty info
  return info 
}


var SegmentShapeClass * ShapeClass = &ShapeClass {SEGMENT_SHAPE}

func (seg *SegmentShape) Init(body * Body, a, b Vect, r Float) (*SegmentShape) {
  seg.a = a
  seg.b = b
  seg.n = b.Sub(a).Normalize().Perp()
  seg.r = r;  
  seg.Shape.Init(SegmentShapeClass, body)  
  return seg;
}

func SegmentShapeNew(body * Body, a, b Vect, r Float) (*SegmentShape) {
  return SegmentShapeAlloc().Init(body, a, b, r)
}

func (seg * SegmentShape) A() (Vect) {
  return seg.a
}

func (seg * SegmentShape) B() (Vect) {
  return seg.b
}

func (seg * SegmentShape) Normal() (Vect) {
  return seg.n
}

func (seg * SegmentShape) Radius() (Float) {
  return seg.r
}


// Unsafe API 
func (circle * CircleShape) SetRadius(r Float) (Float) {
  circle.r = r
  return circle.r
}

func (circle * CircleShape) SetOffset(o Vect) (Vect) {
  circle.c = o
  return circle.c
}


func (seg * SegmentShape) Setendpoints(a, b Vect) {
  seg.a = a
  seg.b = b
  seg.n = b.Sub(a).Normalize().Perp()
}

func (seg * SegmentShape) SetRadius(r Float) (Float) {
  seg.r = r
  return seg.r
}



