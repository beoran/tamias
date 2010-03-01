package tamias

type struct SegmentQueryInfo {
  // shape that was hit, NULL if no collision
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
type HashValue int

// Enumeration of shape types.
type ShapeType int

const (
  CP_CIRCLE_SHAPE	= ShapeType(0)
  CP_SEGMENT_SHAPE	= ShapeType(1)
  CP_POLY_SHAPE		= ShapeType(2)
  CP_NUM_SHAPES		= ShapeType(3)
  CP_NO_GROUP		= GroupType(0)  
)  

const (
  CP_LAYER_0		= 1 << iota
  CP_LAYER_1		= 1 << iota
  CP_LAYER_2		= 1 << iota
  CP_LAYER_3		= 1 << iota
  CP_LAYER_4		= 1 << iota
  CP_LAYER_5		= 1 << iota
  CP_LAYER_6		= 1 << iota
  CP_LAYER_7		= 1 << iota
  CP_LAYER_8		= 1 << iota
  CP_LAYER_9		= 1 << iota
  CP_LAYER_10		= 1 << iota
  CP_LAYER_11		= 1 << iota
  CP_LAYER_12		= 1 << iota
  CP_LAYER_13		= 1 << iota
  CP_ALL_LAYERS		= -1
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

func (shape * CircleShape) Radius() (Float) {
  return shape.r
}

func (shape * CircleShape) Offset() (Float) {
  // return shape.
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

(info *SegmentQuery) HitPoint(cpVect start, cpVect end) (Vect) {
	return start.Lerp(end, info.t) 
}

(info *SegmentQuery) HitDist(cpVect start, cpVect end) (Float) {
	return start.Dist(start, end) * info.t
}

var ( 
  SHAPE_ID_COUNTER := HashValue(0)
)  

func ResetShapeIdCounter() {
	SHAPE_ID_COUNTER = HashValue(0)
}

func (shape * Shape) Init(klass *ShapeClass, cpbody *Body)
{
	shape.ShapeClass = klass;	
	shape.hashid 	 = SHAPE_ID_COUNTER
	SHAPE_ID_COUNTER++	
	
	shape.body 	= body;
	shape.sensor 	= 0;
	
	shape.e 	= Float(0.0)
	shape.u 	= Float(0.0f)
	shape.surface_v = VZERO
	
	shape.collision_type 	= 0
	shape.group 		= CP_NO_GROUP
	shape.layers 		= CP_ALL_LAYERS
	shape.data 		= nil	
	shape.CacheBB()	
	return shape;
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




func CircleShapeAlloc() (* cpCircleShape) {
	return &cpCircleShape{};
}

 
func bbFromCircle(c Vect, r Float) (BB)
{
	return BBNew(c.x-r, c.y-r, c.x+r, c.y+r);
}

func (circle * Circle) CacheBB(p Vect, rot Vect) (BB) {
  circle.tc	=	p.Add(circle.c.Rotate(rot))
  return bbFromCircle(circle.tc, circle.r)
}

func (circle * Circle) PointQuery(cpVect p) (bool) {	
	return circle->tc.Near(p, circle->r)
}



func CircleSegmentQuery(shape *Shape, center Vect, r Float, a, b Vect) (info * SegmentQueryInfo) {
	// umm... gross I normally frown upon such things (sic)
	aa := a.Sub(center);
	bb := b.Sub(center);
	
	qa := aa.Dot(aa) - Float(2.0)*aa.Dot(bb) + bb.Dot(bb)
	qb := Float(-2.0)*aa.Dot(aa) + Float(2.0)*aa.Dot(bb)
	qc := aa.Dot(aa) - r*r;
	
	det:= qb*qb - Float(4.)0*qa*qc;
	info= &SeqmentQueryInfo{}	
	if det >= Float(0.0) {
		t := (-qb - det.Sqrt())/(Float(2.0)*qa);
		if Float(0.0)<= t && t <= Float(1.0) { 
			info->shape = shape;
			info->t = t;
			info->n = a.Lerp(b, t).Normalize()
		}
	}
  return info 
}


func (circle * Circle) SegmentQuery(a, b Vect) (info * SegmentQueryInfo) {
  return CircleSegmentQuery(circle, circle.c, cicle.r, a, b)
}

const CircleShapeClass = ShapeClass { CP_CIRCLE_SHAPE }


func (circle * CircleShape) Init(cpBody *body, cpFloat radius, cpVect offset) (* CircleShape) {
	circle.c = offset;
	circle.r = radius;	
	Shape.Init(circle, CircleShapeClass, body);	
	return circle;
}


func CircleShapeNew(radius Float, offset Vect) (*CircleShape) {
  return CircleShapeAlloc().Init(radius, offset)
}

func (circle * CircleShape) Radius() (Float) {
  return circle.radius
}

func (circle * CircleShape) Offset() (Vect) {
  return circle.offset
}

func SegmentShapeAlloc() (*SegmentShape) {
  return &SegmentShape{}
}

(seg * SegmentShape) CacheBB(p, rot Vect) (BB) {
  l, r, s, t Float
  
  seg.ta = p.Add(seg.a.Rotate(rot));
  seg.tb = p.Add(seg.b.Rotate(rot));
  seg.tn = p.Add(seg.n.Rotate(rot));
    
  if(seg.ta.x < seg.tb.x){
    l = seg.ta.x
    r = seg.tb.x
  } else {
    l = seg.tb.x
    r = seg.ta.x
  }
  
  if(seg.ta.y < seg.tb.y){
    s = seg.ta.y
    t = seg.tb.y
  } else {
    s = seg.tb.y
    t = seg.ta.y
  }
  
  rad := seg->r
  return BBNew(l - rad, s - rad, r + rad, t + rad)
}

(seg * SegmentShape) PointQuery(p Vect) (bool) {
  if !seg.bb.ContainsVect(p) { 
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
    if dt < (dtMin - seg->r) {
      return false
    } else {
      return seg.ta.Sub(p).Lengthsq() < (seg.r * seg.r) 
    }
  } else {
    if dt < dtMax {
      return true;
    } else {
      if dt < (dtMax + seg->r) {
        return seg.tb.Sub(p).Lengthsq() < (seg.r * seg.r)
      } else {
        return false
      }
    }
  }  
  return true
}


func (segment * SegmentShape) SegmentQuery(center Vect, a, b Vect) (info * SegmentQueryInfo) {
  
  n := seg.tn;
  // flip n if a is behind the axis
  if a.Dot(n) < seg.ta.Dot(n) { 
    n := n.Neg()
  }  
  
  info = &SegmentQueryInfo{}
  an  := a.Dot(n)
  bn  := b.Dot(n)
  d   := seg.ta.Dot(n) + seg.r
  t   := (d - an)/(bn - an)
  
  if  Float(0.0) < t && t < Float(1.0f) {
    
    point := a.Lerp(b,t)
    dt    := - seg.tn.Cross(point)
    dtMin := - seg.tn.Cross(seg.ta)
    dtMax := - seg.tn.Cross(seg.tb)
        
    if(dtMin < dt && dt < dtMax){
      info.shape = shape;
      info.t = t;
      info.n = n;
      
      return info; // don't continue on and check endcaps
    }
  }
  
  if seg.r {
    info1 := circleSegmentQuery(shape, seg->ta, seg->r, a, b)
    info2 := circleSegmentQuery(shape, seg->tb, seg->r, a, b)
    
    if info1.shape && !info2.shape {
      info = info1
    } else if info2.shape && !info1.shape {
      info = info2
    } else if info1.shape && info2.shape {
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


var SegmentShapeClass = &ShapeClass {SEGMENT_SHAPE}

func (seg *SegmentShape) Init(body * Body, a, b Vect, r Float) (*SegmentShape) {
  seg.a = a
  seg.b = b
  seg.n = b.Sub(a).Normalize().Perp()
  seg.r = r;  
  Shape.Init(seg, SegmentShapeClass, body)  
  return seg;
}

func SegmentShapeNew(body * Body, a, b Vect, r Float) (*SegmentShape) {
  SegmentShapeAlloc().Init(body, a, b, r)
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
  seg.n = b.Sub(a).Normalize.Perp()
}

func (seg * SegmentShape) SetRadius(r Float) (Float) {
  seg.r = r
  return seg.r
}


