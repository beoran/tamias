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



func (circle * Circle) SegmentQuery(center Vect, r Float, a, b Vect) (info * SegmentQueryInfo)
{
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
}

const CircleShapeClass = ShapeClass { CP_CIRCLE_SHAPE }


func (circle * CircleShape) Init(cpBody *body, cpFloat radius, cpVect offset) (* CircleShape) {
	circle.c = offset;
	circle.r = radius;	
	Shape.Init(circle, CircleShapeClass, body);	
	return circle;
}

/*
cpShape *
cpCircleShapeNew(cpBody *body, cpFloat radius, cpVect offset)
{
	return (cpShape *)cpCircleShapeInit(cpCircleShapeAlloc(), body, radius, offset);
}

CP_DefineShapeGetter(cpCircleShape, cpVect, c, Offset)
CP_DefineShapeGetter(cpCircleShape, cpFloat, r, Radius)

cpSegmentShape *
cpSegmentShapeAlloc(void)
{
	return (cpSegmentShape *)cpcalloc(1, sizeof(cpSegmentShape));
}

static cpBB
cpSegmentShapeCacheData(cpShape *shape, cpVect p, cpVect rot)
{
	cpSegmentShape *seg = (cpSegmentShape *)shape;
	
	seg->ta = cpvadd(p, cpvrotate(seg->a, rot));
	seg->tb = cpvadd(p, cpvrotate(seg->b, rot));
	seg->tn = cpvrotate(seg->n, rot);
	
	cpFloat l,r,s,t;
	
	if(seg->ta.x < seg->tb.x){
		l = seg->ta.x;
		r = seg->tb.x;
	} else {
		l = seg->tb.x;
		r = seg->ta.x;
	}
	
	if(seg->ta.y < seg->tb.y){
		s = seg->ta.y;
		t = seg->tb.y;
	} else {
		s = seg->tb.y;
		t = seg->ta.y;
	}
	
	cpFloat rad = seg->r;
	return cpBBNew(l - rad, s - rad, r + rad, t + rad);
}

static int
cpSegmentShapePointQuery(cpShape *shape, cpVect p){
	if(!cpBBcontainsVect(shape->bb, p)) return 0;
	
	cpSegmentShape *seg = (cpSegmentShape *)shape;
	
	// Calculate normal distance from segment.
	cpFloat dn = cpvdot(seg->tn, p) - cpvdot(seg->ta, seg->tn);
	cpFloat dist = cpfabs(dn) - seg->r;
	if(dist > 0.0f) return 0;
	
	// Calculate tangential distance along segment.
	cpFloat dt = -cpvcross(seg->tn, p);
	cpFloat dtMin = -cpvcross(seg->tn, seg->ta);
	cpFloat dtMax = -cpvcross(seg->tn, seg->tb);
	
	// Decision tree to decide which feature of the segment to collide with.
	if(dt <= dtMin){
		if(dt < (dtMin - seg->r)){
			return 0;
		} else {
			return cpvlengthsq(cpvsub(seg->ta, p)) < (seg->r*seg->r);
		}
	} else {
		if(dt < dtMax){
			return 1;
		} else {
			if(dt < (dtMax + seg->r)) {
				return cpvlengthsq(cpvsub(seg->tb, p)) < (seg->r*seg->r);
			} else {
				return 0;
			}
		}
	}
	
	return 1;	
}

static void
cpSegmentShapeSegmentQuery(cpShape *shape, cpVect a, cpVect b, cpSegmentQueryInfo *info)
{
	cpSegmentShape *seg = (cpSegmentShape *)shape;
	cpVect n = seg->tn;
	// flip n if a is behind the axis
	if(cpvdot(a, n) < cpvdot(seg->ta, n))
		n = cpvneg(n);
	
	cpFloat an = cpvdot(a, n);
	cpFloat bn = cpvdot(b, n);
	cpFloat d = cpvdot(seg->ta, n) + seg->r;
	
	cpFloat t = (d - an)/(bn - an);
	if(0.0f < t && t < 1.0f){
		cpVect point = cpvlerp(a, b, t);
		cpFloat dt = -cpvcross(seg->tn, point);
		cpFloat dtMin = -cpvcross(seg->tn, seg->ta);
		cpFloat dtMax = -cpvcross(seg->tn, seg->tb);
		
		if(dtMin < dt && dt < dtMax){
			info->shape = shape;
			info->t = t;
			info->n = n;
			
			return; // don't continue on and check endcaps
		}
	}
	
	if(seg->r) {
		cpSegmentQueryInfo info1; info1.shape = NULL;
		cpSegmentQueryInfo info2; info2.shape = NULL;
		circleSegmentQuery(shape, seg->ta, seg->r, a, b, &info1);
		circleSegmentQuery(shape, seg->tb, seg->r, a, b, &info2);
		
		if(info1.shape && !info2.shape){
			(*info) = info1;
		} else if(info2.shape && !info1.shape){
			(*info) = info2;
		} else if(info1.shape && info2.shape){
			if(info1.t < info2.t){
				(*info) = info1;
			} else {
				(*info) = info2;
			}
		}
	}
}

static const cpShapeClass cpSegmentShapeClass = {
	CP_SEGMENT_SHAPE,
	cpSegmentShapeCacheData,
	NULL,
	cpSegmentShapePointQuery,
	cpSegmentShapeSegmentQuery,
};

cpSegmentShape *
cpSegmentShapeInit(cpSegmentShape *seg, cpBody *body, cpVect a, cpVect b, cpFloat r)
{
	seg->a = a;
	seg->b = b;
	seg->n = cpvperp(cpvnormalize(cpvsub(b, a)));
	
	seg->r = r;
	
	cpShapeInit((cpShape *)seg, &cpSegmentShapeClass, body);
	
	return seg;
}

cpShape*
cpSegmentShapeNew(cpBody *body, cpVect a, cpVect b, cpFloat r)
{
	return (cpShape *)cpSegmentShapeInit(cpSegmentShapeAlloc(), body, a, b, r);
}

CP_DefineShapeGetter(cpSegmentShape, cpVect, a, A)
CP_DefineShapeGetter(cpSegmentShape, cpVect, b, B)
CP_DefineShapeGetter(cpSegmentShape, cpVect, n, Normal)
CP_DefineShapeGetter(cpSegmentShape, cpFloat, r, Radius)

// Unsafe API (chipmunk_unsafe.h)

void
cpCircleShapeSetRadius(cpShape *shape, cpFloat radius)
{
	cpAssert(shape->klass == &cpCircleShapeClass, "Shape is not a circle shape.");
	cpCircleShape *circle = (cpCircleShape *)shape;
	
	circle->r = radius;
}

void
cpCircleShapeSetOffset(cpShape *shape, cpVect offset)
{
	cpAssert(shape->klass == &cpCircleShapeClass, "Shape is not a circle shape.");
	cpCircleShape *circle = (cpCircleShape *)shape;
	
	circle->c = offset;
}

void
cpSegmentShapeSetEndpoints(cpShape *shape, cpVect a, cpVect b)
{
	cpAssert(shape->klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	cpSegmentShape *seg = (cpSegmentShape *)shape;
	
	seg->a = a;
	seg->b = b;
	seg->n = cpvperp(cpvnormalize(cpvsub(b, a)));
}

void
cpSegmentShapeSetRadius(cpShape *shape, cpFloat radius)
{
	cpAssert(shape->klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	cpSegmentShape *seg = (cpSegmentShape *)shape;
	
	seg->r = radius;
}


*/