package tamias


// Number of frames that contact information should persist.
var CONTACT_PERSISTENCE = 1

// User collision handler function types.
type CollisionFunc func(arb * Arbiter, space * Space, data interface{}) (int)

// Structure for holding collision pair function information.
// Used internally.
type CollisionHandler struct {
  a, b CollisionType  
  begin, preSolve, postSolve, separate CollisionFunc
  data interface {}
}

const CP_MAX_CONTACTS_PER_ARBITER = 6

type ContactBufferHeader struct {
  stamp       int
  next        * ContactBufferHeader
  numContacts uint
} 

type CollisionFuncMap map[HashValue] CollisionFunc

type ContactMap map[HashValue] Contact

type Space struct {
  // *** User definable fields  
  // Number of iterations to use in the impulse solver to solve contacts.
  Iterations int
  
  // Number of iterations to use in the impulse solver to solve elastic collisions.
  ElasticIterations int
  
  // Default gravity to supply when integrating rigid body motions.
  Gravity Vect
  
  // Default damping to supply when integrating rigid body motions.
  Damping Float
  
  // *** Internally Used Fields  
  // When the space is locked, you should not add or remove objects;  
  locked int
  
  // Time stamp. Is incremented on every call to cpSpaceStep().
  stamp int

  // The static and active shape spatial hashes.
  staticShapes SpaceHash
  activeShapes SpaceHash
  
  // List of bodies in the system.
  bodies *Array
  
  // List of active arbiters for the impulse solver.
  arbiters, pooledArbiters *Array; 
  
  // Linked list ring of contact buffers.
  // Head is the current buffer. Tail is the oldest buffer.
  // The list points in the direction of tail.head.
  contactBuffersHead, contactBuffersTail *ContactBufferHeader;
  
  // List of buffers to be free()ed when destroying the space.
  // Not needed in Go
  // cpArray *allocatedBuffers;
  
  // Persistant contact set.
  contactSet ContactMap;
  
  // List of constraints in the system.
  constraints *Array;
  
  // Set of collisionpair functions.
  collFuncSet CollisionFuncMap;
  // Default collision handler.
  defaultHandler CollisionHandler;
    
  postStepCallbacks *HashSet;  
  
} 

/*

type contactSet [2]*Shape;

// Equal function for contactSet.
func (set * contactSet) Equals(arbi interface {}) bool { 
  arb, ok := arbi.(*Arbiter)
  if !ok { return false; }
  a       := set[0]
  b       := set[1]
  if a == arb.private_a && b == arb.private_b { return true }
  if b == arb.private_a && a == arb.private_b { return true }
  return false
}   

// Transformation function for contactSet.
// Not needed, I think. 
// contactSetTrans(cpShape **shapes, cpSpace *space)
func (check * CollisionHandler) Equals(pairi interface {}) bool {
  pair, ok := pairi.(*CollisionHandler)
  if !ok { return false; }
  if pair.a == check.a && pair.b == check.b { return true }
  if pair.b == check.a && pair.a == check.b { return true }
  return false  
}


type PostStepFunc func(obj, data interface{})

type postStepCallback struct {
  fun PostStepFunc
  obj, data interface{}
}

func (a * postStepCallback) Equals(bi interface{}) bool { 
  b, ok := arbi.(*CollisionHandler)
  if !ok { return false; }
  return a.obj == b.obj   
}

// Default collision functions.
func alwaysCollide(arb * Arbiter, space * Space, data interface{}) (int) { 
  return 1;
}

func nothing(arb * Arbiter, space * Space, data interface{}) (int) {
  return 0;
}

func (shape * Shape) GetBB() (BB) {
  return shape.BB
}  

const CONTACTS_BUFFER_SIZE = 100

type ContactBuffer struct {
  header ContactBufferHeader;
  contacts [CONTACTS_BUFFER_SIZE]Contact;
}

func AllocContactBufferHeader() (*ContactBufferHeader) {
  return &ContactBufferHeader{}
}

func (header *ContactBufferHeader) Init(space * Space) (*ContactBufferHeader) {
  header.stamp        = space.stamp
  header.next         = space.contactBuffersTail
  header.numContacts  = 0
  return header
}

func (header *ContactBufferHeader) ContactBufferHeaderNew(space * Space) (
      *ContactBufferHeader) {
  return AllocContactBufferHeader().Init(space) 
}  

func SpaceAlloc() (*Space) {
  return &Space{}  
}

var DEFAULT_DIM_SIZE            = Float(100.0)
var DEFAULT_COUNT               = 1000
var DEFAULT_ITERATIONS          = 10
var DEFAULT_ELASTIC_ITERATIONS  = 0


var defaultHandler = CollisionHandler{ 0, 0, alwaysCollide, alwaysCollide, nothing, nothing, nil};

func (space *Space) Init() (*Space) {
  space.iterations        = DEFAULT_ITERATIONS
  space.elasticIterations = DEFAULT_ELASTIC_ITERATIONS
  space.Gravity           = VZERO
  space.Damping           = Float(1.0)
  space.locked            = 0
  space.stamp             = 0
  space.staticShapes      = SpaceHashNew(DEFAULT_DIM_SIZE, DEFAULT_COUNT)
  space.activeShapes      = SpaceHashNew(DEFAULT_DIM_SIZE, DEFAULT_COUNT)
  space.bodies            = ArrayNew(0)
  space.arbiters          = ArrayNew(0)
  space.pooledArbiters    = ArrayNew(0)
  header                 := ContactBufferHeaderNew(space)
  space.contactBuffersTail= header
  space.contactBuffersHead= header
  header.next             = header 
  // set up ring buffer in cyclical way
  space.contactSet        = make(ContactSet)  
  space.constraints       = ArrayNew(0)
  space.defaultHandler    = defaultHandler
  space.collFuncSet       = make(CollisionFuncMap)
  space.postStepCallbacks = HashSetNew(0)
  
  return space
}  

func (space *Space) New() (*Space) {
  return SpaceAlloc().Init()
}


func (space * Space) AddCollisionHandler(a, b CollisionType,
  begin, preSolve, postSolve, separate CollisionFunc, data interface{}) {
  // Remove any old function so the new one will get added.
  space.RemoveCollisionHandler(space, a, b)
  handler = CollisionHandler { a , b, begin, 
            presolve, postSolve, separate , data } 
  space.collFuncSet[HASH_PAIR(a, b)] = &handler
}

func (space * Space)RemoveCollisionHandler(a, b CollisionType) {
  space.collFuncSet[HASH_PAIR(a, b)] = nil, false
}

func (space * Space) SetDefaultHandler( a, b CollisionType, 
  begin, preSolve, postSolve, separate CollisionFunc, data interface{}) {
  // Remove any old function so the new one will get added.
  space.RemoveCollisionHandler(space, a, b)
  handler = CollisionHandler { a , b, begin, 
            presolve, postSolve, separate , data }             
  space.defaultHandler = &handler
}


func (space * Space) AssertUnlocked() {
  Assert(!space.locked,  "This addition/removal cannot be done safely during a call to cpSpaceStep(). Put these calls into a Post Step Callback.")
}
  
func (space * Space) AddShape(shape * Shape) (* Shape) {
  Assert(shape.body != nil, "Cannot add a shape with a nil body.")
  old := space.activeShapes.Find(shape, shape.hashid)
  Assert(old == nil, "Cannot add the same shape more than once")
  space.AssertUnlocked()
  shape.CacheBB()
  space.activeShapes.Insert(shape, shape.hashid, shape.bb)
  return shape
}
  
func (space * Space) AddStaticShape(shape * Shape) (* Shape) {
  Assert(shape.body != nil, "Cannot add a static shape with a nil body.")
  old := space.staticShapes.Find(shape, shape.hashid)
  Assert(old == nil, "Cannot add the same static shape more than once")
  space.AssertUnlocked()
  shape.CacheBB()
  space.staticShapes.Insert(shape, shape.hashid, shape.bb)
  return shape
}


func (space * Space) AddBody(body * Body) (* Body) {
  Assert(!space.bodies.Contains(body), 
          "Cannot add the same body more than once.")
  space.Bodies.Push(body)
  return body
}


func (space * Space) AddConstraint(constraint * Constraint) {
  Assert(!space.constraints.Contains(constraint), "Cannot add the same constraint more than once.")  
  space.constraints.Push(constraint);  
  return constraint;
}

typedef struct removalContext {
  cpSpace *space;
  cpShape *shape;
} removalContext;

// Hashset filter func to throw away old arbiters.
static int
contactSetFilterRemovedShape(cpArbiter *arb, removalContext *context)
{
  if(context.shape == arb.private_a || context.shape == arb.private_b){
    arb.handler.separate(arb, context.space, arb.handler.data);
    cpArrayPush(context.space.pooledArbiters, arb);
    return 0;
  }
  
  return 1;
}


func (space * Space) RemoveShape(shape * Shape) {
  space.AssertUnlocked()    
  for k, v := range space.contactSet { 
    if shape == v.private_a || shape == v.private_b {
      v.handler.separate(v, space, v.handler.data)
      space.contactSet[k] = nil, false
      // remove all contacts of this shape 
    }
  }  
  space.activeShapes.Remove(shape, shape.hashid)
}

func (space * Space) RemoveStaticShape(shape * Shape) {
  space.AssertUnlocked()
  for k, v := range space.contactSet { 
    if shape == v.private_a || shape == v.private_b {
      v.handler.separate(v, space, v.handler.data)
      space.contactSet[k] = nil, false
      // remove all contacts of this shape 
    }
  }  
  space.staticShapes.Remove(shape, shape.hashid)
}


func (space * Space) RemoveBody(body * Body) {
  space.AssertUnlocked()
  space.bodies.DeleteObj(body)  
}

func (space * Space) RemoveConstraint(constraint * Constraint) {
  space.AssertUnlocked()
  space.constraints.DeleteObj(constraint)  
}

func (space * Space) AddPostStepCallback(fun * PostStepFunc, 
      obj, data interface{}) {
  callback :=  &postStepCallback{fun, obj, data};
  space.postStepCallbacks.Insert(callback, HashValue(obj))
  // BDM: This also sucks
}

func removeShape(shape * Shape, space * space) {
   space.RemoveShape(shape)
}   

func (space * Space) PostStepRemoveShape(shape * shape) {
  shape.AddPostStepCallback(removeShape, shape, space)
} 

type pointQueryContext struct {
  layers  Layers;
  group   Group;
  fun     SpacePointQueryFunc;
  data    interface{};
}

func pointQueryHelper(point * Vect, shape * Shape, 
      context * pointQueryContext) (bool)  {

  if shape.group > 0 && context.group == shape.group  { return false } 
  // no collision is in the same nonzero group
  if (context.layers & shape.layers) == 0  { return false }
  // no collision if in different, non-overlapping layers
  
  // call the callback if the shape responds true to the point query
  if shape.PointQuery(*point) {   
    return context.fun(shape, context.data)
  } 
}

func (space * Space) SpacePointQuery(point Vect, layers Layers, 
  group Group, fun SpacePointQueryFunc, data interface{}) {
  context := pointQueryContext{layers, group, fun, data};
  space.activeShapes.PointQuery(point, pointQueryHelper, &context);
  space.staticShapes.PointQuery(point, pointQueryHelper, &context);
}

/*
Allthis sucks a bit. Better recode it to be more Go-like
static void
rememberLastPointQuery(cpShape *shape, cpShape **outShape)
{
  (*outShape) = shape;
}

cpShape *
cpSpacePointQueryFirst(cpSpace *space, cpVect point, cpLayers layers, cpGroup group)
{
  cpShape *shape = NULL;
  cpSpacePointQuery(space, point, layers, group, (cpSpacePointQueryFunc)rememberLastPointQuery, &shape);
  
  return shape;
}
*/

func (space * Space) EachBody() (chan *Body) {
  out := make(chan * Body)
  go func() {   
    for i:=0; i < space.bodies.Size(); i++ {
      out <- space.bodies.Index(i).(*Body)
    } 
  }()
  return out
} 

/*
typedef struct segQueryContext {
  cpVect start, end;
  cpLayers layers;
  cpGroup group;
  cpSpaceSegmentQueryFunc func;
  int anyCollision;
} segQueryContext;

static cpFloat
segQueryFunc(segQueryContext *context, cpShape *shape, void *data)
{
  cpSegmentQueryInfo info;
  
  if(
    !(shape.group && context.group == shape.group) && (context.layers&shape.layers) &&
    cpShapeSegmentQuery(shape, context.start, context.end, &info)
  ){
    if(context.func){
      context.func(shape, info.t, info.n, data);
    }
    
    context.anyCollision = 1;
  }
  
  return 1.0f;
}

int
cpSpaceSegmentQuery(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSpaceSegmentQueryFunc func, void *data)
{
  segQueryContext context = {
    start, end,
    layers, group,
    func,
    0,
  };
  
  cpSpaceHashSegmentQuery(space.staticShapes, &context, start, end, 1.0f, (cpSpaceHashSegmentQueryFunc)segQueryFunc, data);
  cpSpaceHashSegmentQuery(space.activeShapes, &context, start, end, 1.0f, (cpSpaceHashSegmentQueryFunc)segQueryFunc, data);
  
  return context.anyCollision;
}

typedef struct segQueryFirstContext {
  cpVect start, end;
  cpLayers layers;
  cpGroup group;
} segQueryFirstContext;

static cpFloat
segQueryFirst(segQueryFirstContext *context, cpShape *shape, cpSegmentQueryInfo *out)
{
  cpSegmentQueryInfo info;// = {NULL, 1.0f, cpvzero};
  if(
    !(shape.group && context.group == shape.group) && (context.layers&shape.layers) &&
    cpShapeSegmentQuery(shape, context.start, context.end, &info)
  ){
    if(info.t < out.t){
      out.shape = info.shape;
      out.t = info.t;
      out.n = info.n;
    }
    
    return info.t;
  }
  
  return 1.0f;
}

cpShape *
cpSpaceSegmentQueryFirst(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSegmentQueryInfo *out)
{
  cpSegmentQueryInfo info = {NULL, 1.0f, cpvzero};
  if(out){
    (*out) = info;
  } else {
    out = &info;
  }
  
  out.t = 1.0f;
  
  segQueryFirstContext context = {
    start, end,
    layers, group
  };
  
  cpSpaceHashSegmentQuery(space.staticShapes, &context, start, end, 1.0f, (cpSpaceHashSegmentQueryFunc)segQueryFirst, out);
  cpSpaceHashSegmentQuery(space.activeShapes, &context, start, end, out.t, (cpSpaceHashSegmentQueryFunc)segQueryFirst, out);
  
  return out.shape;
}

#pragma mark BB Query functions

typedef struct bbQueryContext {
  cpLayers layers;
  cpGroup group;
  cpSpaceBBQueryFunc func;
  void *data;
} bbQueryContext;

static void 
bbQueryHelper(cpBB *bb, cpShape *shape, bbQueryContext *context)
{
  if(
    !(shape.group && context.group == shape.group) && (context.layers&shape.layers) &&
    cpBBintersects(*bb, shape.bb)
  ){
    context.func(shape, context.data);
  }
}

void
cpSpaceBBQuery(cpSpace *space, cpBB bb, cpLayers layers, cpGroup group, cpSpaceBBQueryFunc func, void *data)
{
  bbQueryContext context = {layers, group, func, data};
  cpSpaceHashQuery(space.activeShapes, &bb, bb, (cpSpaceHashQueryFunc)bbQueryHelper, &context);
  cpSpaceHashQuery(space.staticShapes, &bb, bb, (cpSpaceHashQueryFunc)bbQueryHelper, &context);
}

#pragma mark Spatial Hash Management

// Iterator function used for updating shape BBoxes.
static void
updateBBCache(cpShape *shape, void *unused)
{
  cpShapeCacheBB(shape);
}

void
cpSpaceResizeStaticHash(cpSpace *space, cpFloat dim, int count)
{
  cpSpaceHashResize(space.staticShapes, dim, count);
  cpSpaceHashRehash(space.staticShapes);
}

void
cpSpaceResizeActiveHash(cpSpace *space, cpFloat dim, int count)
{
  cpSpaceHashResize(space.activeShapes, dim, count);
}

void 
cpSpaceRehashStatic(cpSpace *space)
{
  cpSpaceHashEach(space.staticShapes, (cpSpaceHashIterator)&updateBBCache, NULL);
  cpSpaceHashRehash(space.staticShapes);
}

#pragma mark Collision Detection Functions

static cpContactBufferHeader *
cpSpaceGetFreeContactBuffer(cpSpace *space)
{
  if(space.stamp - space.contactBuffersTail.stamp > cp_contact_persistence){
    cpContactBufferHeader *header = space.contactBuffersTail;
    space.contactBuffersTail = header.next;
    
    return cpContactBufferHeaderInit(header, space);
  } else {
    cpContactBufferHeader *header = cpSpaceAllocContactBuffer(space);
    return cpContactBufferHeaderInit(header, space);
  }
}

static void
cpSpacePushNewContactBuffer(cpSpace *space)
{
//  for(cpContactBuffer *buffer = space.contactBuffersTail; buffer != space.contactBuffersHead; buffer = buffer.next){
//    printf("%p . ", buffer);
//  }
//  printf("%p (head)\n", space.contactBuffersHead);
  
  cpContactBufferHeader *buffer = cpSpaceGetFreeContactBuffer(space);
  space.contactBuffersHead.next = buffer;
  space.contactBuffersHead = buffer;
}

static inline int
queryReject(cpShape *a, cpShape *b)
{
  return
    // BBoxes must overlap
    !cpBBintersects(a.bb, b.bb)
    // Don't collide shapes attached to the same body.
    || a.body == b.body
    // Don't collide objects in the same non-zero group
    || (a.group && b.group && a.group == b.group)
    // Don't collide objects that don't share at least on layer.
    || !(a.layers & b.layers);
}

// Callback from the spatial hash.
static void
queryFunc(cpShape *a, cpShape *b, cpSpace *space)
{
  // Reject any of the simple cases
  if(queryReject(a,b)) return;
  
  // Find the collision pair function for the shapes.
  struct{cpCollisionType a, b;} ids = {a.collision_type, b.collision_type};
  cpHashValue collHashID = CP_HASH_PAIR(a.collision_type, b.collision_type);
  cpCollisionHandler *handler = (cpCollisionHandler *)cpHashSetFind(space.collFuncSet, collHashID, &ids);
  
  int sensor = a.sensor || b.sensor;
  if(sensor && handler == &space.defaultHandler) return;
  
  // Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
  if(a.klass.type > b.klass.type){
    cpShape *temp = a;
    a = b;
    b = temp;
  }
  
  if(space.contactBuffersHead.numContacts + CP_MAX_CONTACTS_PER_ARBITER > CP_CONTACTS_BUFFER_SIZE){
    // contact buffer could overflow on the next collision, push a fresh one.
    cpSpacePushNewContactBuffer(space);
  }
  
  // Narrow-phase collision detection.
  cpContact *contacts = ((cpContactBuffer *)(space.contactBuffersHead)).contacts + space.contactBuffersHead.numContacts;
  int numContacts = cpCollideShapes(a, b, contacts);
  if(!numContacts) return; // Shapes are not colliding.
  space.contactBuffersHead.numContacts += numContacts;
  
  // Get an arbiter from space.contactSet for the two shapes.
  // This is where the persistant contact magic comes from.
  cpShape *shape_pair[] = {a, b};
  cpHashValue arbHashID = CP_HASH_PAIR((size_t)a, (size_t)b);
  cpArbiter *arb = (cpArbiter *)cpHashSetInsert(space.contactSet, arbHashID, shape_pair, space);
  cpArbiterUpdate(arb, contacts, numContacts, handler, a, b); // retains the contacts array
  
  // Call the begin function first if it's the first step
  if(arb.stamp == -1 && !handler.begin(arb, space, handler.data)){
    cpArbiterIgnore(arb); // permanently ignore the collision until separation
  }
  
  if(
    // Ignore the arbiter if it has been flagged
    (arb.state != cpArbiterStateIgnore) && 
    // Call preSolve
    handler.preSolve(arb, space, handler.data) &&
    // Process, but don't add collisions for sensors.
    !sensor
  ){
    cpArrayPush(space.arbiters, arb);
  } else {
//    cpfree(arb.contacts);
    space.contactBuffersHead.numContacts -= numContacts;
    arb.contacts = NULL;
    arb.numContacts = 0;
  }
  
  // Time stamp the arbiter so we know it was used recently.
  arb.stamp = space.stamp;
}

// Iterator for active/static hash collisions.
static void
active2staticIter(cpShape *shape, cpSpace *space)
{
  cpSpaceHashQuery(space.staticShapes, shape, shape.bb, (cpSpaceHashQueryFunc)queryFunc, space);
}

// Hashset filter func to throw away old arbiters.
static int
contactSetFilter(cpArbiter *arb, cpSpace *space)
{
  int ticks = space.stamp - arb.stamp;
  
  // was used last frame, but not this one
  if(ticks == 1){
    arb.handler.separate(arb, space, arb.handler.data);
    arb.stamp = -1; // mark it as a new pair again.
  }
  
  if(ticks >= cp_contact_persistence){
    cpArrayPush(space.pooledArbiters, arb);
    return 0;
  }
  
  return 1;
}

// Hashset filter func to call and throw away post step callbacks.
static int
postStepCallbackSetFilter(postStepCallback *callback, cpSpace *space)
{
  callback.func(space, callback.obj, callback.data);
  cpfree(callback);
  
  return 0;
}

#pragma mark All Important cpSpaceStep() Function

void
cpSpaceStep(cpSpace *space, cpFloat dt)
{
  if(!dt) return; // don't step if the timestep is 0!
  
  cpFloat dt_inv = 1.0f/dt;

  cpArray *bodies = space.bodies;
  cpArray *constraints = space.constraints;
  
  space.locked = 1;
  
  // Empty the arbiter list.
  space.arbiters.num = 0;

  // Integrate positions.
  for(int i=0; i<bodies.num; i++){
    cpBody *body = (cpBody *)bodies.arr[i];
    body.position_func(body, dt);
  }
  
  // Pre-cache BBoxes and shape data.
  cpSpaceHashEach(space.activeShapes, (cpSpaceHashIterator)updateBBCache, NULL);
  
  // Collide!
  cpSpacePushNewContactBuffer(space);
  cpSpaceHashEach(space.activeShapes, (cpSpaceHashIterator)active2staticIter, space);
  cpSpaceHashQueryRehash(space.activeShapes, (cpSpaceHashQueryFunc)queryFunc, space);
  
  // Clear out old cached arbiters and dispatch untouch functions
  cpHashSetFilter(space.contactSet, (cpHashSetFilterFunc)contactSetFilter, space);

  // Prestep the arbiters.
  cpArray *arbiters = space.arbiters;
  for(int i=0; i<arbiters.num; i++)
    cpArbiterPreStep((cpArbiter *)arbiters.arr[i], dt_inv);

  // Prestep the constraints.
  for(int i=0; i<constraints.num; i++){
    cpConstraint *constraint = (cpConstraint *)constraints.arr[i];
    constraint.klass.preStep(constraint, dt, dt_inv);
  }

  for(int i=0; i<space.elasticIterations; i++){
    for(int j=0; j<arbiters.num; j++)
      cpArbiterApplyImpulse((cpArbiter *)arbiters.arr[j], 1.0f);
      
    for(int j=0; j<constraints.num; j++){
      cpConstraint *constraint = (cpConstraint *)constraints.arr[j];
      constraint.klass.applyImpulse(constraint);
    }
  }

  // Integrate velocities.
  cpFloat damping = cpfpow(1.0f/space.damping, -dt);
  for(int i=0; i<bodies.num; i++){
    cpBody *body = (cpBody *)bodies.arr[i];
    body.velocity_func(body, space.gravity, damping, dt);
  }

  for(int i=0; i<arbiters.num; i++)
    cpArbiterApplyCachedImpulse((cpArbiter *)arbiters.arr[i]);
  
  // run the old-style elastic solver if elastic iterations are disabled
  cpFloat elasticCoef = (space.elasticIterations ? 0.0f : 1.0f);
  
  // Run the impulse solver.
  for(int i=0; i<space.iterations; i++){
    for(int j=0; j<arbiters.num; j++)
      cpArbiterApplyImpulse((cpArbiter *)arbiters.arr[j], elasticCoef);
      
    for(int j=0; j<constraints.num; j++){
      cpConstraint *constraint = (cpConstraint *)constraints.arr[j];
      constraint.klass.applyImpulse(constraint);
    }
  }
  
  space.locked = 0;
  
  // run the post solve callbacks
  for(int i=0; i<arbiters.num; i++){
    cpArbiter *arb = arbiters.arr[i];
    
    cpCollisionHandler *handler = arb.handler;
    handler.postSolve(arb, space, handler.data);
    
    arb.state = cpArbiterStateNormal;
  }
  
  // Run the post step callbacks
  // Use filter as an easy way to clear out the queue as it runs
  cpHashSetFilter(space.postStepCallbacks, (cpHashSetFilterFunc)postStepCallbackSetFilter, space);
  
//  cpFloat dvsq = cpvdot(space.gravity, space.gravity);
//  dvsq *= dt*dt * space.damping*space.damping;
//  for(int i=0; i<bodies.num; i++)
//    cpBodyMarkLowEnergy(bodies.arr[i], dvsq, space.sleepTicks);
  
  // Increment the stamp.
  space.stamp++;
}
*/
