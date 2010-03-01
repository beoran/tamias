package tamias
import "container/vector"


// The spatial hash is Chipmunk's default (and currently only) spatial 
// index type.
// Based on a chained hash table.

interface SpaceHashElement {
  GetBounds()(BB)
} 


// Used internally to track objects added to the hash
type Handle struct {
  // Pointer to the object
  obj SpaceHashElement
  // Retain count
  retain int
  // Query stamp. Used to make sure two objects
  // aren't identified twice in the same query.
  stamp int
}

// Linked list element for in the chains.
type SpaceHashBin struct {
  handle Handle *
  next * SpaceHashBin
} 


typedef struct cpSpaceHash{
  // Number of cells in the table.
  numcells int
  // Dimensions of the cells.
  celldim Float
  
  // Hashset of the handles and the recycled ones.
  handleSet     []HashSet
  pooledHandles Array
  
  // The table and the recycled bins.
  table         []*SpaceHashBin
  pooledBins    Array
  
  allocatedBuffers Array
    
  // Incremented on each query. See cpHandle.stamp.
  stamp int
} 

func (hand *Handle) Init(obj * SpaceHashElement) (*Handle) { 
  hand.obj    = obj
  hand.retain = 0
  hand.stamp  = 0
  return hand
}

func (hand *Handle) Retain() { 
  hand.retain++
}


func (hand *Handle) Release(pooledHandles * Array) {
  hand.retain--
  if hand.retain < 1 { 
    pooledHandles.Push(hand)
  }    
}


func SpaceHashAlloc() (*SpaceHash) {
  return &SpaceHash{}
}

func (hash * SpaceHash) AllocTable(int numcells) {
  hash.numcells = numcells
  hash.table = make([]*SpaceHashBin, numcells)
} 

// Equality function for the handleset.
func (handle * Handle) Equals(other * Handle) (bool) {  
  return (hand.obj == other.obj)
}

// Transformation function for the handleset.
func (handle *Handle) handleSetTrans (hash * SpaceHash)
{
  if(hash.pooledHandles.num == 0){
    // handle pool is exhausted, make more    
    count 	:= 100    
    buffer 	:= make([]Handle, count)
    for int i=0; i<count; i++ { 
      hash.pooledHandles.Push(buffer[i])
    }  
  }
  
  cpHandle *hand = cpHandleInit(cpArrayPop(hash.pooledHandles), obj)
  cpHandleRetain(hand)
  
  return hand
}

func (hash *SpaceHash) Init(celldim Float, numcells int) (* SpaceHash) {
  hash.AllocTable(next_prime(numcells))  
  hash.celldim 		= celldim
 
  hash.handleSet     	= HashSetNew(0) 
  hash.pooledHandles 	= ArrayNew(0)
  
  hash.pooledBins    	= nil
  hash.allocatedBuffers = ArrayNew(0)
  
  hash.stamp 		= 1  
  return hash
}

func SpaceHashNew(celldim Float, numcells int) (* SpaceHash) { 
  return SpaceHashAlloc().Init(celldim, numcells)
}

func (hash * SpaceHash) recycleBin(SpaceHashBin *bin) {
  bin.next 	  = hash.pooledBins
  hash.pooledBins = bin
}

func (hash * SpaceHash) clearHashCell(int idx) {
  bin := hash.table[idx]
  for bin { 
    next := bin.next
    bin.handle.Release(hash.pooledHandles)
    hash.recycleBin(bin)
    bin   = next
  }
  hash.table[idx] = nil
}

// Clear all cells in the hashtable.
func (hash * SpaceHash) clearHash() { 
  for int i=0; i<hash.numcells; i++ {  
    hash.clearHashCell(i)
  }  
}

func (hash * SpaceHash) Destroy() { 
  hash.clearHash()
  hash.handleSet.free()
  
  hash.allocatedBuffers = nil
  hash.pooledHandles	= nil  
  hash.table		= nil
}

func (hash * SpaceHash) Free() { 
  hash.Destroy()  
}  


func (hash * SpaceHash) Resize(celdim Float, numcells int) { 
  // Clear the hash to release the old handle locks.
  hash.clearHash()  
  hash.celldim = celldim
  hash.AllocTable(next_prime(numcells))
}

// Return true if the chain contains the handle.
func (bin *SpaceHashBin) containsHandle(hand *Handle) (bool) {
  if bin != nil {
    if (bin.handle == hand) return true
    bin = bin.next
  }  
  return false
}

// Get a recycled or new bin.
func (hash * SpaceHashBin) getEmptyBin() (bin * SpaceHashBin) {
  bin := hash.pooledBins
  
  if bin != nil {
    hash.pooledBins = bin.next
    return bin
  } else {
    // Pool is exhausted, make more
    buffer := make([]SpaceHashBin, 100)
    hash.allocatedBuffers.Push(buffer)
    
    // push all but the first one, return the first instead
    for  i:=1; i<count; i++ { 
      hash.recycleBin(&buffer[i])
    }  
    return &buffer[0]
  }
}

// The hash function itself.
func hash_func(x, y, n HashValue) (HashValue) {
  return (x*1640531513 ^ y*2654435789) % n
}

// Much faster than (int)floor(f)
// Profiling showed floor() to be a sizable performance hog
// XXX: check this for golang.
func (Float f) floor_int() (int) {                                 
  i := int(f)
  if f < Float(0.0) && f != Float(i) { 
    return i - 1
  }
  return i 
}

func (hash * SpaceHash) hashHandle(hand * Handle, bb BB) 
  // Find the dimensions in cell coordinates.
  dim 	:= hash.celldim
  // Fix by ShiftZ
  l := floor_int(bb.l / dim)
  r := floor_int(bb.r / dim)
  b := floor_int(bb.b / dim)
  t := floor_int(bb.t / dim)
  
  n := hash.numcells
  for  i:=l ;  i<=r; i++ {
    for j:=b ; j<=t ; j++ {
      idx := hash_func(HashValue(i), HashValue(j), HashValue(n))
      bin := hash.table[idx]
      
      // Don't add an object twice to the same cell.
      if bin.containsHandle(hand) { continue; }
      hand.Retain()
      
      // Insert a new bin for the handle in this cell.
      newBin 		:= hash.getEmptyBin()
      newBin.handle 	= hand
      newBin.next 	= bin
      hash.table[idx] 	= newBin
    }
  }
}

func (hash * SpaceHash) Insert(hand * Handle, obj * HashElement,  bb BB) {
  hand := hash.handleSet.Insert(hashid, obj, hash)
  hash.hashHandle(hand, bb)
}

func (hash * SpaceHash) Insert(obj * HashElement,  hashid HashValue) {
  hand := hash.handleSet.find(hashid, obj)
  hash.hashHandle(hand, obj.GetBB())
}

func (hash * SpaceHash) RehashObject(obj * HashElement,  hashid HashValue) {
  hand := hash.handleSet.find(hashid, obj)
  hash.hashHandle(hand, obj.GetBB())
} 

// Hashset iterator function for rehashing the spatial hash. (hash hash hash hash?)
func handleRehashHelper(hand * Handle, hash * SpaceHash) {
  hash.hashHandle(hash, hand, hand.obj.GetBB())
}

func (cpSpaceHash *hash) Rehash() {
  clearHash(hash)  
  // Rehash all of the handles.
  hash.handleSet.Each(handleRehashHelper, hash)
}

func (hash * SpaceHash) Remove(obj * HashElement,  hashid HashValue) {
  hand := hash.handlSet.Remove(hashid, obj)  
  if hand != nil {
    hand.obj = nil
    hand.Release(hash.pooledHandles)
  }
}

// Used by the cpSpaceHashEach() iterator.
typedef struct eachPair {
  cpSpaceHashIterator func
  void *data
} eachPair

// Calls the user iterator function. (Gross I know.)
static void
eachHelper(void *elt, void *data)
{
  cpHandle *hand = (cpHandle *)elt
  eachPair *pair = (eachPair *)data
  
  pair.func(hand.obj, pair.data)
}

// Iterate over the objects in the spatial hash.
void
cpSpaceHashEach(cpSpaceHash *hash, cpSpaceHashIterator func, void *data)
{
  // Bundle the callback up to send to the hashset iterator.
  eachPair pair = {func, data}
  
  cpHashSetEach(hash.handleSet, &eachHelper, &pair)
}

// Calls the callback function for the objects in a given chain.
static inline void
query(cpSpaceHash *hash, cpSpaceHashBin *bin, void *obj, cpSpaceHashQueryFunc func, void *data)
{
  for(; bin; bin = bin.next){
    cpHandle *hand = bin.handle
    void *other = hand.obj
    
    // Skip over certain conditions
    if(
      // Have we already tried this pair in this query?
      hand.stamp == hash.stamp
      // Is obj the same as other?
      || obj == other 
      // Has other been removed since the last rehash?
      || !other
      ) continue
    
    func(obj, other, data)

    // Stamp that the handle was checked already against this object.
    hand.stamp = hash.stamp
  }
}

void
cpSpaceHashPointQuery(cpSpaceHash *hash, cpVect point, cpSpaceHashQueryFunc func, void *data)
{
  cpFloat dim = hash.celldim
  int idx = hash_func(floor_int(point.x/dim), floor_int(point.y/dim), hash.numcells);  // Fix by ShiftZ
  
  query(hash, hash.table[idx], &point, func, data)

  // Increment the stamp.
  // Only one cell is checked, but query() requires it anyway.
  hash.stamp++
}

void
cpSpaceHashQuery(cpSpaceHash *hash, void *obj, cpBB bb, cpSpaceHashQueryFunc func, void *data)
{
  // Get the dimensions in cell coordinates.
  cpFloat dim = hash.celldim
  int l = floor_int(bb.l/dim);  // Fix by ShiftZ
  int r = floor_int(bb.r/dim)
  int b = floor_int(bb.b/dim)
  int t = floor_int(bb.t/dim)
  
  int n = hash.numcells
  
  // Iterate over the cells and query them.
  for(int i=l; i<=r; i++){
    for(int j=b; j<=t; j++){
      int idx = hash_func(i,j,n)
      query(hash, hash.table[idx], obj, func, data)
    }
  }
  
  // Increment the stamp.
  hash.stamp++
}

// Similar to struct eachPair above.
typedef struct queryRehashPair {
  cpSpaceHash *hash
  cpSpaceHashQueryFunc func
  void *data
} queryRehashPair

// Hashset iterator func used with cpSpaceHashQueryRehash().
static void
handleQueryRehashHelper(void *elt, void *data)
{
  cpHandle *hand = (cpHandle *)elt
  
  // Unpack the user callback data.
  queryRehashPair *pair = (queryRehashPair *)data
  cpSpaceHash *hash = pair.hash
  cpSpaceHashQueryFunc func = pair.func

  cpFloat dim = hash.celldim
  int n = hash.numcells

  void *obj = hand.obj
  cpBB bb = hash.bbfunc(obj)

  int l = floor_int(bb.l/dim)
  int r = floor_int(bb.r/dim)
  int b = floor_int(bb.b/dim)
  int t = floor_int(bb.t/dim)

  for(int i=l; i<=r; i++){
    for(int j=b; j<=t; j++){
//      // exit the loops if the object has been deleted in func().
//      if(!hand.obj) goto break_out
      
      int idx = hash_func(i,j,n)
      cpSpaceHashBin *bin = hash.table[idx]
      
      if(containsHandle(bin, hand)) continue
      
      cpHandleRetain(hand); // this MUST be done first in case the object is removed in func()
      query(hash, bin, obj, func, pair.data)
      
      cpSpaceHashBin *newBin = getEmptyBin(hash)
      newBin.handle = hand
      newBin.next = bin
      hash.table[idx] = newBin
    }
  }
  
//  break_out:
  // Increment the stamp for each object we hash.
  hash.stamp++
}

void
cpSpaceHashQueryRehash(cpSpaceHash *hash, cpSpaceHashQueryFunc func, void *data)
{
  clearHash(hash)
  
  queryRehashPair pair = {hash, func, data}
  cpHashSetEach(hash.handleSet, &handleQueryRehashHelper, &pair)
}

static inline cpFloat
segmentQuery(cpSpaceHash *hash, cpSpaceHashBin *bin, void *obj, cpSpaceHashSegmentQueryFunc func, void *data)
{
  cpFloat t = 1.0f
   
  for(; bin; bin = bin.next){
    cpHandle *hand = bin.handle
    void *other = hand.obj
    
    // Skip over certain conditions
    if(
      // Have we already tried this pair in this query?
      hand.stamp == hash.stamp
      // Has other been removed since the last rehash?
      || !other
      ) continue
    
    // Stamp that the handle was checked already against this object.
    hand.stamp = hash.stamp
    
    t = cpfmin(t, func(obj, other, data))
  }
  
  return t
}

// modified from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
void cpSpaceHashSegmentQuery(cpSpaceHash *hash, void *obj, cpVect a, cpVect b, cpFloat t_exit, cpSpaceHashSegmentQueryFunc func, void *data)
{
  a = cpvmult(a, 1.0f/hash.celldim)
  b = cpvmult(b, 1.0f/hash.celldim)
  
  cpFloat dt_dx = 1.0f/cpfabs(b.x - a.x), dt_dy = 1.0f/cpfabs(b.y - a.y)
  
  int cell_x = floor_int(a.x), cell_y = floor_int(a.y)

  cpFloat t = 0

  int x_inc, y_inc
  cpFloat temp_v, temp_h

  if (b.x > a.x){
    x_inc = 1
    temp_h = (cpffloor(a.x + 1.0f) - a.x)
  } else {
    x_inc = -1
    temp_h = (a.x - cpffloor(a.x))
  }

  if (b.y > a.y){
    y_inc = 1
    temp_v = (cpffloor(a.y + 1.0f) - a.y)
  } else {
    y_inc = -1
    temp_v = (a.y - cpffloor(a.y))
  }
  
  // fix NANs in horizontal directions
  cpFloat next_h = (temp_h ? temp_h*dt_dx : dt_dx)
  cpFloat next_v = (temp_v ? temp_v*dt_dy : dt_dy)

  int n = hash.numcells
  while(t < t_exit){
    int idx = hash_func(cell_x, cell_y, n)
    t_exit = cpfmin(t_exit, segmentQuery(hash, hash.table[idx], obj, func, data))

    if (next_v < next_h){
      cell_y += y_inc
      t = next_v
      next_v += dt_dy
    } else {
      cell_x += x_inc
      t = next_h
      next_h += dt_dx
    }
  }
  
  hash.stamp++
}
