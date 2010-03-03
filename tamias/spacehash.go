package tamias
// import "container/vector"


// The spatial hash is Chipmunk's default (and currently only) spatial 
// index type.
// Based on a chained hash table.

type SpaceHashElement interface  {
  HashElement
  GetBB()(BB)
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
  handle * Handle
  next * SpaceHashBin
} 


type SpaceHash struct {
  // Number of cells in the table.
  numcells int
  // Dimensions of the cells.
  celldim Float
  
  // Hashset of the handles and the recycled ones.
  handleSet     *HashSet
  pooledHandles *Array
  
  // The table and the recycled bins.
  table         []*SpaceHashBin
  pooledBins    *SpaceHashBin
  
  allocatedBuffers *Array
    
  // Incremented on each query. See cpHandle.stamp.
  stamp int
} 

func (hand *Handle) Init(obj SpaceHashElement) (*Handle) { 
  hand.obj    = obj
  hand.retain = 0
  hand.stamp  = 0
  return hand
}

// Equality function for the handleset.
func (hand *Handle) Equals(el interface {}) (bool) {
  other, ok := el.(*Handle)
  if !ok { return false; }
  return hand == other
}

// Equality function for the SpaceHash
func (hash *SpaceHash) Equals(el interface {}) (bool) {  
  other, ok := el.(*SpaceHash)  
  if !ok { return false; }
  return hash == other
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

func (hash * SpaceHash) AllocTable(numcells int) {
  hash.numcells = numcells
  hash.table = make([]*SpaceHashBin, numcells)
} 


// Transformation function for the handleset.
/*
func (handle *Handle) handleSetTrans (hash * SpaceHash) (*Handle) {
  if(hash.pooledHandles.num == 0){
    // handle pool is exhausted, make more    
    count 	:= 100    
    buffer 	:= make([]Handle, count)
    for i:=0; i<count; i++ { 
      hash.pooledHandles.Push(buffer[i])
    }  
  }
  
  hand := handle.Init(hash.pooledHandles.Pop(), obj)
  hand.Retain()
  
  return hand
}
*/

func (hash *SpaceHash) Init(celldim Float, numcells int) (* SpaceHash) {
  hash.AllocTable(next_prime(numcells))  
  
  hash.celldim          = celldim   
  hash.handleSet        = HashSetNew(0) 
  hash.pooledHandles 	  = ArrayNew(0)    
  hash.pooledBins    	  = nil
  hash.allocatedBuffers = ArrayNew(0)  
  hash.stamp 		        = 1  
  
  return hash
}

func SpaceHashNew(celldim Float, numcells int) (* SpaceHash) { 
  return SpaceHashAlloc().Init(celldim, numcells)
}

func (hash * SpaceHash) recycleBin(bin *SpaceHashBin) {
  bin.next 	      = hash.pooledBins
  hash.pooledBins = bin
}

func (hash * SpaceHash) clearHashCell(idx int) {
  bin := hash.table[idx]
  for bin != nil { 
    next := bin.next
    bin.handle.Release(hash.pooledHandles)
    hash.recycleBin(bin)
    bin   = next
  }
  hash.table[idx] = nil
}

// Clear all cells in the hashtable.
func (hash * SpaceHash) clearHash() { 
  for i:=0; i<hash.numcells; i++ {  
    hash.clearHashCell(i)
  }  
}

func (hash * SpaceHash) Destroy() { 
  hash.clearHash()
  hash.handleSet.Free()
  
  hash.allocatedBuffers = nil
  hash.pooledHandles	= nil  
  hash.table		= nil
}

func (hash * SpaceHash) Free() { 
  hash.Destroy()  
}  


func (hash * SpaceHash) Resize(celldim Float, numcells int) { 
  // Clear the hash to release the old handle locks.
  hash.clearHash()  
  hash.celldim = celldim
  hash.AllocTable(next_prime(numcells))
}

// Return true if the chain contains the handle.
func (bin *SpaceHashBin) containsHandle(hand *Handle) (bool) {
  if bin != nil {
    if (bin.handle == hand) { return true }
    bin = bin.next
  }  
  return false
}

// Get a recycled or new bin.
func (hash * SpaceHash) getEmptyBin() (* SpaceHashBin) {
  bin := hash.pooledBins
  
  if bin != nil {
    hash.pooledBins = bin.next
    return bin
  } 
  // Pool is exhausted, make more
  count  := 100
  buffer := make([]SpaceHashBin, count)
  hash.allocatedBuffers.Push(buffer)
  
  // push all but the first one, return the first instead
  for  i:=1; i<count; i++ { 
    hash.recycleBin(&buffer[i])
  }  
  return &buffer[0] 
}

// The hash function itself.
func hash_func(x, y, n HashValue) (HashValue) {
  return (x*1640531513 ^ y*2654435789) % n
}

// Much faster than (int)floor(f)
// Profiling showed floor() to be a sizable performance hog (in C)
// XXX: check this for golang.
func (f Float) floor_int() (int) {
  i := int(f)
  if f < Float(0.0) && f != Float(i) { 
    return i - 1
  }
  return i 
}

func (hash * SpaceHash) cellDimensions(bb BB) (int, int, int, int) {
  // Find the dimensions in cell coordinates.
  dim 	:= hash.celldim
  // Fix by ShiftZ
  l := (bb.L / dim).floor_int()
  r := (bb.R / dim).floor_int()
  b := (bb.B / dim).floor_int()
  t := (bb.T / dim).floor_int()
  return l, r, b, t
}


func (hash * SpaceHash) hashHandle(hand * Handle, bb BB) { 
  // Find the dimensions in cell coordinates.
  l, r, b, t := hash.cellDimensions(bb)
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

func (hash * SpaceHash) InsertHandle(hand * Handle, obj HashElement, hashid HashValue, bb BB) {
  hand = hash.handleSet.Insert(hashid, obj).(*Handle)
  hash.hashHandle(hand, bb)
}

func (hash * SpaceHash) Insert(obj SpaceHashElement,  hashid HashValue) {
  hand := hash.handleSet.Find(hashid, obj).(*Handle)
  hash.hashHandle(hand, obj.GetBB())
}

func (hash * SpaceHash) RehashObject(obj SpaceHashElement,  hashid HashValue) {
  hand := hash.handleSet.Find(hashid, obj).(*Handle)
  hash.hashHandle(hand, obj.GetBB())
} 

// Hashset iterator function for rehashing the spatial hash. (hash hash hash hash?)
func handleRehashHelper(bin, data HashElement) {  
  var hand * Handle     = bin.(*Handle) 
  var hash * SpaceHash  = data.(*SpaceHash)   
  hash.hashHandle(hand, hand.obj.GetBB())
}

func (hash * SpaceHash) Rehash() {
  hash.clearHash()  
  // Rehash all of the handles.
  hash.handleSet.Each(handleRehashHelper, hash)
}

func (hash * SpaceHash) Remove(obj HashElement,  hashid HashValue) {
  hand := hash.handleSet.Remove(hashid, obj).(*Handle)  
  if hand != nil {
    hand.obj = nil
    hand.Release(hash.pooledHandles)
  }
}



type SpaceHashIterator func(a, b HashElement)
type SpaceHashQueryFunc func(a, b, c HashElement) (bool) 

// Used by the cpSpaceHashEach() iterator.
type eachPair struct {
  fun 		SpaceHashIterator
  data   	HashElement
}

// Equals function for eachPair
func (pair *eachPair) Equals(el interface {}) (bool) {  
  other, ok := el.(*eachPair)  
  if !ok { return false; }
  return pair == other
}


// Calls the user iterator function. (Gross I know.)
func eachHelper(bin , data HashElement) {
  var hand * Handle     = bin.(*Handle)
  var pair * eachPair   = data.(*eachPair)
  pair.fun(hand.obj, pair.data)
}

// Iterate over the objects in the spatial hash.
func (hash *SpaceHash) Each(fun SpaceHashIterator, data HashElement) {
  // Bundle the callback up to send to the hashset iterator.
  pair := &eachPair{fun, data}  
  hash.handleSet.Each(eachHelper, pair)
}
// Calls the callback function for the objects in a given chain.
func (hash *SpaceHash) query(bin * SpaceHashBin, obj HashElement, 				
      fun SpaceHashQueryFunc, data HashElement) {
  for ; bin != nil ; bin = bin.next {
    hand 	  := bin.handle
    other 	:= hand.obj    
    // Skip over certain conditions
    if hand.stamp == hash.stamp || obj.Equals(other) || other == nil { 
      continue 
    } 
    // Have we already tried this pair in this query?     
    // Is obj the same as other?
    // Has other been removed since the last rehash?    
    fun(obj, other, data)
    // Stamp that the handle was checked already against this object.
    hand.stamp = hash.stamp
  }
}

func (hash * SpaceHash) PointQuery(point Vect, fun SpaceHashQueryFunc, 
      data HashElement) {      
  dim := hash.celldim
  xf  := (point.X/dim).floor_int()
  yf  := (point.Y/dim).floor_int()
  hc  := hash.numcells
  idx := hash_func(HashValue(xf),  HashValue(yf), HashValue(hc))  
  // Fix by ShiftZ  
  hash.query(hash.table[idx], &point, fun, data)
  // Increment the stamp.
  // Only one cell is checked, but query() requires it anyway.
  hash.stamp++
}

func (hash * SpaceHash) SpaceQuery(obj HashElement, bb BB, 
      fun SpaceHashQueryFunc, data HashElement) {
  // Get the dimensions in cell coordinates.
  l, r, b, t := hash.cellDimensions(bb)   
  n := hash.numcells
  
  // Iterate over the cells and query them.
  for i:=l ; i<=r; i++ {
    for j := b; j<=t; j++ {
      idx := hash_func(HashValue(i), HashValue(j), HashValue(n))
      hash.query(hash.table[idx], obj, fun, data)
    }
  }  
  // Increment the stamp.
  hash.stamp++
}

// Similar to struct eachPair above.
type queryRehashPair struct {
  hash * SpaceHash
  fun SpaceHashQueryFunc
  data HashElement
} 

func (pair *queryRehashPair) Equals(el interface {}) (bool) {  
  other, ok := el.(*queryRehashPair)  
  if !ok { return false; }
  return pair == other
}

// Hashset iterator func used with cpSpaceHashQueryRehash().
func handleQueryRehashHelper(p1, p2 HashElement) {  
  var hand * Handle           = p1.(*Handle)
  var pair * queryRehashPair  = p2.(*queryRehashPair)  
  // Unpack the user callback data.  
  hash 	:= pair.hash
  fun  	:= pair.fun

  // dim 	:= hash.celldim
  n  	:= hash.numcells

  obj 	:= hand.obj
  bb 	  := obj.GetBB()
  var l, r, b, t int
  l, r, b , t = hash.cellDimensions(bb)

  for i := l; i<=r; i++ {
    for j := b; j<=t; j++ {
//  // exit the loops if the object has been deleted in func().
//      if(!hand.obj) goto break_out
      
      idx := hash_func(HashValue(i), HashValue(j), HashValue(n))
      bin := hash.table[idx]
      
      if bin.containsHandle(hand) { continue  }      
      hand.Retain() 
      // this MUST be done first in case the object is removed in func()
      hash.query(bin, obj, fun, pair.data)
      
      newBin 	       := hash.getEmptyBin()
      newBin.handle 	= hand
      newBin.next 	= bin
      hash.table[idx] 	= newBin
    }
  }  
  //  break_out:
  // Increment the stamp for each object we hash.
  hash.stamp++
}

func (hash * SpaceHash) hashRehash(fun SpaceHashQueryFunc, data HashElement) {
  hash.clearHash()  
  pair := &queryRehashPair{hash, fun, data}
  hash.handleSet.Each(handleQueryRehashHelper, pair)
}

type SpaceHashSegmentQueryFunc func (obj, other, data HashElement) (Float)


func (hash * SpaceHash) segmentQuery(bin *SpaceHashBin , obj HashElement, 
  fun SpaceHashSegmentQueryFunc, data HashElement) (Float) {
  
  t := Float(1.0)
   
  for  ; bin != nil ; bin = bin.next {
    hand := bin.handle
    other := hand.obj
    
    // Skip over certain conditions
    if hand.stamp == hash.stamp || other == nil { continue; }
      // Have we already tried this pair in this query?
      // Has other been removed since the last rehash?    
    // Stamp that the handle was checked already against this object.
    hand.stamp = hash.stamp
    t = t.Min(fun(obj, other, data))    
  }
  return t
}

// modified from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
func (hash * SpaceHash) SegmentQuery(obj HashElement, a, b Vect, t_exit Float, 			fun SpaceHashSegmentQueryFunc, data HashElement) {
  a = a.Mult(Float(1.0)/hash.celldim)
  b = b.Mult(Float(1.0)/hash.celldim)
  
  dt_dx  := Float(1.0)/((b.X - a.X).Abs())
  dt_dy  := Float(1.0)/((b.Y - a.Y).Abs())
  cell_x := (a.X).floor_int() 
  cell_y := (a.Y).floor_int()
  t 	   := Float(0.0)
  
  var x_inc	, y_inc int
  var temp_v, temp_h Float

  if b.X > a.X {
    x_inc  = 1
    temp_h = (a.X + Float(1.0)).Floor() - a.X
  } else {
    x_inc  = -1
    temp_h = a.X - a.X.Floor()
  }

  if b.Y > a.Y {
    y_inc  = 1
    temp_v = (a.Y + Float(1.0)).Floor() - a.Y
  } else {
    y_inc = -1
    temp_v = a.Y - a.Y.Floor()
  }
  
  // fix NANs in horizontal directions
  next_h := dt_dx
  if temp_h != 0.0 { 
    next_h = temp_h*dt_dx
  }
  
  next_v := dt_dy
  if temp_v != 0.0 { 
    next_v = temp_v*dt_dy
  }

  n := hash.numcells
  for t < t_exit {
    idx := hash_func(HashValue(cell_x), HashValue(cell_y), HashValue(n))
    t_exit = t_exit.Min(hash.segmentQuery(hash.table[idx], obj, fun, data))
    
    if (next_v < next_h){
      cell_y += y_inc
      t       = next_v
      next_v += dt_dy
    } else {
      cell_x += x_inc
      t       = next_h
      next_h += dt_dx
    }
  }  
  hash.stamp++
}
