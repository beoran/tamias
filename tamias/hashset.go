package tamias

// HashSet uses a chained hashtable implementation.
// Other than the transformation functions, there is nothing fancy going on.

type HashElement interface {
  (*HashElement) Equals(*HashElement) (bool)
  (*HashElement) Iterate() (*HashElement)
  (*HashElement) Filter() (bool)
}

// cpHashSetBin's form the linked lists in the chained hash table.
type HashSetBin cpHashSetBin {
  // Pointer to the element.
  elt * HashElement
  // Hash value of the element.
  hash * HashValue
  // Next element in the chain.
  next * HashSetBin
}

/*
// Equality function. Returns true if ptr is equal to elt.
typedef int (*cpHashSetEqlFunc)(void *ptr, void *elt);
// Used by cpHashSetInsert(). Called to transform the ptr into an element.
typedef void *(*cpHashSetTransFunc)(void *ptr, void *data);
// Iterator function for a hashset.
typedef void (*cpHashSetIterFunc)(void *elt, void *data);
// Filter function. Returns false if elt should be dropped.
typedef int (*cpHashSetFilterFunc)(void *elt, void *data);
*/

type HashSet struct {
  // Number of elements stored in the table.
  int entries;
  // Number of cells in the table.
  int size;
  
  /*
  cpHashSetEqlFunc eql;
  cpHashSetTransFunc trans;
  (*HashElement) Iterate(*HashElement) (bool)
  */
  
  // Default value returned by cpHashSetFind() when no element is found.
  // Defaults to NULL.
  default_value * HashElement;
  
  // The table and recycled bins
  table       []*HashSetBin
  pooledbins    *HashSetBin
  // Will use Go's array for this in stead.
  // cpArray *allocatedBuffers; 
}



// Used for resizing hash tables.
// Values approximately double.
// http://planetmath.org/encyclopedia/GoodHashTablePrimes.html
var primes int[] = [
  5,
  13,
  23,
  47,
  97,
  193,
  389,
  769,
  1543,
  3079,
  6151,
  12289,
  24593,
  49157,
  98317,
  196613,
  393241,
  786433,
  1572869,
  3145739,
  6291469,
  12582917,
  25165843,
  50331653,
  100663319,
  201326611,
  402653189,
  805306457,
  1610612741,
  0,
];

next_prime(int n) (int)
{
  int i = 0;
  while(n > primes[i]) {
    i++
    Assert(primes[i] > 0, 
    "Tried to resize a hash table to a size greater than 1610612741 O_o"); 
    // realistically this should never happen
  }  
  return primes[i]
}


func (set * HashSet) Destroy() {
  set.table       = nil
  set.pooledbins  = nil
}

func (set * HashSet) Free() {

}

func HashSetAlloc() (* HashSet) {
  return &HashSet{}
} 

func (set * HashSet) Init(size int) (* HashSet) {
  set.size          = next_prime(size)
  set.entries       = 0
  set.default_value = nil
  set.table         = make([]*HashSetBin, 0, set.size)
  set.pooledbins    = nil
  // set.buffers       = nil
}


func HashSetNew(size int) (* HashSet) {
  return HashSetAlloc.Init(size)
} 

func (set * HashSet) IsFull() bool { 
  return (set.entries >= set.size);
}  

func (set * HashSet) Resize() {
  // Get the next approximate doubled prime.
  newsize := next_prim(set.size + 1)
  // Allocate a new table.
  newtable := make([]*HashSetBin, set.size, set.newsize)
  for 
  // Iterate over the chains.
  for  i:=0 ; i < set.size ; i++ {
    // Rehash the bins into the new table.
    bin := set->table[i];
    while (bin != nil) {
      next         := bin.next      
      idx          := bin.hash % newsize;
      bin.next      = newtable[idx]
      newtable[idx] = bin; 
      bin           = next;
    }
  }
  set.table = newtable
  set.size  = newsize    
}  


func (set * HashSet) recycleBin(bin * HashSetBin) {
  bin.next        = set.pooledbins
  set.pooledbins  = bin  
  bin.elt         = nil
}  
  
func (set * HashSet) getUnusedBin() (* HashSetBin) {    
  bin := set.pooledbins
  if bin != nil {
    set.pooledbins = bin.next
    return bin
  } else {
    // Pool is exhausted, make 100 more
    count   := 100
    newbins := make([]HashSetBin, count)
    // push all but the first one, return the first instead
    for i:=1; i<count; i++ { 
      set.recycleBin(newbins[i])
    }    
    return newbins[0]
  }
}

// Find the correct has bin for this element, or nil if not found
func (set *HashSet) findBin(hash HashValue, ptr * Hashelement) (*HashSetBin) {
  idx := hash % set->size
  bin := set->table[idx]
  // Follow the chained elements in the bin until the element is equal
  for bin != nil && !ptr.Equals(bin->elt) { 
    bin = bin->next
  }
  return bin;
}

// Find the correct has bin for this element, or nil if not found
// Also returns the bin before the current bin
func (set *HashSet) findBinPrev(hash HashValue, ptr * Hashelement) (
bin *HashSetBin, prev *HashSetBin) {
  idx := hash % set->size
  prev = nil
  bin  = set->table[idx]
  // Follow the chained elements in the bin until the element is equal
  for bin != nil && !ptr.Equals(bin->elt) {
    prev  = bin
    bin   = bin->next
  }
  return bin, prev;
}

  

func (set * HashSet) Insert(hash HashValue, ptr * HashElement) (* HashElement) {
  idx := hash % set.size
  
  // Find the bin with the matching element.
  bin := set.findBin(hash, ptr)
  
  // Create it if necessary.
  if bin == nil {
    bin = getUnusedBin(set)
    bin->hash = hash
    bin->elt  = ptr //XXX Transformation not called here. Is it needed?
    
    bin->next = set->table[idx]
    set->table[idx] = bin
    
    set->entries++
    
    // Resize the set if it's full.
    if set.IsFull() { 
      set.Resize()
    }  
  } else { 
  // this is not in Chipmunk original but seems like a bug not to do it
    bin.elt = ptr
  }  
  return bin.elt
}  

func (set * HashSet) Remove(hash HashValue, ptr * HashElement) (* HashElement) {
  bin, prev := set.findBinPrev(hash, ptr)
  // Remove it if it exists.
  if(bin) {
    // Update the previous bin next pointer to point to the next bin.
    if prev { 
      prev.next = bin.next
    }  
    set.entries--    
    return_value := bin.elt    
    recycleBin(set, bin)    
    return return_value
  }  
  return nil;
}

func (set * Hash) Find(hash HashValue, ptr * HashElement) (* HashElement) {
  bin := set.findBin()
  if bin {
    return bin.elt 
  }
  return set.default_value
}

 

/* 
XXX: how to iterate and filter? 
void
cpHashSetEach(cpHashSet *set, cpHashSetIterFunc func, void *data)
{
  for(int i=0; i<set->size; i++){
    cpHashSetBin *bin = set->table[i];
    while(bin){
      cpHashSetBin *next = bin->next;
      func(bin->elt, data);
      bin = next;
    }
  }
}


void
cpHashSetFilter(cpHashSet *set, cpHashSetFilterFunc func, void *data)
{
  // Iterate over all the chains.
  for(int i=0; i<set->size; i++){
    // The rest works similarly to cpHashSetRemove() above.
    cpHashSetBin **prev_ptr = &set->table[i];
    cpHashSetBin *bin = set->table[i];
    while(bin){
      cpHashSetBin *next = bin->next;
      
      if(func(bin->elt, data)){
        prev_ptr = &bin->next;
      } else {
        (*prev_ptr) = next;

        set->entries--;
        recycleBin(set, bin);
      }
      
      bin = next;
    }
  }
}
*/
