package tamias

// HashMap is a wrapped hash map, that uses the go map
// it's an alternative for HashSet
/*
type HashValue int64

type HashElement interface {
  Equals(interface {}) (bool)
}
*/

type HashMap struct { 
  table map[HashValue] HashElement
}

func HashMapAlloc() (* HashMap) {
  return &HashMap{}
} 

func (hash * HashMap) Init(size int) (* HashMap) {
  hash.table         = make(map[HashValue] HashElement, size)
  return hash
  // set.buffers       = nil
}

func HashMapNew(size int) (HashMap) {
  return HashSetAlloc().Init(size)
} 

 

func (hash * HashSet) Insert(key HashValue, value HashElement) (HashElement) {
  hash.table[key] = value  
   
  idx := set.hashIndex(hash)
  
  // Find the bin with the matching element.
  bin := set.findBin(hash, ptr)
  
  // Create it if necessary.
  if bin == nil {
    bin = set.getUnusedBin()
    bin.hash = hash
    bin.elt  = ptr //XXX Transformation not called here. Is it needed?
    
    bin.next = set.table[idx]
    set.table[idx] = bin
    
    set.entries++
    
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

func (set * HashSet) Remove(hash HashValue, ptr HashElement) (HashElement) {
  bin, prev := set.findBinPrev(hash, ptr)
  // Remove it if it exists.
  if bin != nil {
    // Update the previous bin next pointer to point to the next bin.
    if prev != nil { 
      prev.next = bin.next
    }  
    set.entries--    
    return_value := bin.elt
    set.recycleBin(bin)
    return return_value
  }  
  return nil;
}

func (set * HashSet) Find(hash HashValue, ptr HashElement) (HashElement) {
  bin := set.findBin(hash, ptr)
  if bin != nil {
    return bin.elt 
  }
  return set.default_value
}

type HashSetIterFunc func(bin, data HashElement) 

func (set * HashSet) Each(fun HashSetIterFunc, data HashElement) { 
  for i:=0 ; i<set.size ; i++ {
    bin := set.table[i];
    for bin != nil {
      next := bin.next;
      fun(bin.elt, data);
      bin = next;
    }
  }
}

type HashSetFilterFunc func(bin, data HashElement) (bool)