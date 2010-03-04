package tamias
import "container/list"
import "fmt"
import "exp/iterable"

// Spacemap is an alternative implementation of the spacehash

type SpaceMapElement interface{
  GetBB() (*BB)  
}

type SpaceMapKey uint64

type SpaceMapCell struct {
  Shapes * list.List
}

type rawSpaceMap map[SpaceMapKey] SpaceMapCell



type SpaceMap struct {  
  table map[SpaceMapKey] SpaceMapCell;
  // Number of cells in the table.
  numcells int
  // Dimensions of the cells.
  cellsize Float
  // Incremented on every query
  stamp int
}

func (cell * SpaceMapCell) Init() (* SpaceMapCell) {
  cell.Shapes = list.New()
  return cell
}

// Find finds a space map element in a cell, or return nil if not found
func (cell * SpaceMapCell) Find(el SpaceMapElement) (SpaceMapElement){
      finder    := func (val interface {}) (bool) { 
	       testel := val.(SpaceMapElement)
	       return el == testel
      }  
      found := iterable.Find((cell.Shapes), finder)
      if found == nil { return nil }      
      return found.(SpaceMapElement)
}

// Removes as space map element from a space map cell and removes it,
// or return nil if not found
func (cell * SpaceMapCell) Remove(el SpaceMapElement) (SpaceMapElement) {
      var e *list.Element
      for e = cell.Shapes.Front() ; e != nil ; e = e.Next() {
        val := e.Value
        if val.(SpaceMapElement) == el {
          cell.Shapes.Remove(e)
          return el
        }
      }
      return nil
}



// Insert inserts a space map element into the cell
func (cell * SpaceMapCell) Insert(el SpaceMapElement) (SpaceMapElement) { 
      cell.Shapes.PushBack(el)
      return el
}

func (cell * SpaceMapCell) String() (string) {
  return fmt.Sprint("SpaceMap: ", iterable.Data(cell.Shapes))
}

// SpaceMapAllocate allocates a new spacemap and returns a pointer to it 
func SpaceMapAllocate() (* SpaceMap) {
  return &SpaceMap{}
}

// Init initializes the SpaceMap with the given cell size and amount of cells
func (sm * SpaceMap) Init(cellsize Float, numcells int) (*SpaceMap) {
  sm.numcells = numcells
  sm.cellsize = cellsize
  sm.table    = make(rawSpaceMap,  sm.numcells)
  for i := 0; i < sm.numcells ; i++ {
    cell, ok := sm.table[SpaceMapKey(i)]
    if !ok {
      cell          = SpaceMapCell{}
      cell.Init()  
      sm.table[SpaceMapKey(i)] = cell        
    }
    cell.Init()
  }  
  return sm
}

func SpaceMapNew(cellsize Float, numcells int) (*SpaceMap) {
  return new(SpaceMap).Init(cellsize, numcells)
}


// The hash function itself.
func space_hash(x, y, n uint64) (SpaceMapKey) {
  return SpaceMapKey((x*1640531513 ^ y*2654435789) % n)
}

// cellDimensions finds the dimensions of a bounds box in cell coordinates.
// Returns them in order left, top, right, bottom
func (sm * SpaceMap) cellDimensions(bb * BB) (int, int, int, int) {  
  dim 	:= sm.cellsize
  // Fix by ShiftZ  
  l := (bb.L / dim).floor_int()  
  t := (bb.T / dim).floor_int()
  r := (bb.R / dim).floor_int()
  b := (bb.B / dim).floor_int()  
  return l, t, r, b
}

// Insert inserts an element into the SpaceMap
func (sm * SpaceMap) Insert(el SpaceMapElement) (SpaceMapElement) {
  bb := el.GetBB()
  l, t, r, b := sm.cellDimensions(bb)
  fmt.Println("Insert!", l, "->", r, " & ", t,"->", b)
  // the bounds box may be in several cells, so iterate over them
  for i := l ; i <= r ; i++ {
    for j := b ; j <= t ; j++ {
      cell      := sm.FindPoint(i, j)
      old       := cell.Find(el)
      fmt.Println("Skip!", i, j)
      if old != nil { continue }
      // Already in this spatial cell. Move on to the next one 
      fmt.Println("Inserting element: ", i, j)
      cell.Insert(el)
      // Add it to this cell.      
    }
  }
  return el
}

// Looks up the SpaceMapCell that the point with coordinates X and Y is in.
// returns nil if not found 
func (sm * SpaceMap) FindPoint(x, y int) (*SpaceMapCell) {
  xx := uint64(x)
  yy := uint64(y)
  hk := space_hash(xx, yy, uint64(sm.numcells))
  fmt.Println("hk: ", hk)
  cell := sm.table[hk]
  return &cell
}

// Looks up the SpaceMapCell that the vector is in
func (sm * SpaceMap) FindVect(p Vect) (*SpaceMapCell) {
  return sm.FindPoint( int(p.X.Floor()), int(p.Y.Floor()) )
}

// Returns a vector with all SpaceMapElements in the given bounds box.
// Or returns nil if nothing was found
func (sm * SpaceMap) FindBB(bb *BB) (* list.List) {
  result := list.New()
  num    := 0
  l, t, r, b := sm.cellDimensions(bb)
  // the bounds box may be in several cells, so iterate over them
  for i := l ; i <= r ; i++ {
    for j := t ; j <= b ; j++ {
      cell      := sm.FindPoint(i, j)
      for found := range cell.Shapes.Iter() { 
	       result.PushBack(found)
         num++ 
      }
    }
  }
  if num < 1 { return nil }
  return result
}

// RehashObject moves the object to the right hash bucket. 
// Call this after it has moved. 
func (sm * SpaceMap) RehashObject(el SpaceMapElement) {


}



func (sm * SpaceMap) String() (string) {
  return fmt.Sprint("SpaceMap: ", sm.table )
}









