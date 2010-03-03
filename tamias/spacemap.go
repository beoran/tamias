package tamias
import "container/vector"
import "exp/iterable"

// Spacemap is an alternative implementation of the spacehash

type SpaceMapElement interface{
  GetBB() (*BB)  
}

type SpaceMapKey int64

type SpaceMapCell struct {
  Shapes vector.Vector
}
type rawSpaceMap map[SpaceMapKey] SpaceMapCell;

type SpaceMap struct {  
  table map[SpaceMapKey] SpaceMapCell;
  // Number of cells in the table.
  numcells int
  // Dimensions of the cells.
  cellsize Float
  // Incremented on every query
  stamp int
}

// find a space map element in a cell, or return nil if not found
func (cell * SpaceMapCell) Find(el SpaceMapElement) (SpaceMapElement){
      finder    := func (val interface {}) (bool) {
	testel := val.(SpaceMapElement)
	return el == testel
      }  
      found := iterable.Find(&(cell.Shapes), finder)
      return found.(SpaceMapElement)
}

// Inserts a space map element into the cell
func (cell * SpaceMapCell) Insert(el SpaceMapElement) (SpaceMapElement) { 
      cell.Shapes.Push(el)
      return el
}

func SpaceMapAllocate() (* SpaceMap) {
  return &SpaceMap{}
}

func (sm * SpaceMap) Init(cellsize Float, numcells int) (*SpaceMap) {
  sm.numcells = numcells
  sm.cellsize = cellsize
  sm.table    = make(rawSpaceMap,  sm.numcells)
  return sm
}

// The hash function itself.
func space_hash(x, y, n int64) (SpaceMapKey) {
  return SpaceMapKey((x*1640531513 ^ y*2654435789) % n)
}

func (sm * SpaceMap) cellDimensions(bb * BB) (int, int, int, int) {
  // Find the dimensions in cell coordinates.
  dim 	:= sm.cellsize
  // Fix by ShiftZ  
  l := (bb.L / dim).floor_int()  
  t := (bb.T / dim).floor_int()
  r := (bb.R / dim).floor_int()
  b := (bb.B / dim).floor_int()  
  return l, t, r, b
}


func (sm * SpaceMap) Insert(el SpaceMapElement) (SpaceMapElement) {
  bb := el.GetBB()
  l, t, r, b := sm.cellDimensions(bb)
  // the bounds box may be in several cells, so iterate over them
  for i := l ; i <= r ; i++ {
    for j := t ; j <= b ; j++ {
      cell      := sm.FindPoint(i, j)
      old       := cell.Find(el)
      if old != nil { continue }
      // Already in this spatial cell. Move on to the next one 
      cell.Insert(el)
      // Add it to this cell.      
    }
  }
  return el
}

// Looks up the SpaceMapCell that the point with coordinates X and Y is in.
func (sm * SpaceMap) FindPoint(x, y int) (*SpaceMapCell) {
  xx := int64(x)
  yy := int64(y)
  hk := space_hash(xx, yy, int64(sm.numcells))
  cell := sm.table[hk]
  return &cell
}

func (sm * SpaceMap) FindVect(p Vect) (*SpaceMapCell) {
  return sm.FindPoint( int(p.X.Floor()), int(p.Y.Floor()) )
}

// Returns a vector wil all SpaveMapElements in the given bounds box.
func (sm * SpaceMap) FindBB(bb *BB) (* vector.Vector) {
  result := &vector.Vector{}
  l, t, r, b := sm.cellDimensions(bb)
  // the bounds box may be in several cells, so iterate over them
  for i := l ; i <= r ; i++ {
    for j := t ; j <= b ; j++ {
      cell      := sm.FindPoint(i, j)
      for found := range cell.Shapes.Iter() { 
	result.Push(found)
      }
    }
  }
  return result
}



