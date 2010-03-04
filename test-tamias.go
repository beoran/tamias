package main

import "fmt"
import "os"
import "tamias"
import "exp/iterable"
/*
import "exp/draw"
import "exp/draw/x11"
*/

type testbool bool

type  testresults struct {
  ok, failed, total int  
}

var suite = testresults{0,0,0}

func (cond testbool) should(err string, args ...) {
  assert(bool(cond), err, args)
}

func error(error string, args ...) {
  suite.failed++ 
  suite.total++   
  fmt.Fprintln(os.Stderr, "Failed assertion nr", suite.total, ":", error, args);
}

func no_error() {
  suite.ok++ 
  suite.total++ 
}

func assert(cond bool, err string,  args ...)  {
  if cond {
    no_error()
    return
  }
  error(err, args) 
}

func TestResults() {
  fmt.Fprintf(os.Stderr, "Test results: %d/%d test passed, %d/%d failed.\n",
    suite.ok, suite.total, suite.failed, suite.total)
}

func TestFloat() {
  f1 := tamias.Float(10.0)
  f2 := tamias.Float(20.0)
  assert(f1 == 10.0, "Float should equal it's initial value 10.0", f1)
  assert(f1.Equals(10.0), "Float should equal it's initial value 10.0", f1)
  assert(f1.Min(f2) == f1, "Min must work properly.", f1)
  assert(f1.Max(f2) == f2, "Max must work properly.", f1)
  assert(f2.Min(f1) == f1, "Min must work properly in reverse.", f1)
  assert(f2.Max(f1) == f2, "Max must work properly in reverse.", f1)  
}

func TestBB() {
  bb := tamias.BBMake(10.0, 20.0, 40.0, 80.0)  
  // assert(bb != nil , "Bounds Box must be constructable")
  assert(bb.L == 10.0, "Bounds Box L must be 10.0", bb.L)
  assert(bb.T == 20.0, "Bounds Box T must be 20.0", bb.T)
  assert(bb.R == 40.0, "Bounds Box R must be 40.0", bb.R)
  assert(bb.B == 80.0, "Bounds Box B must be 80.0", bb.B)
  b2 := bb.Grow(10.0)
  assert(b2.Contains(bb), "Contains and grow work correctly.", bb, b2)
}


func TestShape() {
  body := tamias.BodyNew(10.0, 0.0)
  box  := tamias.BoxShapeNew(body, 20.0, 30.0)  
  box.CacheBB(body.Pos(), body.Rot())
  assert(box.GetBB() != nil, "Box must have a bounds box")
  if box.GetBB() != nil { 
    // the object should have been placed at (0,0), so half the BB
    // is positve and half negative
    // chipmunk, and hence Tamias too, use a normal Carthesian 
    // coordinate system where the zero is in the bottom left 
    assert(box.Shape.BB == box.GetBB(), "BB and GetBB() are the same")  
    assert(box.GetBB().L == -10.0, "Box must have BB.L -10.0", box.GetBB().L)
    assert(box.GetBB().T == 15.0, "Box must have BB.T -15.0", box.GetBB().T)
    assert(box.GetBB().R == 10.0, "Box must have BB.L 10.0", box.GetBB().R)
    assert(box.GetBB().B == -15.0, "Box must have BB.T -15.0", box.GetBB().B)
  }  
}

func TestVect() {  
  v1 := tamias.VF(3.0, 4.0)
  v2 := tamias.V(1.0, 0.0)
  // tamias.V(3.0, 4.0)
  assert(v1.X == 3.0, "v1.X should be 3.0", v1.X)
  assert(v1.Y == 4.0, "v1.Y should be 4.0", v1.Y)
  assert(v2.X == 1.0, "v1.X should be 1.0", v2.X)
  assert(v2.Y == 0.0, "v1.Y should be 0.0", v2.Y)
  assert(v1.Length() == 5.0, "Vector length should be 5.")
  assert(v1.Equals(v1), "Vector should be equal to itself.")
  assert(v1.Add(v2).X == 4.0, "Vector Sum X should be 4.")
  assert(v1.Add(v2).Y == 4.0, "Vector Sum X should be 4.")
  vm :=	v1.Mult(tamias.Float(2.0))
  assert(vm.Y == 8.0, "Vector Mult Y should be 8.0.", vm.X, vm.Y)
  
  
}

func TestSpaceMap() {  
  sm    := tamias.SpaceMapNew(10.0, 25)
  body  := tamias.BodyNew(10.0, 0.0)
  box   := tamias.BoxShapeNew(body, 20.0, 30.0)
  assert(sm != nil, "SpaceMap should be constructable")
  sm.Insert(box)
  bb    := box.GetBB().Grow(10.0)
  found := sm.FindBB(&bb)
  assert(found != nil, "SpaceMap should find back inserted items.", bb)
  if found != nil { 
    block := func(el interface {})(bool) {
      fmt.Println((el.(*tamias.PolyShape)))
      return el.(*tamias.PolyShape) == box
    } 
    res  := iterable.Find(found, block)
    assert(res != nil, "SpaceMap should find back the *right* inserted items.", found)
  } else {
    fmt.Printf(sm.String()) 
    // fmt.Printf()
  }  
   
  
}


type XHashEl string

type XHash map[int64] XHashEl

func main() {
  /* 
  w, err := x11.NewWindow()
  if err != nil { 
    error(err.String())
    error("Cannot open X11 window"); 
    return
  }
  w.Screen().Set(100, 100, draw.Red)
  */
  // defer w.Screen().Close() 
  /*
  m := make(XHash)
  m[123456789] = "Hello"
  m[189] = "World"
  m[789] = "Ok!"
  for k, v := range m {
    println(k)
    println(v)
  }  
  // w.Draw(100, 100, draw.Red)
  */  
  TestFloat()
  TestVect()  
  TestBB()  
  TestShape()
  TestSpaceMap()
  TestResults()
  
}
