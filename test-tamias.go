package main

import "fmt"
import "os"
import "tamias"
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
  assert(f1 == 10.0, "Float should equal it's initial value 10.0", f1)
  assert(f1.Equals(10.0), "Float should equal it's initial value 10.0", f1)
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
  TestResults()
  
}
