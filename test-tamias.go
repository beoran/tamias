package main

import "fmt"
import "os"
import "tamias"

func error(error string) {
  fmt.Fprintln(os.Stderr, error);
}

func assert(cond bool, err string)  {
  if cond { 
    return 
  }
  error(err) 
}

func TestVect() {
  var v1 tamias.Vect
  v1 = tamias.VF(3.0, 4.0)
  // tamias.V(3.0, 4.0)
  
  assert(v1.Length() == 5.0, "Vector length should be 5.")
}

type XHashEl string

type XHash map[int64] XHashEl

func main() {
  m := make(XHash)
  // v := tamias.VF(2.0, 3.0)
  m[123456789] = "Hello"
  m[189] = "World"
  m[789] = "Ok!"
  for k, v := range m {
    println(k)
    println(v)
  }  
  
  // opencv.Init() .init is now called automatically
  // TestLoadRelease()
  // TestSave() 
  TestVect()
  error("ok")
}
