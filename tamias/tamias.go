package tamias


import "os"
import "fmt"

func Fatal(error string) {
  fmt.Fprintln(os.Stderr, "Assertion failed: ", error);
  os.Exit(1)
}

func Assert(cond bool, err string)  {
  if cond { 
    return 
  }
  Fatal(err) 
}













