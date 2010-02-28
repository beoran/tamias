package tamias

//Float, a floating point type for Tamias, with some method sugar.

import "math"

type Float float;

func F64Float(in float64) (Float) {
  return Float(in)
}

func (self Float) Float64() (float64) {
  return float64(self)
}

func (self Float) Sqrt() (Float) {
  return Float(math.Sqrt(self.Float64()))
}


func (a Float) Max(b Float) (Float) { 
  if a > b { 
    return a
  }
  return b
}

func (a Float) Min(b Float) (Float) { 
  if a < b { 
    return a
  }
  return b
}

func (a Float) Mod(b Float) (Float) { 
  return Float(math.Fmod(a.Float64(), b.Float64()))
}

func (a Float) Abs() (Float) { 
  if a < 0.0 { 
    return -a
  }
  return a
}

func (a Float) Sin() (Float) { 
  return Float(math.Sin(a.Float64()))
}

func (a Float) Cos() (Float) { 
  return Float(math.Cos(a.Float64()))
}

func (a Float) Tan() (Float) { 
  return Float(math.Tan(a.Float64()))
}

func (a Float) Acos() (Float) { 
  return Float(math.Acos(a.Float64()))
}

func (a Float) Asin() (Float) { 
  return Float(math.Asin(a.Float64()))
}

func (a Float) Atan() (Float) {
  return Float(math.Atan(a.Float64()))
} 


func (x Float) Atan2(y Float) (Float) {
  return Float(math.Atan2(y.Float64(), x.Float64()))
} 

func (f Float) Clamp(min Float, max Float) (Float) { 
  return f.Max(min).Min(max)
}

func (f1 Float) Lerp(f2 Float, t Float) (Float) { 
  return (f1 * (1.0 - t)) + (f2 * t);
} 

func (f1 Float) Flerpconst(f2 Float, d Float) (Float) { 
  return f1 + (f2 - f1).Clamp(-d, d)
}

func (a Float) VectForAngle() (Vect) {
  return V(a.Cos(), a.Sin())
}

var INFINITY = Float(math.Inf(1))

