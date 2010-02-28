/* Copyright (c) 2010 Beoran
 * Copyright (c) 2007 Scott Lembcke
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package tamias

import "fmt"

// import "math"

// type Float float;


type Vect struct {
  X Float
  Y Float
}

var VZERO=Vect{0.0,0.0};

func V(x Float, y Float) (Vect) {
  result := Vect{X:x, Y:y}
  return result
}

func VF(y float, x float) (Vect) {
  return V(Float(x), Float(y))
}

// non-inlined functions
/*
cpFloat cpvlength(const cpVect v);
cpVect cpvslerp(const cpVect v1, const cpVect v2, const cpFloat t);
cpVect cpvslerpconst(const cpVect v1, const cpVect v2, const cpFloat a);
cpVect cpvforangle(const cpFloat a); // convert radians to a normalized vector
cpFloat cpvtoangle(const cpVect v); // convert a vector to radians
char *cpvstr(const cpVect v); // get a string representation of a vector
*/

func (v1 Vect) Add(v2 Vect) (Vect) {
  return V(v1.X + v2.X, v1.Y + v2.Y)
}

func (v Vect) Neg() (Vect) {
  return V(-v.X, -v.Y)
}

func (v1 Vect) Sub(v2 Vect) (Vect) {
  return V(v1.X - v2.X, v1.Y - v2.Y)
}

func (v1 Vect) Mult(s Float) (Vect) {
  return V(v1.X * s, v1.Y * s)
}

func (v1 Vect) Dot(v2 Vect) (Float) {
  return v1.X * v2.X + v1.Y * v2.Y
}

func (v1 Vect) Cross(v2 Vect) (Float) {
  return v1.X * v2.Y - v1.Y * v2.X
}

func (v Vect) Perp() (Vect) {
  return V(-v.Y, v.X)
}

func (v Vect) Rperp() (Vect) {
  return V(v.Y, -v.X)
}

func (v1 Vect) Project(v2 Vect) (Vect) {
  return v2.Mult(v1.Dot(v2) / v2.Dot(v2))
}

func (v1 Vect) Rotate(v2 Vect) (Vect) {
  return V(v1.X*v2.X - v1.Y*v2.Y, v1.X*v2.Y + v1.Y*v2.X)
}

func (v1 Vect) Unrotate(v2 Vect) (Vect) {
  return V(v1.X*v2.X + v1.Y*v2.Y, v1.Y*v2.X - v1.X*v2.Y)
}

func (v Vect) Lengthsq() (Float) {
  return v.Dot(v)
}

func (v Vect) Length() (Float) {
  return v.Lengthsq().Sqrt()
}


func (v1 Vect) Lerp(v2 Vect, t Float) (Vect) {
  aid1 := v1.Mult(1.0 - t)
  aid2 := v2.Mult(t)
  return aid1.Add(aid2)
}

func (v Vect) Normalize() (Vect) {
  return v.Mult(1.0 / v.Length())
}

func (v Vect) NormalizeSafe() (Vect) {
  if v.X == 0.0 && v.Y == 0.0 { 
    return VZERO 
  } 
  return v.Normalize()
}


func (v Vect) Clamp(len Float) (Vect) {
  if  v.Lengthsq() > (len * len) {
    return v.Normalize().Mult(len)
  } 
  return v
}

func (v1 Vect) Lerpconst(v2 Vect, d Float) (Vect) {
  return v1.Add(v2.Sub(v1).Clamp(d));
}

func (v1 Vect) Dist(v2 Vect) (Float) {
  return v1.Sub(v2).Length()
}

func (v1 Vect) Distsq(v2 Vect) (Float) {
  return v1.Sub(v2).Lengthsq()
}


func (v1 Vect) Near(v2 Vect, dist Float) (bool) {
  return v1.Distsq(v2) < dist*dist
}


func (v1 Vect) Slerp(v2 Vect, t Float) (Vect) {
  omega := v1.Dot(v2).Acos();
  if(omega != 0.0){
    denom := 1.0 / omega.Sin();
    par1  := ((1.0 - t)*omega).Sin() *denom
    par2  := ((t)*omega).Sin() *denom
    aid1  := v1.Mult(par1)
    aid2  := v2.Mult(par2)
    return aid1.Add(aid2) 
  }     
  // else
  return v1;
}

func (v1 Vect) Slerpconst(v2 Vect, a Float) (Vect) {
  angle := v1.Dot(v2).Acos()
  return v1.Slerp(v2, a.Min(angle) / angle)
}

func ForAngle(a Float) (Vect)  {
  return V(a.Cos(), a.Sin());
}

func (v Vect) ToAngle() (Float) {
  return v.Y.Atan2(v.X)
}

func (v Vect) String() (string) {
  var str string
  fmt.Sprintf(str, "(% .3f, % .3f)", v.X, v.Y)
  return str
}

