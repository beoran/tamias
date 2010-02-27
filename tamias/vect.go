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

type Float float;


type Vect struct {
  x Float
  y Float
}

const VZERO=Vect{0.0,0.0};

func V(x Float, y Float) Vect {
  return Vect{0.0,0.0};
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
  return V(v1.x + v2.x, v1.y + v2.y);
}

func (v Vect) Neg() (Vect) {
  return V(-v.x, -v.y);
}

func (v1 Vect) Sub(v2 Vect) (Vect) {
  return V(v1.x - v2.x, v1.y - v2.y);
}

func (v1 Vect) Mult(s Float) (Vect) {
  return V(v1.x * s, v1.y * s);
}

func (v1 Vect) Dot(v2 Vect) (Float) {
  return V(v1.x * v2.x + v1.y * v2.y);
}

func (v1 Vect) Cross(v2 Vect) (Float) {
  return V(v1.x * v2.y - v1.y * v2.x);
}

func (v Vect) Perp() (Vect) {
  return V(-v.y, v.x);
}

func (v Vect) Rperp() (Vect) {
  return V(v.y, -v.x);
}

func (v1 Vect) Project(v2 Vect) (Float) {
  return v2.Mult(v1.Dot(v2) / v2.Dot(v2))
}

func (v1 Vect) Rotate(v2 Vect) (Float) {
  return V(v1.x*v2.x - v1.y*v2.y, v1.x*v2.y + v1.y*v2.x)
}

func (v1 Vect) Unrotate(v2 Vect) (Float) {
  return V(v1.x*v2.x + v1.y*v2.y, v1.y*v2.x - v1.x*v2.y)
}

func (v Vect) Lengthsq() (Float) {
  return v.Dot(v)
}

func (v1 Vect) Lerp(v2 Vect, t Float) (Float) {
  aid1 := v1.Mult(1.0 - t)
  aid2 := v2.Mult(t)
  return aid1.Add(aid2)
}

func (v Vect) Normalize() (Vect) {
  return v.Mult(1.0 / v.Length())
}

func (v Vect) NormalizeSafe() (Vect) {
  if v.x == 0.0 && v.y == 0.0 { 
    return VZERO 
  } 
  return v.Normalize()
}


func (v Vect) Clamp(len Float) (Float) {
  if  v.Lengthsq() > (len * len) {
    return v.Normalize().Mult(len)
  } 
  return v
}

func (v1 Vect) Lerpconst(v2 Vect, d Float) (Float) {
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

