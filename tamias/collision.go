package tamias


type CollisionFunc func (a *Shape, b * Shape, * Contact) (bool);


// Add contact points for circle to circle collisions.
// Used by several collision tests.
func circle2circleQuery(p1, p2 Vect, r1, r2 Float, con * Contact) (int) {
  mindist := r1 + r2;
  delta   := p2.Sub(p1)
  distsq  := delta.Lengthsq()
  if distsq >= mindist*mindist { 
    return 0
  }  
  
  dist    := distsq.Sqrt()
  // To avoid singularities, do nothing in the case of dist = 0.
  nz_dist := dist
  if dist == 0.0 { 
   nz_dist := INFINITY
  }
  
  c1 := p1.Add(delta.Mult(Float(0.5) + (r1 - Float(0.5)*mindist) / nz_dist))
  c2 := delta.Mult(Float(1.0) / nz_dist)
  con.Init(c1, c2, dist - mindist, 0) 
  return 1
}

// Collide circle shapes.
func circle2circle(circ1, circ2 *CircleShape, arr * Contact) (int)
{
  return circle2circleQuery(circ1.tc, circ2.tc, circ1.r, circ2.r, arr);
}

// Collide circles to segment shapes.
func circle2segment(circ *CircleShape, seg *SegmentShape, con *Contact) (int) {
  // Radius sum
  rsum  := circ.r + seg.r
  
  // Calculate normal distance from segment.
  dn    := seg.tn.Dot(circ.tc) - seg.ta.Dot(seg.tn)
  dist  := dn.Abs() - rsum;
  if dist > Float(0.0) return 0;
  
  // Calculate tangential distance along segment.
  dt    := - seg.tn.Cross(circ.tc)
  dtMin := - seg.tn.Cross(seg.ta)
  dtMax := - seg.tn.Cross(seg.tb)
  
  // Decision tree to decide which feature of the segment to collide with.
  if dt < dtMin {
    if dt < (dtMin - rsum) {
      return 0
    } else {
      return circle2circleQuery(circ.tc, seg.ta, circ.r, seg.r, con)
    }
  } else {
    if dt < dtMax  {
      n := seg.tn.Neg();
      if dn < 0.0 { 
        n = seg.tn
      }    
      c1 := n.Mult(circ.r + dist * Float(0.5)) 
      con.Init(circ.tc, c1, n, dist, 0)
      return 1
    } else {
      if dt < (dtMax + rsum) {
        return circle2circleQuery(circ.tc, seg.tb, circ.r, seg.r, con)
      } else {
        return 0
      }
    }
  }  
  return 1
}

// Helper function for working with contact buffers
// This used to malloc/realloc memory on the fly but was repurposed.
func nextContactPoint(arr []Contact, num int) (contact * cpContact, num int) 
{
  if (num <= MAX_CONTACTS_PER_ARBITER) { 
    num = num + 1
  }    
  return arr[num], num;
}

// Find the minimum separating axis for the given poly and axis list.
func findMSA(poly *PolyShape, axes []PolyShapeAxis, num int) (result int, 
  min_out Float) {
  
  min_index   := 0
  min         := poly.ShapeValueOnAxis(axes.n[0], axes.d[0])
  if min > Float(0.0) { 
    return -1, Float(-1.0)
  }

  for i:=1; i<num; i++ {
    dist := poly.ShapeValueOnAxis(axes[i].n, axes[i].d)
    if dist > Float(0.0) {
      return -1, Float(-1.0)
    } else if dist > min {
      min       = dist
      min_index = i
    }
  }
    
  min_out = min
  return min_index, min_out
}

// Add contacts for penetrating vertexes.
func findVerts(arr []Contact, poly1, poly2 *PolyShape, n Vect, dist Float)
{
  num := 0;  
  for i:=0; i<poly1.numVerts; i++ {
    v := poly1->tVerts[i]
    if poly2.ContainsVertsPartial(v, n.Neg()) { 
       con, num := nextContactPoint(arr, num)
       con.Init(v, n, dist, HASH_PAIR(poly1.Shape.hashid, i))
    }   
  }
  
  for i:=0; i<poly2.numVerts; i++ {
    v := poly2->tVerts[i]
    if poly1.ContainsVertsPartial(v, n.Neg()) { 
       con, num := nextContactPoint(arr, num)
       con.Init(v, n, dist, HASH_PAIR(poly2.Shape.hashid, i))
    }
  }
  //  if(!num)
  //    addContactPoint(arr, &size, &num, cpContactNew(shape1->body->p, n, dist, 0));
  return num;
}

// Collide poly shapes together.
func poly2poly(poly1, poly2 *PolyShape, arr []cpContact) (int)
{  
  mini1, min1 := findMSA(poly2, poly1.tAxes, poly1.numVerts)
  if mini1 == -1 { 
    return 0
  }  
  
  mini2, min2 := findMSA(poly2, poly1.tAxes, poly1.numVerts)
  if mini2 == -1 { 
    return 0
  }   
  // There is overlap, find the penetrating verts
  if(min1 > min2) { 
    return findVerts(arr, poly1, poly2, poly1.tAxes[mini1].n, min1)
  } else {
    return findVerts(arr, poly1, poly2, poly2.tAxes[mini2].n.Neg(), min2)
  }  
}

// Like cpPolyValueOnAxis(), but for segments.
func segValueOnAxis(seg * SegmentShape, n Vect, d Float) (Float)
{
  a := n.dot(seg.ta) - seg.r
  b := n.dot(seg.tb) - seg.r
  return a.Min(b) - d
}

// Identify vertexes that have penetrated the segment.
func findPointsBehindSeg(arr [] Contact, num int, seg * SegmentShape, 
     poly PolyShape, pDist Float, coef Float) (int)
{
  dta := seg.tn.Cross(seg->ta)
  dtb := seg.tn.Cross(seg->tb)
  n   := seg.tn.Mult(coef)
  for i:=0; i < poly->numVerts ; i++) {
    v := poly->tVerts[i]
    if v.Dot(n) < (seg.tn.Dot(seg.ta)* coef + seg.r) {     
      dt := seg.tn.Cross(v)
      if dta >= dt && dt >= dtb {
        con, num := nextContactPoint(arr, num)
        con.Init(v, n, pDist, HASH_PAIR(poly.Shape.hashid, i))
      }
    }
  }
  return num
}

// This one is complicated and gross. Just don't go there... (sic)
// TODO: Comment me!
func seg2poly(seg SegmentShape, poly *PolyShape, arr []cpContact ) (int)
{
  axes    := poly->tAxes
  segD    := seg.tn.Dot(seg.ta)
  minNorm := poly.ShapeValueOnAxis(seg.tn, segD) - seg.r
  minNeg  := poly.ShapeValueOnAxis(seg.tn.Neg(), -segD) - seg.r
  
  if minNeg > float(0.0) || minNorm > float(0.0) { 
    return 0
  }  
  
  // Find mimimum 
  mini      := 0
  poly_min  := segValueOnAxis(seg, axes[0].n, axes[0].d)  
  if poly_min > Float(0.0) { 
    return 0
  }  
  
  for  i:=0; i < poly->numVerts ; i++ {
    dist := segValueOnAxis(seg, axes[i].n, axes[i].d);
    if dist > Float(0.0) {
      return 0
    } else if dist > poly_min {
      poly_min = dist
      mini = i
    }
  }
  
  num     := 0
  poly_n  := axes[mini].n.Neg();
  va      := seg.ta.Add(poly_n.Mult(seg.r))
  vb      := seg.tb.Add(poly_n.Mult(seg.r))
  if poly.ContainsVert(va) { 
    con, num := nextContactPoint(arr, num)
    con.Init(va, poly_n, poly_min, HASH_PAIR(seg.Shape.hashid, 0))
  }
  
  if poly.ContainsVert(vb) { 
    con, num := nextContactPoint(arr, num)
    con.Init(va, poly_n, poly_min, HASH_PAIR(seg.Shape.hashid, 1))
  }   
  // Floating point precision problems here.
  // This will have to do for now.
  poly_min -= CollisionSlop
  if minNorm >= poly_min || minNeg >= poly_min {
    if(minNorm > minNeg) {
      num = findPointsBehindSeg(arr, num, seg, poly, minNorm, Float(1.0));      
    } else {
      num = findPointsBehindSeg(arr, num, seg, poly, minNeg, Float(-1.0));
    }  
  }
  
  // If no other collision points are found, try colliding endpoints.
  if(num == 0) {
    poly_a := poly.tVerts[mini];
    poly_b := poly.tVerts[(mini + 1)%poly.numVerts];
    
    if circle2circleQuery(seg.ta, poly_a, seg.r, 0.0f, arr[0]) > 0 {
      return 1;
    }  
      
    if circle2circleQuery(seg.tb, poly_a, seg.r, 0.0f, arr[0]) > 0 {
      return 1;
    } 
      
    if circle2circleQuery(seg.ta, poly_b, seg.r, 0.0f, arr[0]) > 0 {
      return 1;
    }  
      
    if circle2circleQuery(seg.tb, poly_b, seg.r, 0.0f, arr[0]) > 0 {
      return 1;
    }  
  }

  return num;
}

// This one is less gross, but still gross. (sic)
// TODO: Comment me!
func circle2poly(cic *CircleShape, poly *PolyShape, con *Contact) (int) {
  axes := poly->tAxes;
  
  // find minimum  
  mini := 0;
  min  := axes[0].n.Dot(circ.tc) - axes[0].d - circ.r
  for int i=0; i<poly->numVerts; i++ {
    dist := axes[i].n.Dot(circ.tc) - axes[i].d - circ.r
    if dist > 0.0 {
      return 0
    } else if dist > min {
      min   = dist
      mini  = i
    }
  }
  
  n   := axes[mini].n;
  a   := poly.tVerts[mini];
  b   := poly.tVerts[(mini + 1)%poly.numVerts];
  dta := n.Cross(a)
  dtb := n.Cross(b) 
  dt  := n.Cross(n, circ.tc);
  if dt < dtb {
    return circle2circleQuery(circ.tc, b, circ.r, Float(0.0), con)
  } else if dt < dta {
    con.Init(circ.tc.Sub(n.Mult(circ.r + min / Float(2.0))), n.Neg(), min, 0)  
    return 1;
  } else {
    return circle2circleQuery(circ.tc, a, circ.r, Float(0.0), con)
  }
}

/*
TODO: this must probably be done differently in Go...

//static const collisionFunc builtinCollisionFuncs[9] = {
//  circle2circle,
//  NULL,
//  NULL,
//  circle2segment,
//  NULL,
//  NULL,
//  circle2poly,
//  seg2poly,
//  poly2poly,
//};
//static const collisionFunc *colfuncs = builtinCollisionFuncs;

static collisionFunc *colfuncs = NULL;

static void
addColFunc(cpShapeType a, cpShapeType b, collisionFunc func)
{
  colfuncs[a + b*CP_NUM_SHAPES] = func;
}

#ifdef __cplusplus
extern "C" {
#endif
  void cpInitCollisionFuncs(void);
  
  // Initializes the array of collision functions.
  // Called by cpInitChipmunk().
  void
  cpInitCollisionFuncs(void)
  {
    if(!colfuncs)
      colfuncs = (collisionFunc *)cpcalloc(CP_NUM_SHAPES*CP_NUM_SHAPES, sizeof(collisionFunc));
    
    addColFunc(CP_CIRCLE_SHAPE,  CP_CIRCLE_SHAPE,  circle2circle);
    addColFunc(CP_CIRCLE_SHAPE,  CP_SEGMENT_SHAPE, circle2segment);
    addColFunc(CP_SEGMENT_SHAPE, CP_POLY_SHAPE,    seg2poly);
    addColFunc(CP_CIRCLE_SHAPE,  CP_POLY_SHAPE,    circle2poly);
    addColFunc(CP_POLY_SHAPE,    CP_POLY_SHAPE,    poly2poly);
  } 
#ifdef __cplusplus
}
#endif
*/

func CollideShapes(cpShape *a, cpShape *b, cpContact *arr) (int) {
  // Their shape types must be in order.
  Assert(a->ShapeClass.Type <= b->ShapeClass->Type, 
    "Collision shapes passed to cpCollideShapes() are not sorted.");
  /*
  collisionFunc cfunc = colfuncs[a->klass->type + b->klass->type*CP_NUM_SHAPES];
  return (cfunc) ? cfunc(a, b, arr) : 0;
  */
  // TODO: make this work
  return 0  
}



