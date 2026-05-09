package detour

// Vcross computes the cross product of two vectors (v1 x v2).
func Vcross(dest, v1, v2 []float32) {
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]
}

// Vdot computes the dot product of two vectors.
func Vdot(v1, v2 []float32) float32 {
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
}

// Vmad performs a scaled vector addition (v1 + v2 * s).
func Vmad(dest, v1, v2 []float32, s float32) {
	dest[0] = v1[0] + v2[0]*s
	dest[1] = v1[1] + v2[1]*s
	dest[2] = v1[2] + v2[2]*s
}

// Vlerp performs a linear interpolation between two vectors.
func Vlerp(dest, v1, v2 []float32, t float32) {
	dest[0] = v1[0] + (v2[0]-v1[0])*t
	dest[1] = v1[1] + (v2[1]-v1[1])*t
	dest[2] = v1[2] + (v2[2]-v1[2])*t
}

// Vadd performs vector addition.
func Vadd(dest, v1, v2 []float32) {
	dest[0] = v1[0] + v2[0]
	dest[1] = v1[1] + v2[1]
	dest[2] = v1[2] + v2[2]
}

// Vsub performs vector subtraction.
func Vsub(dest, v1, v2 []float32) {
	dest[0] = v1[0] - v2[0]
	dest[1] = v1[1] - v2[1]
	dest[2] = v1[2] - v2[2]
}

// Vscale scales the vector by the specified value.
func Vscale(dest, v []float32, t float32) {
	dest[0] = v[0] * t
	dest[1] = v[1] * t
	dest[2] = v[2] * t
}

// Vmin selects the minimum value of each element from the specified vectors.
func Vmin(mn, v []float32) {
	if v[0] < mn[0] {
		mn[0] = v[0]
	}
	if v[1] < mn[1] {
		mn[1] = v[1]
	}
	if v[2] < mn[2] {
		mn[2] = v[2]
	}
}

// Vmax selects the maximum value of each element from the specified vectors.
func Vmax(mx, v []float32) {
	if v[0] > mx[0] {
		mx[0] = v[0]
	}
	if v[1] > mx[1] {
		mx[1] = v[1]
	}
	if v[2] > mx[2] {
		mx[2] = v[2]
	}
}

// Vset sets the vector elements to the specified values.
func Vset(dest []float32, x, y, z float32) {
	dest[0] = x
	dest[1] = y
	dest[2] = z
}

// Vcopy copies a vector.
func Vcopy(dest, a []float32) {
	dest[0] = a[0]
	dest[1] = a[1]
	dest[2] = a[2]
}

// Vlen computes the scalar length of a vector.
func Vlen(v []float32) float32 {
	return MathSqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
}

// VlenSqr computes the squared length of a vector.
func VlenSqr(v []float32) float32 {
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
}

// Vdist computes the distance between two points.
func Vdist(v1, v2 []float32) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return MathSqrtf(dx*dx + dy*dy + dz*dz)
}

// VdistSqr computes the squared distance between two points.
func VdistSqr(v1, v2 []float32) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return dx*dx + dy*dy + dz*dz
}

// Vdist2D computes the 2D distance on the xz-plane.
func Vdist2D(v1, v2 []float32) float32 {
	dx := v2[0] - v1[0]
	dz := v2[2] - v1[2]
	return MathSqrtf(dx*dx + dz*dz)
}

// Vdist2DSqr computes the squared 2D distance on the xz-plane.
func Vdist2DSqr(v1, v2 []float32) float32 {
	dx := v2[0] - v1[0]
	dz := v2[2] - v1[2]
	return dx*dx + dz*dz
}

// Vnormalize normalizes a vector.
func Vnormalize(v []float32) {
	d := 1.0 / MathSqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
	v[0] *= d
	v[1] *= d
	v[2] *= d
}

// Vequal performs a 'sloppy' colocation check.
func Vequal(p0, p1 []float32) bool {
	const thr = 1.0 / (16384.0 * 16384.0)
	d := VdistSqr(p0, p1)
	return d < thr
}

// Visfinite checks that the vector's components are all finite.
func Visfinite(v []float32) bool {
	return MathIsfinite(v[0]) && MathIsfinite(v[1]) && MathIsfinite(v[2])
}

// Visfinite2D checks that the vector's 2D components are finite.
func Visfinite2D(v []float32) bool {
	return MathIsfinite(v[0]) && MathIsfinite(v[2])
}

// Vdot2D computes dot product on the xz-plane.
func Vdot2D(u, v []float32) float32 {
	return u[0]*v[0] + u[2]*v[2]
}

// Vperp2D computes the xz-plane 2D perp product (uz*vx - ux*vz).
func Vperp2D(u, v []float32) float32 {
	return u[2]*v[0] - u[0]*v[2]
}

// TriArea2D computes signed xz-plane area of triangle ABC.
func TriArea2D(a, b, c []float32) float32 {
	abx := b[0] - a[0]
	abz := b[2] - a[2]
	acx := c[0] - a[0]
	acz := c[2] - a[2]
	return acx*abz - abx*acz
}

// OverlapQuantBounds determines if two AABBs (quantized) overlap.
func OverlapQuantBounds(amin, amax, bmin, bmax []uint16) bool {
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		return false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		return false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		return false
	}
	return true
}

// OverlapBounds determines if two AABBs overlap.
func OverlapBounds(amin, amax, bmin, bmax []float32) bool {
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		return false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		return false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		return false
	}
	return true
}

// ClosestPtPointTriangle derives the closest point on a triangle from the reference point.
func ClosestPtPointTriangle(closest, p, a, b, c []float32) {
	// Check if P in vertex region outside A
	ab := make([]float32, 3)
	ac := make([]float32, 3)
	ap := make([]float32, 3)
	Vsub(ab, b, a)
	Vsub(ac, c, a)
	Vsub(ap, p, a)
	d1 := Vdot(ab, ap)
	d2 := Vdot(ac, ap)
	if d1 <= 0.0 && d2 <= 0.0 {
		Vcopy(closest, a)
		return
	}

	// Check if P in vertex region outside B
	bp := make([]float32, 3)
	Vsub(bp, p, b)
	d3 := Vdot(ab, bp)
	d4 := Vdot(ac, bp)
	if d3 >= 0.0 && d4 <= d3 {
		Vcopy(closest, b)
		return
	}

	// Check if P in edge region of AB
	vc := d1*d4 - d3*d2
	if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
		v := d1 / (d1 - d3)
		closest[0] = a[0] + v*ab[0]
		closest[1] = a[1] + v*ab[1]
		closest[2] = a[2] + v*ab[2]
		return
	}

	// Check if P in vertex region outside C
	cp := make([]float32, 3)
	Vsub(cp, p, c)
	d5 := Vdot(ab, cp)
	d6 := Vdot(ac, cp)
	if d6 >= 0.0 && d5 <= d6 {
		Vcopy(closest, c)
		return
	}

	// Check if P in edge region of AC
	vb := d5*d2 - d1*d6
	if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
		w := d2 / (d2 - d6)
		closest[0] = a[0] + w*ac[0]
		closest[1] = a[1] + w*ac[1]
		closest[2] = a[2] + w*ac[2]
		return
	}

	// Check if P in edge region of BC
	va := d3*d6 - d5*d4
	if va <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0 {
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		closest[0] = b[0] + w*(c[0]-b[0])
		closest[1] = b[1] + w*(c[1]-b[1])
		closest[2] = b[2] + w*(c[2]-b[2])
		return
	}

	// P inside face region
	denom := 1.0 / (va + vb + vc)
	v := vb * denom
	w := vc * denom
	closest[0] = a[0] + ab[0]*v + ac[0]*w
	closest[1] = a[1] + ab[1]*v + ac[1]*w
	closest[2] = a[2] + ab[2]*v + ac[2]*w
}

// IntersectSegmentPoly2D checks segment-polygon intersection on the xz-plane.
func IntersectSegmentPoly2D(p0, p1, verts []float32, nverts int) (bool, float32, float32, int, int) {
	const eps = 0.000001

	tmin := float32(0)
	tmax := float32(1)
	segMin := -1
	segMax := -1

	dir := make([]float32, 3)
	Vsub(dir, p1, p0)

	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		edge := make([]float32, 3)
		diff := make([]float32, 3)
		Vsub(edge, verts[i*3:(i*3)+3], verts[j*3:(j*3)+3])
		Vsub(diff, p0, verts[j*3:(j*3)+3])
		n := Vperp2D(edge, diff)
		d := Vperp2D(dir, edge)
		if MathFabsf(d) < eps {
			if n < 0 {
				return false, 0, 0, 0, 0
			}
			continue
		}
		t := n / d
		if d < 0 {
			if t > tmin {
				tmin = t
				segMin = j
				if tmin > tmax {
					return false, 0, 0, 0, 0
				}
			}
		} else {
			if t < tmax {
				tmax = t
				segMax = j
				if tmax < tmin {
					return false, 0, 0, 0, 0
				}
			}
		}
	}

	return true, tmin, tmax, segMin, segMax
}

// DistancePtSegSqr2D computes the squared distance between a point and a segment in 2D.
func DistancePtSegSqr2D(pt, p, q []float32) (float32, float32) {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]
	return dx*dx + dz*dz, t
}

// CalcPolyCenter computes the centroid of a convex polygon.
func CalcPolyCenter(tc []float32, idx []uint16, verts []float32) {
	tc[0] = 0
	tc[1] = 0
	tc[2] = 0
	nidx := len(idx)
	for j := 0; j < nidx; j++ {
		v := verts[idx[j]*3 : idx[j]*3+3]
		tc[0] += v[0]
		tc[1] += v[1]
		tc[2] += v[2]
	}
	s := 1.0 / float32(nidx)
	tc[0] *= s
	tc[1] *= s
	tc[2] *= s
}

// ClosestHeightPointTriangle computes the height of the closest point on a triangle.
func ClosestHeightPointTriangle(p, a, b, c []float32) (bool, float32) {
	const eps = 1e-6

	v0 := make([]float32, 3)
	v1 := make([]float32, 3)
	v2 := make([]float32, 3)
	Vsub(v0, c, a)
	Vsub(v1, b, a)
	Vsub(v2, p, a)

	denom := v0[0]*v1[2] - v0[2]*v1[0]
	if MathFabsf(denom) < eps {
		return false, 0
	}

	u := v1[2]*v2[0] - v1[0]*v2[2]
	v := v0[0]*v2[2] - v0[2]*v2[0]

	if denom < 0 {
		denom = -denom
		u = -u
		v = -v
	}

	if u >= 0.0 && v >= 0.0 && (u+v) <= denom {
		h := a[1] + (v0[1]*u+v1[1]*v)/denom
		return true, h
	}
	return false, 0
}

// PointInPolygon determines if a point is inside a convex polygon on the xz-plane.
func PointInPolygon(pt, verts []float32, nverts int) bool {
	c := false
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vi := verts[i*3 : i*3+3]
		vj := verts[j*3 : j*3+3]
		if ((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0])*(pt[2]-vi[2])/(vj[2]-vi[2])+vi[0]) {
			c = !c
		}
	}
	return c
}

// DistancePtPolyEdgesSqr computes distance from point to polygon edges.
func DistancePtPolyEdgesSqr(pt, verts []float32, nverts int, ed, et []float32) bool {
	c := false
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vi := verts[i*3 : i*3+3]
		vj := verts[j*3 : j*3+3]
		if ((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0])*(pt[2]-vi[2])/(vj[2]-vi[2])+vi[0]) {
			c = !c
		}
		ed[j], et[j] = DistancePtSegSqr2D(pt, vj, vi)
	}
	return c
}

func projectPoly(axis, poly []float32, npoly int) (float32, float32) {
	rmin := Vdot2D(axis, poly[0:3])
	rmax := rmin
	for i := 1; i < npoly; i++ {
		d := Vdot2D(axis, poly[i*3:i*3+3])
		if d < rmin {
			rmin = d
		}
		if d > rmax {
			rmax = d
		}
	}
	return rmin, rmax
}

func overlapRange(amin, amax, bmin, bmax, eps float32) bool {
	if (amin+eps) > bmax || (amax-eps) < bmin {
		return false
	}
	return true
}

// OverlapPolyPoly2D determines if two convex polygons overlap on the xz-plane.
func OverlapPolyPoly2D(polya []float32, npolya int, polyb []float32, npolyb int) bool {
	const eps = 1e-4

	for i, j := 0, npolya-1; i < npolya; j, i = i, i+1 {
		va := polya[j*3 : j*3+3]
		vb := polya[i*3 : i*3+3]
		n := []float32{vb[2] - va[2], 0, -(vb[0] - va[0])}
		amin, amax := projectPoly(n, polya, npolya)
		bmin, bmax := projectPoly(n, polyb, npolyb)
		if !overlapRange(amin, amax, bmin, bmax, eps) {
			return false
		}
	}
	for i, j := 0, npolyb-1; i < npolyb; j, i = i, i+1 {
		va := polyb[j*3 : j*3+3]
		vb := polyb[i*3 : i*3+3]
		n := []float32{vb[2] - va[2], 0, -(vb[0] - va[0])}
		amin, amax := projectPoly(n, polya, npolya)
		bmin, bmax := projectPoly(n, polyb, npolyb)
		if !overlapRange(amin, amax, bmin, bmax, eps) {
			return false
		}
	}
	return true
}

// RandomPointInConvexPoly returns a random point in a convex polygon.
func RandomPointInConvexPoly(pts []float32, npts int, areas []float32, s, t float32, out []float32) {
	// Calc triangle areas
	areasum := float32(0)
	for i := 2; i < npts; i++ {
		areas[i] = TriArea2D(pts[0:3], pts[(i-1)*3:(i-1)*3+3], pts[i*3:i*3+3])
		if areas[i] < 0.001 {
			areas[i] = 0.001
		}
		areasum += areas[i]
	}
	// Find sub triangle weighted by area
	thr := s * areasum
	acc := float32(0)
	u := float32(1)
	tri := npts - 1
	for i := 2; i < npts; i++ {
		dacc := areas[i]
		if thr >= acc && thr < (acc+dacc) {
			u = (thr - acc) / dacc
			tri = i
			break
		}
		acc += dacc
	}

	v := MathSqrtf(t)

	a := 1 - v
	b := (1 - u) * v
	c := u * v
	pa := pts[0:3]
	pb := pts[(tri-1)*3 : (tri-1)*3+3]
	pc := pts[tri*3 : tri*3+3]

	out[0] = a*pa[0] + b*pb[0] + c*pc[0]
	out[1] = a*pa[1] + b*pb[1] + c*pc[1]
	out[2] = a*pa[2] + b*pb[2] + c*pc[2]
}

func vperpXZ(a, b []float32) float32 {
	return a[0]*b[2] - a[2]*b[0]
}

// IntersectSegSeg2D checks if two segments intersect in 2D.
func IntersectSegSeg2D(ap, aq, bp, bq []float32) (bool, float32, float32) {
	u := make([]float32, 3)
	v := make([]float32, 3)
	w := make([]float32, 3)
	Vsub(u, aq, ap)
	Vsub(v, bq, bp)
	Vsub(w, ap, bp)
	d := vperpXZ(u, v)
	if MathFabsf(d) < 1e-6 {
		return false, 0, 0
	}
	s := vperpXZ(v, w) / d
	t := vperpXZ(u, w) / d
	return true, s, t
}

// NextPow2 returns the next power of two.
func NextPow2(v uint32) uint32 {
	v--
	v |= v >> 1
	v |= v >> 2
	v |= v >> 4
	v |= v >> 8
	v |= v >> 16
	v++
	return v
}

// Ilog2 returns the integer log base 2 of a value (>0).
func Ilog2(v uint32) uint32 {
	var r uint32
	var shift uint32
	r = 0
	if v > 0xffff {
		r = 16
		v >>= 16
	}
	if v > 0xff {
		shift = 8
		v >>= shift
		r |= shift
	}
	if v > 0xf {
		shift = 4
		v >>= shift
		r |= shift
	}
	if v > 0x3 {
		shift = 2
		v >>= shift
		r |= shift
	}
	r |= (v >> 1)
	return r
}

// Align4 aligns an integer to 4 bytes.
func Align4(x int) int {
	return (x + 3) & ^3
}

// OppositeTile returns the opposite side.
func OppositeTile(side int) int {
	return (side + 4) & 0x7
}

// Swap swaps two values.
func Swap[T any](a, b *T) {
	*a, *b = *b, *a
}

// Min returns the minimum of two values.
func Min[T int | int32 | uint32 | float32](a, b T) T {
	if a < b {
		return a
	}
	return b
}

// Max returns the maximum of two values.
func Max[T int | int32 | uint32 | float32](a, b T) T {
	if a > b {
		return a
	}
	return b
}

// Abs returns the absolute value.
func Abs[T int | int32 | float32](a T) T {
	if a < 0 {
		return -a
	}
	return a
}

// Sqr returns the square of the value.
func Sqr[T int | uint32 | float32](a T) T {
	return a * a
}

// Clamp clamps the value to the specified range.
func Clamp[T int | int32 | uint32 | float32](v, mn, mx T) T {
	if v < mn {
		return mn
	}
	if v > mx {
		return mx
	}
	return v
}

// clampInt clamps an int to [0, 0xffff].
func clampInt16(v int) uint16 {
	if v < 0 {
		return 0
	}
	if v > 0xffff {
		return 0xffff
	}
	return uint16(v)
}

// IntersectSegmentSeg2D checks if two line segments intersect on the xz-plane.
// Returns (intersects, s, t) where s is the parameter along segment A (ap->aq)
// and t is the parameter along segment B (bp->bq).
func IntersectSegmentSeg2D(ap, aq, bp, bq []float32) (bool, float32, float32) {
	u := []float32{aq[0] - ap[0], aq[1] - ap[1], aq[2] - ap[2]}
	v := []float32{bq[0] - bp[0], bq[1] - bp[1], bq[2] - bp[2]}
	w := []float32{ap[0] - bp[0], ap[1] - bp[1], ap[2] - bp[2]}
	d := Vperp2D(u, v)
	if MathFabsf(d) < 1e-6 {
		return false, 0, 0
	}
	s := Vperp2D(v, w) / d
	t := Vperp2D(u, w) / d
	return true, s, t
}
