package recast

import (
	"math"
	"sort"
)

// HeightPatch represents a rectangular region of height data extracted from the compact heightfield.
type HeightPatch struct {
	data          []uint16
	xmin, ymin    int
	width, height int
}

const unsetHeight uint16 = 0xffff

// EdgeValues for triangulation.
const (
	evUndef = -1
	evHull  = -2
)

// ContourHole represents a hole in a contour.
type ContourHole struct {
	contour              *Contour
	minx, minz, leftmost int
}

// ContourRegion groups an outline contour with its holes.
type ContourRegion struct {
	outline *Contour
	holes   []ContourHole
	nholes  int
}

// PotentialDiagonal represents a candidate diagonal for hole merging.
type PotentialDiagonal struct {
	vert int
	dist int
}

func clamp(v, mn, mx int) int {
	if v < mn {
		return mn
	}
	if v > mx {
		return mx
	}
	return v
}

func vdot2(a, b [3]float32) float32 {
	return a[0]*b[0] + a[2]*b[2]
}

func vdistSq2(p, q [3]float32) float32 {
	dx := q[0] - p[0]
	dy := q[2] - p[2]
	return dx*dx + dy*dy
}

func vdist2(p, q [3]float32) float32 {
	return float32(math.Sqrt(float64(vdistSq2(p, q))))
}

func vcross2(p1, p2, p3 [3]float32) float32 {
	u1 := p2[0] - p1[0]
	v1 := p2[2] - p1[2]
	u2 := p3[0] - p1[0]
	v2 := p3[2] - p1[2]
	return u1*v2 - v1*u2
}

func circumCircle(p1, p2, p3 [3]float32) (c [3]float32, r float32, ok bool) {
	const eps float32 = 1e-6

	// Compute edges from p1.
	// v1 is intentionally left as zero vector here (mirrors the original C code structure).
	v2 := Vsub(p2, p1)
	v3 := Vsub(p3, p1)

	// 2D cross product of v2 and v3 in xz plane.
	cp := v2[0]*v3[2] - v3[0]*v2[2]
	if float32(math.Abs(float64(cp))) > eps {
		v2Sq := vdot2(v2, v2)
		v3Sq := vdot2(v3, v3)
		c[0] = (v2Sq*(v3[2]-0) + v3Sq*(0-v2[2]) + 0*(v2[0]-v3[0])) / (2 * cp)
		c[1] = 0
		c[2] = (v2Sq*(v3[0]-0) + v3Sq*(0-v2[0]) + 0*(v2[0]-v3[0])) / (2 * cp)
		r = vdist2(c, v2)
		c = Vadd(c, p1)
		return c, r, true
	}

	return p1, 0, false
}

func distPtTri(p, a, b, c [3]float32) float32 {
	v0 := Vsub(c, a)
	v1 := Vsub(b, a)
	v2 := Vsub(p, a)

	dot00 := vdot2(v0, v0)
	dot01 := vdot2(v0, v1)
	dot02 := vdot2(v0, v2)
	dot11 := vdot2(v1, v1)
	dot12 := vdot2(v1, v2)

	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	const eps float32 = 1e-4
	if u >= -eps && v >= -eps && (u+v) <= 1+eps {
		y := a[1] + v0[1]*u + v1[1]*v
		return float32(math.Abs(float64(y - p[1])))
	}
	return math.MaxFloat32
}

func distancePtSeg(pt, p, q [3]float32) float32 {
	pqx := q[0] - p[0]
	pqy := q[1] - p[1]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dy := pt[1] - p[1]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqy*pqy + pqz*pqz
	t := pqx*dx + pqy*dy + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	dx = p[0] + t*pqx - pt[0]
	dy = p[1] + t*pqy - pt[1]
	dz = p[2] + t*pqz - pt[2]

	return dx*dx + dy*dy + dz*dz
}

func distancePtSeg2d(pt, p, q [3]float32) float32 {
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

	return dx*dx + dz*dz
}

func distToTriMesh(p [3]float32, verts []float32, tris []int, ntris int) float32 {
	dmin := float32(math.MaxFloat32)
	for i := 0; i < ntris; i++ {
		va := toV3(verts[tris[i*4+0]*3:])
		vb := toV3(verts[tris[i*4+1]*3:])
		vc := toV3(verts[tris[i*4+2]*3:])
		d := distPtTri(p, va, vb, vc)
		if d < dmin {
			dmin = d
		}
	}
	if dmin == math.MaxFloat32 {
		return -1
	}
	return dmin
}

func toV3(s []float32) [3]float32 {
	return [3]float32{s[0], s[1], s[2]}
}

func distToPoly(nvert int, verts []float32, p [3]float32) float32 {
	dmin := float32(math.MaxFloat32)
	var i, j int
	c := false
	for i, j = 0, nvert-1; i < nvert; j = i {
		i++
		vi := toV3(verts[i*3:])
		vj := toV3(verts[j*3:])
		if (vi[2] > p[2]) != (vj[2] > p[2]) &&
			p[0] < (vj[0]-vi[0])*(p[2]-vi[2])/(vj[2]-vi[2])+vi[0] {
			c = !c
		}
		dmin = MinF(dmin, distancePtSeg2d(p, vj, vi))
	}
	if c {
		return -dmin
	}
	return dmin
}

func getHeight(fx, fy, fz float32, cs, ics, ch float32, radius int, hp *HeightPatch) uint16 {
	ix := int(math.Floor(float64(fx*ics + 0.01)))
	iz := int(math.Floor(float64(fz*ics + 0.01)))
	ix = clamp(ix-hp.xmin, 0, hp.width-1)
	iz = clamp(iz-hp.ymin, 0, hp.height-1)
	h := hp.data[ix+iz*hp.width]
	if h == unsetHeight {
		x := 1
		z := 0
		dx := 1
		dz := 0
		maxSize := radius*2 + 1
		maxIter := maxSize*maxSize - 1

		nextRingIterStart := 8
		nextRingIters := 16

		dmin := float32(math.MaxFloat32)
		for i := 0; i < maxIter; i++ {
			nx := ix + x
			nz := iz + z

			if nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height {
				nh := hp.data[nx+nz*hp.width]
				if nh != unsetHeight {
					d := float32(math.Abs(float64(nh)*float64(ch) - float64(fy)))
					if d < dmin {
						h = nh
						dmin = d
					}
				}
			}

			if i+1 == nextRingIterStart {
				if h != unsetHeight {
					break
				}
				nextRingIterStart += nextRingIters
				nextRingIters += 8
			}

			if (x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1-z)) {
				tmp := dx
				dx = -dz
				dz = tmp
			}
			x += dx
			z += dz
		}
	}
	return h
}

func findEdge(edges []int, nedges, s, t int) int {
	for i := 0; i < nedges; i++ {
		e := edges[i*4:]
		if (e[0] == s && e[1] == t) || (e[0] == t && e[1] == s) {
			return i
		}
	}
	return evUndef
}

func addEdge(edges []int, nedges *int, maxEdges, s, t, l, r int) int {
	if *nedges >= maxEdges {
		return evUndef
	}

	e := findEdge(edges, *nedges, s, t)
	if e == evUndef {
		edge := edges[*nedges*4:]
		edge[0] = s
		edge[1] = t
		edge[2] = l
		edge[3] = r
		*nedges++
		return *nedges - 1
	}
	return evUndef
}

func updateLeftFace(e []int, s, t, f int) {
	if e[0] == s && e[1] == t && e[2] == evUndef {
		e[2] = f
	} else if e[1] == s && e[0] == t && e[3] == evUndef {
		e[3] = f
	}
}

func overlapSegSeg2d(a, b, c, d [3]float32) int {
	a1 := vcross2(a, b, d)
	a2 := vcross2(a, b, c)
	if a1*a2 < 0 {
		a3 := vcross2(c, d, a)
		a4 := a3 + a2 - a1
		if a3*a4 < 0 {
			return 1
		}
	}
	return 0
}

func overlapEdges(pts []float32, edges []int, nedges, s1, t1 int) bool {
	ps1 := toV3(pts[s1*3:])
	pt1 := toV3(pts[t1*3:])

	for i := 0; i < nedges; i++ {
		s0 := edges[i*4+0]
		t0 := edges[i*4+1]
		if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
			continue
		}
		if overlapSegSeg2d(toV3(pts[s0*3:]), toV3(pts[t0*3:]), ps1, pt1) != 0 {
			return true
		}
	}
	return false
}

func completeFacet(pts []float32, npts int, edges []int, nedges *int, maxEdges int, nfaces *int, e int) {
	const eps float32 = 1e-5

	edge := edges[e*4:]

	var s, t int
	if edge[2] == evUndef {
		s = edge[0]
		t = edge[1]
	} else if edge[3] == evUndef {
		s = edge[1]
		t = edge[0]
	} else {
		return
	}

	pt := npts
	c := [3]float32{}
	r := float32(-1)
	for u := 0; u < npts; u++ {
		if u == s || u == t {
			continue
		}
		ps := toV3(pts[s*3:])
		ptv := toV3(pts[t*3:])
		pu := toV3(pts[u*3:])
		if vcross2(ps, ptv, pu) > eps {
			if r < 0 {
				pt = u
				c, r, _ = circumCircle(ps, ptv, pu)
				continue
			}
			d := vdist2(c, pu)
			const tol float32 = 0.001
			if d > r*(1+tol) {
				continue
			} else if d < r*(1-tol) {
				pt = u
				c, r, _ = circumCircle(ps, ptv, pu)
			} else {
				if overlapEdges(pts, edges, *nedges, s, u) {
					continue
				}
				if overlapEdges(pts, edges, *nedges, t, u) {
					continue
				}
				pt = u
				c, r, _ = circumCircle(ps, ptv, pu)
			}
		}
	}

	if pt < npts {
		updateLeftFace(edges[e*4:], s, t, *nfaces)

		e = findEdge(edges, *nedges, pt, s)
		if e == evUndef {
			addEdge(edges, nedges, maxEdges, pt, s, *nfaces, evUndef)
		} else {
			updateLeftFace(edges[e*4:], pt, s, *nfaces)
		}

		e = findEdge(edges, *nedges, t, pt)
		if e == evUndef {
			addEdge(edges, nedges, maxEdges, t, pt, *nfaces, evUndef)
		} else {
			updateLeftFace(edges[e*4:], t, pt, *nfaces)
		}

		*nfaces++
	} else {
		updateLeftFace(edges[e*4:], s, t, evHull)
	}
}

func delaunayHull(npts int, pts []float32, nhull int, hull []int, tris *[]int, edges *[]int) {
	nfaces := 0
	nedges := 0
	maxEdges := npts * 10
	*edges = make([]int, maxEdges*4)

	for i, j := 0, nhull-1; i < nhull; j = i {
		i++
		addEdge(*edges, &nedges, maxEdges, hull[j], hull[i], evHull, evUndef)
	}

	currentEdge := 0
	for currentEdge < nedges {
		if (*edges)[currentEdge*4+2] == evUndef {
			completeFacet(pts, npts, *edges, &nedges, maxEdges, &nfaces, currentEdge)
		}
		if (*edges)[currentEdge*4+3] == evUndef {
			completeFacet(pts, npts, *edges, &nedges, maxEdges, &nfaces, currentEdge)
		}
		currentEdge++
	}

	*tris = make([]int, nfaces*4)
	for i := 0; i < nfaces*4; i++ {
		(*tris)[i] = -1
	}

	for i := 0; i < nedges; i++ {
		e := (*edges)[i*4:]
		if e[3] >= 0 {
			t := (*tris)[e[3]*4:]
			if t[0] == -1 {
				t[0] = e[0]
				t[1] = e[1]
			} else if t[0] == e[1] {
				t[2] = e[0]
			} else if t[1] == e[0] {
				t[2] = e[1]
			}
		}
		if e[2] >= 0 {
			t := (*tris)[e[2]*4:]
			if t[0] == -1 {
				t[0] = e[1]
				t[1] = e[0]
			} else if t[0] == e[0] {
				t[2] = e[1]
			} else if t[1] == e[1] {
				t[2] = e[0]
			}
		}
	}

	for i := 0; i < len(*tris)/4; i++ {
		t := (*tris)[i*4:]
		if t[0] == -1 || t[1] == -1 || t[2] == -1 {
			t[0] = (*tris)[len(*tris)-4]
			t[1] = (*tris)[len(*tris)-3]
			t[2] = (*tris)[len(*tris)-2]
			t[3] = (*tris)[len(*tris)-1]
			*tris = (*tris)[:len(*tris)-4]
			i--
		}
	}
}

func polyMinExtent(verts []float32, nverts int) float32 {
	minDist := float32(math.MaxFloat32)
	for i := 0; i < nverts; i++ {
		ni := (i + 1) % nverts
		p1 := toV3(verts[i*3:])
		p2 := toV3(verts[ni*3:])
		maxEdgeDist := float32(0)
		for j := 0; j < nverts; j++ {
			if j == i || j == ni {
				continue
			}
			d := distancePtSeg2d(toV3(verts[j*3:]), p1, p2)
			if d > maxEdgeDist {
				maxEdgeDist = d
			}
		}
		if maxEdgeDist < minDist {
			minDist = maxEdgeDist
		}
	}
	return float32(math.Sqrt(float64(minDist)))
}

func prevIdx(i, n int) int {
	if i-1 >= 0 {
		return i - 1
	}
	return n - 1
}

func nextIdx(i, n int) int {
	if i+1 < n {
		return i + 1
	}
	return 0
}

func triangulateHull(nverts int, verts []float32, nhull int, hull []int, nin int, tris *[]int) {
	start := 0
	left := 1
	right := nhull - 1

	dmin := float32(math.MaxFloat32)
	for i := 0; i < nhull; i++ {
		if hull[i] >= nin {
			continue
		}
		pi := prevIdx(i, nhull)
		ni := nextIdx(i, nhull)
		pv := [3]float32{verts[hull[pi]*3], verts[hull[pi]*3+1], verts[hull[pi]*3+2]}
		cv := [3]float32{verts[hull[i]*3], verts[hull[i]*3+1], verts[hull[i]*3+2]}
		nv := [3]float32{verts[hull[ni]*3], verts[hull[ni]*3+1], verts[hull[ni]*3+2]}
		d := vdist2(pv, cv) + vdist2(cv, nv) + vdist2(nv, pv)
		if d < dmin {
			start = i
			left = ni
			right = pi
			dmin = d
		}
	}

	*tris = append(*tris, hull[start], hull[left], hull[right], 0)

	for nextIdx(left, nhull) != right {
		nleft := nextIdx(left, nhull)
		nright := prevIdx(right, nhull)

		cvleft := [3]float32{verts[hull[left]*3], verts[hull[left]*3+1], verts[hull[left]*3+2]}
		nvleft := [3]float32{verts[hull[nleft]*3], verts[hull[nleft]*3+1], verts[hull[nleft]*3+2]}
		cvright := [3]float32{verts[hull[right]*3], verts[hull[right]*3+1], verts[hull[right]*3+2]}
		nvright := [3]float32{verts[hull[nright]*3], verts[hull[nright]*3+1], verts[hull[nright]*3+2]}
		dleft := vdist2(cvleft, nvleft) + vdist2(nvleft, cvright)
		dright := vdist2(cvright, nvright) + vdist2(cvleft, nvright)

		if dleft < dright {
			*tris = append(*tris, hull[left], hull[nleft], hull[right], 0)
			left = nleft
		} else {
			*tris = append(*tris, hull[left], hull[nright], hull[right], 0)
			right = nright
		}
	}
}

func getJitterX(i int) float32 {
	return float32((((i * 0x8da6b343) & 0xffff) / 65535.0 * 2.0) - 1.0)
}

func getJitterY(i int) float32 {
	return float32((((i * 0xd8163841) & 0xffff) / 65535.0 * 2.0) - 1.0)
}

func onHull(a, b, nhull int, hull []int) bool {
	if a >= nhull || b >= nhull {
		return false
	}

	for j, i := nhull-1, 0; i < nhull; j = i {
		i++
		if a == hull[j] && b == hull[i] {
			return true
		}
	}
	return false
}

func setTriFlags(tris *[]int, nhull int, hull []int) {
	const detailEdgeBoundary = 0x1

	for i := 0; i < len(*tris); i += 4 {
		a := (*tris)[i+0]
		b := (*tris)[i+1]
		c := (*tris)[i+2]
		var flags uint16
		if onHull(a, b, nhull, hull) {
			flags |= detailEdgeBoundary << 0
		}
		if onHull(b, c, nhull, hull) {
			flags |= detailEdgeBoundary << 2
		}
		if onHull(c, a, nhull, hull) {
			flags |= detailEdgeBoundary << 4
		}
		(*tris)[i+3] = int(flags)
	}
}

// ---- Contour Building Functions (from RecastContour.cpp) ----

func getCornerHeight(x, y, i, dir int, chf *CompactHeightfield) (int, bool) {
	s := chf.Spans[i]
	ch := int(s.Y)
	dirp := (dir + 1) & 0x3

	var regs [4]uint32
	regs[0] = uint32(chf.Spans[i].Reg) | (uint32(chf.Areas[i]) << 16)

	if Con(&s, dir) != notConnected {
		ax := x + DirOffsetX(dir)
		ay := y + DirOffsetZ(dir)
		ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&s, dir)
		as := chf.Spans[ai]
		ch = max(ch, int(as.Y))
		regs[1] = uint32(chf.Spans[ai].Reg) | (uint32(chf.Areas[ai]) << 16)
		if Con(&as, dirp) != notConnected {
			ax2 := ax + DirOffsetX(dirp)
			ay2 := ay + DirOffsetZ(dirp)
			ai2 := int(chf.Cells[ax2+ay2*chf.Width].Index) + Con(&as, dirp)
			as2 := chf.Spans[ai2]
			ch = max(ch, int(as2.Y))
			regs[2] = uint32(chf.Spans[ai2].Reg) | (uint32(chf.Areas[ai2]) << 16)
		}
	}
	if Con(&s, dirp) != notConnected {
		ax := x + DirOffsetX(dirp)
		ay := y + DirOffsetZ(dirp)
		ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&s, dirp)
		as := chf.Spans[ai]
		ch = max(ch, int(as.Y))
		regs[3] = uint32(chf.Spans[ai].Reg) | (uint32(chf.Areas[ai]) << 16)
		if Con(&as, dir) != notConnected {
			ax2 := ax + DirOffsetX(dir)
			ay2 := ay + DirOffsetZ(dir)
			ai2 := int(chf.Cells[ax2+ay2*chf.Width].Index) + Con(&as, dir)
			as2 := chf.Spans[ai2]
			ch = max(ch, int(as2.Y))
			regs[2] = uint32(chf.Spans[ai2].Reg) | (uint32(chf.Areas[ai2]) << 16)
		}
	}
		isBorderVertex := false

	for j := 0; j < 4; j++ {
		a := j
		b := (j + 1) & 0x3
		c := (j + 2) & 0x3
		d := (j + 3) & 0x3

		twoSameExts := (regs[a]&uint32(borderReg) != 0) && (regs[b]&uint32(borderReg) != 0) && regs[a] == regs[b]
		twoInts := (regs[c]|regs[d])&uint32(borderReg) == 0
		intsSameArea := (regs[c] >> 16) == (regs[d] >> 16)
		noZeros := regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0
		if twoSameExts && twoInts && intsSameArea && noZeros {
			isBorderVertex = true
			break
		}
	}

	return ch, isBorderVertex
}

func walkContour(x, y, i int, chf *CompactHeightfield, flags []uint8, points []int) []int {
	var dir uint8 = 0
	for (flags[i] & (1 << dir)) == 0 {
		dir++
	}

	startDir := dir
	starti := i

	area := chf.Areas[i]

	iter := 0
	for iter < 40000 {
		iter++
		if flags[i]&(1<<dir) != 0 {
			isBorderVertex := false
			isAreaBorder := false
			px := x
			var py int; py, isBorderVertex = getCornerHeight(x, y, i, int(dir), chf)
			pz := y
			switch dir {
			case 0:
				pz++
			case 1:
				px++
				pz++
			case 2:
				px++
			}

			r := 0
			s := chf.Spans[i]
			if Con(&s, int(dir)) != notConnected {
				ax := x + DirOffsetX(int(dir))
				ay := y + DirOffsetZ(int(dir))
				ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&s, int(dir))
				r = int(chf.Spans[ai].Reg)
				if area != chf.Areas[ai] {
					isAreaBorder = true
				}
			}
			if isBorderVertex {
				r |= borderVertex
			}
			if isAreaBorder {
				r |= areaBorder
			}
			points = append(points, px, py, pz, r)

			flags[i] &= ^(1 << dir)
			dir = (dir + 1) & 0x3
		} else {
			ni := -1
			nx := x + DirOffsetX(int(dir))
			ny := y + DirOffsetZ(int(dir))
			s := chf.Spans[i]
			if Con(&s, int(dir)) != notConnected {
				nc := chf.Cells[nx+ny*chf.Width]
				ni = int(nc.Index) + Con(&s, int(dir))
			}
			if ni == -1 {
				return points
			}
			x = nx
			y = ny
			i = ni
			dir = (dir + 3) & 0x3
		}

		if starti == i && startDir == dir {
			break
		}
	}
	return points
}

func distancePtSegInt(x, z, px, pz, qx, qz int) float32 {
	pqx := float32(qx - px)
	pqz := float32(qz - pz)
	dx := float32(x - px)
	dz := float32(z - pz)
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

	dx = float32(px) + t*pqx - float32(x)
	dz = float32(pz) + t*pqz - float32(z)

	return dx*dx + dz*dz
}

func simplifyContour(points []int, maxError float32, maxEdgeLen int, buildFlags int) []int {
	simplified := make([]int, 0, 64)
	hasConnections := false
	for i := 0; i < len(points); i += 4 {
		if (points[i+3] & contourRegMask) != 0 {
			hasConnections = true
			break
		}
	}

	if hasConnections {
		for i := 0; i < len(points)/4; i++ {
			ii := (i + 1) % (len(points) / 4)
			differentRegs := (points[i*4+3] & contourRegMask) != (points[ii*4+3] & contourRegMask)
			areaBorders := (points[i*4+3] & areaBorder) != (points[ii*4+3] & areaBorder)
			if differentRegs || areaBorders {
				simplified = append(simplified, points[i*4+0], points[i*4+1], points[i*4+2], i)
			}
		}
	}

	if len(simplified) == 0 {
		llx := points[0]
		lly := points[1]
		llz := points[2]
		lli := 0
		urx := points[0]
		ury := points[1]
		urz := points[2]
		uri := 0
		for i := 0; i < len(points); i += 4 {
			x := points[i+0]
			y := points[i+1]
			z := points[i+2]
			if x < llx || (x == llx && z < llz) {
				llx = x
				lly = y
				llz = z
				lli = i / 4
			}
			if x > urx || (x == urx && z > urz) {
				urx = x
				ury = y
				urz = z
				uri = i / 4
			}
		}
		simplified = append(simplified, llx, lly, llz, lli)
		simplified = append(simplified, urx, ury, urz, uri)
	}

	pn := len(points) / 4
	for i := 0; i < len(simplified)/4; {
		ii := (i + 1) % (len(simplified) / 4)

		ax := simplified[i*4+0]
		az := simplified[i*4+2]
		ai := simplified[i*4+3]

		bx := simplified[ii*4+0]
		bz := simplified[ii*4+2]
		bi := simplified[ii*4+3]

		maxd := float32(0)
		maxi := -1
		var ci, cinc, endi int

		if bx > ax || (bx == ax && bz > az) {
			cinc = 1
			ci = (ai + cinc) % pn
			endi = bi
		} else {
			cinc = pn - 1
			ci = (bi + cinc) % pn
			endi = ai
			ax, bx = bx, ax
			az, bz = bz, az
		}

		if (points[ci*4+3]&contourRegMask) == 0 ||
			(points[ci*4+3]&areaBorder) != 0 {
			for ci != endi {
				d := distancePtSegInt(points[ci*4+0], points[ci*4+2], ax, az, bx, bz)
				if d > maxd {
					maxd = d
					maxi = ci
				}
				ci = (ci + cinc) % pn
			}
		}

		if maxi != -1 && maxd > (maxError*maxError) {
			newVert := []int{points[maxi*4+0], points[maxi*4+1], points[maxi*4+2], maxi}
			simplified = append(simplified[:(i+1)*4], append(newVert, simplified[(i+1)*4:]...)...)
		} else {
			i++
		}
	}

	if maxEdgeLen > 0 && (buildFlags&int(contourTessWallEdges|contourTessAreaEdges)) != 0 {
		for i := 0; i < len(simplified)/4; {
			ii := (i + 1) % (len(simplified) / 4)

			ax := simplified[i*4+0]
			az := simplified[i*4+2]
			ai := simplified[i*4+3]

			bx := simplified[ii*4+0]
			bz := simplified[ii*4+2]
			bi := simplified[ii*4+3]

			maxi := -1
			ci := (ai + 1) % pn

			tess := false
			if (buildFlags&int(contourTessWallEdges)) != 0 && (points[ci*4+3]&contourRegMask) == 0 {
				tess = true
			}
			if (buildFlags&int(contourTessAreaEdges)) != 0 && (points[ci*4+3]&areaBorder) != 0 {
				tess = true
			}

			if tess {
				dx := bx - ax
				dz := bz - az
				if dx*dx+dz*dz > maxEdgeLen*maxEdgeLen {
					n := 0
					if bi < ai {
						n = (bi + pn - ai)
					} else {
						n = (bi - ai)
					}
					if n > 1 {
						if bx > ax || (bx == ax && bz > az) {
							maxi = (ai + n/2) % pn
						} else {
							maxi = (ai + (n+1)/2) % pn
						}
					}
				}
			}

			if maxi != -1 {
				newVert := []int{points[maxi*4+0], points[maxi*4+1], points[maxi*4+2], maxi}
				simplified = append(simplified[:(i+1)*4], append(newVert, simplified[(i+1)*4:]...)...)
			} else {
				i++
			}
		}
	}

	for i := 0; i < len(simplified)/4; i++ {
		ai := (simplified[i*4+3] + 1) % pn
		bi := simplified[i*4+3]
		simplified[i*4+3] = (points[ai*4+3] & (contourRegMask | areaBorder)) | (points[bi*4+3] & borderVertex)
	}

	return simplified
}

func calcAreaOfPolygon2D(verts []int, nverts int) int {
	area := 0
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vi := verts[i*4:]
		vj := verts[j*4:]
		area += vi[0]*vj[2] - vj[0]*vi[2]
	}
	return (area + 1) / 2
}

func prev(i, n int) int {
	if i-1 >= 0 {
		return i - 1
	}
	return n - 1
}

func next(i, n int) int {
	if i+1 < n {
		return i + 1
	}
	return 0
}

func intersectSegContour(d0, d1 []int, i, n int, verts []int) bool {
	for k := 0; k < n; k++ {
		k1 := next(k, n)
		if i == k || i == k1 {
			continue
		}
		p0 := verts[k*4:]
		p1 := verts[k1*4:]
		if vequalInt(d0, p0) || vequalInt(d1, p0) || vequalInt(d0, p1) || vequalInt(d1, p1) {
			continue
		}
		if intersectInt(d0, d1, p0, p1) {
			return true
		}
	}
	return false
}

func inConeIntContour(i, n int, verts []int, pj []int) bool {
	pi := verts[i*4:]
	pi1 := verts[next(i, n)*4:]
	pin1 := verts[prev(i, n)*4:]

	if leftOnInt(pin1, pi, pi1) {
		return leftInt(pi, pj, pin1) && leftInt(pj, pi, pi1)
	}
	return !(leftOnInt(pi, pj, pi1) && leftOnInt(pj, pi, pin1))
}

func removeDegenerateSegments(simplified []int) []int {
	npts := len(simplified) / 4
	for i := 0; i < npts; i++ {
		ni := next(i, npts)

		if vequalInt(simplified[i*4:], simplified[ni*4:]) {
			for j := i; j < len(simplified)/4-1; j++ {
				simplified[j*4+0] = simplified[(j+1)*4+0]
				simplified[j*4+1] = simplified[(j+1)*4+1]
				simplified[j*4+2] = simplified[(j+1)*4+2]
				simplified[j*4+3] = simplified[(j+1)*4+3]
			}
			simplified = simplified[:len(simplified)-4]
			npts--
		}
	}
	return simplified
}

func mergeContours(ca, cb *Contour, ia, ib int) bool {
	maxVerts := ca.Nverts + cb.Nverts + 2
	verts := make([]int, maxVerts*4)

	nv := 0

	for i := 0; i <= ca.Nverts; i++ {
		src := ca.Verts[((ia+i)%ca.Nverts)*4:]
		dst := verts[nv*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		nv++
	}

	for i := 0; i <= cb.Nverts; i++ {
		src := cb.Verts[((ib+i)%cb.Nverts)*4:]
		dst := verts[nv*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		nv++
	}

	ca.Verts = verts
	ca.Nverts = nv

	cb.Verts = nil
	cb.Nverts = 0

	return true
}

func findLeftMostVertex(contour *Contour) (minx, minz, leftmost int) {
	minx = contour.Verts[0]
	minz = contour.Verts[2]
	leftmost = 0
	for i := 1; i < contour.Nverts; i++ {
		x := contour.Verts[i*4+0]
		z := contour.Verts[i*4+2]
		if x < minx || (x == minx && z < minz) {
			minx = x
			minz = z
			leftmost = i
		}
	}
	return
}

func compareHolesByMinX(holes []ContourHole) {
	sort.Slice(holes, func(i, j int) bool {
		if holes[i].minx == holes[j].minx {
			return holes[i].minz < holes[j].minz
		}
		return holes[i].minx < holes[j].minx
	})
}

func compareDiagDistByDist(diags []PotentialDiagonal) {
	sort.Slice(diags, func(i, j int) bool {
		return diags[i].dist < diags[j].dist
	})
}

func mergeRegionHoles(ctx *Context, region *ContourRegion) {
	for i := 0; i < region.nholes; i++ {
		mx, mz, lm := findLeftMostVertex(region.holes[i].contour)
		region.holes[i].minx = mx
		region.holes[i].minz = mz
		region.holes[i].leftmost = lm
	}

	compareHolesByMinX(region.holes[:region.nholes])

	maxVerts := region.outline.Nverts
	for i := 0; i < region.nholes; i++ {
		maxVerts += region.holes[i].contour.Nverts
	}

	diags := make([]PotentialDiagonal, maxVerts)
	outline := region.outline

	for i := 0; i < region.nholes; i++ {
		hole := region.holes[i].contour

		index := -1
		bestVertex := region.holes[i].leftmost
		for iter := 0; iter < hole.Nverts; iter++ {
			ndiags := 0
			corner := hole.Verts[bestVertex*4:]
			for j := 0; j < outline.Nverts; j++ {
				if inConeIntContour(j, outline.Nverts, outline.Verts, corner) {
					dx := outline.Verts[j*4+0] - corner[0]
					dz := outline.Verts[j*4+2] - corner[2]
					diags[ndiags].vert = j
					diags[ndiags].dist = dx*dx + dz*dz
					ndiags++
				}
			}
			compareDiagDistByDist(diags[:ndiags])

			index = -1
			for j := 0; j < ndiags; j++ {
				pt := outline.Verts[diags[j].vert*4:]
				intersect := intersectSegContour(pt, corner, diags[j].vert, outline.Nverts, outline.Verts)
				for k := i; k < region.nholes && !intersect; k++ {
					intersect = intersect || intersectSegContour(pt, corner, -1, region.holes[k].contour.Nverts, region.holes[k].contour.Verts)
				}
				if !intersect {
					index = diags[j].vert
					break
				}
			}
			if index != -1 {
				break
			}
			bestVertex = (bestVertex + 1) % hole.Nverts
		}

		if index == -1 {
			ctx.Log(LogWarning, "mergeHoles: Failed to find merge points for outline and hole.")
			continue
		}
		if !mergeContours(outline, hole, index, bestVertex) {
			ctx.Log(LogWarning, "mergeHoles: Failed to merge contours.")
			continue
		}
	}
}

// BuildContours builds and returns a contour set from a compact heightfield.
func BuildContours(ctx *Context, chf *CompactHeightfield, maxError float32, maxEdgeLen int, buildFlags int) *ContourSet {
	w := chf.Width
	h := chf.Height
	borderSize := chf.BorderSize

	defer ctx.ScopedTimer(TimerBuildContours)()

	cset := &ContourSet{}
	cset.Bmin = chf.Bmin
	cset.Bmax = chf.Bmax
	if borderSize > 0 {
		pad := float32(borderSize) * chf.Cs
		cset.Bmin[0] += pad
		cset.Bmin[2] += pad
		cset.Bmax[0] -= pad
		cset.Bmax[2] -= pad
	}
	cset.Cs = chf.Cs
	cset.Ch = chf.Ch
	cset.Width = chf.Width - chf.BorderSize*2
	cset.Height = chf.Height - chf.BorderSize*2
	cset.BorderSize = chf.BorderSize
	cset.MaxError = maxError

	maxContours := max(int(chf.MaxRegions), 8)
	cset.Conts = make([]Contour, maxContours)
	cset.Nconts = 0

	flags := make([]uint8, chf.SpanCount)

	ctx.StartTimer(TimerBuildContoursTrace)

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				var res uint8 = 0
				s := chf.Spans[i]
				if chf.Spans[i].Reg == 0 || (chf.Spans[i].Reg&borderReg) != 0 {
					flags[i] = 0
					continue
				}
				for dir := 0; dir < 4; dir++ {
					r := uint16(0)
					if Con(&s, dir) != notConnected {
						ax := x + DirOffsetX(dir)
						ay := y + DirOffsetZ(dir)
						ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
						r = chf.Spans[ai].Reg
					}
					if r == chf.Spans[i].Reg {
						res |= (1 << dir)
					}
				}
				flags[i] = res ^ 0xf
			}
		}
	}

	ctx.StopTimer(TimerBuildContoursTrace)

	verts := make([]int, 0, 256)
	simplified := make([]int, 0, 64)

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				if flags[i] == 0 || flags[i] == 0xf {
					flags[i] = 0
					continue
				}
				reg := chf.Spans[i].Reg
				if reg == 0 || (reg&borderReg) != 0 {
					continue
				}
				area := chf.Areas[i]

				verts = verts[:0]
				simplified = simplified[:0]

				ctx.StartTimer(TimerBuildContoursTrace)
				verts = walkContour(x, y, i, chf, flags, verts)
				ctx.StopTimer(TimerBuildContoursTrace)

				ctx.StartTimer(TimerBuildContoursSimplify)
				simplified = simplifyContour(verts, maxError, maxEdgeLen, buildFlags)
				simplified = removeDegenerateSegments(simplified)
				ctx.StopTimer(TimerBuildContoursSimplify)

				if len(simplified)/4 >= 3 {
					if cset.Nconts >= maxContours {
						oldMax := maxContours
						maxContours *= 2
						newConts := make([]Contour, maxContours)
						copy(newConts, cset.Conts[:cset.Nconts])
						cset.Conts = newConts
						ctx.Log(LogWarning, "BuildContours: Expanding max contours from %d to %d.", oldMax, maxContours)
					}

					cont := &cset.Conts[cset.Nconts]
					cset.Nconts++

					cont.Nverts = len(simplified) / 4
					cont.Verts = make([]int, cont.Nverts*4)
					copy(cont.Verts, simplified)

					if borderSize > 0 {
						for j := 0; j < cont.Nverts; j++ {
							v := cont.Verts[j*4:]
							v[0] -= borderSize
							v[2] -= borderSize
						}
					}

					cont.Nrvets = len(verts) / 4
					cont.RVerts = make([]int, cont.Nrvets*4)
					copy(cont.RVerts, verts)
					if borderSize > 0 {
						for j := 0; j < cont.Nrvets; j++ {
							v := cont.RVerts[j*4:]
							v[0] -= borderSize
							v[2] -= borderSize
						}
					}

					cont.Reg = reg
					cont.Area = area
				}
			}
		}
	}

	if cset.Nconts > 0 {
		winding := make([]int8, cset.Nconts)
		nholes := 0
		for i := 0; i < cset.Nconts; i++ {
			cont := &cset.Conts[i]
			if calcAreaOfPolygon2D(cont.Verts, cont.Nverts) < 0 {
				winding[i] = -1
				nholes++
			} else {
				winding[i] = 1
			}
		}

		if nholes > 0 {
			nregions := int(chf.MaxRegions) + 1
			regions := make([]ContourRegion, nregions)
			holes := make([]ContourHole, cset.Nconts)

			for i := 0; i < cset.Nconts; i++ {
				cont := cset.Conts[i]
				if winding[i] > 0 {
					if regions[cont.Reg].outline != nil {
						ctx.Log(LogError, "BuildContours: Multiple outlines for region %d.", cont.Reg)
					}
					regions[cont.Reg].outline = &cset.Conts[i]
				} else {
					regions[cont.Reg].nholes++
				}
			}

			index := 0
			for i := 0; i < nregions; i++ {
				if regions[i].nholes > 0 {
					regions[i].holes = holes[index:]
					index += regions[i].nholes
					regions[i].nholes = 0
				}
			}

			for i := 0; i < cset.Nconts; i++ {
				cont := cset.Conts[i]
				if winding[i] < 0 {
					regions[cont.Reg].holes[regions[cont.Reg].nholes].contour = &cset.Conts[i]
					regions[cont.Reg].nholes++
				}
			}

			for i := 0; i < nregions; i++ {
				reg := &regions[i]
				if reg.nholes == 0 {
					continue
				}
				if reg.outline != nil {
					mergeRegionHoles(ctx, reg)
				} else {
					ctx.Log(LogError, "BuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i)
				}
			}
		}
	}

	return cset
}
