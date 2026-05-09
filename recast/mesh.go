package recast

import (
	"math"
)

// Edge represents an edge in the polygon mesh building.
type Edge struct {
	vert     [2]uint16
	polyEdge [2]uint16
	poly     [2]uint16
}

const vertexBucketCount = 1 << 12

func computeVertexHash(x, y, z int) int {
	const h1 uint32 = 0x8da6b343
	const h2 uint32 = 0xd8163841
	const h3 uint32 = 0xcb1ab31f
	n := h1*uint32(x) + h2*uint32(y) + h3*uint32(z)
	return int(n & (vertexBucketCount - 1))
}

func addVertex(x, y, z uint16, verts []uint16, firstVert []int, nextVert []int, nv *int) uint16 {
	bucket := computeVertexHash(int(x), 0, int(z))
	i := firstVert[bucket]

	for i != -1 {
		v := verts[i*3:]
		if v[0] == x && Abs(int(v[1])-int(y)) <= 2 && v[2] == z {
			return uint16(i)
		}
		i = nextVert[i]
	}

	// Could not find, create new.
	i = *nv
	*nv++
	v := verts[i*3:]
	v[0] = x
	v[1] = y
	v[2] = z
	nextVert[i] = firstVert[bucket]
	firstVert[bucket] = i

	return uint16(i)
}

func area2Int(a, b, c []int) int {
	return (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
}

func leftInt(a, b, c []int) bool {
	return area2Int(a, b, c) < 0
}

func leftOnInt(a, b, c []int) bool {
	return area2Int(a, b, c) <= 0
}

func collinearInt(a, b, c []int) bool {
	return area2Int(a, b, c) == 0
}

func intersectPropInt(a, b, c, d []int) bool {
	if collinearInt(a, b, c) || collinearInt(a, b, d) ||
		collinearInt(c, d, a) || collinearInt(c, d, b) {
		return false
	}

	return (leftInt(a, b, c) != leftInt(a, b, d)) && (leftInt(c, d, a) != leftInt(c, d, b))
}

func betweenInt(a, b, c []int) bool {
	if !collinearInt(a, b, c) {
		return false
	}
	if a[0] != b[0] {
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]))
	}
	return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]))
}

func intersectInt(a, b, c, d []int) bool {
	if intersectPropInt(a, b, c, d) {
		return true
	}
	if betweenInt(a, b, c) || betweenInt(a, b, d) ||
		betweenInt(c, d, a) || betweenInt(c, d, b) {
		return true
	}
	return false
}

func vequalInt(a, b []int) bool {
	return a[0] == b[0] && a[2] == b[2]
}

func diagonalie(i, j, n int, verts []int, indices []int) bool {
	d0 := verts[(indices[i]&0x0fffffff)*4:]
	d1 := verts[(indices[j]&0x0fffffff)*4:]

	for k := 0; k < n; k++ {
		k1 := nextIdx(k, n)
		if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
			p0 := verts[(indices[k]&0x0fffffff)*4:]
			p1 := verts[(indices[k1]&0x0fffffff)*4:]

			if vequalInt(d0, p0) || vequalInt(d1, p0) || vequalInt(d0, p1) || vequalInt(d1, p1) {
				continue
			}

			if intersectInt(d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}

func inConeInt(i, j, n int, verts []int, indices []int) bool {
	pi := verts[(indices[i]&0x0fffffff)*4:]
	pj := verts[(indices[j]&0x0fffffff)*4:]
	pi1 := verts[(indices[nextIdx(i, n)]&0x0fffffff)*4:]
	pin1 := verts[(indices[prevIdx(i, n)]&0x0fffffff)*4:]

	if leftOnInt(pin1, pi, pi1) {
		return leftInt(pi, pj, pin1) && leftInt(pj, pi, pi1)
	}
	return !(leftOnInt(pi, pj, pi1) && leftOnInt(pj, pi, pin1))
}

func diagonalInt(i, j, n int, verts []int, indices []int) bool {
	return inConeInt(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices)
}

func diagonalieLoose(i, j, n int, verts []int, indices []int) bool {
	d0 := verts[(indices[i]&0x0fffffff)*4:]
	d1 := verts[(indices[j]&0x0fffffff)*4:]

	for k := 0; k < n; k++ {
		k1 := nextIdx(k, n)
		if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
			p0 := verts[(indices[k]&0x0fffffff)*4:]
			p1 := verts[(indices[k1]&0x0fffffff)*4:]

			if vequalInt(d0, p0) || vequalInt(d1, p0) || vequalInt(d0, p1) || vequalInt(d1, p1) {
				continue
			}

			if intersectPropInt(d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}

func inConeLoose(i, j, n int, verts []int, indices []int) bool {
	pi := verts[(indices[i]&0x0fffffff)*4:]
	pj := verts[(indices[j]&0x0fffffff)*4:]
	pi1 := verts[(indices[nextIdx(i, n)]&0x0fffffff)*4:]
	pin1 := verts[(indices[prevIdx(i, n)]&0x0fffffff)*4:]

	if leftOnInt(pin1, pi, pi1) {
		return leftOnInt(pi, pj, pin1) && leftOnInt(pj, pi, pi1)
	}
	return !(leftOnInt(pi, pj, pi1) && leftOnInt(pj, pi, pin1))
}

func diagonalLoose(i, j, n int, verts []int, indices []int) bool {
	return inConeLoose(i, j, n, verts, indices) && diagonalieLoose(i, j, n, verts, indices)
}

func triangulate(n int, verts []int, indices []int, tris []int) int {
	ntris := 0
	dstIdx := 0

	// The last bit of the index is used to indicate if the vertex can be removed.
	for i := 0; i < n; i++ {
		i1 := nextIdx(i, n)
		i2 := nextIdx(i1, n)
		if diagonalInt(i, i2, n, verts, indices) {
			indices[i1] |= 0x80000000
		}
	}

	for n > 3 {
		minLen := -1
		mini := -1
		for i := 0; i < n; i++ {
			i1 := nextIdx(i, n)
			if indices[i1]&0x80000000 != 0 {
				p0 := verts[(indices[i]&0x0fffffff)*4:]
				p2 := verts[(indices[nextIdx(i1, n)]&0x0fffffff)*4:]

				dx := p2[0] - p0[0]
				dy := p2[2] - p0[2]
				len := dx*dx + dy*dy

				if minLen < 0 || len < minLen {
					minLen = len
					mini = i
				}
			}
		}

		if mini == -1 {
			// We might get here because the contour has overlapping segments.
			// Try to recover by loosening the inCone test.
			minLen = -1
			mini = -1
			for i := 0; i < n; i++ {
				i1 := nextIdx(i, n)
				i2 := nextIdx(i1, n)
				if diagonalLoose(i, i2, n, verts, indices) {
					p0 := verts[(indices[i]&0x0fffffff)*4:]
					p2 := verts[(indices[nextIdx(i2, n)]&0x0fffffff)*4:]
					dx := p2[0] - p0[0]
					dy := p2[2] - p0[2]
					len := dx*dx + dy*dy

					if minLen < 0 || len < minLen {
						minLen = len
						mini = i
					}
				}
			}
			if mini == -1 {
				// The contour is messed up.
				return -ntris
			}
		}

		i := mini
		i1 := nextIdx(i, n)
		i2 := nextIdx(i1, n)

		tris[dstIdx] = indices[i] & 0x0fffffff
		dstIdx++
		tris[dstIdx] = indices[i1] & 0x0fffffff
		dstIdx++
		tris[dstIdx] = indices[i2] & 0x0fffffff
		dstIdx++
		ntris++

		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--
		for k := i1; k < n; k++ {
			indices[k] = indices[k+1]
		}

		if i1 >= n {
			i1 = 0
		}
		i = prevIdx(i1, n)
		// Update diagonal flags.
		if diagonalInt(prevIdx(i, n), i1, n, verts, indices) {
			indices[i] |= 0x80000000
		} else {
			indices[i] &= 0x0fffffff
		}

		if diagonalInt(i, nextIdx(i1, n), n, verts, indices) {
			indices[i1] |= 0x80000000
		} else {
			indices[i1] &= 0x0fffffff
		}
	}

	// Append the remaining triangle.
	tris[dstIdx] = indices[0] & 0x0fffffff
	dstIdx++
	tris[dstIdx] = indices[1] & 0x0fffffff
	dstIdx++
	tris[dstIdx] = indices[2] & 0x0fffffff
	dstIdx++
	ntris++

	return ntris
}

func countPolyVerts(p []uint16, nvp int) int {
	for i := 0; i < nvp; i++ {
		if p[i] == meshNullIdx {
			return i
		}
	}
	return nvp
}

func uleftCorrect(a, b, c []uint16) bool {
	return (int(b[0])-int(a[0]))*(int(c[2])-int(a[2]))-
		(int(c[0])-int(a[0]))*(int(b[2])-int(a[2])) < 0
}

func getPolyMergeValue(pa, pb []uint16, verts []uint16, ea, eb *int, nvp int) int {
	na := countPolyVerts(pa, nvp)
	nb := countPolyVerts(pb, nvp)

	// If the merged polygon would be too big, do not merge.
	if na+nb-2 > nvp {
		return -1
	}

	// Check if the polygons share an edge.
	*ea = -1
	*eb = -1

	for i := 0; i < na; i++ {
		va0 := pa[i]
		va1 := pa[(i+1)%na]
		if va0 > va1 {
			va0, va1 = va1, va0
		}
		for j := 0; j < nb; j++ {
			vb0 := pb[j]
			vb1 := pb[(j+1)%nb]
			if vb0 > vb1 {
				vb0, vb1 = vb1, vb0
			}
			if va0 == vb0 && va1 == vb1 {
				*ea = i
				*eb = j
				break
			}
		}
	}

	// No common edge, cannot merge.
	if *ea == -1 || *eb == -1 {
		return -1
	}

	// Check to see if the merged polygon would be convex.
	var va, vb, vc uint16

	va = pa[(*ea+na-1)%na]
	vb = pa[*ea]
	vc = pb[(*eb+2)%nb]
	if !uleftCorrect(verts[va*3:va*3+3], verts[vb*3:vb*3+3], verts[vc*3:vc*3+3]) {
		return -1
	}

	va = pb[(*eb+nb-1)%nb]
	vb = pb[*eb]
	vc = pa[(*ea+2)%na]
	if !uleftCorrect(verts[va*3:va*3+3], verts[vb*3:vb*3+3], verts[vc*3:vc*3+3]) {
		return -1
	}

	va = pa[*ea]
	vb = pa[(*ea+1)%na]

	dx := int(verts[va*3+0]) - int(verts[vb*3+0])
	dy := int(verts[va*3+2]) - int(verts[vb*3+2])

	return dx*dx + dy*dy
}

func mergePolyVerts(pa, pb []uint16, ea, eb int, tmp []uint16, nvp int) {
	na := countPolyVerts(pa, nvp)
	nb := countPolyVerts(pb, nvp)

	// Merge polygons.
	for i := 0; i < nvp; i++ {
		tmp[i] = meshNullIdx
	}
	n := 0
	// Add pa
	for i := 0; i < na-1; i++ {
		tmp[n] = pa[(ea+1+i)%na]
		n++
	}
	// Add pb
	for i := 0; i < nb-1; i++ {
		tmp[n] = pb[(eb+1+i)%nb]
		n++
	}

	copy(pa, tmp[:nvp])
}

func pushFront(v int, arr *[]int, an *int) {
	*arr = append(*arr, 0)
	for i := *an; i > 0; i-- {
		(*arr)[i] = (*arr)[i-1]
	}
	(*arr)[0] = v
	*an++
}

func pushBack(v int, arr *[]int, an *int) {
	(*arr)[*an] = v
	*an++
}

func buildMeshAdjacency(polys []uint16, npolys, nverts, vertsPerPoly int) bool {
	// Based on code by Eric Lengyel from:
	// https://web.archive.org/web/20080704083314/http://www.terathon.com/code/edges.php

	maxEdgeCount := npolys * vertsPerPoly
	firstEdge := make([]uint16, nverts+maxEdgeCount)
	nextEdge := firstEdge[nverts:]
	edgeCount := 0

	edges := make([]Edge, maxEdgeCount)

	for i := 0; i < nverts; i++ {
		firstEdge[i] = meshNullIdx
	}

	for i := 0; i < npolys; i++ {
		t := polys[i*vertsPerPoly*2:]
		for j := 0; j < vertsPerPoly; j++ {
			if t[j] == meshNullIdx {
				break
			}
			v0 := t[j]
			var v1 uint16
			if j+1 >= vertsPerPoly || t[j+1] == meshNullIdx {
				v1 = t[0]
			} else {
				v1 = t[j+1]
			}
			if v0 < v1 {
				edge := &edges[edgeCount]
				edge.vert[0] = v0
				edge.vert[1] = v1
				edge.poly[0] = uint16(i)
				edge.polyEdge[0] = uint16(j)
				edge.poly[1] = uint16(i)
				edge.polyEdge[1] = 0
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0]
				firstEdge[v0] = uint16(edgeCount)
				edgeCount++
			}
		}
	}

	for i := 0; i < npolys; i++ {
		t := polys[i*vertsPerPoly*2:]
		for j := 0; j < vertsPerPoly; j++ {
			if t[j] == meshNullIdx {
				break
			}
			v0 := t[j]
			var v1 uint16
			if j+1 >= vertsPerPoly || t[j+1] == meshNullIdx {
				v1 = t[0]
			} else {
				v1 = t[j+1]
			}
			if v0 > v1 {
				for e := firstEdge[v1]; e != meshNullIdx; e = nextEdge[e] {
					edge := &edges[e]
					if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
						edge.poly[1] = uint16(i)
						edge.polyEdge[1] = uint16(j)
						break
					}
				}
			}
		}
	}

	// Store adjacency
	for i := 0; i < edgeCount; i++ {
		e := edges[i]
		if e.poly[0] != e.poly[1] {
			p0 := polys[int(e.poly[0])*vertsPerPoly*2:]
			p1 := polys[int(e.poly[1])*vertsPerPoly*2:]
			p0[vertsPerPoly+int(e.polyEdge[0])] = e.poly[1]
			p1[vertsPerPoly+int(e.polyEdge[1])] = e.poly[0]
		}
	}

	return true
}

func canRemoveVertex(mesh *PolyMesh, rem uint16) bool {
	nvp := mesh.Nvp

	// Count number of polygons to remove.
	numTouchedVerts := 0
	numRemainingEdges := 0
	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		numRemoved := 0
		numVerts := 0
		for j := 0; j < nv; j++ {
			if p[j] == rem {
				numTouchedVerts++
				numRemoved++
			}
			numVerts++
		}
		if numRemoved > 0 {
			numRemainingEdges += numVerts - (numRemoved + 1)
		}
	}

	// There would be too few edges remaining to create a polygon.
	if numRemainingEdges <= 2 {
		return false
	}

	// Find edges which share the removed vertex.
	maxEdges := numTouchedVerts * 2
	nedges := 0
	edges := make([]int, maxEdges*3)

	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)

		// Collect edges which touches the removed vertex.
		for j, k := 0, nv-1; j < nv; k = j {
			j++
			if p[j] == rem || p[k] == rem {
				// Arrange edge so that a=rem.
				a := int(p[j])
				b := int(p[k])
				if b == int(rem) {
					a, b = b, a
				}

				// Check if the edge exists
				exists := false
				for m := 0; m < nedges; m++ {
					e := edges[m*3:]
					if e[1] == b {
						// Exists, increment vertex share count.
						e[2]++
						exists = true
					}
				}
				// Add new edge.
				if !exists {
					e := edges[nedges*3:]
					e[0] = a
					e[1] = b
					e[2] = 1
					nedges++
				}
			}
		}
	}

	// There should be no more than 2 open edges.
	numOpenEdges := 0
	for i := 0; i < nedges; i++ {
		if edges[i*3+2] < 2 {
			numOpenEdges++
		}
	}
	if numOpenEdges > 2 {
		return false
	}

	return true
}

func removeVertex(mesh *PolyMesh, rem uint16, maxTris int) bool {
	nvp := mesh.Nvp

	// Count number of polygons to remove.
	numRemovedVerts := 0
	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		for j := 0; j < nv; j++ {
			if p[j] == rem {
				numRemovedVerts++
			}
		}
	}

	nedges := 0
	edges := make([]int, numRemovedVerts*nvp*4)

	nhole := 0
	hole := make([]int, numRemovedVerts*nvp)

	nhreg := 0
	hreg := make([]int, numRemovedVerts*nvp)

	nharea := 0
	harea := make([]int, numRemovedVerts*nvp)

	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		hasRem := false
		for j := 0; j < nv; j++ {
			if p[j] == rem {
				hasRem = true
				break
			}
		}
		if hasRem {
			// Collect edges which does not touch the removed vertex.
			for j, k := 0, nv-1; j < nv; k = j {
				j++
				if p[j] != rem && p[k] != rem {
					e := edges[nedges*4:]
					e[0] = int(p[k])
					e[1] = int(p[j])
					e[2] = int(mesh.Regs[i])
					e[3] = int(mesh.Areas[i])
					nedges++
				}
			}
			// Remove the polygon.
			p2 := mesh.Polys[(mesh.Npolys-1)*nvp*2:]
			if &p[0] != &p2[0] {
				copy(p, p2[:nvp])
			}
			for j := nvp; j < nvp*2; j++ {
				p[j] = meshNullIdx
			}
			mesh.Regs[i] = mesh.Regs[mesh.Npolys-1]
			mesh.Areas[i] = mesh.Areas[mesh.Npolys-1]
			mesh.Npolys--
			i--
		}
	}

	// Remove vertex.
	for i := int(rem); i < mesh.Nverts-1; i++ {
		mesh.Verts[i*3+0] = mesh.Verts[(i+1)*3+0]
		mesh.Verts[i*3+1] = mesh.Verts[(i+1)*3+1]
		mesh.Verts[i*3+2] = mesh.Verts[(i+1)*3+2]
	}
	mesh.Nverts--

	// Adjust indices to match the removed vertex layout.
	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		nv := countPolyVerts(p, nvp)
		for j := 0; j < nv; j++ {
			if p[j] > rem {
				p[j]--
			}
		}
	}
	for i := 0; i < nedges; i++ {
		if edges[i*4+0] > int(rem) {
			edges[i*4+0]--
		}
		if edges[i*4+1] > int(rem) {
			edges[i*4+1]--
		}
	}

	if nedges == 0 {
		return true
	}

	// Start with one vertex, keep appending connected segments to the start and end of the hole.
	hole[nhole] = edges[0]
	nhole++
	hreg[nhreg] = edges[2]
	nhreg++
	harea[nharea] = edges[3]
	nharea++

	for nedges > 0 {
		match := false

		for i := 0; i < nedges; i++ {
			ea := edges[i*4+0]
			eb := edges[i*4+1]
			r := edges[i*4+2]
			a := edges[i*4+3]
			add := false
			if hole[0] == eb {
				// The segment matches the beginning of the hole boundary.
				pushFront(ea, &hole, &nhole)
				pushFront(r, &hreg, &nhreg)
				pushFront(a, &harea, &nharea)
				add = true
			} else if hole[nhole-1] == ea {
				// The segment matches the end of the hole boundary.
				pushBack(eb, &hole, &nhole)
				pushBack(r, &hreg, &nhreg)
				pushBack(a, &harea, &nharea)
				add = true
			}
			if add {
				// The edge segment was added, remove it.
				edges[i*4+0] = edges[(nedges-1)*4+0]
				edges[i*4+1] = edges[(nedges-1)*4+1]
				edges[i*4+2] = edges[(nedges-1)*4+2]
				edges[i*4+3] = edges[(nedges-1)*4+3]
				nedges--
				match = true
				i--
			}
		}

		if !match {
			break
		}
	}

	tris := make([]int, nhole*3)
	tverts := make([]int, nhole*4)
	thole := make([]int, nhole)

	// Generate temp vertex array for triangulation.
	for i := 0; i < nhole; i++ {
		pi := hole[i]
		tverts[i*4+0] = int(mesh.Verts[pi*3+0])
		tverts[i*4+1] = int(mesh.Verts[pi*3+1])
		tverts[i*4+2] = int(mesh.Verts[pi*3+2])
		tverts[i*4+3] = 0
		thole[i] = i
	}

	// Triangulate the hole.
	ntris := triangulate(nhole, tverts, thole, tris)
	if ntris < 0 {
		ntris = -ntris
	}

	// Merge the hole triangles back to polygons.
	polys := make([]uint16, (ntris+1)*nvp)
	// Initialize to MESH_NULL_IDX (0xffff) like C++ version
	for i := range polys {
		polys[i] = meshNullIdx
	}
	pregs := make([]uint16, ntris)
	pareas := make([]uint8, ntris)

	tmpPoly := polys[ntris*nvp:]

	// Build initial polygons.
	npolys := 0
	for j := 0; j < ntris; j++ {
		t := tris[j*3:]
		if t[0] != t[1] && t[0] != t[2] && t[1] != t[2] {
			polys[npolys*nvp+0] = uint16(hole[t[0]])
			polys[npolys*nvp+1] = uint16(hole[t[1]])
			polys[npolys*nvp+2] = uint16(hole[t[2]])

			// If this polygon covers multiple region types then mark it as such
			if hreg[t[0]] != hreg[t[1]] || hreg[t[1]] != hreg[t[2]] {
				pregs[npolys] = multipleRegs
			} else {
				pregs[npolys] = uint16(hreg[t[0]])
			}

			pareas[npolys] = uint8(harea[t[0]])
			npolys++
		}
	}
	if npolys == 0 {
		return true
	}

	// Merge polygons.
	if nvp > 3 {
		for {
			// Find best polygons to merge.
			bestMergeVal := 0
			bestPa := 0
			bestPb := 0
			bestEa := 0
			bestEb := 0

			for j := 0; j < npolys-1; j++ {
				pj := polys[j*nvp:]
				for k := j + 1; k < npolys; k++ {
					pk := polys[k*nvp:]
					var ea, eb int
					v := getPolyMergeValue(pj, pk, mesh.Verts, &ea, &eb, nvp)
					if v > bestMergeVal {
						bestMergeVal = v
						bestPa = j
						bestPb = k
						bestEa = ea
						bestEb = eb
					}
				}
			}

			if bestMergeVal > 0 {
				// Found best, merge.
				pa := polys[bestPa*nvp:]
				pb := polys[bestPb*nvp:]
				mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp)
				if pregs[bestPa] != pregs[bestPb] {
					pregs[bestPa] = multipleRegs
				}

				last := polys[(npolys-1)*nvp:]
				if &pb[0] != &last[0] {
					copy(pb, last[:nvp])
				}
				pregs[bestPb] = pregs[npolys-1]
				pareas[bestPb] = pareas[npolys-1]
				npolys--
			} else {
				break
			}
		}
	}

	// Store polygons.
	for i := 0; i < npolys; i++ {
		if mesh.Npolys >= maxTris {
			break
		}
		p := mesh.Polys[mesh.Npolys*nvp*2:]
		for j := 0; j < nvp*2; j++ {
			p[j] = meshNullIdx
		}
		for j := 0; j < nvp; j++ {
			p[j] = polys[i*nvp+j]
		}
		mesh.Regs[mesh.Npolys] = pregs[i]
		mesh.Areas[mesh.Npolys] = pareas[i]
		mesh.Npolys++
		if mesh.Npolys > maxTris {
			return false
		}
	}

	return true
}

// BuildPolyMesh builds a polygon mesh from the contour set.
func BuildPolyMesh(cset *ContourSet, nvp int, mesh *PolyMesh) {
	mesh.Bmin = cset.Bmin
	mesh.Bmax = cset.Bmax
	mesh.Cs = cset.Cs
	mesh.Ch = cset.Ch
	mesh.BorderSize = cset.BorderSize
	mesh.MaxEdgeError = cset.MaxError

	maxVertices := 0
	maxTris := 0
	maxVertsPerCont := 0
	for i := 0; i < cset.Nconts; i++ {
		if cset.Conts[i].Nverts < 3 {
			continue
		}
		maxVertices += cset.Conts[i].Nverts
		maxTris += cset.Conts[i].Nverts - 2
		if cset.Conts[i].Nverts > maxVertsPerCont {
			maxVertsPerCont = cset.Conts[i].Nverts
		}
	}

	if maxVertices >= 0xfffe {
		return
	}

	vflags := make([]uint8, maxVertices)

	mesh.Verts = make([]uint16, maxVertices*3)
	mesh.Polys = make([]uint16, maxTris*nvp*2)
	// Initialize to MESH_NULL_IDX (0xffff) like C++ version
	for i := range mesh.Polys {
		mesh.Polys[i] = meshNullIdx
	}
	mesh.Regs = make([]uint16, maxTris)
	mesh.Areas = make([]uint8, maxTris)

	mesh.Nverts = 0
	mesh.Npolys = 0
	mesh.Nvp = nvp
	mesh.MaxPolys = maxTris

	nextVert := make([]int, maxVertices)
	firstVert := make([]int, vertexBucketCount)
	for i := 0; i < vertexBucketCount; i++ {
		firstVert[i] = -1
	}

	indices := make([]int, maxVertsPerCont)
	tris := make([]int, maxVertsPerCont*3)
	polys := make([]uint16, (maxVertsPerCont+1)*nvp)
	// Initialize to MESH_NULL_IDX (0xffff) like C++ version
	for i := range polys {
		polys[i] = meshNullIdx
	}
	tmpPoly := polys[maxVertsPerCont*nvp:]

	for i := 0; i < cset.Nconts; i++ {
		cont := cset.Conts[i]

		// Skip null contours.
		if cont.Nverts < 3 {
			continue
		}

		// Triangulate contour
		for j := 0; j < cont.Nverts; j++ {
			indices[j] = j
		}

		ntris := triangulate(cont.Nverts, cont.Verts, indices, tris)
		if ntris <= 0 {
			ntris = -ntris
		}

		// Add and merge vertices.
		for j := 0; j < cont.Nverts; j++ {
			v := cont.Verts[j*4:]
			indices[j] = int(addVertex(uint16(v[0]), uint16(v[1]), uint16(v[2]),
				mesh.Verts, firstVert, nextVert, &mesh.Nverts))
			if v[3]&borderVertex != 0 {
				// This vertex should be removed.
				vflags[indices[j]] = 1
			}
		}

		// Build initial polygons.
		npolys := 0
		for j := 0; j < maxVertsPerCont*nvp; j++ {
			polys[j] = meshNullIdx
		}
		for j := 0; j < ntris; j++ {
			t := tris[j*3:]
			if t[0] != t[1] && t[0] != t[2] && t[1] != t[2] {
				polys[npolys*nvp+0] = uint16(indices[t[0]])
				polys[npolys*nvp+1] = uint16(indices[t[1]])
				polys[npolys*nvp+2] = uint16(indices[t[2]])
				npolys++
			}
		}
		if npolys == 0 {
			continue
		}

		// Merge polygons.
		if nvp > 3 {
			for {
				// Find best polygons to merge.
				bestMergeVal := 0
				bestPa := 0
				bestPb := 0
				bestEa := 0
				bestEb := 0

				for j := 0; j < npolys-1; j++ {
					pj := polys[j*nvp:]
					for k := j + 1; k < npolys; k++ {
						pk := polys[k*nvp:]
						var ea, eb int
						v := getPolyMergeValue(pj, pk, mesh.Verts, &ea, &eb, nvp)
						if v > bestMergeVal {
							bestMergeVal = v
							bestPa = j
							bestPb = k
							bestEa = ea
							bestEb = eb
						}
					}
				}

				if bestMergeVal > 0 {
					// Found best, merge.
					pa := polys[bestPa*nvp:]
					pb := polys[bestPb*nvp:]
					mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp)
					lastPoly := polys[(npolys-1)*nvp:]
					if &pb[0] != &lastPoly[0] {
						copy(pb, lastPoly[:nvp])
					}
					npolys--
				} else {
					break
				}
			}
		}

		// Store polygons.
		for j := 0; j < npolys; j++ {
			p := mesh.Polys[mesh.Npolys*nvp*2:]
			q := polys[j*nvp:]
			for k := 0; k < nvp; k++ {
				p[k] = q[k]
			}
			mesh.Regs[mesh.Npolys] = cont.Reg
			mesh.Areas[mesh.Npolys] = cont.Area
			mesh.Npolys++
			if mesh.Npolys > maxTris {
				return
			}
		}
	}

	// Remove edge vertices.
	for i := 0; i < mesh.Nverts; i++ {
		if vflags[i] != 0 {
			if !canRemoveVertex(mesh, uint16(i)) {
				continue
			}
			if !removeVertex(mesh, uint16(i), maxTris) {
				return
			}
			// Fixup vertex flags
			for j := i; j < mesh.Nverts; j++ {
				vflags[j] = vflags[j+1]
			}
			i--
		}
	}

	// Calculate adjacency.
	if !buildMeshAdjacency(mesh.Polys, mesh.Npolys, mesh.Nverts, nvp) {
		return
	}

	// Find portal edges
	if mesh.BorderSize > 0 {
		w := cset.Width
		h := cset.Height
		for i := 0; i < mesh.Npolys; i++ {
			p := mesh.Polys[i*2*nvp:]
			for j := 0; j < nvp; j++ {
				if p[j] == meshNullIdx {
					break
				}
				// Skip connected edges.
				if p[nvp+j] != meshNullIdx {
					continue
				}
				nj := j + 1
				if nj >= nvp || p[nj] == meshNullIdx {
					nj = 0
				}
				va := mesh.Verts[p[j]*3:]
				vb := mesh.Verts[p[nj]*3:]

				if int(va[0]) == 0 && int(vb[0]) == 0 {
					p[nvp+j] = 0x8000
				} else if int(va[2]) == h && int(vb[2]) == h {
					p[nvp+j] = 0x8000 | 1
				} else if int(va[0]) == w && int(vb[0]) == w {
					p[nvp+j] = 0x8000 | 2
				} else if int(va[2]) == 0 && int(vb[2]) == 0 {
					p[nvp+j] = 0x8000 | 3
				}
			}
		}
	}

	// Just allocate the mesh flags array. The user is responsible to fill it.
	mesh.Flags = make([]uint16, mesh.Npolys)
}

// MergePolyMeshes merges multiple polygon meshes into one.
func MergePolyMeshes(meshes []*PolyMesh, mesh *PolyMesh) {
	if len(meshes) == 0 {
		return
	}

	mesh.Nvp = meshes[0].Nvp
	mesh.Cs = meshes[0].Cs
	mesh.Ch = meshes[0].Ch
	mesh.Bmin = meshes[0].Bmin
	mesh.Bmax = meshes[0].Bmax

	maxVerts := 0
	maxPolys := 0
	maxVertsPerMesh := 0
	for i := 0; i < len(meshes); i++ {
		Vmin(&mesh.Bmin, &meshes[i].Bmin)
		Vmax(&mesh.Bmax, &meshes[i].Bmax)
		if meshes[i].Nverts > maxVertsPerMesh {
			maxVertsPerMesh = meshes[i].Nverts
		}
		maxVerts += meshes[i].Nverts
		maxPolys += meshes[i].Npolys
	}

	mesh.Nverts = 0
	mesh.Verts = make([]uint16, maxVerts*3)

	mesh.Npolys = 0
	mesh.Polys = make([]uint16, maxPolys*2*mesh.Nvp)
	for i := range mesh.Polys {
		mesh.Polys[i] = meshNullIdx
	}

	mesh.Regs = make([]uint16, maxPolys)
	mesh.Areas = make([]uint8, maxPolys)
	mesh.Flags = make([]uint16, maxPolys)

	nextVert := make([]int, maxVerts)
	firstVert := make([]int, vertexBucketCount)
	for i := 0; i < vertexBucketCount; i++ {
		firstVert[i] = -1
	}

	vremap := make([]uint16, maxVertsPerMesh)

	for i := 0; i < len(meshes); i++ {
		pmesh := meshes[i]

		ox := uint16(math.Floor(float64((pmesh.Bmin[0]-mesh.Bmin[0])/mesh.Cs + 0.5)))
		oz := uint16(math.Floor(float64((pmesh.Bmin[2]-mesh.Bmin[2])/mesh.Cs + 0.5)))

		isMinX := ox == 0
		isMinZ := oz == 0
		isMaxX := uint16(math.Floor(float64((mesh.Bmax[0]-pmesh.Bmax[0])/mesh.Cs+0.5))) == 0
		isMaxZ := uint16(math.Floor(float64((mesh.Bmax[2]-pmesh.Bmax[2])/mesh.Cs+0.5))) == 0
		isOnBorder := isMinX || isMinZ || isMaxX || isMaxZ

		for j := 0; j < pmesh.Nverts; j++ {
			v := pmesh.Verts[j*3:]
			vremap[j] = addVertex(v[0]+ox, v[1], v[2]+oz,
				mesh.Verts, firstVert, nextVert, &mesh.Nverts)
		}

		for j := 0; j < pmesh.Npolys; j++ {
			tgt := mesh.Polys[mesh.Npolys*2*mesh.Nvp:]
			src := pmesh.Polys[j*2*mesh.Nvp:]
			mesh.Regs[mesh.Npolys] = pmesh.Regs[j]
			mesh.Areas[mesh.Npolys] = pmesh.Areas[j]
			mesh.Flags[mesh.Npolys] = pmesh.Flags[j]
			mesh.Npolys++
			for k := 0; k < mesh.Nvp; k++ {
				if src[k] == meshNullIdx {
					break
				}
				tgt[k] = vremap[src[k]]
			}

			if isOnBorder {
				for k := mesh.Nvp; k < mesh.Nvp*2; k++ {
					if src[k]&0x8000 != 0 && src[k] != 0xffff {
						dir := src[k] & 0xf
						switch dir {
						case 0: // Portal x-
							if isMinX {
								tgt[k] = src[k]
							}
						case 1: // Portal z+
							if isMaxZ {
								tgt[k] = src[k]
							}
						case 2: // Portal x+
							if isMaxX {
								tgt[k] = src[k]
							}
						case 3: // Portal z-
							if isMinZ {
								tgt[k] = src[k]
							}
						}
					}
				}
			}
		}
	}

	// Calculate adjacency.
	buildMeshAdjacency(mesh.Polys, mesh.Npolys, mesh.Nverts, mesh.Nvp)
}

// CopyPolyMesh creates a deep copy of a polygon mesh.
func CopyPolyMesh(src *PolyMesh, dst *PolyMesh) {
	dst.Nverts = src.Nverts
	dst.Npolys = src.Npolys
	dst.MaxPolys = src.Npolys
	dst.Nvp = src.Nvp
	dst.Bmin = src.Bmin
	dst.Bmax = src.Bmax
	dst.Cs = src.Cs
	dst.Ch = src.Ch
	dst.BorderSize = src.BorderSize
	dst.MaxEdgeError = src.MaxEdgeError

	dst.Verts = make([]uint16, len(src.Verts))
	copy(dst.Verts, src.Verts)

	dst.Polys = make([]uint16, len(src.Polys))
	copy(dst.Polys, src.Polys)

	dst.Regs = make([]uint16, len(src.Regs))
	copy(dst.Regs, src.Regs)

	dst.Areas = make([]uint8, len(src.Areas))
	copy(dst.Areas, src.Areas)

	dst.Flags = make([]uint16, len(src.Flags))
	copy(dst.Flags, src.Flags)
}
