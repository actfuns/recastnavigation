package detour_crowd

import (
	"math"

	"github.com/actfuns/recastnavigation/detour"
)

// PathCorridor represents a dynamic polygon corridor used to plan agent movement.
type PathCorridor struct {
	pos    [3]float32
	target [3]float32

	path    []PolyRef
	npath   int
	maxPath int
}

// NewPathCorridor creates a new path corridor.
func NewPathCorridor() *PathCorridor {
	return &PathCorridor{}
}

// Init allocates the corridor's path buffer.
func (c *PathCorridor) Init(maxPath int) bool {
	c.path = make([]PolyRef, maxPath)
	c.npath = 0
	c.maxPath = maxPath
	return true
}

// Reset resets the path corridor to the specified position.
func (c *PathCorridor) Reset(ref PolyRef, pos [3]float32) {
	c.pos = pos
	c.target = pos
	c.path[0] = ref
	c.npath = 1
}

// FindCorners finds the corners in the corridor from the position toward the target.
func (c *PathCorridor) FindCorners(cornerVerts []float32, cornerFlags []uint8,
	cornerPolys []PolyRef, maxCorners int,
	navquery *detour.NavMeshQuery, filter *QueryFilter) int {

	const minTargetDist = 0.01

	verts, flags, refs := cornerVerts, cornerFlags, cornerPolys
	ncorners, _ := navquery.FindStraightPath(c.pos, c.target, c.path[:c.npath], c.npath, verts, flags, refs, maxCorners, 0)

	// Prune points in the beginning of the path which are too close.
	for ncorners > 0 {
		cv := cornerVertsToVec(cornerVerts, 0)
		if (cornerFlags[0]&StraightPathOffMeshConnection != 0) ||
			vecDist2DSqr(cv, c.pos) > minTargetDist*minTargetDist {
			break
		}
		ncorners--
		if ncorners > 0 {
			copy(cornerFlags, cornerFlags[1:1+ncorners])
			copy(cornerPolys, cornerPolys[1:1+ncorners])
			copy(cornerVerts, cornerVerts[3:3+ncorners*3])
		}
	}

	// Prune points after an off-mesh connection.
	for i := 0; i < ncorners; i++ {
		if cornerFlags[i]&StraightPathOffMeshConnection != 0 {
			ncorners = i + 1
			break
		}
	}

	return ncorners
}

// OptimizePathVisibility attempts to optimize the path if the specified point is visible.
func (c *PathCorridor) OptimizePathVisibility(next [3]float32, pathOptimizationRange float32, navquery *detour.NavMeshQuery, filter *QueryFilter) {

	goal := next
	dist := vecDist2D(c.pos, goal)

	// If too close to the goal, do not try to optimize.
	if dist < 0.01 {
		return
	}

	// Overshoot a little.
	dist = float32(math.Min(float64(dist+0.01), float64(pathOptimizationRange)))

	delta := vecSub(goal, c.pos)
	goal = vecMad(c.pos, delta, pathOptimizationRange/dist)

	const maxRes = 32
	res := make([]PolyRef, maxRes)
	hit := &RaycastHit{}
	// Use default options (0) and no previous ref for visibility optimization.
	navquery.Raycast(c.path[0], c.pos, goal, filter, 0, 0, hit)
	nres := hit.PathCount
	if nres > 1 && hit.T > 0.99 {
		copy(res[:nres], hit.Path[:nres])
		c.npath = mergeCorridorStartShortcut(c.path, c.npath, c.maxPath, res, nres)
	}
}

// OptimizePathTopology attempts to optimize the path using a local area search.
func (c *PathCorridor) OptimizePathTopology(navquery *detour.NavMeshQuery, filter *QueryFilter) bool {
	if c.npath < 3 {
		return false
	}

	const maxIter = 32
	const maxRes = 32

	res := make([]PolyRef, maxRes)
	nres := 0
	navquery.FindPathSliced(c.path[0], c.path[c.npath-1], c.pos, c.target, filter, 0)
	navquery.UpdateSlicedPath(maxIter)
	nres, err := navquery.GetPathFromSlicedPath(res, maxRes)

	if err == nil && nres > 0 {
		c.npath = mergeCorridorStartShortcut(c.path, c.npath, c.maxPath, res, nres)
		return true
	}

	return false
}

// MoveOverOffmeshConnection advances the path over an off-mesh connection.
func (c *PathCorridor) MoveOverOffmeshConnection(offMeshConRef PolyRef, refs *[2]PolyRef, navquery *detour.NavMeshQuery) ([3]float32, [3]float32, bool) {

	// Advance the path up to and over the off-mesh connection.
	var prevRef PolyRef
	polyRef := c.path[0]
	npos := 0
	for npos < c.npath && polyRef != offMeshConRef {
		prevRef = polyRef
		polyRef = c.path[npos]
		npos++
	}
	if npos == c.npath {
		return [3]float32{}, [3]float32{}, false
	}

	// Prune path
	for i := npos; i < c.npath; i++ {
		c.path[i-npos] = c.path[i]
	}
	c.npath -= npos

	refs[0] = prevRef
	refs[1] = polyRef

	nav := navquery.GetAttachedNavMesh()
	if nav == nil {
		return [3]float32{}, [3]float32{}, false
	}

	startPos, endPos, err := nav.GetOffMeshConnectionPolyEndPoints(refs[0], refs[1])
	if err != nil {
		return [3]float32{}, [3]float32{}, false
	}

	c.pos = endPos
	return startPos, endPos, true
}

// FixPathStart fixes the start of the path corridor.
func (c *PathCorridor) FixPathStart(safeRef PolyRef, safePos [3]float32) bool {
	c.pos = safePos
	if c.npath < 3 && c.npath > 0 {
		c.path[2] = c.path[c.npath-1]
		c.path[0] = safeRef
		c.path[1] = 0
		c.npath = 3
	} else {
		c.path[0] = safeRef
		c.path[1] = 0
	}
	return true
}

// TrimInvalidPath trims invalid segments from the path.
func (c *PathCorridor) TrimInvalidPath(safeRef PolyRef, safePos [3]float32, navquery *detour.NavMeshQuery, filter *QueryFilter) bool {

	// Keep valid path as far as possible.
	n := 0
	for n < c.npath && navquery.IsValidPolyRef(c.path[n], filter) {
		n++
	}

	switch n {
	case c.npath:
		return true
	case 0:
		c.pos = safePos
		c.path[0] = safeRef
		c.npath = 1
	default:
		c.npath = n
	}

	tgt := c.target
	result, err := navquery.ClosestPointOnPolyBoundary(c.path[c.npath-1], tgt)
	if err == nil {
		c.target = result
	}

	return true
}

// IsValid checks if the current corridor path's polygon references remain valid.
func (c *PathCorridor) IsValid(maxLookAhead int, navquery *detour.NavMeshQuery, filter *QueryFilter) bool {
	n := recastMin(c.npath, maxLookAhead)
	for i := 0; i < n; i++ {
		if !navquery.IsValidPolyRef(c.path[i], filter) {
			return false
		}
	}
	return true
}

// MovePosition moves the position from the current location to the desired location.
func (c *PathCorridor) MovePosition(npos [3]float32, navquery *detour.NavMeshQuery, filter *QueryFilter) bool {
	const maxVisited = 16
	result := make([]float32, maxVisited*3)
	visited := make([]PolyRef, maxVisited)
	nvisited, err := navquery.MoveAlongSurface(c.path[0], c.pos, npos, filter, result, visited, maxVisited)
	if err == nil {
		c.npath = mergeCorridorStartMoved(c.path, c.npath, c.maxPath, visited, nvisited)

		// Adjust the position to stay on top of the navmesh.
		var resPos [3]float32
		copy(resPos[:], result[(nvisited-1)*3:(nvisited-1)*3+3])
		h, _ := navquery.GetPolyHeight(c.path[0], resPos)
		resPos[1] = h
		c.pos = resPos
		return true
	}
	return false
}

// MoveTargetPosition moves the target from the current location to the desired location.
func (c *PathCorridor) MoveTargetPosition(npos [3]float32, navquery *detour.NavMeshQuery, filter *QueryFilter) bool {
	const maxVisited = 16
	result := make([]float32, maxVisited*3)
	visited := make([]PolyRef, maxVisited)
	nvisited, err := navquery.MoveAlongSurface(c.path[c.npath-1], c.target, npos, filter, result, visited, maxVisited)
	if err == nil {
		c.npath = mergeCorridorEndMoved(c.path, c.npath, c.maxPath, visited, nvisited)
		var resPos [3]float32
		copy(resPos[:], result[(nvisited-1)*3:(nvisited-1)*3+3])
		c.target = resPos
		return true
	}
	return false
}

// SetCorridor loads a new path and target into the corridor.
func (c *PathCorridor) SetCorridor(target [3]float32, path []PolyRef, npath int) {
	c.target = target
	copy(c.path[:npath], path[:npath])
	c.npath = npath
}

// GetPos returns the current position within the corridor.
func (c *PathCorridor) GetPos() [3]float32 {
	return c.pos
}

// GetTarget returns the current target within the corridor.
func (c *PathCorridor) GetTarget() [3]float32 {
	return c.target
}

// GetFirstPoly returns the first polygon reference in the corridor.
func (c *PathCorridor) GetFirstPoly() PolyRef {
	if c.npath > 0 {
		return c.path[0]
	}
	return 0
}

// GetLastPoly returns the last polygon reference in the corridor.
func (c *PathCorridor) GetLastPoly() PolyRef {
	if c.npath > 0 {
		return c.path[c.npath-1]
	}
	return 0
}

// GetPath returns the corridor's path.
func (c *PathCorridor) GetPath() []PolyRef {
	return c.path[:c.npath]
}

// GetPathCount returns the number of polygons in the current corridor path.
func (c *PathCorridor) GetPathCount() int {
	return c.npath
}

// --- Free functions for corridor merging ---

func mergeCorridorStartMoved(path []PolyRef, npath, maxPath int, visited []PolyRef, nvisited int) int {
	furthestPath := -1
	furthestVisited := -1

	// Find furthest common polygon.
	for i := npath - 1; i >= 0; i-- {
		found := false
		for j := nvisited - 1; j >= 0; j-- {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.
	req := nvisited - furthestVisited
	orig := recastMin(furthestPath+1, npath)
	size := recastMax(0, npath-orig)
	if req+size > maxPath {
		size = maxPath - req
	}
	if size > 0 {
		copy(path[req:req+size], path[orig:orig+size])
	}

	// Store visited
	for i := 0; i < recastMin(req, maxPath); i++ {
		path[i] = visited[(nvisited-1)-i]
	}

	return req + size
}

func mergeCorridorEndMoved(path []PolyRef, npath, maxPath int, visited []PolyRef, nvisited int) int {
	furthestPath := -1
	furthestVisited := -1

	// Find furthest common polygon.
	for i := 0; i < npath; i++ {
		found := false
		for j := nvisited - 1; j >= 0; j-- {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.
	ppos := furthestPath + 1
	vpos := furthestVisited + 1
	count := recastMin(nvisited-vpos, maxPath-ppos)
	if count > 0 {
		copy(path[ppos:ppos+count], visited[vpos:vpos+count])
	}

	return ppos + count
}

func mergeCorridorStartShortcut(path []PolyRef, npath, maxPath int, visited []PolyRef, nvisited int) int {
	furthestPath := -1
	furthestVisited := -1

	// Find furthest common polygon.
	for i := npath - 1; i >= 0; i-- {
		found := false
		for j := nvisited - 1; j >= 0; j-- {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.
	req := furthestVisited
	if req <= 0 {
		return npath
	}

	orig := furthestPath
	size := recastMax(0, npath-orig)
	if req+size > maxPath {
		size = maxPath - req
	}
	if size > 0 {
		copy(path[req:req+size], path[orig:orig+size])
	}

	// Store visited
	for i := 0; i < req; i++ {
		path[i] = visited[i]
	}

	return req + size
}

// Straight path flags
const (
	StraightPathStart             = 0x01
	StraightPathEnd               = 0x02
	StraightPathOffMeshConnection = 0x04
)

// Helper vector functions

func vecDist2DSqr(a, b [3]float32) float32 {
	dx := a[0] - b[0]
	dz := a[2] - b[2]
	return dx*dx + dz*dz
}

func vecDist2D(a, b [3]float32) float32 {
	return float32(math.Sqrt(float64(vecDist2DSqr(a, b))))
}

func vecSub(v1, v2 [3]float32) [3]float32 {
	return [3]float32{v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]}
}

func vecMad(v1, v2 [3]float32, s float32) [3]float32 {
	return [3]float32{v1[0] + v2[0]*s, v1[1] + v2[1]*s, v1[2] + v2[2]*s}
}

func vecScale(v [3]float32, s float32) [3]float32 {
	return [3]float32{v[0] * s, v[1] * s, v[2] * s}
}

func vecLenSqr(v [3]float32) float32 {
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
}

func vecLen(v [3]float32) float32 {
	return float32(math.Sqrt(float64(vecLenSqr(v))))
}

func vecNormalize(v [3]float32) [3]float32 {
	d := vecLen(v)
	if d > 0.0001 {
		return vecScale(v, 1.0/d)
	}
	return v
}

func vecDot2D(a, b [3]float32) float32 {
	return a[0]*b[0] + a[2]*b[2]
}

func vecPerp2D(a, b [3]float32) float32 {
	return a[0]*b[2] - a[2]*b[0]
}

func triArea2D(a, b, c [3]float32) float32 {
	ax := b[0] - a[0]
	az := b[2] - a[2]
	bx := c[0] - a[0]
	bz := c[2] - a[2]
	return ax*bz - az*bx
}

func vecLerp(a, b [3]float32, t float32) [3]float32 {
	return [3]float32{
		a[0] + (b[0]-a[0])*t,
		a[1] + (b[1]-a[1])*t,
		a[2] + (b[2]-a[2])*t,
	}
}

func vecAdd(a, b [3]float32) [3]float32 {
	return [3]float32{a[0] + b[0], a[1] + b[1], a[2] + b[2]}
}

// cornerVert extracts a [3]float32 vertex from the cornerVerts flat array.
func cornerVert(cornerVerts []float32, idx int) [3]float32 {
	i := idx * 3
	return [3]float32{cornerVerts[i], cornerVerts[i+1], cornerVerts[i+2]}
}

// cornerVertsToVec extracts a [3]float32 from a flat float32 array at a given index offset (in elements, not vectors).
func cornerVertsToVec(verts []float32, idx int) [3]float32 {
	i := idx * 3
	return [3]float32{verts[i], verts[i+1], verts[i+2]}
}

func recastMin(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func recastMax(a, b int) int {
	if a > b {
		return a
	}
	return b
}
