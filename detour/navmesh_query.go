package detour

import (
	"errors"
	"math"
)

const DefaultMaxPathLength = 512

// distPtSegSqr2D calculates squared distance from point to segment in 2D (XZ plane)
func distPtSegSqr2D(pt, p1, p2 [3]float32) (float32, float32) {
	dx := p2[0] - p1[0]
	dz := p2[2] - p1[2]

	var t float32
	if dx*dx+dz*dz < 1e-6 {
		t = 0
	} else {
		t = ((pt[0]-p1[0])*dx + (pt[2]-p1[2])*dz) / (dx*dx + dz*dz)
		if t < 0 {
			t = 0
		} else if t > 1 {
			t = 1
		}
	}

	closestX := p1[0] + t*dx
	closestZ := p1[2] + t*dz

	dx = pt[0] - closestX
	dz = pt[2] - closestZ

	return dx*dx + dz*dz, t
}

// closestPointOnPolyBoundary finds the closest point on a polygon's boundary
func (q *NavMeshQuery) closestPointOnPolyBoundary(ref PolyRef, pos [3]float32) ([3]float32, error) {
	tile, poly := q.Nav.GetTileAndPolyByRefUnsafe(ref)
	if poly == nil {
		return [3]float32{}, ErrInvalidParam
	}

	nv := int(poly.VertCount)
	if nv < 3 || nv > VertsPerPolygon {
		return [3]float32{}, ErrInvalidParam
	}

	// Collect vertices into flat array (matches C++: float verts[DT_VERTS_PER_POLYGON*3])
	var verts [VertsPerPolygon * 3]float32
	for i := 0; i < nv; i++ {
		vi := poly.Verts[i] * 3
		verts[i*3] = tile.Verts[vi]
		verts[i*3+1] = tile.Verts[vi+1]
		verts[i*3+2] = tile.Verts[vi+2]
	}

	// DistancePtPolyEdgesSqr returns true if point is inside,
	// and fills edged/edget with per-edge distance/t values (matches C++)
	var edged [VertsPerPolygon]float32
	var edget [VertsPerPolygon]float32
	if DistancePtPolyEdgesSqr(pos, verts[:], nv, edged[:], edget[:]) {
		return pos, nil
	}

	// Point is outside the polygon, clamp to nearest edge (matches C++)
	imin := 0
	dmin := edged[0]
	for i := 1; i < nv; i++ {
		if edged[i] < dmin {
			dmin = edged[i]
			imin = i
		}
	}

	// Vlerp between edge vertices using edget[imin]
	imin3 := imin * 3
	next3 := ((imin + 1) % nv) * 3
	t := edget[imin]
	return [3]float32{
		verts[imin3] + (verts[next3]-verts[imin3])*t,
		verts[imin3+1] + (verts[next3+1]-verts[imin3+1])*t,
		verts[imin3+2] + (verts[next3+2]-verts[imin3+2])*t,
	}, nil
}

// NeighbourSeg stores a segment from a polygon's neighbours.
type NeighbourSeg struct {
	Seg [6]float32
	Ref PolyRef
}

// QueryFilter is a filter for navigation mesh queries.
type QueryFilter struct {
	AreaCost     [MaxAreas]float32
	ExcludeFlags uint16
	IncludeFlags uint16
}

// NewQueryFilter creates and initializes a new QueryFilter.
func NewQueryFilter() *QueryFilter {
	f := &QueryFilter{}
	for i := 0; i < MaxAreas; i++ {
		f.AreaCost[i] = 1.0
	}
	return f
}

// PassFilter returns true if the polygon can be visited.
func (f *QueryFilter) PassFilter(ref PolyRef, tile *MeshTile, poly *Poly) bool {
	if poly.Flags&f.ExcludeFlags != 0 {
		return false
	}
	if poly.Flags&f.IncludeFlags == 0 {
		return false
	}
	return true
}

// GetCost returns the cost to travel through the polygon.
func (f *QueryFilter) GetCost(pa, pb [3]float32, prevRef PolyRef, prevTile *MeshTile, prevPoly *Poly, curRef PolyRef, curTile *MeshTile, curPoly *Poly, nextRef PolyRef, nextTile *MeshTile, nextPoly *Poly) float32 {
	dx := pb[0] - pa[0]
	dy := pb[1] - pa[1]
	dz := pb[2] - pa[2]
	return float32(math.Sqrt(float64(dx*dx+dy*dy+dz*dz))) * f.AreaCost[curPoly.GetArea()]
}

// NavMeshQuery provides pathfinding and navigation mesh querying services.
type NavMeshQuery struct {
	Nav          *NavMesh
	Filter       *QueryFilter
	NodePool     *NodePool
	OpenList     *NodeQueue
	QueryData    QueryData
	TinyNodePool *NodePool

	// Internal buffers (replaces package-level globals)
	neiRefs [32]PolyRef
	neiPos  [32 * 3]float32
	bestPos [3]float32
	hitPos  [3]float32

	// Pre-allocated path buffers (zero-allocation pathfinding)
	pathBuf           [DefaultMaxPathLength]PolyRef
	straightPath      [DefaultMaxPathLength * 3]float32
	straightPathFlags [DefaultMaxPathLength]uint8
	straightPathRefs  [DefaultMaxPathLength]PolyRef
	pathLen           int
}

// NewNavMeshQuery creates a new NavMeshQuery.
func NewNavMeshQuery() *NavMeshQuery {
	return &NavMeshQuery{}
}

// Init initializes the NavMeshQuery.
func (q *NavMeshQuery) Init(nav *NavMesh, maxNodes int) error {
	q.Nav = nav
	var err error
	q.NodePool, err = NewNodePool(maxNodes, int(NextPow2(uint32(maxNodes/4))))
	if err != nil {
		return ErrInvalidParam
	}
	q.OpenList = NewNodeQueue(maxNodes)
	q.TinyNodePool, err = NewNodePool(512, int(NextPow2(uint32(512/4))))
	if err != nil {
		return ErrInvalidParam
	}
	return nil
}

// FindPath finds a path from the start polygon to the end polygon.
func (q *NavMeshQuery) FindPath(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, path []PolyRef) (int, error) {
	if startRef == 0 || endRef == 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil {
		return 0, ErrInvalidParam
	}
	if cap(path) == 0 {
		return 0, ErrBufferTooSmall
	}

	// Validate input
	if !q.Nav.IsValidPolyRef(startRef) || !q.Nav.IsValidPolyRef(endRef) {
		return 0, ErrInvalidParam
	}

	maxPath := cap(path)
	path = path[:maxPath]

	// Check if start and end are the same
	if startRef == endRef {
		path[0] = startRef
		return 1, nil
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Pos = startPos
	startNode.Cost = 0
	sdx := startPos[0] - endPos[0]
	sdy := startPos[1] - endPos[1]
	sdz := startPos[2] - endPos[2]
	startNode.Total = float32(math.Sqrt(float64(sdx*sdx+sdy*sdy+sdz*sdz))) * H_SCALE
	startNode.ID = startRef
	startNode.Flags = NodeOpen

	q.OpenList.Push(startNode)

	lastBestNode := startNode
	lastBestNodeCost := startNode.Total

	outOfNodes := false

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &^= NodeOpen
		bestNode.Flags |= NodeClosed

		// Reached the goal, stop searching.
		if bestNode.ID == endRef {
			lastBestNode = bestNode
			break
		}

		// Get current poly and tile.
		bestRef := bestNode.ID
		bestTile, bestPoly := q.Nav.GetTileAndPolyByRefUnsafe(bestRef)

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		var parentTile *MeshTile
		var parentPoly *Poly
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}
		if parentRef != 0 {
			parentTile, parentPoly = q.Nav.GetTileAndPolyByRefUnsafe(parentRef)
		}

		for l := bestPoly.FirstLink; l != NullLink; l = bestTile.Links[l].Next {
			neighbourRef := bestTile.Links[l].Ref

			// Skip invalid ids and do not expand back to where we came from.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			// Get neighbour poly and tile.
			neighbourTile, neighbourPoly := q.Nav.GetTileAndPolyByRefUnsafe(neighbourRef)

			if !filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// deal explicitly with crossing tile boundaries
			crossSide := uint8(0)
			if bestTile.Links[l].Side != 0xff {
				crossSide = bestTile.Links[l].Side >> 1
			}

			// get the node
			neighbourNode := q.NodePool.GetNode(neighbourRef, crossSide)
			if neighbourNode == nil {
				outOfNodes = true
				continue
			}

			// If the node is visited the first time, calculate node position.
			if neighbourNode.Flags == 0 {
				neighbourNode.Pos = q.getEdgeMidPoint(bestRef, neighbourRef, bestTile, neighbourTile, bestPoly, neighbourPoly)
			}

			// Calculate cost and heuristic.
			var cost float32
			var heuristic float32

			// Special case for last node.
			if neighbourRef == endRef {
				// Cost
				curCost := filter.GetCost(bestNode.Pos, neighbourNode.Pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				endCost := filter.GetCost(neighbourNode.Pos, endPos,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly,
					0, nil, nil)

				cost = bestNode.Cost + curCost + endCost
				heuristic = 0
			} else {
				// Cost
				curCost := filter.GetCost(bestNode.Pos, neighbourNode.Pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				cost = bestNode.Cost + curCost
				ep := neighbourNode.Pos
				hdx := ep[0] - endPos[0]
				hdy := ep[1] - endPos[1]
				hdZ := ep[2] - endPos[2]
				heuristic = float32(math.Sqrt(float64(hdx*hdx+hdy*hdy+hdZ*hdZ))) * H_SCALE
			}

			total := cost + heuristic

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&NodeOpen) != 0 && total >= neighbourNode.Total {
				continue
			}
			// The node is already visited and processed, and the new result is worse, skip.
			if (neighbourNode.Flags&NodeClosed) != 0 && total >= neighbourNode.Total {
				continue
			}

			// Add or update the node.
			neighbourNode.Pidx = q.NodePool.GetNodeIdx(bestNode)
			neighbourNode.ID = neighbourRef
			neighbourNode.Flags &^= NodeClosed
			neighbourNode.Cost = cost
			neighbourNode.Total = total

			if (neighbourNode.Flags & NodeOpen) != 0 {
				// Already in open, update node location.
				q.OpenList.Modify(neighbourNode)
			} else {
				// Put the node in open list.
				neighbourNode.Flags |= NodeOpen
				q.OpenList.Push(neighbourNode)
			}

			// Update nearest node to target so far.
			if heuristic < lastBestNodeCost {
				lastBestNodeCost = heuristic
				lastBestNode = neighbourNode
			}
		}
	}

	n := q.getPathToNode(lastBestNode, path, maxPath)
	var err error = nil

	if lastBestNode.ID != endRef {
		err = ErrPartialResult
	}

	if outOfNodes {
		if err != nil {
			err = errors.Join(err, ErrOutOfNodes)
		} else {
			err = ErrOutOfNodes
		}
	}

	return n, err
}

// FindPathDirect finds a path and straight path in one call, using pre-allocated internal buffers.
// Returns (pathLength, straightPathLength, err).
// The caller can access the path via q.GetPathRefs() and straight path via q.GetStraightPath().
func (q *NavMeshQuery) FindPathDirect(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, maxPath int) (int, int, error) {
	pathLen, err := q.FindPath(startRef, endRef, startPos, endPos, filter, q.pathBuf[:])
	if pathLen == 0 {
		return 0, 0, err
	}
	straightLen, err := q.FindStraightPath(startPos, endPos,
		q.pathBuf[:pathLen], pathLen,
		q.straightPath[:], q.straightPathFlags[:], q.straightPathRefs[:],
		maxPath, StraightPathAreaCrossings)
	return pathLen, straightLen, err
}

// GetPathRefs returns the last path found as a slice.
func (q *NavMeshQuery) GetPathRefs(pathLen int) []PolyRef {
	return q.pathBuf[:pathLen]
}

// GetStraightPath returns the last straight path found as a slice.
func (q *NavMeshQuery) GetStraightPath(straightLen int) []float32 {
	return q.straightPath[:straightLen*3]
}

func getPortalPoints(from, to PolyRef, fromTile, toTile *MeshTile, fromPoly, toPoly *Poly, left, right *[3]float32) bool {
	// Find the link that connects the two polygons
	for l := fromPoly.FirstLink; l != NullLink; l = fromTile.Links[l].Next {
		if fromTile.Links[l].Ref == to {
			edge := int(fromTile.Links[l].Edge)

			// Handle off-mesh connections.
			if fromPoly.GetType() == PolyTypeOffMeshConnection {
				vi := fromPoly.Verts[edge] * 3
				left[0], left[1], left[2] = fromTile.Verts[vi], fromTile.Verts[vi+1], fromTile.Verts[vi+2]
				*right = *left
				return true
			}
			if toPoly.GetType() == PolyTypeOffMeshConnection {
				for l2 := toPoly.FirstLink; l2 != NullLink; l2 = toTile.Links[l2].Next {
					if toTile.Links[l2].Ref == from {
						v := int(toTile.Links[l2].Edge)
						vi := toPoly.Verts[v] * 3
						left[0], left[1], left[2] = toTile.Verts[vi], toTile.Verts[vi+1], toTile.Verts[vi+2]
						*right = *left
						return true
					}
				}
				return false
			}

			// Get the edge vertices
			nv := int(fromPoly.VertCount)
			va := edge
			vb := (edge + 1) % nv

			via := fromPoly.Verts[va] * 3
			vib := fromPoly.Verts[vb] * 3
			left[0], left[1], left[2] = fromTile.Verts[via], fromTile.Verts[via+1], fromTile.Verts[via+2]
			right[0], right[1], right[2] = fromTile.Verts[vib], fromTile.Verts[vib+1], fromTile.Verts[vib+2]

			// If the link is at tile boundary, clamp the vertices to the link width.
			if fromTile.Links[l].Side != 0xff {
				if fromTile.Links[l].Bmin != 0 || fromTile.Links[l].Bmax != 255 {
					s := float32(1.0 / 255.0)
					tmin := float32(fromTile.Links[l].Bmin) * float32(s)
					tmax := float32(fromTile.Links[l].Bmax) * float32(s)
					// Keep originals for lerp since left/right are updated in-place
					lx, ly, lz := left[0], left[1], left[2]
					rx, ry, rz := right[0], right[1], right[2]
					left[0] = lx + (rx-lx)*tmin
					left[1] = ly + (ry-ly)*tmin
					left[2] = lz + (rz-lz)*tmin
					right[0] = lx + (rx-lx)*tmax
					right[1] = ly + (ry-ly)*tmax
					right[2] = lz + (rz-lz)*tmax
				}
			}
			return true
		}
	}
	return false
}

func (q *NavMeshQuery) getEdgeMidPoint(from, to PolyRef, fromTile, toTile *MeshTile, fromPoly, toPoly *Poly) [3]float32 {
	// Find the link that connects the two polygons
	for l := fromPoly.FirstLink; l != NullLink; l = fromTile.Links[l].Next {
		if fromTile.Links[l].Ref == to {
			edge := int(fromTile.Links[l].Edge)

			// Handle off-mesh connections.
			if fromPoly.GetType() == PolyTypeOffMeshConnection {
				vi := fromPoly.Verts[edge] * 3
				return [3]float32{fromTile.Verts[vi], fromTile.Verts[vi+1], fromTile.Verts[vi+2]}
			}
			if toPoly.GetType() == PolyTypeOffMeshConnection {
				for l2 := toPoly.FirstLink; l2 != NullLink; l2 = toTile.Links[l2].Next {
					if toTile.Links[l2].Ref == from {
						v := int(toTile.Links[l2].Edge)
						vi := toPoly.Verts[v] * 3
						return [3]float32{toTile.Verts[vi], toTile.Verts[vi+1], toTile.Verts[vi+2]}
					}
				}
				return [3]float32{}
			}

			// Get the edge vertices
			nv := int(fromPoly.VertCount)
			va := edge
			vb := (edge + 1) % nv

			via := fromPoly.Verts[va] * 3
			vib := fromPoly.Verts[vb] * 3
			l0, l1, l2 := fromTile.Verts[via], fromTile.Verts[via+1], fromTile.Verts[via+2]
			r0, r1, r2 := fromTile.Verts[vib], fromTile.Verts[vib+1], fromTile.Verts[vib+2]

			// If the link is at tile boundary, clamp the vertices to the link width.
			if fromTile.Links[l].Side != 0xff {
				if fromTile.Links[l].Bmin != 0 || fromTile.Links[l].Bmax != 255 {
					s := float32(1.0 / 255.0)
					tmin := float32(fromTile.Links[l].Bmin) * float32(s)
					tmax := float32(fromTile.Links[l].Bmax) * float32(s)
					lx := l0 + (r0-l0)*tmin
					ly := l1 + (r1-l1)*tmin
					lz := l2 + (r2-l2)*tmin
					rx := l0 + (r0-l0)*tmax
					ry := l1 + (r1-l1)*tmax
					rz := l2 + (r2-l2)*tmax
					return [3]float32{(lx + rx) * 0.5, (ly + ry) * 0.5, (lz + rz) * 0.5}
				}
			}
			return [3]float32{(l0 + r0) * 0.5, (l1 + r1) * 0.5, (l2 + r2) * 0.5}
		}
	}
	return [3]float32{}
}

// triArea2D calculates signed triangle area in 2D (XZ plane)
// Matches C++ triArea2D: (c-a)x(b-a) = acx*abz - abx*acz
func triArea2D(a, b, c [3]float32) float32 {
	abx := b[0] - a[0]
	abz := b[2] - a[2]
	acx := c[0] - a[0]
	acz := c[2] - a[2]
	return acx*abz - abx*acz
}

// appendVertex appends a vertex to the straight path
func appendVertex(pos [3]float32, flags uint8, ref PolyRef,
	straightPath []float32, straightPathFlags []uint8, straightPathRefs []PolyRef,
	straightPathCount *int, maxStraightPath int) error {

	// If pos equals last vertex (epsilon comparison), just update flags and ref (no duplicate)
	if *straightPathCount > 0 {
		lastIdx := (*straightPathCount - 1) * 3
		lastPos := [3]float32{straightPath[lastIdx], straightPath[lastIdx+1], straightPath[lastIdx+2]}
		if Vequal(lastPos, pos) {
			if straightPathFlags != nil {
				straightPathFlags[*straightPathCount-1] = flags
			}
			if straightPathRefs != nil {
				straightPathRefs[*straightPathCount-1] = ref
			}
			return ErrInProgress
		}
	}

	if *straightPathCount >= maxStraightPath {
		return ErrBufferTooSmall
	}

	idx := *straightPathCount
	copy(straightPath[idx*3:], pos[:])
	if straightPathFlags != nil {
		straightPathFlags[idx] = flags
	}
	if straightPathRefs != nil {
		straightPathRefs[idx] = ref
	}
	*straightPathCount++

	return ErrInProgress
}

// appendPortals appends portal crossings to the straight path
func (q *NavMeshQuery) appendPortals(startIdx, endIdx int, endPos [3]float32, path []PolyRef,
	straightPath []float32, straightPathFlags []uint8, straightPathRefs []PolyRef,
	straightPathCount *int, maxStraightPath int, options int) error {

	if *straightPathCount == 0 {
		return ErrInProgress
	}

	var startPos [3]float32
	copy(startPos[:], straightPath[(*straightPathCount-1)*3:])

	for i := startIdx; i < endIdx; i++ {
		from := path[i]
		fromTile, fromPoly := q.Nav.GetTileAndPolyByRefUnsafe(from)

		to := path[i+1]
		toTile, toPoly := q.Nav.GetTileAndPolyByRefUnsafe(to)

		var left, right [3]float32
		ok := getPortalPoints(from, to, fromTile, toTile, fromPoly, toPoly, &left, &right)
		if !ok {
			break
		}

		// Skip if only area crossings requested and areas are the same
		if options&StraightPathAreaCrossings != 0 {
			if fromPoly.GetArea() == toPoly.GetArea() {
				continue
			}
		}

		// Calculate intersection of segment (startPos-endPos) with portal (left-right)
		ok, _, t := intersectSegSeg2D(startPos, endPos, left, right)
		if ok {
			var pt [3]float32
			pt[0] = left[0] + t*(right[0]-left[0])
			pt[1] = left[1] + t*(right[1]-left[1])
			pt[2] = left[2] + t*(right[2]-left[2])

			stat := appendVertex(pt, 0, path[i+1],
				straightPath, straightPathFlags, straightPathRefs,
				straightPathCount, maxStraightPath)
			if stat != ErrInProgress {
				return stat
			}
		}
	}

	return ErrInProgress
}

// intersectSegSeg2D checks if two 2D segments intersect and returns parameters
func intersectSegSeg2D(p1, p2, q1, q2 [3]float32) (bool, float32, float32) {
	const eps float32 = 1e-6

	rx := p2[0] - p1[0]
	rz := p2[2] - p1[2]
	sx := q2[0] - q1[0]
	sz := q2[2] - q1[2]

	denom := rx*sz - rz*sx
	if denom > -eps && denom < eps {
		return false, 0, 0 // Parallel
	}

	qpx := q1[0] - p1[0]
	qpz := q1[2] - p1[2]

	s := (qpx*sz - qpz*sx) / denom
	t := (qpx*rz - qpz*rx) / denom

	return s >= 0 && s <= 1 && t >= 0 && t <= 1, s, t
}

func (q *NavMeshQuery) getPathToNode(endNode *Node, path []PolyRef, maxPath int) int {
	n := 0
	node := endNode
	for node != nil && n < maxPath {
		path[n] = node.ID
		n++
		if node.Pidx == 0 {
			break
		}
		node = q.NodePool.GetNodeAtIdx(node.Pidx)
	}
	// Reverse path
	for i, j := 0, n-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return n
}

// FindStraightPath finds a straight path from start to end within the corridor.
func (q *NavMeshQuery) FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, straightPath []float32, straightPathFlags []uint8, straightPathRefs []PolyRef, maxStraightPath int, options int) (int, error) {
	if maxStraightPath <= 0 {
		return 0, ErrInvalidParam
	}
	if path == nil || pathSize == 0 {
		return 0, ErrInvalidParam
	}
	if len(straightPath) < maxStraightPath*3 || len(straightPathFlags) < maxStraightPath || len(straightPathRefs) < maxStraightPath {
		return 0, ErrBufferTooSmall
	}

	straightPathCount := 0

	// Clamp start position to first polygon boundary
	closestStartPos, err := q.closestPointOnPolyBoundary(path[0], startPos)
	if err != nil {
		return 0, ErrInvalidParam
	}

	// Clamp end position to last polygon boundary
	closestEndPos, err := q.closestPointOnPolyBoundary(path[pathSize-1], endPos)
	if err != nil {
		return 0, ErrInvalidParam
	}

	// Add start point
	stat := appendVertex(closestStartPos, StraightPathStart, path[0],
		straightPath, straightPathFlags, straightPathRefs,
		&straightPathCount, maxStraightPath)
	if stat != ErrInProgress {
		return straightPathCount, stat
	}

	if pathSize > 1 {
		var portalApex, portalLeft, portalRight [3]float32
		portalApex = closestStartPos
		portalLeft = portalApex
		portalRight = portalApex
		apexIndex := 0
		leftIndex := 0
		rightIndex := 0

		var leftPolyType, rightPolyType uint8
		leftPolyRef := path[0]
		rightPolyRef := path[0]

		for i := 0; i < pathSize; i++ {
			var ok bool
			var left, right [3]float32
			var toType uint8

			if i+1 < pathSize {
				// Get portal points between current and next polygon
				fromTile, fromPoly := q.Nav.GetTileAndPolyByRefUnsafe(path[i])
				toTile, toPoly := q.Nav.GetTileAndPolyByRefUnsafe(path[i+1])
				ok = getPortalPoints(path[i], path[i+1], fromTile, toTile, fromPoly, toPoly, &left, &right)
				if !ok {
					// Failed to get portal points - clamp end point and return
					closestEndPos, err = q.closestPointOnPolyBoundary(path[i], endPos)
					if err != nil {
						return straightPathCount, ErrInvalidParam
					}

					// Append portals along current straight path segment
					if options&(StraightPathAreaCrossings|StraightPathAllCrossings) != 0 {
						q.appendPortals(apexIndex, i, closestEndPos, path,
							straightPath, straightPathFlags, straightPathRefs,
							&straightPathCount, maxStraightPath, options)
					}

					appendVertex(closestEndPos, 0, path[i],
						straightPath, straightPathFlags, straightPathRefs,
						&straightPathCount, maxStraightPath)

					return straightPathCount, ErrPartialResult
				}

				// If starting really close to the portal, advance
				if i == 0 {
					d, _ := distPtSegSqr2D(portalApex, left, right)
					if d < 0.001*0.001 {
						continue
					}
				}
			} else {
				// End of the path - use closest end position as portal
				left = closestEndPos
				right = closestEndPos
				toType = PolyTypeGround
			}

			// Right vertex check
			if triArea2D(portalApex, portalRight, right) <= 0.0 {
				if portalApex == portalRight || triArea2D(portalApex, portalLeft, right) > 0.0 {
					// Update portal right
					portalRight = right
					if i+1 < pathSize {
						rightPolyRef = path[i+1]
					} else {
						rightPolyRef = 0
					}
					rightPolyType = toType
					rightIndex = i
				} else {
					// Append portals along current straight path segment
					if options&(StraightPathAreaCrossings|StraightPathAllCrossings) != 0 {
						stat = q.appendPortals(apexIndex, leftIndex, portalLeft, path,
							straightPath, straightPathFlags, straightPathRefs,
							&straightPathCount, maxStraightPath, options)
						if stat != ErrInProgress {
							return straightPathCount, stat
						}
					}

					portalApex = portalLeft
					apexIndex = leftIndex

					var flags uint8
					if rightPolyRef == 0 {
						flags = StraightPathEnd
					} else if rightPolyType == PolyTypeOffMeshConnection {
						flags = StraightPathOffMeshConnection
					}
					ref := rightPolyRef

					stat = appendVertex(portalApex, flags, ref,
						straightPath, straightPathFlags, straightPathRefs,
						&straightPathCount, maxStraightPath)
					if stat != ErrInProgress {
						return straightPathCount, stat
					}

					portalLeft = portalApex
					portalRight = portalApex
					leftIndex = apexIndex
					rightIndex = apexIndex

					// Restart from apex
					i = apexIndex
					continue
				}
			}

			// Left vertex check
			if triArea2D(portalApex, portalLeft, left) >= 0.0 {
				if portalApex == portalLeft || triArea2D(portalApex, portalRight, left) < 0.0 {
					// Update portal left
					portalLeft = left
					if i+1 < pathSize {
						leftPolyRef = path[i+1]
					} else {
						leftPolyRef = 0
					}
					leftPolyType = toType
					leftIndex = i
				} else {
					// Append portals along current straight path segment
					if options&(StraightPathAreaCrossings|StraightPathAllCrossings) != 0 {
						stat = q.appendPortals(apexIndex, rightIndex, portalRight, path,
							straightPath, straightPathFlags, straightPathRefs,
							&straightPathCount, maxStraightPath, options)
						if stat != ErrInProgress {
							return straightPathCount, stat
						}
					}

					portalApex = portalRight
					apexIndex = rightIndex

					var flags uint8
					if leftPolyRef == 0 {
						flags = StraightPathEnd
					} else if leftPolyType == PolyTypeOffMeshConnection {
						flags = StraightPathOffMeshConnection
					}
					ref := leftPolyRef

					stat = appendVertex(portalApex, flags, ref,
						straightPath, straightPathFlags, straightPathRefs,
						&straightPathCount, maxStraightPath)
					if stat != ErrInProgress {
						return straightPathCount, stat
					}

					portalLeft = portalApex
					portalRight = portalApex
					leftIndex = apexIndex
					rightIndex = apexIndex

					// Restart from apex
					i = apexIndex
					continue
				}
			}
		}

		// Append portals along the current straight path segment
		if options&(StraightPathAreaCrossings|StraightPathAllCrossings) != 0 {
			q.appendPortals(apexIndex, pathSize-1, closestEndPos, path,
				straightPath, straightPathFlags, straightPathRefs,
				&straightPathCount, maxStraightPath, options)
		}
	}

	// Add end point
	appendVertex(closestEndPos, StraightPathEnd, 0,
		straightPath, straightPathFlags, straightPathRefs,
		&straightPathCount, maxStraightPath)

	return straightPathCount, nil
}

// MoveAlongSurface moves from the start position along the surface to find a position
// constrained by the navigation mesh.
// Uses BFS with a simple stack (matching C++ dtNavMeshQuery::moveAlongSurface).
func (q *NavMeshQuery) MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result []float32, visited []PolyRef, maxVisitedSize int) (int, error) {
	if startRef == 0 || filter == nil || result == nil || maxVisitedSize <= 0 {
		return 0, ErrInvalidParam
	}
	if !Visfinite(startPos) || !Visfinite(endPos) {
		return 0, ErrInvalidParam
	}

	q.TinyNodePool.Clear()

	const maxStack = 48
	stack := make([]*Node, 0, maxStack)

	startNode := q.TinyNodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeClosed
	stack = append(stack, startNode)

	bestPos := startPos
	bestDist := float32(math.MaxFloat32)
	var bestNode *Node

	// Search constraints.
	searchPos := Vlerp(startPos, endPos, 0.5)
	searchRadSqr := Vdist(startPos, endPos)
	searchRadSqr = searchRadSqr/2.0 + 0.001
	searchRadSqr *= searchRadSqr

	var verts [VertsPerPolygon * 3]float32

	for len(stack) > 0 {
		// Pop front.
		curNode := stack[0]
		stack = stack[1:]

		// Get poly and tile.
		curRef := curNode.ID
		curTile, curPoly := q.Nav.GetTileAndPolyByRefUnsafe(curRef)

		// Collect vertices.
		nverts := int(curPoly.VertCount)
		for i := 0; i < nverts; i++ {
			copy(verts[i*3:i*3+3], curTile.Verts[curPoly.Verts[i]*3:curPoly.Verts[i]*3+3])
		}

		// If target is inside the poly, stop search.
		if PointInPolygon(endPos, verts[:], nverts) {
			bestNode = curNode
			bestPos = endPos
			break
		}

		// Find wall edges and find nearest point inside the walls.
		for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
			// Find links to neighbours.
			const maxNeis = 8
			neis := make([]PolyRef, 0, maxNeis)

			if curPoly.Neis[j]&ExtLink != 0 {
				// Tile border.
				for l := curPoly.FirstLink; l != NullLink; l = curTile.Links[l].Next {
					link := &curTile.Links[l]
					if int(link.Edge) == j {
						if link.Ref != 0 {
							neiTile, neiPoly := q.Nav.GetTileAndPolyByRefUnsafe(link.Ref)
							if filter.PassFilter(link.Ref, neiTile, neiPoly) {
								if len(neis) < maxNeis {
									neis = append(neis, link.Ref)
								}
							}
						}
					}
				}
			} else if curPoly.Neis[j] != 0 {
				// Internal edge.
				idx := uint32(curPoly.Neis[j] - 1)
				ref := q.Nav.GetPolyRefBase(curTile) | PolyRef(idx)
				if filter.PassFilter(ref, curTile, &curTile.Polys[idx]) {
					neis = append(neis, ref)
				}
			}

			if len(neis) == 0 {
				// Wall edge, calc distance.
				vj := Vcopy(verts[j*3 : j*3+3])
				vi := Vcopy(verts[i*3 : i*3+3])
				distSqr, tseg := DistancePtSegSqr2D(endPos, vj, vi)
				if distSqr < bestDist {
					// Update nearest distance.
					bestPos = Vlerp(vj, vi, tseg)
					bestDist = distSqr
					bestNode = curNode
				}
			} else {
				for k := 0; k < len(neis); k++ {
					// Skip if no node can be allocated.
					neighbourNode := q.TinyNodePool.GetNode(neis[k], 0)
					if neighbourNode == nil {
						continue
					}
					// Skip if already visited.
					if neighbourNode.Flags&NodeClosed != 0 {
						continue
					}

					// Skip the link if it is too far from search constraint.
					vj := Vcopy(verts[j*3 : j*3+3])
					vi := Vcopy(verts[i*3 : i*3+3])
					distSqr, _ := DistancePtSegSqr2D(searchPos, vj, vi)
					if distSqr > searchRadSqr {
						continue
					}

					// Mark as the node as visited and push to queue.
					if len(stack) < maxStack {
						neighbourNode.Pidx = q.TinyNodePool.GetNodeIdx(curNode)
						neighbourNode.Flags |= NodeClosed
						stack = append(stack, neighbourNode)
					}
				}
			}
		}
	}

	// Build result path (polygon centers + bestPos at the end).
	n := 0
	if bestNode != nil {
		// Reverse the path from bestNode to start using pidx.
		var prev *Node
		node := bestNode
		for node != nil {
			next := q.TinyNodePool.GetNodeAtIdx(node.Pidx)
			node.Pidx = q.TinyNodePool.GetNodeIdx(prev)
			prev = node
			node = next
		}

		// Store result: position for each visited polygon (startPos for start, center for others).
		node = prev
		pathIdx := 0
		for node != nil && pathIdx < maxVisitedSize {
			visited[pathIdx] = node.ID

			var pt [3]float32
			if node.ID == startRef {
				pt = startPos
			} else {
				tile, poly := q.Nav.GetTileAndPolyByRefUnsafe(node.ID)
				pt = CalcPolyCenter(poly.Verts[:poly.VertCount], tile.Verts)
				pt[1] = startPos[1]
			}
			copy(result[pathIdx*3:], pt[:])

			pathIdx++
			node = q.TinyNodePool.GetNodeAtIdx(node.Pidx)
		}

		// Overwrite the last position with bestPos (closest to endPos).
		if pathIdx > 0 {
			copy(result[(pathIdx-1)*3:], bestPos[:])
		}

		n = pathIdx
	}

	return n, nil
}

// Raycast performs a raycast along the navigation mesh surface.
// Uses dtIntersectSegmentPoly2D-style polygon edge walking
// (matching C++ dtNavMeshQuery::raycast).
func (q *NavMeshQuery) Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32, prevRef PolyRef, hit *RaycastHit) error {
	if startRef == 0 || filter == nil || hit == nil {
		return ErrInvalidParam
	}
	if !Visfinite(startPos) || !Visfinite(endPos) {
		return ErrInvalidParam
	}
	if prevRef != 0 && !q.Nav.IsValidPolyRef(prevRef) {
		return ErrInvalidParam
	}

	hit.T = 0
	hit.HitNormal = [3]float32{}
	hit.PathCount = 0
	hit.PathCost = 0

	dir := Vsub(endPos, startPos)
	curPos := startPos
	curRef := startRef
	curTile, curPoly := q.Nav.GetTileAndPolyByRefUnsafe(curRef)

	var prevTile *MeshTile
	var prevPoly *Poly
	if prevRef != 0 {
		prevTile, prevPoly = q.Nav.GetTileAndPolyByRefUnsafe(prevRef)
	} else {
		prevTile = curTile
		prevPoly = curPoly
	}

	var verts [(VertsPerPolygon + 1) * 3]float32
	status := error(nil)
	n := 0

	for curRef != 0 {
		// Collect vertices of current polygon.
		nv := int(curPoly.VertCount)
		for i := 0; i < nv; i++ {
			vi := curPoly.Verts[i] * 3
			verts[i*3] = curTile.Verts[vi]
			verts[i*3+1] = curTile.Verts[vi+1]
			verts[i*3+2] = curTile.Verts[vi+2]
		}

		// Cast ray against current polygon.
		hitRes, _, tmax, _, segMax := IntersectSegmentPoly2D(startPos, endPos, verts[:], nv)
		if !hitRes {
			// Could not hit the polygon, keep the old t and report hit.
			hit.PathCount = n
			return status
		}

		hit.HitEdgeIndex = segMax

		// Keep track of furthest t so far.
		if tmax > hit.T {
			hit.T = tmax
		}

		// Store visited polygons.
		if n < MaxRaycastPathSize {
			hit.Path[n] = curRef
			n++
		} else {
			status = ErrBufferTooSmall
		}

		// Ray end is completely inside the polygon.
		if segMax == -1 {
			hit.T = float32(math.MaxFloat32)
			hit.PathCount = n

			// Add the cost.
			if options&RaycastUseCosts != 0 {
				hit.PathCost += filter.GetCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, curRef, curTile, curPoly)
			}
			return status
		}

		// Follow neighbours.
		var nextRef PolyRef
		var nextTile *MeshTile
		var nextPoly *Poly

		for l := curPoly.FirstLink; l != NullLink; l = curTile.Links[l].Next {
			link := &curTile.Links[l]

			// Find link which contains this edge.
			if int(link.Edge) != segMax {
				continue
			}

			// Get pointer to the next polygon.
			nextTile, nextPoly = q.Nav.GetTileAndPolyByRefUnsafe(link.Ref)

			// Skip off-mesh connections.
			if nextPoly.GetType() == PolyTypeOffMeshConnection {
				continue
			}

			// Skip links based on filter.
			if !filter.PassFilter(link.Ref, nextTile, nextPoly) {
				continue
			}

			// If the link is internal, just return the ref.
			if link.Side == 0xff {
				nextRef = link.Ref
				break
			}

			// If the link is at tile boundary,
			// Check if the link spans the whole edge, and accept.
			if link.Bmin == 0 && link.Bmax == 255 {
				nextRef = link.Ref
				break
			}

			// Check for partial edge links.
			v0 := curPoly.Verts[link.Edge]
			v1 := curPoly.Verts[(link.Edge+1)%uint8(curPoly.VertCount)]
			vi0 := v0 * 3
			vi1 := v1 * 3
			left := [3]float32{curTile.Verts[vi0], curTile.Verts[vi0+1], curTile.Verts[vi0+2]}
			right := [3]float32{curTile.Verts[vi1], curTile.Verts[vi1+1], curTile.Verts[vi1+2]}

			// Check that the intersection lies inside the link portal.
			if link.Side == 0 || link.Side == 4 {
				// Calculate link size.
				s := float32(1.0 / 255.0)
				lmin := left[2] + (right[2]-left[2])*(float32(link.Bmin)*s)
				lmax := left[2] + (right[2]-left[2])*(float32(link.Bmax)*s)
				if lmin > lmax {
					lmin, lmax = lmax, lmin
				}
				// Find Z intersection.
				z := startPos[2] + (endPos[2]-startPos[2])*tmax
				if z >= lmin && z <= lmax {
					nextRef = link.Ref
					break
				}
			} else if link.Side == 2 || link.Side == 6 {
				// Calculate link size.
				s := float32(1.0 / 255.0)
				lmin := left[0] + (right[0]-left[0])*(float32(link.Bmin)*s)
				lmax := left[0] + (right[0]-left[0])*(float32(link.Bmax)*s)
				if lmin > lmax {
					lmin, lmax = lmax, lmin
				}
				// Find X intersection.
				x := startPos[0] + (endPos[0]-startPos[0])*tmax
				if x >= lmin && x <= lmax {
					nextRef = link.Ref
					break
				}
			}
		}

		// Add cost.
		if options&RaycastUseCosts != 0 {
			lastPos := curPos
			curPos = Vmad(startPos, dir, hit.T)
			// Correct height.
			e1 := Vcopy(verts[segMax*3 : segMax*3+3])
			e2Idx := (segMax + 1) % nv
			e2 := Vcopy(verts[e2Idx*3 : e2Idx*3+3])
			eDir := Vsub(e2, e1)
			diff := Vsub(curPos, e1)
			var s float32
			if eDir[0]*eDir[0] > eDir[2]*eDir[2] {
				s = diff[0] / eDir[0]
			} else {
				s = diff[2] / eDir[2]
			}
			curPos[1] = e1[1] + eDir[1]*s

			hit.PathCost += filter.GetCost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly)
		}

		if nextRef == 0 {
			// No neighbour, we hit a wall.
			// Calculate hit normal.
			a := segMax
			b := segMax + 1
			if b >= nv {
				b = 0
			}
			va := Vcopy(verts[a*3 : a*3+3])
			vb := Vcopy(verts[b*3 : b*3+3])
			dx := vb[0] - va[0]
			dz := vb[2] - va[2]
			hit.HitNormal = Vnormalize([3]float32{dz, 0, -dx})

			hit.PathCount = n
			return status
		}

		// Move to next polygon.
		prevRef = curRef
		prevTile = curTile
		prevPoly = curPoly
		curRef = nextRef
		curTile = nextTile
		curPoly = nextPoly
	}

	hit.PathCount = n
	return status
}

// RaycastHit stores the result of a raycast operation.
type RaycastHit struct {
	T            float32
	HitNormal    [3]float32
	HitCount     int
	Path         [MaxRaycastPathSize]PolyRef
	PathCount    int
	Pos          [3]float32
	Cost         float32
	PathCost     float32
	HitEdgeIndex int
}

const MaxRaycastPathSize = 256

// FindPolysAroundCircle finds polygons within a circle around a position.
// Uses Dijkstra expansion (matching C++ dtNavMeshQuery::findPolysAroundCircle).
func (q *NavMeshQuery) FindPolysAroundCircle(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error) {
	if startRef == 0 {
		return 0, ErrInvalidParam
	}
	if !Visfinite(centerPos) {
		return 0, ErrInvalidParam
	}
	if radius < 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil || maxResult < 0 {
		return 0, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pos = centerPos
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	status := error(nil)
	n := 0
	radiusSqr := radius * radius

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		// Get poly and tile.
		bestRef := bestNode.ID
		bestTile, bestPoly := q.Nav.GetTileAndPolyByRefUnsafe(bestRef)

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		var parentTile *MeshTile
		var parentPoly *Poly
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}
		if parentRef != 0 {
			parentTile, parentPoly = q.Nav.GetTileAndPolyByRefUnsafe(parentRef)
		}

		if n < maxResult {
			if resultRef != nil {
				resultRef[n] = bestRef
			}
			if resultParent != nil {
				resultParent[n] = parentRef
			}
			if resultCost != nil {
				resultCost[n] = bestNode.Total
			}
			n++
		} else {
			status = ErrBufferTooSmall
		}

		for l := bestPoly.FirstLink; l != NullLink; l = bestTile.Links[l].Next {
			link := &bestTile.Links[l]
			neighbourRef := link.Ref
			// Skip invalid neighbours and do not follow back to parent.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			// Expand to neighbour.
			neighbourTile, neighbourPoly := q.Nav.GetTileAndPolyByRefUnsafe(neighbourRef)

			// Do not advance if the polygon is excluded by the filter.
			if !filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// Find edge and calc distance to the edge.
			var va, vb [3]float32
			ok := getPortalPoints(bestRef, neighbourRef, bestTile, neighbourTile, bestPoly, neighbourPoly, &va, &vb)
			if !ok {
				continue
			}

			// If the circle is not touching the next polygon, skip it.
			distSqr, _ := distPtSegSqr2D(centerPos, va, vb)
			if distSqr > radiusSqr {
				continue
			}

			neighbourNode := q.NodePool.GetNode(neighbourRef, 0)
			if neighbourNode == nil {
				status = ErrOutOfNodes
				continue
			}

			if neighbourNode.Flags&NodeClosed != 0 {
				continue
			}

			// Set neighbour node position (midpoint of portal edge) on first encounter.
			if neighbourNode.Flags == 0 {
				neighbourNode.Pos = Vlerp(va, vb, 0.5)
			}

			cost := filter.GetCost(
				bestNode.Pos, neighbourNode.Pos,
				parentRef, parentTile, parentPoly,
				bestRef, bestTile, bestPoly,
				neighbourRef, neighbourTile, neighbourPoly)

			total := bestNode.Total + cost

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&NodeOpen) != 0 && total >= neighbourNode.Total {
				continue
			}

			neighbourNode.ID = neighbourRef
			neighbourNode.Pidx = q.NodePool.GetNodeIdx(bestNode)
			neighbourNode.Total = total

			if (neighbourNode.Flags & NodeOpen) != 0 {
				q.OpenList.Modify(neighbourNode)
			} else {
				neighbourNode.Flags = NodeOpen
				q.OpenList.Push(neighbourNode)
			}
		}
	}

	return n, status
}

// FindPolysAroundShape finds polygons within a convex shape around a position.
// FindPolysAroundShape finds polygons within a convex shape around a position.
// Uses Dijkstra expansion (matching C++ dtNavMeshQuery::findPolysAroundShape).
func (q *NavMeshQuery) FindPolysAroundShape(startRef PolyRef, verts []float32, nverts int, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error) {
	if startRef == 0 || verts == nil || nverts < 3 {
		return 0, ErrInvalidParam
	}
	if filter == nil || maxResult < 0 {
		return 0, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	// Compute center of the query polygon.
	var centerPos [3]float32
	for i := 0; i < nverts; i++ {
		centerPos[0] += verts[i*3]
		centerPos[1] += verts[i*3+1]
		centerPos[2] += verts[i*3+2]
	}
	s := 1.0 / float32(nverts)
	centerPos[0] *= s
	centerPos[1] *= s
	centerPos[2] *= s

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pos = centerPos
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	status := error(nil)
	n := 0

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		// Get poly and tile.
		bestRef := bestNode.ID
		bestTile, bestPoly := q.Nav.GetTileAndPolyByRefUnsafe(bestRef)

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		var parentTile *MeshTile
		var parentPoly *Poly
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}
		if parentRef != 0 {
			parentTile, parentPoly = q.Nav.GetTileAndPolyByRefUnsafe(parentRef)
		}

		if n < maxResult {
			if resultRef != nil {
				resultRef[n] = bestRef
			}
			if resultParent != nil {
				resultParent[n] = parentRef
			}
			if resultCost != nil {
				resultCost[n] = bestNode.Total
			}
			n++
		} else {
			status = ErrBufferTooSmall
		}

		for l := bestPoly.FirstLink; l != NullLink; l = bestTile.Links[l].Next {
			link := &bestTile.Links[l]
			neighbourRef := link.Ref
			// Skip invalid neighbours and do not follow back to parent.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			// Expand to neighbour.
			neighbourTile, neighbourPoly := q.Nav.GetTileAndPolyByRefUnsafe(neighbourRef)

			// Do not advance if the polygon is excluded by the filter.
			if !filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// Find edge and calc distance to the edge.
			var va, vb [3]float32
			ok := getPortalPoints(bestRef, neighbourRef, bestTile, neighbourTile, bestPoly, neighbourPoly, &va, &vb)
			if !ok {
				continue
			}

			// If the portal edge does not intersect the query polygon, skip.
			hit, _, _, _, _ := IntersectSegmentPoly2D(va, vb, verts, nverts)
			if !hit {
				continue
			}

			neighbourNode := q.NodePool.GetNode(neighbourRef, 0)
			if neighbourNode == nil {
				status = ErrOutOfNodes
				continue
			}

			if neighbourNode.Flags&NodeClosed != 0 {
				continue
			}

			// Set neighbour node position (midpoint of portal edge) on first encounter.
			if neighbourNode.Flags == 0 {
				neighbourNode.Pos = Vlerp(va, vb, 0.5)
			}

			cost := filter.GetCost(
				bestNode.Pos, neighbourNode.Pos,
				parentRef, parentTile, parentPoly,
				bestRef, bestTile, bestPoly,
				neighbourRef, neighbourTile, neighbourPoly)

			total := bestNode.Total + cost

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&NodeOpen) != 0 && total >= neighbourNode.Total {
				continue
			}

			neighbourNode.ID = neighbourRef
			neighbourNode.Pidx = q.NodePool.GetNodeIdx(bestNode)
			neighbourNode.Total = total

			if (neighbourNode.Flags & NodeOpen) != 0 {
				q.OpenList.Modify(neighbourNode)
			} else {
				neighbourNode.Flags = NodeOpen
				q.OpenList.Push(neighbourNode)
			}
		}
	}

	return n, status
}

// FindDistanceToWall finds the distance from a position to the nearest wall.
func (q *NavMeshQuery) FindDistanceToWall(startRef PolyRef, centerPos [3]float32, maxWallDistance float32, filter *QueryFilter) (float32, [3]float32, [3]float32, error) {
	if startRef == 0 {
		return 0, [3]float32{}, [3]float32{}, ErrInvalidParam
	}
	if filter == nil {
		return 0, [3]float32{}, [3]float32{}, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, [3]float32{}, [3]float32{}, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	radiusSqr := maxWallDistance * maxWallDistance
	foundDist := float32(math.MaxFloat32)
	var foundPos [3]float32
	var foundNormal [3]float32

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		tile, poly, _ := q.Nav.GetTileAndPolyByRef(bestNode.ID)

		nv := int(poly.VertCount)
		// Check each edge of the polygon
		for i := 0; i < nv; i++ {
			// Check if this edge is a wall (not connected)
			isConnected := false
			for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
				if tile.Links[l].Edge == uint8(i) {
					isConnected = true
					break
				}
			}

			if isConnected {
				continue
			}

			va := Vcopy(tile.Verts[poly.Verts[i]*3 : poly.Verts[i]*3+3])
			vb := Vcopy(tile.Verts[poly.Verts[(i+1)%nv]*3 : poly.Verts[(i+1)%nv]*3+3])

			d, t := DistancePtSegSqr2D(centerPos, va, vb)
			if d < foundDist {
				foundDist = d
				// Calculate closest point on edge
				foundPos = Vlerp(va, vb, t)
				// Calculate normal pointing inward
				foundNormal[0] = -(vb[2] - va[2])
				foundNormal[1] = 0
				foundNormal[2] = vb[0] - va[0]
				foundNormal = Vnormalize(foundNormal)
			}
		}

		// Expand to neighbours
		for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
			link := &tile.Links[l]
			if link.Ref == 0 {
				continue
			}
			neiTile, neiPoly := q.Nav.GetTileAndPolyByRefUnsafe(link.Ref)
			if !filter.PassFilter(link.Ref, neiTile, neiPoly) {
				continue
			}

			var closestPt [3]float32
			closestPt, _ = q.Nav.closestPointOnPoly(link.Ref, centerPos)
			distSqr := Vdist2DSqr(closestPt, centerPos)

			if distSqr > radiusSqr {
				continue
			}

			nei := q.NodePool.GetNode(link.Ref, 0)
			if nei == nil {
				continue
			}
			if nei.Flags&NodeClosed != 0 {
				continue
			}

			if (nei.Flags & NodeOpen) != 0 {
				if nei.Total > distSqr {
					nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
					nei.Total = distSqr
					nei.ID = link.Ref
					q.OpenList.Modify(nei)
				}
			} else {
				nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
				nei.Total = distSqr
				nei.ID = link.Ref
				nei.Flags = NodeOpen
				q.OpenList.Push(nei)
			}
		}
	}

	return float32(math.Sqrt(float64(foundDist))), foundPos, foundNormal, nil
}

// FindLocalNeighbourhood finds the polygons reachable from startRef within a radius.
// Uses connectivity-based BFS (matching C++ dtNavMeshQuery::findLocalNeighbourhood).
func (q *NavMeshQuery) FindLocalNeighbourhood(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, maxResult int) (int, error) {
	if startRef == 0 {
		return 0, ErrInvalidParam
	}
	if !Visfinite(centerPos) {
		return 0, ErrInvalidParam
	}
	if radius < 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil || maxResult < 0 {
		return 0, ErrInvalidParam
	}

	q.TinyNodePool.Clear()

	startNode := q.TinyNodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.ID = startRef
	startNode.Flags = NodeClosed

	const maxStack = 48
	stack := make([]*Node, 0, maxStack)
	stack = append(stack, startNode)

	radiusSqr := radius * radius

	var pa [VertsPerPolygon * 3]float32
	var pb [VertsPerPolygon * 3]float32

	status := error(nil)

	n := 0
	if n < maxResult {
		resultRef[n] = startNode.ID
		if resultParent != nil {
			resultParent[n] = 0
		}
		n++
	} else {
		status = ErrBufferTooSmall
	}

	for len(stack) > 0 {
		// Pop front (BFS: use as a queue).
		curNode := stack[0]
		stack = stack[1:]

		curRef := curNode.ID
		curTile, curPoly := q.Nav.GetTileAndPolyByRefUnsafe(curRef)

		for l := curPoly.FirstLink; l != NullLink; l = curTile.Links[l].Next {
			link := &curTile.Links[l]
			neighbourRef := link.Ref
			if neighbourRef == 0 {
				continue
			}

			neighbourNode := q.TinyNodePool.GetNode(neighbourRef, 0)
			if neighbourNode == nil {
				continue
			}
			if neighbourNode.Flags&NodeClosed != 0 {
				continue
			}

			neighbourTile, neighbourPoly := q.Nav.GetTileAndPolyByRefUnsafe(neighbourRef)

			// Skip off-mesh connections.
			if neighbourPoly.GetType() == PolyTypeOffMeshConnection {
				continue
			}

			// Do not advance if the polygon is excluded by the filter.
			if !filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// Find edge and calc distance to the edge.
			var va, vb [3]float32
			ok := getPortalPoints(curRef, neighbourRef, curTile, neighbourTile, curPoly, neighbourPoly, &va, &vb)
			if !ok {
				continue
			}

			// If the circle is not touching the next polygon, skip it.
			distSqr, _ := distPtSegSqr2D(centerPos, va, vb)
			if distSqr > radiusSqr {
				continue
			}

			// Mark node visited.
			neighbourNode.Flags |= NodeClosed
			neighbourNode.Pidx = q.TinyNodePool.GetNodeIdx(curNode)

			// Check that the polygon does not collide with existing polygons.
			npa := int(neighbourPoly.VertCount)
			for k := 0; k < npa; k++ {
				vIdx := neighbourPoly.Verts[k]
				pa[k*3] = neighbourTile.Verts[vIdx*3]
				pa[k*3+1] = neighbourTile.Verts[vIdx*3+1]
				pa[k*3+2] = neighbourTile.Verts[vIdx*3+2]
			}

			overlap := false
			for j := 0; j < n; j++ {
				pastRef := resultRef[j]

				// Connected polys do not overlap.
				connected := false
				for k := curPoly.FirstLink; k != NullLink; k = curTile.Links[k].Next {
					if curTile.Links[k].Ref == pastRef {
						connected = true
						break
					}
				}
				if connected {
					continue
				}

				// Potentially overlapping.
				pastTile, pastPoly := q.Nav.GetTileAndPolyByRefUnsafe(pastRef)

				npb := int(pastPoly.VertCount)
				for k := 0; k < npb; k++ {
					vIdx := pastPoly.Verts[k]
					pb[k*3] = pastTile.Verts[vIdx*3]
					pb[k*3+1] = pastTile.Verts[vIdx*3+1]
					pb[k*3+2] = pastTile.Verts[vIdx*3+2]
				}

				if OverlapPolyPoly2D(pa[:npa*3], npa, pb[:npb*3], npb) {
					overlap = true
					break
				}
			}
			if overlap {
				continue
			}

			// This poly is fine, store and advance.
			if n < maxResult {
				resultRef[n] = neighbourRef
				if resultParent != nil {
					resultParent[n] = curRef
				}
				n++
			} else {
				status = ErrBufferTooSmall
			}

			if len(stack) < maxStack {
				stack = append(stack, neighbourNode)
			}
		}
	}

	return n, status
}

func (q *NavMeshQuery) GetPolyWallSegments(ref PolyRef, filter *QueryFilter, segs []NeighbourSeg, maxSegs int) (int, error) {
	if ref == 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil {
		return 0, ErrInvalidParam
	}

	tile, poly, err := q.Nav.GetTileAndPolyByRef(ref)
	if err != nil {
		return 0, err
	}

	n := 0
	nv := int(poly.VertCount)
	for i := 0; i < nv; i++ {
		// Find if this edge is connected
		connected := false
		for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
			if tile.Links[l].Edge == uint8(i) {
				connected = true
				break
			}
		}

		va := Vcopy(tile.Verts[poly.Verts[i]*3 : poly.Verts[i]*3+3])
		vb := Vcopy(tile.Verts[poly.Verts[(i+1)%nv]*3 : poly.Verts[(i+1)%nv]*3+3])

		if !connected {
			if n < maxSegs {
				copy(segs[n].Seg[0:3], va[:])
				copy(segs[n].Seg[3:6], vb[:])
				segs[n].Ref = 0
				n++
			}
		} else {
			// Find the neighbour polygon
			for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
				if tile.Links[l].Edge == uint8(i) {
					if n < maxSegs {
						segs[n].Ref = tile.Links[l].Ref
						copy(segs[n].Seg[0:3], va[:])
						copy(segs[n].Seg[3:6], vb[:])
						n++
					}
					break
				}
			}
		}
	}

	return n, nil
}

// ClosestPointOnPoly projects a point onto a polygon.
func (q *NavMeshQuery) ClosestPointOnPoly(ref PolyRef, pos [3]float32) ([3]float32, bool, error) {
	if ref == 0 {
		return [3]float32{}, false, ErrInvalidParam
	}
	tile, poly, err := q.Nav.GetTileAndPolyByRef(ref)
	if err != nil {
		return [3]float32{}, false, err
	}
	if poly.GetType() == PolyTypeOffMeshConnection {
		// For off-mesh connection, project onto line segment
		v0 := Vcopy(tile.Verts[poly.Verts[0]*3 : poly.Verts[0]*3+3])
		v1 := Vcopy(tile.Verts[poly.Verts[1]*3 : poly.Verts[1]*3+3])
		_, t := DistancePtSegSqr2D(pos, v0, v1)
		closest := Vlerp(v0, v1, t)
		return closest, false, nil
	}

	// Find height on polygon's detail mesh
	var h float32
	h, hasHeight := q.Nav.getPolyHeight(tile, poly, pos)
	if hasHeight {
		var closest [3]float32
		closest[0] = pos[0]
		closest[1] = h
		closest[2] = pos[2]
		return closest, true, nil
	}

	// Find closest point on polygon boundary
	closest, _ := q.Nav.closestPointOnPoly(ref, pos)
	return closest, false, nil
}

// GetAttachedNavMesh returns the attached navigation mesh.
func (q *NavMeshQuery) GetAttachedNavMesh() *NavMesh {
	return q.Nav
}

// ClosestPointOnPolyBoundary finds the closest point on the polygon boundary.
func (q *NavMeshQuery) ClosestPointOnPolyBoundary(ref PolyRef, pos [3]float32) ([3]float32, error) {
	if ref == 0 {
		return [3]float32{}, ErrInvalidParam
	}
	tile, poly, err := q.Nav.GetTileAndPolyByRef(ref)
	if err != nil {
		return [3]float32{}, err
	}

	nv := int(poly.VertCount)
	if nv == 0 {
		return [3]float32{}, ErrInvalidParam
	}

	bestDist := float32(math.MaxFloat32)
	bestT := float32(0)
	var bestVa, bestVb [3]float32

	for i := 0; i < nv; i++ {
		va := Vcopy(tile.Verts[poly.Verts[i]*3 : poly.Verts[i]*3+3])
		vb := Vcopy(tile.Verts[poly.Verts[(i+1)%nv]*3 : poly.Verts[(i+1)%nv]*3+3])
		d, t := DistancePtSegSqr2D(pos, va, vb)
		if d < bestDist {
			bestDist = d
			bestT = t
			bestVa = va
			bestVb = vb
		}
	}

	closest := Vlerp(bestVa, bestVb, bestT)
	return closest, nil
}

// GetPolyHeight returns the height of the polygon at the given position.
func (q *NavMeshQuery) GetPolyHeight(ref PolyRef, pos [3]float32) (float32, error) {
	if ref == 0 {
		return 0, ErrInvalidParam
	}
	tile, poly, err := q.Nav.GetTileAndPolyByRef(ref)
	if err != nil {
		return 0, err
	}

	if poly.GetType() == PolyTypeOffMeshConnection {
		// Return the height at the closest point on the connection
		v0 := Vcopy(tile.Verts[poly.Verts[0]*3 : poly.Verts[0]*3+3])
		v1 := Vcopy(tile.Verts[poly.Verts[1]*3 : poly.Verts[1]*3+3])
		_, t := DistancePtSegSqr2D(pos, v0, v1)
		h := v0[1] + (v1[1]-v0[1])*t
		return h, nil
	}

	h, ok := q.Nav.getPolyHeight(tile, poly, pos)
	if ok {
		return h, nil
	}
	return 0, ErrPointNotOnPoly
}

func (q *NavMeshQuery) FindNearestPoly(center, halfExtents [3]float32, filter *QueryFilter) (PolyRef, [3]float32, error) {
	if filter == nil {
		return 0, [3]float32{}, ErrInvalidParam
	}

	bmin := Vsub(center, halfExtents)
	bmax := Vadd(center, halfExtents)

	nearest := PolyRef(0)
	nearestDistSqr := float32(math.MaxFloat32)
	var nearestPtLocal [3]float32

	for tileIndex, maxTiles := 0, int(q.Nav.MaxTiles); tileIndex < maxTiles; tileIndex++ {
		tile := &q.Nav.Tiles[tileIndex]
		if tile.Header == nil {
			continue
		}

		var polys [128]PolyRef
		polyCount := q.Nav.queryPolygonsInTile(tile, bmin, bmax, polys[:], 128)

		for i := 0; i < polyCount; i++ {
			ref := polys[i]
			_, poly := q.Nav.GetTileAndPolyByRefUnsafe(ref)
			if !filter.PassFilter(ref, tile, poly) {
				continue
			}

			closestPt, posOverPoly := q.Nav.closestPointOnPoly(ref, center)

			diff := Vsub(center, closestPt)
			var d float32
			if posOverPoly {
				d = Abs(diff[1])
				d = Max(0.0, d-tile.Header.WalkableClimb)
				d = d * d
			} else {
				d = VlenSqr(diff)
			}

			if d < nearestDistSqr {
				nearestPtLocal = closestPt
				nearestDistSqr = d
				nearest = ref
			}
		}
	}

	if nearest != 0 {
		return nearest, nearestPtLocal, nil
	}
	return 0, [3]float32{}, ErrPolyNotFound
}

// FindRandomPoint finds a random point in the navigation mesh.
// Uses area-weighted reservoir sampling (matching C++ dtNavMeshQuery::findRandomPoint).
func (q *NavMeshQuery) FindRandomPoint(filter *QueryFilter, randomFunc func() float32) (PolyRef, [3]float32, error) {
	if filter == nil || randomFunc == nil {
		return 0, [3]float32{}, ErrInvalidParam
	}

	// Randomly pick one tile using reservoir sampling.
	var hitTile *MeshTile
	var tsum float32
	for i, maxTiles := 0, int(q.Nav.MaxTiles); i < maxTiles; i++ {
		t := &q.Nav.Tiles[i]
		if t.Header == nil {
			continue
		}
		tsum += 1.0 // each tile has equal weight
		if randomFunc()*tsum <= 1.0 {
			hitTile = t
		}
	}
	if hitTile == nil {
		return 0, [3]float32{}, ErrTileNotFound
	}

	// Randomly pick one polygon weighted by polygon area.
	var poly *Poly
	polyRef := PolyRef(0)
	base := q.Nav.GetPolyRefBase(hitTile)
	var areaSum float32

	for i := 0; i < int(hitTile.Header.PolyCount); i++ {
		p := &hitTile.Polys[i]
		// Do not return off-mesh connection polygons.
		if p.GetType() != PolyTypeGround {
			continue
		}
		// Must pass filter.
		ref := base | PolyRef(i)
		if !filter.PassFilter(ref, hitTile, p) {
			continue
		}

		// Calc area of the polygon.
		var polyArea float32
		for j := 2; j < int(p.VertCount); j++ {
			va := Vcopy(hitTile.Verts[p.Verts[0]*3 : p.Verts[0]*3+3])
			vb := Vcopy(hitTile.Verts[p.Verts[j-1]*3 : p.Verts[j-1]*3+3])
			vc := Vcopy(hitTile.Verts[p.Verts[j]*3 : p.Verts[j]*3+3])
			polyArea += TriArea2D(va, vb, vc)
		}

		// Choose random polygon weighted by area, using reservoir sampling.
		areaSum += polyArea
		if randomFunc()*areaSum <= polyArea {
			poly = p
			polyRef = ref
		}
	}

	if poly == nil {
		return 0, [3]float32{}, ErrPolyNotFound
	}

	// Randomly pick point on polygon.
	var verts [VertsPerPolygon * 3]float32
	nv := int(poly.VertCount)
	for j := 0; j < nv; j++ {
		copy(verts[j*3:j*3+3], hitTile.Verts[poly.Verts[j]*3:poly.Verts[j]*3+3])
	}

	areas := make([]float32, nv)
	s := randomFunc()
	t := randomFunc()
	resultPt := RandomPointInConvexPoly(verts[:], nv, areas, s, t)

	// Project point onto polygon to get proper height.
	closest, _, _ := q.ClosestPointOnPoly(polyRef, resultPt)
	resultPt = closest

	return polyRef, resultPt, nil
}

// FindRandomPointAroundCircle finds a random point within a circle around a position.
// Uses Dijkstra expansion with area-weighted reservoir sampling
// (matching C++ dtNavMeshQuery::findRandomPointAroundCircle).
func (q *NavMeshQuery) FindRandomPointAroundCircle(startRef PolyRef, centerPos [3]float32, maxRadius float32, filter *QueryFilter, randomFunc func() float32) (PolyRef, [3]float32, error) {
	if startRef == 0 || filter == nil || randomFunc == nil {
		return 0, [3]float32{}, ErrInvalidParam
	}
	if !Visfinite(centerPos) || maxRadius < 0 {
		return 0, [3]float32{}, ErrInvalidParam
	}

	startTile, startPoly := q.Nav.GetTileAndPolyByRefUnsafe(startRef)
	if !filter.PassFilter(startRef, startTile, startPoly) {
		return 0, [3]float32{}, ErrStartPolyNotPassFilter
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, [3]float32{}, ErrOutOfNodes
	}
	startNode.Pos = centerPos
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	radiusSqr := maxRadius * maxRadius
	var areaSum float32
	var randomTile *MeshTile
	var randomPoly *Poly
	randomPolyRef := PolyRef(0)

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		bestRef := bestNode.ID
		bestTile, bestPoly := q.Nav.GetTileAndPolyByRefUnsafe(bestRef)

		// Place random locations on ground.
		if bestPoly.GetType() == PolyTypeGround {
			// Calc area of the polygon.
			var polyArea float32
			for j := 2; j < int(bestPoly.VertCount); j++ {
				va := Vcopy(bestTile.Verts[bestPoly.Verts[0]*3 : bestPoly.Verts[0]*3+3])
				vb := Vcopy(bestTile.Verts[bestPoly.Verts[j-1]*3 : bestPoly.Verts[j-1]*3+3])
				vc := Vcopy(bestTile.Verts[bestPoly.Verts[j]*3 : bestPoly.Verts[j]*3+3])
				polyArea += TriArea2D(va, vb, vc)
			}
			// Choose random polygon weighted by area, using reservoir sampling.
			areaSum += polyArea
			if randomFunc()*areaSum <= polyArea {
				randomTile = bestTile
				randomPoly = bestPoly
				randomPolyRef = bestRef
			}
		}

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}

		for l := bestPoly.FirstLink; l != NullLink; l = bestTile.Links[l].Next {
			link := &bestTile.Links[l]
			neighbourRef := link.Ref
			// Skip invalid neighbours and do not follow back to parent.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			neighbourTile, neighbourPoly := q.Nav.GetTileAndPolyByRefUnsafe(neighbourRef)

			if !filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// Find edge and calc distance to the edge.
			var va, vb [3]float32
			ok := getPortalPoints(bestRef, neighbourRef, bestTile, neighbourTile, bestPoly, neighbourPoly, &va, &vb)
			if !ok {
				continue
			}

			// If the circle is not touching the next polygon, skip it.
			distSqr, _ := distPtSegSqr2D(centerPos, va, vb)
			if distSqr > radiusSqr {
				continue
			}

			neighbourNode := q.NodePool.GetNode(neighbourRef, 0)
			if neighbourNode == nil {
				continue
			}

			if neighbourNode.Flags&NodeClosed != 0 {
				continue
			}

			// Cost (simple distance-based for random search).
			if neighbourNode.Flags == 0 {
				neighbourNode.Pos = Vlerp(va, vb, 0.5)
			}
			total := bestNode.Total + Vdist(bestNode.Pos, neighbourNode.Pos)

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&NodeOpen) != 0 && total >= neighbourNode.Total {
				continue
			}

			neighbourNode.ID = neighbourRef
			neighbourNode.Pidx = q.NodePool.GetNodeIdx(bestNode)
			neighbourNode.Total = total

			if (neighbourNode.Flags & NodeOpen) != 0 {
				q.OpenList.Modify(neighbourNode)
			} else {
				neighbourNode.Flags = NodeOpen
				q.OpenList.Push(neighbourNode)
			}
		}
	}

	if randomPoly == nil {
		return 0, [3]float32{}, ErrPolyNotFound
	}

	// Randomly pick point on polygon.
	nv := int(randomPoly.VertCount)
	var verts [VertsPerPolygon * 3]float32
	for j := 0; j < nv; j++ {
		copy(verts[j*3:j*3+3], randomTile.Verts[randomPoly.Verts[j]*3:randomPoly.Verts[j]*3+3])
	}
	areas := make([]float32, nv)
	s := randomFunc()
	t := randomFunc()
	resultPt := RandomPointInConvexPoly(verts[:], nv, areas, s, t)

	// Project point onto polygon to get proper height.
	closest, _, _ := q.ClosestPointOnPoly(randomPolyRef, resultPt)
	resultPt = closest

	return randomPolyRef, resultPt, nil
}

func (q *NavMeshQuery) QueryPolygons(center, halfExtents [3]float32, filter *QueryFilter, polys []PolyRef, maxPolys int) (int, error) {

	if filter == nil {
		return 0, ErrInvalidParam
	}
	if maxPolys <= 0 {
		return 0, ErrInvalidParam
	}

	var bmin, bmax [3]float32
	bmin = Vsub(center, halfExtents)
	bmax = Vadd(center, halfExtents)

	n := 0
	for tileIndex, maxTiles := 0, int(q.Nav.MaxTiles); tileIndex < maxTiles; tileIndex++ {
		tile := &q.Nav.Tiles[tileIndex]
		if tile.Header == nil {
			continue
		}

		// Check tile bounds against query box
		if !OverlapBounds(bmin, bmax, tile.Header.Bmin, tile.Header.Bmax) {
			continue
		}

		polyCount := q.Nav.queryPolygonsInTile(tile, bmin, bmax, polys[n:], maxPolys-n)

		// Filter results
		k := n
		for i := 0; i < polyCount; i++ {
			ref := polys[k]
			_, poly := q.Nav.GetTileAndPolyByRefUnsafe(ref)
			if filter.PassFilter(ref, tile, poly) {
				k++
			}
		}
		n = k
	}

	if n > maxPolys {
		n = maxPolys
	}

	return n, nil
}

// IsValidPolyRef checks the validity of a polygon reference.
func (q *NavMeshQuery) IsValidPolyRef(ref PolyRef, filter *QueryFilter) bool {
	if ref == 0 {
		return false
	}
	tile, poly := q.Nav.GetTileAndPolyByRefUnsafe(ref)
	if tile == nil || poly == nil {
		return false
	}
	if !filter.PassFilter(ref, tile, poly) {
		return false
	}
	return true
}

// IsInClosedList checks if a given polygon is in the closed list.
func (q *NavMeshQuery) IsInClosedList(ref PolyRef) bool {
	node := q.NodePool.FindNode(ref, 0)
	if node == nil {
		return false
	}
	return (node.Flags & NodeClosed) != 0
}

// GetNodePool returns the node pool.
func (q *NavMeshQuery) GetNodePool() *NodePool {
	return q.NodePool
}

// GetOpenList returns the open list.
func (q *NavMeshQuery) GetOpenList() *NodeQueue {
	return q.OpenList
}

// Sliced pathfinding methods

// FindPathSliced begins a sliced pathfinding query.
func (q *NavMeshQuery) FindPathSliced(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
	if startRef == 0 || endRef == 0 {
		return ErrInvalidParam
	}
	if filter == nil {
		return ErrInvalidParam
	}

	// Validate input
	if !q.Nav.IsValidPolyRef(startRef) || !q.Nav.IsValidPolyRef(endRef) {
		return ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = Vdist(startPos, endPos) * H_SCALE
	startNode.Pos = startPos
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	q.QueryData.Err = ErrInProgress
	q.QueryData.StartRef = startRef
	q.QueryData.EndRef = endRef
	q.QueryData.StartPos = [3]float32{startPos[0], startPos[1], startPos[2]}
	q.QueryData.EndPos = [3]float32{endPos[0], endPos[1], endPos[2]}
	q.QueryData.Filter = filter
	q.QueryData.Options = options
	q.QueryData.RaycastLimitSqr = RayCastLimitProportions * RayCastLimitProportions
	q.QueryData.LastBestNode = startNode
	q.QueryData.LastBestNodeCost = startNode.Total

	return ErrInProgress
}

// UpdateSlicedPath updates the sliced pathfinding query by performing up to maxIter iterations.
func (q *NavMeshQuery) UpdateSlicedPath(maxIter int) error {
	// If a terminal error occurred (not "in progress"), return it.
	if q.QueryData.Err != nil && q.QueryData.Err != ErrInProgress {
		return q.QueryData.Err
	}
	// If the query completed, return nil.
	if q.QueryData.Err == nil {
		return nil
	}

	iter := 0
	for iter < maxIter && !q.OpenList.Empty() {
		iter++

		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		// Check if reached goal
		if bestNode.ID == q.QueryData.EndRef {
			q.QueryData.LastBestNode = bestNode
			q.QueryData.Err = nil
			return nil
		}

		// Get current poly and tile.
		bestRef := bestNode.ID
		bestTile, bestPoly := q.Nav.GetTileAndPolyByRefUnsafe(bestRef)

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		var parentTile *MeshTile
		var parentPoly *Poly
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}
		if parentRef != 0 {
			parentTile, parentPoly = q.Nav.GetTileAndPolyByRefUnsafe(parentRef)
		}

		for l := bestPoly.FirstLink; l != NullLink; l = bestTile.Links[l].Next {
			neighbourRef := bestTile.Links[l].Ref

			// Skip invalid ids and do not expand back to where we came from.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}

			// Get neighbour poly and tile.
			neighbourTile, neighbourPoly := q.Nav.GetTileAndPolyByRefUnsafe(neighbourRef)

			if !q.QueryData.Filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}

			// deal explicitly with crossing tile boundaries
			crossSide := uint8(0)
			if bestTile.Links[l].Side != 0xff {
				crossSide = bestTile.Links[l].Side >> 1
			}

			neighbourNode := q.NodePool.GetNode(neighbourRef, crossSide)
			if neighbourNode == nil {
				q.QueryData.Err = ErrOutOfNodes
				continue
			}

			// If the node is visited the first time, calculate node position.
			if neighbourNode.Flags == 0 {
				neighbourNode.Pos = q.getEdgeMidPoint(bestRef, neighbourRef, bestTile, neighbourTile, bestPoly, neighbourPoly)
			}

			// Calculate cost and heuristic.
			var cost float32
			var heuristic float32

			// Special case for last node.
			if neighbourRef == q.QueryData.EndRef {
				curCost := q.QueryData.Filter.GetCost(bestNode.Pos, neighbourNode.Pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				endCost := q.QueryData.Filter.GetCost(neighbourNode.Pos, q.QueryData.EndPos,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly,
					0, nil, nil)

				cost = bestNode.Cost + curCost + endCost
				heuristic = 0
			} else {
				curCost := q.QueryData.Filter.GetCost(bestNode.Pos, neighbourNode.Pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly)
				cost = bestNode.Cost + curCost
				heuristic = Vdist(neighbourNode.Pos, q.QueryData.EndPos) * H_SCALE
			}

			total := cost + heuristic

			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.Flags&NodeOpen) != 0 && total >= neighbourNode.Total {
				continue
			}
			// The node is already visited and processed, and the new result is worse, skip.
			if (neighbourNode.Flags&NodeClosed) != 0 && total >= neighbourNode.Total {
				continue
			}

			// Add or update the node.
			neighbourNode.Pidx = q.NodePool.GetNodeIdx(bestNode)
			neighbourNode.ID = neighbourRef
			neighbourNode.Flags &^= NodeClosed
			neighbourNode.Cost = cost
			neighbourNode.Total = total

			if (neighbourNode.Flags & NodeOpen) != 0 {
				q.OpenList.Modify(neighbourNode)
			} else {
				neighbourNode.Flags |= NodeOpen
				q.OpenList.Push(neighbourNode)
			}

			// Update nearest node to target so far.
			if heuristic < q.QueryData.LastBestNodeCost {
				q.QueryData.LastBestNodeCost = heuristic
				q.QueryData.LastBestNode = neighbourNode
			}
		}
	}

	if q.OpenList.Empty() {
		// Open list is empty, search is done
		q.QueryData.Err = nil
	}

	return ErrInProgress
}

// GetPathFromSlicedPath retrieves the path from a sliced pathfinding query.
func (q *NavMeshQuery) GetPathFromSlicedPath(path []PolyRef, maxPath int) (int, error) {
	if q.QueryData.Err != nil {
		return 0, q.QueryData.Err
	}

	n := 0
	if q.QueryData.LastBestNode != nil {
		n = q.getPathToNode(q.QueryData.LastBestNode, path, maxPath)
	}

	return n, q.QueryData.Err
}

// AppendVertex adds a vertex to the straight path.
func (q *NavMeshQuery) AppendVertex(pos [3]float32, flags uint8, ref PolyRef, straightPath []float32, straightPathFlags []uint8, straightPathRefs []PolyRef, maxStraightPath int) (int, error) {
	if maxStraightPath <= 0 {
		return 0, ErrInvalidParam
	}

	// Find insertion point (should be at the end or we need to find the right spot)
	vertSize := 0
	for vertSize < maxStraightPath && straightPathFlags[vertSize]&StraightPathEnd == 0 {
		vertSize++
	}

	if vertSize >= maxStraightPath {
		return vertSize, ErrPartialResult
	}

	copy(straightPath[vertSize*3:], pos[:])
	straightPathFlags[vertSize] = flags
	straightPathRefs[vertSize] = ref

	return vertSize + 1, nil
}

// AppendPortals adds portal vertices between two polygons to the straight path.
func (q *NavMeshQuery) AppendPortals(fromIdx, toIdx int, endPos [3]float32, path []PolyRef, pathSize int, straightPath []float32, straightPathFlags []uint8, straightPathRefs []PolyRef, maxStraightPath int, options int) (int, error) {
	if maxStraightPath <= 0 {
		return 0, ErrInvalidParam
	}

	vertSize := 0
	for vertSize < maxStraightPath && straightPathFlags[vertSize]&StraightPathEnd == 0 {
		vertSize++
	}

	if vertSize >= maxStraightPath {
		return vertSize, ErrPartialResult
	}

	// Iterate through the path range adding portal points
	for i := fromIdx; i < toIdx && i < pathSize; i++ {
		if i+1 >= pathSize {
			break
		}

		var l, r [3]float32
		fromTile, fromPoly := q.Nav.GetTileAndPolyByRefUnsafe(path[i])
		toTile, toPoly := q.Nav.GetTileAndPolyByRefUnsafe(path[i+1])
		getPortalPoints(path[i], path[i+1], fromTile, toTile, fromPoly, toPoly, &l, &r)

		// Add left and right points (or midpoint)
		if vertSize+2 <= maxStraightPath {
			copy(straightPath[vertSize*3:], l[:])
			straightPathFlags[vertSize] = 0
			straightPathRefs[vertSize] = path[i]
			vertSize++

			copy(straightPath[vertSize*3:], r[:])
			straightPathFlags[vertSize] = 0
			straightPathRefs[vertSize] = path[i+1]
			vertSize++
		} else {
			// Add midpoint as a single portal
			var mid [3]float32
			mid[0] = (l[0] + r[0]) * 0.5
			mid[1] = (l[1] + r[1]) * 0.5
			mid[2] = (l[2] + r[2]) * 0.5
			copy(straightPath[vertSize*3:], mid[:])
			straightPathFlags[vertSize] = 0
			straightPathRefs[vertSize] = path[i]
			vertSize++
		}
	}

	// Add end point
	if vertSize < maxStraightPath {
		copy(straightPath[vertSize*3:], endPos[:])
		straightPathFlags[vertSize] = StraightPathEnd
		if vertSize > 0 {
			straightPathRefs[vertSize] = path[toIdx]
		} else {
			straightPathRefs[vertSize] = 0
		}
		vertSize++
	}

	return vertSize, nil
}
