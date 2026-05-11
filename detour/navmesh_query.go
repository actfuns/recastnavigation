package detour

import (
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

	// Get polygon vertices -- use stack-allocated fixed array (max VertsPerPolygon=6)
	var verts [VertsPerPolygon][3]float32
	for i := 0; i < nv; i++ {
		vIdx := poly.Verts[i]
		verts[i][0] = tile.Verts[vIdx*3]
		verts[i][1] = tile.Verts[vIdx*3+1]
		verts[i][2] = tile.Verts[vIdx*3+2]
	}

	// If point is inside the polygon, return the point directly (matching C++ behavior)
	flatVerts := make([]float32, nv*3)
	for i := 0; i < nv; i++ {
		flatVerts[i*3] = verts[i][0]
		flatVerts[i*3+1] = verts[i][1]
		flatVerts[i*3+2] = verts[i][2]
	}
	var dummyEd [VertsPerPolygon]float32
	var dummyEt [VertsPerPolygon]float32
	if DistancePtPolyEdgesSqr(pos, flatVerts, nv, dummyEd[:], dummyEt[:]) {
		return pos, nil
	}

	// Find closest point on polygon edges
	minDistSqr := float32(math.MaxFloat32)
	bestT := float32(0)
	bestEdge := 0

	for i := 0; i < nv; i++ {
		j := (i + 1) % nv

		d, t := distPtSegSqr2D(pos, verts[i], verts[j])

		if d < minDistSqr {
			minDistSqr = d
			bestT = t
			bestEdge = i
		}
	}

	// Calculate closest point
	j := (bestEdge + 1) % nv
	var closest [3]float32
	closest[0] = verts[bestEdge][0] + bestT*(verts[j][0]-verts[bestEdge][0])
	closest[1] = verts[bestEdge][1] + bestT*(verts[j][1]-verts[bestEdge][1])
	closest[2] = verts[bestEdge][2] + bestT*(verts[j][2]-verts[bestEdge][2])

	return closest, nil
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
	if poly.Flags&f.IncludeFlags != f.IncludeFlags {
		return false
	}
	return true
}

// GetCost returns the cost to travel through the polygon.
func (f *QueryFilter) GetCost(pa, pb [3]float32, prevRef PolyRef, prevTile *MeshTile, prevPoly *Poly, curRef PolyRef, curTile *MeshTile, curPoly *Poly, nextRef PolyRef, nextTile *MeshTile, nextPoly *Poly) float32 {
	return Vdist(pa, pb) * f.AreaCost[curPoly.GetArea()]
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
func (q *NavMeshQuery) FindPath(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, maxPath int) ([]PolyRef, int, error) {
	if startRef == 0 || endRef == 0 {
		return nil, 0, ErrInvalidParam
	}
	if maxPath <= 0 {
		return nil, 0, ErrInvalidParam
	}
	if filter == nil {
		return nil, 0, ErrInvalidParam
	}

	// Validate input
	if !q.Nav.IsValidPolyRef(startRef) || !q.Nav.IsValidPolyRef(endRef) {
		return nil, 0, ErrInvalidParam
	}

	// Check if start and end are the same
	path := make([]PolyRef, maxPath)
	if startRef == endRef {
		path[0] = startRef
		return path[:1], 1, nil
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return nil, 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = Vdist(startPos, endPos) * H_SCALE
	startNode.ID = startRef
	startNode.Flags = NodeOpen

	q.OpenList.Push(startNode)

	lastBestNode := startNode
	lastBestNodeCost := startNode.Total

	outOfNodes := false

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		// Reached the goal, stop searching.
		if bestNode.ID == endRef {
			lastBestNode = bestNode
			break
		}

		// Get current poly and tile.
		bestRef := bestNode.ID
		bestTile, bestPoly, _ := q.Nav.GetTileAndPolyByRef(bestRef)

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		var parentTile *MeshTile
		var parentPoly *Poly
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}
		if parentRef != 0 {
			parentTile, parentPoly, _ = q.Nav.GetTileAndPolyByRef(parentRef)
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
				heuristic = Vdist(neighbourNode.Pos, endPos) * H_SCALE
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
		err = ErrOutOfNodes
	}

	return path[:n], n, err
}

// FindPathDirect finds a path and straight path in one call, using pre-allocated internal buffers.
// Returns (pathLength, straightPathLength, err).
// The caller can access the path via q.GetPathRefs() and straight path via q.GetStraightPath().
func (q *NavMeshQuery) FindPathDirect(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, maxPath int) (int, int, error) {
	path, pathLen, err := q.FindPath(startRef, endRef, startPos, endPos, filter, maxPath)
	if pathLen == 0 {
		return 0, 0, err
	}
	copy(q.pathBuf[:], path)
	straightPath, straightPathFlags, straightPathRefs, straightLen, err := q.FindStraightPath(startPos, endPos,
		q.pathBuf[:pathLen], pathLen,
		maxPath, StraightPathAreaCrossings)
	copy(q.straightPath[:], straightPath)
	copy(q.straightPathFlags[:], straightPathFlags)
	copy(q.straightPathRefs[:], straightPathRefs)
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

func (q *NavMeshQuery) getEdgeMidPoint(from, to PolyRef, fromTile, toTile *MeshTile, fromPoly, toPoly *Poly) [3]float32 {
	left, right, _ := q.getPortalPoints(from, to, fromTile, toTile, fromPoly, toPoly)
	var pos [3]float32
	pos[0] = (left[0] + right[0]) * 0.5
	pos[1] = (left[1] + right[1]) * 0.5
	pos[2] = (left[2] + right[2]) * 0.5
	return pos
}

func (q *NavMeshQuery) getPortalPoints(from, to PolyRef, fromTile, toTile *MeshTile, fromPoly, toPoly *Poly) ([3]float32, [3]float32, bool) {
	// Find the link that connects the two polygons
	for l := fromPoly.FirstLink; l != NullLink; l = fromTile.Links[l].Next {
		if fromTile.Links[l].Ref == to {
			edge := int(fromTile.Links[l].Edge)

			// Handle off-mesh connections.
			if fromPoly.GetType() == PolyTypeOffMeshConnection {
				left := Vcopy(fromTile.Verts[fromPoly.Verts[edge]*3 : fromPoly.Verts[edge]*3+3])
				right := Vcopy(fromTile.Verts[fromPoly.Verts[edge]*3 : fromPoly.Verts[edge]*3+3])
				return left, right, true
			}
			if toPoly.GetType() == PolyTypeOffMeshConnection {
				// Find link in toPoly that points back to fromPoly
				for l2 := toPoly.FirstLink; l2 != NullLink; l2 = toTile.Links[l2].Next {
					if toTile.Links[l2].Ref == from {
						v := int(toTile.Links[l2].Edge)
						left := Vcopy(toTile.Verts[toPoly.Verts[v]*3 : toPoly.Verts[v]*3+3])
						right := Vcopy(toTile.Verts[toPoly.Verts[v]*3 : toPoly.Verts[v]*3+3])
						return left, right, true
					}
				}
				return [3]float32{}, [3]float32{}, false
			}

			// Get the edge vertices
			nv := int(fromPoly.VertCount)
			va := edge
			vb := (edge + 1) % nv
			left := Vcopy(fromTile.Verts[fromPoly.Verts[va]*3 : fromPoly.Verts[va]*3+3])
			right := Vcopy(fromTile.Verts[fromPoly.Verts[vb]*3 : fromPoly.Verts[vb]*3+3])

			// If the link is at tile boundary, clamp the vertices to the link width.
			if fromTile.Links[l].Side != 0xff {
				if fromTile.Links[l].Bmin != 0 || fromTile.Links[l].Bmax != 255 {
					s := 1.0 / 255.0
					tmin := float32(fromTile.Links[l].Bmin) * float32(s)
					tmax := float32(fromTile.Links[l].Bmax) * float32(s)
						var va3, vb3 [3]float32
						copy(va3[:], fromTile.Verts[fromPoly.Verts[va]*3:fromPoly.Verts[va]*3+3])
						copy(vb3[:], fromTile.Verts[fromPoly.Verts[vb]*3:fromPoly.Verts[vb]*3+3])
						left = Vlerp(va3, vb3, tmin)
						right = Vlerp(va3, vb3, tmax)
					}
				}
				return left, right, true
			}
		}
		return [3]float32{}, [3]float32{}, false
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

	// If pos equals last vertex, just update flags and ref (no duplicate)
	if *straightPathCount > 0 {
		lastIdx := (*straightPathCount - 1) * 3
		if straightPath[lastIdx] == pos[0] && straightPath[lastIdx+1] == pos[1] && straightPath[lastIdx+2] == pos[2] {
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
		var ok bool
		left, right, ok = q.getPortalPoints(from, to, fromTile, toTile, fromPoly, toPoly)
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
func (q *NavMeshQuery) FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, maxStraightPath int, options int) ([]float32, []uint8, []PolyRef, int, error) {
	if maxStraightPath <= 0 {
		return nil, nil, nil, 0, ErrInvalidParam
	}
	if path == nil || pathSize == 0 {
		return nil, nil, nil, 0, ErrInvalidParam
	}

	straightPath := make([]float32, maxStraightPath*3)
	straightPathFlags := make([]uint8, maxStraightPath)
	straightPathRefs := make([]PolyRef, maxStraightPath)
	straightPathCount := 0

	// Clamp start position to first polygon boundary
	closestStartPos, err := q.closestPointOnPolyBoundary(path[0], startPos)
	if err != nil {
		return nil, nil, nil, 0, ErrInvalidParam
	}

	// Clamp end position to last polygon boundary
	closestEndPos, err := q.closestPointOnPolyBoundary(path[pathSize-1], endPos)
	if err != nil {
		return nil, nil, nil, 0, ErrInvalidParam
	}

	// Add start point
	stat := appendVertex(closestStartPos, StraightPathStart, path[0],
		straightPath, straightPathFlags, straightPathRefs,
		&straightPathCount, maxStraightPath)
	if stat != ErrInProgress {
		return straightPath, straightPathFlags, straightPathRefs, straightPathCount, stat
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
				left, right, ok = q.getPortalPoints(path[i], path[i+1], fromTile, toTile, fromPoly, toPoly)
				if !ok {
					// Failed to get portal points - clamp end point and return
					closestEndPos, err = q.closestPointOnPolyBoundary(path[i], endPos)
					if err != nil {
						return straightPath, straightPathFlags, straightPathRefs, straightPathCount, ErrInvalidParam
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

					return straightPath, straightPathFlags, straightPathRefs, straightPathCount, ErrPartialResult
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
							return straightPath, straightPathFlags, straightPathRefs, straightPathCount, stat
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
						return straightPath, straightPathFlags, straightPathRefs, straightPathCount, stat
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
							return straightPath, straightPathFlags, straightPathRefs, straightPathCount, stat
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
						return straightPath, straightPathFlags, straightPathRefs, straightPathCount, stat
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

	return straightPath, straightPathFlags, straightPathRefs, straightPathCount, nil
}

// MoveAlongSurface moves from the start position along the surface to find a position
// constrained by the navigation mesh.
func (q *NavMeshQuery) MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result []float32, visited []PolyRef, maxVisitedSize int) (int, error) {
	if startRef == 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil {
		return 0, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	bestNode := startNode
	bestDist := float32(math.MaxFloat32)

	for !q.OpenList.Empty() {
		bestNode = q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		tile, poly, _ := q.Nav.GetTileAndPolyByRef(bestNode.ID)

		// Check distance to end point
		var closestPt [3]float32
		closestPt, _ = q.Nav.closestPointOnPoly(bestNode.ID, endPos)
		dist := Vdist2DSqr(closestPt, endPos)
		if dist < bestDist {
			bestDist = dist
		}

		// Expand neighbours
		for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
			link := &tile.Links[l]
			if link.Ref == 0 {
				continue
			}
			neiTile, neiPoly := q.Nav.GetTileAndPolyByRefUnsafe(link.Ref)
			if !filter.PassFilter(link.Ref, neiTile, neiPoly) {
				continue
			}

			nei := q.NodePool.GetNode(link.Ref, 0)
			if nei == nil {
				continue
			}
			if nei.Flags&NodeClosed != 0 {
				continue
			}

			// Heuristic: distance from neighbour to end
			var closestNeiPt [3]float32
			closestNeiPt, _ = q.Nav.closestPointOnPoly(link.Ref, endPos)
			neiDist := Vdist2DSqr(closestNeiPt, endPos)

			if (nei.Flags & NodeOpen) != 0 {
				if nei.Total > neiDist {
					nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
					nei.Total = neiDist
					nei.ID = link.Ref
					q.OpenList.Modify(nei)
				}
			} else {
				nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
				nei.Total = neiDist
				nei.ID = link.Ref
				nei.Flags = NodeOpen
				q.OpenList.Push(nei)
			}
		}
	}

	// Build result
	var pts [256]float32
	npts := 0

	// Walk from best node back to start
	node := bestNode
	for node != nil && npts < 128 {
		var pt [3]float32
		if node.ID == startRef {
			copy(pt[:], startPos[:])
		} else {
			tile, poly := q.Nav.GetTileAndPolyByRefUnsafe(node.ID)
			pt = CalcPolyCenter(poly.Verts[:poly.VertCount], tile.Verts)
			pt[1] = startPos[1]
		}
		// Insert at beginning
		for k := npts * 3; k >= 3; k-- {
			pts[k] = pts[k-3]
		}
		copy(pts[:], pt[:])
		npts++

		if node.Pidx == 0 {
			break
		}
		node = q.NodePool.GetNodeAtIdx(node.Pidx)
	}

	if npts > 0 {
		copy(result, pts[npts*3-3:])
	}

	// Save visited polygons
	nvisited := 0
	if visited != nil {
		node = bestNode
		for node != nil && nvisited < maxVisitedSize {
			visited[nvisited] = node.ID
			nvisited++
			if node.Pidx == 0 {
				break
			}
			node = q.NodePool.GetNodeAtIdx(node.Pidx)
		}
		// Reverse
		for i, j := 0, nvisited-1; i < j; i, j = i+1, j-1 {
			visited[i], visited[j] = visited[j], visited[i]
		}
	}

	return nvisited, nil
}

// Raycast performs a raycast along the navigation mesh surface.
func (q *NavMeshQuery) Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32, prevRef PolyRef, hit *RaycastHit) error {
	if startRef == 0 {
		return ErrInvalidParam
	}
	if filter == nil {
		return ErrInvalidParam
	}
	if hit == nil {
		return ErrInvalidParam
	}

	hit.T = 0
	hit.HitNormal[0] = 0
	hit.HitNormal[1] = 0
	hit.HitNormal[2] = 0
	hit.HitCount = 0
	hit.PathCount = 0
	hit.Pos[0] = endPos[0]
	hit.Pos[1] = endPos[1]
	hit.Pos[2] = endPos[2]

	curRef := startRef
	curPos := startPos
	curTile, curPoly := q.Nav.GetTileAndPolyByRefUnsafe(curRef)

	t := float32(0)

	for {
		// Check if we hit the end
		if t >= 1 {
			break
		}

		nv := int(curPoly.VertCount)
		for i, j := 0, nv-1; i < nv; j, i = i, i+1 {
			// Skip if this edge has no connection
			if curPoly.Neis[i] == 0 {
				continue
			}

				var va, vb [3]float32
				copy(va[:], curTile.Verts[curPoly.Verts[i]*3:curPoly.Verts[i]*3+3])
				copy(vb[:], curTile.Verts[curPoly.Verts[j]*3:curPoly.Verts[j]*3+3])

				// Check if the ray segment intersects this edge
				ok, s, _ := IntersectSegSeg2D(startPos, endPos, va, vb)

			// Check if the ray segment intersects this edge
			if !ok || s <= t || s >= 1 {
				continue
			}

			// Check if the edge is between the start and current position
			dir := endPos[0] - startPos[0]
			if dir*(va[2]-curPos[2]) > dir*(vb[2]-curPos[2]) {
				continue
			}

			// Cross the edge to the neighbour polygon
			var neiRef PolyRef
			neiFound := false
			for l := curPoly.FirstLink; l != NullLink; l = curTile.Links[l].Next {
				if curTile.Links[l].Edge == uint8(i) {
					neiRef = curTile.Links[l].Ref
					neiFound = true
					break
				}
			}

			if !neiFound {
				continue
			}

			neiTile, neiPoly := q.Nav.GetTileAndPolyByRefUnsafe(neiRef)
_, _ = q.Nav.closestPointOnPoly(neiRef, curPos)

			// Check if the polygon passes the filter
			if !filter.PassFilter(neiRef, neiTile, neiPoly) {
				// Hit a wall
				hit.T = s
				hit.Pos = Vlerp(startPos, endPos, s)
				hit.HitNormal[0] = -(vb[2] - va[2])
				hit.HitNormal[1] = 0
				hit.HitNormal[2] = vb[0] - va[0]
				hit.HitNormal = Vnormalize(hit.HitNormal)

				if hit.PathCount < MaxRaycastPathSize {
					hit.Path[hit.PathCount] = curRef
					hit.PathCount++
				}
				hit.HitCount++
				return nil
			}

			// Move to neighbour
			t = s
			curPos[0] = startPos[0] + (endPos[0]-startPos[0])*t
			curPos[1] = startPos[1] + (endPos[1]-startPos[1])*t
			curPos[2] = startPos[2] + (endPos[2]-startPos[2])*t

			curRef = neiRef
			curTile = neiTile
			curPoly = neiPoly

			if hit.PathCount < MaxRaycastPathSize {
				hit.Path[hit.PathCount] = curRef
				hit.PathCount++
			}

			break
		}
	}

	hit.T = 1
	copy(hit.Pos[:], endPos[:])
	return nil
}

// RaycastHit stores the result of a raycast operation.
type RaycastHit struct {
	T         float32
	HitNormal [3]float32
	HitCount  int
	Path      [MaxRaycastPathSize]PolyRef
	PathCount int
	Pos       [3]float32
	Cost      float32
}

const MaxRaycastPathSize = 256

// FindPolysAroundCircle finds polygons within a circle around a position.
func (q *NavMeshQuery) FindPolysAroundCircle(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error) {
	if startRef == 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil {
		return 0, ErrInvalidParam
	}
	if maxResult <= 0 {
		return 0, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	radiusSqr := radius * radius

	n := 0
	if n < maxResult {
		resultRef[n] = startRef
		resultParent[n] = 0
		resultCost[n] = 0
		n++
	}

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		tile, poly, _ := q.Nav.GetTileAndPolyByRef(bestNode.ID)

		for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
			link := &tile.Links[l]
			if link.Ref == 0 {
				continue
			}
			neiTile, neiPoly := q.Nav.GetTileAndPolyByRefUnsafe(link.Ref)
			if !filter.PassFilter(link.Ref, neiTile, neiPoly) {
				continue
			}

			nei := q.NodePool.GetNode(link.Ref, 0)
			if nei == nil {
				continue
			}
			if nei.Flags&NodeClosed != 0 {
				continue
			}

			// Check distance from center
			var closestPt [3]float32
			closestPt, _ = q.Nav.closestPointOnPoly(link.Ref, centerPos)
			distSqr := Vdist2DSqr(closestPt, centerPos)

			if distSqr > radiusSqr {
				continue
			}

			curCost := bestNode.Cost + filter.GetCost(closestPt, centerPos, 0, nil, nil, bestNode.ID, tile, poly, link.Ref, neiTile, neiPoly)

			if (nei.Flags & NodeOpen) != 0 {
				if nei.Total > distSqr || nei.Cost > curCost {
					nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
					nei.Cost = curCost
					nei.Total = distSqr
					nei.ID = link.Ref
					q.OpenList.Modify(nei)
				}
			} else {
				nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
				nei.Cost = curCost
				nei.Total = distSqr
				nei.ID = link.Ref
				nei.Flags = NodeOpen
				q.OpenList.Push(nei)
			}

			if n < maxResult {
				resultRef[n] = link.Ref
				resultParent[n] = bestNode.ID
				resultCost[n] = curCost
				n++
			}
		}
	}

	return n, nil
}

// FindPolysAroundShape finds polygons within a convex shape around a position.
func (q *NavMeshQuery) FindPolysAroundShape(startRef PolyRef, verts []float32, nverts int, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error) {
	if startRef == 0 {
		return 0, ErrInvalidParam
	}
	if filter == nil {
		return 0, ErrInvalidParam
	}
	if maxResult <= 0 {
		return 0, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	n := 0
	if n < maxResult {
		resultRef[n] = startRef
		resultParent[n] = 0
		resultCost[n] = 0
		n++
	}

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		tile, poly, _ := q.Nav.GetTileAndPolyByRef(bestNode.ID)

		for l := poly.FirstLink; l != NullLink; l = tile.Links[l].Next {
			link := &tile.Links[l]
			if link.Ref == 0 {
				continue
			}
			neiTile, neiPoly := q.Nav.GetTileAndPolyByRefUnsafe(link.Ref)
			if !filter.PassFilter(link.Ref, neiTile, neiPoly) {
				continue
			}

			nei := q.NodePool.GetNode(link.Ref, 0)
			if nei == nil {
				continue
			}
			if nei.Flags&NodeClosed != 0 {
				continue
			}

			// Check if polygon overlaps the query shape
			var closestPt [3]float32
			closestPt = Vcopy(verts)

			// Use PointInPolygon check with the query shape
			if !PointInPolygon(closestPt, verts, nverts) {
				continue
			}

			var center [3]float32
			for k := 0; k < nverts; k++ {
				center[0] += verts[k*3]
				center[1] += verts[k*3+1]
				center[2] += verts[k*3+2]
			}
			s := 1.0 / float32(nverts)
			center[0] *= s
			center[1] *= s
			center[2] *= s

			curCost := bestNode.Cost + filter.GetCost(closestPt, center, 0, nil, nil, bestNode.ID, tile, poly, link.Ref, neiTile, neiPoly)

			if (nei.Flags & NodeOpen) != 0 {
				if nei.Cost > curCost {
					nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
					nei.Cost = curCost
					nei.ID = link.Ref
					q.OpenList.Modify(nei)
				}
			} else {
				nei.Pidx = q.NodePool.GetNodeIdx(bestNode)
				nei.Cost = curCost
				nei.ID = link.Ref
				nei.Flags = NodeOpen
				q.OpenList.Push(nei)
			}

			if n < maxResult {
				resultRef[n] = link.Ref
				resultParent[n] = bestNode.ID
				resultCost[n] = curCost
				n++
			}
		}
	}

	return n, nil
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

			va := Vcopy(tile.Verts[poly.Verts[i]*3:poly.Verts[i]*3+3])
			vb := Vcopy(tile.Verts[poly.Verts[(i+1)%nv]*3:poly.Verts[(i+1)%nv]*3+3])

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

		va := Vcopy(tile.Verts[poly.Verts[i]*3:poly.Verts[i]*3+3])
		vb := Vcopy(tile.Verts[poly.Verts[(i+1)%nv]*3:poly.Verts[(i+1)%nv]*3+3])

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
		va := Vcopy(tile.Verts[poly.Verts[i]*3:poly.Verts[i]*3+3])
		vb := Vcopy(tile.Verts[poly.Verts[(i+1)%nv]*3:poly.Verts[(i+1)%nv]*3+3])
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
	return 0, ErrFailure
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

	for tileIndex := 0; tileIndex < q.Nav.MaxTiles; tileIndex++ {
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
				if d > halfExtents[1] {
					d = d - halfExtents[1]
					d = d * d
				} else {
					d = 0
				}
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
	return 0, [3]float32{}, ErrFailure
}

// FindRandomPoint finds a random point in the navigation mesh.
func (q *NavMeshQuery) FindRandomPoint(filter *QueryFilter, randomFunc func() float32) (PolyRef, [3]float32, error) {
	if filter == nil || randomFunc == nil {
		return 0, [3]float32{}, ErrInvalidParam
	}
	var resultPt [3]float32
	var resultRef PolyRef

	totPolyCount := 0
	for tileIndex := 0; tileIndex < q.Nav.MaxTiles; tileIndex++ {
		tile := &q.Nav.Tiles[tileIndex]
		if tile.Header == nil {
			continue
		}
		for i := 0; i < int(tile.Header.PolyCount); i++ {
			poly := &tile.Polys[i]
			if poly.GetType() == PolyTypeOffMeshConnection {
				continue
			}
			if filter.PassFilter(q.Nav.GetPolyRefBase(tile)|PolyRef(i), tile, poly) {
				totPolyCount++
			}
		}
	}

	if totPolyCount == 0 {
		return 0, [3]float32{}, ErrFailure
	}

	// Pick a random polygon weighted by area
	r := randomFunc() * float32(totPolyCount)
	areaAccum := float32(0)
	hitPoly := -1
	var hitTile *MeshTile

	for tileIndex := 0; tileIndex < q.Nav.MaxTiles && hitPoly < 0; tileIndex++ {
		tile := &q.Nav.Tiles[tileIndex]
		if tile.Header == nil {
			continue
		}
		for i := 0; i < int(tile.Header.PolyCount); i++ {
			poly := &tile.Polys[i]
			if poly.GetType() == PolyTypeOffMeshConnection {
				continue
			}
			if !filter.PassFilter(q.Nav.GetPolyRefBase(tile)|PolyRef(i), tile, poly) {
				continue
			}
			areaAccum++
			if r < areaAccum {
				hitPoly = i
				hitTile = tile
				break
			}
		}
	}

	if hitPoly < 0 {
		return 0, [3]float32{}, ErrFailure
	}

	// Pick a random point within the polygon
	var verts [VertsPerPolygon * 3]float32
	poly := &hitTile.Polys[hitPoly]
	nv := int(poly.VertCount)
	for i := 0; i < nv; i++ {
		copy(verts[i*3:i*3+3], hitTile.Verts[poly.Verts[i]*3:poly.Verts[i]*3+3])
	}

	// Use detail triangles
	pd := &hitTile.DetailMeshes[hitPoly]
	if pd.TriCount > 0 {
		ti := int(randomFunc() * float32(pd.TriCount))
		if ti >= int(pd.TriCount) {
			ti = int(pd.TriCount) - 1
		}
		tris := hitTile.DetailTris[(int(pd.TriBase)+ti)*4:]
		var triVerts [3][]float32
		for j := 0; j < 3; j++ {
			if tris[j] < uint8(poly.VertCount) {
				triVerts[j] = hitTile.Verts[poly.Verts[tris[j]]*3 : poly.Verts[tris[j]]*3+3]
			} else {
				triVerts[j] = hitTile.DetailVerts[(int(pd.VertBase)+int(tris[j])-int(poly.VertCount))*3 : (int(pd.VertBase)+int(tris[j])-int(poly.VertCount))*3+3]
			}
		}
		s := randomFunc()
		t := randomFunc()
		if s+t > 1 {
			s = 1 - s
			t = 1 - t
		}
		u := 1 - s - t
		resultPt[0] = triVerts[0][0]*u + triVerts[1][0]*s + triVerts[2][0]*t
		resultPt[1] = triVerts[0][1]*u + triVerts[1][1]*s + triVerts[2][1]*t
		resultPt[2] = triVerts[0][2]*u + triVerts[1][2]*s + triVerts[2][2]*t
	} else {
		// Fallback: use polygon centroid
		resultPt = CalcPolyCenter(poly.Verts[:nv], hitTile.Verts)
	}

	resultRef = q.Nav.GetPolyRefBase(hitTile) | PolyRef(hitPoly)
	return resultRef, resultPt, nil
}

// FindRandomPointAroundCircle finds a random point within a circle around a position.
func (q *NavMeshQuery) FindRandomPointAroundCircle(startRef PolyRef, centerPos [3]float32, maxRadius float32, filter *QueryFilter, randomFunc func() float32) (PolyRef, [3]float32, error) {
	if startRef == 0 || filter == nil || randomFunc == nil {
		return 0, [3]float32{}, ErrInvalidParam
	}

	q.NodePool.Clear()
	q.OpenList.Clear()

	startNode := q.NodePool.GetNode(startRef, 0)
	if startNode == nil {
		return 0, [3]float32{}, ErrOutOfNodes
	}
	startNode.Pidx = 0
	startNode.Cost = 0
	startNode.Total = 0
	startNode.ID = startRef
	startNode.Flags = NodeOpen
	q.OpenList.Push(startNode)

	radiusSqr := maxRadius * maxRadius

	for !q.OpenList.Empty() {
		bestNode := q.OpenList.Pop()
		bestNode.Flags &= ^uint8(0) ^ NodeOpen
		bestNode.Flags |= NodeClosed

		tile, poly, _ := q.Nav.GetTileAndPolyByRef(bestNode.ID)

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

	// Find a random connected polygon
	var polys [128]PolyRef
	polyCount := 0
	for i := 0; i < q.NodePool.nodeCount && polyCount < 128; i++ {
		if q.NodePool.nodes[i].ID != 0 {
			polys[polyCount] = q.NodePool.nodes[i].ID
			polyCount++
		}
	}

	if polyCount == 0 {
		return 0, [3]float32{}, ErrFailure
	}

	hitIdx := int(randomFunc() * float32(polyCount))
	if hitIdx >= polyCount {
		hitIdx = polyCount - 1
	}
	resultRef := polys[hitIdx]

	tile, poly := q.Nav.GetTileAndPolyByRefUnsafe(resultRef)
	nv := int(poly.VertCount)
	var verts [VertsPerPolygon * 3]float32
	for i := 0; i < nv; i++ {
		copy(verts[i*3:i*3+3], tile.Verts[poly.Verts[i]*3:poly.Verts[i]*3+3])
	}
	areas := make([]float32, nv)
	s := randomFunc()
	t := randomFunc()
	resultPt := RandomPointInConvexPoly(verts[:], nv, areas, s, t)

	return resultRef, resultPt, nil
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
	for tileIndex := 0; tileIndex < q.Nav.MaxTiles; tileIndex++ {
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
		bestTile, bestPoly, _ := q.Nav.GetTileAndPolyByRef(bestRef)

		// Get parent poly and tile.
		parentRef := PolyRef(0)
		var parentTile *MeshTile
		var parentPoly *Poly
		if bestNode.Pidx != 0 {
			parentRef = q.NodePool.GetNodeAtIdx(bestNode.Pidx).ID
		}
		if parentRef != 0 {
			parentTile, parentPoly, _ = q.Nav.GetTileAndPolyByRef(parentRef)
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
		l, r, _ = q.getPortalPoints(path[i], path[i+1], fromTile, toTile, fromPoly, toPoly)

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
