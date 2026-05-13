package detour_crowd

import (
	"fmt"
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

// createTestNavMeshQueryEx creates a NavMeshQuery with configurable MaxTiles and maxNodes.
func createTestNavMeshQueryEx(t testing.TB, maxTiles int32, maxNodes int) *detour.NavMeshQuery {
	t.Helper()

	const gridSize = 10
	stride := gridSize + 1

	verts := make([]uint16, 0, stride*stride*3)
	for z := 0; z <= gridSize; z++ {
		for x := 0; x <= gridSize; x++ {
			verts = append(verts, uint16(x), 0, uint16(z))
		}
	}

	polys := make([]uint16, 0, gridSize*gridSize*12)
	flags := make([]uint16, 0, gridSize*gridSize)
	areas := make([]uint8, 0, gridSize*gridSize)
	const boundaryMask uint16 = 0x800f

	for z := 0; z < gridSize; z++ {
		for x := 0; x < gridSize; x++ {
			idx := z*stride + x
			polys = append(polys,
				uint16(idx), uint16(idx+1), uint16(idx+gridSize+2),
				uint16(idx), uint16(idx+gridSize+2), uint16(idx+gridSize+1),
			)
			if z > 0 {
				polys = append(polys, uint16((z-1)*gridSize+x))
			} else {
				polys = append(polys, boundaryMask)
			}
			if x < gridSize-1 {
				polys = append(polys, uint16(z*gridSize+x+1))
			} else {
				polys = append(polys, boundaryMask)
			}
			polys = append(polys, boundaryMask)
			polys = append(polys, boundaryMask)
			if z < gridSize-1 {
				polys = append(polys, uint16((z+1)*gridSize+x))
			} else {
				polys = append(polys, boundaryMask)
			}
			if x > 0 {
				polys = append(polys, uint16(z*gridSize+x-1))
			} else {
				polys = append(polys, boundaryMask)
			}
			flags = append(flags, 0xffff)
			areas = append(areas, 63)
		}
	}

	params := &detour.NavMeshCreateParams{
		Verts:          verts,
		VertCount:      (gridSize + 1) * (gridSize + 1),
		Polys:          polys,
		PolyAreas:      areas,
		PolyFlags:      flags,
		PolyCount:      gridSize * gridSize,
		Nvp:            6,
		WalkableHeight: 2.0,
		WalkableRadius: 0,
		WalkableClimb:  1.0,
		Cs:             1.0,
		Ch:             1.0,
		BuildBvTree:    false,
		Bmin:           [3]float32{0, 0, 0},
		Bmax:           [3]float32{float32(gridSize), 1, float32(gridSize)},
	}

	data, _, ok := detour.CreateNavMeshData(params)
	if !ok || data == nil {
		t.Fatal("CreateNavMeshData failed")
	}

	nav := &detour.NavMesh{}
	err := nav.Init(&detour.NavMeshParams{
		Orig:       [3]float32{0, 0, 0},
		TileWidth:  float32(gridSize),
		TileHeight: float32(gridSize),
		MaxTiles:   maxTiles,
		MaxPolys:   4096,
	})
	if err != nil {
		t.Fatalf("NavMesh.Init: %v", err)
	}

	_, err = nav.AddTile(data, 0, 0)
	if err != nil {
		t.Fatalf("AddTile: %v", err)
	}

	q := detour.NewNavMeshQuery()
	if err := q.Init(nav, maxNodes); err != nil {
		t.Fatalf("NavMeshQuery.Init: %v", err)
	}
	return q
}

// TestDetailedComparison prints detailed Go crowd data for manual comparison with C++.
// Run with: go test -v -run TestDetailedComparison ./detour_crowd/
//
// Uses maxTiles=1 (matching C++ bench_crowd.cpp) and maxNodes=512 (matching dtCrowd MAX_COMMON_NODES).
func TestDetailedComparison(t *testing.T) {
	navQuery := createTestNavMeshQueryEx(t, 1, 512)
	nav := navQuery.Nav

	tile0 := nav.GetTileAt(0, 0, 0)
	if tile0 == nil {
		t.Fatal("GetTileAt(0,0,0) failed")
	}

	baseRef := nav.GetPolyRefBase(tile0)
	fmt.Printf("  Tile(0) baseRef = %d\n", baseRef)
	fmt.Printf("  NavMesh: maxTiles=%d\n", nav.GetMaxTiles())
	fmt.Printf("  Tile(0): header.PolyCount=%d\n", tile0.Header.PolyCount)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	for i := 0; i < detour.MaxAreas; i++ {
		filter.AreaCost[i] = 1.0
	}

	halfExtents := [3]float32{2, 4, 2}
	startPos := [3]float32{1, 0, 1}
	endPos := [3]float32{8, 0, 8}

	startRef, startNearest, err := navQuery.FindNearestPoly(startPos, halfExtents, filter)
	if err != nil || startRef == 0 {
		t.Fatalf("FindNearestPoly start failed: ref=%d err=%v", startRef, err)
	}
	endRef, endNearest, err := navQuery.FindNearestPoly(endPos, halfExtents, filter)
	if err != nil || endRef == 0 {
		t.Fatalf("FindNearestPoly end failed: ref=%d err=%v", endRef, err)
	}

	fmt.Printf("  startRef=%d startNearest=(%.4f,%.4f,%.4f)\n",
		startRef, startNearest[0], startNearest[1], startNearest[2])
	fmt.Printf("  endRef=%d endNearest=(%.4f,%.4f,%.4f)\n",
		endRef, endNearest[0], endNearest[1], endNearest[2])

	// --- FindPath ---
	pathBuf := make([]detour.PolyRef, 256)
	npath, _ := navQuery.FindPath(startRef, endRef, startNearest, endNearest, filter, pathBuf)
	path := pathBuf[:npath]
	fmt.Printf("  FindPath: status=0x40000000 npath=%d\n", len(path))
	for i, p := range path {
		fmt.Printf("    [%d] %d (polyIndex=%d)\n", i, p, p&0x3fffffff)
	}

	// --- FindStraightPath ---
	spBuf := make([]detour.PolyRef, 256)
	spCnt, _ := navQuery.FindPath(startRef, endRef, startNearest, endNearest, filter, spBuf)
	straightPath := make([]float32, 256*3)
	straightPathFlags := make([]uint8, 256)
	straightPathRefs := make([]detour.PolyRef, 256)
	nstraight, _ := navQuery.FindStraightPath(startNearest, endNearest, spBuf, spCnt,
		straightPath, straightPathFlags, straightPathRefs, 256, 0)
	fmt.Printf("  FindStraightPath: status=0x40000000 nstraight=%d\n", nstraight)
	for i := 0; i < nstraight; i++ {
		fmt.Printf("    [%d] (%.4f,%.4f,%.4f) flags=%d poly=%d\n",
			i, straightPath[i*3], straightPath[i*3+1], straightPath[i*3+2],
			straightPathFlags[i], straightPathRefs[i])
	}

	// --- Crowd ---
	c := NewCrowd()
	if !c.Init(16, 0.6, navQuery) {
		t.Fatal("Crowd.Init failed")
	}

	cf := detour.NewQueryFilter()
	cf.IncludeFlags = 0xffff
	for i := 0; i < detour.MaxAreas; i++ {
		cf.AreaCost[i] = 1.0
	}
	c.SetFilter(0, cf)

	params := &CrowdAgentParams{
		Radius:                0.5,
		Height:                2.0,
		MaxAcceleration:       8.0,
		MaxSpeed:              3.5,
		CollisionQueryRange:   6,
		PathOptimizationRange: 10,
		SeparationWeight:      3.0,
		UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo,
		ObstacleAvoidanceType: 0,
		QueryFilterType:       0,
	}

	idx := c.AddAgent(startNearest, params)
	fmt.Printf("  agentIdx=%d\n", idx)
	if idx < 0 {
		t.Fatal("AddAgent failed")
	}

	ag := c.GetAgent(idx)
	fmt.Printf("  After AddAgent: state=%d targetState=%d corridor.npath=%d firstPoly=%d\n",
		ag.state, ag.targetState, ag.corridor.GetPathCount(), ag.corridor.GetFirstPoly())

	if !c.RequestMoveTarget(idx, endRef, endNearest) {
		t.Fatal("RequestMoveTarget failed")
	}
	ag = c.GetAgent(idx)
	fmt.Printf("  After RequestMoveTarget: targetState=%d\n", ag.targetState)

	const dt = 1.0 / 60.0
	for f := 1; f <= 30; f++ {
		c.Update(dt, nil)
		ag = c.GetAgent(idx)
		printFrame := (f <= 3) || (f <= 30 && f%5 == 0)
		if printFrame {
			fmt.Printf("  frame %2d: pos=(%.4f,%.4f,%.4f) state=%d targetState=%d ncorners=%d",
				f, ag.npos[0], ag.npos[1], ag.npos[2],
				ag.state, ag.targetState, ag.ncorners)
			if ag.ncorners > 0 {
				fmt.Printf(" corner0=(%.2f,%.2f,%.2f)",
					ag.cornerVerts[0], ag.cornerVerts[1], ag.cornerVerts[2])
			}
			fmt.Println()
		}
	}

	ag = c.GetAgent(idx)
	pathCount := ag.corridor.GetPathCount()
	fmt.Printf("  Final corridor (%d):\n", pathCount)
	corrPath := ag.corridor.GetPath()
	for i := 0; i < pathCount; i++ {
		fmt.Printf("    [%d] %d\n", i, corrPath[i])
	}
}
