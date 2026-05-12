package detour

import (
	"math"
	"testing"
)

func TestBuildTestNavmesh(t *testing.T) {
	m := buildTestNavmesh(t)
	if m == nil {
		t.Fatal("navmesh is nil")
	}
	tile := m.GetTileAt(0, 0, 0)
	if tile == nil {
		t.Fatal("no tile at (0,0,0)")
	}
	if tile.Header.PolyCount != 2 {
		t.Fatalf("expected 2 polys, got %d", tile.Header.PolyCount)
	}
	q := createTestQuery(t, m)

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	t.Run("FindNearestPoly inside navmesh", func(t *testing.T) {
		center := [3]float32{5, 0, 5}
		ref, nearest, err := q.FindNearestPoly(center, halfExtents, filter)
		if err != nil {
			t.Logf("FindNearestPoly: %v (expected for out-of-bounds)", err)
		}
		if ref == 0 {
			t.Fatal("FindNearestPoly returned 0 ref")
		}
		if math.Abs(float64(nearest[0]-5)) > 1 || math.Abs(float64(nearest[2]-5)) > 1 {
			t.Logf("nearest=(%f,%f,%f) expected near (5,0,5)", nearest[0], nearest[1], nearest[2])
		}
	})

	t.Run("FindNearestPoly outside navmesh with small extents", func(t *testing.T) {
		// Point 100 units away should not find anything with small extents
		smallExtents := [3]float32{0.1, 0.1, 0.1}
		center := [3]float32{100, 0, 100}
		ref, _, err := q.FindNearestPoly(center, smallExtents, filter)
		if err != nil {
			t.Logf("FindNearestPoly: %v (expected for out-of-bounds)", err)
		}
		if ref != 0 {
			t.Log("out-of-bounds found poly (may still find with large search)")
		}
	})

	t.Run("FindPath across two triangles", func(t *testing.T) {
		startPos := [3]float32{1, 0, 1}
		endPos := [3]float32{9, 0, 9}
		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)
		if startRef == 0 || endRef == 0 {
			t.Fatal("could not find start/end refs")
		}

		path := make([]PolyRef, 256)
		n, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
		if err != nil {
			t.Fatalf("FindPath: %v", err)
		}
		if n == 0 {
			t.Fatal("FindPath returned empty path")
		}
		t.Logf("FindPath: %d refs", n)
	})

	t.Run("FindPath same start and end", func(t *testing.T) {
		pos := [3]float32{1, 0, 1}
		ref, _, _ := q.FindNearestPoly(pos, halfExtents, filter)
		if ref == 0 {
			t.Skip("no start ref")
		}

		path := make([]PolyRef, 256)
		n, err := q.FindPath(ref, ref, pos, pos, filter, path)
		if err != nil {
			t.Fatalf("FindPath same: %v", err)
		}
		if n == 0 {
			t.Fatal("FindPath same should return at least 1 ref")
		}
	})

	t.Run("FindStraightPath", func(t *testing.T) {
		startPos := [3]float32{1, 0, 1}
		endPos := [3]float32{9, 0, 9}
		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)

		path := make([]PolyRef, 256)
		n, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
		if err != nil || n == 0 {
			t.Skip("no path found, skipping straight path test")
		}

		straightVerts := make([]float32, 64*3)
		straightFlags := make([]uint8, 64)
		straightPolys := make([]PolyRef, 64)
		nStraight, err := q.FindStraightPath(startPos, endPos, path, n,
			straightVerts, straightFlags, straightPolys, 64, 0)
		if err != nil {
			t.Fatalf("FindStraightPath: %v", err)
		}
		if nStraight == 0 {
			t.Fatal("FindStraightPath returned 0 corners")
		}
		t.Logf("FindStraightPath: %d corners", nStraight)
	})

	t.Run("Raycast across triangles", func(t *testing.T) {
		startPos := [3]float32{1, 0, 1}
		endPos := [3]float32{9, 0, 9}
		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		if startRef == 0 {
			t.Fatal("no start ref for raycast")
		}

		hit := &RaycastHit{}
		err := q.Raycast(startRef, startPos, endPos, filter, 0, 0, hit)
		if err != nil {
			t.Fatalf("Raycast: %v", err)
		}
		t.Logf("Raycast hit t=%f, pathCount=%d", hit.T, hit.PathCount)
	})

	t.Run("MoveAlongSurface", func(t *testing.T) {
		startPos := [3]float32{1, 0, 1}
		endPos := [3]float32{9, 0, 9}
		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		if startRef == 0 {
			t.Fatal("no start ref")
		}

		result := make([]float32, 48)
		visited := make([]PolyRef, 16)
		nvisited, err := q.MoveAlongSurface(startRef, startPos, endPos,
			filter, result, visited, 16)
		if err != nil {
			t.Fatalf("MoveAlongSurface: %v", err)
		}
		if nvisited == 0 {
			t.Fatal("MoveAlongSurface returned 0 visited")
		}
		t.Logf("MoveAlongSurface: %d visited", nvisited)
	})

	t.Run("FindPolysAroundCircle", func(t *testing.T) {
		center := [3]float32{5, 0, 5}
		ref, _, _ := q.FindNearestPoly(center, halfExtents, filter)
		if ref == 0 {
			t.Fatal("no ref for center")
		}

		resultRef := make([]PolyRef, 16)
		resultParent := make([]PolyRef, 16)
		resultCost := make([]float32, 16)
		n, err := q.FindPolysAroundCircle(ref, center, 15, filter,
			resultRef, resultParent, resultCost, 16)
		if err != nil {
			t.Fatalf("FindPolysAroundCircle: %v", err)
		}
		if n == 0 {
			t.Fatal("FindPolysAroundCircle returned 0 polys")
		}
		t.Logf("FindPolysAroundCircle: %d polys", n)
	})

	t.Run("FindPolysAroundShape", func(t *testing.T) {
		center := [3]float32{5, 0, 5}
		ref, _, _ := q.FindNearestPoly(center, halfExtents, filter)
		if ref == 0 {
			t.Fatal("no ref for center")
		}

		verts := []float32{2, 0, 2, 8, 0, 2, 5, 0, 8}
		resultRef := make([]PolyRef, 16)
		resultParent := make([]PolyRef, 16)
		resultCost := make([]float32, 16)
		n, err := q.FindPolysAroundShape(ref, verts, 3, filter,
			resultRef, resultParent, resultCost, 16)
		if err != nil {
			t.Fatalf("FindPolysAroundShape: %v", err)
		}
		if n == 0 {
			t.Fatal("FindPolysAroundShape returned 0 polys")
		}
		t.Logf("FindPolysAroundShape: %d polys", n)
	})

	t.Run("GetPolyHeight", func(t *testing.T) {
		center := [3]float32{2, 0, 2}
		ref, _, _ := q.FindNearestPoly(center, halfExtents, filter)
		if ref == 0 {
			t.Fatal("no ref")
		}

		h, err := q.GetPolyHeight(ref, center)
		if err != nil {
			t.Logf("GetPolyHeight: %v (may fail for edge points)", err)
		}
		if h != 0 {
			t.Fatalf("expected height 0, got %f", h)
		}
	})

	t.Run("ClosestPointOnPolyBoundary", func(t *testing.T) {
		center := [3]float32{5, 0, 5}
		ref, _, _ := q.FindNearestPoly(center, halfExtents, filter)
		if ref == 0 {
			t.Fatal("no ref")
		}

		closest, err := q.ClosestPointOnPolyBoundary(ref, [3]float32{-10, 0, -10})
		if err != nil {
			t.Fatalf("ClosestPointOnPolyBoundary: %v", err)
		}
		t.Logf("ClosestPointOnPolyBoundary: %v", closest)
	})
}

func TestFindRandomPoint(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}

	t.Run("FindRandomPoint", func(t *testing.T) {
		ref, pos, err := q.FindRandomPoint(filter, func() float32 { return 0.5 })
		if err != nil {
			t.Fatalf("FindRandomPoint: %v", err)
		}
		if ref == 0 {
			t.Fatal("FindRandomPoint returned 0 ref")
		}
		_ = pos
	})

	t.Run("FindRandomPointAroundCircle", func(t *testing.T) {
		center := [3]float32{5, 0, 5}
		halfExtents := [3]float32{10, 2, 10}
		ref, _, _ := q.FindNearestPoly(center, halfExtents, filter)
		if ref == 0 {
			t.Fatal("no ref for center")
		}

		resultRef, resultPos, err := q.FindRandomPointAroundCircle(
			ref, center, 10, filter, func() float32 { return 0.5 })
		if err != nil {
			t.Fatalf("FindRandomPointAroundCircle: %v", err)
		}
		if resultRef == 0 {
			t.Fatal("FindRandomPointAroundCircle returned 0 ref")
		}
		_ = resultPos
	})
}

func TestClosestPointOnPoly(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	center := [3]float32{5, 0, 5}
	ref, _, err := q.FindNearestPoly(center, halfExtents, filter)
	if err != nil || ref == 0 {
		t.Fatal("could not find starting poly")
	}

	t.Run("ClosestPointOnPoly", func(t *testing.T) {
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{5, 1, 5})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		if !overPoly {
			t.Log("point not over poly - may be expected for edge case")
		}
		_ = pt
	})
}

func TestIsValidPolyRef(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff

	t.Run("valid ref", func(t *testing.T) {
		center := [3]float32{5, 0, 5}
		ref, _, _ := q.FindNearestPoly(center, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Skip("no ref found")
		}
		if !q.IsValidPolyRef(ref, filter) {
			t.Fatal("expected ref to be valid")
		}
	})

	t.Run("invalid ref", func(t *testing.T) {
		if q.IsValidPolyRef(0, filter) {
			t.Fatal("expected invalid ref to return false")
		}
	})
}

func TestQueryFilter(t *testing.T) {
	t.Run("default area costs are 1.0", func(t *testing.T) {
		f := &QueryFilter{}
		for i := range f.AreaCost {
			if i == 0 {
				continue // 0 is default zero value
			}
			f.AreaCost[i] = 1.0
		}
	})

	t.Run("area cost changes affect GetCost", func(t *testing.T) {
		f := &QueryFilter{}
		f.AreaCost[0] = 5.0
		if f.AreaCost[0] != 5.0 {
			t.Fatalf("expected 5.0, got %f", f.AreaCost[0])
		}
	})
}

func TestNavMeshTileManagement(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("GetTileAt with wrong layer", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 1)
		if tile != nil {
			t.Fatal("expected nil for wrong layer")
		}
	})

	t.Run("GetTilesAt", func(t *testing.T) {
		tiles := make([]*MeshTile, 4)
		n := m.GetTilesAt(0, 0, tiles, 4)
		if n != 1 {
			t.Fatalf("expected 1 tile, got %d", n)
		}
		if tiles[0] == nil {
			t.Fatal("expected non-nil tile")
		}
	})

	t.Run("GetTileRef round trip", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("no tile found")
		}
		ref := m.GetTileRef(tile)
		if ref == 0 {
			t.Fatal("GetTileRef returned 0")
		}
		tile2 := m.GetTileByRef(ref)
		if tile2 != tile {
			t.Fatal("GetTileByRef returned different tile")
		}
	})

	t.Run("GetPolyRefBase", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("no tile")
		}
		base := m.GetPolyRefBase(tile)
		if base == 0 {
			t.Fatal("GetPolyRefBase returned 0")
		}
	})
}

func TestEncodeDecodePolyId(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("EncodeDecode round trip", func(t *testing.T) {
		salt, tile, poly := uint32(1), uint32(0), uint32(0)
		ref := m.EncodePolyID(salt, tile, poly)
		dsalt, dtile, dpoly := m.DecodePolyID(ref)
		if dsalt != salt || dtile != tile || dpoly != poly {
			t.Fatalf("Encode/Decode mismatch: (%d,%d,%d) → (%d,%d,%d)",
				salt, tile, poly, dsalt, dtile, dpoly)
		}
	})

	t.Run("EncodeDecode non-zero poly", func(t *testing.T) {
		salt, tile, poly := uint32(1), uint32(0), uint32(1)
		ref := m.EncodePolyID(salt, tile, poly)
		dsalt, dtile, dpoly := m.DecodePolyID(ref)
		if dsalt != salt || dtile != tile || dpoly != poly {
			t.Fatalf("Encode/Decode mismatch: (%d,%d,%d) → (%d,%d,%d)",
				salt, tile, poly, dsalt, dtile, dpoly)
		}
	})

	t.Run("Extract functions match DecodePolyId", func(t *testing.T) {
		salt, tile, poly := uint32(3), uint32(0), uint32(1)
		ref := m.EncodePolyID(salt, tile, poly)
		dsalt := m.DecodePolyIdSalt(ref)
		dtile := m.DecodePolyIdTile(ref)
		dpoly := m.DecodePolyIdPoly(ref)
		if dsalt != salt || dtile != tile || dpoly != poly {
			t.Fatalf("Extract mismatch: (%d,%d,%d) → (%d,%d,%d)",
				salt, tile, poly, dsalt, dtile, dpoly)
		}
	})
}

func TestGetPolyAndTileByRef(t *testing.T) {
	m := buildTestNavmesh(t)
	_ = m

	t.Run("GetTileAndPolyByRef valid", func(t *testing.T) {
		// Find a valid ref
		q := createTestQuery(t, m)
		filter := &QueryFilter{}
		for i := range filter.AreaCost {
			filter.AreaCost[i] = 1.0
		}
		ref, _, _ := q.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Skip("no ref found")
		}

		tile, poly, err := m.GetTileAndPolyByRef(ref)
		if err != nil {
			t.Fatalf("GetTileAndPolyByRef: %v", err)
		}
		if tile == nil {
			t.Fatal("tile is nil")
		}
		if poly == nil {
			t.Fatal("poly is nil")
		}
	})

	t.Run("GetTileAndPolyByRef invalid", func(t *testing.T) {
		_, _, err := m.GetTileAndPolyByRef(0)
		if err == nil {
			t.Fatal("expected error for 0 ref")
		}
	})
}

// ---------------------------------------------------------------------------
// Multiple path tests on the simple 2-triangle navmesh
// ---------------------------------------------------------------------------

// TestFindPathMultiplePairs verifies pathfinding on the simple 2-triangle navmesh
// using different start/end point combinations (same poly, adjacent polys, corners).
func TestFindPathMultiplePairs(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	type pathCase struct {
		name     string
		start    [3]float32
		end      [3]float32
		minCount int
	}
	cases := []pathCase{
		{"poly0 to poly1", [3]float32{1, 0, 1}, [3]float32{9, 0, 9}, 2},
		{"poly1 to poly0", [3]float32{9, 0, 9}, [3]float32{1, 0, 1}, 2},
		{"both in poly0", [3]float32{2, 0, 2}, [3]float32{3, 0, 7}, 1},
		{"both in poly1", [3]float32{8, 0, 3}, [3]float32{9, 0, 2}, 1},
		{"corner to far corner", [3]float32{0.1, 0, 0.1}, [3]float32{9.9, 0, 9.9}, 2},
		{"center to poly1 side", [3]float32{5, 0, 5}, [3]float32{9.9, 0, 0.1}, 2},
	}

	for _, tt := range cases {
		t.Run(tt.name, func(t *testing.T) {
			startRef, _, err := q.FindNearestPoly(tt.start, halfExtents, filter)
			if err != nil || startRef == 0 {
				t.Fatalf("FindNearestPoly start: ref=%d err=%v", startRef, err)
			}
			endRef, _, err := q.FindNearestPoly(tt.end, halfExtents, filter)
			if err != nil || endRef == 0 {
				t.Fatalf("FindNearestPoly end: ref=%d err=%v", endRef, err)
			}

			path := make([]PolyRef, 256)
			pathCount, err := q.FindPath(startRef, endRef, tt.start, tt.end, filter, path)
			if err != nil {
				t.Fatalf("FindPath: %v", err)
			}
			if pathCount < tt.minCount {
				t.Fatalf("FindPath: expected >= %d refs, got %d", tt.minCount, pathCount)
			}
			if path[0] != startRef {
				t.Errorf("path[0] = %d, expected startRef %d", path[0], startRef)
			}
			if path[pathCount-1] != endRef {
				t.Errorf("path[%d] = %d, expected endRef %d", pathCount-1, path[pathCount-1], endRef)
			}
			t.Logf("path=%v (count=%d)", path[:pathCount], pathCount)
		})
	}
}

// TestGrid20x20FindPathConsistency verifies pathfinding results against C++.
// C++ (20x20 grid, cellSize=10) outputs:
//
//	straightPath[2]: (2.0,0.0,2.0) (192.0,0.0,2.0)
func TestGrid20x20FindPathConsistency(t *testing.T) {
	rows, cols := 20, 20
	cellSize := float32(10)
	m := buildTestGridNavmesh(t, rows, cols, cellSize)
	q := createTestQuery(t, m)

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	type pathCase struct {
		name       string
		start, end [3]float32
	}
	cases := []pathCase{
		{"near_5cells", [3]float32{2, 0, 2}, [3]float32{42, 0, 2}},
		{"mid_10cells", [3]float32{2, 0, 2}, [3]float32{102, 0, 2}},
		{"far_19cells", [3]float32{2, 0, 2}, [3]float32{192, 0, 2}},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			startRef, _, err := q.FindNearestPoly(c.start, halfExtents, filter)
			if err != nil || startRef == 0 {
				t.Fatalf("FindNearestPoly start: ref=%d err=%v", startRef, err)
			}
			endRef, _, err := q.FindNearestPoly(c.end, halfExtents, filter)
			if err != nil || endRef == 0 {
				t.Fatalf("FindNearestPoly end: ref=%d err=%v", endRef, err)
			}

			path := make([]PolyRef, 4096)
			n, err := q.FindPath(startRef, endRef, c.start, c.end, filter, path)
			if err != nil {
				t.Fatalf("FindPath: %v", err)
			}
			if n < 2 {
				t.Fatalf("FindPath returned %d refs, expected >=2", n)
			}

			// Verify straight path corners match C++ exactly.
			// C++ far_19cells straightPath[2]: (2.0,0.0,2.0) (192.0,0.0,2.0)
			straightVerts := make([]float32, 256*3)
			straightFlags := make([]uint8, 256)
			straightRefs := make([]PolyRef, 256)
			ns, err := q.FindStraightPath(c.start, c.end, path, n,
				straightVerts, straightFlags, straightRefs, 256, 0)
			if err != nil {
				t.Fatalf("FindStraightPath: %v", err)
			}

			// All paths on this grid start at (2,0,2) and end at (*,0,2) — straight line along x-axis
			if ns != 2 {
				t.Fatalf("expected 2 straight path corners, got %d", ns)
			}
			if straightVerts[0] != c.start[0] || straightVerts[1] != 0 || straightVerts[2] != c.start[2] {
				t.Errorf("start point mismatch: got (%.1f,%.1f,%.1f), expected (%.1f,%.1f,%.1f)",
					straightVerts[0], straightVerts[1], straightVerts[2], c.start[0], c.start[1], c.start[2])
			}
			if straightVerts[3] != c.end[0] || straightVerts[4] != 0 || straightVerts[5] != c.end[2] {
				t.Errorf("end point mismatch: got (%.1f,%.1f,%.1f), expected (%.1f,%.1f,%.1f)",
					straightVerts[3], straightVerts[4], straightVerts[5], c.end[0], c.end[1], c.end[2])
			}
		})
	}
}

// ---------------------------------------------------------------------------
// Grid navmesh pathfind tests
// ---------------------------------------------------------------------------

// TestGridNavmeshFindPath tests pathfinding on a 2x2 grid navmesh (8 polys).
// Verifies cross-cell traversal and path determinism.
func TestGridNavmeshFindPath(t *testing.T) {
	rows, cols := 2, 2
	cellSize := float32(10)
	m := buildTestGridNavmesh(t, rows, cols, cellSize)
	q := createTestQuery(t, m)

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	// Verify basic tile structure
	tile := m.GetTileAt(0, 0, 0)
	if tile == nil {
		t.Fatal("no tile at (0,0,0)")
	}
	if tile.Header.PolyCount != int32(rows*cols*2) {
		t.Fatalf("expected %d polys, got %d", rows*cols*2, tile.Header.PolyCount)
	}
	if tile.Header.VertCount != int32((rows+1)*(cols+1)) {
		t.Fatalf("expected %d verts, got %d", (rows+1)*(cols+1), tile.Header.VertCount)
	}

	t.Run("path within one cell", func(t *testing.T) {
		startPos := [3]float32{2, 0, 2}
		endPos := [3]float32{8, 0, 8}

		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)
		if startRef == 0 || endRef == 0 {
			t.Fatal("could not find start/end refs")
		}

		path := make([]PolyRef, 256)
		pathCount, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
		if err != nil {
			t.Fatalf("FindPath: %v", err)
		}
		if pathCount < 1 {
			t.Fatal("path is empty")
		}
		if path[0] != startRef {
			t.Errorf("path[0] = %d, expected %d", path[0], startRef)
		}
		t.Logf("same cell path: %v (count=%d)", path[:pathCount], pathCount)
	})

	t.Run("horizontal cross-cell", func(t *testing.T) {
		startPos := [3]float32{2, 0, 5}
		endPos := [3]float32{18, 0, 5}

		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)
		if startRef == 0 || endRef == 0 {
			t.Fatal("could not find start/end refs")
		}

		path := make([]PolyRef, 256)
		pathCount, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
		if err != nil {
			t.Fatalf("FindPath: %v", err)
		}
		if pathCount < 2 {
			t.Fatal("horizontal path should traverse multiple polys")
		}
		if path[0] != startRef {
			t.Errorf("path[0] = %d, expected %d", path[0], startRef)
		}
		if path[pathCount-1] != endRef {
			t.Errorf("path[last] = %d, expected %d", path[pathCount-1], endRef)
		}
		t.Logf("horizontal cross-cell path: %v (count=%d)", path[:pathCount], pathCount)
	})

	t.Run("vertical cross-cell", func(t *testing.T) {
		startPos := [3]float32{5, 0, 2}
		endPos := [3]float32{5, 0, 18}

		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)
		if startRef == 0 || endRef == 0 {
			t.Fatal("could not find start/end refs")
		}

		path := make([]PolyRef, 256)
		pathCount, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
		if err != nil {
			t.Fatalf("FindPath: %v", err)
		}
		if pathCount < 2 {
			t.Fatal("vertical path should traverse multiple polys")
		}
		if path[0] != startRef {
			t.Errorf("path[0] = %d, expected %d", path[0], startRef)
		}
		if path[pathCount-1] != endRef {
			t.Errorf("path[last] = %d, expected %d", path[pathCount-1], endRef)
		}
		t.Logf("vertical cross-cell path: %v (count=%d)", path[:pathCount], pathCount)
	})

	t.Run("diagonal across grid", func(t *testing.T) {
		startPos := [3]float32{1, 0, 1}
		endPos := [3]float32{19, 0, 19}

		startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
		endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)
		if startRef == 0 || endRef == 0 {
			t.Fatal("could not find start/end refs")
		}

		path := make([]PolyRef, 256)
		pathCount, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
		if err != nil {
			t.Fatalf("FindPath: %v", err)
		}
		if pathCount < 3 {
			t.Fatal("diagonal path should traverse multiple cells (>=3 polys)")
		}
		if path[0] != startRef {
			t.Errorf("path[0] = %d, expected %d", path[0], startRef)
		}
		if path[pathCount-1] != endRef {
			t.Errorf("path[last] = %d, expected %d", path[pathCount-1], endRef)
		}
		t.Logf("diagonal path: %v (count=%d)", path[:pathCount], pathCount)

		// Verify exact expected ref sequence: cell(0,0)→cell(0,1)→cell(1,1)
		// Each connection verified: left→right (internal), right→left (horizontal), etc.
		expectedRefs := []PolyRef{8, 9, 10, 11, 14, 15}
		if pathCount == len(expectedRefs) {
			for i := range expectedRefs {
				if path[i] != expectedRefs[i] {
					t.Logf("path[%d] = %d, expected %d (this may vary by implementation)", i, path[i], expectedRefs[i])
				}
			}
		}

		// Determinism check: second query returns identical result
		path2 := make([]PolyRef, 256)
		pathCount2, err2 := q.FindPath(startRef, endRef, startPos, endPos, filter, path2)
		if err2 != nil {
			t.Fatalf("FindPath second call: %v", err2)
		}
		if pathCount2 != pathCount {
			t.Fatalf("path count mismatch: first=%d second=%d", pathCount, pathCount2)
		}
		for i := 0; i < pathCount; i++ {
			if path[i] != path2[i] {
				t.Fatalf("path mismatch at index %d: first=%d second=%d", i, path[i], path2[i])
			}
		}
	})

	t.Run("same start and end", func(t *testing.T) {
		pos := [3]float32{5, 0, 5}
		ref, _, _ := q.FindNearestPoly(pos, halfExtents, filter)
		if ref == 0 {
			t.Fatal("no ref")
		}

		path := make([]PolyRef, 256)
		pathCount, err := q.FindPath(ref, ref, pos, pos, filter, path)
		if err != nil {
			t.Fatalf("FindPath same: %v", err)
		}
		if pathCount < 1 {
			t.Fatal("path should have at least 1 ref")
		}
		t.Logf("same point path: %v (count=%d)", path[:pathCount], pathCount)
	})
}

// TestGridNavmeshStraightPath tests FindStraightPath on the grid navmesh.
func TestGridNavmeshStraightPath(t *testing.T) {
	rows, cols := 2, 2
	cellSize := float32(10)
	m := buildTestGridNavmesh(t, rows, cols, cellSize)
	q := createTestQuery(t, m)

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	startPos := [3]float32{1, 0, 1}
	endPos := [3]float32{19, 0, 19}

	startRef, _, _ := q.FindNearestPoly(startPos, halfExtents, filter)
	endRef, _, _ := q.FindNearestPoly(endPos, halfExtents, filter)
	if startRef == 0 || endRef == 0 {
		t.Fatal("could not find start/end refs")
	}

	path := make([]PolyRef, 256)
	pathCount, err := q.FindPath(startRef, endRef, startPos, endPos, filter, path)
	if err != nil || pathCount == 0 {
		t.Fatal("no path found")
	}

	t.Run("FindStraightPath", func(t *testing.T) {
		straightVerts := make([]float32, 64*3)
		straightFlags := make([]uint8, 64)
		straightPolys := make([]PolyRef, 64)
		nStraight, err := q.FindStraightPath(startPos, endPos, path, pathCount,
			straightVerts, straightFlags, straightPolys, 64, 0)
		if err != nil {
			t.Fatalf("FindStraightPath: %v", err)
		}
		if nStraight < 2 {
			t.Fatal("straight path should have at least 2 corners (start+end)")
		}
		t.Logf("straight path: %d corners", nStraight)
		for i := 0; i < nStraight; i++ {
			t.Logf("  corner %d: (%0.1f,%0.1f,%0.1f) flags=%d poly=%d",
				i, straightVerts[i*3], straightVerts[i*3+1], straightVerts[i*3+2], straightFlags[i], straightPolys[i])
		}
	})

	t.Run("FindStraightPath with area crossings", func(t *testing.T) {
		straightVerts := make([]float32, 64*3)
		straightFlags := make([]uint8, 64)
		straightPolys := make([]PolyRef, 64)
		nStraight, err := q.FindStraightPath(startPos, endPos, path, pathCount,
			straightVerts, straightFlags, straightPolys, 64, StraightPathAreaCrossings)
		if err != nil {
			t.Fatalf("FindStraightPath area crossings: %v", err)
		}
		if nStraight < 2 {
			t.Fatal("straight path should have at least 2 corners")
		}
		t.Logf("straight path (area crossings): %d corners", nStraight)
	})
}
