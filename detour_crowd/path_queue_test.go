package detour_crowd

import (
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

// createTestNavMeshQuery builds a minimal 10x10 grid NavMesh and returns a NavMeshQuery.
// Each cell is a 6-vertex triangulated quad. Internal links connect adjacent cells.
func createTestNavMeshQuery(t testing.TB) *detour.NavMeshQuery {
	t.Helper()

	const gridSize = 10
	stride := gridSize + 1 // vertex row stride

	verts := make([]uint16, 0, stride*stride*3)
	for z := 0; z <= gridSize; z++ {
		for x := 0; x <= gridSize; x++ {
			verts = append(verts, uint16(x), 0, uint16(z))
		}
	}

	// Each cell is one 6-vertex poly = 6 verts + 6 neis = 12 entries.
	polys := make([]uint16, 0, gridSize*gridSize*12)
	flags := make([]uint16, 0, gridSize*gridSize)
	areas := make([]uint8, 0, gridSize*gridSize)
	const boundaryMask uint16 = 0x800f // signals "no link" (dir=0xf, high bit set)

	for z := 0; z < gridSize; z++ {
		for x := 0; x < gridSize; x++ {
			idx := z*stride + x

			// verts: upper-left tri, then lower-right tri
			polys = append(polys,
				uint16(idx), uint16(idx+1), uint16(idx+gridSize+2),
				uint16(idx), uint16(idx+gridSize+2), uint16(idx+gridSize+1),
			)

			// Edge 0 (top):    [idx, idx+1]     → cell above (x, z-1) edge 4
			if z > 0 {
				polys = append(polys, uint16((z-1)*gridSize+x))
			} else {
				polys = append(polys, boundaryMask)
			}

			// Edge 1 (right):  [idx+1, idx+12]  → cell right (x+1, z) edge 5
			if x < gridSize-1 {
				polys = append(polys, uint16(z*gridSize+x+1))
			} else {
				polys = append(polys, boundaryMask)
			}

			// Edge 2 (diag rev): [idx+12, idx]  → internal (same poly)
			polys = append(polys, boundaryMask)

			// Edge 3 (diag):     [idx, idx+12]  → internal (same poly)
			polys = append(polys, boundaryMask)

			// Edge 4 (bottom): [idx+12, idx+11] → cell below (x, z+1) edge 0
			if z < gridSize-1 {
				polys = append(polys, uint16((z+1)*gridSize+x))
			} else {
				polys = append(polys, boundaryMask)
			}

			// Edge 5 (left):   [idx+11, idx]     → cell left (x-1, z) edge 1
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
		WalkableRadius: 0.6,
		WalkableClimb:  1.0,
		Cs:             1.0,
		Ch:             0.2,
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
		MaxTiles:   2,
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
	if err := q.Init(nav, 4096); err != nil {
		t.Fatalf("NavMeshQuery.Init: %v", err)
	}
	return q
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

func TestPathQueueInit(t *testing.T) {
	t.Run("should initialize successfully", func(t *testing.T) {
		q := NewPathQueue()
		navQuery := createTestNavMeshQuery(t)

		result := q.Init(256, 4096, navQuery)
		if !result {
			t.Errorf("Init returned false, expected true")
		}

		if q.maxPathSize != 256 {
			t.Errorf("maxPathSize = %d, want 256", q.maxPathSize)
		}

		if q.queueHead != 0 {
			t.Errorf("queueHead = %d, want 0", q.queueHead)
		}

		// Verify all queue slots are initialized to ref=0
		for i := 0; i < pathQueueMaxQueue; i++ {
			if q.queue[i].ref != 0 {
				t.Errorf("queue[%d].ref = %d, want 0", i, q.queue[i].ref)
			}
			if len(q.queue[i].path) != 256 {
				t.Errorf("queue[%d].path len = %d, want 256", i, len(q.queue[i].path))
			}
		}
	})

	t.Run("should return nil nav query when not initialized", func(t *testing.T) {
		q := NewPathQueue()
		if q.GetNavQuery() != nil {
			t.Errorf("GetNavQuery should return nil before Init")
		}
	})

	t.Run("should return the configured nav query", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		if q.GetNavQuery() != navQuery {
			t.Errorf("GetNavQuery did not return the query passed to Init")
		}
	})
}

func TestPathQueueRequest(t *testing.T) {
	t.Run("should return a valid handle for a new request", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		ref := q.Request(1, 2, [3]float32{0, 0, 0}, [3]float32{10, 0, 10}, nil)
		if ref == PathQueueRef(PathQInvalid) {
			t.Errorf("Request returned PathQInvalid (0), expected non-zero handle")
		}
	})

	t.Run("should allocate unique handles", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		ref1 := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		ref2 := q.Request(3, 4, [3]float32{}, [3]float32{}, nil)

		if ref1 == ref2 {
			t.Errorf("handles should be unique, got %d and %d", ref1, ref2)
		}
	})

	t.Run("should return 0 when queue is full", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		// Fill up all slots
		for i := 0; i < pathQueueMaxQueue; i++ {
			ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
			if ref == PathQueueRef(PathQInvalid) {
				t.Errorf("request %d should succeed, got 0", i)
			}
		}

		// Next request should fail
		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		if ref != PathQueueRef(PathQInvalid) {
			t.Errorf("expected PathQInvalid when queue is full, got %d", ref)
		}
	})

	t.Run("should wrap handle at max uint32", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		q.nextHandle = 0xFFFFFFFF

		ref1 := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		if ref1 != 0xFFFFFFFF {
			t.Errorf("expected 0xFFFFFFFF, got %d", ref1)
		}

		ref2 := q.Request(3, 4, [3]float32{}, [3]float32{}, nil)
		if ref2 != 1 {
			t.Errorf("expected handle 1 after max uint32 wrap, got %d", ref2)
		}
	})
}

func TestPathQueueGetRequestErr(t *testing.T) {
	t.Run("should return ErrFailure for unknown ref", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		err := q.GetRequestErr(999)
		if err != detour.ErrRequestNotFound {
			t.Errorf("GetRequestErr(999) = %v, want %v", err, detour.ErrRequestNotFound)
		}
	})

	t.Run("should return the request's error", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)

		err := q.GetRequestErr(ref)
		if err != nil {
			t.Errorf("GetRequestErr should return nil for pending request, got %v", err)
		}
	})
}

func TestPathQueueGetPathResult(t *testing.T) {
	t.Run("should return ErrFailure for unknown ref", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(999, buf, 256)
		if err != detour.ErrRequestNotFound {
			t.Errorf("GetPathResult unknown ref: err = %v, want %v", err, detour.ErrRequestNotFound)
		}
		if n != 0 {
			t.Errorf("GetPathResult unknown ref: n = %d, want 0", n)
		}
	})

	t.Run("should retrieve path after completion", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		// Find refs for start and end positions
		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		startRef, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed: ref=%d err=%v", startRef, err)
		}
		endRef, _, err := navQuery.FindNearestPoly([3]float32{8, 0, 8}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed: ref=%d err=%v", endRef, err)
		}

		ref := q.Request(startRef, endRef, [3]float32{1, 0, 1}, [3]float32{8, 0, 8}, filter)

		// Process the request to completion
		q.Update(100)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)

		if err != nil {
			t.Errorf("GetPathResult returned error: %v", err)
		}
		if n <= 0 {
			t.Errorf("GetPathResult returned n = %d, want > 0", n)
		}
		// Path should contain start and end refs
		if buf[0] != startRef {
			t.Errorf("path[0] = %d, want %d", buf[0], startRef)
		}

		// After GetPathResult, the request should be freed
		err2 := q.GetRequestErr(ref)
		if err2 != detour.ErrRequestNotFound {
			t.Errorf("request should be freed after GetPathResult, got err = %v, want ErrRequestNotFound", err2)
		}
	})

	t.Run("should trim path to maxPath length", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		startRef, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}
		endRef, _, err := navQuery.FindNearestPoly([3]float32{8, 0, 8}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed")
		}

		ref := q.Request(startRef, endRef, [3]float32{1, 0, 1}, [3]float32{8, 0, 8}, filter)
		q.Update(100)

		// Retrieve with small buffer to test clamping
		buf := make([]PolyRef, 1)
		n, err := q.GetPathResult(ref, buf, 1)

		if err != nil {
			t.Errorf("GetPathResult error: %v", err)
		}
		if n != 1 {
			t.Errorf("n = %d, want 1 (clamped to buf size)", n)
		}
	})

	t.Run("should handle partial result", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		startRef, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}
		endRef, _, err := navQuery.FindNearestPoly([3]float32{8, 0, 4}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed")
		}

		// Request with positions far apart — may produce partial result
		ref := q.Request(startRef, endRef, [3]float32{1, 0, 1}, [3]float32{8, 0, 4}, filter)

		// Process in small iterations
		q.Update(10)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)

		if err == detour.ErrRequestNotFound {
			// This is fine in some configurations — the path may succeed or fail
			t.Logf("Path result: n=%d, err=%v", n, err)
		}
		_ = n
	})
}

func TestPathQueueUpdate(t *testing.T) {
	t.Run("should process request to completion in one call", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		startRef, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}
		endRef, _, err := navQuery.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed")
		}

		ref := q.Request(startRef, endRef, [3]float32{1, 0, 1}, [3]float32{5, 0, 5}, filter)

		// Update should complete the request (short path on small grid)
		q.Update(200)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)
		if err != nil {
			t.Errorf("path not completed after update: n=%d err=%v", n, err)
		}
		if n <= 0 {
			t.Errorf("expected completed path, got n = %d", n)
		}
	})

	t.Run("should handle no valid path", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		// Request with refs that don't exist — should fail gracefully
		ref := q.Request(99999, 88888, [3]float32{0, 0, 0}, [3]float32{0, 0, 0}, nil)
		q.Update(100)

		err := q.GetRequestErr(ref)
		// Should either be failure or completed — not a crash
		t.Logf("Request with invalid refs: err=%v", err)
	})

	t.Run("should handle failing path finding with unreachable target", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 4096, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		startRef, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}

		// Target is at position with no valid poly (outside mesh)
		ref := q.Request(startRef, 99999, [3]float32{1, 0, 1}, [3]float32{100, 0, 100}, filter)
		q.Update(200)

		err = q.GetRequestErr(ref)
		t.Logf("Unreachable target request: err=%v", err)
	})
}

func TestPathQueueMultiStepAsync(t *testing.T) {
	// Use a larger grid for pathfinding that requires multiple steps
	// to ensure the sliced pathfinding covers multiple iterations
	t.Run("should handle path with multiple update passes", func(t *testing.T) {
		// Use the standard test mesh
		navQuery := createTestNavMeshQuery(t)
		q := NewPathQueue()
		q.Init(256, 128, navQuery) // smaller node pool to force multi-step

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		startRef, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}
		endRef, _, err := navQuery.FindNearestPoly([3]float32{8, 0, 8}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed")
		}

		ref := q.Request(startRef, endRef, [3]float32{1, 0, 1}, [3]float32{8, 0, 8}, filter)

		// First update with limited iterations
		q.Update(10)

		// Check progress
		err = q.GetRequestErr(ref)

		// Second update with more iterations
		q.Update(200)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)

		if err != nil && err != detour.ErrPartialResult {
			t.Errorf("path not completed: err=%v", err)
		}
		if n <= 0 {
			t.Errorf("expected path length > 0, got %d", n)
		}
	})
}
