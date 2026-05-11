package detour_crowd

import (
	"testing"
)

// mockNavQueryForBoundary implements NavMeshQueryInterface for LocalBoundary tests.
type mockNavQueryForBoundary struct {
	isValidPolyRefFunc func(PolyRef, *QueryFilter) bool
}

func (m *mockNavQueryForBoundary) FindNearestPoly(pos [3]float32, halfExtents [3]float32, filter *QueryFilter) (PolyRef, [3]float32, error) {
	return 0, [3]float32{}, nil
}

func (m *mockNavQueryForBoundary) IsValidPolyRef(ref PolyRef, filter *QueryFilter) bool {
	if m.isValidPolyRefFunc != nil {
		return m.isValidPolyRefFunc(ref, filter)
	}
	return true
}

func (m *mockNavQueryForBoundary) MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result []float32, visited []PolyRef, maxVisitedSize int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForBoundary) GetPolyHeight(ref PolyRef, pos [3]float32) (float32, error) {
	return 0, nil
}

func (m *mockNavQueryForBoundary) ClosestPointOnPoly(ref PolyRef, pos [3]float32) ([3]float32, bool, error) {
	return [3]float32{}, true, nil
}

func (m *mockNavQueryForBoundary) FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, maxStraightPath int, options int) ([]float32, []uint8, []PolyRef, int, error) {
	return nil, nil, nil, 0, nil
}

func (m *mockNavQueryForBoundary) Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32, prevRef PolyRef, hit *RaycastHit) error {
	return nil
}

func (m *mockNavQueryForBoundary) FindPathSliced(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
	return nil
}

func (m *mockNavQueryForBoundary) UpdateSlicedPath(maxIter int) error {
	return nil
}

func (m *mockNavQueryForBoundary) GetPathFromSlicedPath(path []PolyRef, maxPath int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForBoundary) GetAttachedNavMesh() *NavMesh {
	return nil
}

func (m *mockNavQueryForBoundary) ClosestPointOnPolyBoundary(ref PolyRef, pos [3]float32) ([3]float32, error) {
	return [3]float32{}, nil
}

func (m *mockNavQueryForBoundary) FindPolysAroundCircle(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForBoundary) FindLocalNeighbourhood(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, maxResult int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForBoundary) GetPolyWallSegments(ref PolyRef, filter *QueryFilter, segs []NeighbourSeg, maxSegs int) (int, error) {
	return 0, nil
}

func TestLocalBoundaryNewAndReset(t *testing.T) {
	t.Run("NewLocalBoundary should initialize to reset state", func(t *testing.T) {
		b := NewLocalBoundary()

		if b.nsegs != 0 {
			t.Errorf("nsegs = %d, want 0", b.nsegs)
		}
		if b.npolys != 0 {
			t.Errorf("npolys = %d, want 0", b.npolys)
		}
		if b.center[0] != mathMaxFloat32 || b.center[1] != mathMaxFloat32 || b.center[2] != mathMaxFloat32 {
			t.Errorf("center = %v, want all mathMaxFloat32", b.center)
		}
	})

	t.Run("Reset should clear all data", func(t *testing.T) {
		b := NewLocalBoundary()

		s := [6]float32{0, 0, 0, 10, 0, 10}
		b.addSegment(5.0, &s)
		b.polys[0] = 42
		b.npolys = 1

		if b.nsegs != 1 {
			t.Errorf("nsegs = %d, want 1 before reset", b.nsegs)
		}

		b.Reset()

		if b.nsegs != 0 {
			t.Errorf("after Reset nsegs = %d, want 0", b.nsegs)
		}
		if b.npolys != 0 {
			t.Errorf("after Reset npolys = %d, want 0", b.npolys)
		}
		if b.center[0] != mathMaxFloat32 || b.center[1] != mathMaxFloat32 || b.center[2] != mathMaxFloat32 {
			t.Errorf("after Reset center = %v, want all mathMaxFloat32", b.center)
		}
	})
}

func TestLocalBoundaryAddSegment(t *testing.T) {
	t.Run("should add first segment", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 10, 0, 10}
		b.addSegment(5.0, &s)

		if b.nsegs != 1 {
			t.Errorf("nsegs = %d, want 1", b.nsegs)
		}
		if b.segs[0].d != 5.0 {
			t.Errorf("segs[0].d = %f, want 5.0", b.segs[0].d)
		}
		if b.segs[0].s != s {
			t.Errorf("segs[0].s = %v, want %v", b.segs[0].s, s)
		}
	})

	t.Run("should keep segments sorted by distance ascending", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		b.addSegment(10.0, &s)
		b.addSegment(5.0, &s)
		b.addSegment(1.0, &s)
		b.addSegment(3.0, &s)

		if b.nsegs != 4 {
			t.Errorf("nsegs = %d, want 4", b.nsegs)
		}

		expected := []float32{1.0, 3.0, 5.0, 10.0}
		for i, exp := range expected {
			if b.segs[i].d != exp {
				t.Errorf("segs[%d].d = %f, want %f", i, b.segs[i].d, exp)
			}
		}
	})

	t.Run("should insert segment in the middle", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		b.addSegment(10.0, &s)
		b.addSegment(5.0, &s)
		b.addSegment(7.5, &s)

		if b.nsegs != 3 {
			t.Errorf("nsegs = %d, want 3", b.nsegs)
		}

		if b.segs[0].d != 5.0 {
			t.Errorf("segs[0].d = %f, want 5.0", b.segs[0].d)
		}
		if b.segs[1].d != 7.5 {
			t.Errorf("segs[1].d = %f, want 7.5", b.segs[1].d)
		}
		if b.segs[2].d != 10.0 {
			t.Errorf("segs[2].d = %f, want 10.0", b.segs[2].d)
		}
	})

	t.Run("should append segment with largest distance at the end", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		b.addSegment(1.0, &s)
		b.addSegment(3.0, &s)
		b.addSegment(2.0, &s) // goes in the middle
		b.addSegment(5.0, &s) // goes at the end

		if b.nsegs != 4 {
			t.Errorf("nsegs = %d, want 4", b.nsegs)
		}

		if b.segs[3].d != 5.0 {
			t.Errorf("segs[3].d = %f, want 5.0 (largest at end)", b.segs[3].d)
		}
	})

	t.Run("should not exceed max segments limit", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		for i := 0; i < maxLocalSegs+5; i++ {
			b.addSegment(float32(i+1)*10.0, &s)
		}

		if b.nsegs != maxLocalSegs {
			t.Errorf("nsegs = %d, want %d", b.nsegs, maxLocalSegs)
		}
	})

	t.Run("should drop furthest segment when inserting at capacity", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		for i := 0; i < maxLocalSegs; i++ {
			b.addSegment(float32(i+1)*10.0, &s)
		}

		// Insert a closer segment (should displace the furthest)
		b.addSegment(5.0, &s)

		if b.nsegs != maxLocalSegs {
			t.Errorf("nsegs = %d, want %d", b.nsegs, maxLocalSegs)
		}
		if b.segs[0].d != 5.0 {
			t.Errorf("segs[0].d = %f, want 5.0 (inserted closer segment)", b.segs[0].d)
		}
		// The segment with distance 80 should be displaced
		if b.segs[maxLocalSegs-1].d == 80.0 {
			t.Errorf("furthest segment 80.0 was not displaced by insertion")
		}
	})

	t.Run("should reject segment further than the last at capacity", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		for i := 0; i < maxLocalSegs; i++ {
			b.addSegment(float32(i+1)*10.0, &s)
		}

		b.addSegment(200.0, &s)

		if b.nsegs != maxLocalSegs {
			t.Errorf("nsegs = %d, want %d", b.nsegs, maxLocalSegs)
		}
		if b.segs[maxLocalSegs-1].d != 80.0 {
			t.Errorf("segs[%d].d = %f, want 80.0 (furthest should remain unchanged)", maxLocalSegs-1, b.segs[maxLocalSegs-1].d)
		}
	})

	t.Run("should preserve segment data after insertion", func(t *testing.T) {
		b := NewLocalBoundary()

		s1 := [6]float32{1, 2, 3, 4, 5, 6}
		s2 := [6]float32{7, 8, 9, 10, 11, 12}
		s3 := [6]float32{13, 14, 15, 16, 17, 18}

		b.addSegment(10.0, &s1)
		b.addSegment(30.0, &s2)
		b.addSegment(20.0, &s3) // insert between s1 and s2

		if b.segs[0].s != s1 {
			t.Errorf("segs[0].s changed after insertion")
		}
		if b.segs[1].s != s3 {
			t.Errorf("segs[1].s = %v, want %v (inserted segment)", b.segs[1].s, s3)
		}
		if b.segs[2].s != s2 {
			t.Errorf("segs[2].s = %v, want %v", b.segs[2].s, s2)
		}
	})
}

func TestLocalBoundaryGetCenter(t *testing.T) {
	t.Run("GetCenter should return current center", func(t *testing.T) {
		b := NewLocalBoundary()
		center := b.GetCenter()
		if center == nil {
			t.Errorf("GetCenter returned nil")
		}
		if *center != b.center {
			t.Errorf("GetCenter returned %v, want %v", *center, b.center)
		}
	})
}

func TestLocalBoundaryGetSegmentCount(t *testing.T) {
	t.Run("should return zero initially", func(t *testing.T) {
		b := NewLocalBoundary()
		if b.GetSegmentCount() != 0 {
			t.Errorf("GetSegmentCount = %d, want 0", b.GetSegmentCount())
		}
	})

	t.Run("should return correct count after adding segments", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 1, 0, 1}

		b.addSegment(1.0, &s)
		b.addSegment(2.0, &s)

		if b.GetSegmentCount() != 2 {
			t.Errorf("GetSegmentCount = %d, want 2", b.GetSegmentCount())
		}
	})
}

func TestLocalBoundaryGetSegment(t *testing.T) {
	t.Run("should return segment at index", func(t *testing.T) {
		b := NewLocalBoundary()
		s := [6]float32{0, 0, 0, 10, 0, 10}
		b.addSegment(5.0, &s)

		seg := b.GetSegment(0)
		if seg == nil {
			t.Errorf("GetSegment(0) returned nil")
			return
		}
		if *seg != s {
			t.Errorf("GetSegment(0) = %v, want %v", *seg, s)
		}
	})
}

func TestLocalBoundaryIsValid(t *testing.T) {
	t.Run("should return false when npolys is zero", func(t *testing.T) {
		b := NewLocalBoundary()
		mock := &mockNavQueryForBoundary{}
		if b.IsValid(mock, nil) {
			t.Errorf("IsValid with npolys=0 should return false")
		}
	})

	t.Run("should return true when all polys are valid", func(t *testing.T) {
		b := NewLocalBoundary()
		b.polys[0] = 1
		b.polys[1] = 2
		b.npolys = 2

		mock := &mockNavQueryForBoundary{
			isValidPolyRefFunc: func(ref PolyRef, filter *QueryFilter) bool {
				return ref == 1 || ref == 2
			},
		}

		if !b.IsValid(mock, nil) {
			t.Errorf("IsValid with valid polys should return true")
		}
	})

	t.Run("should return false when any poly is invalid", func(t *testing.T) {
		b := NewLocalBoundary()
		b.polys[0] = 1
		b.polys[1] = 2
		b.npolys = 2

		mock := &mockNavQueryForBoundary{
			isValidPolyRefFunc: func(ref PolyRef, filter *QueryFilter) bool {
				return ref != 2 // only ref 1 is valid
			},
		}

		if b.IsValid(mock, nil) {
			t.Errorf("IsValid should return false when poly 2 is invalid")
		}
	})

	t.Run("should short-circuit on first invalid poly", func(t *testing.T) {
		b := NewLocalBoundary()
		b.polys[0] = 1
		b.polys[1] = 2
		b.polys[2] = 3
		b.npolys = 3

		var checkedRefs []PolyRef
		mock := &mockNavQueryForBoundary{
			isValidPolyRefFunc: func(ref PolyRef, filter *QueryFilter) bool {
				checkedRefs = append(checkedRefs, ref)
				return ref != 2 // ref 2 is invalid
			},
		}

		b.IsValid(mock, nil)

		// Should have only checked ref 1 and 2, not reached ref 3
		if len(checkedRefs) != 2 || checkedRefs[0] != 1 || checkedRefs[1] != 2 {
			t.Errorf("expected short-circuit at ref 2, checked refs = %v", checkedRefs)
		}
	})

	t.Run("should not check beyond npolys count", func(t *testing.T) {
		b := NewLocalBoundary()
		b.polys[0] = 1
		b.polys[1] = 99 // slot within npolys
		b.polys[2] = 999 // slot beyond npolys
		b.npolys = 2

		var checkedRefs []PolyRef
		mock := &mockNavQueryForBoundary{
			isValidPolyRefFunc: func(ref PolyRef, filter *QueryFilter) bool {
				checkedRefs = append(checkedRefs, ref)
				return true
			},
		}

		b.IsValid(mock, nil)

		for _, r := range checkedRefs {
			if r == 999 {
				t.Errorf("IsValid checked poly at index 2 which is beyond npolys=2")
			}
		}
	})

	t.Run("should pass through the filter parameter", func(t *testing.T) {
		b := NewLocalBoundary()
		b.polys[0] = 1
		b.npolys = 1

		calledWithFilter := false
		fakeFilter := &QueryFilter{}

		mock := &mockNavQueryForBoundary{
			isValidPolyRefFunc: func(ref PolyRef, filter *QueryFilter) bool {
				if filter == fakeFilter {
					calledWithFilter = true
				}
				return true
			},
		}

		b.IsValid(mock, fakeFilter)

		if !calledWithFilter {
			t.Errorf("IsValid should pass filter to IsValidPolyRef")
		}
	})
}
