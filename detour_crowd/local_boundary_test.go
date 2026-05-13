package detour_crowd

import (
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

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
	navQuery := createTestNavMeshQuery(t)

	t.Run("should return false when npolys is zero", func(t *testing.T) {
		b := NewLocalBoundary()
		if b.IsValid(navQuery, nil) {
			t.Errorf("IsValid with npolys=0 should return false")
		}
	})

	t.Run("should return true when all polys are valid", func(t *testing.T) {
		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		// Get a real valid ref from the navmesh
		ref, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || ref == 0 {
			t.Fatalf("FindNearestPoly failed")
		}

		b := NewLocalBoundary()
		b.polys[0] = ref
		b.polys[1] = ref
		b.npolys = 2

		if !b.IsValid(navQuery, filter) {
			t.Errorf("IsValid with valid polys should return true")
		}
	})

	t.Run("should return false when any poly is invalid", func(t *testing.T) {
		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		ref, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || ref == 0 {
			t.Fatalf("FindNearestPoly failed")
		}

		b := NewLocalBoundary()
		b.polys[0] = ref
		b.polys[1] = 0 // invalid ref
		b.npolys = 2

		if b.IsValid(navQuery, filter) {
			t.Errorf("IsValid should return false when poly 1 is 0 (invalid)")
		}
	})

	t.Run("should short-circuit on first invalid poly", func(t *testing.T) {
		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		ref, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || ref == 0 {
			t.Fatalf("FindNearestPoly failed")
		}

		b := NewLocalBoundary()
		b.polys[0] = ref
		b.polys[1] = 0 // invalid ref — should short-circuit here
		b.polys[2] = ref
		b.npolys = 3

		if b.IsValid(navQuery, filter) {
			t.Errorf("IsValid should return false when polis[1] is invalid")
		}
	})

	t.Run("should not check beyond npolys count", func(t *testing.T) {
		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff

		ref, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || ref == 0 {
			t.Fatalf("FindNearestPoly failed")
		}

		b := NewLocalBoundary()
		b.polys[0] = ref
		b.polys[1] = ref
		b.polys[2] = 0 // beyond npolys, should be ignored
		b.npolys = 2

		if !b.IsValid(navQuery, filter) {
			t.Errorf("IsValid should return true since slot 2 is beyond npolys")
		}
	})

	t.Run("should pass through the filter parameter", func(t *testing.T) {
		// Create a filter that excludes all polys
		excludeFilter := detour.NewQueryFilter()
		excludeFilter.ExcludeFlags = 0xffff
		excludeFilter.IncludeFlags = 0xffff

		includeFilter := detour.NewQueryFilter()
		includeFilter.IncludeFlags = 0xffff

		ref, _, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, includeFilter)
		if err != nil || ref == 0 {
			t.Fatalf("FindNearestPoly failed")
		}

		b := NewLocalBoundary()
		b.polys[0] = ref
		b.npolys = 1

		// With excludeFilter, the ref should be invalid
		if b.IsValid(navQuery, excludeFilter) {
			t.Errorf("IsValid should return false with filter that excludes all polys")
		}

		// With includeFilter, the ref should be valid
		if !b.IsValid(navQuery, includeFilter) {
			t.Errorf("IsValid should return true with filter that includes the poly")
		}
	})
}
