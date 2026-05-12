package detour

import (
	"math"
	"testing"
)

func TestRandomPointInConvexPoly(t *testing.T) {
	pts := []float32{
		0, 0, 0,
		0, 0, 1,
		1, 0, 0,
	}
	npts := 3
	areas := make([]float32, 6)

	t.Run("Properly works when the argument 's' is 0.0f", func(t *testing.T) {
		out := RandomPointInConvexPoly(pts, npts, areas, 0.0, 1.0)
		if math.Abs(float64(out[0])) > 1e-4 {
			t.Errorf("expected out[0] ≈ 0, got %v", out[0])
		}
		if math.Abs(float64(out[1])) > 1e-4 {
			t.Errorf("expected out[1] ≈ 0, got %v", out[1])
		}
		if math.Abs(float64(out[2]-1)) > 1e-4 {
			t.Errorf("expected out[2] ≈ 1, got %v", out[2])
		}
	})

	t.Run("Properly works when the argument 's' is 0.5f", func(t *testing.T) {
		out := RandomPointInConvexPoly(pts, npts, areas, 0.5, 1.0)
		if math.Abs(float64(out[0]-0.5)) > 1e-4 {
			t.Errorf("expected out[0] ≈ 0.5, got %v", out[0])
		}
		if math.Abs(float64(out[1])) > 1e-4 {
			t.Errorf("expected out[1] ≈ 0, got %v", out[1])
		}
		if math.Abs(float64(out[2]-0.5)) > 1e-4 {
			t.Errorf("expected out[2] ≈ 0.5, got %v", out[2])
		}
	})

	t.Run("Properly works when the argument 's' is 1.0f", func(t *testing.T) {
		out := RandomPointInConvexPoly(pts, npts, areas, 1.0, 1.0)
		if math.Abs(float64(out[0]-1)) > 1e-4 {
			t.Errorf("expected out[0] ≈ 1, got %v", out[0])
		}
		if math.Abs(float64(out[1])) > 1e-4 {
			t.Errorf("expected out[1] ≈ 0, got %v", out[1])
		}
		if math.Abs(float64(out[2])) > 1e-4 {
			t.Errorf("expected out[2] ≈ 0, got %v", out[2])
		}
	})
}

// Realistic polygon vertices for a 6-vertex grid cell (two triangles forming a square cell).
// Square cell from (0,0)-(10,0)-(10,10)-(0,10) split into two triangles.
var benchPolyVerts = []float32{
	0, 0, 0, // v0
	10, 0, 0, // v1
	10, 0, 10, // v2
	0, 0, 10, // v3
	0, 0, 0, // v4
	10, 0, 10, // v5
}

var benchPolyNverts = 6

// Ray segment that crosses the entire polygon.
var benchSegP0 = [3]float32{-1, 0, 5}
var benchSegP1 = [3]float32{11, 0, 5}

func BenchmarkIntersectSegmentPoly2D(b *testing.B) {
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		IntersectSegmentPoly2D(benchSegP0, benchSegP1, benchPolyVerts, benchPolyNverts)
	}
}

func BenchmarkIntersectSegmentPoly2D_Inline(b *testing.B) {
	b.ReportAllocs()
	var tmin, tmax float32
	var segMin, segMax int
	for i := 0; i < b.N; i++ {
		intersectSegPoly2D(benchSegP0, benchSegP1, benchPolyVerts, benchPolyNverts, &tmin, &tmax, &segMin, &segMax)
	}
}

var benchPt = [3]float32{5, 0, 5}
var benchP = [3]float32{0, 0, 0}
var benchQ = [3]float32{10, 0, 0}

func BenchmarkDistancePtSegSqr2D(b *testing.B) {
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		DistancePtSegSqr2D(benchPt, benchP, benchQ)
	}
}

// Prevents compiler from optimizing away benchmark results
var benchSink float32
