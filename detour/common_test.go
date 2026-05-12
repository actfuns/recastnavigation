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
