package detour

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

// TestRandomPointInConvexPoly tests the RandomPointInConvexPoly function
func TestRandomPointInConvexPoly(t *testing.T) {
	t.Run("Properly works when the argument 's' is 1.0f", func(t *testing.T) {
		pts := []float32{
			0, 0, 0,
			0, 0, 1,
			1, 0, 0,
		}
		npts := 3
		areas := make([]float32, 6)
		out := make([]float32, 3)

		RandomPointInConvexPoly(pts, npts, areas, 0.0, 1.0, out)
		assert.InDelta(t, 0.0, out[0], 0.0001)
		assert.InDelta(t, 0.0, out[1], 0.0001)
		assert.InDelta(t, 1.0, out[2], 0.0001)

		RandomPointInConvexPoly(pts, npts, areas, 0.5, 1.0, out)
		assert.InDelta(t, 1.0/2.0, out[0], 0.0001)
		assert.InDelta(t, 0.0, out[1], 0.0001)
		assert.InDelta(t, 1.0/2.0, out[2], 0.0001)

		RandomPointInConvexPoly(pts, npts, areas, 1.0, 1.0, out)
		assert.InDelta(t, 1.0, out[0], 0.0001)
		assert.InDelta(t, 0.0, out[1], 0.0001)
		assert.InDelta(t, 0.0, out[2], 0.0001)
	})
}
