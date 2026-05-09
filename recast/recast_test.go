package recast

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

// TestSwap tests the Swap function
func TestSwap(t *testing.T) {
	t.Run("Swap two values", func(t *testing.T) {
		one := 1
		two := 2
		Swap(&one, &two)
		assert.Equal(t, 2, one)
		assert.Equal(t, 1, two)
	})
}

// TestMin tests the Min function
func TestMin(t *testing.T) {
	t.Run("Min returns the lowest value", func(t *testing.T) {
		assert.Equal(t, 1, Min(1, 2))
		assert.Equal(t, 1, Min(2, 1))
	})

	t.Run("Min with equal args", func(t *testing.T) {
		assert.Equal(t, 1, Min(1, 1))
	})
}

// TestMax tests the Max function
func TestMax(t *testing.T) {
	t.Run("Max returns the greatest value", func(t *testing.T) {
		assert.Equal(t, 2, Max(1, 2))
		assert.Equal(t, 2, Max(2, 1))
	})

	t.Run("Max with equal args", func(t *testing.T) {
		assert.Equal(t, 1, Max(1, 1))
	})
}

// TestAbs tests the Abs function
func TestAbs(t *testing.T) {
	t.Run("Abs returns the absolute value", func(t *testing.T) {
		assert.Equal(t, 1, Abs(-1))
		assert.Equal(t, 1, Abs(1))
		assert.Equal(t, 0, Abs(0))
	})
}

// TestSqr tests the Sqr function
func TestSqr(t *testing.T) {
	t.Run("Sqr squares a number", func(t *testing.T) {
		assert.InDelta(t, float32(4), Sqr(2), 0.0001)
		assert.InDelta(t, float32(16), Sqr(-4), 0.0001)
		assert.InDelta(t, float32(0), Sqr(0), 0.0001)
	})
}

// TestClamp tests the Clamp function
func TestClamp(t *testing.T) {
	t.Run("Higher than range", func(t *testing.T) {
		assert.Equal(t, 1, Clamp(2, 0, 1))
	})

	t.Run("Within range", func(t *testing.T) {
		assert.Equal(t, 1, Clamp(1, 0, 2))
	})

	t.Run("Lower than range", func(t *testing.T) {
		assert.Equal(t, 1, Clamp(0, 1, 2))
	})
}

// TestSqrt tests the Sqrt function
func TestSqrt(t *testing.T) {
	t.Run("Sqrt gets the sqrt of a number", func(t *testing.T) {
		assert.InDelta(t, 2.0, Sqrt(4), 0.0001)
		assert.InDelta(t, 9.0, Sqrt(81), 0.0001)
	})
}

// TestVcross tests the Vcross function
func TestVcross(t *testing.T) {
	t.Run("Computes cross product", func(t *testing.T) {
		v1 := [3]float32{3, -3, 1}
		v2 := [3]float32{4, 9, 2}
		var result [3]float32
		Vcross(&result, &v1, &v2)
		assert.InDelta(t, -15.0, result[0], 0.0001)
		assert.InDelta(t, -2.0, result[1], 0.0001)
		assert.InDelta(t, 39.0, result[2], 0.0001)
	})

	t.Run("Cross product with itself is zero", func(t *testing.T) {
		v1 := [3]float32{3, -3, 1}
		var result [3]float32
		Vcross(&result, &v1, &v1)
		assert.InDelta(t, 0.0, result[0], 0.0001)
		assert.InDelta(t, 0.0, result[1], 0.0001)
		assert.InDelta(t, 0.0, result[2], 0.0001)
	})
}

// TestVdot tests the Vdot function
func TestVdot(t *testing.T) {
	t.Run("Dot normalized vector with itself", func(t *testing.T) {
		v1 := [3]float32{1, 0, 0}
		result := Vdot(&v1, &v1)
		assert.InDelta(t, 1.0, result, 0.0001)
	})

	t.Run("Dot zero vector with anything is zero", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{0, 0, 0}
		result := Vdot(&v1, &v2)
		assert.InDelta(t, 0.0, result, 0.0001)
	})
}

// TestVmad tests the Vmad function
func TestVmad(t *testing.T) {
	t.Run("scaled add two vectors", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{0, 2, 4}
		var result [3]float32
		Vmad(&result, &v1, &v2, 2)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 6.0, result[1], 0.0001)
		assert.InDelta(t, 11.0, result[2], 0.0001)
	})

	t.Run("second vector is scaled, first is not", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{5, 6, 7}
		var result [3]float32
		Vmad(&result, &v1, &v2, 0)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 2.0, result[1], 0.0001)
		assert.InDelta(t, 3.0, result[2], 0.0001)
	})
}

// TestVadd tests the Vadd function
func TestVadd(t *testing.T) {
	t.Run("add two vectors", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{5, 6, 7}
		var result [3]float32
		Vadd(&result, &v1, &v2)
		assert.InDelta(t, 6.0, result[0], 0.0001)
		assert.InDelta(t, 8.0, result[1], 0.0001)
		assert.InDelta(t, 10.0, result[2], 0.0001)
	})
}

// TestVsub tests the Vsub function
func TestVsub(t *testing.T) {
	t.Run("subtract two vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 3}
		v2 := [3]float32{1, 2, 3}
		var result [3]float32
		Vsub(&result, &v1, &v2)
		assert.InDelta(t, 4.0, result[0], 0.0001)
		assert.InDelta(t, 2.0, result[1], 0.0001)
		assert.InDelta(t, 0.0, result[2], 0.0001)
	})
}

// TestVmin tests the Vmin function
func TestVmin(t *testing.T) {
	t.Run("selects the min component from the vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		v2 := [3]float32{1, 2, 9}
		Vmin(&v1, &v2)
		assert.InDelta(t, 1.0, v1[0], 0.0001)
		assert.InDelta(t, 2.0, v1[1], 0.0001)
		assert.InDelta(t, 0.0, v1[2], 0.0001)
	})

	t.Run("v1 is min", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{4, 5, 6}
		Vmin(&v1, &v2)
		assert.InDelta(t, 1.0, v1[0], 0.0001)
		assert.InDelta(t, 2.0, v1[1], 0.0001)
		assert.InDelta(t, 3.0, v1[2], 0.0001)
	})

	t.Run("v2 is min", func(t *testing.T) {
		v1 := [3]float32{4, 5, 6}
		v2 := [3]float32{1, 2, 3}
		Vmin(&v1, &v2)
		assert.InDelta(t, 1.0, v1[0], 0.0001)
		assert.InDelta(t, 2.0, v1[1], 0.0001)
		assert.InDelta(t, 3.0, v1[2], 0.0001)
	})
}

// TestVmax tests the Vmax function
func TestVmax(t *testing.T) {
	t.Run("selects the max component from the vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		v2 := [3]float32{1, 2, 9}
		Vmax(&v1, &v2)
		assert.InDelta(t, 5.0, v1[0], 0.0001)
		assert.InDelta(t, 4.0, v1[1], 0.0001)
		assert.InDelta(t, 9.0, v1[2], 0.0001)
	})

	t.Run("v2 is max", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{4, 5, 6}
		Vmax(&v1, &v2)
		assert.InDelta(t, 4.0, v1[0], 0.0001)
		assert.InDelta(t, 5.0, v1[1], 0.0001)
		assert.InDelta(t, 6.0, v1[2], 0.0001)
	})

	t.Run("v1 is max", func(t *testing.T) {
		v1 := [3]float32{4, 5, 6}
		v2 := [3]float32{1, 2, 3}
		Vmax(&v1, &v2)
		assert.InDelta(t, 4.0, v1[0], 0.0001)
		assert.InDelta(t, 5.0, v1[1], 0.0001)
		assert.InDelta(t, 6.0, v1[2], 0.0001)
	})
}

// TestVcopy tests the Vcopy function
func TestVcopy(t *testing.T) {
	t.Run("copies a vector into another vector", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		result := [3]float32{1, 2, 9}
		Vcopy(&result, &v1)
		assert.InDelta(t, 5.0, result[0], 0.0001)
		assert.InDelta(t, 4.0, result[1], 0.0001)
		assert.InDelta(t, 0.0, result[2], 0.0001)
		assert.InDelta(t, 5.0, v1[0], 0.0001)
		assert.InDelta(t, 4.0, v1[1], 0.0001)
		assert.InDelta(t, 0.0, v1[2], 0.0001)
	})
}

// TestVdist tests the Vdist function
func TestVdist(t *testing.T) {
	t.Run("distance between two vectors", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{1, 3, 1}
		result := Vdist(&v1, &v2)
		assert.InDelta(t, 3.4641, result, 0.0001)
	})

	t.Run("Distance from zero is magnitude", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{0, 0, 0}
		distance := Vdist(&v1, &v2)
		magnitude := Sqrt(Sqr(v1[0]) + Sqr(v1[1]) + Sqr(v1[2]))
		assert.InDelta(t, magnitude, distance, 0.0001)
	})
}

// TestVdistSqr tests the VdistSqr function
func TestVdistSqr(t *testing.T) {
	t.Run("squared distance between two vectors", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{1, 3, 1}
		result := VdistSqr(&v1, &v2)
		assert.InDelta(t, 12.0, result, 0.0001)
	})

	t.Run("squared distance from zero is squared magnitude", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{0, 0, 0}
		distance := VdistSqr(&v1, &v2)
		magnitude := Sqr(v1[0]) + Sqr(v1[1]) + Sqr(v1[2])
		assert.InDelta(t, magnitude, distance, 0.0001)
	})
}

// TestVnormalize tests the Vnormalize function
func TestVnormalize(t *testing.T) {
	t.Run("normalizing reduces magnitude to 1", func(t *testing.T) {
		v := [3]float32{3, 3, 3}
		Vnormalize(&v)
		expected := Sqrt(1.0 / 3.0)
		assert.InDelta(t, expected, v[0], 0.0001)
		assert.InDelta(t, expected, v[1], 0.0001)
		assert.InDelta(t, expected, v[2], 0.0001)
		magnitude := Sqrt(Sqr(v[0]) + Sqr(v[1]) + Sqr(v[2]))
		assert.InDelta(t, 1.0, magnitude, 0.0001)
	})
}

// TestCalcBounds tests the CalcBounds function
func TestCalcBounds(t *testing.T) {
	t.Run("bounds of one vector", func(t *testing.T) {
		verts := []float32{1, 2, 3}
		bmin, bmax := CalcBounds(verts, 1)

		assert.InDelta(t, verts[0], bmin[0], 0.0001)
		assert.InDelta(t, verts[1], bmin[1], 0.0001)
		assert.InDelta(t, verts[2], bmin[2], 0.0001)

		assert.InDelta(t, verts[0], bmax[0], 0.0001)
		assert.InDelta(t, verts[1], bmax[1], 0.0001)
		assert.InDelta(t, verts[2], bmax[2], 0.0001)
	})

	t.Run("bounds of more than one vector", func(t *testing.T) {
		verts := []float32{
			1, 2, 3,
			0, 2, 5,
		}
		bmin, bmax := CalcBounds(verts, 2)

		assert.InDelta(t, 0.0, bmin[0], 0.0001)
		assert.InDelta(t, 2.0, bmin[1], 0.0001)
		assert.InDelta(t, 3.0, bmin[2], 0.0001)

		assert.InDelta(t, 1.0, bmax[0], 0.0001)
		assert.InDelta(t, 2.0, bmax[1], 0.0001)
		assert.InDelta(t, 5.0, bmax[2], 0.0001)
	})
}

// TestCalcGridSize tests the CalcGridSize function
func TestCalcGridSize(t *testing.T) {
	t.Run("computes the size of an x & z axis grid", func(t *testing.T) {
		verts := []float32{
			1, 2, 3,
			0, 2, 6,
		}
		bmin, bmax := CalcBounds(verts, 2)

		cellSize := float32(1.5)

		width, height := CalcGridSize(&bmin, &bmax, cellSize)

		assert.Equal(t, 1, width)
		assert.Equal(t, 2, height)
	})
}

// TestCreateHeightfield tests the CreateHeightfield function
func TestCreateHeightfield(t *testing.T) {
	t.Run("create a heightfield", func(t *testing.T) {
		ctx := NewContext(false)
		verts := []float32{
			1, 2, 3,
			0, 2, 6,
		}
		bmin, bmax := CalcBounds(verts, 2)

		cellSize := float32(1.5)
		cellHeight := float32(2)

		width, height := CalcGridSize(&bmin, &bmax, cellSize)

		var heightfield Heightfield
		result := CreateHeightfield(ctx, &heightfield, width, height, &bmin, &bmax, cellSize, cellHeight)

		assert.True(t, result)

		assert.Equal(t, width, heightfield.Width)
		assert.Equal(t, height, heightfield.Height)

		assert.InDelta(t, bmin[0], heightfield.Bmin[0], 0.0001)
		assert.InDelta(t, bmin[1], heightfield.Bmin[1], 0.0001)
		assert.InDelta(t, bmin[2], heightfield.Bmin[2], 0.0001)

		assert.InDelta(t, bmax[0], heightfield.Bmax[0], 0.0001)
		assert.InDelta(t, bmax[1], heightfield.Bmax[1], 0.0001)
		assert.InDelta(t, bmax[2], heightfield.Bmax[2], 0.0001)

		assert.InDelta(t, cellSize, heightfield.Cs, 0.0001)
		assert.InDelta(t, cellHeight, heightfield.Ch, 0.0001)

		assert.NotNil(t, heightfield.Spans)
		assert.Nil(t, heightfield.Pools)
		assert.Nil(t, heightfield.FreeList)
	})
}

// TestMarkWalkableTriangles tests the MarkWalkableTriangles function
func TestMarkWalkableTriangles(t *testing.T) {
	ctx := NewContext(false)
	walkableSlopeAngle := float32(45)
	verts := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
	}
	nv := 3
	walkableTri := []int{0, 1, 2}
	unwalkableTri := []int{0, 2, 1}
	nt := 1

	t.Run("One walkable triangle", func(t *testing.T) {
		areas := []uint8{NullArea}
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkableTri, nt, areas)
		assert.Equal(t, uint8(WalkableArea), areas[0])
	})

	t.Run("One non-walkable triangle", func(t *testing.T) {
		areas := []uint8{NullArea}
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkableTri, nt, areas)
		assert.Equal(t, uint8(NullArea), areas[0])
	})

	t.Run("Non-walkable triangle area id's are not modified", func(t *testing.T) {
		areas := []uint8{42}
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkableTri, nt, areas)
		assert.Equal(t, uint8(42), areas[0])
	})

	t.Run("Slopes equal to the max slope are considered unwalkable", func(t *testing.T) {
		walkableSlopeAngle = 0
		areas := []uint8{NullArea}
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkableTri, nt, areas)
		assert.Equal(t, uint8(NullArea), areas[0])
	})
}

// TestClearUnwalkableTriangles tests the ClearUnwalkableTriangles function
func TestClearUnwalkableTriangles(t *testing.T) {
	ctx := NewContext(false)
	walkableSlopeAngle := float32(45)
	verts := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
	}
	nv := 3
	walkableTri := []int{0, 1, 2}
	unwalkableTri := []int{0, 2, 1}
	nt := 1

	t.Run("Sets area ID of unwalkable triangle to NullArea", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkableTri, nt, areas)
		assert.Equal(t, uint8(NullArea), areas[0])
	})

	t.Run("Does not modify walkable triangle area ID's", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkableTri, nt, areas)
		assert.Equal(t, uint8(42), areas[0])
	})

	t.Run("Slopes equal to the max slope are considered unwalkable", func(t *testing.T) {
		walkableSlopeAngle = 0
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkableTri, nt, areas)
		assert.Equal(t, uint8(NullArea), areas[0])
	})
}

// Helper function to check span properties
func checkSpan(t *testing.T, span *Span, expectedSmin, expectedSmax uint32, expectedArea uint32, hasNext bool) {
	assert.NotNil(t, span)
	assert.Equal(t, expectedSmin, span.Smin)
	assert.Equal(t, expectedSmax, span.Smax)
	assert.Equal(t, expectedArea, span.Area)
	if hasNext {
		assert.NotNil(t, span.Next)
	} else {
		assert.Nil(t, span.Next)
	}
}

// TestRasterizeTriangles tests the RasterizeTriangles function
func TestRasterizeTriangles(t *testing.T) {
	ctx := NewContext(false)
	verts := []float32{
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		0, 0, 1,
	}
	tris := []int{
		0, 1, 2,
		0, 3, 1,
	}
	areas := []uint8{1, 2}
	bmin, bmax := CalcBounds(verts, 4)

	cellSize := float32(0.5)
	cellHeight := float32(0.5)

	width, height := CalcGridSize(&bmin, &bmax, cellSize)

	var solid Heightfield
	assert.True(t, CreateHeightfield(ctx, &solid, width, height, &bmin, &bmax, cellSize, cellHeight))

	flagMergeThr := 1

	t.Run("Rasterize some triangles", func(t *testing.T) {
		assert.True(t, RasterizeTriangles(ctx, verts, 4, tris, areas, 2, &solid, flagMergeThr))

		// Check spans at specific positions
		checkSpan(t, solid.Spans[0+0*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[0+1*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[0+2*width], 0, 1, 2, false)
		checkSpan(t, solid.Spans[0+3*width], 0, 1, 2, false)
		assert.Nil(t, solid.Spans[1+0*width])
		checkSpan(t, solid.Spans[1+1*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[1+2*width], 0, 1, 2, false)
		assert.Nil(t, solid.Spans[1+3*width])
	})

	t.Run("Unsigned short overload", func(t *testing.T) {
		// Reset heightfield
		solid.Spans = make([]*Span, solid.Width*solid.Height)

		utris := []uint16{
			0, 1, 2,
			0, 3, 1,
		}
		assert.True(t, RasterizeTrianglesUShort(ctx, verts, 4, utris, areas, 2, &solid, flagMergeThr))

		// Check spans at specific positions
		checkSpan(t, solid.Spans[0+0*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[0+1*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[0+2*width], 0, 1, 2, false)
		checkSpan(t, solid.Spans[0+3*width], 0, 1, 2, false)
		assert.Nil(t, solid.Spans[1+0*width])
		checkSpan(t, solid.Spans[1+1*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[1+2*width], 0, 1, 2, false)
		assert.Nil(t, solid.Spans[1+3*width])
	})

	t.Run("Triangle list overload", func(t *testing.T) {
		// Reset heightfield
		solid.Spans = make([]*Span, solid.Width*solid.Height)

		vertsList := []float32{
			0, 0, 0,
			1, 0, 0,
			0, 0, -1,
			0, 0, 0,
			0, 0, 1,
			1, 0, 0,
		}

		assert.True(t, RasterizeTrianglesVerts(ctx, vertsList, areas, 2, &solid, flagMergeThr))

		// Check spans at specific positions
		checkSpan(t, solid.Spans[0+0*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[0+1*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[0+2*width], 0, 1, 2, false)
		checkSpan(t, solid.Spans[0+3*width], 0, 1, 2, false)
		assert.Nil(t, solid.Spans[1+0*width])
		checkSpan(t, solid.Spans[1+1*width], 0, 1, 1, false)
		checkSpan(t, solid.Spans[1+2*width], 0, 1, 2, false)
		assert.Nil(t, solid.Spans[1+3*width])
	})
}
