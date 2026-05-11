package recast

import (
	"testing"
	"time"

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

// TestClampF32 tests the ClampF32 function
func TestClampF32(t *testing.T) {
	assert.InDelta(t, float32(0.5), ClampF32(0.5, 0, 1), 0.0001)
	assert.InDelta(t, float32(0), ClampF32(-0.5, 0, 1), 0.0001)
	assert.InDelta(t, float32(1), ClampF32(1.5, 0, 1), 0.0001)
}

// TestMinF tests the MinF function
func TestMinF(t *testing.T) {
	assert.InDelta(t, float32(1.5), MinF(1.5, 2.0), 0.0001)
	assert.InDelta(t, float32(1.5), MinF(2.0, 1.5), 0.0001)
}

// TestMaxF tests the MaxF function
func TestMaxF(t *testing.T) {
	assert.InDelta(t, float32(2.0), MaxF(1.5, 2.0), 0.0001)
	assert.InDelta(t, float32(2.0), MaxF(2.0, 1.5), 0.0001)
}

// TestAbsF tests the AbsF function
func TestAbsF(t *testing.T) {
	assert.InDelta(t, float32(3.5), AbsF(-3.5), 0.0001)
	assert.InDelta(t, float32(0), AbsF(0), 0.0001)
	assert.InDelta(t, float32(2.0), AbsF(2.0), 0.0001)
}

// TestVcross tests the Vcross function
func TestVcross(t *testing.T) {
	t.Run("Computes cross product", func(t *testing.T) {
		v1 := [3]float32{3, -3, 1}
		v2 := [3]float32{4, 9, 2}
		result := Vcross(v1, v2)
		assert.InDelta(t, -15.0, result[0], 0.0001)
		assert.InDelta(t, -2.0, result[1], 0.0001)
		assert.InDelta(t, 39.0, result[2], 0.0001)
	})

	t.Run("Cross product with itself is zero", func(t *testing.T) {
		v1 := [3]float32{3, -3, 1}
		result := Vcross(v1, v1)
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
		result := Vmad(v1, v2, 2)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 6.0, result[1], 0.0001)
		assert.InDelta(t, 11.0, result[2], 0.0001)
	})

	t.Run("second vector is scaled, first is not", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{5, 6, 7}
		result := Vmad(v1, v2, 0)
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
		result := Vadd(v1, v2)
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
		result := Vsub(v1, v2)
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
		result := Vmin(v1, v2)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 2.0, result[1], 0.0001)
		assert.InDelta(t, 0.0, result[2], 0.0001)
	})

	t.Run("v1 is min", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{4, 5, 6}
		result := Vmin(v1, v2)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 2.0, result[1], 0.0001)
		assert.InDelta(t, 3.0, result[2], 0.0001)
	})

	t.Run("v2 is min", func(t *testing.T) {
		v1 := [3]float32{4, 5, 6}
		v2 := [3]float32{1, 2, 3}
		result := Vmin(v1, v2)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 2.0, result[1], 0.0001)
		assert.InDelta(t, 3.0, result[2], 0.0001)
	})
}

// TestVmax tests the Vmax function
func TestVmax(t *testing.T) {
	t.Run("selects the max component from the vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		v2 := [3]float32{1, 2, 9}
		result := Vmax(v1, v2)
		assert.InDelta(t, 5.0, result[0], 0.0001)
		assert.InDelta(t, 4.0, result[1], 0.0001)
		assert.InDelta(t, 9.0, result[2], 0.0001)
	})

	t.Run("v2 is max", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{4, 5, 6}
		result := Vmax(v1, v2)
		assert.InDelta(t, 4.0, result[0], 0.0001)
		assert.InDelta(t, 5.0, result[1], 0.0001)
		assert.InDelta(t, 6.0, result[2], 0.0001)
	})

	t.Run("v1 is max", func(t *testing.T) {
		v1 := [3]float32{4, 5, 6}
		v2 := [3]float32{1, 2, 3}
		result := Vmax(v1, v2)
		assert.InDelta(t, 4.0, result[0], 0.0001)
		assert.InDelta(t, 5.0, result[1], 0.0001)
		assert.InDelta(t, 6.0, result[2], 0.0001)
	})
}

// TestVcopy tests the Vcopy function
func TestVcopy(t *testing.T) {
	t.Run("copies a vector into another vector", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		result := Vcopy(v1)
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
		result := Vnormalize(v)
		expected := Sqrt(1.0 / 3.0)
		assert.InDelta(t, expected, result[0], 0.0001)
		assert.InDelta(t, expected, result[1], 0.0001)
		assert.InDelta(t, expected, result[2], 0.0001)
		magnitude := Sqrt(Sqr(result[0]) + Sqr(result[1]) + Sqr(result[2]))
		assert.InDelta(t, 1.0, magnitude, 0.0001)
	})
}

// TestVsafeNormalize tests the VsafeNormalize function
func TestVsafeNormalize(t *testing.T) {
	t.Run("normal non-zero vector", func(t *testing.T) {
		v := [3]float32{3, 0, 0}
		result := VsafeNormalize(v)
		assert.InDelta(t, 1.0, result[0], 0.0001)
		assert.InDelta(t, 0.0, result[1], 0.0001)
		assert.InDelta(t, 0.0, result[2], 0.0001)
	})

	t.Run("zero vector returns unchanged", func(t *testing.T) {
		v := [3]float32{0, 0, 0}
		result := VsafeNormalize(v)
		assert.Equal(t, [3]float32{0, 0, 0}, result)
	})
}

// TestCalcTriNormal tests the CalcTriNormal function
func TestCalcTriNormal(t *testing.T) {
	v0 := [3]float32{0, 0, 0}
	v1 := [3]float32{1, 0, 0}
	v2 := [3]float32{0, 0, 1}
	normal := CalcTriNormal(v0, v1, v2)
	assert.InDelta(t, 0.0, normal[0], 0.0001)
	assert.InDelta(t, -1.0, normal[1], 0.0001)
	assert.InDelta(t, 0.0, normal[2], 0.0001)
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

// TestHeightFieldSpanCount tests the HeightFieldSpanCount function
func TestHeightFieldSpanCount(t *testing.T) {
	ctx := NewContext(false)
	var hf Heightfield
	hf.Width = 2
	hf.Height = 2
	hf.Spans = make([]*Span, 4)

	hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1} // has a walkable span
	hf.Spans[1] = nil                              // no span
	// [2], [3] are nil

	count := HeightFieldSpanCount(ctx, &hf)
	assert.Equal(t, 1, count)
}

// TestDirOffsetX tests the DirOffsetX function
func TestDirOffsetX(t *testing.T) {
	assert.Equal(t, -1, DirOffsetX(0)) // -x
	assert.Equal(t, 0, DirOffsetX(1))  // +z
	assert.Equal(t, 1, DirOffsetX(2))  // +x
	assert.Equal(t, 0, DirOffsetX(3))  // -z
	// wraps with &0x03
	assert.Equal(t, -1, DirOffsetX(4))
	assert.Equal(t, 0, DirOffsetX(5))
}

// TestDirOffsetZ tests the DirOffsetZ function
func TestDirOffsetZ(t *testing.T) {
	assert.Equal(t, 0, DirOffsetZ(0))  // -x
	assert.Equal(t, 1, DirOffsetZ(1))  // +z
	assert.Equal(t, 0, DirOffsetZ(2))  // +x
	assert.Equal(t, -1, DirOffsetZ(3)) // -z
}

// TestDirForOffset tests the DirForOffset function
func TestDirForOffset(t *testing.T) {
	assert.Equal(t, 0, DirForOffset(-1, 0)) // -x -> dir 0
	assert.Equal(t, 1, DirForOffset(0, 1))  // +z -> dir 1
	assert.Equal(t, 2, DirForOffset(1, 0))  // +x -> dir 2
	assert.Equal(t, 3, DirForOffset(0, -1)) // -z -> dir 3
	assert.Equal(t, -1, DirForOffset(0, 0)) // no offset
}

// TestSetConCon tests SetCon and Con round-trip
func TestSetConCon(t *testing.T) {
	var span CompactSpan
	SetCon(&span, 0, 1)
	SetCon(&span, 1, 2)
	SetCon(&span, 2, 3)
	SetCon(&span, 3, 4)

	assert.Equal(t, 1, Con(&span, 0))
	assert.Equal(t, 2, Con(&span, 1))
	assert.Equal(t, 3, Con(&span, 2))
	assert.Equal(t, 4, Con(&span, 3))

	// Overwrite a direction
	SetCon(&span, 1, 63)
	assert.Equal(t, 63, Con(&span, 1))
	// Other directions unchanged
	assert.Equal(t, 1, Con(&span, 0))
}

// --- Context tests ---

func TestNewContext(t *testing.T) {
	ctx := NewContext(false)
	assert.NotNil(t, ctx)
	assert.False(t, ctx.logEnabled)
	assert.False(t, ctx.timerEnabled)

	ctx2 := NewContext(true)
	assert.NotNil(t, ctx2)
	assert.True(t, ctx2.logEnabled)
	assert.True(t, ctx2.timerEnabled)
}

func TestContextLog(t *testing.T) {
	t.Run("log disabled does nothing", func(t *testing.T) {
		ctx := NewContext(false)
		ctx.Log(LogWarning, "should not panic %d", 42)
	})

	t.Run("log with callback receives message", func(t *testing.T) {
		ctx := NewContext(true)
		var capturedCategory LogCategory
		var capturedMsg string
		ctx.SetLogFunc(func(category LogCategory, msg string) {
			capturedCategory = category
			capturedMsg = msg
		})
		ctx.Log(LogWarning, "test %d", 123)
		assert.Equal(t, LogWarning, capturedCategory)
		assert.Equal(t, "test 123", capturedMsg)
	})

	t.Run("enable/disable log toggle", func(t *testing.T) {
		ctx := NewContext(false)
		logged := false
		ctx.SetLogFunc(func(LogCategory, string) { logged = true })
		ctx.Log(LogWarning, "msg")
		assert.False(t, logged)

		ctx.EnableLog(true)
		ctx.Log(LogWarning, "msg")
		assert.True(t, logged)
	})
}

func TestContextTimer(t *testing.T) {
	t.Run("timer disabled returns -1", func(t *testing.T) {
		ctx := NewContext(false)
		d := ctx.GetAccumulatedTime(TimerRasterizeTriangles)
		assert.Equal(t, time.Duration(-1), d)
	})

	t.Run("timer measures elapsed time", func(t *testing.T) {
		ctx := NewContext(true)
		ctx.StartTimer(TimerRasterizeTriangles)
		ctx.StopTimer(TimerRasterizeTriangles)
		d := ctx.GetAccumulatedTime(TimerRasterizeTriangles)
		assert.Greater(t, d, time.Duration(0))
	})

	t.Run("scoped timer", func(t *testing.T) {
		ctx := NewContext(true)
		func() {
			defer ctx.ScopedTimer(TimerRasterizeTriangles)()
		}()
		d := ctx.GetAccumulatedTime(TimerRasterizeTriangles)
		assert.Greater(t, d, time.Duration(0))
	})

	t.Run("reset timers clears accumulated time", func(t *testing.T) {
		ctx := NewContext(true)
		ctx.StartTimer(TimerRasterizeTriangles)
		ctx.StopTimer(TimerRasterizeTriangles)
		ctx.ResetTimers()
		d := ctx.GetAccumulatedTime(TimerRasterizeTriangles)
		assert.Equal(t, time.Duration(-1), d)
	})

	t.Run("unknown timer returns -1", func(t *testing.T) {
		ctx := NewContext(true)
		d := ctx.GetAccumulatedTime(TimerBuildCompactHeightfield)
		assert.Equal(t, time.Duration(-1), d)
	})

	t.Run("enable/disable timer toggle", func(t *testing.T) {
		ctx := NewContext(false)
		assert.Equal(t, time.Duration(-1), ctx.GetAccumulatedTime(TimerRasterizeTriangles))

		ctx.EnableTimer(true)
		ctx.StartTimer(TimerRasterizeTriangles)
		ctx.StopTimer(TimerRasterizeTriangles)
		assert.Greater(t, ctx.GetAccumulatedTime(TimerRasterizeTriangles), time.Duration(0))
	})
}

// --- Nil ctx error path tests ---

func TestBuildCompactHeightfieldNilCtx(t *testing.T) {
	var hf Heightfield
	var chf CompactHeightfield
	_, err := BuildCompactHeightfield(nil, 1, 1, &hf, &chf)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "recast: ctx must not be nil")
}

func TestErodeWalkableAreaNilCtx(t *testing.T) {
	var chf CompactHeightfield
	_, err := ErodeWalkableArea(nil, 1, &chf)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "ctx must not be nil")
}

func TestMedianFilterWalkableAreaNilCtx(t *testing.T) {
	var chf CompactHeightfield
	_, err := MedianFilterWalkableArea(nil, &chf)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "ctx must not be nil")
}

func TestMarkBoxAreaNilCtx(t *testing.T) {
	bmin := [3]float32{0, 0, 0}
	bmax := [3]float32{1, 1, 1}
	err := MarkBoxArea(nil, &bmin, &bmax, 1, &CompactHeightfield{})
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "ctx must not be nil")
}

func TestMarkConvexPolyAreaNilCtx(t *testing.T) {
	err := MarkConvexPolyArea(nil, nil, 0, 0, 0, 1, &CompactHeightfield{})
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "ctx must not be nil")
}

func TestMarkCylinderAreaNilCtx(t *testing.T) {
	pos := [3]float32{0, 0, 0}
	err := MarkCylinderArea(nil, &pos, 1, 1, 1, &CompactHeightfield{})
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "ctx must not be nil")
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
		ok, err := RasterizeTriangles(ctx, verts, 4, tris, areas, 2, &solid, flagMergeThr)
		assert.True(t, ok)
		assert.NoError(t, err)

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
		ok, err := RasterizeTrianglesUShort(ctx, verts, 4, utris, areas, 2, &solid, flagMergeThr)
		assert.True(t, ok)
		assert.NoError(t, err)

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

		ok, err := RasterizeTrianglesVerts(ctx, vertsList, areas, 2, &solid, flagMergeThr)
		assert.True(t, ok)
		assert.NoError(t, err)

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
