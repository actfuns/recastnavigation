package recast

import (
	"context"
	"math"
	"testing"
)

func TestSwap(t *testing.T) {
	t.Run("Swap two values", func(t *testing.T) {
		one := 1
		two := 2
		Swap(&one, &two)
		if one != 2 {
			t.Errorf("one = %d, want 2", one)
		}
		if two != 1 {
			t.Errorf("two = %d, want 1", two)
		}
	})
}

func TestMin(t *testing.T) {
	t.Run("Min returns the lowest value.", func(t *testing.T) {
		if Min(1, 2) != 1 {
			t.Errorf("Min(1, 2) = %d, want 1", Min(1, 2))
		}
		if Min(2, 1) != 1 {
			t.Errorf("Min(2, 1) = %d, want 1", Min(2, 1))
		}
	})

	t.Run("Min with equal args", func(t *testing.T) {
		if Min(1, 1) != 1 {
			t.Errorf("Min(1, 1) = %d, want 1", Min(1, 1))
		}
	})
}

func TestMax(t *testing.T) {
	t.Run("Max returns the greatest value.", func(t *testing.T) {
		if Max(1, 2) != 2 {
			t.Errorf("Max(1, 2) = %d, want 2", Max(1, 2))
		}
		if Max(2, 1) != 2 {
			t.Errorf("Max(2, 1) = %d, want 2", Max(2, 1))
		}
	})

	t.Run("Max with equal args", func(t *testing.T) {
		if Max(1, 1) != 1 {
			t.Errorf("Max(1, 1) = %d, want 1", Max(1, 1))
		}
	})
}

func TestAbs(t *testing.T) {
	t.Run("Abs returns the absolute value.", func(t *testing.T) {
		if Abs(-1) != 1 {
			t.Errorf("Abs(-1) = %d, want 1", Abs(-1))
		}
		if Abs(1) != 1 {
			t.Errorf("Abs(1) = %d, want 1", Abs(1))
		}
		if Abs(0) != 0 {
			t.Errorf("Abs(0) = %d, want 0", Abs(0))
		}
	})
}

func TestSqr(t *testing.T) {
	t.Run("Sqr squares a number", func(t *testing.T) {
		if v, want := Sqr(float32(2)), float32(4); math.Abs(float64(v-want)) > 1e-4 {
			t.Errorf("Sqr(2) = %v, want %v", v, want)
		}
		if v, want := Sqr(float32(-4)), float32(16); math.Abs(float64(v-want)) > 1e-4 {
			t.Errorf("Sqr(-4) = %v, want %v", v, want)
		}
		if v, want := Sqr(float32(0)), float32(0); math.Abs(float64(v-want)) > 1e-4 {
			t.Errorf("Sqr(0) = %v, want %v", v, want)
		}
	})
}

func TestClamp(t *testing.T) {
	t.Run("Higher than range", func(t *testing.T) {
		if Clamp(2, 0, 1) != 1 {
			t.Errorf("Clamp(2, 0, 1) = %d, want 1", Clamp(2, 0, 1))
		}
	})

	t.Run("Within range", func(t *testing.T) {
		if Clamp(1, 0, 2) != 1 {
			t.Errorf("Clamp(1, 0, 2) = %d, want 1", Clamp(1, 0, 2))
		}
	})

	t.Run("Lower than range", func(t *testing.T) {
		if Clamp(0, 1, 2) != 1 {
			t.Errorf("Clamp(0, 1, 2) = %d, want 1", Clamp(0, 1, 2))
		}
	})
}

func TestSqrt(t *testing.T) {
	t.Run("Sqrt gets the sqrt of a number", func(t *testing.T) {
		if v, want := Sqrt(4), float32(2); math.Abs(float64(v-want)) > 1e-4 {
			t.Errorf("Sqrt(4) = %v, want %v", v, want)
		}
		if v, want := Sqrt(81), float32(9); math.Abs(float64(v-want)) > 1e-4 {
			t.Errorf("Sqrt(81) = %v, want %v", v, want)
		}
	})
}

func TestVcross(t *testing.T) {
	t.Run("Computes cross product", func(t *testing.T) {
		v1 := [3]float32{3, -3, 1}
		v2 := [3]float32{4, 9, 2}
		result := Vcross(v1, v2)
		if math.Abs(float64(result[0]-(-15))) > 1e-4 {
			t.Errorf("result[0] = %v, want %v", result[0], -15)
		}
		if math.Abs(float64(result[1]-(-2))) > 1e-4 {
			t.Errorf("result[1] = %v, want %v", result[1], -2)
		}
		if math.Abs(float64(result[2]-39)) > 1e-4 {
			t.Errorf("result[2] = %v, want %v", result[2], 39)
		}
	})

	t.Run("Cross product with itself is zero", func(t *testing.T) {
		v1 := [3]float32{3, -3, 1}
		result := Vcross(v1, v1)
		if math.Abs(float64(result[0])) > 1e-4 {
			t.Errorf("result[0] = %v, want 0", result[0])
		}
		if math.Abs(float64(result[1])) > 1e-4 {
			t.Errorf("result[1] = %v, want 0", result[1])
		}
		if math.Abs(float64(result[2])) > 1e-4 {
			t.Errorf("result[2] = %v, want 0", result[2])
		}
	})
}

func TestVdot(t *testing.T) {
	t.Run("Dot normalized vector with itself", func(t *testing.T) {
		v1 := [3]float32{1, 0, 0}
		result := Vdot(v1, v1)
		if math.Abs(float64(result-1)) > 1e-4 {
			t.Errorf("Vdot(v1, v1) = %v, want 1", result)
		}
	})

	t.Run("Dot zero vector with anything is zero", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{0, 0, 0}
		result := Vdot(v1, v2)
		if math.Abs(float64(result)) > 1e-4 {
			t.Errorf("Vdot(v1, v2) = %v, want 0", result)
		}
	})
}

func TestVmad(t *testing.T) {
	t.Run("scaled add two vectors", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{0, 2, 4}
		result := Vmad(v1, v2, float32(2))
		if math.Abs(float64(result[0]-1)) > 1e-4 {
			t.Errorf("result[0] = %v, want 1", result[0])
		}
		if math.Abs(float64(result[1]-6)) > 1e-4 {
			t.Errorf("result[1] = %v, want 6", result[1])
		}
		if math.Abs(float64(result[2]-11)) > 1e-4 {
			t.Errorf("result[2] = %v, want 11", result[2])
		}
	})

	t.Run("second vector is scaled, first is not", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{5, 6, 7}
		result := Vmad(v1, v2, float32(0))
		if math.Abs(float64(result[0]-1)) > 1e-4 {
			t.Errorf("result[0] = %v, want 1", result[0])
		}
		if math.Abs(float64(result[1]-2)) > 1e-4 {
			t.Errorf("result[1] = %v, want 2", result[1])
		}
		if math.Abs(float64(result[2]-3)) > 1e-4 {
			t.Errorf("result[2] = %v, want 3", result[2])
		}
	})
}

func TestVadd(t *testing.T) {
	t.Run("add two vectors", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{5, 6, 7}
		result := Vadd(v1, v2)
		if math.Abs(float64(result[0]-6)) > 1e-4 {
			t.Errorf("result[0] = %v, want 6", result[0])
		}
		if math.Abs(float64(result[1]-8)) > 1e-4 {
			t.Errorf("result[1] = %v, want 8", result[1])
		}
		if math.Abs(float64(result[2]-10)) > 1e-4 {
			t.Errorf("result[2] = %v, want 10", result[2])
		}
	})
}

func TestVsub(t *testing.T) {
	t.Run("subtract two vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 3}
		v2 := [3]float32{1, 2, 3}
		result := Vsub(v1, v2)
		if math.Abs(float64(result[0]-4)) > 1e-4 {
			t.Errorf("result[0] = %v, want 4", result[0])
		}
		if math.Abs(float64(result[1]-2)) > 1e-4 {
			t.Errorf("result[1] = %v, want 2", result[1])
		}
		if math.Abs(float64(result[2]-0)) > 1e-4 {
			t.Errorf("result[2] = %v, want 0", result[2])
		}
	})
}

func TestVmin(t *testing.T) {
	t.Run("selects the min component from the vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		v2 := [3]float32{1, 2, 9}
		v1 = Vmin(v1, v2)
		if math.Abs(float64(v1[0]-1)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 1", v1[0])
		}
		if math.Abs(float64(v1[1]-2)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 2", v1[1])
		}
		if math.Abs(float64(v1[2]-0)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 0", v1[2])
		}
	})

	t.Run("v1 is min", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{4, 5, 6}
		v1 = Vmin(v1, v2)
		if math.Abs(float64(v1[0]-1)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 1", v1[0])
		}
		if math.Abs(float64(v1[1]-2)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 2", v1[1])
		}
		if math.Abs(float64(v1[2]-3)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 3", v1[2])
		}
	})

	t.Run("v2 is min", func(t *testing.T) {
		v1 := [3]float32{4, 5, 6}
		v2 := [3]float32{1, 2, 3}
		v1 = Vmin(v1, v2)
		if math.Abs(float64(v1[0]-1)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 1", v1[0])
		}
		if math.Abs(float64(v1[1]-2)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 2", v1[1])
		}
		if math.Abs(float64(v1[2]-3)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 3", v1[2])
		}
	})
}

func TestVmax(t *testing.T) {
	t.Run("selects the max component from the vectors", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		v2 := [3]float32{1, 2, 9}
		v1 = Vmax(v1, v2)
		if math.Abs(float64(v1[0]-5)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 5", v1[0])
		}
		if math.Abs(float64(v1[1]-4)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 4", v1[1])
		}
		if math.Abs(float64(v1[2]-9)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 9", v1[2])
		}
	})

	t.Run("v2 is max", func(t *testing.T) {
		v1 := [3]float32{1, 2, 3}
		v2 := [3]float32{4, 5, 6}
		v1 = Vmax(v1, v2)
		if math.Abs(float64(v1[0]-4)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 4", v1[0])
		}
		if math.Abs(float64(v1[1]-5)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 5", v1[1])
		}
		if math.Abs(float64(v1[2]-6)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 6", v1[2])
		}
	})

	t.Run("v1 is max", func(t *testing.T) {
		v1 := [3]float32{4, 5, 6}
		v2 := [3]float32{1, 2, 3}
		v1 = Vmax(v1, v2)
		if math.Abs(float64(v1[0]-4)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 4", v1[0])
		}
		if math.Abs(float64(v1[1]-5)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 5", v1[1])
		}
		if math.Abs(float64(v1[2]-6)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 6", v1[2])
		}
	})
}

func TestVcopy(t *testing.T) {
	t.Run("copies a vector into another vector", func(t *testing.T) {
		v1 := [3]float32{5, 4, 0}
		var result [3]float32
		result = Vcopy(v1[:])
		if math.Abs(float64(result[0]-5)) > 1e-4 {
			t.Errorf("result[0] = %v, want 5", result[0])
		}
		if math.Abs(float64(result[1]-4)) > 1e-4 {
			t.Errorf("result[1] = %v, want 4", result[1])
		}
		if math.Abs(float64(result[2]-0)) > 1e-4 {
			t.Errorf("result[2] = %v, want 0", result[2])
		}
		if math.Abs(float64(v1[0]-5)) > 1e-4 {
			t.Errorf("v1[0] = %v, want 5", v1[0])
		}
		if math.Abs(float64(v1[1]-4)) > 1e-4 {
			t.Errorf("v1[1] = %v, want 4", v1[1])
		}
		if math.Abs(float64(v1[2]-0)) > 1e-4 {
			t.Errorf("v1[2] = %v, want 0", v1[2])
		}
	})
}

func TestVdist(t *testing.T) {
	t.Run("distance between two vectors", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{1, 3, 1}
		result := Vdist(v1, v2)
		if math.Abs(float64(result-3.4641)) > 1e-4 {
			t.Errorf("Vdist = %v, want %v", result, 3.4641)
		}
	})

	t.Run("Distance from zero is magnitude", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{0, 0, 0}
		distance := Vdist(v1, v2)
		magnitude := Sqrt(Sqr(v1[0]) + Sqr(v1[1]) + Sqr(v1[2]))
		if math.Abs(float64(distance-magnitude)) > 1e-4 {
			t.Errorf("distance = %v, magnitude = %v", distance, magnitude)
		}
	})
}

func TestVdistSqr(t *testing.T) {
	t.Run("squared distance between two vectors", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{1, 3, 1}
		result := VdistSqr(v1, v2)
		if math.Abs(float64(result-12)) > 1e-4 {
			t.Errorf("VdistSqr = %v, want 12", result)
		}
	})

	t.Run("squared distance from zero is squared magnitude", func(t *testing.T) {
		v1 := [3]float32{3, 1, 3}
		v2 := [3]float32{0, 0, 0}
		distance := VdistSqr(v1, v2)
		magnitude := Sqr(v1[0]) + Sqr(v1[1]) + Sqr(v1[2])
		if math.Abs(float64(distance-magnitude)) > 1e-4 {
			t.Errorf("VdistSqr = %v, magnitude = %v", distance, magnitude)
		}
	})
}

func TestVnormalize(t *testing.T) {
	t.Run("normalizing reduces magnitude to 1", func(t *testing.T) {
		v := [3]float32{3, 3, 3}
		v = Vnormalize(v)
		expected := Sqrt(float32(1.0) / float32(3.0))
		if math.Abs(float64(v[0])-float64(expected)) > 1e-4 {
			t.Errorf("v[0] = %v, want %v", v[0], expected)
		}
		if math.Abs(float64(v[1])-float64(expected)) > 1e-4 {
			t.Errorf("v[1] = %v, want %v", v[1], expected)
		}
		if math.Abs(float64(v[2])-float64(expected)) > 1e-4 {
			t.Errorf("v[2] = %v, want %v", v[2], expected)
		}
		magnitude := Sqrt(Sqr(v[0]) + Sqr(v[1]) + Sqr(v[2]))
		if math.Abs(float64(magnitude-1)) > 1e-4 {
			t.Errorf("magnitude = %v, want 1", magnitude)
		}
	})
}

func TestCalcBounds(t *testing.T) {
	t.Run("bounds of one vector", func(t *testing.T) {
		verts := []float32{1, 2, 3}
		bmin, bmax := CalcBounds(verts, 1)
		if math.Abs(float64(bmin[0]-verts[0])) > 1e-4 {
			t.Errorf("bmin[0] = %v, want %v", bmin[0], verts[0])
		}
		if math.Abs(float64(bmin[1]-verts[1])) > 1e-4 {
			t.Errorf("bmin[1] = %v, want %v", bmin[1], verts[1])
		}
		if math.Abs(float64(bmin[2]-verts[2])) > 1e-4 {
			t.Errorf("bmin[2] = %v, want %v", bmin[2], verts[2])
		}
		if math.Abs(float64(bmax[0]-verts[0])) > 1e-4 {
			t.Errorf("bmax[0] = %v, want %v", bmax[0], verts[0])
		}
		if math.Abs(float64(bmax[1]-verts[1])) > 1e-4 {
			t.Errorf("bmax[1] = %v, want %v", bmax[1], verts[1])
		}
		if math.Abs(float64(bmax[2]-verts[2])) > 1e-4 {
			t.Errorf("bmax[2] = %v, want %v", bmax[2], verts[2])
		}
	})

	t.Run("bounds of more than one vector", func(t *testing.T) {
		verts := []float32{
			1, 2, 3,
			0, 2, 5,
		}
		bmin, bmax := CalcBounds(verts, 2)
		if math.Abs(float64(bmin[0]-0)) > 1e-4 {
			t.Errorf("bmin[0] = %v, want 0", bmin[0])
		}
		if math.Abs(float64(bmin[1]-2)) > 1e-4 {
			t.Errorf("bmin[1] = %v, want 2", bmin[1])
		}
		if math.Abs(float64(bmin[2]-3)) > 1e-4 {
			t.Errorf("bmin[2] = %v, want 3", bmin[2])
		}
		if math.Abs(float64(bmax[0]-1)) > 1e-4 {
			t.Errorf("bmax[0] = %v, want 1", bmax[0])
		}
		if math.Abs(float64(bmax[1]-2)) > 1e-4 {
			t.Errorf("bmax[1] = %v, want 2", bmax[1])
		}
		if math.Abs(float64(bmax[2]-5)) > 1e-4 {
			t.Errorf("bmax[2] = %v, want 5", bmax[2])
		}
	})
}

func TestCalcGridSize(t *testing.T) {
	t.Run("computes the size of an x & z axis grid", func(t *testing.T) {
		verts := []float32{
			1, 2, 3,
			0, 2, 6,
		}
		bmin, bmax := CalcBounds(verts, 2)
		cellSize := float32(1.5)
		width, height := CalcGridSize(bmin, bmax, cellSize)
		if width != 1 {
			t.Errorf("width = %d, want 1", width)
		}
		if height != 2 {
			t.Errorf("height = %d, want 2", height)
		}
	})
}

func TestCreateHeightfield(t *testing.T) {
	t.Run("create a heightfield", func(t *testing.T) {
		verts := []float32{
			1, 2, 3,
			0, 2, 6,
		}
		bmin, bmax := CalcBounds(verts, 2)
		cellSize := float32(1.5)
		cellHeight := float32(2)
		width, height := CalcGridSize(bmin, bmax, cellSize)

		hf := CreateHeightfield(context.Background(), width, height, bmin, bmax, cellSize, cellHeight)

		if hf.Width != width {
			t.Errorf("hf.Width = %d, want %d", hf.Width, width)
		}
		if hf.Height != height {
			t.Errorf("hf.Height = %d, want %d", hf.Height, height)
		}
		for i := 0; i < 3; i++ {
			if math.Abs(float64(hf.Bmin[i]-bmin[i])) > 1e-4 {
				t.Errorf("hf.Bmin[%d] = %v, want %v", i, hf.Bmin[i], bmin[i])
			}
			if math.Abs(float64(hf.Bmax[i]-bmax[i])) > 1e-4 {
				t.Errorf("hf.Bmax[%d] = %v, want %v", i, hf.Bmax[i], bmax[i])
			}
		}
		if math.Abs(float64(hf.Cs-cellSize)) > 1e-4 {
			t.Errorf("hf.Cs = %v, want %v", hf.Cs, cellSize)
		}
		if math.Abs(float64(hf.Ch-cellHeight)) > 1e-4 {
			t.Errorf("hf.Ch = %v, want %v", hf.Ch, cellHeight)
		}
		if hf.Spans == nil {
			t.Errorf("hf.Spans is nil, expected non-nil")
		}
		if hf.Pools != nil {
			t.Errorf("hf.Pools = %v, want nil", hf.Pools)
		}
		if hf.FreeList != nil {
			t.Errorf("hf.FreeList = %v, want nil", hf.FreeList)
		}
	})
}

func TestMarkWalkableTriangles(t *testing.T) {
	var ctx context.Context = nil
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
		if areas[0] != WalkableArea {
			t.Errorf("areas[0] = %d, want %d", areas[0], WalkableArea)
		}
	})

	t.Run("One non-walkable triangle", func(t *testing.T) {
		areas := []uint8{NullArea}
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkableTri, nt, areas)
		if areas[0] != NullArea {
			t.Errorf("areas[0] = %d, want %d", areas[0], NullArea)
		}
	})

	t.Run("Non-walkable triangle area id's are not modified", func(t *testing.T) {
		areas := []uint8{42}
		MarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkableTri, nt, areas)
		if areas[0] != 42 {
			t.Errorf("areas[0] = %d, want 42", areas[0])
		}
	})

	t.Run("Slopes equal to the max slope are considered unwalkable.", func(t *testing.T) {
		areas := []uint8{NullArea}
		MarkWalkableTriangles(ctx, float32(0), verts, nv, walkableTri, nt, areas)
		if areas[0] != NullArea {
			t.Errorf("areas[0] = %d, want %d", areas[0], NullArea)
		}
	})
}

func TestClearUnwalkableTriangles(t *testing.T) {
	var ctx context.Context = nil
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

	t.Run("Sets area ID of unwalkable triangle to RC_NULL_AREA", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkableTri, nt, areas)
		if areas[0] != NullArea {
			t.Errorf("areas[0] = %d, want %d", areas[0], NullArea)
		}
	})

	t.Run("Does not modify walkable triangle area ID's", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkableTri, nt, areas)
		if areas[0] != 42 {
			t.Errorf("areas[0] = %d, want 42", areas[0])
		}
	})

	t.Run("Slopes equal to the max slope are considered unwalkable.", func(t *testing.T) {
		areas := []uint8{42}
		ClearUnwalkableTriangles(ctx, float32(0), verts, nv, walkableTri, nt, areas)
		if areas[0] != NullArea {
			t.Errorf("areas[0] = %d, want %d", areas[0], NullArea)
		}
	})
}

func TestRasterizeTriangles(t *testing.T) {
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
	width, height := CalcGridSize(bmin, bmax, cellSize)
	flagMergeThr := 1

	t.Run("Rasterize some triangles", func(t *testing.T) {
		ctx := context.Background()
		solid := CreateHeightfield(ctx, width, height, bmin, bmax, cellSize, cellHeight)

		err := RasterizeTriangles(ctx, verts, 4, tris, areas, 2, solid, flagMergeThr)
		if err != nil {
			t.Fatalf("RasterizeTriangles returned error: %v", err)
		}

		checkSpan := func(idx int, expectNonNil bool) *Span {
			if expectNonNil {
				if solid.Spans[idx] == nil {
					t.Fatalf("solid.Spans[%d] is nil, expected non-nil", idx)
				}
				return solid.Spans[idx]
			} else {
				if solid.Spans[idx] != nil {
					t.Errorf("solid.Spans[%d] = %v, want nil", idx, solid.Spans[idx])
				}
				return nil
			}
		}

		// Check span existence pattern
		checkSpan(0+0*width, true)
		checkSpan(0+1*width, true)
		checkSpan(0+2*width, true)
		checkSpan(0+3*width, true)
		checkSpan(1+0*width, false)
		checkSpan(1+1*width, true)
		checkSpan(1+2*width, true)
		checkSpan(1+3*width, false)

		// Verify specific span values
		span := checkSpan(0+0*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+0*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+0*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[0+0*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+0*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+1*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+1*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+1*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[0+1*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+1*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+2*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+2*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+2*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[0+2*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+2*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+3*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+3*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+3*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[0+3*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+3*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(1+1*width, true)
		if span.Smin != 0 {
			t.Errorf("span[1+1*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[1+1*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[1+1*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[1+1*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(1+2*width, true)
		if span.Smin != 0 {
			t.Errorf("span[1+2*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[1+2*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[1+2*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[1+2*width].Next = %v, want nil", span.Next)
		}
	})

	t.Run("Unsigned short overload", func(t *testing.T) {
		ctx := context.Background()
		solid := CreateHeightfield(ctx, width, height, bmin, bmax, cellSize, cellHeight)

		utris := []uint16{0, 1, 2, 0, 3, 1}
		err := RasterizeTrianglesUShort(ctx, verts, 4, utris, areas, 2, solid, flagMergeThr)
		if err != nil {
			t.Fatalf("RasterizeTrianglesUShort returned error: %v", err)
		}

		checkSpan := func(idx int, expectNonNil bool) *Span {
			if expectNonNil {
				if solid.Spans[idx] == nil {
					t.Fatalf("solid.Spans[%d] is nil, expected non-nil", idx)
				}
				return solid.Spans[idx]
			} else {
				if solid.Spans[idx] != nil {
					t.Errorf("solid.Spans[%d] = %v, want nil", idx, solid.Spans[idx])
				}
				return nil
			}
		}

		checkSpan(0+0*width, true)
		checkSpan(0+1*width, true)
		checkSpan(0+2*width, true)
		checkSpan(0+3*width, true)
		checkSpan(1+0*width, false)
		checkSpan(1+1*width, true)
		checkSpan(1+2*width, true)
		checkSpan(1+3*width, false)

		span := checkSpan(0+0*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+0*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+0*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[0+0*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+0*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+1*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+1*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+1*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[0+1*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+1*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+2*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+2*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+2*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[0+2*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+2*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+3*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+3*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+3*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[0+3*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+3*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(1+1*width, true)
		if span.Smin != 0 {
			t.Errorf("span[1+1*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[1+1*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[1+1*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[1+1*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(1+2*width, true)
		if span.Smin != 0 {
			t.Errorf("span[1+2*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[1+2*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[1+2*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[1+2*width].Next = %v, want nil", span.Next)
		}
	})

	t.Run("Triangle list overload", func(t *testing.T) {
		ctx := context.Background()
		solid := CreateHeightfield(ctx, width, height, bmin, bmax, cellSize, cellHeight)

		vertsList := []float32{
			0, 0, 0,
			1, 0, 0,
			0, 0, -1,
			0, 0, 0,
			0, 0, 1,
			1, 0, 0,
		}
		err := RasterizeTrianglesVerts(ctx, vertsList, areas, 2, solid, flagMergeThr)
		if err != nil {
			t.Fatalf("RasterizeTrianglesVerts returned error: %v", err)
		}

		checkSpan := func(idx int, expectNonNil bool) *Span {
			if expectNonNil {
				if solid.Spans[idx] == nil {
					t.Fatalf("solid.Spans[%d] is nil, expected non-nil", idx)
				}
				return solid.Spans[idx]
			} else {
				if solid.Spans[idx] != nil {
					t.Errorf("solid.Spans[%d] = %v, want nil", idx, solid.Spans[idx])
				}
				return nil
			}
		}

		checkSpan(0+0*width, true)
		checkSpan(0+1*width, true)
		checkSpan(0+2*width, true)
		checkSpan(0+3*width, true)
		checkSpan(1+0*width, false)
		checkSpan(1+1*width, true)
		checkSpan(1+2*width, true)
		checkSpan(1+3*width, false)

		span := checkSpan(0+0*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+0*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+0*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[0+0*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+0*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+1*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+1*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+1*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[0+1*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+1*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+2*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+2*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+2*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[0+2*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+2*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(0+3*width, true)
		if span.Smin != 0 {
			t.Errorf("span[0+3*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[0+3*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[0+3*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[0+3*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(1+1*width, true)
		if span.Smin != 0 {
			t.Errorf("span[1+1*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[1+1*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 1 {
			t.Errorf("span[1+1*width].Area = %d, want 1", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[1+1*width].Next = %v, want nil", span.Next)
		}

		span = checkSpan(1+2*width, true)
		if span.Smin != 0 {
			t.Errorf("span[1+2*width].Smin = %d, want 0", span.Smin)
		}
		if span.Smax != 1 {
			t.Errorf("span[1+2*width].Smax = %d, want 1", span.Smax)
		}
		if span.Area != 2 {
			t.Errorf("span[1+2*width].Area = %d, want 2", span.Area)
		}
		if span.Next != nil {
			t.Errorf("span[1+2*width].Next = %v, want nil", span.Next)
		}
	})
}
