package recast

import (
	"testing"
)

func TestAddSpan(t *testing.T) {
	ctx := NewContext(false)

	const xSize = 4
	const ySize = 10
	const zSize = 4

	const cellSize float32 = 1.0
	const cellHeight float32 = 2.0

	minBounds := [3]float32{0, 0, 0}
	maxBounds := [3]float32{cellSize * xSize, cellHeight * ySize, cellSize * zSize}

	const area = uint8(42)
	const flagMergeThr = 1

	t.Run("Add a span to an empty heightfield", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 1, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span at (0,0)")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})

	t.Run("Adding invalid or zero-size spans does nothing", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		// min == max
		ok, err := AddSpan(ctx, hf, 0, 0, 0, 0, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] != nil {
			t.Error("expected nil span for zero-size span")
		}

		// min > max
		ok, err = AddSpan(ctx, hf, 0, 0, 1, 0, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] != nil {
			t.Error("expected nil span for inverted span")
		}
	})

	t.Run("Two spans that are not touching are not merged", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 1, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 2, 3, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Next == nil {
			t.Fatal("expected non-nil Next (second span)")
		}
		if hf.Spans[0].Next.Smin != 2 {
			t.Errorf("expected Next.Smin=2, got %d", hf.Spans[0].Next.Smin)
		}
		if hf.Spans[0].Next.Smax != 3 {
			t.Errorf("expected Next.Smax=3, got %d", hf.Spans[0].Next.Smax)
		}
		if hf.Spans[0].Next.Area != uint32(area) {
			t.Errorf("expected Next.Area=%d, got %d", area, hf.Spans[0].Next.Area)
		}
		if hf.Spans[0].Next.Next != nil {
			t.Error("expected Next.Next to be nil")
		}
	})

	t.Run("Two spans with different area ids within the flag merge threshold are merged and the highest area ID is used", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 1, 42, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != 42 {
			t.Errorf("expected Area=42, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 1, 2, 24, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 2 {
			t.Errorf("expected Smax=2, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != 42 { // Higher area ID takes precedent
			t.Errorf("expected Area=42 (higher priority), got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})

	t.Run("Two spans with different area ids outside the flag merge threshold are merged and the area ID of the last span added is used", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 1, 42, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != 42 {
			t.Errorf("expected Area=42, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 1, 8, 24, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 8 {
			t.Errorf("expected Smax=8, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != 24 { // Area ID of the last-added span takes precedent
			t.Errorf("expected Area=24 (last added), got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})

	t.Run("Add a span that gets merged with an existing span", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 1, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 1, 2, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 2 {
			t.Errorf("expected Smax=2, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})

	t.Run("Add a span that merges with two spans above and below", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 1, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 2, 3, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0].Next == nil {
			t.Fatal("expected non-nil Next")
		}
		if hf.Spans[0].Next.Smin != 2 {
			t.Errorf("expected Next.Smin=2, got %d", hf.Spans[0].Next.Smin)
		}
		if hf.Spans[0].Next.Smax != 3 {
			t.Errorf("expected Next.Smax=3, got %d", hf.Spans[0].Next.Smax)
		}
		if hf.Spans[0].Next.Area != uint32(area) {
			t.Errorf("expected Next.Area=%d, got %d", area, hf.Spans[0].Next.Area)
		}
		if hf.Spans[0].Next.Next != nil {
			t.Error("expected Next.Next to be nil")
		}

		// After adding the third span, they should all get merged into a single span.
		ok, err = AddSpan(ctx, hf, 0, 0, 1, 2, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 3 {
			t.Errorf("expected Smax=3, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})

	t.Run("Spans are insertion-sorted in ascending order of Y value", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 2, 3, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		ok, err = AddSpan(ctx, hf, 0, 0, 0, 1, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		ok, err = AddSpan(ctx, hf, 0, 0, 6, 7, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}

		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 1 {
			t.Errorf("expected Smax=1, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next == nil {
			t.Fatal("expected non-nil Next")
		}

		if hf.Spans[0].Next.Smin != 2 {
			t.Errorf("expected Next.Smin=2, got %d", hf.Spans[0].Next.Smin)
		}
		if hf.Spans[0].Next.Smax != 3 {
			t.Errorf("expected Next.Smax=3, got %d", hf.Spans[0].Next.Smax)
		}
		if hf.Spans[0].Next.Area != uint32(area) {
			t.Errorf("expected Next.Area=%d, got %d", area, hf.Spans[0].Next.Area)
		}
		if hf.Spans[0].Next.Next == nil {
			t.Fatal("expected non-nil Next.Next")
		}

		if hf.Spans[0].Next.Next.Smin != 6 {
			t.Errorf("expected Next.Next.Smin=6, got %d", hf.Spans[0].Next.Next.Smin)
		}
		if hf.Spans[0].Next.Next.Smax != 7 {
			t.Errorf("expected Next.Next.Smax=7, got %d", hf.Spans[0].Next.Next.Smax)
		}
		if hf.Spans[0].Next.Next.Area != uint32(area) {
			t.Errorf("expected Next.Next.Area=%d, got %d", area, hf.Spans[0].Next.Next.Area)
		}
		if hf.Spans[0].Next.Next.Next != nil {
			t.Error("expected Next.Next.Next to be nil")
		}
	})

	t.Run("Adding a span inside another span merges them", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 8, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 8 {
			t.Errorf("expected Smax=8, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 2, 3, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 8 {
			t.Errorf("expected Smax=8, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})

	t.Run("Overlapping spans are merged", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		ok, err := AddSpan(ctx, hf, 0, 0, 0, 4, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 4 {
			t.Errorf("expected Smax=4, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}

		ok, err = AddSpan(ctx, hf, 0, 0, 2, 6, area, flagMergeThr)
		if !ok || err != nil {
			t.Fatalf("AddSpan failed: ok=%v, err=%v", ok, err)
		}
		if hf.Spans[0] == nil {
			t.Fatal("expected non-nil span")
		}
		if hf.Spans[0].Smin != 0 {
			t.Errorf("expected Smin=0, got %d", hf.Spans[0].Smin)
		}
		if hf.Spans[0].Smax != 6 {
			t.Errorf("expected Smax=6, got %d", hf.Spans[0].Smax)
		}
		if hf.Spans[0].Area != uint32(area) {
			t.Errorf("expected Area=%d, got %d", area, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next != nil {
			t.Error("expected nil Next")
		}
	})
}

func TestAllocSpan(t *testing.T) {
	ctx := NewContext(false)

	const xSize = 50
	const zSize = 50
	const ySize = 10

	const cellSize float32 = 1.0
	const cellHeight float32 = 2.0

	minBounds := [3]float32{0, 0, 0}
	maxBounds := [3]float32{cellSize * xSize, cellHeight * ySize, cellSize * zSize}

	const area = uint8(42)
	const flagMergeThr = 1

	t.Run("Attempting to add more spans than the span pool size allocates a new page", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				ok, err := AddSpan(ctx, hf, x, z, 0, 1, area, flagMergeThr)
				if !ok || err != nil {
					t.Fatalf("AddSpan at (%d,%d) failed: ok=%v, err=%v", x, z, ok, err)
				}
			}
		}
	})
}

func TestRasterizeTriangle(t *testing.T) {
	ctx := NewContext(false)

	const xSize = 10
	const ySize = 10
	const zSize = 10

	const cellSize float32 = 1.0
	const cellHeight float32 = 1.0

	minBounds := [3]float32{0, 0, 0}
	maxBounds := [3]float32{cellSize * xSize, cellHeight * ySize, cellSize * zSize}

	assertNoSpans := func(t *testing.T, hf *Heightfield) {
		t.Helper()
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if hf.Spans[x+z*hf.Width] != nil {
					t.Errorf("expected nil span at (%d,%d)", x, z)
				}
			}
		}
	}

	t.Run("Simple triangle in XZ plane", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{0, 0, 0}
		v1 := [3]float32{2, 0, 0}
		v2 := [3]float32{0, 0, 2}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if (x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0) {
					if span == nil {
						t.Fatalf("expected non-nil span at (%d,%d)", x, z)
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0 at (%d,%d), got %d", x, z, span.Smin)
					}
					if span.Smax != 1 {
						t.Errorf("expected Smax=1 at (%d,%d), got %d", x, z, span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d at (%d,%d), got %d", WalkableArea, x, z, span.Area)
					}
					if span.Next != nil {
						t.Errorf("expected nil Next at (%d,%d)", x, z)
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("Simple triangle inside a single voxel", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{0, 0, 0}
		v1 := [3]float32{1, 0, 0}
		v2 := [3]float32{0, 0, 1}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if x == 0 && z == 0 {
					if span == nil {
						t.Fatal("expected non-nil span at (0,0)")
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0, got %d", span.Smin)
					}
					if span.Smax != 1 {
						t.Errorf("expected Smax=1, got %d", span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d, got %d", WalkableArea, span.Area)
					}
					if span.Next != nil {
						t.Error("expected nil Next")
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("Triangles are clipped by the heightfield bounds", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{-2, 0, -2}
		v1 := [3]float32{4, 0, -2}
		v2 := [3]float32{-2, 0, 4}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if (x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0) {
					if span == nil {
						t.Fatalf("expected non-nil span at (%d,%d)", x, z)
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0 at (%d,%d), got %d", x, z, span.Smin)
					}
					if span.Smax != 1 {
						t.Errorf("expected Smax=1 at (%d,%d), got %d", x, z, span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d at (%d,%d), got %d", WalkableArea, x, z, span.Area)
					}
					if span.Next != nil {
						t.Errorf("expected nil Next at (%d,%d)", x, z)
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("Triangle outside of heightfield bounds rasterizes to nothing", func(t *testing.T) {
		// Outside xz bounds
		func() {
			hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

			v0 := [3]float32{-5, 0, -5}
			v1 := [3]float32{-5, 0, 5}
			v2 := [3]float32{5, 0, -5}

			ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
			if !ok || err != nil {
				t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
			}
			assertNoSpans(t, hf)
		}()

		// Below y bounds
		func() {
			hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

			v0 := [3]float32{0, -1, 0}
			v1 := [3]float32{5, -1, 5}
			v2 := [3]float32{5, -1, 0}

			ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
			if !ok || err != nil {
				t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
			}
			assertNoSpans(t, hf)
		}()

		// Above y bounds
		func() {
			hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

			v0 := [3]float32{0, 40, 0}
			v1 := [3]float32{5, 40, 5}
			v2 := [3]float32{5, 40, 0}

			ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
			if !ok || err != nil {
				t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
			}
			assertNoSpans(t, hf)
		}()
	})

	t.Run("Voxels are rasterized if the triangle overlaps any part of it at all", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{-1, 0, -1}
		v1 := [3]float32{1.01, 0, -1}
		v2 := [3]float32{-1, 0, 1.01}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if x == 0 && z == 0 {
					if span == nil {
						t.Fatal("expected non-nil span at (0,0)")
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0, got %d", span.Smin)
					}
					if span.Smax != 1 {
						t.Errorf("expected Smax=1, got %d", span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d, got %d", WalkableArea, span.Area)
					}
					if span.Next != nil {
						t.Error("expected nil Next")
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("A rasterized triangle vertical span includes all the voxels it is in any way part of", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{0, 0, 0}
		v1 := [3]float32{0.5, 0, 0.5}
		v2 := [3]float32{0.5, 2.01, 0.5}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if x == 0 && z == 0 {
					if span == nil {
						t.Fatal("expected non-nil span at (0,0)")
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0, got %d", span.Smin)
					}
					if span.Smax != 3 {
						t.Errorf("expected Smax=3, got %d", span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d, got %d", WalkableArea, span.Area)
					}
					if span.Next != nil {
						t.Error("expected nil Next")
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("Sloped triangle produces varying span heights", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{0, 0, 0.5}
		v1 := [3]float32{4, 0, 0.5}
		v2 := [3]float32{2, 2, 0.5}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if (x == 0 && z == 0) || (x == 3 && z == 0) {
					if span == nil {
						t.Fatalf("expected non-nil span at (%d,%d)", x, z)
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0 at (%d,%d), got %d", x, z, span.Smin)
					}
					if span.Smax != 1 {
						t.Errorf("expected Smax=1 at (%d,%d), got %d", x, z, span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d at (%d,%d), got %d", WalkableArea, x, z, span.Area)
					}
					if span.Next != nil {
						t.Errorf("expected nil Next at (%d,%d)", x, z)
					}
				} else if (x == 1 && z == 0) || (x == 2 && z == 0) {
					if span == nil {
						t.Fatalf("expected non-nil span at (%d,%d)", x, z)
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0 at (%d,%d), got %d", x, z, span.Smin)
					}
					if span.Smax != 2 {
						t.Errorf("expected Smax=2 at (%d,%d), got %d", x, z, span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d at (%d,%d), got %d", WalkableArea, x, z, span.Area)
					}
					if span.Next != nil {
						t.Errorf("expected nil Next at (%d,%d)", x, z)
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("Triangle crossing Y bounds gets clamped", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{0, -5, 0}
		v1 := [3]float32{1, -5, 1}
		v2 := [3]float32{0.5, 15, 0.5}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				span := hf.Spans[x+z*hf.Width]
				if x == 0 && z == 0 {
					if span == nil {
						t.Fatal("expected non-nil span at (0,0)")
					}
					if span.Smin != 0 {
						t.Errorf("expected Smin=0, got %d", span.Smin)
					}
					if span.Smax != 10 {
						t.Errorf("expected Smax=10, got %d", span.Smax)
					}
					if span.Area != uint32(WalkableArea) {
						t.Errorf("expected Area=%d, got %d", WalkableArea, span.Area)
					}
					if span.Next != nil {
						t.Error("expected nil Next")
					}
				} else {
					if span != nil {
						t.Errorf("expected nil span at (%d,%d), got Smax=%d", x, z, span.Smax)
					}
				}
			}
		}
	})

	t.Run("Degenerate triangles rasterize to nothing", func(t *testing.T) {
		t.Skip("Currently Recast rasterizes degenerate triangles as if they were a line or point of non-zero volume.")

		// Co-linear points
		func() {
			hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

			v0 := [3]float32{1, 0, 0.5}
			v1 := [3]float32{2, 0, 0.5}
			v2 := [3]float32{4, 0, 0.5}

			ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
			if !ok || err != nil {
				t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
			}
			assertNoSpans(t, hf)
		}()

		// All vertices are the same point
		func() {
			hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

			v0 := [3]float32{0.5, 0, 0.5}

			ok, err := RasterizeTriangle(ctx, v0, v0, v0, WalkableArea, hf, 1)
			if !ok || err != nil {
				t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
			}
			assertNoSpans(t, hf)
		}()
	})

	t.Run("Triangles outside the heightfield with bounding boxes that overlap the heightfield rasterize nothing", func(t *testing.T) {
		hf := CreateHeightfield(ctx, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight)

		v0 := [3]float32{-10, 5.5, -10}
		v1 := [3]float32{-10, 5.5, 3}
		v2 := [3]float32{3, 5.5, -10}

		ok, err := RasterizeTriangle(ctx, v0, v1, v2, WalkableArea, hf, 1)
		if !ok || err != nil {
			t.Fatalf("RasterizeTriangle failed: ok=%v, err=%v", ok, err)
		}

		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if hf.Spans[x+z*hf.Width] != nil {
					t.Errorf("expected nil span at (%d,%d)", x, z)
				}
			}
		}
	})
}
