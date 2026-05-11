package recast

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

// TestAddSpan tests the AddSpan function
func TestAddSpan(t *testing.T) {
	ctx := NewContext(false)

	const xSize = 4
	const ySize = 10
	const zSize = 4

	const cellSize = 1.0
	const cellHeight = 2.0

	minBounds := [3]float32{0.0, 0.0, 0.0}
	maxBounds := [3]float32{cellSize * xSize, cellHeight * ySize, cellSize * zSize}

	var hf Heightfield
	assert.True(t, CreateHeightfield(ctx, &hf, xSize, zSize, &minBounds, &maxBounds, cellSize, cellHeight))

	const area = uint8(42)
	const flagMergeThr = 1

	t.Run("Add a span to an empty heightfield", func(t *testing.T) {
		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)
	})

	t.Run("Adding invalid or zero-size spans does nothing", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// min == max
		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 0, area, flagMergeThr)
		assert.Nil(t, hf.Spans[0])

		// min > max
		_, _ = AddSpan(ctx, &hf, 0, 0, 1, 0, area, flagMergeThr)
		assert.Nil(t, hf.Spans[0])
	})

	t.Run("Two spans that are not touching are not merged", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 2, 3, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.NotNil(t, hf.Spans[0].Next)
		assert.Equal(t, uint32(2), hf.Spans[0].Next.Smin)
		assert.Equal(t, uint32(3), hf.Spans[0].Next.Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Next.Area)
		assert.Nil(t, hf.Spans[0].Next.Next)
	})

	t.Run("Two spans with different area ids within the flag merge threshold are merged and the highest area ID is used", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, 42, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(42), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 1, 2, 24, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(2), hf.Spans[0].Smax)
		assert.Equal(t, uint32(42), hf.Spans[0].Area) // Higher area ID takes precedent
		assert.Nil(t, hf.Spans[0].Next)
	})

	t.Run("Two spans with different area ids outside the flag merge threshold are merged and the area ID of the last span added is used", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, 42, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(42), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 1, 8, 24, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(8), hf.Spans[0].Smax)
		assert.Equal(t, uint32(24), hf.Spans[0].Area) // Area ID of the last-added span takes precedent
		assert.Nil(t, hf.Spans[0].Next)
	})

	t.Run("Add a span that gets merged with an existing span", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 1, 2, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(2), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)
	})

	t.Run("Add a span that merges with two spans above and below", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 2, 3, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0].Next)
		assert.Equal(t, uint32(2), hf.Spans[0].Next.Smin)
		assert.Equal(t, uint32(3), hf.Spans[0].Next.Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Next.Area)
		assert.Nil(t, hf.Spans[0].Next.Next)

		// After adding the third span, they should all get merged into a single span.
		_, _ = AddSpan(ctx, &hf, 0, 0, 1, 2, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(3), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)
	})

	t.Run("Spans are insertion-sorted in ascending order of Y value", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 2, 3, area, flagMergeThr)
		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 1, area, flagMergeThr)
		_, _ = AddSpan(ctx, &hf, 0, 0, 6, 7, area, flagMergeThr)

		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(1), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.NotNil(t, hf.Spans[0].Next)

		assert.Equal(t, uint32(2), hf.Spans[0].Next.Smin)
		assert.Equal(t, uint32(3), hf.Spans[0].Next.Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Next.Area)
		assert.NotNil(t, hf.Spans[0].Next.Next)

		assert.Equal(t, uint32(6), hf.Spans[0].Next.Next.Smin)
		assert.Equal(t, uint32(7), hf.Spans[0].Next.Next.Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Next.Next.Area)
		assert.Nil(t, hf.Spans[0].Next.Next.Next)
	})

	t.Run("Adding a span inside another span merges them", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 8, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(8), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 2, 3, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(8), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)
	})

	t.Run("Overlapping spans are merged", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		_, _ = AddSpan(ctx, &hf, 0, 0, 0, 4, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(4), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)

		_, _ = AddSpan(ctx, &hf, 0, 0, 2, 6, area, flagMergeThr)
		assert.NotNil(t, hf.Spans[0])
		assert.Equal(t, uint32(0), hf.Spans[0].Smin)
		assert.Equal(t, uint32(6), hf.Spans[0].Smax)
		assert.Equal(t, uint32(area), hf.Spans[0].Area)
		assert.Nil(t, hf.Spans[0].Next)
	})
}

// TestAllocSpan tests the span allocation with pool
func TestAllocSpan(t *testing.T) {
	ctx := NewContext(false)

	const xSize = 50
	const zSize = 50
	const ySize = 10

	const cellSize = 1.0
	const cellHeight = 2.0

	minBounds := [3]float32{0.0, 0.0, 0.0}
	maxBounds := [3]float32{cellSize * xSize, cellHeight * ySize, cellSize * zSize}

	var hf Heightfield
	assert.True(t, CreateHeightfield(ctx, &hf, xSize, zSize, &minBounds, &maxBounds, cellSize, cellHeight))

	const area = uint8(42)
	const flagMergeThr = 1

	t.Run("Attempting to add more spans than the span pool size allocates a new page", func(t *testing.T) {
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				_, _ = AddSpan(ctx, &hf, x, z, 0, 1, area, flagMergeThr)
			}
		}
	})
}

// Helper function to check if span exists at position
func requireSpanAt(t *testing.T, hf *Heightfield, x, z int, expectedSmin, expectedSmax uint32, expectedArea uint32) {
	span := hf.Spans[x+z*hf.Width]
	assert.NotNil(t, span)
	assert.Equal(t, expectedSmin, span.Smin)
	assert.Equal(t, expectedSmax, span.Smax)
	assert.Equal(t, expectedArea, span.Area)
	assert.Nil(t, span.Next)
}

func requireNoSpanAt(t *testing.T, hf *Heightfield, x, z int) {
	span := hf.Spans[x+z*hf.Width]
	assert.Nil(t, span)
}

// TestRasterizeTriangle tests the RasterizeTriangle function
func TestRasterizeTriangle(t *testing.T) {
	ctx := NewContext(false)

	const xSize = 10
	const ySize = 10
	const zSize = 10

	const cellSize = 1.0
	const cellHeight = 1.0

	minBounds := [3]float32{0.0, 0.0, 0.0}
	maxBounds := [3]float32{cellSize * xSize, cellHeight * ySize, cellSize * zSize}

	var hf Heightfield
	assert.True(t, CreateHeightfield(ctx, &hf, xSize, zSize, &minBounds, &maxBounds, cellSize, cellHeight))

	t.Run("Simple triangle in XZ plane", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// Triangle in the XZ plane with vertices (0,0,0), (2,0,0), (0,0,2)
		// Clockwise winding order so the normal points in the positive Y
		v0 := [3]float32{0.0, 0.0, 0.0}
		v1 := [3]float32{2.0, 0.0, 0.0}
		v2 := [3]float32{0.0, 0.0, 2.0}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if (x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0) {
					requireSpanAt(t, &hf, x, z, 0, 1, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("Simple triangle inside a single voxel", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// Clockwise winding order so the normal points in the positive Y
		v0 := [3]float32{0.0, 0.0, 0.0}
		v1 := [3]float32{1.0, 0.0, 0.0}
		v2 := [3]float32{0.0, 0.0, 1.0}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if x == 0 && z == 0 {
					requireSpanAt(t, &hf, x, z, 0, 1, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("Triangles are clipped by the heightfield bounds", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// When clipped, this should be the same as the triangle in the previous test
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		v0 := [3]float32{-2.0, 0.0, -2.0}
		v1 := [3]float32{4.0, 0.0, -2.0}
		v2 := [3]float32{-2.0, 0.0, 4.0}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if (x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0) {
					requireSpanAt(t, &hf, x, z, 0, 1, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("Triangle outside of heightfield bounds rasterizes to nothing", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// Outside xz bounds
		v0 := [3]float32{-5.0, 0.0, -5.0}
		v1 := [3]float32{-5.0, 0.0, 5.0}
		v2 := [3]float32{5.0, 0.0, -5.0}
		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that no spans were added
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if hf.Spans[x+z*hf.Width] != nil {
					t.Logf("Unexpected span at (%d, %d): Smin=%d, Smax=%d, Area=%d",
						x, z, hf.Spans[x+z*hf.Width].Smin, hf.Spans[x+z*hf.Width].Smax, hf.Spans[x+z*hf.Width].Area)
				}
				requireNoSpanAt(t, &hf, x, z)
			}
		}

		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// below y bounds
		v0 = [3]float32{0.0, -1.0, 0.0}
		v1 = [3]float32{5.0, -1.0, 5.0}
		v2 = [3]float32{5.0, -1.0, 0.0}
		ok, err = RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that no spans were added
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				requireNoSpanAt(t, &hf, x, z)
			}
		}

		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// above y bounds
		v0 = [3]float32{0.0, 40.0, 0.0}
		v1 = [3]float32{5.0, 40.0, 5.0}
		v2 = [3]float32{5.0, 40.0, 0.0}
		ok, err = RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that no spans were added
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				requireNoSpanAt(t, &hf, x, z)
			}
		}
	})

	t.Run("Voxels are rasterized if the triangle overlaps any part of it at all", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// This triangle when clipped barely overlaps the area of the cell at 0,0

		// Clockwise winding order so the normal points in the positive Y
		v0 := [3]float32{-1.0, 0.0, -1.0}
		v1 := [3]float32{1.01, 0.0, -1.0}
		v2 := [3]float32{-1.0, 0.0, 1.01}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if x == 0 && z == 0 {
					requireSpanAt(t, &hf, x, z, 0, 1, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("A rasterized triangle vertical span includes all the voxels it is in any way part of", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// Clockwise winding order so the normal points in the positive Y
		v0 := [3]float32{0.0, 0.0, 0.0}
		v1 := [3]float32{0.5, 0.0, 0.5}
		v2 := [3]float32{0.5, 2.01, 0.5}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if x == 0 && z == 0 {
					requireSpanAt(t, &hf, x, z, 0, 3, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("Sloped triangle produces varying span heights", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		v0 := [3]float32{0.0, 0.0, 0.5}
		v1 := [3]float32{4.0, 0.0, 0.5}
		v2 := [3]float32{2.0, 2.0, 0.5}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if (x == 0 && z == 0) || (x == 3 && z == 0) {
					requireSpanAt(t, &hf, x, z, 0, 1, WalkableArea)
				} else if (x == 1 && z == 0) || (x == 2 && z == 0) {
					requireSpanAt(t, &hf, x, z, 0, 2, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("Triangle crossing Y bounds gets clamped", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// This triangle should be clipped to both the max and min Y bounds of the heightfield.
		v0 := [3]float32{0.0, -5.0, 0.0}
		v1 := [3]float32{1.0, -5.0, 1.0}
		v2 := [3]float32{0.5, 15.0, 0.5}

		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that only expected cells have spans
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				if x == 0 && z == 0 {
					requireSpanAt(t, &hf, x, z, 0, 10, WalkableArea)
				} else {
					requireNoSpanAt(t, &hf, x, z)
				}
			}
		}
	})

	t.Run("Degenerate triangles rasterize to nothing", func(t *testing.T) {
		t.Skip("Currently Recast rasterizes degenerate triangles as if they were a line or point of non-zero volume.")

		// Co-linear points
		v0 := [3]float32{1.0, 0.0, 0.5}
		v1 := [3]float32{2.0, 0.0, 0.5}
		v2 := [3]float32{4.0, 0.0, 0.5}
		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that no spans were added
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				requireNoSpanAt(t, &hf, x, z)
			}
		}

		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// All vertices are the same point
		v0 = [3]float32{0.5, 0.0, 0.5}
		ok, err = RasterizeTriangle(ctx, &v0, &v0, &v0, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that no spans were added
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				requireNoSpanAt(t, &hf, x, z)
			}
		}
	})

	t.Run("Triangles outside the heightfield with bounding boxes that overlap the heightfield rasterize nothing", func(t *testing.T) {
		// Reset heightfield
		hf.Spans = make([]*Span, hf.Width*hf.Height)

		// This is a minimal repro case for the issue fixed in PR #476
		v0 := [3]float32{-10.0, 5.5, -10.0}
		v1 := [3]float32{-10.0, 5.5, 3.0}
		v2 := [3]float32{3.0, 5.5, -10.0}
		ok, err := RasterizeTriangle(ctx, &v0, &v1, &v2, WalkableArea, &hf, 1)
		assert.True(t, ok)
		assert.NoError(t, err)

		// Check that no spans were added
		for x := 0; x < xSize; x++ {
			for z := 0; z < zSize; z++ {
				requireNoSpanAt(t, &hf, x, z)
			}
		}
	})
}

// TestRasterizeNilCtx tests nil ctx returns an error
func TestRasterizeNilCtx(t *testing.T) {
	var hf Heightfield
	_, err := RasterizeTriangle(nil, &[3]float32{}, &[3]float32{}, &[3]float32{}, 0, &hf, 0)
	assert.Error(t, err)
}
