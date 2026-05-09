package recast

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

// TestFilterLowHangingWalkableObstacles tests the FilterLowHangingWalkableObstacles function
func TestFilterLowHangingWalkableObstacles(t *testing.T) {
	ctx := NewContext(false)
	walkableHeight := 5

	var heightfield Heightfield
	heightfield.Width = 1
	heightfield.Height = 1
	heightfield.Bmin = [3]float32{0, 0, 0}
	heightfield.Bmax = [3]float32{1, 1, 1}
	heightfield.Cs = 1
	heightfield.Ch = 1
	heightfield.Spans = make([]*Span, heightfield.Width*heightfield.Height)
	heightfield.Pools = nil
	heightfield.FreeList = nil

	t.Run("Span with no spans above it is unchanged", func(t *testing.T) {
		span := &Span{
			Area: 1,
			Next: nil,
			Smin: 0,
			Smax: 1,
		}
		heightfield.Spans[0] = span

		FilterLowHangingWalkableObstacles(ctx, walkableHeight, &heightfield)

		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
	})

	t.Run("Span with span above that is higher than walkableHeight is unchanged", func(t *testing.T) {
		// Put the second span just above the first one.
		secondSpan := &Span{
			Area: NullArea,
			Next: nil,
			Smin: 1 + uint32(walkableHeight),
			Smax: 1 + uint32(walkableHeight) + 1,
		}

		span := &Span{
			Area: 1,
			Next: secondSpan,
			Smin: 0,
			Smax: 1,
		}

		heightfield.Spans[0] = span

		FilterLowHangingWalkableObstacles(ctx, walkableHeight, &heightfield)

		// Check that nothing has changed.
		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
		assert.Equal(t, uint32(NullArea), heightfield.Spans[0].Next.Area)

		// Check again but with a more clearance
		secondSpan.Smin += 10
		secondSpan.Smax += 10

		FilterLowHangingWalkableObstacles(ctx, walkableHeight, &heightfield)

		// Check that nothing has changed.
		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
		assert.Equal(t, uint32(NullArea), heightfield.Spans[0].Next.Area)
	})

	t.Run("Marks low obstacles walkable if they're below the walkableClimb", func(t *testing.T) {
		// Reset heightfield
		heightfield.Spans = make([]*Span, heightfield.Width*heightfield.Height)

		// Put the second span just above the first one.
		secondSpan := &Span{
			Area: NullArea,
			Next: nil,
			Smin: 1 + uint32(walkableHeight-1),
			Smax: 1 + uint32(walkableHeight-1) + 1,
		}

		span := &Span{
			Area: 1,
			Next: secondSpan,
			Smin: 0,
			Smax: 1,
		}

		heightfield.Spans[0] = span

		FilterLowHangingWalkableObstacles(ctx, walkableHeight, &heightfield)

		// Check that the second span was changed to walkable.
		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
		assert.Equal(t, uint32(1), heightfield.Spans[0].Next.Area)
	})

	t.Run("Low obstacle that overlaps the walkableClimb distance is not changed", func(t *testing.T) {
		// Reset heightfield
		heightfield.Spans = make([]*Span, heightfield.Width*heightfield.Height)

		// Put the second span just above the first one.
		secondSpan := &Span{
			Area: NullArea,
			Next: nil,
			Smin: 2 + uint32(walkableHeight-1),
			Smax: 2 + uint32(walkableHeight-1) + 1,
		}

		span := &Span{
			Area: 1,
			Next: secondSpan,
			Smin: 0,
			Smax: 1,
		}

		heightfield.Spans[0] = span

		FilterLowHangingWalkableObstacles(ctx, walkableHeight, &heightfield)

		// Check that the second span was not changed.
		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
		assert.Equal(t, uint32(NullArea), heightfield.Spans[0].Next.Area)
	})

	t.Run("Only the first of multiple, low obstacles are marked walkable", func(t *testing.T) {
		// Reset heightfield
		heightfield.Spans = make([]*Span, heightfield.Width*heightfield.Height)

		span := &Span{
			Area: 1,
			Next: nil,
			Smin: 0,
			Smax: 1,
		}
		heightfield.Spans[0] = span

		previousSpan := span
		for i := 0; i < 9; i++ {
			nextSpan := &Span{
				Area: NullArea,
				Next: nil,
				Smin: previousSpan.Smax + uint32(walkableHeight-1),
				Smax: previousSpan.Smax + uint32(walkableHeight-1) + 1,
			}
			previousSpan.Next = nextSpan
			previousSpan = nextSpan
		}

		FilterLowHangingWalkableObstacles(ctx, walkableHeight, &heightfield)

		currentSpan := heightfield.Spans[0]
		for i := 0; i < 10; i++ {
			assert.NotNil(t, currentSpan)
			// only the first and second spans should be marked as walkable
			if i <= 1 {
				assert.Equal(t, uint32(1), currentSpan.Area)
			} else {
				assert.Equal(t, uint32(NullArea), currentSpan.Area)
			}
			currentSpan = currentSpan.Next
		}
	})
}

// TestFilterLedgeSpans tests the FilterLedgeSpans function
func TestFilterLedgeSpans(t *testing.T) {
	ctx := NewContext(false)
	walkableClimb := 5
	walkableHeight := 10

	var heightfield Heightfield
	heightfield.Width = 10
	heightfield.Height = 10
	heightfield.Bmin = [3]float32{0, 0, 0}
	heightfield.Bmax = [3]float32{10, 1, 10}
	heightfield.Cs = 1
	heightfield.Ch = 1
	heightfield.Spans = make([]*Span, heightfield.Width*heightfield.Height)
	heightfield.Pools = nil
	heightfield.FreeList = nil

	t.Run("Edge spans are marked unwalkable", func(t *testing.T) {
		// Create a flat plane.
		for x := 0; x < heightfield.Width; x++ {
			for z := 0; z < heightfield.Height; z++ {
				span := &Span{
					Area: 1,
					Next: nil,
					Smin: 0,
					Smax: 1,
				}
				heightfield.Spans[x+z*heightfield.Width] = span
			}
		}

		FilterLedgeSpans(ctx, walkableHeight, walkableClimb, &heightfield)

		for x := 0; x < heightfield.Width; x++ {
			for z := 0; z < heightfield.Height; z++ {
				span := heightfield.Spans[x+z*heightfield.Width]
				assert.NotNil(t, span)

				if x == 0 || z == 0 || x == 9 || z == 9 {
					assert.Equal(t, uint32(NullArea), span.Area)
				} else {
					assert.Equal(t, uint32(1), span.Area)
				}

				assert.Nil(t, span.Next)
				assert.Equal(t, uint32(0), span.Smin)
				assert.Equal(t, uint32(1), span.Smax)
			}
		}
	})
}

// TestFilterWalkableLowHeightSpans tests the FilterWalkableLowHeightSpans function
func TestFilterWalkableLowHeightSpans(t *testing.T) {
	ctx := NewContext(false)
	walkableHeight := 5

	var heightfield Heightfield
	heightfield.Width = 1
	heightfield.Height = 1
	heightfield.Bmin = [3]float32{0, 0, 0}
	heightfield.Bmax = [3]float32{1, 1, 1}
	heightfield.Cs = 1
	heightfield.Ch = 1
	heightfield.Spans = make([]*Span, heightfield.Width*heightfield.Height)
	heightfield.Pools = nil
	heightfield.FreeList = nil

	t.Run("span nothing above is unchanged", func(t *testing.T) {
		span := &Span{
			Area: 1,
			Next: nil,
			Smin: 0,
			Smax: 1,
		}
		heightfield.Spans[0] = span

		FilterWalkableLowHeightSpans(ctx, walkableHeight, &heightfield)

		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
	})

	t.Run("span with lots of room above is unchanged", func(t *testing.T) {
		overheadSpan := &Span{
			Area: NullArea,
			Next: nil,
			Smin: 10,
			Smax: 11,
		}

		span := &Span{
			Area: 1,
			Next: overheadSpan,
			Smin: 0,
			Smax: 1,
		}
		heightfield.Spans[0] = span

		FilterWalkableLowHeightSpans(ctx, walkableHeight, &heightfield)

		assert.Equal(t, uint32(1), heightfield.Spans[0].Area)
		assert.Equal(t, uint32(NullArea), heightfield.Spans[0].Next.Area)
	})

	t.Run("Span with low hanging obstacle is marked as unwalkable", func(t *testing.T) {
		overheadSpan := &Span{
			Area: NullArea,
			Next: nil,
			Smin: 3,
			Smax: 4,
		}

		span := &Span{
			Area: 1,
			Next: overheadSpan,
			Smin: 0,
			Smax: 1,
		}
		heightfield.Spans[0] = span

		FilterWalkableLowHeightSpans(ctx, walkableHeight, &heightfield)

		assert.Equal(t, uint32(NullArea), heightfield.Spans[0].Area)
		assert.Equal(t, uint32(NullArea), heightfield.Spans[0].Next.Area)
	})
}
