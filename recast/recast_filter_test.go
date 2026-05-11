package recast

import (
	"testing"
)

func TestFilterLowHangingWalkableObstacles(t *testing.T) {
	ctx := &Context{}
	walkableClimb := 5

	hf := &Heightfield{
		Width:  1,
		Height: 1,
		Bmin:   [3]float32{0, 0, 0},
		Bmax:   [3]float32{1, 1, 1},
		Cs:     1,
		Ch:     1,
		Spans:  make([]*Span, 1),
	}

	t.Run("Span with no spans above it is unchanged", func(t *testing.T) {
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1}

		if err := FilterLowHangingWalkableObstacles(ctx, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		if hf.Spans[0].Area != 1 {
			t.Errorf("expected span area 1, got %d", hf.Spans[0].Area)
		}

		hf.Spans[0] = nil
	})

	t.Run("Span with span above that is higher than walkableHeight is unchanged", func(t *testing.T) {
		secondSpan := &Span{
			Smin: uint32(1 + walkableClimb),
			Smax: uint32(1 + walkableClimb + 1),
			Area: NullArea,
		}
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1, Next: secondSpan}

		if err := FilterLowHangingWalkableObstacles(ctx, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		// Check that nothing has changed.
		if hf.Spans[0].Area != 1 {
			t.Errorf("expected first span area 1, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next.Area != NullArea {
			t.Errorf("expected second span area %d, got %d", NullArea, hf.Spans[0].Next.Area)
		}

		// Check again but with more clearance
		secondSpan.Smin += 10
		secondSpan.Smax += 10

		if err := FilterLowHangingWalkableObstacles(ctx, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		// Check that nothing has changed.
		if hf.Spans[0].Area != 1 {
			t.Errorf("expected first span area 1, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next.Area != NullArea {
			t.Errorf("expected second span area %d, got %d", NullArea, hf.Spans[0].Next.Area)
		}

		hf.Spans[0] = nil
	})

	t.Run("Marks low obstacles walkable if they're below the walkableClimb", func(t *testing.T) {
		secondSpan := &Span{
			Smin: uint32(1 + (walkableClimb - 1)),
			Smax: uint32(1 + (walkableClimb - 1) + 1),
			Area: NullArea,
		}
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1, Next: secondSpan}

		if err := FilterLowHangingWalkableObstacles(ctx, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		// Check that the second span was changed to walkable.
		if hf.Spans[0].Area != 1 {
			t.Errorf("expected first span area 1, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next.Area != 1 {
			t.Errorf("expected second span area 1, got %d", hf.Spans[0].Next.Area)
		}

		hf.Spans[0] = nil
	})

	t.Run("Low obstacle that overlaps the walkableClimb distance is not changed", func(t *testing.T) {
		secondSpan := &Span{
			Smin: uint32(2 + (walkableClimb - 1)),
			Smax: uint32(2 + (walkableClimb - 1) + 1),
			Area: NullArea,
		}
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1, Next: secondSpan}

		if err := FilterLowHangingWalkableObstacles(ctx, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		// Check that the second span was NOT changed to walkable.
		if hf.Spans[0].Area != 1 {
			t.Errorf("expected first span area 1, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next.Area != NullArea {
			t.Errorf("expected second span area %d, got %d", NullArea, hf.Spans[0].Next.Area)
		}

		hf.Spans[0] = nil
	})

	t.Run("Only the first of multiple, low obstacles are marked walkable", func(t *testing.T) {
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1}
		previousSpan := hf.Spans[0]
		for i := 0; i < 9; i++ {
			nextSpan := &Span{
				Smin: previousSpan.Smax + uint32(walkableClimb-1),
				Smax: previousSpan.Smax + uint32(walkableClimb-1) + 1,
				Area: NullArea,
			}
			previousSpan.Next = nextSpan
			previousSpan = nextSpan
		}

		if err := FilterLowHangingWalkableObstacles(ctx, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		currentSpan := hf.Spans[0]
		for i := 0; i < 10; i++ {
			if currentSpan == nil {
				t.Fatalf("expected span at index %d to not be nil", i)
			}
			// only the first and second spans should be marked as walkable
			expectedArea := uint32(1)
			if i > 1 {
				expectedArea = NullArea
			}
			if currentSpan.Area != expectedArea {
				t.Errorf("span[%d]: expected area %d, got %d", i, expectedArea, currentSpan.Area)
			}
			currentSpan = currentSpan.Next
		}

		hf.Spans[0] = nil
	})
}

func TestFilterLedgeSpans(t *testing.T) {
	ctx := &Context{}
	walkableHeight := 10
	walkableClimb := 5

	hf := &Heightfield{
		Width:  10,
		Height: 10,
		Bmin:   [3]float32{0, 0, 0},
		Bmax:   [3]float32{10, 1, 10},
		Cs:     1,
		Ch:     1,
		Spans:  make([]*Span, 10*10),
	}

	t.Run("Edge spans are marked unwalkable", func(t *testing.T) {
		// Create a flat plane.
		for x := 0; x < hf.Width; x++ {
			for z := 0; z < hf.Height; z++ {
				hf.Spans[x+z*hf.Width] = &Span{Smin: 0, Smax: 1, Area: 1}
			}
		}

		if err := FilterLedgeSpans(ctx, walkableHeight, walkableClimb, hf); err != nil {
			t.Fatal(err)
		}

		for x := 0; x < hf.Width; x++ {
			for z := 0; z < hf.Height; z++ {
				span := hf.Spans[x+z*hf.Width]
				if span == nil {
					t.Fatalf("expected span at (%d,%d) to not be nil", x, z)
				}

				if x == 0 || z == 0 || x == 9 || z == 9 {
					if span.Area != NullArea {
						t.Errorf("span at (%d,%d): expected NullArea, got %d", x, z, span.Area)
					}
				} else {
					if span.Area != 1 {
						t.Errorf("span at (%d,%d): expected area 1, got %d", x, z, span.Area)
					}
				}

				if span.Next != nil {
					t.Errorf("span at (%d,%d): expected Next to be nil", x, z)
				}
				if span.Smin != 0 {
					t.Errorf("span at (%d,%d): expected Smin 0, got %d", x, z, span.Smin)
				}
				if span.Smax != 1 {
					t.Errorf("span at (%d,%d): expected Smax 1, got %d", x, z, span.Smax)
				}
			}
		}

		// Clear all spans
		for x := 0; x < hf.Width; x++ {
			for z := 0; z < hf.Height; z++ {
				hf.Spans[x+z*hf.Width] = nil
			}
		}
	})
}

func TestFilterWalkableLowHeightSpans(t *testing.T) {
	ctx := &Context{}
	walkableHeight := 5

	hf := &Heightfield{
		Width:  1,
		Height: 1,
		Bmin:   [3]float32{0, 0, 0},
		Bmax:   [3]float32{1, 1, 1},
		Cs:     1,
		Ch:     1,
		Spans:  make([]*Span, 1),
	}

	t.Run("span nothing above is unchanged", func(t *testing.T) {
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1}

		if err := FilterWalkableLowHeightSpans(ctx, walkableHeight, hf); err != nil {
			t.Fatal(err)
		}

		if hf.Spans[0].Area != 1 {
			t.Errorf("expected area 1, got %d", hf.Spans[0].Area)
		}

		hf.Spans[0] = nil
	})

	t.Run("span with lots of room above is unchanged", func(t *testing.T) {
		overheadSpan := &Span{Smin: 10, Smax: 11, Area: NullArea}
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1, Next: overheadSpan}

		if err := FilterWalkableLowHeightSpans(ctx, walkableHeight, hf); err != nil {
			t.Fatal(err)
		}

		if hf.Spans[0].Area != 1 {
			t.Errorf("expected first span area 1, got %d", hf.Spans[0].Area)
		}
		if hf.Spans[0].Next.Area != NullArea {
			t.Errorf("expected second span area %d, got %d", NullArea, hf.Spans[0].Next.Area)
		}

		hf.Spans[0] = nil
	})

	t.Run("Span with low hanging obstacle is marked as unwalkable", func(t *testing.T) {
		overheadSpan := &Span{Smin: 3, Smax: 4, Area: NullArea}
		hf.Spans[0] = &Span{Smin: 0, Smax: 1, Area: 1, Next: overheadSpan}

		if err := FilterWalkableLowHeightSpans(ctx, walkableHeight, hf); err != nil {
			t.Fatal(err)
		}

		if hf.Spans[0].Area != NullArea {
			t.Errorf("expected first span area %d, got %d", NullArea, hf.Spans[0].Area)
		}
		if hf.Spans[0].Next.Area != NullArea {
			t.Errorf("expected second span area %d, got %d", NullArea, hf.Spans[0].Next.Area)
		}

		hf.Spans[0] = nil
	})
}
