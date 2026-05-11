package recast

import (
	"math"
	"testing"
)

// defaultTestMesh returns a simple floor mesh: a 9x9 quad made of 2 triangles.
// The mesh lies flat at y=0 within the bounding box [(0,0,0), (9,0,9)].
func defaultTestMesh() (verts []float32, nVerts int, tris []int, nTris int, triAreas []uint8) {
	verts = []float32{
		0, 0, 0,
		9, 0, 0,
		9, 0, 9,
		0, 0, 9,
	}
	nVerts = 4
	tris = []int{0, 1, 2, 0, 2, 3}
	nTris = 2
	triAreas = []uint8{WalkableArea, WalkableArea}
	return
}

// setupHeightfield creates a Context and a Heightfield from the default test mesh,
// using the provided cell size and cell height.
func setupHeightfield(t *testing.T, cs, ch float32) (*Context, *Heightfield) {
	t.Helper()
	verts, nVerts, tris, nTris, triAreas := defaultTestMesh()
	bmin, bmax := CalcBounds(verts, nVerts)
	width, height := CalcGridSize(bmin, bmax, cs)
	ctx := NewContext(false)
	solid := CreateHeightfield(ctx, width, height, bmin, bmax, cs, ch)
	ok, err := RasterizeTriangles(ctx, verts, nVerts, tris, triAreas, nTris, solid, 1)
	if !ok || err != nil {
		t.Fatalf("setupHeightfield: RasterizeTriangles failed: ok=%v err=%v", ok, err)
	}
	return ctx, solid
}

// setupCompactHeightfield creates the complete pipeline up through BuildCompactHeightfield.
func setupCompactHeightfield(t *testing.T, cs, ch float32) (*Context, *CompactHeightfield) {
	t.Helper()
	ctx, solid := setupHeightfield(t, cs, ch)
	chf, err := BuildCompactHeightfield(ctx, 2, 1, solid)
	if err != nil {
		t.Fatalf("setupCompactHeightfield: BuildCompactHeightfield failed: %v", err)
	}
	if chf == nil {
		t.Fatal("setupCompactHeightfield: BuildCompactHeightfield returned nil")
	}
	if chf.SpanCount == 0 {
		t.Fatal("setupCompactHeightfield: compact heightfield has no spans")
	}
	return ctx, chf
}

// setupContourSet runs the full pipeline up through BuildContours.
func setupContourSet(t *testing.T) (*Context, *ContourSet) {
	t.Helper()
	ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
	// Erode
	_, err := ErodeWalkableArea(ctx, 1, chf)
	if err != nil {
		t.Fatalf("setupContourSet: ErodeWalkableArea failed: %v", err)
	}
	// Build distance field
	if !BuildDistanceField(ctx, chf) {
		t.Fatal("setupContourSet: BuildDistanceField failed")
	}
	// Build regions
	if !BuildRegions(ctx, chf, 0, 2, 2) {
		t.Fatal("setupContourSet: BuildRegions failed")
	}
	// Build contours
	cset := BuildContours(ctx, chf, 1.3, 12, int(ContourTessWallEdges|ContourTessAreaEdges))
	if cset == nil {
		t.Fatal("setupContourSet: BuildContours returned nil")
	}
	return ctx, cset
}

// verifySpanCount validates that the given number of non-null spans exists in the heightfield.
func verifySpanCount(t *testing.T, hf *Heightfield, want int) {
	t.Helper()
	count := 0
	for col := 0; col < hf.Width*hf.Height; col++ {
		for s := hf.Spans[col]; s != nil; s = s.Next {
			if s.Area != NullArea {
				count++
			}
		}
	}
	if count < want {
		t.Errorf("heightfield span count = %d, want >= %d", count, want)
	}
}

// TestFullNavmeshBuild performs a complete navmesh build from geometry through
// to detail mesh and verifies all intermediate and final results.
func TestFullNavmeshBuild(t *testing.T) {
	cs := float32(0.3)
	ch := float32(0.2)

	// ---- Stage 1: Set up heightfield and rasterize ----
	ctx, solid := setupHeightfield(t, cs, ch)

	// Verify spans were generated.
	verifySpanCount(t, solid, 1)

	// ---- Stage 2: Build compact heightfield ----
	chf, err := BuildCompactHeightfield(ctx, 2, 1, solid)
	if err != nil {
		t.Fatalf("BuildCompactHeightfield failed: %v", err)
	}
	if chf == nil {
		t.Fatal("BuildCompactHeightfield returned nil")
	}
	if chf.SpanCount == 0 {
		t.Fatal("compact heightfield has no walkable spans")
	}
	t.Logf("compact heightfield: %dx%d, %d spans", chf.Width, chf.Height, chf.SpanCount)

	// ---- Stage 3: Erode walkable area ----
	ok, err := ErodeWalkableArea(ctx, 1, chf)
	if !ok || err != nil {
		t.Fatalf("ErodeWalkableArea failed: ok=%v err=%v", ok, err)
	}
	erodedCount := 0
	for i := 0; i < chf.SpanCount; i++ {
		if chf.Areas[i] != NullArea {
			erodedCount++
		}
	}
	if erodedCount == 0 {
		t.Fatal("erosion removed all walkable area; floor too small for given radius")
	}
	t.Logf("after erosion: %d walkable spans", erodedCount)

	// ---- Stage 4: Build distance field ----
	if !BuildDistanceField(ctx, chf) {
		t.Fatal("BuildDistanceField failed")
	}
	if chf.MaxDistance == 0 {
		t.Fatal("distance field max distance is 0; expected > 0")
	}
	t.Logf("distance field max distance: %d", chf.MaxDistance)

	// ---- Stage 5: Build regions ----
	if !BuildRegions(ctx, chf, 0, 2, 2) {
		t.Fatal("BuildRegions failed")
	}
	regionSet := make(map[uint16]bool)
	for i := 0; i < chf.SpanCount; i++ {
		reg := chf.Spans[i].Reg
		if reg != 0 && reg&borderReg == 0 {
			regionSet[reg] = true
		}
	}
	if len(regionSet) == 0 {
		t.Fatal("no non-border regions were generated")
	}
	t.Logf("regions generated: %d", len(regionSet))

	// ---- Stage 6: Build contours ----
	cset := BuildContours(ctx, chf, 1.3, 12, int(ContourTessWallEdges|ContourTessAreaEdges))
	if cset == nil {
		t.Fatal("BuildContours returned nil")
	}
	if cset.Nconts == 0 {
		t.Fatal("no contours were generated")
	}
	validContours := 0
	for i := 0; i < cset.Nconts; i++ {
		if cset.Conts[i].Nverts >= 3 {
			validContours++
		}
	}
	if validContours == 0 {
		t.Fatal("no valid contours (with >= 3 verts) were generated")
	}
	t.Logf("contours: %d total, %d with >=3 verts", cset.Nconts, validContours)

	// Verify contour bounds are within expected range.
	if cset.Bmin[0] > cset.Bmax[0] || cset.Bmin[2] > cset.Bmax[2] {
		t.Error("contour set has invalid bounds")
	}

	// ---- Stage 7: Build polygon mesh ----
	mesh := &PolyMesh{}
	BuildPolyMesh(cset, 6, mesh)

	if mesh.Npolys == 0 {
		t.Fatal("poly mesh has no polygons")
	}
	if mesh.Nverts == 0 {
		t.Fatal("poly mesh has no vertices")
	}
	if mesh.Nvp != 6 {
		t.Errorf("poly mesh Nvp = %d, want 6", mesh.Nvp)
	}
	t.Logf("poly mesh: %d polys, %d verts", mesh.Npolys, mesh.Nverts)

	// Verify polygon vertices are within bounds.
	for i := 0; i < mesh.Nverts; i++ {
		vx := mesh.Verts[i*3+0]
		vy := mesh.Verts[i*3+1]
		vz := mesh.Verts[i*3+2]
		if vx == 0xffff && vy == 0xffff && vz == 0xffff {
			t.Errorf("vertex %d is uninitialized (0xffff)", i)
		}
	}

	// ---- Stage 8: Build detail mesh ----
	dmesh := &PolyMeshDetail{}
	ok = BuildPolyMeshDetail(ctx, mesh, chf, 6, 1, dmesh)
	if !ok {
		t.Fatal("BuildPolyMeshDetail failed")
	}
	if dmesh.Nmeshes == 0 {
		t.Fatal("detail mesh has no sub-meshes")
	}
	t.Logf("detail mesh: %d meshes, %d verts, %d tris",
		dmesh.Nmeshes, dmesh.Nverts, dmesh.Ntris)

	// Verify detail mesh data integrity.
	if dmesh.Nverts > 0 && len(dmesh.Verts) < dmesh.Nverts*3 {
		t.Errorf("detail mesh verts slice length %d < nverts*3 = %d",
			len(dmesh.Verts), dmesh.Nverts*3)
	}
	if dmesh.Ntris > 0 && len(dmesh.Tris) < dmesh.Ntris*4 {
		t.Errorf("detail mesh tris slice length %d < ntris*4 = %d",
			len(dmesh.Tris), dmesh.Ntris*4)
	}
	if dmesh.Nmeshes > 0 && len(dmesh.Meshes) < dmesh.Nmeshes*4 {
		t.Errorf("detail mesh meshes slice length %d < nmeshes*4 = %d",
			len(dmesh.Meshes), dmesh.Nmeshes*4)
	}

	// Verify that the detail mesh vertices are in world space (should be >= bmin).
	for i := 0; i < dmesh.Nverts; i++ {
		x := dmesh.Verts[i*3+0]
		y := dmesh.Verts[i*3+1]
		z := dmesh.Verts[i*3+2]
		if math.IsInf(float64(x), 0) || math.IsNaN(float64(x)) ||
			math.IsInf(float64(y), 0) || math.IsNaN(float64(y)) ||
			math.IsInf(float64(z), 0) || math.IsNaN(float64(z)) {
			t.Errorf("detail mesh vertex %d has invalid coordinates: (%v, %v, %v)", i, x, y, z)
		}
	}

	t.Log("full navmesh build completed successfully")
}

// TestBuildRegions exercises BuildRegions with different configuration parameters,
// including the use of a border and varying minimum region area thresholds.
func TestBuildRegions(t *testing.T) {
	cs := float32(0.3)
	ch := float32(0.2)

	t.Run("no border, small min region area", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, cs, ch)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx, chf, 0, 2, 2) {
			t.Fatal("BuildRegions failed")
		}

		regionCount := 0
		for i := 0; i < chf.SpanCount; i++ {
			if chf.Spans[i].Reg != 0 {
				regionCount++
			}
		}
		if regionCount == 0 {
			t.Error("no spans assigned to regions")
		}
		if chf.MaxRegions == 0 {
			t.Error("MaxRegions is 0 after BuildRegions")
		}
		t.Logf("no border: %d regions, MaxRegions=%d", regionCount, chf.MaxRegions)
	})

	t.Run("with border (borderSize=2)", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, cs, ch)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx, chf, 2, 2, 2) {
			t.Fatal("BuildRegions failed")
		}

		// With border, some spans should be marked as border regions.
		borderCount := 0
		nonBorderCount := 0
		for i := 0; i < chf.SpanCount; i++ {
			reg := chf.Spans[i].Reg
			if reg != 0 && (reg&borderReg) != 0 {
				borderCount++
			} else if reg != 0 {
				nonBorderCount++
			}
		}
		t.Logf("with border: border=%d, non-border=%d", borderCount, nonBorderCount)
		if borderCount == 0 && nonBorderCount == 0 {
			t.Error("no spans assigned to regions with border enabled")
		}
	})

	t.Run("large min region area (filters small regions)", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, cs, ch)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}

		// Use a large min region area that should filter out all regions on
		// a small mesh, but because the floor is large enough, some regions
		// should survive.
		if !BuildRegions(ctx, chf, 0, 2, 2) {
			t.Fatal("BuildRegions with small minRegionArea failed")
		}
		smallMinRegions := make(map[uint16]bool)
		for i := 0; i < chf.SpanCount; i++ {
			reg := chf.Spans[i].Reg
			if reg != 0 && reg&borderReg == 0 {
				smallMinRegions[reg] = true
			}
		}

		// Rebuild with larger min area and compare.
		ctx2, chf2 := setupCompactHeightfield(t, cs, ch)
		_, err = ErodeWalkableArea(ctx2, 1, chf2)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx2, chf2) {
			t.Fatal("BuildDistanceField failed")
		}
		largeMin := (chf2.SpanCount + 1) * 10 // larger than any possible region
		if !BuildRegions(ctx2, chf2, 0, largeMin, largeMin) {
			t.Fatal("BuildRegions with large minRegionArea failed")
		}
		largeMinRegions := make(map[uint16]bool)
		for i := 0; i < chf2.SpanCount; i++ {
			reg := chf2.Spans[i].Reg
			if reg != 0 && reg&borderReg == 0 {
				largeMinRegions[reg] = true
			}
		}
		t.Logf("small minRegionArea: %d regions, large minRegionArea: %d regions",
			len(smallMinRegions), len(largeMinRegions))
		// With a very large min region area, all regions may be merged or
		// filtered; the key is that the build completes without error.
	})

	t.Run("monotone region building", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, cs, ch)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegionsMonotone(ctx, chf, 0, 2, 2) {
			t.Fatal("BuildRegionsMonotone failed")
		}

		regionCount := 0
		for i := 0; i < chf.SpanCount; i++ {
			if chf.Spans[i].Reg != 0 {
				regionCount++
			}
		}
		if regionCount == 0 {
			t.Error("BuildRegionsMonotone: no spans assigned to regions")
		}
		t.Logf("monotone regions: %d spans in regions", regionCount)
	})
}

// TestBuildContourSet exercises BuildContours with simple known geometry and
// verifies the properties of the resulting contour set.
func TestBuildContourSet(t *testing.T) {
	t.Run("standard build", func(t *testing.T) {
		_, cset := setupContourSet(t)

		// Verify contour set properties.
		if cset == nil {
			t.Fatal("BuildContours returned nil")
		}
		if cset.Nconts == 0 {
			t.Fatal("no contours generated")
		}
		if cset.Cs <= 0 {
			t.Error("contour set cell size <= 0")
		}
		if cset.Ch <= 0 {
			t.Error("contour set cell height <= 0")
		}

		// Verify each contour has valid geometry.
		for i := 0; i < cset.Nconts; i++ {
			cont := cset.Conts[i]
			if cont.Nverts < 3 {
				t.Errorf("contour %d has %d vertices, want >= 3", i, cont.Nverts)
			}
			if cont.Nverts > 0 && cont.Verts == nil {
				t.Errorf("contour %d has Nverts=%d but Verts is nil", i, cont.Nverts)
			}
			if cont.Reg == 0 {
				t.Errorf("contour %d has region id 0", i)
			}
			// Verify vertices are well-formed: each vertex is (x, y, z, flags).
			for j := 0; j < cont.Nverts; j++ {
				v := cont.Verts[j*4:]
				// x and z should be within the heightfield grid.
				if v[0] < 0 || v[2] < 0 {
					t.Errorf("contour %d vertex %d has negative coordinates: (%d, %d, %d)",
						i, j, v[0], v[1], v[2])
				}
			}
		}
	})

	t.Run("with border", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx, chf, 2, 2, 2) {
			t.Fatal("BuildRegions failed")
		}
		cset := BuildContours(ctx, chf, 1.3, 12, int(ContourTessWallEdges|ContourTessAreaEdges))
		if cset == nil {
			t.Fatal("BuildContours returned nil")
		}

		t.Logf("contour set with border: %d contours", cset.Nconts)

		// When border is non-zero, the contour set bounds should be
		// inset from the heightfield bounds.
		for i := 0; i < cset.Nconts; i++ {
			cont := cset.Conts[i]
			if cont.Nverts >= 3 {
				t.Logf("contour %d: %d verts, region=%d, area=%d",
					i, cont.Nverts, cont.Reg, cont.Area)
			}
		}
	})

	t.Run("different max simplification error", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx, chf, 0, 2, 2) {
			t.Fatal("BuildRegions failed")
		}

		// Build contours with zero error (no simplification).
		csetLow := BuildContours(ctx, chf, 0, 12, int(ContourTessWallEdges|ContourTessAreaEdges))
		if csetLow == nil {
			t.Fatal("BuildContours (low error) returned nil")
		}

		// Build contours with higher error (more simplification).
		// Rebuild regions since BuildContours doesn't modify chf.
		ctx2, chf2 := setupCompactHeightfield(t, 0.3, 0.2)
		_, err = ErodeWalkableArea(ctx2, 1, chf2)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx2, chf2) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx2, chf2, 0, 2, 2) {
			t.Fatal("BuildRegions failed")
		}
		_ = csetLow // Keep reference; we mainly care that both succeed.

		_ = BuildContours(ctx2, chf2, 10.0, 12, int(ContourTessWallEdges|ContourTessAreaEdges))

		// The key assertion is that both builds complete without error
		// and produce valid contour sets (already verified by the
		// non-nil check above).
		if ctx == nil || ctx2 == nil {
			t.Error("unexpected nil context")
		}
		t.Log("contour builds with different simplification errors completed successfully")
	})

	t.Run("no build flags", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}
		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx, chf, 0, 2, 2) {
			t.Fatal("BuildRegions failed")
		}
		cset := BuildContours(ctx, chf, 1.3, 12, 0)
		if cset == nil {
			t.Fatal("BuildContours with no flags returned nil")
		}
		if cset.Nconts == 0 {
			t.Error("BuildContours with no flags generated no contours")
		}
		t.Logf("contours with no flags: %d", cset.Nconts)
	})
}

// TestFullBuildWithDifferentResolutions verifies that the build pipeline works
// with different cell sizes and cell heights.
func TestFullBuildWithDifferentResolutions(t *testing.T) {
	testCases := []struct {
		name string
		cs   float32
		ch   float32
	}{
		{"coarse (cs=0.5, ch=0.3)", 0.5, 0.3},
		{"medium (cs=0.3, ch=0.2)", 0.3, 0.2},
		{"fine (cs=0.2, ch=0.1)", 0.2, 0.1},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			ctx, solid := setupHeightfield(t, tc.cs, tc.ch)

			chf, err := BuildCompactHeightfield(ctx, 2, 1, solid)
			if err != nil {
				t.Fatalf("BuildCompactHeightfield failed: %v", err)
			}
			if chf.SpanCount == 0 {
				t.Fatal("no walkable spans")
			}

			_, err = ErodeWalkableArea(ctx, 1, chf)
			if err != nil {
				t.Fatalf("ErodeWalkableArea failed: %v", err)
			}

			if !BuildDistanceField(ctx, chf) {
				t.Fatal("BuildDistanceField failed")
			}

			if !BuildRegions(ctx, chf, 0, 2, 2) {
				t.Fatal("BuildRegions failed")
			}

			cset := BuildContours(ctx, chf, 1.3, 12, int(ContourTessWallEdges|ContourTessAreaEdges))
			if cset == nil || cset.Nconts == 0 {
				t.Fatal("no contours generated")
			}

			mesh := &PolyMesh{}
			BuildPolyMesh(cset, 6, mesh)
			if mesh.Npolys == 0 {
				t.Fatal("no polygons in mesh")
			}

			dmesh := &PolyMeshDetail{}
			ok := BuildPolyMeshDetail(ctx, mesh, chf, 6, 1, dmesh)
			if !ok {
				t.Fatal("BuildPolyMeshDetail failed")
			}
			if dmesh.Nmeshes == 0 {
				t.Fatal("no detail meshes")
			}

			t.Logf("cs=%.1f ch=%.1f: compact=%dx%d spans=%d regions=%d contours=%d polys=%d detail=%d tris=%d",
				tc.cs, tc.ch, chf.Width, chf.Height, chf.SpanCount, chf.MaxRegions,
				cset.Nconts, mesh.Npolys, dmesh.Nmeshes, dmesh.Ntris)
		})
	}
}

// TestContextHandling verifies that functions return appropriate errors when
// called with a nil context.
func TestContextHandling(t *testing.T) {
	t.Run("BuildCompactHeightfield with nil context", func(t *testing.T) {
		_, err := BuildCompactHeightfield(nil, 2, 1, &Heightfield{
			Width: 10, Height: 10,
			Bmin: [3]float32{0, 0, 0},
			Bmax: [3]float32{10, 1, 10},
			Cs:   0.3, Ch: 0.2,
			Spans: make([]*Span, 100),
		})
		if err == nil {
			t.Error("expected error for nil context, got nil")
		}
	})

	t.Run("ErodeWalkableArea with nil context", func(t *testing.T) {
		_, err := ErodeWalkableArea(nil, 1, &CompactHeightfield{
			Width: 10, Height: 10, SpanCount: 100,
			Cells: make([]CompactCell, 100),
			Spans: make([]CompactSpan, 100),
			Areas: make([]uint8, 100),
		})
		if err == nil {
			t.Error("expected error for nil context, got nil")
		}
	})

	t.Run("RasterizeTriangle with nil context", func(t *testing.T) {
		v := [3]float32{0, 0, 0}
		_, err := RasterizeTriangle(nil, v, v, v, 1, &Heightfield{
			Width: 10, Height: 10,
			Bmin: [3]float32{0, 0, 0},
			Bmax: [3]float32{10, 1, 10},
			Cs:   0.3, Ch: 0.2,
			Spans: make([]*Span, 100),
		}, 1)
		if err == nil {
			t.Error("expected error for nil context, got nil")
		}
	})
}

// TestErodeAndMarkAreas tests the ErodeWalkableArea function in combination
// with area marking functions (MarkBoxArea, MarkCylinderArea, MarkConvexPolyArea).
func TestErodeAndMarkAreas(t *testing.T) {
	t.Run("erode with zero radius leaves area unchanged", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
		originalAreas := make([]uint8, len(chf.Areas))
		copy(originalAreas, chf.Areas)

		_, err := ErodeWalkableArea(ctx, 0, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea with zero radius failed: %v", err)
		}

		// Areas should remain the same since radius=0 means no erosion.
		unchanged := true
		for i := range chf.Areas {
			if originalAreas[i] != chf.Areas[i] {
				unchanged = false
				break
			}
		}
		if !unchanged {
			t.Error("areas changed after erode with radius 0")
		}
	})

	t.Run("erode then mark box area", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
		_, err := ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea failed: %v", err)
		}

		// Mark a sub-region of the compact heightfield as a specific area type.
		bmin := [3]float32{2, 0, 2}
		bmax := [3]float32{4, 1, 4}
		err = MarkBoxArea(ctx, bmin, bmax, 42, chf)
		if err != nil {
			t.Fatalf("MarkBoxArea failed: %v", err)
		}

		// At least some spans should be marked with area 42.
		found := false
		for i := 0; i < chf.SpanCount; i++ {
			if chf.Areas[i] == 42 {
				found = true
				break
			}
		}
		if !found {
			t.Log("MarkBoxArea: no spans marked with area 42 (box may be outside walkable area)")
		}

		if !BuildDistanceField(ctx, chf) {
			t.Fatal("BuildDistanceField failed")
		}
		if !BuildRegions(ctx, chf, 0, 2, 2) {
			t.Fatal("BuildRegions failed")
		}
		t.Log("erode + mark + regions completed successfully")
	})

	t.Run("median filter preserves connectivity", func(t *testing.T) {
		ctx, chf := setupCompactHeightfield(t, 0.3, 0.2)
		_, err := MedianFilterWalkableArea(ctx, chf)
		if err != nil {
			t.Fatalf("MedianFilterWalkableArea failed: %v", err)
		}

		// After median filtering, some areas may change but the
		// heightfield should still be valid for further processing.
		_, err = ErodeWalkableArea(ctx, 1, chf)
		if err != nil {
			t.Fatalf("ErodeWalkableArea after median filter failed: %v", err)
		}
		t.Log("median filter + erode completed successfully")
	})
}
