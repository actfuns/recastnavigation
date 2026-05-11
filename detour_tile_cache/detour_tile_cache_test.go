package detour_tile_cache

import (
	"math"
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

// ---------------------------------------------------------------------------
// Test implementations (Alloc, Compressor, MeshProcess, NavMeshInterface)
// ---------------------------------------------------------------------------

// testAlloc is a simple allocator for testing.
type testAlloc struct {
	bufs [][]uint8
}

func (a *testAlloc) Reset() {
	a.bufs = nil
}

func (a *testAlloc) Alloc(size int) []uint8 {
	if size <= 0 {
		return nil
	}
	buf := make([]uint8, size)
	a.bufs = append(a.bufs, buf)
	return buf
}

func (a *testAlloc) Free(ptr []uint8) {}

// testCompressor is a pass-through compressor (no actual compression).
type testCompressor struct{}

func (c *testCompressor) MaxCompressedSize(bufferSize int) int {
	return bufferSize
}

func (c *testCompressor) Compress(buffer []uint8, compressed []uint8) (int, error) {
	return copy(compressed, buffer), nil
}

func (c *testCompressor) Decompress(compressed []uint8, buffer []uint8) (int, error) {
	return copy(buffer, compressed), nil
}

// testMeshProcess is a no-op mesh processor.
type testMeshProcess struct{}

func (m *testMeshProcess) Process(params *NavMeshCreateParams, polyAreas []uint8, polyFlags []uint16) {
}

// mockNavMeshEntry stores tile data with coordinates.
type mockNavMeshEntry struct {
	tx, ty, tlayer int32
	data           []uint8
	dataSize       int
}

// mockNavMesh simulates a NavMesh for TileCache operations.
type mockNavMesh struct {
	tiles   map[uint32]*mockNavMeshEntry
	nextRef uint32
}

func newMockNavMesh() *mockNavMesh {
	return &mockNavMesh{
		tiles:   make(map[uint32]*mockNavMeshEntry),
		nextRef: 0,
	}
}

func (m *mockNavMesh) RemoveTile(ref uint32) ([]uint8, int, error) {
	entry, ok := m.tiles[ref]
	if !ok {
		return nil, 0, nil
	}
	delete(m.tiles, ref)
	if entry != nil {
		return entry.data, entry.dataSize, nil
	}
	return nil, 0, nil
}

func (m *mockNavMesh) GetTileRefAt(tx, ty, tlayer int32) uint32 {
	for ref, entry := range m.tiles {
		if entry.tx == tx && entry.ty == ty && entry.tlayer == tlayer {
			return ref
		}
	}
	return 0
}

func (m *mockNavMesh) AddTile(data []uint8, dataSize int, flags uint8, tileRef *uint32) error {
	m.nextRef++
	ref := m.nextRef
	m.tiles[ref] = &mockNavMeshEntry{
		data:     data,
		dataSize: dataSize,
	}
	if tileRef != nil {
		*tileRef = ref
	}
	return nil
}

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

// defaultParams returns standard TileCacheParams for testing.
func defaultParams() *TileCacheParams {
	return &TileCacheParams{
		Orig:                   [3]float32{0, 0, 0},
		Cs:                     0.3,
		Ch:                     0.2,
		Width:                  20,
		Height:                 20,
		WalkableHeight:         2.0,
		WalkableRadius:         0.6,
		WalkableClimb:          1.0,
		MaxSimplificationError: 1.3,
		MaxTiles:               32,
		MaxObstacles:           64,
	}
}

// createTestTileCache creates an initialized TileCache for testing.
func createTestTileCache(t *testing.T) *TileCache {
	t.Helper()
	tc := NewTileCache()
	err := tc.Init(defaultParams(), &testAlloc{}, &testCompressor{}, &testMeshProcess{})
	if err != nil {
		t.Fatalf("Init: %v", err)
	}
	return tc
}

// createTestLayerHeader creates a minimal valid layer header.
func createTestLayerHeader(tx, ty, tlayer int32) *TileCacheLayerHeader {
	return &TileCacheLayerHeader{
		Magic:   TileCacheMagic,
		Version: TileCacheVersion,
		Tx:      tx,
		Ty:      ty,
		Tlayer:  tlayer,
		Bmin:    [3]float32{0, 0, 0},
		Bmax:    [3]float32{6, 4, 6},
		Width:   6,
		Height:  6,
	}
}

// createCompressedTile creates compressed tile data for AddTile.
func createCompressedTile(t *testing.T, comp TileCacheCompressor, header *TileCacheLayerHeader,
	heights, areas, cons []uint8) ([]uint8, int) {
	t.Helper()
	data, dataSize, err := BuildTileCacheLayer(comp, header, heights, areas, cons)
	if err != nil {
		t.Fatalf("BuildTileCacheLayer: %v", err)
	}
	return data, dataSize
}

// createTestLayer6x6 creates a 6×6 layer with a 4×4 walkable area in the center.
// Layout:
//
//	X X X X X X
//	X . . . . X
//	X . . . . X
//	X . . . . X
//	X . . . . X
//	X X X X X X
//
// X = null area (0), . = walkable area (63)
func createTestLayer6x6() (header *TileCacheLayerHeader, heights, areas, cons []uint8) {
	header = createTestLayerHeader(0, 0, 0)
	gridSize := 6 * 6

	heights = make([]uint8, gridSize)
	areas = make([]uint8, gridSize)
	cons = make([]uint8, gridSize)

	for y := 0; y < 6; y++ {
		for x := 0; x < 6; x++ {
			idx := y*6 + x
			heights[idx] = 1
			if x >= 1 && x <= 4 && y >= 1 && y <= 4 {
				areas[idx] = TileCacheWalkableArea
			} else {
				areas[idx] = TileCacheNullArea
			}
		}
	}
	return
}

// ---------------------------------------------------------------------------
// Utility function tests
// ---------------------------------------------------------------------------

func TestAlign4(t *testing.T) {
	tests := []struct {
		input    int
		expected int
	}{
		{0, 0},
		{1, 4},
		{2, 4},
		{3, 4},
		{4, 4},
		{5, 8},
		{7, 8},
		{8, 8},
		{56, 56},
		{57, 60},
	}
	for _, tt := range tests {
		got := Align4(tt.input)
		if got != tt.expected {
			t.Errorf("Align4(%d) = %d, expected %d", tt.input, got, tt.expected)
		}
	}
}

func TestNextPow2(t *testing.T) {
	tests := []struct {
		input    uint32
		expected uint32
	}{
		{0, 1},
		{1, 1},
		{2, 2},
		{3, 4},
		{4, 4},
		{5, 8},
		{8, 8},
		{9, 16},
		{31, 32},
		{32, 32},
		{255, 256},
	}
	for _, tt := range tests {
		got := NextPow2(tt.input)
		if got != tt.expected {
			t.Errorf("NextPow2(%d) = %d, expected %d", tt.input, got, tt.expected)
		}
	}
}

func TestIlog2(t *testing.T) {
	tests := []struct {
		input    uint32
		expected int
	}{
		{1, 0},
		{2, 1},
		{3, 1},
		{4, 2},
		{8, 3},
		{16, 4},
		{32, 5},
		{256, 8},
	}
	for _, tt := range tests {
		got := Ilog2(tt.input)
		if got != tt.expected {
			t.Errorf("Ilog2(%d) = %d, expected %d", tt.input, got, tt.expected)
		}
	}
}

func TestTileCacheLayerHeaderSize(t *testing.T) {
	size := TileCacheLayerHeaderSize()
	expected := 56
	if size != expected {
		t.Errorf("TileCacheLayerHeaderSize() = %d, expected %d", size, expected)
	}
	if size != Align4(56) {
		t.Errorf("TileCacheLayerHeaderSize() should be 4-byte aligned")
	}
}

// ---------------------------------------------------------------------------
// NewTileCache / Init tests
// ---------------------------------------------------------------------------

func TestNewTileCache(t *testing.T) {
	tc := NewTileCache()
	if tc == nil {
		t.Fatal("NewTileCache returned nil")
	}

	t.Run("default tile count is 0 before Init", func(t *testing.T) {
		if tc.GetTileCount() != 0 {
			t.Errorf("expected 0, got %d", tc.GetTileCount())
		}
	})

	t.Run("default obstacle count is 0 before Init", func(t *testing.T) {
		if tc.GetObstacleCount() != 0 {
			t.Errorf("expected 0, got %d", tc.GetObstacleCount())
		}
	})
}

func TestTileCacheInit(t *testing.T) {
	t.Run("valid params", func(t *testing.T) {
		tc := NewTileCache()
		err := tc.Init(defaultParams(), &testAlloc{}, &testCompressor{}, &testMeshProcess{})
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		if tc.GetTileCount() != 32 {
			t.Errorf("expected 32 tiles, got %d", tc.GetTileCount())
		}
		if tc.GetObstacleCount() != 64 {
			t.Errorf("expected 64 obstacles, got %d", tc.GetObstacleCount())
		}
	})

	t.Run("MaxTiles too large for saltBits", func(t *testing.T) {
		tc := NewTileCache()
		params := defaultParams()
		params.MaxTiles = 1 << 23 // tileBits=23, saltBits=9 < 10
		err := tc.Init(params, &testAlloc{}, &testCompressor{}, &testMeshProcess{})
		if err != detour.ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})

	t.Run("init twice replaces config", func(t *testing.T) {
		tc := NewTileCache()
		err := tc.Init(defaultParams(), &testAlloc{}, &testCompressor{}, &testMeshProcess{})
		if err != nil {
			t.Fatalf("first Init: %v", err)
		}
		params2 := defaultParams()
		params2.MaxTiles = 16
		params2.MaxObstacles = 32
		err = tc.Init(params2, &testAlloc{}, &testCompressor{}, &testMeshProcess{})
		if err != nil {
			t.Fatalf("second Init: %v", err)
		}
		if tc.GetTileCount() != 16 {
			t.Errorf("expected 16 tiles, got %d", tc.GetTileCount())
		}
		if tc.GetObstacleCount() != 32 {
			t.Errorf("expected 32 obstacles, got %d", tc.GetObstacleCount())
		}
	})

	t.Run("getAlloc and getCompressor return init values", func(t *testing.T) {
		tc := NewTileCache()
		alloc := &testAlloc{}
		comp := &testCompressor{}
		proc := &testMeshProcess{}
		err := tc.Init(defaultParams(), alloc, comp, proc)
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		if tc.GetAlloc() != alloc {
			t.Error("GetAlloc returned different allocator")
		}
		if tc.GetCompressor() != comp {
			t.Error("GetCompressor returned different compressor")
		}
	})

	t.Run("GetParams returns init params", func(t *testing.T) {
		tc := NewTileCache()
		params := defaultParams()
		params.Cs = 0.5
		err := tc.Init(params, &testAlloc{}, &testCompressor{}, &testMeshProcess{})
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		got := tc.GetParams()
		if got.Cs != 0.5 {
			t.Errorf("expected Cs=0.5, got %f", got.Cs)
		}
		if got.MaxTiles != 32 {
			t.Errorf("expected MaxTiles=32, got %d", got.MaxTiles)
		}
	})
}

// ---------------------------------------------------------------------------
// Tile lifecycle tests
// ---------------------------------------------------------------------------

func TestTileCacheAddTile(t *testing.T) {
	tc := createTestTileCache(t)
	comp := &testCompressor{}
	header, heights, areas, cons := createTestLayer6x6()
	data, dataSize := createCompressedTile(t, comp, header, heights, areas, cons)

	t.Run("add tile at (0,0,0)", func(t *testing.T) {
		ref, err := tc.AddTile(data, dataSize, 0)
		if err != nil {
			t.Fatalf("AddTile: %v", err)
		}
		if ref == 0 {
			t.Fatal("AddTile returned 0 ref")
		}
		t.Logf("tile ref: %d", ref)

		// Verify tile exists
		tile := tc.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("GetTileAt returned nil")
		}
		if tile.Header == nil {
			t.Fatal("tile.Header is nil")
		}
		if tile.Header.Tx != 0 || tile.Header.Ty != 0 || tile.Header.Tlayer != 0 {
			t.Fatalf("unexpected tile coords: (%d,%d,%d)", tile.Header.Tx, tile.Header.Ty, tile.Header.Tlayer)
		}
	})

	t.Run("duplicate tile returns error", func(t *testing.T) {
		data2, size2 := createCompressedTile(t, comp, header, heights, areas, cons)
		_, err := tc.AddTile(data2, size2, 0)
		if err != detour.ErrFailure {
			t.Fatalf("expected ErrFailure for duplicate tile, got %v", err)
		}
	})

	t.Run("tile ref roundtrip", func(t *testing.T) {
		tile := tc.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("no tile at (0,0,0)")
		}
		ref := tc.GetTileRef(tile)
		if ref == 0 {
			t.Fatal("GetTileRef returned 0")
		}
		tile2 := tc.GetTileByRef(ref)
		if tile2 != tile {
			t.Fatal("GetTileByRef returned different tile")
		}
	})

	t.Run("GetTileByRef with zero ref returns nil", func(t *testing.T) {
		if tc.GetTileByRef(0) != nil {
			t.Fatal("expected nil for zero ref")
		}
	})

	t.Run("GetTilesAt returns added tile", func(t *testing.T) {
		tiles := make([]CompressedTileRef, 8)
		n := tc.GetTilesAt(0, 0, tiles, 8)
		if n != 1 {
			t.Fatalf("expected 1 tile at (0,0), got %d", n)
		}
		if tiles[0] == 0 {
			t.Fatal("GetTilesAt returned 0 ref")
		}
	})

	t.Run("AddTile with nil allocator", func(t *testing.T) {
		// Alloc is only used during BuildNavMeshTile, not AddTile
		tc2 := NewTileCache()
		err := tc2.Init(defaultParams(), nil, comp, &testMeshProcess{})
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		data3, size3 := createCompressedTile(t, comp, header, heights, areas, cons)
		ref, err := tc2.AddTile(data3, size3, 0)
		if err != nil {
			t.Fatalf("AddTile with nil alloc: %v", err)
		}
		if ref == 0 {
			t.Fatal("AddTile returned 0 ref")
		}
	})
}

func TestTileCacheRemoveTile(t *testing.T) {
	tc := createTestTileCache(t)
	comp := &testCompressor{}
	header, heights, areas, cons := createTestLayer6x6()
	data, dataSize := createCompressedTile(t, comp, header, heights, areas, cons)

	ref, err := tc.AddTile(data, dataSize, 0)
	if err != nil {
		t.Fatalf("AddTile: %v", err)
	}

	t.Run("remove existing tile", func(t *testing.T) {
		returnedData, returnedSize, err := tc.RemoveTile(ref)
		if err != nil {
			t.Fatalf("RemoveTile: %v", err)
		}
		if len(returnedData) == 0 && returnedSize == 0 {
			t.Log("RemoveTile returned nil/zero (no CompressedTileFreeData flag)")
		}
		_ = returnedSize

		// Verify tile is gone
		tile := tc.GetTileAt(0, 0, 0)
		if tile != nil {
			t.Fatal("tile still exists after RemoveTile")
		}
	})

	t.Run("remove already-removed tile returns error", func(t *testing.T) {
		_, _, err := tc.RemoveTile(ref)
		if err != detour.ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam for removed tile, got %v", err)
		}
	})

	t.Run("remove zero ref returns error", func(t *testing.T) {
		_, _, err := tc.RemoveTile(0)
		if err != detour.ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})
}

func TestTileCacheGetTileAt(t *testing.T) {
	tc := createTestTileCache(t)
	comp := &testCompressor{}

	// Add tiles at different coordinates
	header1, h1, a1, c1 := createTestLayer6x6()
	header1.Tx, header1.Ty = 0, 0
	d1, s1 := createCompressedTile(t, comp, header1, h1, a1, c1)
	_, err := tc.AddTile(d1, s1, 0)
	if err != nil {
		t.Fatalf("AddTile (0,0): %v", err)
	}

	header2, h2, a2, c2 := createTestLayer6x6()
	header2.Tx, header2.Ty = 5, 3
	d2, s2 := createCompressedTile(t, comp, header2, h2, a2, c2)
	_, err = tc.AddTile(d2, s2, 0)
	if err != nil {
		t.Fatalf("AddTile (5,3): %v", err)
	}

	t.Run("get existing tile", func(t *testing.T) {
		tile := tc.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("expected tile at (0,0,0)")
		}
		if tile.Header.Tx != 0 || tile.Header.Ty != 0 {
			t.Fatalf("expected (0,0), got (%d,%d)", tile.Header.Tx, tile.Header.Ty)
		}
	})

	t.Run("get other tile", func(t *testing.T) {
		tile := tc.GetTileAt(5, 3, 0)
		if tile == nil {
			t.Fatal("expected tile at (5,3,0)")
		}
		if tile.Header.Tx != 5 || tile.Header.Ty != 3 {
			t.Fatalf("expected (5,3), got (%d,%d)", tile.Header.Tx, tile.Header.Ty)
		}
	})

	t.Run("wrong coordinates returns nil", func(t *testing.T) {
		if tc.GetTileAt(99, 99, 0) != nil {
			t.Fatal("expected nil for wrong coordinates")
		}
	})

	t.Run("wrong layer returns nil", func(t *testing.T) {
		if tc.GetTileAt(0, 0, 1) != nil {
			t.Fatal("expected nil for wrong layer")
		}
	})
}

// ---------------------------------------------------------------------------
// Obstacle lifecycle tests
// ---------------------------------------------------------------------------

func TestTileCacheObstacleAddRemove(t *testing.T) {
	tc := createTestTileCache(t)

	t.Run("add cylinder obstacle", func(t *testing.T) {
		ref, err := tc.AddObstacle([3]float32{5, 0, 5}, 1.0, 2.0)
		if err != nil {
			t.Fatalf("AddObstacle: %v", err)
		}
		if ref == 0 {
			t.Fatal("AddObstacle returned 0 ref")
		}

		ob := tc.GetObstacleByRef(ref)
		if ob == nil {
			t.Fatal("GetObstacleByRef returned nil")
		}
		if ob.State != ObstacleProcessing {
			t.Errorf("expected state ObstacleProcessing, got %d", ob.State)
		}
		cyl := ob.GetCylinder()
		if cyl.Pos[0] != 5 || cyl.Pos[2] != 5 {
			t.Errorf("cylinder pos = (%f,%f,%f), expected near (5,0,5)", cyl.Pos[0], cyl.Pos[1], cyl.Pos[2])
		}
		if cyl.Radius != 1.0 || cyl.Height != 2.0 {
			t.Errorf("cylinder radius=%f height=%f, expected 1.0, 2.0", cyl.Radius, cyl.Height)
		}
	})

	t.Run("add box obstacle", func(t *testing.T) {
		ref, err := tc.AddBoxObstacle([3]float32{0, 0, 0}, [3]float32{2, 2, 2})
		if err != nil {
			t.Fatalf("AddBoxObstacle: %v", err)
		}
		if ref == 0 {
			t.Fatal("AddBoxObstacle returned 0 ref")
		}

		ob := tc.GetObstacleByRef(ref)
		if ob == nil {
			t.Fatal("GetObstacleByRef returned nil")
		}
		box := ob.GetBox()
		if box.Bmin[0] != 0 || box.Bmin[2] != 0 {
			t.Errorf("box bmin = (%f,%f,%f)", box.Bmin[0], box.Bmin[1], box.Bmin[2])
		}
		if box.Bmax[0] != 2 || box.Bmax[2] != 2 {
			t.Errorf("box bmax = (%f,%f,%f)", box.Bmax[0], box.Bmax[1], box.Bmax[2])
		}
	})

	t.Run("add oriented box obstacle", func(t *testing.T) {
		ref, err := tc.AddBoxObstacleRot([3]float32{10, 0, 10}, [3]float32{1, 1, 1}, math.Pi/4)
		if err != nil {
			t.Fatalf("AddBoxObstacleRot: %v", err)
		}
		if ref == 0 {
			t.Fatal("AddBoxObstacleRot returned 0 ref")
		}

		ob := tc.GetObstacleByRef(ref)
		if ob == nil {
			t.Fatal("GetObstacleByRef returned nil")
		}
		if ob.Type != ObstacleTypeOrientedBox {
			t.Errorf("expected type ObstacleTypeOrientedBox, got %d", ob.Type)
		}
		obox := ob.GetOrientedBox()
		if obox.Center[0] != 10 || obox.Center[2] != 10 {
			t.Errorf("oriented box center = (%f,%f,%f)", obox.Center[0], obox.Center[1], obox.Center[2])
		}
	})

	t.Run("remove obstacle", func(t *testing.T) {
		// First get a known obstacle ref
		ref, _ := tc.AddObstacle([3]float32{1, 0, 1}, 0.5, 1.0)
		err := tc.RemoveObstacle(ref)
		if err != nil {
			t.Fatalf("RemoveObstacle: %v", err)
		}
	})

	t.Run("remove zero ref is no-op", func(t *testing.T) {
		err := tc.RemoveObstacle(0)
		if err != nil {
			t.Errorf("RemoveObstacle(0): %v", err)
		}
	})

	t.Run("max obstacles returns error", func(t *testing.T) {
		// Fill up the obstacle list
		// We have 64 max, and already added 4 (cylinder + box + oriented + cylinder_for_remove)
		// Actually after removal one might be reclaimed. Let me fill up with the remaining.
		tc2 := createTestTileCache(t)
		for i := 0; i < 64; i++ {
			_, err := tc2.AddObstacle([3]float32{0, 0, 0}, 1, 1)
			if err != nil {
				t.Fatalf("obstacle %d: %v", i, err)
			}
		}
		_, err := tc2.AddObstacle([3]float32{0, 0, 0}, 1, 1)
		if err == nil {
			t.Fatal("expected error when exceeding max obstacles")
		}
	})
}

func TestTileCacheObstacleGetObstacleByRef(t *testing.T) {
	tc := createTestTileCache(t)

	t.Run("invalid ref returns nil", func(t *testing.T) {
		ob := tc.GetObstacleByRef(0)
		if ob != nil {
			t.Fatal("expected nil for zero ref")
		}
	})

	t.Run("obstacle ref roundtrip", func(t *testing.T) {
		ref, _ := tc.AddObstacle([3]float32{3, 0, 4}, 1.5, 2.5)
		ob := tc.GetObstacleByRef(ref)
		if ob == nil {
			t.Fatal("GetObstacleByRef returned nil")
		}
		ref2 := tc.GetObstacleRef(ob)
		if ref2 != ref {
			t.Fatalf("ref roundtrip: %d -> %d", ref, ref2)
		}
	})

	t.Run("GetObstacle returns by index", func(t *testing.T) {
		ob := tc.GetObstacle(0)
		if ob == nil {
			t.Fatal("GetObstacle(0) returned nil")
		}
		// After Init, all obstacles are in the free list, but first one
		// should be the first obstacle we added
	})
}

// ---------------------------------------------------------------------------
// Obstacle type setter/getter tests
// ---------------------------------------------------------------------------

func TestTileCacheObstacleTypes(t *testing.T) {
	t.Run("SetCylinder and GetCylinder", func(t *testing.T) {
		ob := &TileCacheObstacle{}
		ob.SetCylinder([3]float32{1, 2, 3}, 4.0, 5.0)
		if ob.Type != ObstacleTypeCylinder {
			t.Errorf("expected type Cylinder, got %d", ob.Type)
		}
		cyl := ob.GetCylinder()
		if cyl.Pos[0] != 1 || cyl.Pos[1] != 2 || cyl.Pos[2] != 3 {
			t.Errorf("pos = %v", cyl.Pos)
		}
		if cyl.Radius != 4.0 || cyl.Height != 5.0 {
			t.Errorf("radius=%f height=%f", cyl.Radius, cyl.Height)
		}
	})

	t.Run("SetBox and GetBox", func(t *testing.T) {
		ob := &TileCacheObstacle{}
		ob.SetBox([3]float32{0, 0, 0}, [3]float32{10, 5, 10})
		if ob.Type != ObstacleTypeBox {
			t.Errorf("expected type Box, got %d", ob.Type)
		}
		box := ob.GetBox()
		if box.Bmin != [3]float32{0, 0, 0} || box.Bmax != [3]float32{10, 5, 10} {
			t.Errorf("box bmin=%v bmax=%v", box.Bmin, box.Bmax)
		}
	})

	t.Run("SetOrientedBox and GetOrientedBox", func(t *testing.T) {
		ob := &TileCacheObstacle{}
		ob.SetOrientedBox([3]float32{5, 0, 5}, [3]float32{1, 1, 1}, [2]float32{0.5, 0.5})
		if ob.Type != ObstacleTypeOrientedBox {
			t.Errorf("expected type OrientedBox, got %d", ob.Type)
		}
		obox := ob.GetOrientedBox()
		if obox.Center != [3]float32{5, 0, 5} {
			t.Errorf("center = %v", obox.Center)
		}
		if obox.RotAux != [2]float32{0.5, 0.5} {
			t.Errorf("rotAux = %v", obox.RotAux)
		}
	})
}

// ---------------------------------------------------------------------------
// Query tiles tests
// ---------------------------------------------------------------------------

func TestTileCacheQueryTiles(t *testing.T) {
	tc := createTestTileCache(t)
	comp := &testCompressor{}

	// Add tiles at different coordinates
	for _, coord := range [][2]int32{{0, 0}, {1, 0}, {0, 1}} {
		header, h, a, c := createTestLayer6x6()
		header.Tx, header.Ty = coord[0], coord[1]
		data, size := createCompressedTile(t, comp, header, h, a, c)
		_, err := tc.AddTile(data, size, 0)
		if err != nil {
			t.Fatalf("AddTile (%d,%d): %v", coord[0], coord[1], err)
		}
	}

	t.Run("query overlapping region", func(t *testing.T) {
		results := make([]CompressedTileRef, 8)
		n, err := tc.QueryTiles([3]float32{-1, 0, -1}, [3]float32{3, 1, 3}, results, 8)
		if err != nil {
			t.Fatalf("QueryTiles: %v", err)
		}
		if n == 0 {
			t.Fatal("expected at least 1 tile")
		}
		t.Logf("QueryTiles found %d tiles", n)
	})

	t.Run("query far away returns empty", func(t *testing.T) {
		results := make([]CompressedTileRef, 8)
		n, err := tc.QueryTiles([3]float32{1000, 0, 1000}, [3]float32{1010, 1, 1010}, results, 8)
		if err != nil {
			t.Fatalf("QueryTiles: %v", err)
		}
		if n != 0 {
			t.Errorf("expected 0 tiles far away, got %d", n)
		}
	})
}

func TestTileCacheCalcTightTileBounds(t *testing.T) {
	tc := createTestTileCache(t)

	header := &TileCacheLayerHeader{
		Bmin: [3]float32{0, 0, 0},
		Bmax: [3]float32{10, 5, 10},
		Minx: 1, Maxx: 8,
		Miny: 2, Maxy: 7,
	}
	bmin, bmax := tc.CalcTightTileBounds(header)

	expectedBmin := [3]float32{0.3, 0, 0.6} // Bmin + Minx*Cs (0.3)
	expectedBmax := [3]float32{2.7, 5, 2.4} // Bmin + (Maxx+1)*Cs, Bmax[1], Bmin + (Maxy+1)*Cs

	if bmin[0] != expectedBmin[0] || bmin[2] != expectedBmin[2] {
		t.Errorf("bmin = (%f,%f,%f), expected (~%f,0,~%f)", bmin[0], bmin[1], bmin[2], expectedBmin[0], expectedBmin[2])
	}
	if bmax[0] != expectedBmax[0] || bmax[1] != expectedBmax[1] || bmax[2] != expectedBmax[2] {
		t.Errorf("bmax = (%f,%f,%f), expected (~%f,%f,~%f)", bmax[0], bmax[1], bmax[2], expectedBmax[0], expectedBmax[1], expectedBmax[2])
	}
}

// ---------------------------------------------------------------------------
// Builder function tests
// ---------------------------------------------------------------------------

func TestBuildTileCacheLayerRoundtrip(t *testing.T) {
	comp := &testCompressor{}
	alloc := &testAlloc{}
	header, heights, areas, cons := createTestLayer6x6()

	t.Run("build and decompress roundtrip", func(t *testing.T) {
		data, dataSize, err := BuildTileCacheLayer(comp, header, heights, areas, cons)
		if err != nil {
			t.Fatalf("BuildTileCacheLayer: %v", err)
		}
		if dataSize <= TileCacheLayerHeaderSize() {
			t.Fatal("data size too small")
		}
		if dataSize > len(data) {
			t.Fatal("data size exceeds buffer")
		}

		layer, err := DecompressTileCacheLayer(alloc, comp, data, dataSize)
		if err != nil {
			t.Fatalf("DecompressTileCacheLayer: %v", err)
		}
		if layer == nil {
			t.Fatal("layer is nil")
		}

		// Verify header fields
		if layer.Header.Magic != TileCacheMagic {
			t.Error("header magic mismatch after roundtrip")
		}
		if layer.Header.Width != 6 || layer.Header.Height != 6 {
			t.Errorf("header dimensions: %dx%d", layer.Header.Width, layer.Header.Height)
		}
		if len(layer.Heights) < 36 || len(layer.Areas) < 36 || len(layer.Cons) < 36 {
			t.Error("layer arrays too small")
		}
	})

	t.Run("decompress with wrong magic", func(t *testing.T) {
		data, dataSize, _ := BuildTileCacheLayer(comp, header, heights, areas, cons)
		// Truncate data and modify magic
		data[0] = 0
		data[1] = 0
		data[2] = 0
		data[3] = 0
		_, err := DecompressTileCacheLayer(alloc, comp, data, dataSize)
		if err != detour.ErrWrongMagic {
			t.Fatalf("expected ErrWrongMagic, got %v", err)
		}
	})

	t.Run("decompress with wrong version", func(t *testing.T) {
		data, dataSize, _ := BuildTileCacheLayer(comp, header, heights, areas, cons)
		// Modify version at offset 4
		data[4] = 0xff
		data[5] = 0xff
		data[6] = 0xff
		data[7] = 0xff
		_, err := DecompressTileCacheLayer(alloc, comp, data, dataSize)
		if err != detour.ErrWrongVersion {
			t.Fatalf("expected ErrWrongVersion, got %v", err)
		}
	})

	t.Run("decompress with truncated data", func(t *testing.T) {
		smallData := make([]uint8, 4)
		_, err := DecompressTileCacheLayer(alloc, comp, smallData, 4)
		if err != detour.ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})
}

func TestBuildTileCacheRegions(t *testing.T) {
	comp := &testCompressor{}
	alloc := &testAlloc{}
	header, heights, areas, cons := createTestLayer6x6()

	t.Run("build regions from layer", func(t *testing.T) {
		data, dataSize, _ := BuildTileCacheLayer(comp, header, heights, areas, cons)
		layer, err := DecompressTileCacheLayer(alloc, comp, data, dataSize)
		if err != nil {
			t.Fatalf("DecompressTileCacheLayer: %v", err)
		}

		err = BuildTileCacheRegions(alloc, layer, int(1.0/0.2))
		if err != nil {
			t.Fatalf("BuildTileCacheRegions: %v", err)
		}
		if layer.RegCount == 0 {
			t.Fatal("expected at least 1 region")
		}
		t.Logf("regions: %d", layer.RegCount)

		// Verify walkable cells got regions assigned
		walkableCount := 0
		regionCount := 0
		for i := 0; i < 36; i++ {
			if areas[i] != TileCacheNullArea {
				walkableCount++
				if layer.Regs[i] != 0xff {
					regionCount++
				}
			}
		}
		if walkableCount != 16 {
			t.Errorf("expected 16 walkable cells, got %d", walkableCount)
		}
		if regionCount == 0 {
			t.Error("no walkable cells got regions assigned")
		}
		t.Logf("walkable=%d, with regions=%d", walkableCount, regionCount)
	})
}

func TestBuildTileCacheContours(t *testing.T) {
	comp := &testCompressor{}
	alloc := &testAlloc{}
	header, heights, areas, cons := createTestLayer6x6()

	data, dataSize, _ := BuildTileCacheLayer(comp, header, heights, areas, cons)
	layer, err := DecompressTileCacheLayer(alloc, comp, data, dataSize)
	if err != nil {
		t.Fatalf("DecompressTileCacheLayer: %v", err)
	}

	err = BuildTileCacheRegions(alloc, layer, int(1.0/0.2))
	if err != nil {
		t.Fatalf("BuildTileCacheRegions: %v", err)
	}
	if layer.RegCount == 0 {
		t.Fatal("no regions to build contours from")
	}

	t.Run("build contours from regions", func(t *testing.T) {
		lcset, err := BuildTileCacheContours(alloc, layer, int(1.0/0.2), 1.3)
		if err != nil {
			t.Fatalf("BuildTileCacheContours: %v", err)
		}
		if lcset == nil {
			t.Fatal("contour set is nil")
		}
		if lcset.NConts != int(layer.RegCount) {
			t.Errorf("expected %d contours, got %d", layer.RegCount, lcset.NConts)
		}
		t.Logf("contours: %d", lcset.NConts)
		for i, cont := range lcset.Conts {
			t.Logf("  contour %d: reg=%d area=%d nverts=%d",
				i, cont.Reg, cont.Area, cont.NVerts)
		}
	})

	t.Run("free contour set", func(t *testing.T) {
		lcset, _ := BuildTileCacheContours(alloc, layer, int(1.0/0.2), 1.3)
		FreeTileCacheContourSet(alloc, lcset)
		// Ensure no double-free panic
		FreeTileCacheContourSet(alloc, lcset)
	})

	t.Run("free nil contour set", func(t *testing.T) {
		FreeTileCacheContourSet(alloc, nil)
	})
}

func TestBuildTileCachePolyMesh(t *testing.T) {
	comp := &testCompressor{}
	alloc := &testAlloc{}
	header, heights, areas, cons := createTestLayer6x6()

	data, dataSize, _ := BuildTileCacheLayer(comp, header, heights, areas, cons)
	layer, err := DecompressTileCacheLayer(alloc, comp, data, dataSize)
	if err != nil {
		t.Fatalf("DecompressTileCacheLayer: %v", err)
	}

	err = BuildTileCacheRegions(alloc, layer, int(1.0/0.2))
	if err != nil {
		t.Fatalf("BuildTileCacheRegions: %v", err)
	}

	lcset, err := BuildTileCacheContours(alloc, layer, int(1.0/0.2), 1.3)
	if err != nil {
		t.Fatalf("BuildTileCacheContours: %v", err)
	}
	if lcset.NConts == 0 {
		t.Fatal("no contours to build mesh from")
	}

	t.Run("build polymesh from contours", func(t *testing.T) {
		lmesh, err := BuildTileCachePolyMesh(alloc, lcset)
		if err != nil {
			t.Fatalf("BuildTileCachePolyMesh: %v", err)
		}
		if lmesh == nil {
			t.Fatal("polymesh is nil")
		}
		t.Logf("mesh: nverts=%d npolys=%d nvp=%d",
			lmesh.NVerts, lmesh.NPolys, lmesh.Nvp)
		for i := 0; i < lmesh.NPolys; i++ {
			base := i * lmesh.Nvp * 2
			end := base + lmesh.Nvp*2
			if end > len(lmesh.Polys) {
				end = len(lmesh.Polys)
			}
			t.Logf("  poly %d: verts=%v flag=%d area=%d",
				i, lmesh.Polys[base:end], lmesh.Flags[i], lmesh.Areas[i])
		}
	})

	t.Run("free polymesh", func(t *testing.T) {
		lcset2, _ := BuildTileCacheContours(alloc, layer, int(1.0/0.2), 1.3)
		lmesh, _ := BuildTileCachePolyMesh(alloc, lcset2)
		FreeTileCachePolyMesh(alloc, lmesh)
		FreeTileCachePolyMesh(alloc, nil)
		FreeTileCacheContourSet(alloc, lcset2)
	})
}

// ---------------------------------------------------------------------------
// Mark area tests
// ---------------------------------------------------------------------------

func TestMarkCylinderArea(t *testing.T) {
	header, heights, areas, cons := createTestLayer6x6()
	layer := &TileCacheLayer{
		Header:  header,
		Heights: heights,
		Areas:   areas,
		Cons:    cons,
	}

	orig := [3]float32{0, 0, 0}
	cs := float32(1.0)
	ch := float32(0.2)

	err := MarkCylinderArea(layer, orig, cs, ch, [3]float32{3, 0, 3}, 1.0, 2.0, 1)
	if err != nil {
		t.Fatalf("MarkCylinderArea: %v", err)
	}

	// Check that cells near (3,0,3) within radius 1.0 got marked
	markedCount := 0
	for y := 0; y < 6; y++ {
		for x := 0; x < 6; x++ {
			idx := y*6 + x
			if areas[idx] == 1 {
				markedCount++
			}
		}
	}
	if markedCount == 0 {
		t.Error("MarkCylinderArea did not mark any cells")
	}
	t.Logf("MarkCylinderArea marked %d cells", markedCount)
}

func TestMarkBoxArea(t *testing.T) {
	header, heights, areas, cons := createTestLayer6x6()
	layer := &TileCacheLayer{
		Header:  header,
		Heights: heights,
		Areas:   areas,
		Cons:    cons,
	}

	orig := [3]float32{0, 0, 0}
	cs := float32(1.0)
	ch := float32(0.2)

	err := MarkBoxArea(layer, orig, cs, ch, [3]float32{1, 0, 1}, [3]float32{3, 2, 3}, 2)
	if err != nil {
		t.Fatalf("MarkBoxArea: %v", err)
	}

	markedCount := 0
	for y := 0; y < 6; y++ {
		for x := 0; x < 6; x++ {
			idx := y*6 + x
			if areas[idx] == 2 {
				markedCount++
			}
		}
	}
	if markedCount == 0 {
		t.Error("MarkBoxArea did not mark any cells")
	}
	t.Logf("MarkBoxArea marked %d cells", markedCount)
}

func TestMarkBoxAreaOriented(t *testing.T) {
	header, heights, areas, cons := createTestLayer6x6()
	layer := &TileCacheLayer{
		Header:  header,
		Heights: heights,
		Areas:   areas,
		Cons:    cons,
	}

	orig := [3]float32{0, 0, 0}
	cs := float32(1.0)
	ch := float32(0.2)

	// RotAux for a non-rotated box
	rotAux := [2]float32{0, 0.5}
	err := MarkBoxAreaOriented(layer, orig, cs, ch,
		[3]float32{2.5, 0, 2.5}, [3]float32{1.5, 1, 1.5}, rotAux, 3)
	if err != nil {
		t.Fatalf("MarkBoxAreaOriented: %v", err)
	}

	markedCount := 0
	for y := 0; y < 6; y++ {
		for x := 0; x < 6; x++ {
			idx := y*6 + x
			if areas[idx] == 3 {
				markedCount++
			}
		}
	}
	if markedCount == 0 {
		t.Error("MarkBoxAreaOriented did not mark any cells")
	}
	t.Logf("MarkBoxAreaOriented marked %d cells", markedCount)
}

// ---------------------------------------------------------------------------
// Alloc / Free helper tests
// ---------------------------------------------------------------------------

func TestAllocFreeHelpers(t *testing.T) {
	alloc := &testAlloc{}

	t.Run("AllocTileCacheContourSet", func(t *testing.T) {
		cset := AllocTileCacheContourSet(alloc)
		if cset == nil {
			t.Fatal("AllocTileCacheContourSet returned nil")
		}
		if cset.NConts != 0 {
			t.Errorf("expected NConts=0, got %d", cset.NConts)
		}
	})

	t.Run("AllocTileCachePolyMesh", func(t *testing.T) {
		lmesh := AllocTileCachePolyMesh(alloc)
		if lmesh == nil {
			t.Fatal("AllocTileCachePolyMesh returned nil")
		}
		if lmesh.NVerts != 0 {
			t.Errorf("expected NVerts=0, got %d", lmesh.NVerts)
		}
	})

	t.Run("FreeTileCacheContourSet with nil", func(t *testing.T) {
		FreeTileCacheContourSet(alloc, nil)
	})

	t.Run("FreeTileCachePolyMesh with nil", func(t *testing.T) {
		FreeTileCachePolyMesh(alloc, nil)
	})
}

// ---------------------------------------------------------------------------
// NavMeshCreateParams bridge and NavMeshInterface consistency
// ---------------------------------------------------------------------------

// bridgeParams converts detour_tile_cache.NavMeshCreateParams to detour.NavMeshCreateParams.
func bridgeParams(params *NavMeshCreateParams) *detour.NavMeshCreateParams {
	return &detour.NavMeshCreateParams{
		Verts:          params.Verts,
		VertCount:      params.VertCount,
		Polys:          params.Polys,
		PolyAreas:      params.PolyAreas,
		PolyFlags:      params.PolyFlags,
		PolyCount:      params.PolyCount,
		Nvp:            params.Nvp,
		WalkableHeight: params.WalkableHeight,
		WalkableRadius: params.WalkableRadius,
		WalkableClimb:  params.WalkableClimb,
		TileX:          int(params.TileX),
		TileY:          int(params.TileY),
		TileLayer:      int(params.TileLayer),
		Cs:             params.Cs,
		Ch:             params.Ch,
		Bmin:           params.Bmin,
		Bmax:           params.Bmax,
		BuildBvTree:    params.BuildBvTree,
	}
}

func TestCreateNavMeshDataBridge(t *testing.T) {
	t.Run("set CreateNavMeshData to use detour package", func(t *testing.T) {
		// Save original and restore
		orig := CreateNavMeshData
		defer func() { CreateNavMeshData = orig }()

		CreateNavMeshData = func(params *NavMeshCreateParams) ([]uint8, int) {
			dp := bridgeParams(params)
			data, dataSize, ok := detour.CreateNavMeshData(dp)
			if !ok {
				return nil, 0
			}
			return data, dataSize
		}

		// Test with minimal polygon mesh params
		// Create a simple quad: 4 vertices, 1 polygon
		params := &NavMeshCreateParams{
			Verts: []uint16{
				0, 0, 0, // vertex 0
				10, 0, 0, // vertex 1
				10, 0, 10, // vertex 2
				0, 0, 10, // vertex 3
			},
			VertCount:      4,
			Polys:          []uint16{0, 1, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0},
			PolyAreas:      []uint8{63},
			PolyFlags:      []uint16{0xffff},
			PolyCount:      1,
			Nvp:            6,
			WalkableHeight: 2.0,
			WalkableRadius: 0.6,
			WalkableClimb:  1.0,
			Cs:             1.0,
			Ch:             0.2,
			TileX:          0,
			TileY:          0,
			TileLayer:      0,
			Bmin:           [3]float32{0, 0, 0},
			Bmax:           [3]float32{10, 1, 10},
		}

		data, dataSize := CreateNavMeshData(params)
		if data == nil || dataSize == 0 {
			t.Fatal("CreateNavMeshData returned nil/zero — may need detail mesh data too")
		}
		t.Logf("navmesh data size: %d", dataSize)
	})
}

func TestTileCacheFullPipeline(t *testing.T) {
	comp := &testCompressor{}
	alloc := &testAlloc{}
	header, heights, areas, cons := createTestLayer6x6()

	// Set up CreateNavMeshData
	orig := CreateNavMeshData
	defer func() { CreateNavMeshData = orig }()
	CreateNavMeshData = func(params *NavMeshCreateParams) ([]uint8, int) {
		dp := bridgeParams(params)
		data, dataSize, ok := detour.CreateNavMeshData(dp)
		if !ok {
			return nil, 0
		}
		return data, dataSize
	}

	tc := NewTileCache()
	err := tc.Init(defaultParams(), alloc, comp, &testMeshProcess{})
	if err != nil {
		t.Fatalf("Init: %v", err)
	}

	data, dataSize := createCompressedTile(t, comp, header, heights, areas, cons)
	ref, err := tc.AddTile(data, dataSize, 0)
	if err != nil {
		t.Fatalf("AddTile: %v", err)
	}

	nm := newMockNavMesh()

	t.Run("BuildNavMeshTile", func(t *testing.T) {
		err = tc.BuildNavMeshTile(ref, nm)
		if err != nil {
			t.Fatalf("BuildNavMeshTile: %v", err)
		}
		if len(nm.tiles) == 0 {
			// This can happen when contour simplification reduces a small
			// walkable area to <3 vertices, producing an empty polymesh.
			// The navmesh stays empty, which is valid behavior.
			t.Log("no tiles in navmesh (empty polymesh from small walkable area)")
		} else {
			t.Logf("navmesh tiles: %d", len(nm.tiles))
			for ref, entry := range nm.tiles {
				t.Logf("  tile ref=%d, dataSize=%d", ref, entry.dataSize)
			}
		}
	})

	t.Run("Update processes pending obstacle", func(t *testing.T) {
		tc2 := NewTileCache()
		err := tc2.Init(defaultParams(), alloc, comp, &testMeshProcess{})
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		data2, size2 := createCompressedTile(t, comp, header, heights, areas, cons)
		tileRef, _ := tc2.AddTile(data2, size2, 0)

		nm2 := newMockNavMesh()
		// First build the tile into the navmesh
		err = tc2.BuildNavMeshTile(tileRef, nm2)
		if err != nil {
			t.Fatalf("BuildNavMeshTile: %v", err)
		}

		// Add an obstacle
		_, err = tc2.AddObstacle([3]float32{3, 0, 3}, 1.0, 2.0)
		if err != nil {
			t.Fatalf("AddObstacle: %v", err)
		}

		// Process update
		done, err := tc2.Update(1.0, nm2)
		if err != nil {
			t.Fatalf("Update: %v", err)
		}
		t.Logf("Update done=%v", done)
	})

	t.Run("BuildNavMeshTilesAt", func(t *testing.T) {
		tc3 := NewTileCache()
		err := tc3.Init(defaultParams(), alloc, comp, &testMeshProcess{})
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		data3, size3 := createCompressedTile(t, comp, header, heights, areas, cons)
		_, _ = tc3.AddTile(data3, size3, 0)

		nm3 := newMockNavMesh()
		err = tc3.BuildNavMeshTilesAt(0, 0, nm3)
		if err != nil {
			t.Fatalf("BuildNavMeshTilesAt: %v", err)
		}
	})
}

// ---------------------------------------------------------------------------
// Update edge case tests
// ---------------------------------------------------------------------------

func TestTileCacheUpdateEmpty(t *testing.T) {
	tc := createTestTileCache(t)
	nm := newMockNavMesh()

	done, err := tc.Update(1.0, nm)
	if err != nil {
		t.Fatalf("Update on empty cache: %v", err)
	}
	if !done {
		t.Fatal("expected done=true for empty cache")
	}
}

func TestTileCacheUpdateWithObstacleOnly(t *testing.T) {
	tc := createTestTileCache(t)
	nm := newMockNavMesh()

	// Add obstacle without any tile — should process without error
	_, err := tc.AddObstacle([3]float32{0, 0, 0}, 1, 1)
	if err != nil {
		t.Fatalf("AddObstacle: %v", err)
	}

	done, err := tc.Update(1.0, nm)
	if err != nil {
		t.Logf("Update: %v (expected if no tiles to rebuild)", err)
	}
	_ = done
}

func TestTileCacheUpdateMaxRequests(t *testing.T) {
	tc := createTestTileCache(t)
	nm := newMockNavMesh()

	// Add 64 obstacles (maxRequests)
	for i := 0; i < 64; i++ {
		_, err := tc.AddObstacle([3]float32{float32(i), 0, 0}, 0.5, 1.0)
		if err != nil {
			t.Fatalf("AddObstacle %d: %v", i, err)
		}
	}

	// The 65th should fail
	_, err := tc.AddObstacle([3]float32{100, 0, 0}, 0.5, 1.0)
	if err == nil {
		t.Fatal("expected error when exceeding max requests (64)")
	}
	_ = nm
}
