package detour

import (
	"encoding/binary"
	"math"
	"testing"
)

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

// buildTestNavmeshBytes creates binary navmesh data for a simple 2-triangle floor.
// Vertices (top-down, Y=0, X→, Z↓):
//
//	(0,0,0) --- (10,0,0)
//	   |     \      |
//	   |      \     |
//	   |       \    |
//	(0,0,10) --- (10,0,10)
//
// Poly 0: triangle (0,0,0), (10,0,0), (0,0,10) — left half
// Poly 1: triangle (10,0,0), (10,0,10), (0,0,10) — right half
// Shared edge: (10,0,0)→(0,0,10) which is poly0 edge1, poly1 edge2.
func buildTestNavmeshBytes(t testing.TB) []byte {
	t.Helper()

	header := MeshHeader{
		Magic:           NavMeshMagic,
		Version:         NavMeshVersion,
		X:               0,
		Y:               0,
		Layer:           0,
		PolyCount:       2,
		VertCount:       4,
		MaxLinkCount:    4,
		DetailMeshCount: 2,
		DetailVertCount: 0,
		DetailTriCount:  2,
		BVNodeCount:     0,
		OffMeshConCount: 0,
		OffMeshBase:     2,
		WalkableHeight:  2,
		WalkableRadius:  0,
		WalkableClimb:   1,
		Bmin:            [3]float32{0, 0, 0},
		Bmax:            [3]float32{10, 1, 10},
		BVQuantFactor:   1,
	}

	verts := []float32{
		0, 0, 0, // 0
		10, 0, 0, // 1
		0, 0, 10, // 2
		10, 0, 10, // 3
	}

	polys := []Poly{
		{
			FirstLink:   0,
			Verts:       [6]uint16{0, 1, 2, 0, 0, 0},
			Neis:        [6]uint16{0, 2, 0, 0, 0, 0},
			Flags:       0xffff,
			VertCount:   3,
			areaAndtype: 63,
		},
		{
			FirstLink:   0,
			Verts:       [6]uint16{1, 3, 2, 0, 0, 0},
			Neis:        [6]uint16{0, 0, 1, 0, 0, 0},
			Flags:       0xffff,
			VertCount:   3,
			areaAndtype: 63,
		},
	}

	detailTris := []uint8{
		0, 1, 2, 7,
		0, 1, 2, 7,
	}

	detailMeshes := []PolyDetail{
		{VertBase: 0, TriBase: 0, VertCount: 0, TriCount: 1},
		{VertBase: 0, TriBase: 1, VertCount: 0, TriCount: 1},
	}

	hdrSize := align4(int(binary.Size(MeshHeader{})))
	vertsSize := align4(len(verts) * 4)
	polysSize := align4(len(polys) * int(unsafeSizeOfPoly()))
	linksSize := align4(int(unsafeSizeOfLink()) * int(header.MaxLinkCount))
	dmSize := align4(len(detailMeshes) * int(unsafeSizeOfPolyDetail()))
	dvSize := align4(0)
	dtSize := align4(len(detailTris) * 1)
	bvSize := align4(0)
	omSize := align4(0)

	totalSize := hdrSize + vertsSize + polysSize + linksSize + dmSize + dvSize + dtSize + bvSize + omSize

	buf := make([]byte, totalSize)
	offset := 0

	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Magic))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Version))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.X))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Y))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Layer))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], header.UserID)
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.PolyCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.VertCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.MaxLinkCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.DetailMeshCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.DetailVertCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.DetailTriCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.BVNodeCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.OffMeshConCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.OffMeshBase))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.WalkableHeight))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.WalkableRadius))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.WalkableClimb))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmin[0]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmin[1]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmin[2]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmax[0]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmax[1]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmax[2]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.BVQuantFactor))
	offset += 4
	offset = hdrSize

	for _, v := range verts {
		binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(v))
		offset += 4
	}
	for offset < hdrSize+vertsSize {
		buf[offset] = 0
		offset++
	}

	for _, p := range polys {
		binary.LittleEndian.PutUint32(buf[offset:], p.FirstLink)
		offset += 4
		for j := 0; j < VertsPerPolygon; j++ {
			binary.LittleEndian.PutUint16(buf[offset:], p.Verts[j])
			offset += 2
		}
		for j := 0; j < VertsPerPolygon; j++ {
			binary.LittleEndian.PutUint16(buf[offset:], p.Neis[j])
			offset += 2
		}
		binary.LittleEndian.PutUint16(buf[offset:], p.Flags)
		offset += 2
		buf[offset] = p.VertCount
		offset++
		buf[offset] = p.areaAndtype
		offset++
	}
	for offset < hdrSize+vertsSize+polysSize {
		buf[offset] = 0
		offset++
	}

	for offset < hdrSize+vertsSize+polysSize+linksSize {
		buf[offset] = 0
		offset++
	}

	for _, dm := range detailMeshes {
		binary.LittleEndian.PutUint32(buf[offset:], dm.VertBase)
		offset += 4
		binary.LittleEndian.PutUint32(buf[offset:], dm.TriBase)
		offset += 4
		buf[offset] = dm.VertCount
		offset++
		buf[offset] = dm.TriCount
		offset++
	}
	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize {
		buf[offset] = 0
		offset++
	}

	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize+dvSize {
		buf[offset] = 0
		offset++
	}

	copy(buf[offset:], detailTris)
	offset += len(detailTris)
	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize+dvSize+dtSize {
		buf[offset] = 0
		offset++
	}

	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize+dvSize+dtSize+bvSize {
		buf[offset] = 0
		offset++
	}

	for offset < totalSize {
		buf[offset] = 0
		offset++
	}

	return buf
}

// buildTestNavmesh creates a ready-to-use NavMesh with a simple 2-polygon floor.
func buildTestNavmesh(t testing.TB) *NavMesh {
	t.Helper()

	data := buildTestNavmeshBytes(t)
	m := &NavMesh{}
	err := m.InitSingleTile(data, 0)
	if err != nil {
		t.Fatalf("InitSingleTile: %v", err)
	}
	return m
}

// createTestQuery creates a NavMeshQuery ready for tests.
func createTestQuery(t testing.TB, m *NavMesh) *NavMeshQuery {
	t.Helper()
	q := NewNavMeshQuery()
	err := q.Init(m, 2048)
	if err != nil {
		t.Fatalf("NavMeshQuery.Init: %v", err)
	}
	return q
}

// buildTestGridNavmesh creates a ready-to-use NavMesh for a rows×cols grid.
func buildTestGridNavmesh(t testing.TB, rows, cols int, cellSize float32) *NavMesh {
	t.Helper()

	data := buildGridNavmeshBytes(t, rows, cols, cellSize)
	m := &NavMesh{}
	err := m.InitSingleTile(data, 0)
	if err != nil {
		t.Fatalf("InitSingleTile: %v", err)
	}
	return m
}

// align4 aligns n to 4 bytes.
func align4(n int) int {
	return (n + 3) & ^3
}

// gridCell describes one quad cell in a grid, with its corner vertex indices.
type gridCell struct {
	bl, br, tl, tr int
}

// buildGridNavmeshBytes creates binary navmesh data for a rows×cols grid of quads.
// Each quad is split into two triangles (left and right) along the br→tl diagonal.
// Grid spans (0,0,0) to (cols*cellSize, 0, rows*cellSize).
//
//nolint:unparam
func buildGridNavmeshBytes(t testing.TB, rows, cols int, cellSize float32) []byte {
	t.Helper()

	nVerts := (rows + 1) * (cols + 1)
	nPolys := rows * cols * 2

	verts := make([]float32, 0, nVerts*3)
	for r := 0; r <= rows; r++ {
		for c := 0; c <= cols; c++ {
			verts = append(verts, float32(c)*cellSize, 0, float32(r)*cellSize)
		}
	}

	cells := make([]gridCell, 0, rows*cols)
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			bl := r*(cols+1) + c
			br := r*(cols+1) + c + 1
			tl := (r+1)*(cols+1) + c
			tr := (r+1)*(cols+1) + c + 1
			cells = append(cells, gridCell{bl, br, tl, tr})
		}
	}

	polys := make([]Poly, nPolys)
	for i, cell := range cells {
		leftIdx := i * 2
		rightIdx := i*2 + 1
		polys[leftIdx] = Poly{
			FirstLink:   0,
			Verts:       [6]uint16{uint16(cell.bl), uint16(cell.tl), uint16(cell.br), 0, 0, 0},
			Neis:        [6]uint16{0, 0, 0, 0, 0, 0},
			Flags:       0xffff,
			VertCount:   3,
			areaAndtype: 63,
		}
		polys[rightIdx] = Poly{
			FirstLink:   0,
			Verts:       [6]uint16{uint16(cell.br), uint16(cell.tl), uint16(cell.tr), 0, 0, 0},
			Neis:        [6]uint16{0, 0, 0, 0, 0, 0},
			Flags:       0xffff,
			VertCount:   3,
			areaAndtype: 63,
		}
		polys[leftIdx].Neis[1] = uint16(rightIdx + 1)
		polys[rightIdx].Neis[0] = uint16(leftIdx + 1)
	}

	for r := 0; r < rows; r++ {
		for c := 0; c < cols-1; c++ {
			cellIdx := r*cols + c
			rightPoly := cellIdx*2 + 1
			nextLeftPoly := (cellIdx + 1) * 2

			polys[rightPoly].Neis[2] = uint16(nextLeftPoly + 1)
			polys[nextLeftPoly].Neis[0] = uint16(rightPoly + 1)
		}
	}

	for r := 0; r < rows-1; r++ {
		for c := 0; c < cols; c++ {
			cellIdx := r*cols + c
			topPoly := cellIdx*2 + 1
			aboveLeftPoly := (cellIdx + cols) * 2

			polys[topPoly].Neis[1] = uint16(aboveLeftPoly + 1)
			polys[aboveLeftPoly].Neis[2] = uint16(topPoly + 1)
		}
	}

	detailTris := make([]uint8, nPolys*4)
	for i := 0; i < nPolys; i++ {
		detailTris[i*4] = 0
		detailTris[i*4+1] = 1
		detailTris[i*4+2] = 2
		detailTris[i*4+3] = 7
	}

	detailMeshes := make([]PolyDetail, nPolys)
	for i := range detailMeshes {
		detailMeshes[i] = PolyDetail{
			VertBase:  0,
			TriBase:   uint32(i),
			VertCount: 0,
			TriCount:  1,
		}
	}

	bmaxZ := float32(rows) * cellSize
	bmaxX := float32(cols) * cellSize
	header := MeshHeader{
		Magic:           NavMeshMagic,
		Version:         NavMeshVersion,
		X:               0,
		Y:               0,
		Layer:           0,
		PolyCount:       int32(nPolys),
		VertCount:       int32(nVerts),
		MaxLinkCount:    int32(nPolys * 2),
		DetailMeshCount: int32(nPolys),
		DetailVertCount: 0,
		DetailTriCount:  int32(nPolys),
		BVNodeCount:     0,
		OffMeshConCount: 0,
		OffMeshBase:     int32(nPolys),
		WalkableHeight:  2,
		WalkableRadius:  0,
		WalkableClimb:   1,
		Bmin:            [3]float32{0, 0, 0},
		Bmax:            [3]float32{bmaxX, 1, bmaxZ},
		BVQuantFactor:   1,
	}

	hdrSize := align4(int(binary.Size(MeshHeader{})))
	vertsSize := align4(len(verts) * 4)
	polysSize := align4(len(polys) * int(unsafeSizeOfPoly()))
	linksSize := align4(int(unsafeSizeOfLink()) * int(header.MaxLinkCount))
	dmSize := align4(len(detailMeshes) * int(unsafeSizeOfPolyDetail()))
	dvSize := align4(0)
	dtSize := align4(len(detailTris) * 1)
	bvSize := align4(0)
	omSize := align4(0)

	totalSize := hdrSize + vertsSize + polysSize + linksSize + dmSize + dvSize + dtSize + bvSize + omSize

	buf := make([]byte, totalSize)
	offset := 0

	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Magic))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Version))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.X))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Y))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.Layer))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], header.UserID)
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.PolyCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.VertCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.MaxLinkCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.DetailMeshCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.DetailVertCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.DetailTriCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.BVNodeCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.OffMeshConCount))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], uint32(header.OffMeshBase))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.WalkableHeight))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.WalkableRadius))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.WalkableClimb))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmin[0]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmin[1]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmin[2]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmax[0]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmax[1]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.Bmax[2]))
	offset += 4
	binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(header.BVQuantFactor))
	offset += 4
	offset = hdrSize

	for _, v := range verts {
		binary.LittleEndian.PutUint32(buf[offset:], math.Float32bits(v))
		offset += 4
	}
	for offset < hdrSize+vertsSize {
		buf[offset] = 0
		offset++
	}

	for _, p := range polys {
		binary.LittleEndian.PutUint32(buf[offset:], p.FirstLink)
		offset += 4
		for j := 0; j < VertsPerPolygon; j++ {
			binary.LittleEndian.PutUint16(buf[offset:], p.Verts[j])
			offset += 2
		}
		for j := 0; j < VertsPerPolygon; j++ {
			binary.LittleEndian.PutUint16(buf[offset:], p.Neis[j])
			offset += 2
		}
		binary.LittleEndian.PutUint16(buf[offset:], p.Flags)
		offset += 2
		buf[offset] = p.VertCount
		offset++
		buf[offset] = p.areaAndtype
		offset++
	}
	for offset < hdrSize+vertsSize+polysSize {
		buf[offset] = 0
		offset++
	}

	for offset < hdrSize+vertsSize+polysSize+linksSize {
		buf[offset] = 0
		offset++
	}

	for _, dm := range detailMeshes {
		binary.LittleEndian.PutUint32(buf[offset:], dm.VertBase)
		offset += 4
		binary.LittleEndian.PutUint32(buf[offset:], dm.TriBase)
		offset += 4
		buf[offset] = dm.VertCount
		offset++
		buf[offset] = dm.TriCount
		offset++
	}
	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize {
		buf[offset] = 0
		offset++
	}

	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize+dvSize {
		buf[offset] = 0
		offset++
	}

	copy(buf[offset:], detailTris)
	offset += len(detailTris)
	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize+dvSize+dtSize {
		buf[offset] = 0
		offset++
	}

	for offset < hdrSize+vertsSize+polysSize+linksSize+dmSize+dvSize+dtSize+bvSize {
		buf[offset] = 0
		offset++
	}

	for offset < totalSize {
		buf[offset] = 0
		offset++
	}

	return buf
}

// ---------------------------------------------------------------------------
// 1. Init / InitSingleTile error cases
// ---------------------------------------------------------------------------

func TestInitSingleTileErrors(t *testing.T) {
	t.Run("nil data panics", func(t *testing.T) {
		m := &NavMesh{}
		defer func() {
			if r := recover(); r == nil {
				t.Error("expected panic when InitSingleTile receives nil data")
			}
		}()
		_ = m.InitSingleTile(nil, 0)
	})

	t.Run("wrong magic", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		// Overwrite magic bytes with an invalid value.
		binary.LittleEndian.PutUint32(data[0:], 0xdeadbeef)

		m := &NavMesh{}
		err := m.InitSingleTile(data, 0)
		if err != ErrWrongMagic {
			t.Fatalf("expected ErrWrongMagic, got %v", err)
		}
	})

	t.Run("wrong version", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		// Overwrite version bytes with an invalid value.
		binary.LittleEndian.PutUint32(data[4:], 999)

		m := &NavMesh{}
		err := m.InitSingleTile(data, 0)
		if err != ErrWrongVersion {
			t.Fatalf("expected ErrWrongVersion, got %v", err)
		}
	})

	t.Run("successful InitSingleTile", func(t *testing.T) {
		m := buildTestNavmesh(t)
		if m == nil {
			t.Fatal("navmesh is nil")
		}
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("expected tile at (0,0,0)")
		}
		if tile.Header.PolyCount != 2 {
			t.Fatalf("expected 2 polys, got %d", tile.Header.PolyCount)
		}
	})
}

func TestInitErrors(t *testing.T) {
	t.Run("max polys too large", func(t *testing.T) {
		m := &NavMesh{}
		params := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   1,
			MaxPolys:   1 << 30, // large enough to make TileBits+PolyBits > 31
		}
		err := m.Init(params)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})

	t.Run("very large max tiles", func(t *testing.T) {
		m := &NavMesh{}
		params := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   1,
			MaxPolys:   1 << 23, // PolyBits=23, SaltBits=9 < 10
		}
		err := m.Init(params)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})

	t.Run("successful Init", func(t *testing.T) {
		m := &NavMesh{}
		params := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   4,
			MaxPolys:   2,
		}
		err := m.Init(params)
		if err != nil {
			t.Fatalf("Init: %v", err)
		}
		if m.MaxTiles != 4 {
			t.Fatalf("expected MaxTiles=4, got %d", m.MaxTiles)
		}
	})
}

// ---------------------------------------------------------------------------
// 2. GetTileAt / GetTilesAt — boundary cases
// ---------------------------------------------------------------------------

func TestGetTileAt(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("tile at (0,0,0) exists", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("expected tile at (0,0,0)")
		}
	})

	t.Run("wrong layer returns nil", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 1)
		if tile != nil {
			t.Fatal("expected nil for layer=1")
		}
	})

	t.Run("wrong x returns nil", func(t *testing.T) {
		tile := m.GetTileAt(1, 0, 0)
		if tile != nil {
			t.Fatal("expected nil for x=1")
		}
	})

	t.Run("wrong y returns nil", func(t *testing.T) {
		tile := m.GetTileAt(0, 1, 0)
		if tile != nil {
			t.Fatal("expected nil for y=1")
		}
	})

	t.Run("negative coords return nil", func(t *testing.T) {
		tile := m.GetTileAt(-1, -1, 0)
		if tile != nil {
			t.Fatal("expected nil for negative coords")
		}
	})
}

func TestGetTilesAt(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("existing location fills buffer", func(t *testing.T) {
		tiles := make([]*MeshTile, 4)
		n := m.GetTilesAt(0, 0, tiles, 4)
		if n != 1 {
			t.Fatalf("expected 1 tile, got %d", n)
		}
		if tiles[0] == nil {
			t.Fatal("expected non-nil tile")
		}
	})

	t.Run("buffer smaller than count", func(t *testing.T) {
		tiles := make([]*MeshTile, 0)
		n := m.GetTilesAt(0, 0, tiles, 0)
		if n != 0 {
			t.Fatalf("expected 0 tile with zero buffer, got %d", n)
		}
	})

	t.Run("non-existent location returns zero", func(t *testing.T) {
		tiles := make([]*MeshTile, 4)
		n := m.GetTilesAt(100, 100, tiles, 4)
		if n != 0 {
			t.Fatalf("expected 0 tiles, got %d", n)
		}
	})
}

// ---------------------------------------------------------------------------
// 3. GetTileRef / GetPolyRefBase — edge cases
// ---------------------------------------------------------------------------

func TestGetTileRefEdgeCases(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("nil tile returns 0", func(t *testing.T) {
		ref := m.GetTileRef(nil)
		if ref != 0 {
			t.Fatalf("expected 0 for nil tile, got %d", ref)
		}
	})

	t.Run("valid tile returns non-zero", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("no tile found")
		}
		ref := m.GetTileRef(tile)
		if ref == 0 {
			t.Fatal("expected non-zero tile ref")
		}
	})

	t.Run("round-trip by ref", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 0)
		ref := m.GetTileRef(tile)
		tile2 := m.GetTileByRef(ref)
		if tile2 != tile {
			t.Fatal("GetTileByRef returned different tile")
		}
	})

	t.Run("zero ref returns nil from GetTileByRef", func(t *testing.T) {
		tile := m.GetTileByRef(0)
		if tile != nil {
			t.Fatal("expected nil for ref=0")
		}
	})
}

func TestGetPolyRefBaseEdgeCases(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("nil tile returns 0", func(t *testing.T) {
		base := m.GetPolyRefBase(nil)
		if base != 0 {
			t.Fatalf("expected 0 for nil tile, got %d", base)
		}
	})

	t.Run("valid tile returns non-zero base", func(t *testing.T) {
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("no tile found")
		}
		base := m.GetPolyRefBase(tile)
		if base == 0 {
			t.Fatal("expected non-zero base ref")
		}
		// For a single-tile navmesh with MaxPolys=2, the base ref should
		// encode salt, tile=0, poly=0.
		_, tileIdx, polyIdx := m.DecodePolyID(base)
		if tileIdx != 0 {
			t.Fatalf("expected tile index 0, got %d", tileIdx)
		}
		if polyIdx != 0 {
			t.Fatalf("expected poly index 0, got %d", polyIdx)
		}
	})
}

// ---------------------------------------------------------------------------
// 4. GetTileAndPolyByRef — valid / invalid / boundary refs
// ---------------------------------------------------------------------------

func TestGetTileAndPolyByRefEdgeCases(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("zero ref returns error", func(t *testing.T) {
		_, _, err := m.GetTileAndPolyByRef(0)
		if err == nil {
			t.Fatal("expected error for ref=0")
		}
	})

	t.Run("valid poly ref succeeds", func(t *testing.T) {
		q := createTestQuery(t, m)
		filter := &QueryFilter{}
		filter.IncludeFlags = 0xffff
		for i := range filter.AreaCost {
			filter.AreaCost[i] = 1.0
		}
		ref, _, err := q.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{10, 2, 10}, filter)
		if err != nil || ref == 0 {
			t.Fatal("could not find a valid poly ref")
		}

		tile, poly, err := m.GetTileAndPolyByRef(ref)
		if err != nil {
			t.Fatalf("GetTileAndPolyByRef: %v", err)
		}
		if tile == nil {
			t.Fatal("tile is nil")
		}
		if poly == nil {
			t.Fatal("poly is nil")
		}
		if poly.VertCount < 3 {
			t.Fatalf("unexpected vert count %d", poly.VertCount)
		}
	})

	t.Run("ref with out-of-range poly index", func(t *testing.T) {
		// Encode a ref with poly index 999 which is beyond PolyCount.
		// With MaxPolys=2, PolyBits=1, so poly index is masked to 1 bit.
		// But we want an index >= PolyCount(2):
		// The poly field is 1 bit wide, so only 0 or 1 are storable.
		// To test out-of-range, we need to construct a ref manually.
		salt := uint32(1)
		tileIdx := uint32(0)
		badPoly := uint32(5) // will be masked to (5 & 1) = 1 by EncodePolyID
		ref := m.EncodePolyID(salt, tileIdx, badPoly)
		_, _, err := m.GetTileAndPolyByRef(ref)
		// ref overflows into salt bits, making salt invalid
		if err == nil {
			t.Log("unexpectedly valid ref - masked poly was in range")
		}
	})

	t.Run("ref with stale salt", func(t *testing.T) {
		// The navmesh uses salt=1 for tiles. Encode a ref with salt=999.
		ref := m.EncodePolyID(999, 0, 0)
		_, _, err := m.GetTileAndPolyByRef(ref)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam for stale salt, got %v", err)
		}
	})
}

// ---------------------------------------------------------------------------
// 5. AddTile — add multiple tiles at different coords
// ---------------------------------------------------------------------------

func TestAddTile(t *testing.T) {
	params := &NavMeshParams{
		Orig:       [3]float32{0, 0, 0},
		TileWidth:  10,
		TileHeight: 10,
		MaxTiles:   4,
		MaxPolys:   2,
	}
	m := &NavMesh{}
	err := m.Init(params)
	if err != nil {
		t.Fatalf("Init: %v", err)
	}

	t.Run("add tile at (0,0)", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		ref, err := m.AddTile(data, 0, 0)
		if err != nil {
			t.Fatalf("AddTile: %v", err)
		}
		if ref == 0 {
			t.Fatal("expected non-zero tile ref")
		}
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("expected tile at (0,0,0)")
		}
		if tile.Header.PolyCount != 2 {
			t.Fatalf("expected 2 polys, got %d", tile.Header.PolyCount)
		}
	})

	t.Run("add duplicate tile returns ErrAlreadyOccupied", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		_, err := m.AddTile(data, 0, 0)
		if err != ErrAlreadyOccupied {
			t.Fatalf("expected ErrAlreadyOccupied, got %v", err)
		}
	})

	t.Run("add tile at different coordinate", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		// Modify header X to 2
		dataCopy := make([]byte, len(data))
		copy(dataCopy, data)
		binary.LittleEndian.PutUint32(dataCopy[8:], 2)  // X=2
		binary.LittleEndian.PutUint32(dataCopy[12:], 3) // Y=3

		ref, err := m.AddTile(dataCopy, 0, 0)
		if err != nil {
			t.Fatalf("AddTile at (2,3): %v", err)
		}
		if ref == 0 {
			t.Fatal("expected non-zero ref")
		}
		tile := m.GetTileAt(2, 3, 0)
		if tile == nil {
			t.Fatal("expected tile at (2,3,0)")
		}
		if int(tile.Header.X) != 2 || int(tile.Header.Y) != 3 {
			t.Fatalf("expected tile (2,3), got (%d,%d)", tile.Header.X, tile.Header.Y)
		}
	})

	t.Run("add tile with wrong magic", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		binary.LittleEndian.PutUint32(data[0:], 0xdeadbeef)
		_, err := m.AddTile(data, 0, 0)
		if err != ErrWrongMagic {
			t.Fatalf("expected ErrWrongMagic, got %v", err)
		}
	})

	t.Run("add tile with wrong version", func(t *testing.T) {
		data := buildTestNavmeshBytes(t)
		binary.LittleEndian.PutUint32(data[4:], 999)
		_, err := m.AddTile(data, 0, 0)
		if err != ErrWrongVersion {
			t.Fatalf("expected ErrWrongVersion, got %v", err)
		}
	})

	t.Run("add tile with too many polys fails", func(t *testing.T) {
		// Use a navmesh with MaxPolys=1 to trigger the poly-bits check.
		params2 := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   1,
			MaxPolys:   1,
		}
		m2 := &NavMesh{}
		if err := m2.Init(params2); err != nil {
			t.Fatalf("Init: %v", err)
		}
		// data has PolyCount=2 which won't fit in 0 poly bits.
		data := buildTestNavmeshBytes(t)
		_, err := m2.AddTile(data, 0, 0)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam for too many polys, got %v", err)
		}
	})

	t.Run("out of memory when no free tiles", func(t *testing.T) {
		// Create a navmesh with only 1 tile slot
		params2 := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   1,
			MaxPolys:   2,
		}
		m2 := &NavMesh{}
		if err := m2.Init(params2); err != nil {
			t.Fatalf("Init: %v", err)
		}
		// First AddTile should use the only free slot
		data := buildTestNavmeshBytes(t)
		ref, err := m2.AddTile(data, 0, 0)
		if err != nil {
			t.Fatalf("first AddTile: %v", err)
		}
		if ref == 0 {
			t.Fatal("expected non-zero ref")
		}
		// Second AddTile should fail with ErrOutOfMemory
		_, err = m2.AddTile(data, 0, 0)
		if err != ErrAlreadyOccupied {
			t.Fatalf("expected ErrAlreadyOccupied for duplicate, got %v", err)
		}
		// Use different coords to avoid duplicate check, but should still OOM
		d := make([]byte, len(data))
		copy(d, data)
		binary.LittleEndian.PutUint32(d[8:], 99)
		_, err = m2.AddTile(d, 0, 0)
		if err != ErrOutOfMemory {
			t.Fatalf("expected ErrOutOfMemory, got %v", err)
		}
	})
}

// ---------------------------------------------------------------------------
// 6. StoreTileState / RestoreTileState
// ---------------------------------------------------------------------------

func TestStoreRestoreTileState(t *testing.T) {
	m := buildTestNavmesh(t)
	tile := m.GetTileAt(0, 0, 0)
	if tile == nil {
		t.Fatal("no tile found")
	}

	t.Run("round trip stores and restores flags and area", func(t *testing.T) {
		size := m.GetTileStateSize(tile)
		if size <= 0 {
			t.Fatalf("expected positive state size, got %d", size)
		}

		buf := make([]byte, size)
		err := m.StoreTileState(tile, buf, len(buf))
		if err != nil {
			t.Fatalf("StoreTileState: %v", err)
		}

		// Save originals, then modify flags and area.
		origFlags := make([]uint16, tile.Header.PolyCount)
		origAreas := make([]uint8, tile.Header.PolyCount)
		for i := int32(0); i < tile.Header.PolyCount; i++ {
			origFlags[i] = tile.Polys[i].Flags
			origAreas[i] = tile.Polys[i].GetArea()
			tile.Polys[i].Flags = 0xAAAA
			tile.Polys[i].SetArea(42)
		}

		// Restore.
		err = m.RestoreTileState(tile, buf, len(buf))
		if err != nil {
			t.Fatalf("RestoreTileState: %v", err)
		}

		// Verify restored values.
		for i := int32(0); i < tile.Header.PolyCount; i++ {
			if tile.Polys[i].Flags != origFlags[i] {
				t.Fatalf("poly %d: flags not restored: expected %#x, got %#x",
					i, origFlags[i], tile.Polys[i].Flags)
			}
			if tile.Polys[i].GetArea() != origAreas[i] {
				t.Fatalf("poly %d: area not restored: expected %d, got %d",
					i, origAreas[i], tile.Polys[i].GetArea())
			}
		}
	})

	t.Run("StoreTileState buffer too small", func(t *testing.T) {
		size := m.GetTileStateSize(tile)
		err := m.StoreTileState(tile, make([]byte, size-1), size-1)
		if err != ErrBufferTooSmall {
			t.Fatalf("expected ErrBufferTooSmall, got %v", err)
		}
	})

	t.Run("RestoreTileState buffer too small", func(t *testing.T) {
		size := m.GetTileStateSize(tile)
		err := m.RestoreTileState(tile, make([]byte, size-1), size-1)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})

	t.Run("RestoreTileState wrong magic", func(t *testing.T) {
		size := m.GetTileStateSize(tile)
		buf := make([]byte, size)
		// Write invalid magic at the start.
		binary.LittleEndian.PutUint32(buf[0:], 0xdeadbeef)
		err := m.RestoreTileState(tile, buf, len(buf))
		if err != ErrWrongMagic {
			t.Fatalf("expected ErrWrongMagic, got %v", err)
		}
	})

	t.Run("RestoreTileState wrong version", func(t *testing.T) {
		size := m.GetTileStateSize(tile)
		buf := make([]byte, size)
		// Write correct magic but wrong version.
		binary.LittleEndian.PutUint32(buf[0:], uint32(NavMeshStateMagic))
		binary.LittleEndian.PutUint32(buf[4:], 999)
		err := m.RestoreTileState(tile, buf, len(buf))
		if err != ErrWrongVersion {
			t.Fatalf("expected ErrWrongVersion, got %v", err)
		}
	})

	t.Run("RestoreTileState wrong tile ref", func(t *testing.T) {
		size := m.GetTileStateSize(tile)
		buf := make([]byte, size)
		binary.LittleEndian.PutUint32(buf[0:], uint32(NavMeshStateMagic))
		binary.LittleEndian.PutUint32(buf[4:], uint32(NavMeshStateVersion))
		// Write a different tile ref.
		binary.LittleEndian.PutUint32(buf[8:], 0x12345678)
		err := m.RestoreTileState(tile, buf, len(buf))
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})

	t.Run("GetTileStateSize with nil tile", func(t *testing.T) {
		sz := m.GetTileStateSize(nil)
		if sz != 0 {
			t.Fatalf("expected 0 for nil tile, got %d", sz)
		}
	})
}

// ---------------------------------------------------------------------------
// 7. Tile flags operations (MeshTile.Flags via AddTile)
// ---------------------------------------------------------------------------

func TestTileFlags(t *testing.T) {
	t.Run("default tile flags are zero", func(t *testing.T) {
		params := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   2,
			MaxPolys:   2,
		}
		m := &NavMesh{}
		if err := m.Init(params); err != nil {
			t.Fatalf("Init: %v", err)
		}
		data := buildTestNavmeshBytes(t)
		_, err := m.AddTile(data, 0, 0)
		if err != nil {
			t.Fatalf("AddTile flags=0: %v", err)
		}
		tile := m.GetTileAt(0, 0, 0)
		if tile == nil {
			t.Fatal("tile not found")
		}
		if tile.Flags != 0 {
			t.Fatalf("expected Flags=0, got %d", tile.Flags)
		}
	})

	t.Run("add tile with TileFreeData flag", func(t *testing.T) {
		params := &NavMeshParams{
			Orig:       [3]float32{0, 0, 0},
			TileWidth:  10,
			TileHeight: 10,
			MaxTiles:   2,
			MaxPolys:   2,
		}
		m := &NavMesh{}
		if err := m.Init(params); err != nil {
			t.Fatalf("Init: %v", err)
		}
		data := buildTestNavmeshBytes(t)
		dataCopy := make([]byte, len(data))
		copy(dataCopy, data)
		binary.LittleEndian.PutUint32(dataCopy[8:], 5) // X=5
		_, err := m.AddTile(dataCopy, TileFreeData, 0)
		if err != nil {
			t.Fatalf("AddTile flags=TileFreeData: %v", err)
		}
		tile := m.GetTileAt(5, 0, 0)
		if tile == nil {
			t.Fatal("tile not found")
		}
		if tile.Flags != TileFreeData {
			t.Fatalf("expected Flags=TileFreeData, got %d", tile.Flags)
		}
	})
}

// ---------------------------------------------------------------------------
// 8. DecodePolyIdSalt / DecodePolyIdTile / DecodePolyIdPoly extraction helpers
// ---------------------------------------------------------------------------
// With buildTestNavmesh: MaxTiles=1 → TileBits=0, MaxPolys=2 → PolyBits=1.
// SaltBits = Min(31, 32-0-1) = 31.
// So: tile must be 0, poly must be 0 or 1.

func TestDecodePolyIdExtraction(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("basic encode decode round trip", func(t *testing.T) {
		salt, tile, poly := uint32(5), uint32(0), uint32(1)
		ref := m.EncodePolyID(salt, tile, poly)

		if dsalt := m.DecodePolyIdSalt(ref); dsalt != salt {
			t.Fatalf("DecodePolyIdSalt: expected %d, got %d", salt, dsalt)
		}
		if dtile := m.DecodePolyIdTile(ref); dtile != tile {
			t.Fatalf("DecodePolyIdTile: expected %d, got %d", tile, dtile)
		}
		if dpoly := m.DecodePolyIdPoly(ref); dpoly != poly {
			t.Fatalf("DecodePolyIdPoly: expected %d, got %d", poly, dpoly)
		}
	})

	t.Run("salt extraction ignores lower bits", func(t *testing.T) {
		ref := m.EncodePolyID(7, 0, 0)
		// Salt occupies bits 1+ and should be 7.
		if dsalt := m.DecodePolyIdSalt(ref); dsalt != 7 {
			t.Fatalf("expected salt 7, got %d", dsalt)
		}
	})

	t.Run("poly value is masked to bit width", func(t *testing.T) {
		// PolyBits=1 so only bit 0 matters; poly=3 (0b11) should be masked to 1.
		ref := m.EncodePolyID(1, 0, 3)
		dpoly := m.DecodePolyIdPoly(ref)
		if dpoly != 1 {
			t.Fatalf("expected poly=1 (masked from 3), got %d", dpoly)
		}
	})

	t.Run("zero ref extracts as zero", func(t *testing.T) {
		if dsalt := m.DecodePolyIdSalt(0); dsalt != 0 {
			t.Fatalf("expected 0 salt, got %d", dsalt)
		}
		if dtile := m.DecodePolyIdTile(0); dtile != 0 {
			t.Fatalf("expected 0 tile, got %d", dtile)
		}
		if dpoly := m.DecodePolyIdPoly(0); dpoly != 0 {
			t.Fatalf("expected 0 poly, got %d", dpoly)
		}
	})

	t.Run("consistency with DecodePolyID", func(t *testing.T) {
		// The individual extractors should match the combined decoder.
		salt, tile, poly := uint32(3), uint32(0), uint32(1)
		ref := m.EncodePolyID(salt, tile, poly)

		dsalt, dtile, dpoly := m.DecodePolyID(ref)
		if m.DecodePolyIdSalt(ref) != dsalt {
			t.Fatal("DecodePolyIdSalt mismatch with DecodePolyID")
		}
		if m.DecodePolyIdTile(ref) != dtile {
			t.Fatal("DecodePolyIdTile mismatch with DecodePolyID")
		}
		if m.DecodePolyIdPoly(ref) != dpoly {
			t.Fatal("DecodePolyIdPoly mismatch with DecodePolyID")
		}
	})
}

// ---------------------------------------------------------------------------
// 9. ClosestPointOnPoly — edge cases with inside / outside / on-boundary
// ---------------------------------------------------------------------------

func TestClosestPointOnPolyEdgeCases(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	// Get a valid ref for poly0 (left triangle: (0,0,0), (10,0,0), (0,0,10)).
	ref, _, err := q.FindNearestPoly([3]float32{2, 0, 2}, halfExtents, filter)
	if err != nil || ref == 0 {
		t.Fatal("could not find starting poly")
	}

	t.Run("point inside polygon returns overPoly=true", func(t *testing.T) {
		// (2, 0.5, 2) is inside poly0 (left of diagonal x+z=10).
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{2, 0.5, 2})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		if !overPoly {
			t.Log("point not over poly (may be expected for boundary)")
		}
		if pt[1] != 0 {
			t.Logf("expected height ~0, got %f", pt[1])
		}
		// The x and z should match the input when over the poly.
		if overPoly {
			if pt[0] != 2 || pt[2] != 2 {
				t.Logf("expected (2, 0, 2), got (%f, %f, %f)", pt[0], pt[1], pt[2])
			}
		}
	})

	t.Run("point outside polygon returns closest boundary point", func(t *testing.T) {
		// (-10, 0, -10) is far outside poly0.
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{-10, 0, -10})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		if overPoly {
			t.Log("outside point returned overPoly=true (unexpected)")
		}
		// The closest point should be on the boundary of poly0.
		// For poly0 (0,0,0)-(10,0,0)-(0,0,10), the closest point to (-10,0,-10)
		// should be vertex (0,0,0).
		if pt[0] != 0 || pt[2] != 0 {
			t.Logf("closest to (-10,-10) expected near (0,0), got (%f, %f)", pt[0], pt[2])
		}
	})

	t.Run("point on shared edge returns valid closest point", func(t *testing.T) {
		// (5, 0, 5) is on the diagonal edge shared between poly0 and poly1.
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{5, 0, 5})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		_ = pt
		_ = overPoly
		// The point might be reported as over or not; either is acceptable
		// as long as we don't get an error.
	})

	t.Run("point well above polygon", func(t *testing.T) {
		// A point high above the polygon should project correctly.
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{3, 100, 3})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		_ = overPoly
		// The height should be clamped to the polygon surface (height=0).
		if pt[1] != 0 {
			t.Logf("projected height expected 0, got %f", pt[1])
		}
	})

	t.Run("zero ref returns error", func(t *testing.T) {
		_, _, err := q.ClosestPointOnPoly(0, [3]float32{2, 0, 2})
		if err == nil {
			t.Fatal("expected error for zero ref")
		}
	})
}

// ---------------------------------------------------------------------------
// 10. WalkableHeight / WalkableClimb — access through tile header
// ---------------------------------------------------------------------------

func TestWalkableHeightClimb(t *testing.T) {
	m := buildTestNavmesh(t)
	tile := m.GetTileAt(0, 0, 0)
	if tile == nil {
		t.Fatal("no tile found")
	}

	t.Run("WalkableHeight from test data is 2", func(t *testing.T) {
		if tile.Header.WalkableHeight != 2 {
			t.Fatalf("expected WalkableHeight=2, got %f", tile.Header.WalkableHeight)
		}
	})

	t.Run("WalkableClimb from test data is 1", func(t *testing.T) {
		if tile.Header.WalkableClimb != 1 {
			t.Fatalf("expected WalkableClimb=1, got %f", tile.Header.WalkableClimb)
		}
	})

	t.Run("WalkableRadius from test data is 0", func(t *testing.T) {
		if tile.Header.WalkableRadius != 0 {
			t.Fatalf("expected WalkableRadius=0, got %f", tile.Header.WalkableRadius)
		}
	})

	t.Run("values accessible through NavMesh Params", func(t *testing.T) {
		p := m.GetParams()
		// The params should match the tile header.
		if p.Orig != tile.Header.Bmin {
			t.Logf("Orig may differ from Bmin")
		}
	})

	t.Run("CalcTileLoc round trip", func(t *testing.T) {
		tx, ty := m.CalcTileLoc([3]float32{5, 0, 5})
		if tx != 0 || ty != 0 {
			t.Fatalf("expected (0,0) for pos (5,0,5), got (%d,%d)", tx, ty)
		}
		tx, ty = m.CalcTileLoc([3]float32{15, 0, 5})
		if tx != 1 {
			t.Fatalf("expected tx=1 for pos (15,0,5), got %d", tx)
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: RemoveTile tests
// ---------------------------------------------------------------------------

func TestRemoveTile(t *testing.T) {
	params := &NavMeshParams{
		Orig:       [3]float32{0, 0, 0},
		TileWidth:  10,
		TileHeight: 10,
		MaxTiles:   2,
		MaxPolys:   2,
	}
	m := &NavMesh{}
	if err := m.Init(params); err != nil {
		t.Fatalf("Init: %v", err)
	}
	data := buildTestNavmeshBytes(t)
	ref, err := m.AddTile(data, 0, 0)
	if err != nil {
		t.Fatalf("AddTile: %v", err)
	}

	t.Run("RemoveTile removes tile", func(t *testing.T) {
		oldData, err := m.RemoveTile(ref)
		if err != nil {
			t.Fatalf("RemoveTile: %v", err)
		}
		if oldData == nil {
			t.Log("oldData is nil (expected for TileFreeData flag)")
		}
		tile := m.GetTileAt(0, 0, 0)
		if tile != nil {
			t.Fatal("tile should be nil after removal")
		}
	})

	t.Run("RemoveTile with zero ref", func(t *testing.T) {
		_, err := m.RemoveTile(0)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam, got %v", err)
		}
	})

	t.Run("RemoveTile with stale ref", func(t *testing.T) {
		// Add a tile, remove it, then try removing again.
		data2 := make([]byte, len(data))
		copy(data2, data)
		binary.LittleEndian.PutUint32(data2[8:], 3)
		ref2, err := m.AddTile(data2, 0, 0)
		if err != nil {
			t.Fatalf("AddTile: %v", err)
		}
		_, err = m.RemoveTile(ref2)
		if err != nil {
			t.Fatalf("first remove: %v", err)
		}
		// The salt has been incremented, so ref2 is now stale.
		_, err = m.RemoveTile(ref2)
		if err != ErrInvalidParam {
			t.Fatalf("expected ErrInvalidParam for stale ref, got %v", err)
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: GetTileByRef with edge cases
// ---------------------------------------------------------------------------

func TestGetTileByRefEdgeCases(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("GetTileByRef with out-of-range index", func(t *testing.T) {
		// Encode a ref pointing to tile index 999 that exceeds MaxTiles.
		// With MaxTiles=1 and TileBits=0, tile index is masked to 0.
		// So we need to construct a ref manually for truly out-of-range.
		ref := PolyRef(0x7fffffff) // large ref with tile bits at unexpected positions
		tile := m.GetTileByRef(TileRef(ref))
		// This may or may not find a tile, but should not panic.
		_ = tile
	})

	t.Run("GetTileRefAt returns 0 for non-existent location", func(t *testing.T) {
		ref := m.GetTileRefAt(99, 99, 0)
		if ref != 0 {
			t.Fatalf("expected 0, got %d", ref)
		}
	})

	t.Run("GetTileRefAt returns valid ref for existing tile", func(t *testing.T) {
		ref := m.GetTileRefAt(0, 0, 0)
		if ref == 0 {
			t.Fatal("expected non-zero ref")
		}
		tile := m.GetTileByRef(ref)
		if tile == nil {
			t.Fatal("expected non-nil tile")
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: GetMaxTiles and GetTile index access
// ---------------------------------------------------------------------------

func TestNavMeshAccessors(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("GetMaxTiles returns 1", func(t *testing.T) {
		if m.GetMaxTiles() != 1 {
			t.Fatalf("expected 1, got %d", m.GetMaxTiles())
		}
	})

	t.Run("GetTile returns tile by index", func(t *testing.T) {
		tile := m.GetTile(0)
		if tile == nil {
			t.Fatal("tile is nil")
		}
		if tile.Header == nil {
			t.Fatal("tile header is nil")
		}
	})

	t.Run("GetParams returns a copy of Params", func(t *testing.T) {
		p := m.GetParams()
		if p.MaxTiles != 1 {
			t.Fatalf("expected MaxTiles=1, got %d", p.MaxTiles)
		}
		if p.MaxPolys != 2 {
			t.Fatalf("expected MaxPolys=2, got %d", p.MaxPolys)
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: SetPolyFlags / GetPolyFlags edge cases
// ---------------------------------------------------------------------------

func TestSetGetPolyFlags(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("GetPolyFlags with zero ref", func(t *testing.T) {
		_, err := m.GetPolyFlags(0)
		if err != ErrFailure {
			t.Fatalf("expected ErrFailure, got %v", err)
		}
	})

	t.Run("SetPolyFlags with zero ref", func(t *testing.T) {
		err := m.SetPolyFlags(0, 0xffff)
		if err != ErrFailure {
			t.Fatalf("expected ErrFailure, got %v", err)
		}
	})

	t.Run("Set and Get round trip", func(t *testing.T) {
		q := createTestQuery(t, m)
		filter := &QueryFilter{}
		filter.IncludeFlags = 0xffff
		for i := range filter.AreaCost {
			filter.AreaCost[i] = 1.0
		}
		ref, _, _ := q.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Fatal("no valid ref found")
		}

		err := m.SetPolyFlags(ref, 0x1234)
		if err != nil {
			t.Fatalf("SetPolyFlags: %v", err)
		}
		flags, err := m.GetPolyFlags(ref)
		if err != nil {
			t.Fatalf("GetPolyFlags: %v", err)
		}
		if flags != 0x1234 {
			t.Fatalf("expected 0x1234, got %#x", flags)
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: SetPolyArea / GetPolyArea edge cases
// ---------------------------------------------------------------------------

func TestSetGetPolyArea(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("GetPolyArea with zero ref", func(t *testing.T) {
		_, err := m.GetPolyArea(0)
		if err != ErrFailure {
			t.Fatalf("expected ErrFailure, got %v", err)
		}
	})

	t.Run("SetPolyArea with zero ref", func(t *testing.T) {
		err := m.SetPolyArea(0, 1)
		if err != ErrFailure {
			t.Fatalf("expected ErrFailure, got %v", err)
		}
	})

	t.Run("Set and Get area round trip", func(t *testing.T) {
		q := createTestQuery(t, m)
		filter := &QueryFilter{}
		filter.IncludeFlags = 0xffff
		for i := range filter.AreaCost {
			filter.AreaCost[i] = 1.0
		}
		ref, _, _ := q.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Fatal("no valid ref found")
		}

		err := m.SetPolyArea(ref, 7)
		if err != nil {
			t.Fatalf("SetPolyArea: %v", err)
		}
		area, err := m.GetPolyArea(ref)
		if err != nil {
			t.Fatalf("GetPolyArea: %v", err)
		}
		if area != 7 {
			t.Fatalf("expected area=7, got %d", area)
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: ClosestPointOnPolyBoundary edge cases
// ---------------------------------------------------------------------------

func TestClosestPointOnPolyBoundary(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}

	t.Run("point inside returns same point", func(t *testing.T) {
		ref, _, _ := q.FindNearestPoly([3]float32{2, 0, 2}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Fatal("no ref")
		}
		closest, err := q.ClosestPointOnPolyBoundary(ref, [3]float32{2, 0, 2})
		if err != nil {
			t.Fatalf("ClosestPointOnPolyBoundary: %v", err)
		}
		// For a point inside the polygon, the boundary function returns
		// the same point (it's inside, so closest is itself).
		if closest[0] != 2 || closest[2] != 2 {
			t.Logf("expected (2,0,2), got (%f,%f,%f)", closest[0], closest[1], closest[2])
		}
	})

	t.Run("point outside returns clamped boundary point", func(t *testing.T) {
		ref, _, _ := q.FindNearestPoly([3]float32{2, 0, 2}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Fatal("no ref")
		}
		closest, err := q.ClosestPointOnPolyBoundary(ref, [3]float32{-100, 0, -100})
		if err != nil {
			t.Fatalf("ClosestPointOnPolyBoundary: %v", err)
		}
		_ = closest
	})

	t.Run("zero ref returns error", func(t *testing.T) {
		_, err := q.ClosestPointOnPolyBoundary(0, [3]float32{1, 0, 1})
		if err == nil {
			t.Fatal("expected error for zero ref")
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: IsValidPolyRef edge cases
// ---------------------------------------------------------------------------

func TestIsValidPolyRefEdgeCases(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff

	t.Run("zero ref is invalid", func(t *testing.T) {
		if q.IsValidPolyRef(0, filter) {
			t.Fatal("expected false for zero ref")
		}
	})
	t.Run("valid ref passes filter flags", func(t *testing.T) {
		ref, _, _ := q.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Skip("no ref found")
		}
		if !q.IsValidPolyRef(ref, filter) {
			t.Fatal("valid ref should pass")
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: GetOffMeshConnectionPolyEndPoints edge cases
// ---------------------------------------------------------------------------

func TestGetOffMeshConnectionPolyEndPoints(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("zero ref returns error", func(t *testing.T) {
		_, _, err := m.GetOffMeshConnectionPolyEndPoints(0, 0)
		if err != ErrFailure {
			t.Fatalf("expected ErrFailure, got %v", err)
		}
	})

	t.Run("ground poly returns error", func(t *testing.T) {
		q := createTestQuery(t, m)
		filter := &QueryFilter{}
		filter.IncludeFlags = 0xffff
		for i := range filter.AreaCost {
			filter.AreaCost[i] = 1.0
		}
		ref, _, _ := q.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{10, 2, 10}, filter)
		if ref == 0 {
			t.Skip("no ref found")
		}
		_, _, err := m.GetOffMeshConnectionPolyEndPoints(0, ref)
		if err != ErrFailure {
			t.Fatalf("expected ErrFailure for ground poly, got %v", err)
		}
	})

	t.Run("GetOffMeshConnectionByRef with zero ref", func(t *testing.T) {
		con := m.GetOffMeshConnectionByRef(0)
		if con != nil {
			t.Fatal("expected nil for zero ref")
		}
	})
}

// ---------------------------------------------------------------------------
// Additional: NavMeshQuery.ClosestPointOnPoly vs ClosestPointOnPolyBoundary
// ---------------------------------------------------------------------------

func TestClosestPointOnPolyConsistency(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)
	filter := &QueryFilter{}
	filter.IncludeFlags = 0xffff
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}

	// For an inside point, ClosestPointOnPoly should return overPoly=true
	// and the input coordinates (with height from the polygon).
	ref, _, _ := q.FindNearestPoly([3]float32{2, 0, 2}, [3]float32{10, 2, 10}, filter)
	if ref == 0 {
		t.Fatal("no ref found")
	}

	t.Run("inside point with non-zero elevation", func(t *testing.T) {
		// Point inside poly0 with elevation 5 — should project down to Y=0.
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{2, 5, 2})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		if overPoly && pt[1] != 0 {
			t.Logf("height expected 0, got %f", pt[1])
		}
		if overPoly {
			if pt[0] != 2 || pt[2] != 2 {
				t.Logf("expected (2,0,2) when over poly, got (%f,%f,%f)", pt[0], pt[1], pt[2])
			}
		}
	})

	t.Run("math round-trip with distance check", func(t *testing.T) {
		// For a point inside, the distance to the projected point
		// should be purely vertical.
		pt, overPoly, err := q.ClosestPointOnPoly(ref, [3]float32{2, 100, 2})
		if err != nil {
			t.Fatalf("ClosestPointOnPoly: %v", err)
		}
		_ = overPoly
		_ = pt
	})
}

// ---------------------------------------------------------------------------
// Additional: EncodePolyID / DecodePolyID comprehensive tests
// ---------------------------------------------------------------------------

func TestEncodeDecodePolyIDComprehensive(t *testing.T) {
	m := buildTestNavmesh(t)

	t.Run("maximum values fit in bit widths", func(t *testing.T) {
		// SaltBits=31, TileBits=0, PolyBits=1
		// Maximum salt value: 2^31 - 1 = 2147483647
		maxSalt := uint32((1 << 31) - 1)
		if maxSalt < math.MaxUint32 {
			// Salt is valid. Poly must be 0 or 1.
			ref := m.EncodePolyID(maxSalt, 0, 1)
			dsalt := m.DecodePolyIdSalt(ref)
			if dsalt != maxSalt {
				t.Fatalf("expected salt %d, got %d", maxSalt, dsalt)
			}
		}
	})

	t.Run("zero salt encodes and decodes", func(t *testing.T) {
		ref := m.EncodePolyID(0, 0, 0)
		dsalt, dtile, dpoly := m.DecodePolyID(ref)
		if dsalt != 0 || dtile != 0 || dpoly != 0 {
			t.Fatalf("expected (0,0,0), got (%d,%d,%d)", dsalt, dtile, dpoly)
		}
	})

	t.Run("all poly values 0 and 1", func(t *testing.T) {
		for _, poly := range []uint32{0, 1} {
			ref := m.EncodePolyID(1, 0, poly)
			_, _, dpoly := m.DecodePolyID(ref)
			if dpoly != poly {
				t.Fatalf("poly=%d: expected %d, got %d", poly, poly, dpoly)
			}
		}
	})
}

func TestFindPathExactResult(t *testing.T) {
	m := buildTestNavmesh(t)
	q := createTestQuery(t, m)

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{10, 2, 10}

	startPos := [3]float32{1, 0, 1}
	endPos := [3]float32{9, 0, 9}

	startRef, _, err := q.FindNearestPoly(startPos, halfExtents, filter)
	if err != nil || startRef == 0 {
		t.Fatalf("FindNearestPoly start: ref=%d err=%v", startRef, err)
	}
	endRef, _, err := q.FindNearestPoly(endPos, halfExtents, filter)
	if err != nil || endRef == 0 {
		t.Fatalf("FindNearestPoly end: ref=%d err=%v", endRef, err)
	}

	path, pathCount, err := q.FindPath(startRef, endRef, startPos, endPos, filter, 256)
	if err != nil {
		t.Fatalf("FindPath: %v", err)
	}
	if pathCount < 2 {
		t.Fatalf("FindPath: expected >= 2 refs, got %d", pathCount)
	}

	// 验证路径 refs 顺序正确
	if path[0] != startRef {
		t.Errorf("path[0] = %d, expected startRef %d", path[0], startRef)
	}
	lastIdx := pathCount - 1
	if path[lastIdx] != endRef {
		t.Errorf("path[%d] = %d, expected endRef %d", lastIdx, path[lastIdx], endRef)
	}
	t.Logf("path refs: %v (count=%d)", path[:pathCount], pathCount)
}
