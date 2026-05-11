package detour_tile_cache

import (
	"github.com/actfuns/recastnavigation/detour"
	"math"
	"unsafe"
)

const (
	maxRequests = 64
	maxUpdate   = 64
)

// TileCache manages dynamic obstacles on a navigation mesh by rebuilding
// affected tiles when obstacles are added or removed.
type TileCache struct {
	tileLutSize  int
	tileLutMask  int
	posLookup    []*CompressedTile
	nextFreeTile *CompressedTile
	tiles        []CompressedTile

	saltBits uint32
	tileBits uint32

	params TileCacheParams

	talloc TileCacheAlloc
	tcomp  TileCacheCompressor
	tmproc TileCacheMeshProcess

	obstacles        []TileCacheObstacle
	nextFreeObstacle *TileCacheObstacle

	reqs  [maxRequests]obstacleRequest
	nreqs int

	update  [maxUpdate]CompressedTileRef
	nupdate int
}

type obstacleRequest struct {
	action int
	ref    ObstacleRef
}

// NewTileCache creates a new tile cache.
func NewTileCache() *TileCache {
	tc := &TileCache{}
	return tc
}

// GetAlloc returns the allocator.
func (tc *TileCache) GetAlloc() TileCacheAlloc {
	return tc.talloc
}

// GetCompressor returns the compressor.
func (tc *TileCache) GetCompressor() TileCacheCompressor {
	return tc.tcomp
}

// GetParams returns the tile cache params.
func (tc *TileCache) GetParams() *TileCacheParams {
	return &tc.params
}

// GetTileCount returns the maximum number of tiles.
func (tc *TileCache) GetTileCount() int {
	return int(tc.params.MaxTiles)
}

// GetTile returns the tile at the given index.
func (tc *TileCache) GetTile(i int) *CompressedTile {
	return &tc.tiles[i]
}

// GetObstacleCount returns the maximum number of obstacles.
func (tc *TileCache) GetObstacleCount() int {
	return int(tc.params.MaxObstacles)
}

// GetObstacle returns the obstacle at the given index.
func (tc *TileCache) GetObstacle(i int) *TileCacheObstacle {
	return &tc.obstacles[i]
}

// GetObstacleByRef returns the obstacle for the given reference.
func (tc *TileCache) GetObstacleByRef(ref ObstacleRef) *TileCacheObstacle {
	if ref == 0 {
		return nil
	}
	idx := decodeObstacleIdObstacle(ref)
	if int(idx) >= int(tc.params.MaxObstacles) {
		return nil
	}
	ob := &tc.obstacles[idx]
	salt := decodeObstacleIdSalt(ref)
	if ob.Salt != salt {
		return nil
	}
	return ob
}

// GetObstacleRef returns the reference for the given obstacle.
func (tc *TileCache) GetObstacleRef(ob *TileCacheObstacle) ObstacleRef {
	if ob == nil {
		return 0
	}
	idx := uint32(0)
	for i := range tc.obstacles {
		if &tc.obstacles[i] == ob {
			idx = uint32(i)
			break
		}
	}
	return encodeObstacleId(ob.Salt, idx)
}

// Init initializes the tile cache.
func (tc *TileCache) Init(params *TileCacheParams, talloc TileCacheAlloc,
	tcomp TileCacheCompressor, tmproc TileCacheMeshProcess) error {

	tc.talloc = talloc
	tc.tcomp = tcomp
	tc.tmproc = tmproc
	tc.nreqs = 0
	tc.params = *params

	// Allocate space for obstacles.
	tc.obstacles = make([]TileCacheObstacle, params.MaxObstacles)
	tc.nextFreeObstacle = nil
	for i := int(params.MaxObstacles) - 1; i >= 0; i-- {
		tc.obstacles[i].Salt = 1
		tc.obstacles[i].Next = tc.nextFreeObstacle
		tc.nextFreeObstacle = &tc.obstacles[i]
	}

	// Init tiles
	tc.tileLutSize = int(NextPow2(uint32(params.MaxTiles) / 4))
	if tc.tileLutSize == 0 {
		tc.tileLutSize = 1
	}
	tc.tileLutMask = tc.tileLutSize - 1

	tc.tiles = make([]CompressedTile, params.MaxTiles)
	tc.posLookup = make([]*CompressedTile, tc.tileLutSize)

	tc.nextFreeTile = nil
	for i := int(params.MaxTiles) - 1; i >= 0; i-- {
		tc.tiles[i].Salt = 1
		tc.tiles[i].Next = tc.nextFreeTile
		tc.nextFreeTile = &tc.tiles[i]
	}

	// Init ID generator values.
	tc.tileBits = uint32(Ilog2(NextPow2(uint32(params.MaxTiles))))
	tc.saltBits = uint32(math.Min(31, float64(32-tc.tileBits)))
	if tc.saltBits < 10 {
		return detour.ErrInvalidParam
	}

	return nil
}

// GetTilesAt returns tiles at the given tile coordinates.
func (tc *TileCache) GetTilesAt(tx, ty int, tiles []CompressedTileRef, maxTiles int) int {
	n := 0
	h := computeTileHash(tx, ty, tc.tileLutMask)
	tile := tc.posLookup[h]
	for tile != nil {
		if tile.Header != nil && tile.Header.Tx == int32(tx) && tile.Header.Ty == int32(ty) {
			if n < maxTiles {
				tiles[n] = tc.GetTileRef(tile)
				n++
			}
		}
		tile = tile.Next
	}
	return n
}

// GetTileAt returns the tile at the given coordinates and layer.
func (tc *TileCache) GetTileAt(tx, ty, tlayer int) *CompressedTile {
	h := computeTileHash(tx, ty, tc.tileLutMask)
	tile := tc.posLookup[h]
	for tile != nil {
		if tile.Header != nil && tile.Header.Tx == int32(tx) &&
			tile.Header.Ty == int32(ty) && tile.Header.Tlayer == int32(tlayer) {
			return tile
		}
		tile = tile.Next
	}
	return nil
}

// GetTileRef returns a reference for the given tile.
func (tc *TileCache) GetTileRef(tile *CompressedTile) CompressedTileRef {
	if tile == nil {
		return 0
	}
	it := uint32(0)
	for i := range tc.tiles {
		if &tc.tiles[i] == tile {
			it = uint32(i)
			break
		}
	}
	return tc.encodeTileId(tile.Salt, it)
}

// GetTileByRef returns the tile for the given reference.
func (tc *TileCache) GetTileByRef(ref CompressedTileRef) *CompressedTile {
	if ref == 0 {
		return nil
	}
	tileIndex := tc.decodeTileIdTile(ref)
	tileSalt := tc.decodeTileIdSalt(ref)
	if int(tileIndex) >= int(tc.params.MaxTiles) {
		return nil
	}
	tile := &tc.tiles[tileIndex]
	if tile.Salt != tileSalt {
		return nil
	}
	return tile
}

// AddTile adds a tile to the cache.
func (tc *TileCache) AddTile(data []uint8, dataSize int, flags uint8, result *CompressedTileRef) error {
	if len(data) < TileCacheLayerHeaderSize() {
		return detour.ErrInvalidParam
	}

	// Use the data directly as a header.
	header := (*TileCacheLayerHeader)(unsafe.Pointer(&data[0]))

	if header.Magic != TileCacheMagic {
		return detour.ErrWrongMagic
	}
	if header.Version != TileCacheVersion {
		return detour.ErrWrongVersion
	}

	// Make sure the location is free.
	if tc.GetTileAt(int(header.Tx), int(header.Ty), int(header.Tlayer)) != nil {
		return detour.ErrFailure
	}

	// Allocate a tile.
	tile := tc.nextFreeTile
	if tile == nil {
		return detour.ErrOutOfMemory
	}
	tc.nextFreeTile = tile.Next
	tile.Next = nil

	// Insert tile into the position lut.
	h := computeTileHash(int(header.Tx), int(header.Ty), tc.tileLutMask)
	tile.Next = tc.posLookup[h]
	tc.posLookup[h] = tile

	// Init tile.
	headerSize := TileCacheLayerHeaderSize()
	tile.Header = header
	tile.Data = data
	tile.DataSize = dataSize
	tile.Compressed = data[headerSize:]
	tile.CompressedSize = dataSize - headerSize
	tile.Flags = uint32(flags)

	if result != nil {
		*result = tc.GetTileRef(tile)
	}

	return nil
}

// RemoveTile removes a tile from the cache.
func (tc *TileCache) RemoveTile(ref CompressedTileRef, data *[]uint8, dataSize *int) error {
	if ref == 0 {
		return detour.ErrInvalidParam
	}
	tileIndex := tc.decodeTileIdTile(ref)
	tileSalt := tc.decodeTileIdSalt(ref)
	if int(tileIndex) >= int(tc.params.MaxTiles) {
		return detour.ErrInvalidParam
	}
	tile := &tc.tiles[tileIndex]
	if tile.Salt != tileSalt {
		return detour.ErrInvalidParam
	}

	// Remove tile from hash lookup.
	h := computeTileHash(int(tile.Header.Tx), int(tile.Header.Ty), tc.tileLutMask)
	var prev *CompressedTile
	cur := tc.posLookup[h]
	for cur != nil {
		if cur == tile {
			if prev != nil {
				prev.Next = cur.Next
			} else {
				tc.posLookup[h] = cur.Next
			}
			break
		}
		prev = cur
		cur = cur.Next
	}

	// Reset tile.
	if tile.Flags&CompressedTileFreeData != 0 {
		tile.Data = nil
		tile.DataSize = 0
		if data != nil {
			*data = nil
		}
		if dataSize != nil {
			*dataSize = 0
		}
	} else {
		if data != nil {
			*data = tile.Data
		}
		if dataSize != nil {
			*dataSize = tile.DataSize
		}
	}

	tile.Header = nil
	tile.Data = nil
	tile.DataSize = 0
	tile.Compressed = nil
	tile.CompressedSize = 0
	tile.Flags = 0

	// Update salt, should never be zero.
	tile.Salt = (tile.Salt + 1) & ((1 << tc.saltBits) - 1)
	if tile.Salt == 0 {
		tile.Salt++
	}

	// Add to free list.
	tile.Next = tc.nextFreeTile
	tc.nextFreeTile = tile

	return nil
}

// AddObstacle adds a cylindrical obstacle.
func (tc *TileCache) AddObstacle(pos *[3]float32, radius, height float32, result *ObstacleRef) error {
	if tc.nreqs >= maxRequests {
		return detour.ErrBufferTooSmall
	}

	ob := tc.nextFreeObstacle
	if ob == nil {
		return detour.ErrOutOfMemory
	}
	tc.nextFreeObstacle = ob.Next
	ob.Next = nil

	salt := ob.Salt
	// Reset obstacle struct (zero out via assignment)
	*ob = TileCacheObstacle{}
	ob.Salt = salt
	ob.State = ObstacleProcessing
	ob.SetCylinder(*pos, radius, height)

	req := &tc.reqs[tc.nreqs]
	tc.nreqs++
	*req = obstacleRequest{}
	req.action = RequestAdd
	req.ref = tc.GetObstacleRef(ob)

	if result != nil {
		*result = req.ref
	}

	return nil
}

// AddBoxObstacle adds an axis-aligned box obstacle.
func (tc *TileCache) AddBoxObstacle(bmin, bmax *[3]float32, result *ObstacleRef) error {
	if tc.nreqs >= maxRequests {
		return detour.ErrBufferTooSmall
	}

	ob := tc.nextFreeObstacle
	if ob == nil {
		return detour.ErrOutOfMemory
	}
	tc.nextFreeObstacle = ob.Next
	ob.Next = nil

	salt := ob.Salt
	*ob = TileCacheObstacle{}
	ob.Salt = salt
	ob.State = ObstacleProcessing
	ob.SetBox(*bmin, *bmax)

	req := &tc.reqs[tc.nreqs]
	tc.nreqs++
	*req = obstacleRequest{}
	req.action = RequestAdd
	req.ref = tc.GetObstacleRef(ob)

	if result != nil {
		*result = req.ref
	}

	return nil
}

// AddBoxObstacleRot adds an oriented box obstacle with Y rotation.
func (tc *TileCache) AddBoxObstacleRot(center, halfExtents *[3]float32, yRadians float32, result *ObstacleRef) error {
	if tc.nreqs >= maxRequests {
		return detour.ErrBufferTooSmall
	}

	ob := tc.nextFreeObstacle
	if ob == nil {
		return detour.ErrOutOfMemory
	}
	tc.nextFreeObstacle = ob.Next
	ob.Next = nil

	salt := ob.Salt
	*ob = TileCacheObstacle{}
	ob.Salt = salt
	ob.State = ObstacleProcessing
	ob.Type = ObstacleTypeOrientedBox
	ob.orientedBox.Center = *center
	ob.orientedBox.HalfExtents = *halfExtents

	coshalf := float32(math.Cos(float64(0.5 * yRadians)))
	sinhalf := float32(math.Sin(float64(-0.5 * yRadians)))
	ob.orientedBox.RotAux[0] = coshalf * sinhalf
	ob.orientedBox.RotAux[1] = coshalf*coshalf - 0.5

	req := &tc.reqs[tc.nreqs]
	tc.nreqs++
	*req = obstacleRequest{}
	req.action = RequestAdd
	req.ref = tc.GetObstacleRef(ob)

	if result != nil {
		*result = req.ref
	}

	return nil
}

// RemoveObstacle removes an obstacle by reference.
func (tc *TileCache) RemoveObstacle(ref ObstacleRef) error {
	if ref == 0 {
		return nil
	}
	if tc.nreqs >= maxRequests {
		return detour.ErrBufferTooSmall
	}

	req := &tc.reqs[tc.nreqs]
	tc.nreqs++
	*req = obstacleRequest{}
	req.action = RequestRemove
	req.ref = ref

	return nil
}

// QueryTiles queries tiles overlapping the given bounding box.
func (tc *TileCache) QueryTiles(bmin, bmax *[3]float32, results []CompressedTileRef, resultCount *int, maxResults int) error {
	const maxTiles = 32
	tiles := make([]CompressedTileRef, maxTiles)

	n := 0
	tw := float32(tc.params.Width) * tc.params.Cs
	th := float32(tc.params.Height) * tc.params.Cs
	tx0 := int(math.Floor(float64((bmin[0] - tc.params.Orig[0]) / tw)))
	tx1 := int(math.Floor(float64((bmax[0] - tc.params.Orig[0]) / tw)))
	ty0 := int(math.Floor(float64((bmin[2] - tc.params.Orig[2]) / th)))
	ty1 := int(math.Floor(float64((bmax[2] - tc.params.Orig[2]) / th)))

	for ty := ty0; ty <= ty1; ty++ {
		for tx := tx0; tx <= tx1; tx++ {
			ntiles := tc.GetTilesAt(tx, ty, tiles, maxTiles)
			for i := 0; i < ntiles; i++ {
				tile := &tc.tiles[tc.decodeTileIdTile(tiles[i])]
				var tbmin, tbmax [3]float32
				tc.CalcTightTileBounds(tile.Header, &tbmin, &tbmax)

				if overlapBounds(bmin, bmax, &tbmin, &tbmax) {
					if n < maxResults {
						results[n] = tiles[i]
						n++
					}
				}
			}
		}
	}

	*resultCount = n
	return nil
}

// Update processes pending obstacle requests and rebuilds affected tiles.
func (tc *TileCache) Update(dt float32, navmesh NavMeshInterface, upToDate *bool) error {
	if tc.nupdate == 0 {
		// Process requests.
		for i := 0; i < tc.nreqs; i++ {
			req := &tc.reqs[i]

			idx := decodeObstacleIdObstacle(req.ref)
			if int(idx) >= int(tc.params.MaxObstacles) {
				continue
			}
			ob := &tc.obstacles[idx]
			salt := decodeObstacleIdSalt(req.ref)
			if ob.Salt != salt {
				continue
			}

			switch req.action {
			case RequestAdd:
				// Find touched tiles.
				var bmin, bmax [3]float32
				tc.GetObstacleBounds(ob, &bmin, &bmax)

				var ntouched int
				tc.QueryTiles(&bmin, &bmax, ob.Touched[:], &ntouched, MaxTouchedTiles)
				ob.NTouched = uint8(ntouched)
				// Add tiles to update list.
				ob.NPending = 0
				for j := 0; j < int(ob.NTouched); j++ {
					if tc.nupdate < maxUpdate {
						if !containsRef(tc.update[:tc.nupdate], ob.Touched[j]) {
							tc.update[tc.nupdate] = ob.Touched[j]
							tc.nupdate++
						}
						ob.Pending[ob.NPending] = ob.Touched[j]
						ob.NPending++
					}
				}
			case RequestRemove:
				// Prepare to remove obstacle.
				ob.State = ObstacleRemoving
				// Add tiles to update list.
				ob.NPending = 0
				for j := 0; j < int(ob.NTouched); j++ {
					if tc.nupdate < maxUpdate {
						if !containsRef(tc.update[:tc.nupdate], ob.Touched[j]) {
							tc.update[tc.nupdate] = ob.Touched[j]
							tc.nupdate++
						}
						ob.Pending[ob.NPending] = ob.Touched[j]
						ob.NPending++
					}
				}
			}
		}

		tc.nreqs = 0
	}

	var err error = nil
	// Process updates
	if tc.nupdate > 0 {
		ref := tc.update[0]
		err = tc.BuildNavMeshTile(ref, navmesh)
		tc.nupdate--
		if tc.nupdate > 0 {
			copy(tc.update[:tc.nupdate], tc.update[1:tc.nupdate+1])
		}

		// Update obstacle states.
		for i := 0; i < int(tc.params.MaxObstacles); i++ {
			ob := &tc.obstacles[i]
			if ob.State == ObstacleProcessing || ob.State == ObstacleRemoving {
				// Remove handled tile from pending list.
				for j := 0; j < int(ob.NPending); j++ {
					if ob.Pending[j] == ref {
						ob.Pending[j] = ob.Pending[int(ob.NPending)-1]
						ob.NPending--
						break
					}
				}

				// If all pending tiles processed, change state.
				if ob.NPending == 0 {
					switch ob.State {
					case ObstacleProcessing:
						ob.State = ObstacleProcessed
					case ObstacleRemoving:
						ob.State = ObstacleEmpty
						ob.Salt = (ob.Salt + 1) & ((1 << 16) - 1)
						if ob.Salt == 0 {
							ob.Salt++
						}
						ob.Next = tc.nextFreeObstacle
						tc.nextFreeObstacle = ob
					}
				}
			}
		}
	}

	if upToDate != nil {
		*upToDate = tc.nupdate == 0 && tc.nreqs == 0
	}

	return err
}

// BuildNavMeshTilesAt builds all tiles at the given coordinates.
func (tc *TileCache) BuildNavMeshTilesAt(tx, ty int, navmesh NavMeshInterface) error {
	const maxTiles = 32
	tiles := make([]CompressedTileRef, maxTiles)
	ntiles := tc.GetTilesAt(tx, ty, tiles, maxTiles)

	for i := 0; i < ntiles; i++ {
		err := tc.BuildNavMeshTile(tiles[i], navmesh)
		if err != nil {
			return err
		}
	}

	return nil
}

// BuildNavMeshTile builds a navigation mesh tile from a compressed tile.
func (tc *TileCache) BuildNavMeshTile(ref CompressedTileRef, navmesh NavMeshInterface) error {
	idx := tc.decodeTileIdTile(ref)
	if int(idx) >= int(tc.params.MaxTiles) {
		return detour.ErrInvalidParam
	}
	tile := &tc.tiles[idx]
	salt := tc.decodeTileIdSalt(ref)
	if tile.Salt != salt {
		return detour.ErrInvalidParam
	}

	tc.talloc.Reset()

	// Decompress tile layer data.
	layer, err := DecompressTileCacheLayer(tc.talloc, tc.tcomp, tile.Data, tile.DataSize)
	if err != nil {
		return err
	}

	walkableClimbVx := int(tc.params.WalkableClimb / tc.params.Ch)

	// Rasterize obstacles.
	for i := 0; i < int(tc.params.MaxObstacles); i++ {
		ob := &tc.obstacles[i]
		if ob.State == ObstacleEmpty || ob.State == ObstacleRemoving {
			continue
		}
		if containsRef(ob.Touched[:int(ob.NTouched)], ref) {
			switch ob.Type {
			case ObstacleTypeCylinder:
				MarkCylinderArea(layer, &tile.Header.Bmin, tc.params.Cs, tc.params.Ch,
					&ob.cylinder.Pos, ob.cylinder.Radius, ob.cylinder.Height, 0)
			case ObstacleTypeBox:
				MarkBoxArea(layer, &tile.Header.Bmin, tc.params.Cs, tc.params.Ch,
					&ob.box.Bmin, &ob.box.Bmax, 0)
			case ObstacleTypeOrientedBox:
				MarkBoxAreaOriented(layer, &tile.Header.Bmin, tc.params.Cs, tc.params.Ch,
					&ob.orientedBox.Center, &ob.orientedBox.HalfExtents, &ob.orientedBox.RotAux, 0)
			}
		}
	}

	// Build navmesh
	err = BuildTileCacheRegions(tc.talloc, layer, walkableClimbVx)
	if err != nil {
		return err
	}

	lcset := AllocTileCacheContourSet(tc.talloc)
	if lcset == nil {
		return detour.ErrOutOfMemory
	}
	err = BuildTileCacheContours(tc.talloc, layer, walkableClimbVx,
		tc.params.MaxSimplificationError, lcset)
	if err != nil {
		return err
	}

	lmesh := AllocTileCachePolyMesh(tc.talloc)
	if lmesh == nil {
		return detour.ErrOutOfMemory
	}
	err = BuildTileCachePolyMesh(tc.talloc, lcset, lmesh)
	if err != nil {
		return err
	}

	// Early out if the mesh tile is empty.
	if lmesh.NPolys == 0 {
		navmesh.RemoveTile(navmesh.GetTileRefAt(tile.Header.Tx, tile.Header.Ty, tile.Header.Tlayer), nil, nil)
		return nil
	}

	params := &NavMeshCreateParams{
		Verts:          lmesh.Verts,
		VertCount:      lmesh.NVerts,
		Polys:          lmesh.Polys,
		PolyAreas:      lmesh.Areas,
		PolyFlags:      lmesh.Flags,
		PolyCount:      lmesh.NPolys,
		Nvp:            6,
		WalkableHeight: tc.params.WalkableHeight,
		WalkableRadius: tc.params.WalkableRadius,
		WalkableClimb:  tc.params.WalkableClimb,
		TileX:          tile.Header.Tx,
		TileY:          tile.Header.Ty,
		TileLayer:      tile.Header.Tlayer,
		Cs:             tc.params.Cs,
		Ch:             tc.params.Ch,
		BuildBvTree:    false,
		Bmin:           tile.Header.Bmin,
		Bmax:           tile.Header.Bmax,
	}

	if tc.tmproc != nil {
		tc.tmproc.Process(params, lmesh.Areas, lmesh.Flags)
	}

	navData, navDataSize := CreateNavMeshData(params)
	if navData == nil {
		return detour.ErrFailure
	}

	// Remove existing tile.
	navmesh.RemoveTile(navmesh.GetTileRefAt(tile.Header.Tx, tile.Header.Ty, tile.Header.Tlayer), nil, nil)

	// Add new tile, or leave the location empty.
	err = navmesh.AddTile(navData, navDataSize, 0x01, nil)
	if err != nil {
		return err
	}

	return nil
}

// CalcTightTileBounds calculates tight bounds for a tile.
func (tc *TileCache) CalcTightTileBounds(header *TileCacheLayerHeader, bmin, bmax *[3]float32) {
	cs := tc.params.Cs
	bmin[0] = header.Bmin[0] + float32(header.Minx)*cs
	bmin[1] = header.Bmin[1]
	bmin[2] = header.Bmin[2] + float32(header.Miny)*cs
	bmax[0] = header.Bmin[0] + float32(header.Maxx+1)*cs
	bmax[1] = header.Bmax[1]
	bmax[2] = header.Bmin[2] + float32(header.Maxy+1)*cs
}

// GetObstacleBounds calculates bounds for an obstacle.
func (tc *TileCache) GetObstacleBounds(ob *TileCacheObstacle, bmin, bmax *[3]float32) {
	switch ob.Type {
	case ObstacleTypeCylinder:
		cl := &ob.cylinder
		bmin[0] = cl.Pos[0] - cl.Radius
		bmin[1] = cl.Pos[1]
		bmin[2] = cl.Pos[2] - cl.Radius
		bmax[0] = cl.Pos[0] + cl.Radius
		bmax[1] = cl.Pos[1] + cl.Height
		bmax[2] = cl.Pos[2] + cl.Radius
	case ObstacleTypeBox:
		*bmin = ob.box.Bmin
		*bmax = ob.box.Bmax
	case ObstacleTypeOrientedBox:
		orientedBox := &ob.orientedBox
		maxr := float32(1.41) * float32(math.Max(float64(orientedBox.HalfExtents[0]), float64(orientedBox.HalfExtents[2])))
		bmin[0] = orientedBox.Center[0] - maxr
		bmax[0] = orientedBox.Center[0] + maxr
		bmin[1] = orientedBox.Center[1] - orientedBox.HalfExtents[1]
		bmax[1] = orientedBox.Center[1] + orientedBox.HalfExtents[1]
		bmin[2] = orientedBox.Center[2] - maxr
		bmax[2] = orientedBox.Center[2] + maxr
	}
}

// ID encoding/decoding

func (tc *TileCache) encodeTileId(salt, it uint32) CompressedTileRef {
	return (CompressedTileRef(salt) << tc.tileBits) | CompressedTileRef(it)
}

func (tc *TileCache) decodeTileIdSalt(ref CompressedTileRef) uint32 {
	saltMask := (CompressedTileRef(1) << tc.saltBits) - 1
	return uint32((ref >> tc.tileBits) & saltMask)
}

func (tc *TileCache) decodeTileIdTile(ref CompressedTileRef) uint32 {
	tileMask := (CompressedTileRef(1) << tc.tileBits) - 1
	return uint32(ref & tileMask)
}

func encodeObstacleId(salt uint16, it uint32) ObstacleRef {
	return (ObstacleRef(salt) << 16) | ObstacleRef(it)
}

func decodeObstacleIdSalt(ref ObstacleRef) uint16 {
	const saltMask ObstacleRef = (1 << 16) - 1
	return uint16((ref >> 16) & saltMask)
}

func decodeObstacleIdObstacle(ref ObstacleRef) uint32 {
	const tileMask ObstacleRef = (1 << 16) - 1
	return uint32(ref & tileMask)
}

// Helper functions

func containsRef(a []CompressedTileRef, v CompressedTileRef) bool {
	for i := range a {
		if a[i] == v {
			return true
		}
	}
	return false
}

func computeTileHash(x, y, mask int) int {
	const h1 uint32 = 0x8da6b343
	const h2 uint32 = 0xd8163841
	n := h1*uint32(x) + h2*uint32(y)
	return int(n & uint32(mask))
}

func overlapBounds(aMin, aMax, bMin, bMax *[3]float32) bool {
	return aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2]
}
