// Package detour_tile_cache implements dynamic obstacle management for navigation meshes.
// It allows adding and removing obstacles at runtime by rebuilding affected tiles.
package detour_tile_cache

// Constants
const (
	TileCacheMagic   = 'D'<<24 | 'T'<<16 | 'L'<<8 | 'R' // 'DTLR'
	TileCacheVersion = 1

	TileCacheNullArea     = 0
	TileCacheWalkableArea = 63
	TileCacheNullIdx      = 0xffff

	// MaxTouchedTiles is the maximum number of tiles an obstacle can touch.
	MaxTouchedTiles = 8
)

// CompressedTileFlags
const (
	CompressedTileFreeData = 0x01 // Navmesh owns the tile memory and should free it.
)

// Obstacle state constants
const (
	ObstacleEmpty      = 0
	ObstacleProcessing = 1
	ObstacleProcessed  = 2
	ObstacleRemoving   = 3
)

// Obstacle type constants
const (
	ObstacleTypeCylinder    = 0
	ObstacleTypeBox         = 1 // AABB
	ObstacleTypeOrientedBox = 2 // OBB
)

// ObstacleRequestAction constants
const (
	RequestAdd    = 0
	RequestRemove = 1
)

// CompressedTileRef is a reference to a compressed tile.
type CompressedTileRef = uint32

// ObstacleRef is a reference to an obstacle.
type ObstacleRef = uint32

// TileCacheLayerHeader contains metadata for a tile cache layer.
type TileCacheLayerHeader struct {
	Magic          int32
	Version        int32
	Tx, Ty, Tlayer int32
	Bmin           [3]float32
	Bmax           [3]float32
	Hmin           uint16
	Hmax           uint16
	Width          uint8
	Height         uint8
	Minx           uint8
	Maxx           uint8
	Miny           uint8
	Maxy           uint8
}

// TileCacheLayerHeaderSize returns the aligned size of TileCacheLayerHeader.
func TileCacheLayerHeaderSize() int {
	return Align4(56) // actual size of the struct
}

// TileCacheLayer represents a layer of compressed tile data.
type TileCacheLayer struct {
	Header   *TileCacheLayerHeader
	RegCount uint8
	Heights  []uint8
	Areas    []uint8
	Cons     []uint8
	Regs     []uint8
}

// TileCacheContour represents a contour in the tile cache.
type TileCacheContour struct {
	NVerts int
	Verts  []uint8
	Reg    uint8
	Area   uint8
}

// TileCacheContourSet represents a set of contours.
type TileCacheContourSet struct {
	NConts int
	Conts  []TileCacheContour
}

// TileCachePolyMesh represents a polygon mesh in the tile cache.
type TileCachePolyMesh struct {
	Nvp    int
	NVerts int
	NPolys int
	Verts  []uint16
	Polys  []uint16
	Flags  []uint16
	Areas  []uint8
}

// CompressedTile represents a compressed tile in the tile cache.
type CompressedTile struct {
	Salt           uint32
	Header         *TileCacheLayerHeader
	Compressed     []uint8
	CompressedSize int
	Data           []uint8
	DataSize       int
	Flags          uint32
	Next           *CompressedTile
}

// ObstacleTypeCylinder represents a cylindrical obstacle.
type ObstacleCylinder struct {
	Pos    [3]float32
	Radius float32
	Height float32
}

// ObstacleBox represents an axis-aligned box obstacle.
type ObstacleBox struct {
	Bmin [3]float32
	Bmax [3]float32
}

// ObstacleOrientedBox represents an oriented box obstacle (rotatable around Y).
type ObstacleOrientedBox struct {
	Center      [3]float32
	HalfExtents [3]float32
	RotAux      [2]float32 // {cos(0.5*angle)*sin(-0.5*angle); cos(0.5*angle)*cos(0.5*angle) - 0.5}
}

// TileCacheObstacle represents an obstacle in the tile cache.
type TileCacheObstacle struct {
	cylinder    ObstacleCylinder
	box         ObstacleBox
	orientedBox ObstacleOrientedBox

	Touched  [MaxTouchedTiles]CompressedTileRef
	Pending  [MaxTouchedTiles]CompressedTileRef
	Salt     uint16
	Type     uint8
	State    uint8
	NTouched uint8
	NPending uint8
	Next     *TileCacheObstacle
}

// GetCylinder returns the cylinder data. The obstacle must be of type ObstacleCylinder.
func (o *TileCacheObstacle) GetCylinder() *ObstacleCylinder { return &o.cylinder }

// GetBox returns the box data. The obstacle must be of type ObstacleBox.
func (o *TileCacheObstacle) GetBox() *ObstacleBox { return &o.box }

// GetOrientedBox returns the oriented box data. The obstacle must be of type ObstacleOrientedBox.
func (o *TileCacheObstacle) GetOrientedBox() *ObstacleOrientedBox { return &o.orientedBox }

// SetCylinder sets the cylinder data and type.
func (o *TileCacheObstacle) SetCylinder(pos [3]float32, radius, height float32) {
	o.cylinder.Pos = pos
	o.cylinder.Radius = radius
	o.cylinder.Height = height
	o.Type = ObstacleTypeCylinder
}

// SetBox sets the box data and type.
func (o *TileCacheObstacle) SetBox(bmin, bmax [3]float32) {
	o.box.Bmin = bmin
	o.box.Bmax = bmax
	o.Type = ObstacleTypeBox
}

// SetOrientedBox sets the oriented box data and type.
func (o *TileCacheObstacle) SetOrientedBox(center, halfExtents [3]float32, rotAux [2]float32) {
	o.orientedBox.Center = center
	o.orientedBox.HalfExtents = halfExtents
	o.orientedBox.RotAux = rotAux
	o.Type = ObstacleTypeOrientedBox
}

// TileCacheParams holds configuration parameters for the tile cache.
type TileCacheParams struct {
	Orig                   [3]float32
	Cs, Ch                 float32
	Width, Height          int32
	WalkableHeight         float32
	WalkableRadius         float32
	WalkableClimb          float32
	MaxSimplificationError float32
	MaxTiles               int32
	MaxObstacles           int32
}

// TileCacheAlloc is the interface for memory allocation used by the tile cache.
type TileCacheAlloc interface {
	Reset()
	Alloc(size int) []uint8
	Free(ptr []uint8)
}

// TileCacheCompressor is the interface for data compression used by the tile cache.
type TileCacheCompressor interface {
	MaxCompressedSize(bufferSize int) int
	Compress(buffer []uint8, compressed []uint8, compressedSize *int) error
	Decompress(compressed []uint8, buffer []uint8, bufferSize *int) error
}

// TileCacheMeshProcess is the interface for processing mesh data during tile building.
type TileCacheMeshProcess interface {
	Process(params *NavMeshCreateParams, polyAreas []uint8, polyFlags []uint16)
}

// NavMeshCreateParams holds parameters for creating navigation mesh data.
type NavMeshCreateParams struct {
	Verts          []uint16
	VertCount      int
	Polys          []uint16
	PolyAreas      []uint8
	PolyFlags      []uint16
	PolyCount      int
	Nvp            int
	WalkableHeight float32
	WalkableRadius float32
	WalkableClimb  float32
	TileX          int32
	TileY          int32
	TileLayer      int32
	Cs             float32
	Ch             float32
	BuildBvTree    bool
	Bmin           [3]float32
	Bmax           [3]float32
}

// NavMeshInterface defines the subset of NavMesh methods used by TileCache.
type NavMeshInterface interface {
	RemoveTile(ref uint32, data *[]uint8, dataSize *int) error
	GetTileRefAt(tx, ty, tlayer int32) uint32
	AddTile(data []uint8, dataSize int, flags uint8, tileRef *uint32) error
}

// Utility functions

// Align4 aligns an integer to 4 bytes.
func Align4(x int) int {
	return (x + 3) & ^3
}

// NextPow2 returns the next power of two >= v.
func NextPow2(v uint32) uint32 {
	r := uint32(1)
	for r < v {
		r <<= 1
	}
	return r
}

// Ilog2 returns the integer log base 2 of v (v > 0).
func Ilog2(v uint32) int {
	r := 0
	for v > 1 {
		v >>= 1
		r++
	}
	return r
}
