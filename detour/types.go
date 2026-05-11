package detour

// PolyRef is a handle to a polygon within a navigation mesh tile.
type PolyRef uint32

// TileRef is a handle to a tile within a navigation mesh.
type TileRef uint32

const (
	// SaltBits is the number of salt bits in the tile ID.
	SaltBits = 10
	// TileBits is the number of tile bits in the tile ID.
	TileBits = 21
	// PolyBits is the number of poly bits in the tile ID.
	PolyBits = 21
)

// Constants
const (
	VertsPerPolygon     = 6
	NavMeshMagic        = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V'
	NavMeshVersion      = 7
	NavMeshStateMagic   = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S'
	NavMeshStateVersion = 1

	ExtLink         = 0x8000
	NullLink        = 0xffffffff
	OffMeshConBidir = 1
	MaxAreas        = 64

	NullIdx NodeIndex = 0xffff

	NodeParentBits   = 24
	NodeStateBits    = 2
	MaxStatesPerNode = 1 << NodeStateBits

	RayCastLimitProportions = 50.0

	MeshNullIdx = 0xffff
)

// TileFlags
const (
	TileFreeData = 0x01
)

// StraightPathFlags
const (
	StraightPathStart             = 0x01
	StraightPathEnd               = 0x02
	StraightPathOffMeshConnection = 0x04
)

// StraightPathOptions
const (
	StraightPathAreaCrossings = 0x01
	StraightPathAllCrossings  = 0x02
)

// FindPathOptions
const (
	FindPathAnyAngle = 0x02
)

// RaycastOptions
const (
	RaycastUseCosts = 0x01
)

// DetailTriEdgeFlags
const (
	DetailEdgeBoundary = 0x01
)

// PolyTypes
const (
	PolyTypeGround            = 0
	PolyTypeOffMeshConnection = 1
)

// NodeFlags
const (
	NodeOpen           = 0x01
	NodeClosed         = 0x02
	NodeParentDetached = 0x04
)

// NodeIndex type
type NodeIndex uint16

// Poly is a polygon within a MeshTile object.
type Poly struct {
	FirstLink   uint32
	Verts       [VertsPerPolygon]uint16
	Neis        [VertsPerPolygon]uint16
	Flags       uint16
	VertCount   uint8
	areaAndtype uint8
}

func (p *Poly) SetArea(a uint8) {
	p.areaAndtype = (p.areaAndtype & 0xc0) | (a & 0x3f)
}

func (p *Poly) SetType(t uint8) {
	p.areaAndtype = (p.areaAndtype & 0x3f) | (t << 6)
}

func (p *Poly) GetArea() uint8 {
	return p.areaAndtype & 0x3f
}

func (p *Poly) GetType() uint8 {
	return p.areaAndtype >> 6
}

// PolyDetail defines the location of detail sub-mesh data within a MeshTile.
type PolyDetail struct {
	VertBase  uint32
	TriBase   uint32
	VertCount uint8
	TriCount  uint8
}

// Link defines a link between polygons.
type Link struct {
	Ref  PolyRef
	Next uint32
	Edge uint8
	Side uint8
	Bmin uint8
	Bmax uint8
}

// BVNode is a bounding volume node.
type BVNode struct {
	Bmin [3]uint16
	Bmax [3]uint16
	I    int32
}

// OffMeshConnection defines a navigation mesh off-mesh connection.
type OffMeshConnection struct {
	Pos    [6]float32
	Rad    float32
	Poly   uint16
	Flags  uint8
	Side   uint8
	UserID uint32
}

// MeshHeader provides high level information related to a MeshTile object.
type MeshHeader struct {
	Magic           int32
	Version         int32
	X               int32
	Y               int32
	Layer           int32
	UserID          uint32
	PolyCount       int32
	VertCount       int32
	MaxLinkCount    int32
	DetailMeshCount int32
	DetailVertCount int32
	DetailTriCount  int32
	BVNodeCount     int32
	OffMeshConCount int32
	OffMeshBase     int32
	WalkableHeight  float32
	WalkableRadius  float32
	WalkableClimb   float32
	Bmin            [3]float32
	Bmax            [3]float32
	BVQuantFactor   float32
}

// MeshTile defines a navigation mesh tile.
type MeshTile struct {
	Salt          uint32
	LinksFreeList uint32
	Header        *MeshHeader
	Polys         []Poly
	Verts         []float32
	Links         []Link
	DetailMeshes  []PolyDetail
	DetailVerts   []float32
	DetailTris    []uint8
	BVTree        []BVNode
	OffMeshCons   []OffMeshConnection
	Data          []byte
	Flags         int
	Next          *MeshTile
}

// NavMeshParams configuration parameters used to define multi-tile navigation meshes.
type NavMeshParams struct {
	Orig       [3]float32
	TileWidth  float32
	TileHeight float32
	MaxTiles   int32
	MaxPolys   int32
}

// GetDetailTriEdgeFlags gets flags for edge in detail triangle.
func GetDetailTriEdgeFlags(triFlags uint8, edgeIndex int) int {
	return int((triFlags >> (edgeIndex * 2)) & 0x3)
}

// Node is a pathfinding node.
type Node struct {
	Pos   [3]float32
	Cost  float32
	Total float32
	Pidx  uint32 // actually only uses NodeParentBits bits
	State uint8  // actually only uses NodeStateBits bits
	Flags uint8  // actually only uses 3 bits
	ID    PolyRef
}

// OffMeshConClass is used during tile building for off-mesh connection classification.
type OffMeshConClass uint8

const (
	OffMeshConClassXP OffMeshConClass = 1 << 0
	OffMeshConClassZP OffMeshConClass = 1 << 1
	OffMeshConClassXM OffMeshConClass = 1 << 2
	OffMeshConClassZM OffMeshConClass = 1 << 3
)

// QueryData is used for sliced path queries.
type QueryData struct {
	Err              error
	LastBestNode     *Node
	LastBestNodeCost float32
	StartRef         PolyRef
	EndRef           PolyRef
	StartPos         [3]float32
	EndPos           [3]float32
	Filter           *QueryFilter
	Options          uint32
	RaycastLimitSqr  float32
}

// H_SCALE is the search heuristic scale.
const H_SCALE = 0.999
