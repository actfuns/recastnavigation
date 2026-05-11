// Package recast implements navigation mesh generation.
package recast

import "math"

// Constants

// Pi is the value of PI used by Recast.
const Pi = 3.14159265

// SpanHeightBits defines the number of bits allocated to span smin and smax.
const SpanHeightBits = 13

// SpanMaxHeight defines the maximum value for span smin and smax.
const SpanMaxHeight = (1 << SpanHeightBits) - 1

// SpansPerPool is the number of spans allocated per span pool.
const SpansPerPool = 2048

// MaxHeightfieldHeight is the maximum height value used internally.
const MaxHeightfieldHeight = 0xffff

// BorderReg is the heightfield border flag.
// If a heightfield region ID has this bit set, then the region is a border
// region and its spans are considered un-walkable.
const BorderReg = 0x8000

// MultipleRegs indicates polygon touches multiple regions.
const MultipleRegs = 0

// BorderVertex is the border vertex flag.
const BorderVertex = 0x10000

// AreaBorder is the area border flag.
const AreaBorder = 0x20000

// ContourRegMask is applied to the region id field of contour vertices to extract the region id.
const ContourRegMask = 0xffff

// MeshNullIdx is a value which indicates an invalid index within a mesh.
const MeshNullIdx = 0xffff

// NullArea represents the null area.
const NullArea = 0

// WalkableArea is the default area id used to indicate a walkable polygon.
const WalkableArea = 63

// NotConnected is the value returned by Con if the specified direction is not connected.
const NotConnected = 0x3f

// Epsilon is used for floating point comparisons.
const Epsilon = 1e-6

// MiterLimit defines the limit at which a miter becomes a bevel.
const MiterLimit = 1.20

// Log categories

// LogCategory represents a Recast log category.
type LogCategory int

const (
	LogProgress LogCategory = 1 + iota // A progress log entry.
	LogWarning                         // A warning log entry.
	LogError                           // An error log entry.
)

// TimerLabel represents a Recast performance timer category.
type TimerLabel int

const (
	TimerTotal TimerLabel = iota
	TimerTemp
	TimerRasterizeTriangles
	TimerBuildCompactHeightfield
	TimerBuildContours
	TimerBuildContoursTrace
	TimerBuildContoursSimplify
	TimerFilterBorder
	TimerFilterWalkable
	TimerMedianArea
	TimerFilterLowObstacles
	TimerBuildPolyMesh
	TimerMergePolyMesh
	TimerErodeArea
	TimerMarkBoxArea
	TimerMarkCylinderArea
	TimerMarkConvexPolyArea
	TimerBuildDistanceField
	TimerBuildDistanceFieldDist
	TimerBuildDistanceFieldBlur
	TimerBuildRegions
	TimerBuildRegionsWatershed
	TimerBuildRegionsExpand
	TimerBuildRegionsFlood
	TimerBuildRegionsFilter
	TimerBuildLayers
	TimerBuildPolyMeshDetail
	TimerMergePolyMeshDetail
	MaxTimers
)

// BuildContoursFlags represents contour build flags.
type BuildContoursFlags int

const (
	ContourTessWallEdges BuildContoursFlags = 1 << iota // Tessellate solid (impassable) edges during contour simplification.
	ContourTessAreaEdges                                // Tessellate edges between areas during contour simplification.
)

// Axis used for polygon clipping.
type Axis int

const (
	AxisX Axis = 0
	AxisY Axis = 1
	AxisZ Axis = 2
)

// Span represents a span in a heightfield.
type Span struct {
	Smin uint32 // The lower limit of the span. (Inclusive) [Limit: < Smax]
	Smax uint32 // The upper limit of the span. (Exclusive) [Limit: <= SpanMaxHeight]
	Area uint32 // The area id assigned to the span.
	Next *Span  // The next span higher up in column.
}

// SpanPool is a memory pool used for quick allocation of spans within a heightfield.
type SpanPool struct {
	Next  *SpanPool
	Items [SpansPerPool]Span
}

// Config specifies a configuration to use when performing Recast builds.
type Config struct {
	Width      int        // The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	Height     int        // The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	TileSize   int        // The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	BorderSize int        // The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	Cs         float32    // The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu]
	Ch         float32    // The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	Bmin       [3]float32 // The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	Bmax       [3]float32 // The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]

	WalkableSlopeAngle     float32 // The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]
	WalkableHeight         int     // Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
	WalkableClimb          int     // Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
	WalkableRadius         int     // The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: >=0] [Units: vx]
	MaxEdgeLen             int     // The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx]
	MaxSimplificationError float32 // The maximum distance a simplified contour's border edges should deviate the original raw contour. [Limit: >=0] [Units: vx]
	MinRegionArea          int     // The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx]
	MergeRegionArea        int     // Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit: >=0] [Units: vx]
	MaxVertsPerPoly        int     // The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process. [Limit: >= 3]
	DetailSampleDist       float32 // Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
	DetailSampleMaxError   float32 // The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.) [Limit: >=0] [Units: wu]
}

// Heightfield is a dynamic heightfield representing obstructed space.
type Heightfield struct {
	Width    int        // The width of the heightfield. (Along the x-axis in cell units.)
	Height   int        // The height of the heightfield. (Along the z-axis in cell units.)
	Bmin     [3]float32 // The minimum bounds in world space. [(x, y, z)]
	Bmax     [3]float32 // The maximum bounds in world space. [(x, y, z)]
	Cs       float32    // The size of each cell. (On the xz-plane.)
	Ch       float32    // The height of each cell. (The minimum increment along the y-axis.)
	Spans    []*Span    // Heightfield of spans (width*height).
	Pools    *SpanPool  // Linked list of span pools.
	FreeList *Span      // The next free span.
}

// CompactCell provides information on the content of a cell column in a compact heightfield.
type CompactCell struct {
	Index uint32 // Index to the first span in the column. (24 bits)
	Count uint32 // Number of spans in the column. (8 bits)
}

// CompactSpan represents a span of unobstructed space within a compact heightfield.
type CompactSpan struct {
	Y   uint16 // The lower extent of the span. (Measured from the heightfield's base.)
	Reg uint16 // The id of the region the span belongs to. (Or zero if not in a region.)
	Con uint32 // Packed neighbor connection data. (24 bits)
	H   uint8  // The height of the span. (Measured from Y.) (8 bits)
}

// CompactHeightfield is a compact, static heightfield representing unobstructed space.
type CompactHeightfield struct {
	Width          int
	Height         int
	SpanCount      int
	WalkableHeight int
	WalkableClimb  int
	BorderSize     int
	MaxDistance    uint16
	MaxRegions     uint16
	Bmin           [3]float32
	Bmax           [3]float32
	Cs             float32
	Ch             float32
	Cells          []CompactCell
	Spans          []CompactSpan
	Dist           []uint16
	Areas          []uint8
}

// HeightfieldLayer represents a heightfield layer within a layer set.
type HeightfieldLayer struct {
	Bmin    [3]float32
	Bmax    [3]float32
	Cs      float32
	Ch      float32
	Width   int
	Height  int
	MinX    int
	MaxX    int
	MinY    int
	MaxY    int
	HMin    int
	HMax    int
	Heights []uint8
	Areas   []uint8
	Cons    []uint8
}

// HeightfieldLayerSet represents a set of heightfield layers.
type HeightfieldLayerSet struct {
	Layers  []HeightfieldLayer
	NLayers int
}

// Contour represents a simple, non-overlapping contour in field space.
type Contour struct {
	Verts  []int  // Simplified contour vertex and connection data. [Size: 4 * nverts]
	Nverts int    // The number of vertices in the simplified contour.
	RVerts []int  // Raw contour vertex and connection data. [Size: 4 * nrverts]
	Nrvets int    // The number of vertices in the raw contour.
	Reg    uint16 // The region id of the contour.
	Area   uint8  // The area id of the contour.
}

// ContourSet represents a group of related contours.
type ContourSet struct {
	Conts      []Contour
	Nconts     int
	Bmin       [3]float32
	Bmax       [3]float32
	Cs         float32
	Ch         float32
	Width      int
	Height     int
	BorderSize int
	MaxError   float32
}

// PolyMesh represents a polygon mesh suitable for use in building a navigation mesh.
type PolyMesh struct {
	Verts        []uint16 // The mesh vertices. [Form: (x, y, z) * nverts]
	Polys        []uint16 // Polygon and neighbor data. [Length: maxpolys * 2 * nvp]
	Regs         []uint16 // The region id assigned to each polygon. [Length: maxpolys]
	Flags        []uint16 // The user defined flags for each polygon. [Length: maxpolys]
	Areas        []uint8  // The area id assigned to each polygon. [Length: maxpolys]
	Nverts       int      // The number of vertices.
	Npolys       int      // The number of polygons.
	MaxPolys     int      // The number of allocated polygons.
	Nvp          int      // The maximum number of vertices per polygon.
	Bmin         [3]float32
	Bmax         [3]float32
	Cs           float32
	Ch           float32
	BorderSize   int
	MaxEdgeError float32
}

// PolyMeshDetail contains triangle meshes that represent detailed height data associated
// with the polygons in its associated polygon mesh object.
type PolyMeshDetail struct {
	Meshes  []uint32  // The sub-mesh data. [Size: 4 * nmeshes]
	Verts   []float32 // The mesh vertices. [Size: 3 * nverts]
	Tris    []uint8   // The mesh triangles. [Size: 4 * ntris]
	Nmeshes int       // The number of sub-meshes defined by meshes.
	Nverts  int       // The number of vertices in verts.
	Ntris   int       // The number of triangles in tris.
}

// Helper functions

// Min returns the minimum of two values.
func Min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// MinF returns the minimum of two float32 values.
func MinF(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

// Max returns the maximum of two values.
func Max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// MaxF returns the maximum of two float32 values.
func MaxF(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

// Abs returns the absolute value.
func Abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

// AbsF returns the absolute value for float32.
func AbsF(a float32) float32 {
	if a < 0 {
		return -a
	}
	return a
}

// Sqr returns the square of the value.
func Sqr(a float32) float32 {
	return a * a
}

// Clamp clamps the value to the specified range.
func Clamp(value, minInclusive, maxInclusive int) int {
	if value < minInclusive {
		return minInclusive
	}
	if value > maxInclusive {
		return maxInclusive
	}
	return value
}

// ClampF32 clamps the float32 value to the specified range.
func ClampF32(value, minInclusive, maxInclusive float32) float32 {
	if value < minInclusive {
		return minInclusive
	}
	if value > maxInclusive {
		return maxInclusive
	}
	return value
}

// Swap swaps the values of two variables.
func Swap[T any](a, b *T) {
	*a, *b = *b, *a
}

// Sqrt returns the square root of the value.
func Sqrt(x float32) float32 {
	return float32(math.Sqrt(float64(x)))
}

// Vector helper functions

// Vcross derives the cross product of two vectors.
func Vcross(v1, v2 [3]float32) [3]float32 {
	return [3]float32{
		v1[1]*v2[2] - v1[2]*v2[1],
		v1[2]*v2[0] - v1[0]*v2[2],
		v1[0]*v2[1] - v1[1]*v2[0],
	}
}

// Vdot derives the dot product of two vectors.
func Vdot(v1, v2 *[3]float32) float32 {
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
}

// Vmad performs a scaled vector addition.
func Vmad(v1, v2 [3]float32, s float32) [3]float32 {
	return [3]float32{
		v1[0] + v2[0]*s,
		v1[1] + v2[1]*s,
		v1[2] + v2[2]*s,
	}
}

// Vadd performs a vector addition.
func Vadd(v1, v2 [3]float32) [3]float32 {
	return [3]float32{
		v1[0] + v2[0],
		v1[1] + v2[1],
		v1[2] + v2[2],
	}
}

// Vsub performs a vector subtraction.
func Vsub(v1, v2 [3]float32) [3]float32 {
	return [3]float32{
		v1[0] - v2[0],
		v1[1] - v2[1],
		v1[2] - v2[2],
	}
}

// Vmin selects the minimum value of each element from the specified vectors.
func Vmin(mn, v [3]float32) [3]float32 {
	return [3]float32{
		min(mn[0], v[0]),
		min(mn[1], v[1]),
		min(mn[2], v[2]),
	}
}

// Vmax selects the maximum value of each element from the specified vectors.
func Vmax(mx, v [3]float32) [3]float32 {
	return [3]float32{
		max(mx[0], v[0]),
		max(mx[1], v[1]),
		max(mx[2], v[2]),
	}
}

// Vcopy performs a vector copy.
func Vcopy(v [3]float32) [3]float32 {
	return [3]float32{v[0], v[1], v[2]}
}

// Vdist returns the distance between two points.
func Vdist(v1, v2 *[3]float32) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return Sqrt(dx*dx + dy*dy + dz*dz)
}

// VdistSqr returns the square of the distance between two points.
func VdistSqr(v1, v2 *[3]float32) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return dx*dx + dy*dy + dz*dz
}

// Vnormalize normalizes the vector.
func Vnormalize(v [3]float32) [3]float32 {
	d := 1.0 / Sqrt(Sqr(v[0])+Sqr(v[1])+Sqr(v[2]))
	return [3]float32{v[0] * d, v[1] * d, v[2] * d}
}

// SetCon sets the neighbor connection data for the specified direction.
func SetCon(span *CompactSpan, direction int, neighborIndex int) {
	shift := uint32(direction) * 6
	con := span.Con
	span.Con = (con & ^(0x3f << shift)) | ((uint32(neighborIndex) & 0x3f) << shift)
}

// Con gets neighbor connection data for the specified direction.
func Con(span *CompactSpan, direction int) int {
	shift := uint32(direction) * 6
	return int((span.Con >> shift) & 0x3f)
}

// DirOffsetX gets the standard width (x-axis) offset for the specified direction.
func DirOffsetX(direction int) int {
	offset := [4]int{-1, 0, 1, 0}
	return offset[direction&0x03]
}

// DirOffsetZ gets the standard height (z-axis) offset for the specified direction.
func DirOffsetZ(direction int) int {
	offset := [4]int{0, 1, 0, -1}
	return offset[direction&0x03]
}

// DirForOffset gets the direction for the specified offset. One of x and z should be 0.
func DirForOffset(offsetX, offsetZ int) int {
	dirs := [5]int{3, 0, -1, 2, 1}
	return dirs[((offsetZ+1)<<1)+offsetX]
}

// VsafeNormalize normalizes the vector if the length is greater than zero.
func VsafeNormalize(v [3]float32) [3]float32 {
	sqMag := Sqr(v[0]) + Sqr(v[1]) + Sqr(v[2])
	if sqMag > Epsilon {
		inverseMag := 1.0 / Sqrt(sqMag)
		return [3]float32{v[0] * inverseMag, v[1] * inverseMag, v[2] * inverseMag}
	}
	return v
}

// CalcTriNormal calculates the normal of a triangle.
func CalcTriNormal(v0, v1, v2 [3]float32) [3]float32 {
	e0 := Vsub(v1, v0)
	e1 := Vsub(v2, v0)
	n := Vcross(e0, e1)
	return Vnormalize(n)
}

// OverlapBounds checks whether two bounding boxes overlap.
func OverlapBounds(aMin, aMax, bMin, bMax *[3]float32) bool {
	return aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2]
}

// PointInPoly checks if a point is contained within a polygon.
func PointInPoly(numVerts int, verts []float32, point *[3]float32) bool {
	inPoly := false
	for i, j := 0, numVerts-1; i < numVerts; j, i = i, i+1 {
		vi := verts[i*3 : i*3+3]
		vj := verts[j*3 : j*3+3]

		if (vi[2] > point[2]) == (vj[2] > point[2]) {
			continue
		}

		if point[0] >= (vj[0]-vi[0])*(point[2]-vi[2])/(vj[2]-vi[2])+vi[0] {
			continue
		}
		inPoly = !inPoly
	}
	return inPoly
}
