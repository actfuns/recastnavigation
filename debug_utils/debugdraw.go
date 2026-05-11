// Package debug_utils implements debug visualization utilities for recastnavigation.
package debug_utils

type DrawPrimitives int

const (
	DrawPoints DrawPrimitives = iota
	DrawLines
	DrawTris
	DrawQuads
)

// Pi is the value of PI used by debug drawing.
const Pi = 3.14159265

// DebugDraw is the abstract interface for debug rendering.
type DebugDraw interface {
	DepthMask(state bool)
	Texture(state bool)
	Begin(prim DrawPrimitives, size float32)
	VertexPosColor(pos [3]float32, color uint32)
	VertexXYZColor(x, y, z float32, color uint32)
	VertexPosColorUV(pos [3]float32, color uint32, uv [2]float32)
	VertexXYZColorUV(x, y, z float32, color uint32, u, v float32)
	End()
	AreaToCol(area uint32) uint32
}

// RGBA packs RGBA values into a uint32 color.
func RGBA(r, g, b, a int) uint32 {
	return uint32(r) | (uint32(g) << 8) | (uint32(b) << 16) | (uint32(a) << 24)
}

// RGBAf packs float RGBA values into a uint32 color.
func RGBAf(fr, fg, fb, fa float32) uint32 {
	r := uint8(fr * 255.0)
	g := uint8(fg * 255.0)
	b := uint8(fb * 255.0)
	a := uint8(fa * 255.0)
	return RGBA(int(r), int(g), int(b), int(a))
}

func bit(a, b int) int {
	return (a & (1 << b)) >> b
}

// IntToCol converts an integer index to a packed color.
func IntToCol(i, a int) uint32 {
	r := bit(i, 1) + bit(i, 3)*2 + 1
	g := bit(i, 2) + bit(i, 4)*2 + 1
	b := bit(i, 0) + bit(i, 5)*2 + 1
	return RGBA(r*63, g*63, b*63, a)
}

// IntToColFloat converts an integer index to float color.
func IntToColFloat(i int) [3]float32 {
	r := bit(i, 0) + bit(i, 3)*2 + 1
	g := bit(i, 1) + bit(i, 4)*2 + 1
	b := bit(i, 2) + bit(i, 5)*2 + 1
	return [3]float32{
		1 - float32(r*63)/255.0,
		1 - float32(g*63)/255.0,
		1 - float32(b*63)/255.0,
	}
}

// MultCol multiplies the RGB channels of a color by a factor.
func MultCol(col uint32, d uint32) uint32 {
	r := col & 0xff
	g := (col >> 8) & 0xff
	b := (col >> 16) & 0xff
	a := (col >> 24) & 0xff
	return RGBA(int((r*d)>>8), int((g*d)>>8), int((b*d)>>8), int(a))
}

// DarkenCol darkens a color by halving RGB.
func DarkenCol(col uint32) uint32 {
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000)
}

// LerpCol linearly interpolates between two colors.
func LerpCol(ca, cb uint32, u uint32) uint32 {
	ra := ca & 0xff
	ga := (ca >> 8) & 0xff
	ba := (ca >> 16) & 0xff
	aa := (ca >> 24) & 0xff
	rb := cb & 0xff
	gb := (cb >> 8) & 0xff
	bb := (cb >> 16) & 0xff
	ab := (cb >> 24) & 0xff

	r := (ra*(255-u) + rb*u) / 255
	g := (ga*(255-u) + gb*u) / 255
	b := (ba*(255-u) + bb*u) / 255
	a := (aa*(255-u) + ab*u) / 255
	return RGBA(int(r), int(g), int(b), int(a))
}

// TransCol replaces the alpha channel of a color.
func TransCol(c uint32, a uint32) uint32 {
	return (a << 24) | (c & 0x00ffffff)
}

// CalcBoxColors calculates face colors for a box.
func CalcBoxColors(colTop, colSide uint32) [6]uint32 {
	return [6]uint32{
		MultCol(colTop, 250),
		MultCol(colSide, 140),
		MultCol(colSide, 165),
		MultCol(colSide, 217),
		MultCol(colSide, 165),
		MultCol(colSide, 217),
	}
}

// CylinderWire draws a cylinder wireframe.
func CylinderWire(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	AppendCylinderWire(dd, minx, miny, minz, maxx, maxy, maxz, col)
	dd.End()
}

// BoxWire draws a box wireframe.
func BoxWire(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	AppendBoxWire(dd, minx, miny, minz, maxx, maxy, maxz, col)
	dd.End()
}

// Arc draws an arc line.
func Arc(dd DebugDraw, x0, y0, z0, x1, y1, z1, h, as0, as1 float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	AppendArc(dd, x0, y0, z0, x1, y1, z1, h, as0, as1, col)
	dd.End()
}

// Arrow draws an arrow.
func Arrow(dd DebugDraw, x0, y0, z0, x1, y1, z1, as0, as1 float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	AppendArrow(dd, x0, y0, z0, x1, y1, z1, as0, as1, col)
	dd.End()
}

// Circle draws a circle.
func Circle(dd DebugDraw, x, y, z, r float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	AppendCircle(dd, x, y, z, r, col)
	dd.End()
}

// Cross draws a cross.
func Cross(dd DebugDraw, x, y, z, size float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	AppendCross(dd, x, y, z, size, col)
	dd.End()
}

// Box draws a filled box.
func Box(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, fcol []uint32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawQuads, 1.0)
	AppendBox(dd, minx, miny, minz, maxx, maxy, maxz, fcol)
	dd.End()
}

// Cylinder draws a filled cylinder.
func Cylinder(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawTris, 1.0)
	AppendCylinder(dd, minx, miny, minz, maxx, maxy, maxz, col)
	dd.End()
}

// GridXZ draws a grid on the XZ plane.
func GridXZ(dd DebugDraw, ox, oy, oz float32, w, h int, size float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}
	dd.Begin(DrawLines, lineWidth)
	for i := 0; i <= h; i++ {
		dd.VertexXYZColor(ox, oy, oz+float32(i)*size, col)
		dd.VertexXYZColor(ox+float32(w)*size, oy, oz+float32(i)*size, col)
	}
	for i := 0; i <= w; i++ {
		dd.VertexXYZColor(ox+float32(i)*size, oy, oz, col)
		dd.VertexXYZColor(ox+float32(i)*size, oy, oz+float32(h)*size, col)
	}
	dd.End()
}

// Append functions (no begin/end)

var (
	cylDir    [16 * 2]float32
	cylInit   bool
	circDir   [40 * 2]float32
	circInit  bool
	numArcPts = 8
)

// AppendCylinderWire appends cylinder wireframe vertices.
func AppendCylinderWire(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	if !cylInit {
		cylInit = true
		for i := 0; i < 16; i++ {
			a := float32(i) / 16.0 * Pi * 2
			cylDir[i*2] = cos(a)
			cylDir[i*2+1] = sin(a)
		}
	}
	cx := (maxx + minx) / 2
	cz := (maxz + minz) / 2
	rx := (maxx - minx) / 2
	rz := (maxz - minz) / 2

	for i, j := 0, 15; i < 16; j, i = i, i+1 {
		dd.VertexXYZColor(cx+cylDir[j*2]*rx, miny, cz+cylDir[j*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[i*2]*rx, miny, cz+cylDir[i*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[j*2]*rx, maxy, cz+cylDir[j*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[i*2]*rx, maxy, cz+cylDir[i*2+1]*rz, col)
	}
	for i := 0; i < 16; i += 4 {
		dd.VertexXYZColor(cx+cylDir[i*2]*rx, miny, cz+cylDir[i*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[i*2]*rx, maxy, cz+cylDir[i*2+1]*rz, col)
	}
}

// AppendBoxWire appends box wireframe vertices.
func AppendBoxWire(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	// Top
	dd.VertexXYZColor(minx, miny, minz, col)
	dd.VertexXYZColor(maxx, miny, minz, col)
	dd.VertexXYZColor(maxx, miny, minz, col)
	dd.VertexXYZColor(maxx, miny, maxz, col)
	dd.VertexXYZColor(maxx, miny, maxz, col)
	dd.VertexXYZColor(minx, miny, maxz, col)
	dd.VertexXYZColor(minx, miny, maxz, col)
	dd.VertexXYZColor(minx, miny, minz, col)

	// bottom
	dd.VertexXYZColor(minx, maxy, minz, col)
	dd.VertexXYZColor(maxx, maxy, minz, col)
	dd.VertexXYZColor(maxx, maxy, minz, col)
	dd.VertexXYZColor(maxx, maxy, maxz, col)
	dd.VertexXYZColor(maxx, maxy, maxz, col)
	dd.VertexXYZColor(minx, maxy, maxz, col)
	dd.VertexXYZColor(minx, maxy, maxz, col)
	dd.VertexXYZColor(minx, maxy, minz, col)

	// Sides
	dd.VertexXYZColor(minx, miny, minz, col)
	dd.VertexXYZColor(minx, maxy, minz, col)
	dd.VertexXYZColor(maxx, miny, minz, col)
	dd.VertexXYZColor(maxx, maxy, minz, col)
	dd.VertexXYZColor(maxx, miny, maxz, col)
	dd.VertexXYZColor(maxx, maxy, maxz, col)
	dd.VertexXYZColor(minx, miny, maxz, col)
	dd.VertexXYZColor(minx, maxy, maxz, col)
}

// AppendBoxPoints appends box point vertices (wireframe with all edges).
func AppendBoxPoints(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	// Top
	dd.VertexXYZColor(minx, miny, minz, col)
	dd.VertexXYZColor(maxx, miny, minz, col)
	dd.VertexXYZColor(maxx, miny, minz, col)
	dd.VertexXYZColor(maxx, miny, maxz, col)
	dd.VertexXYZColor(maxx, miny, maxz, col)
	dd.VertexXYZColor(minx, miny, maxz, col)
	dd.VertexXYZColor(minx, miny, maxz, col)
	dd.VertexXYZColor(minx, miny, minz, col)

	// bottom
	dd.VertexXYZColor(minx, maxy, minz, col)
	dd.VertexXYZColor(maxx, maxy, minz, col)
	dd.VertexXYZColor(maxx, maxy, minz, col)
	dd.VertexXYZColor(maxx, maxy, maxz, col)
	dd.VertexXYZColor(maxx, maxy, maxz, col)
	dd.VertexXYZColor(minx, maxy, maxz, col)
	dd.VertexXYZColor(minx, maxy, maxz, col)
	dd.VertexXYZColor(minx, maxy, minz, col)
}

// AppendBox appends filled box quad vertices.
func AppendBox(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, fcol []uint32) {
	if dd == nil {
		return
	}
	verts := [8][3]float32{
		{minx, miny, minz},
		{maxx, miny, minz},
		{maxx, miny, maxz},
		{minx, miny, maxz},
		{minx, maxy, minz},
		{maxx, maxy, minz},
		{maxx, maxy, maxz},
		{minx, maxy, maxz},
	}
	inds := [6][4]int{
		{7, 6, 5, 4},
		{0, 1, 2, 3},
		{1, 5, 6, 2},
		{3, 7, 4, 0},
		{2, 6, 7, 3},
		{0, 4, 5, 1},
	}
	for i := 0; i < 6; i++ {
		dd.VertexXYZColor(verts[inds[i][0]][0], verts[inds[i][0]][1], verts[inds[i][0]][2], fcol[i])
		dd.VertexXYZColor(verts[inds[i][1]][0], verts[inds[i][1]][1], verts[inds[i][1]][2], fcol[i])
		dd.VertexXYZColor(verts[inds[i][2]][0], verts[inds[i][2]][1], verts[inds[i][2]][2], fcol[i])
		dd.VertexXYZColor(verts[inds[i][3]][0], verts[inds[i][3]][1], verts[inds[i][3]][2], fcol[i])
	}
}

// AppendCylinder appends filled cylinder triangle vertices.
func AppendCylinder(dd DebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	if !cylInit {
		cylInit = true
		for i := 0; i < 16; i++ {
			a := float32(i) / 16.0 * Pi * 2
			cylDir[i*2] = cos(a)
			cylDir[i*2+1] = sin(a)
		}
	}

	col2 := MultCol(col, 160)
	cx := (maxx + minx) / 2
	cz := (maxz + minz) / 2
	rx := (maxx - minx) / 2
	rz := (maxz - minz) / 2

	for i := 2; i < 16; i++ {
		a, b, c := 0, i-1, i
		dd.VertexXYZColor(cx+cylDir[a*2]*rx, miny, cz+cylDir[a*2+1]*rz, col2)
		dd.VertexXYZColor(cx+cylDir[b*2]*rx, miny, cz+cylDir[b*2+1]*rz, col2)
		dd.VertexXYZColor(cx+cylDir[c*2]*rx, miny, cz+cylDir[c*2+1]*rz, col2)
	}
	for i := 2; i < 16; i++ {
		a, b, c := 0, i, i-1
		dd.VertexXYZColor(cx+cylDir[a*2]*rx, maxy, cz+cylDir[a*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[b*2]*rx, maxy, cz+cylDir[b*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[c*2]*rx, maxy, cz+cylDir[c*2+1]*rz, col)
	}
	for i, j := 0, 15; i < 16; j, i = i, i+1 {
		dd.VertexXYZColor(cx+cylDir[i*2]*rx, miny, cz+cylDir[i*2+1]*rz, col2)
		dd.VertexXYZColor(cx+cylDir[j*2]*rx, miny, cz+cylDir[j*2+1]*rz, col2)
		dd.VertexXYZColor(cx+cylDir[j*2]*rx, maxy, cz+cylDir[j*2+1]*rz, col)

		dd.VertexXYZColor(cx+cylDir[i*2]*rx, miny, cz+cylDir[i*2+1]*rz, col2)
		dd.VertexXYZColor(cx+cylDir[j*2]*rx, maxy, cz+cylDir[j*2+1]*rz, col)
		dd.VertexXYZColor(cx+cylDir[i*2]*rx, maxy, cz+cylDir[i*2+1]*rz, col)
	}
}

func evalArc(x0, y0, z0, dx, dy, dz, h, u float32) (float32, float32, float32) {
	resX := x0 + dx*u
	resY := y0 + dy*u + h*(1-(u*2-1)*(u*2-1))
	resZ := z0 + dz*u
	return resX, resY, resZ
}

func vecCross(v1, v2 [3]float32) [3]float32 {
	return [3]float32{
		v1[1]*v2[2] - v1[2]*v2[1],
		v1[2]*v2[0] - v1[0]*v2[2],
		v1[0]*v2[1] - v1[1]*v2[0],
	}
}

func vecNormalize(v [3]float32) [3]float32 {
	d := 1.0 / sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
	return [3]float32{v[0] * d, v[1] * d, v[2] * d}
}

func vecSub(v1, v2 [3]float32) [3]float32 {
	return [3]float32{v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]}
}

func vecDistSqr(v1, v2 [3]float32) float32 {
	x := v1[0] - v2[0]
	y := v1[1] - v2[1]
	z := v1[2] - v2[2]
	return x*x + y*y + z*z
}

func appendArrowHead(dd DebugDraw, p, q [3]float32, s float32, col uint32) {
	const eps = 0.001
	if dd == nil {
		return
	}
	if vecDistSqr(p, q) < eps*eps {
		return
	}
	ay := [3]float32{0, 1, 0}

	az := vecSub(q, p)
	az = vecNormalize(az)
	ax := vecCross(ay, az)
	ay = vecCross(az, ax)
	ay = vecNormalize(ay)

	dd.VertexPosColor(p, col)
	dd.VertexXYZColor(p[0]+az[0]*s+ax[0]*s/3, p[1]+az[1]*s+ax[1]*s/3, p[2]+az[2]*s+ax[2]*s/3, col)

	dd.VertexPosColor(p, col)
	dd.VertexXYZColor(p[0]+az[0]*s-ax[0]*s/3, p[1]+az[1]*s-ax[1]*s/3, p[2]+az[2]*s-ax[2]*s/3, col)
}

// AppendArc appends arc line vertices.
func AppendArc(dd DebugDraw, x0, y0, z0, x1, y1, z1, h, as0, as1 float32, col uint32) {
	if dd == nil {
		return
	}
	dx := x1 - x0
	dy := y1 - y0
	dz := z1 - z0
	length := sqrt(dx*dx + dy*dy + dz*dz)

	const pad = 0.05
	arcPtsScale := (1.0 - pad*2) / float32(numArcPts)

	px, py, pz := evalArc(x0, y0, z0, dx, dy, dz, length*h, pad)
	for i := 1; i <= numArcPts; i++ {
		u := pad + float32(i)*arcPtsScale
		ptx, pty, ptz := evalArc(x0, y0, z0, dx, dy, dz, length*h, u)
		dd.VertexXYZColor(px, py, pz, col)
		dd.VertexXYZColor(ptx, pty, ptz, col)
		px, py, pz = ptx, pty, ptz
	}

	// End arrows
	if as0 > 0.001 {
		px, py, pz = evalArc(x0, y0, z0, dx, dy, dz, length*h, pad)
		qx, qy, qz := evalArc(x0, y0, z0, dx, dy, dz, length*h, pad+0.05)
		appendArrowHead(dd, [3]float32{px, py, pz}, [3]float32{qx, qy, qz}, as0, col)
	}
	if as1 > 0.001 {
		px, py, pz = evalArc(x0, y0, z0, dx, dy, dz, length*h, 1-pad)
		qx, qy, qz := evalArc(x0, y0, z0, dx, dy, dz, length*h, 1-(pad+0.05))
		appendArrowHead(dd, [3]float32{px, py, pz}, [3]float32{qx, qy, qz}, as1, col)
	}
}

// AppendArrow appends arrow line vertices.
func AppendArrow(dd DebugDraw, x0, y0, z0, x1, y1, z1, as0, as1 float32, col uint32) {
	if dd == nil {
		return
	}
	dd.VertexXYZColor(x0, y0, z0, col)
	dd.VertexXYZColor(x1, y1, z1, col)

	p := [3]float32{x0, y0, z0}
	q := [3]float32{x1, y1, z1}
	if as0 > 0.001 {
		appendArrowHead(dd, p, q, as0, col)
	}
	if as1 > 0.001 {
		appendArrowHead(dd, q, p, as1, col)
	}
}

// AppendCircle appends circle line vertices.
func AppendCircle(dd DebugDraw, x, y, z, r float32, col uint32) {
	if dd == nil {
		return
	}
	if !circInit {
		circInit = true
		for i := 0; i < 40; i++ {
			a := float32(i) / 40.0 * Pi * 2
			circDir[i*2] = cos(a)
			circDir[i*2+1] = sin(a)
		}
	}
	for i, j := 0, 39; i < 40; j, i = i, i+1 {
		dd.VertexXYZColor(x+circDir[j*2]*r, y, z+circDir[j*2+1]*r, col)
		dd.VertexXYZColor(x+circDir[i*2]*r, y, z+circDir[i*2+1]*r, col)
	}
}

// AppendCross appends cross line vertices.
func AppendCross(dd DebugDraw, x, y, z, s float32, col uint32) {
	if dd == nil {
		return
	}
	dd.VertexXYZColor(x-s, y, z, col)
	dd.VertexXYZColor(x+s, y, z, col)
	dd.VertexXYZColor(x, y-s, z, col)
	dd.VertexXYZColor(x, y+s, z, col)
	dd.VertexXYZColor(x, y, z-s, col)
	dd.VertexXYZColor(x, y, z+s, col)
}

// DisplayList stores a list of debug draw commands for later replay.
type DisplayList struct {
	pos       []float32
	color     []uint32
	size      int
	cap       int
	prim      DrawPrimitives
	primSize  float32
	depthMask bool
}

// NewDisplayList creates a new display list.
func NewDisplayList(cap int) *DisplayList {
	if cap < 8 {
		cap = 8
	}
	return &DisplayList{
		pos:       make([]float32, cap*3),
		color:     make([]uint32, cap),
		size:      0,
		cap:       cap,
		prim:      DrawLines,
		primSize:  1.0,
		depthMask: true,
	}
}

func (dl *DisplayList) resize(cap int) {
	newPos := make([]float32, cap*3)
	copy(newPos, dl.pos[:dl.size*3])
	newColor := make([]uint32, cap)
	copy(newColor, dl.color[:dl.size])
	dl.pos = newPos
	dl.color = newColor
	dl.cap = cap
}

// DepthMask sets depth mask state.
func (dl *DisplayList) DepthMask(state bool) {
	dl.depthMask = state
}

// Texture is a no-op for DisplayList.
func (dl *DisplayList) Texture(state bool) {}

// Begin starts recording primitives.
func (dl *DisplayList) Begin(prim DrawPrimitives, size float32) {
	dl.Clear()
	dl.prim = prim
	dl.primSize = size
}

// VertexPosColor records a vertex with position array and color.
func (dl *DisplayList) VertexPosColor(pos [3]float32, color uint32) {
	dl.VertexXYZColor(pos[0], pos[1], pos[2], color)
}

// VertexXYZColor records a vertex with x,y,z and color.
func (dl *DisplayList) VertexXYZColor(x, y, z float32, color uint32) {
	if dl.size+1 >= dl.cap {
		dl.resize(dl.cap * 2)
	}
	p := dl.pos[dl.size*3 : dl.size*3+3]
	p[0] = x
	p[1] = y
	p[2] = z
	dl.color[dl.size] = color
	dl.size++
}

// VertexPosColorUV records a vertex with UV (unused in display list).
func (dl *DisplayList) VertexPosColorUV(pos [3]float32, color uint32, uv [2]float32) {
	dl.VertexXYZColor(pos[0], pos[1], pos[2], color)
}

// VertexXYZColorUV records a vertex with UV (unused in display list).
func (dl *DisplayList) VertexXYZColorUV(x, y, z float32, color uint32, u, v float32) {
	dl.VertexXYZColor(x, y, z, color)
}

// End ends recording.
func (dl *DisplayList) End() {}

// AreaToCol returns color for area (uses IntToCol).
func (dl *DisplayList) AreaToCol(area uint32) uint32 {
	return IntToCol(int(area), 255)
}

// Clear clears the display list.
func (dl *DisplayList) Clear() {
	dl.size = 0
}

// Draw replays the display list to the given debug draw interface.
func (dl *DisplayList) Draw(dd DebugDraw) {
	if dd == nil || dl.size == 0 {
		return
	}
	dd.DepthMask(dl.depthMask)
	dd.Begin(dl.prim, dl.primSize)
	for i := 0; i < dl.size; i++ {
		p := [3]float32{dl.pos[i*3], dl.pos[i*3+1], dl.pos[i*3+2]}
		dd.VertexPosColor(p, dl.color[i])
	}
	dd.End()
}
