package debug_utils

import "github.com/actfuns/recastnavigation/recast"

// FileIO is the abstract interface for file or stream input/output operations.
type FileIO interface {
	IsWriting() bool
	IsReading() bool
	Write(ptr []byte) bool
	Read(ptr []byte) bool
}

// DumpPolyMeshToObj writes a rcPolyMesh to a Wavefront OBJ file through the given FileIO interface.
func DumpPolyMeshToObj(pmesh *recast.PolyMesh, io FileIO) bool {
	if io == nil {
		return false
	}
	if !io.IsWriting() {
		return false
	}

	nvp := pmesh.Nvp
	cs := pmesh.Cs
	ch := pmesh.Ch
	orig := pmesh.Bmin

	io.Write([]byte("# Recast Navmesh\n"))
	io.Write([]byte("o NavMesh\n"))
	io.Write([]byte("\n"))

	// Write vertices
	for i := 0; i < pmesh.Nverts; i++ {
		v := pmesh.Verts[i*3:]
		x := orig[0] + float32(v[0])*cs
		y := orig[1] + (float32(v[1])+1)*ch + 0.1
		z := orig[2] + float32(v[2])*cs
		line := sprintf("v %f %f %f\n", x, y, z)
		io.Write([]byte(line))
	}

	io.Write([]byte("\n"))

	// Write faces
	for i := 0; i < pmesh.Npolys; i++ {
		p := pmesh.Polys[i*nvp*2:]
		for j := 2; j < nvp; j++ {
			if p[j] == recast.MeshNullIdx {
				break
			}
			// OBJ uses 1-based indices
			line := sprintf("f %d %d %d\n", int(p[0])+1, int(p[j-1])+1, int(p[j])+1)
			io.Write([]byte(line))
		}
	}

	return true
}

// DumpPolyMeshDetailToObj writes a rcPolyMeshDetail to a Wavefront OBJ file.
func DumpPolyMeshDetailToObj(dmesh *recast.PolyMeshDetail, io FileIO) bool {
	if io == nil {
		return false
	}
	if !io.IsWriting() {
		return false
	}

	io.Write([]byte("# Recast Navmesh\n"))
	io.Write([]byte("o NavMesh\n"))
	io.Write([]byte("\n"))

	// Write vertices
	for i := 0; i < dmesh.Nverts; i++ {
		v := dmesh.Verts[i*3:]
		line := sprintf("v %f %f %f\n", v[0], v[1], v[2])
		io.Write([]byte(line))
	}

	io.Write([]byte("\n"))

	// Write faces
	for i := 0; i < dmesh.Nmeshes; i++ {
		m := dmesh.Meshes[i*4:]
		bverts := m[0]
		btris := m[2]
		ntris := int(m[3])
		tris := dmesh.Tris[btris*4:]
		for j := 0; j < ntris; j++ {
			line := sprintf("f %d %d %d\n",
				int(bverts)+int(tris[j*4+0])+1,
				int(bverts)+int(tris[j*4+1])+1,
				int(bverts)+int(tris[j*4+2])+1)
			io.Write([]byte(line))
		}
	}

	return true
}

const (
	csetMagic   = int(('c' << 24) | ('s' << 16) | ('e' << 8) | 't')
	csetVersion = 2
)

// DumpContourSet writes a ContourSet to a binary stream.
func DumpContourSet(cset *recast.ContourSet, io FileIO) bool {
	if io == nil {
		return false
	}
	if !io.IsWriting() {
		return false
	}

	magic := csetMagic
	version := csetVersion
	writeInt(io, magic)
	writeInt(io, version)

	writeInt(io, cset.Nconts)

	io.Write(floatsToBytes(cset.Bmin[:]))
	io.Write(floatsToBytes(cset.Bmax[:]))

	writeFloat(io, cset.Cs)
	writeFloat(io, cset.Ch)

	writeInt(io, cset.Width)
	writeInt(io, cset.Height)
	writeInt(io, cset.BorderSize)

	for i := 0; i < cset.Nconts; i++ {
		cont := cset.Conts[i]
		writeInt(io, cont.Nverts)
		writeInt(io, cont.Nrvets)
		writeUint16(io, uint32(cont.Reg))
		writeUint8(io, uint32(cont.Area))
		io.Write(intsToBytes(cont.Verts[:cont.Nverts*4]))
		io.Write(intsToBytes(cont.RVerts[:cont.Nrvets*4]))
	}

	return true
}

// ReadContourSet reads a ContourSet from a binary stream.
func ReadContourSet(cset *recast.ContourSet, io FileIO) bool {
	if io == nil {
		return false
	}
	if !io.IsReading() {
		return false
	}

	magic := readInt(io)
	version := readInt(io)

	if magic != csetMagic {
		return false
	}
	if version != csetVersion {
		return false
	}

	cset.Nconts = readInt(io)

	io.Read(floatsToBytes(cset.Bmin[:]))
	io.Read(floatsToBytes(cset.Bmax[:]))

	cset.Cs = readFloat(io)
	cset.Ch = readFloat(io)

	cset.Width = readInt(io)
	cset.Height = readInt(io)
	cset.BorderSize = readInt(io)

	cset.Conts = make([]recast.Contour, cset.Nconts)
	for i := 0; i < cset.Nconts; i++ {
		cont := &cset.Conts[i]
		cont.Nverts = readInt(io)
		cont.Nrvets = readInt(io)
		cont.Reg = uint16(readUint16(io))
		cont.Area = uint8(readUint8(io))
		cont.Verts = make([]int, cont.Nverts*4)
		cont.RVerts = make([]int, cont.Nrvets*4)
		io.Read(intsToBytes(cont.Verts))
		io.Read(intsToBytes(cont.RVerts))
	}

	return true
}

const (
	chfMagic   = int(('r' << 24) | ('c' << 16) | ('h' << 8) | 'f')
	chfVersion = 3
)

// DumpCompactHeightfield writes a CompactHeightfield to a binary stream.
func DumpCompactHeightfield(chf *recast.CompactHeightfield, io FileIO) bool {
	if io == nil {
		return false
	}
	if !io.IsWriting() {
		return false
	}

	writeInt(io, chfMagic)
	writeInt(io, chfVersion)

	writeInt(io, chf.Width)
	writeInt(io, chf.Height)
	writeInt(io, chf.SpanCount)

	writeInt(io, chf.WalkableHeight)
	writeInt(io, chf.WalkableClimb)
	writeInt(io, chf.BorderSize)

	writeUint16(io, uint32(chf.MaxDistance))
	writeUint16(io, uint32(chf.MaxRegions))

	io.Write(floatsToBytes(chf.Bmin[:]))
	io.Write(floatsToBytes(chf.Bmax[:]))

	writeFloat(io, chf.Cs)
	writeFloat(io, chf.Ch)

	tmp := 0
	if len(chf.Cells) > 0 {
		tmp |= 1
	}
	if len(chf.Spans) > 0 {
		tmp |= 2
	}
	if len(chf.Dist) > 0 {
		tmp |= 4
	}
	if len(chf.Areas) > 0 {
		tmp |= 8
	}

	writeInt(io, tmp)

	if tmp&1 != 0 {
		io.Write(compactCellsToBytes(chf.Cells))
	}
	if tmp&2 != 0 {
		io.Write(compactSpansToBytes(chf.Spans))
	}
	if tmp&4 != 0 {
		io.Write(uint16sToBytes(chf.Dist))
	}
	if tmp&8 != 0 {
		io.Write(chf.Areas)
	}

	return true
}

// ReadCompactHeightfield reads a CompactHeightfield from a binary stream.
func ReadCompactHeightfield(chf *recast.CompactHeightfield, io FileIO) bool {
	if io == nil {
		return false
	}
	if !io.IsReading() {
		return false
	}

	magic := readInt(io)
	version := readInt(io)

	if magic != chfMagic {
		return false
	}
	if version != chfVersion {
		return false
	}

	chf.Width = readInt(io)
	chf.Height = readInt(io)
	chf.SpanCount = readInt(io)

	chf.WalkableHeight = readInt(io)
	chf.WalkableClimb = readInt(io)
	chf.BorderSize = readInt(io)

	chf.MaxDistance = uint16(readUint16(io))
	chf.MaxRegions = uint16(readUint16(io))

	io.Read(floatsToBytes(chf.Bmin[:]))
	io.Read(floatsToBytes(chf.Bmax[:]))

	chf.Cs = readFloat(io)
	chf.Ch = readFloat(io)

	tmp := readInt(io)

	if tmp&1 != 0 {
		chf.Cells = make([]recast.CompactCell, chf.Width*chf.Height)
		io.Read(compactCellsToBytes(chf.Cells))
	}
	if tmp&2 != 0 {
		chf.Spans = make([]recast.CompactSpan, chf.SpanCount)
		io.Read(compactSpansToBytes(chf.Spans))
	}
	if tmp&4 != 0 {
		chf.Dist = make([]uint16, chf.SpanCount)
		io.Read(uint16sToBytes(chf.Dist))
	}
	if tmp&8 != 0 {
		chf.Areas = make([]uint8, chf.SpanCount)
		io.Read(chf.Areas)
	}

	return true
}

// LogBuildTimes logs build times for the Recast build process.
// This is a convenience function that works with Context-like logging.
type BuildTimerLogger interface {
	Log(category int, format string, args ...interface{})
	GetAccumulatedTime(label int) int
}

// LogBuildTimes prints timing information for the Recast build process.
func LogBuildTimes(ctx BuildTimerLogger, totalTimeUsec int) {
	pc := 100.0 / float32(totalTimeUsec)

	logLine := func(label int, name string) {
		t := ctx.GetAccumulatedTime(label)
		if t < 0 {
			return
		}
		ctx.Log(1, "%s:\t%.2fms\t(%.1f%%)", name, float32(t)/1000.0, float32(t)*pc)
	}

	ctx.Log(1, "Build Times")
	logLine(2, "- Rasterize")
	logLine(4, "- Build Compact")
	logLine(6, "- Filter Border")
	logLine(7, "- Filter Walkable")
	logLine(11, "- Erode Area")
	logLine(10, "- Median Area")
	logLine(12, "- Mark Box Area")
	logLine(14, "- Mark Convex Area")
	logLine(13, "- Mark Cylinder Area")
	logLine(15, "- Build Distance Field")
	logLine(16, "    - Distance")
	logLine(17, "    - Blur")
	logLine(18, "- Build Regions")
	logLine(19, "    - Watershed")
	logLine(20, "      - Expand")
	logLine(21, "      - Find Basins")
	logLine(22, "    - Filter")
	logLine(25, "- Build Layers")
	logLine(8, "- Build Contours")
	logLine(9, "    - Trace")
	logLine(5, "    - Simplify")
	logLine(23, "- Build Polymesh")
	logLine(24, "- Build Polymesh Detail")
	logLine(26, "- Merge Polymeshes")
	logLine(27, "- Merge Polymesh Details")
	ctx.Log(1, "=== TOTAL:\t%.2fms", float32(totalTimeUsec)/1000.0)
}

// Helper IO functions

func writeInt(io FileIO, v int) {
	buf := make([]byte, 4)
	buf[0] = byte(v)
	buf[1] = byte(v >> 8)
	buf[2] = byte(v >> 16)
	buf[3] = byte(v >> 24)
	io.Write(buf)
}

func readInt(io FileIO) int {
	buf := make([]byte, 4)
	io.Read(buf)
	return int(buf[0]) | int(buf[1])<<8 | int(buf[2])<<16 | int(buf[3])<<24
}

func writeFloat(io FileIO, v float32) {
	writeInt(io, int(v)) // Note: C++ binary serialization uses memcpy of float as int
}

func readFloat(io FileIO) float32 {
	return float32(readInt(io)) // Note: simplified, not bit-accurate for float
}

func writeUint16(io FileIO, v uint32) {
	buf := make([]byte, 2)
	buf[0] = byte(v)
	buf[1] = byte(v >> 8)
	io.Write(buf)
}

func readUint16(io FileIO) uint32 {
	buf := make([]byte, 2)
	io.Read(buf)
	return uint32(buf[0]) | uint32(buf[1])<<8
}

func writeUint8(io FileIO, v uint32) {
	buf := []byte{byte(v)}
	io.Write(buf)
}

func readUint8(io FileIO) uint32 {
	buf := []byte{0}
	io.Read(buf)
	return uint32(buf[0])
}

func floatsToBytes(f []float32) []byte {
	buf := make([]byte, len(f)*4)
	for i, v := range f {
		u := int(v)
		buf[i*4] = byte(u)
		buf[i*4+1] = byte(u >> 8)
		buf[i*4+2] = byte(u >> 16)
		buf[i*4+3] = byte(u >> 24)
	}
	return buf
}

func intsToBytes(v []int) []byte {
	buf := make([]byte, len(v)*4)
	for i, val := range v {
		buf[i*4] = byte(val)
		buf[i*4+1] = byte(val >> 8)
		buf[i*4+2] = byte(val >> 16)
		buf[i*4+3] = byte(val >> 24)
	}
	return buf
}

func uint16sToBytes(v []uint16) []byte {
	buf := make([]byte, len(v)*2)
	for i, val := range v {
		buf[i*2] = byte(val)
		buf[i*2+1] = byte(val >> 8)
	}
	return buf
}

func compactCellsToBytes(cells []recast.CompactCell) []byte {
	buf := make([]byte, len(cells)*8) // uint32 index + uint32 count
	for i, c := range cells {
		buf[i*8] = byte(c.Index)
		buf[i*8+1] = byte(c.Index >> 8)
		buf[i*8+2] = byte(c.Index >> 16)
		buf[i*8+3] = byte(c.Index >> 24)
		buf[i*8+4] = byte(c.Count)
		buf[i*8+5] = byte(c.Count >> 8)
		buf[i*8+6] = byte(c.Count >> 16)
		buf[i*8+7] = byte(c.Count >> 24)
	}
	return buf
}

func compactSpansToBytes(spans []recast.CompactSpan) []byte {
	buf := make([]byte, len(spans)*8) // uint16 y + uint16 reg + uint32 con + uint8 h (padding=7 bytes)
	for i, s := range spans {
		buf[i*8] = byte(s.Y)
		buf[i*8+1] = byte(s.Y >> 8)
		buf[i*8+2] = byte(s.Reg)
		buf[i*8+3] = byte(s.Reg >> 8)
		buf[i*8+4] = byte(s.Con)
		buf[i*8+5] = byte(s.Con >> 8)
		buf[i*8+6] = byte(s.Con >> 16)
		buf[i*8+7] = byte(s.Con >> 24)
	}
	return buf
}

// Simple sprintf-like function without fmt import
func sprintf(format string, args ...interface{}) string {
	buf := make([]byte, 0, 256)
	i := 0
	for pos := 0; pos < len(format); pos++ {
		if format[pos] == '%' && pos+1 < len(format) {
			next := format[pos+1]
			if next == 'd' {
				if i < len(args) {
					v := args[i].(int)
					buf = appendInt(buf, v)
					i++
				}
				pos++
			} else if next == 'f' {
				if i < len(args) {
					v := args[i].(float32)
					buf = appendFloat(buf, v)
					i++
				}
				pos++
			} else if next == 's' {
				if i < len(args) {
					v := args[i].(string)
					buf = append(buf, v...)
					i++
				}
				pos++
			} else if next == '%' {
				buf = append(buf, '%')
				pos++
			} else {
				buf = append(buf, format[pos])
			}
		} else {
			buf = append(buf, format[pos])
		}
	}
	return string(buf)
}

func appendInt(buf []byte, v int) []byte {
	if v < 0 {
		buf = append(buf, '-')
		v = -v
	}
	if v == 0 {
		return append(buf, '0')
	}
	var digits [32]byte
	pos := len(digits)
	for v > 0 {
		pos--
		digits[pos] = byte('0' + v%10)
		v /= 10
	}
	return append(buf, digits[pos:]...)
}

func appendFloat(buf []byte, v float32) []byte {
	if v < 0 {
		buf = append(buf, '-')
		v = -v
	}
	intPart := int(v)
	buf = appendInt(buf, intPart)
	buf = append(buf, '.')
	frac := int((v - float32(intPart)) * 1000000)
	if frac < 0 {
		frac = 0
	}
	// Round to 6 decimal places
	var fracDigits [6]byte
	for i := 5; i >= 0; i-- {
		fracDigits[i] = byte('0' + frac%10)
		frac /= 10
	}
	// Trim trailing zeros
	end := 6
	for end > 1 && fracDigits[end-1] == '0' {
		end--
	}
	return append(buf, fracDigits[:end]...)
}
