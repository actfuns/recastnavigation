package detour_tile_cache

import (
	"github.com/actfuns/recastnavigation/detour"
	"math"
	"unsafe"
)

// ---------------------------------------------------------------------------
// Alloc / Free helpers
// ---------------------------------------------------------------------------

// FreeTileCacheContourSet frees a contour set allocated by AllocTileCacheContourSet.
func FreeTileCacheContourSet(alloc TileCacheAlloc, cset *TileCacheContourSet) {
	if cset == nil {
		return
	}
	for i := 0; i < cset.NConts; i++ {
		alloc.Free(cset.Conts[i].Verts)
	}
	alloc.Free(unsafe.Slice((*uint8)(unsafe.Pointer(&cset.Conts[0])), len(cset.Conts)*int(unsafe.Sizeof(cset.Conts[0]))))
	alloc.Free(ptrToSlice(cset))
}

// AllocTileCacheContourSet allocates an empty contour set.
func AllocTileCacheContourSet(alloc TileCacheAlloc) *TileCacheContourSet {
	cset := &TileCacheContourSet{}
	return cset
}

// FreeTileCachePolyMesh frees a poly mesh allocated by AllocTileCachePolyMesh.
func FreeTileCachePolyMesh(alloc TileCacheAlloc, lmesh *TileCachePolyMesh) {
	if lmesh == nil {
		return
	}
	alloc.Free(sliceToBytes(lmesh.Verts))
	alloc.Free(sliceToBytes(lmesh.Polys))
	alloc.Free(sliceToBytes(lmesh.Flags))
	alloc.Free(sliceToBytes(lmesh.Areas))
	alloc.Free(ptrToSlice(lmesh))
}

// AllocTileCachePolyMesh allocates an empty poly mesh.
func AllocTileCachePolyMesh(alloc TileCacheAlloc) *TileCachePolyMesh {
	lmesh := &TileCachePolyMesh{}
	return lmesh
}

// ---------------------------------------------------------------------------
// BuildTileCacheLayer - compress and serialize a layer
// ---------------------------------------------------------------------------

// BuildTileCacheLayer compresses a tile cache layer into a byte buffer.
func BuildTileCacheLayer(comp TileCacheCompressor, header *TileCacheLayerHeader,
	heights, areas, cons []uint8) ([]uint8, int, error) {

	headerSize := TileCacheLayerHeaderSize()
	gridSize := int(header.Width) * int(header.Height)
	maxDataSize := headerSize + comp.MaxCompressedSize(gridSize*3)

	data := make([]uint8, maxDataSize)

	// Copy header
	hdrBytes := (*[56]uint8)(unsafe.Pointer(header))
	copy(data, hdrBytes[:headerSize])

	// Concatenate grid data for compression.
	bufferSize := gridSize * 3
	buffer := make([]uint8, bufferSize)
	copy(buffer, heights)
	copy(buffer[gridSize:], areas)
	copy(buffer[gridSize*2:], cons)

	// Compress
	compressed := data[headerSize:]
	var compressedSize int
	err := comp.Compress(buffer, compressed, &compressedSize)
	if err != nil {
		return nil, 0, err
	}

	return data, headerSize + compressedSize, nil
}

// ---------------------------------------------------------------------------
// FreeTileCacheLayer
// ---------------------------------------------------------------------------

// FreeTileCacheLayer frees a tile cache layer.
func FreeTileCacheLayer(alloc TileCacheAlloc, layer *TileCacheLayer) {
	alloc.Free(ptrToSlice(layer))
}

// ---------------------------------------------------------------------------
// DecompressTileCacheLayer
// ---------------------------------------------------------------------------

// DecompressTileCacheLayer decompresses a tile cache layer.
func DecompressTileCacheLayer(alloc TileCacheAlloc, comp TileCacheCompressor,
	compressed []uint8, compressedSize int) (*TileCacheLayer, error) {

	if len(compressed) < TileCacheLayerHeaderSize() {
		return nil, detour.ErrInvalidParam
	}

	compressedHeader := (*TileCacheLayerHeader)(unsafe.Pointer(&compressed[0]))
	if compressedHeader.Magic != TileCacheMagic {
		return nil, detour.ErrWrongMagic
	}
	if compressedHeader.Version != TileCacheVersion {
		return nil, detour.ErrWrongVersion
	}

	layerSize := Align4(int(unsafe.Sizeof(TileCacheLayer{})))
	headerSize := TileCacheLayerHeaderSize()
	gridSize := int(compressedHeader.Width) * int(compressedHeader.Height)
	bufferSize := layerSize + headerSize + gridSize*4

	buffer := alloc.Alloc(bufferSize)
	if buffer == nil {
		return nil, detour.ErrOutOfMemory
	}

	// Zero fill
	for i := range buffer {
		buffer[i] = 0
	}

	layer := (*TileCacheLayer)(unsafe.Pointer(&buffer[0]))
	header := (*TileCacheLayerHeader)(unsafe.Pointer(&buffer[layerSize]))
	grids := buffer[layerSize+headerSize:]

	// Copy header
	copy(ptrToSlice(header)[:headerSize], compressed[:headerSize])

	// Decompress grid
	var size int
	err := comp.Decompress(compressed[headerSize:compressedSize], grids, &size)
	if err != nil {
		alloc.Free(buffer)
		return nil, err
	}

	layer.Header = header
	layer.Heights = grids[:gridSize]
	layer.Areas = grids[gridSize : gridSize*2]
	layer.Cons = grids[gridSize*2 : gridSize*3]
	layer.Regs = grids[gridSize*3:]

	return layer, nil
}

// ---------------------------------------------------------------------------
// BuildTileCacheRegions
// ---------------------------------------------------------------------------

type dtLayerSweepSpan struct {
	ns  uint16
	id  uint8
	nei uint8
}

const dtLayerMaxNeis = 16

type dtLayerMonotoneRegion struct {
	area   int
	neis   [dtLayerMaxNeis]uint8
	nneis  uint8
	regId  uint8
	areaId uint8
}

// BuildTileCacheRegions partitions walkable area into monotone regions.
func BuildTileCacheRegions(alloc TileCacheAlloc, layer *TileCacheLayer, walkableClimb int) error {
	w := int(layer.Header.Width)
	h := int(layer.Header.Height)

	// Initialize regs to 0xff
	for i := range layer.Regs {
		layer.Regs[i] = 0xff
	}

	nsweeps := w
	sweeps := make([]dtLayerSweepSpan, nsweeps)

	var regId uint8 = 0

	for y := 0; y < h; y++ {
		var prevCount [256]uint8
		if regId > 0 {
			for i := 0; i < int(regId); i++ {
				prevCount[i] = 0
			}
		}
		var sweepId uint8 = 0

		for x := 0; x < w; x++ {
			idx := x + y*w
			if layer.Areas[idx] == TileCacheNullArea {
				continue
			}

			var sid uint8 = 0xff

			// -x
			if x > 0 {
				xidx := (x - 1) + y*w
				if isConnected(layer, idx, xidx, walkableClimb) {
					if layer.Regs[xidx] != 0xff {
						sid = layer.Regs[xidx]
					}
				}
			}

			if sid == 0xff {
				sid = sweepId
				sweepId++
				sweeps[sid].nei = 0xff
				sweeps[sid].ns = 0
			}

			// -y
			if y > 0 {
				yidx := x + (y-1)*w
				if isConnected(layer, idx, yidx, walkableClimb) {
					nr := layer.Regs[yidx]
					if nr != 0xff {
						if sweeps[sid].ns == 0 {
							sweeps[sid].nei = nr
						}
						if sweeps[sid].nei == nr {
							sweeps[sid].ns++
							prevCount[nr]++
						} else {
							sweeps[sid].nei = 0xff
						}
					}
				}
			}

			layer.Regs[idx] = sid
		}

		// Create unique ID.
		for i := uint8(0); i < sweepId; i++ {
			if sweeps[i].nei != 0xff && uint16(prevCount[sweeps[i].nei]) == sweeps[i].ns {
				sweeps[i].id = sweeps[i].nei
			} else {
				if regId == 255 {
					return detour.ErrBufferTooSmall
				}
				sweeps[i].id = regId
				regId++
			}
		}

		// Remap local sweep ids to region ids.
		for x := 0; x < w; x++ {
			idx := x + y*w
			if layer.Regs[idx] != 0xff {
				layer.Regs[idx] = sweeps[layer.Regs[idx]].id
			}
		}
	}

	// Allocate and init layer regions.
	nregs := int(regId)
	regs := make([]dtLayerMonotoneRegion, nregs)
	for i := 0; i < nregs; i++ {
		regs[i].regId = 0xff
	}

	// Find region neighbours.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			idx := x + y*w
			ri := layer.Regs[idx]
			if ri == 0xff {
				continue
			}

			regs[ri].area++
			regs[ri].areaId = layer.Areas[idx]

			if y > 0 {
				ymi := x + (y-1)*w
				if isConnected(layer, idx, ymi, walkableClimb) {
					rai := layer.Regs[ymi]
					if rai != 0xff && rai != ri {
						addUniqueLast(&regs[ri].neis, &regs[ri].nneis, rai)
						addUniqueLast(&regs[rai].neis, &regs[rai].nneis, ri)
					}
				}
			}
		}
	}

	for i := 0; i < nregs; i++ {
		regs[i].regId = uint8(i)
	}

	// Merge regions.
	for i := 0; i < nregs; i++ {
		reg := &regs[i]

		merge := -1
		mergea := 0
		for j := 0; j < int(reg.nneis); j++ {
			nei := reg.neis[j]
			regn := &regs[nei]
			if reg.regId == regn.regId {
				continue
			}
			if reg.areaId != regn.areaId {
				continue
			}
			if regn.area > mergea {
				if canMerge(reg.regId, regn.regId, regs) {
					mergea = regn.area
					merge = int(nei)
				}
			}
		}
		if merge != -1 {
			oldId := reg.regId
			newId := regs[merge].regId
			for j := 0; j < nregs; j++ {
				if regs[j].regId == oldId {
					regs[j].regId = newId
				}
			}
		}
	}

	// Compact ids.
	var remap [256]uint8
	for i := 0; i < nregs; i++ {
		remap[regs[i].regId] = 1
	}
	regId = 0
	for i := 0; i < 256; i++ {
		if remap[i] != 0 {
			remap[i] = regId
			regId++
		}
	}
	for i := 0; i < nregs; i++ {
		regs[i].regId = remap[regs[i].regId]
	}

	layer.RegCount = regId

	for i := 0; i < w*h; i++ {
		if layer.Regs[i] != 0xff {
			layer.Regs[i] = regs[layer.Regs[i]].regId
		}
	}

	return nil
}

// ---------------------------------------------------------------------------
// BuildTileCacheContours
// ---------------------------------------------------------------------------

type dtTempContour struct {
	verts  []uint8
	nverts int
	cverts int
	poly   []uint16
	npoly  int
	cpoly  int
}

type rcEdge struct {
	vert     [2]uint16
	polyEdge [2]uint16
	poly     [2]uint16
}

// BuildTileCacheContours builds contours from a tile cache layer.
func BuildTileCacheContours(alloc TileCacheAlloc, layer *TileCacheLayer,
	walkableClimb int, maxError float32, lcset *TileCacheContourSet) error {

	w := int(layer.Header.Width)
	h := int(layer.Header.Height)

	lcset.NConts = int(layer.RegCount)
	lcset.Conts = make([]TileCacheContour, lcset.NConts)

	// Allocate temp buffer for contour tracing.
	maxTempVerts := (w + h) * 2 * 2

	tempVerts := alloc.Alloc(maxTempVerts * 4)
	if tempVerts == nil {
		return detour.ErrOutOfMemory
	}
	defer alloc.Free(tempVerts)

	tempPolyBytes := alloc.Alloc(maxTempVerts * 2)
	if tempPolyBytes == nil {
		return detour.ErrOutOfMemory
	}
	defer alloc.Free(tempPolyBytes)

	tempPoly16 := unsafe.Slice((*uint16)(unsafe.Pointer(&tempPolyBytes[0])), maxTempVerts)
	temp := dtTempContour{
		verts:  tempVerts,
		cverts: maxTempVerts * 4,
		poly:   tempPoly16,
		cpoly:  maxTempVerts,
	}

	// Find contours.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			idx := x + y*w
			ri := layer.Regs[idx]
			if ri == 0xff {
				continue
			}

			cont := &lcset.Conts[ri]

			if cont.NVerts > 0 {
				continue
			}

			cont.Reg = ri
			cont.Area = layer.Areas[idx]

			if !walkContour(layer, x, y, &temp) {
				return detour.ErrBufferTooSmall
			}

			simplifyContour(&temp, maxError)

			// Store contour.
			cont.NVerts = temp.nverts
			if cont.NVerts > 0 {
				cont.Verts = alloc.Alloc(4 * temp.nverts)
				if cont.Verts == nil {
					return detour.ErrOutOfMemory
				}

				for i, j := 0, temp.nverts-1; i < temp.nverts; j, i = i, i+1 {
					dst := cont.Verts[j*4:]
					v := temp.verts[j*4:]
					vn := temp.verts[i*4:]
					nei := vn[3] // The neighbour reg is stored at segment vertex of a segment.
					shouldRemove := false
					lh := getCornerHeight(layer, int(v[0]), int(v[1]), int(v[2]),
						walkableClimb, &shouldRemove)

					dst[0] = v[0]
					dst[1] = lh
					dst[2] = v[2]

					// Store portal direction and remove err to the fourth component.
					dst[3] = 0x0f
					if nei != 0xff && nei >= 0xf8 {
						dst[3] = nei - 0xf8
					}
					if shouldRemove {
						dst[3] |= 0x80
					}
				}
			}
		}
	}

	return nil
}

// ---------------------------------------------------------------------------
// BuildTileCachePolyMesh
// ---------------------------------------------------------------------------

const vertexBucketCount2 = 1 << 8

func computeVertexHash2(x, y, z int) int {
	const h1 uint32 = 0x8da6b343
	const h2 uint32 = 0xd8163841
	const h3 uint32 = 0xcb1ab31f
	n := h1*uint32(x) + h2*uint32(y) + h3*uint32(z)
	return int(n & (vertexBucketCount2 - 1))
}

func addVertex(x, y, z uint16, verts []uint16, firstVert []uint16, nextVert []uint16, nv *int) uint16 {
	bucket := computeVertexHash2(int(x), 0, int(z))
	i := firstVert[bucket]

	for i != TileCacheNullIdx {
		v := verts[int(i)*3:]
		if v[0] == x && v[2] == z && absInt(int(v[1])-int(y)) <= 2 {
			return i
		}
		i = nextVert[i]
	}

	// Could not find, create new.
	i = uint16(*nv)
	*nv++
	v := verts[int(i)*3:]
	v[0] = x
	v[1] = y
	v[2] = z
	nextVert[i] = firstVert[bucket]
	firstVert[bucket] = i

	return i
}

// BuildTileCachePolyMesh builds a polygon mesh from a contour set.
func BuildTileCachePolyMesh(alloc TileCacheAlloc, lcset *TileCacheContourSet, mesh *TileCachePolyMesh) error {
	maxVertices := 0
	maxTris := 0
	maxVertsPerCont := 0
	for i := 0; i < lcset.NConts; i++ {
		if lcset.Conts[i].NVerts < 3 {
			continue
		}
		maxVertices += lcset.Conts[i].NVerts
		maxTris += lcset.Conts[i].NVerts - 2
		if lcset.Conts[i].NVerts > maxVertsPerCont {
			maxVertsPerCont = lcset.Conts[i].NVerts
		}
	}

	mesh.Nvp = maxVertsPerPoly
	mesh.NVerts = 0
	mesh.NPolys = 0

	vflags := make([]uint8, maxVertices)
	mesh.Verts = make([]uint16, maxVertices*3)
	mesh.Polys = make([]uint16, maxTris*maxVertsPerPoly*2)
	mesh.Areas = make([]uint8, maxTris)
	mesh.Flags = make([]uint16, maxTris)

	// Initialize polys to 0xffff
	for i := range mesh.Polys {
		mesh.Polys[i] = TileCacheNullIdx
	}

	firstVert := make([]uint16, vertexBucketCount2)
	for i := range firstVert {
		firstVert[i] = TileCacheNullIdx
	}

	nextVert := make([]uint16, maxVertices)
	indices := make([]uint16, maxVertsPerCont)
	tris := make([]uint16, maxVertsPerCont*3)
	polys := make([]uint16, maxVertsPerCont*maxVertsPerPoly)

	for i := 0; i < lcset.NConts; i++ {
		cont := &lcset.Conts[i]

		if cont.NVerts < 3 {
			continue
		}

		// Triangulate contour
		for j := 0; j < cont.NVerts; j++ {
			indices[j] = uint16(j)
		}

		ntris := triangulate(cont.NVerts, cont.Verts, indices, tris)
		if ntris <= 0 {
			ntris = -ntris
		}

		// Add and merge vertices.
		for j := 0; j < cont.NVerts; j++ {
			v := cont.Verts[j*4:]
			indices[j] = addVertex(uint16(v[0]), uint16(v[1]), uint16(v[2]),
				mesh.Verts, firstVert, nextVert, &mesh.NVerts)
			if v[3]&0x80 != 0 {
				vflags[indices[j]] = 1
			}
		}

		// Build initial polygons.
		npolys := 0
		for j := 0; j < maxVertsPerCont*maxVertsPerPoly; j++ {
			polys[j] = TileCacheNullIdx
		}
		for j := 0; j < ntris; j++ {
			t := tris[j*3:]
			if t[0] != t[1] && t[0] != t[2] && t[1] != t[2] {
				polys[npolys*maxVertsPerPoly+0] = indices[t[0]]
				polys[npolys*maxVertsPerPoly+1] = indices[t[1]]
				polys[npolys*maxVertsPerPoly+2] = indices[t[2]]
				npolys++
			}
		}
		if npolys == 0 {
			continue
		}

		// Merge polygons.
		if maxVertsPerPoly > 3 {
			for {
				bestMergeVal := 0
				bestPa, bestPb := 0, 0
				bestEa, bestEb := 0, 0

				for j := 0; j < npolys-1; j++ {
					pj := polys[j*maxVertsPerPoly:]
					for k := j + 1; k < npolys; k++ {
						pk := polys[k*maxVertsPerPoly:]
						ea, eb := 0, 0
						v := getPolyMergeValue(pj, pk, mesh.Verts, &ea, &eb)
						if v > bestMergeVal {
							bestMergeVal = v
							bestPa = j
							bestPb = k
							bestEa = ea
							bestEb = eb
						}
					}
				}

				if bestMergeVal > 0 {
					pa := polys[bestPa*maxVertsPerPoly:]
					pb := polys[bestPb*maxVertsPerPoly:]
					mergePolys(pa, pb, bestEa, bestEb)
					copy(pb, polys[(npolys-1)*maxVertsPerPoly:(npolys-1)*maxVertsPerPoly+maxVertsPerPoly])
					npolys--
				} else {
					break
				}
			}
		}

		// Store polygons.
		for j := 0; j < npolys; j++ {
			p := mesh.Polys[mesh.NPolys*maxVertsPerPoly*2:]
			q := polys[j*maxVertsPerPoly:]
			for k := 0; k < maxVertsPerPoly; k++ {
				p[k] = q[k]
			}
			mesh.Areas[mesh.NPolys] = cont.Area
			mesh.NPolys++
			if mesh.NPolys > maxTris {
				return detour.ErrBufferTooSmall
			}
		}
	}

	// Remove edge vertices.
	for i := 0; i < mesh.NVerts; i++ {
		if vflags[i] != 0 {
			if !canRemoveVertex(mesh, uint16(i)) {
				continue
			}
			err := removeVertex(mesh, uint16(i), maxTris)
			if err != nil {
				return err
			}
			// Remove vertex - mesh.NVerts is already decremented inside removeVertex()
			for j := i; j < mesh.NVerts; j++ {
				vflags[j] = vflags[j+1]
			}
			i--
		}
	}

	// Calculate adjacency.
	if !buildMeshAdjacency(mesh.Polys, mesh.NPolys, mesh.Verts, mesh.NVerts, lcset) {
		return detour.ErrOutOfMemory
	}

	return nil
}

// ---------------------------------------------------------------------------
// Mark functions
// ---------------------------------------------------------------------------

// MarkCylinderArea marks a cylindrical area in the tile cache layer.
func MarkCylinderArea(layer *TileCacheLayer, orig *[3]float32, cs, ch float32,
	pos *[3]float32, radius, height float32, areaId uint8) error {

	var bmin, bmax [3]float32
	bmin[0] = pos[0] - radius
	bmin[1] = pos[1]
	bmin[2] = pos[2] - radius
	bmax[0] = pos[0] + radius
	bmax[1] = pos[1] + height
	bmax[2] = pos[2] + radius

	r2 := (radius/cs + 0.5) * (radius/cs + 0.5)

	w := int(layer.Header.Width)
	h := int(layer.Header.Height)
	ics := 1.0 / cs
	ich := 1.0 / ch

	px := (pos[0] - orig[0]) * ics
	pz := (pos[2] - orig[2]) * ics

	minx := int(math.Floor(float64((bmin[0] - orig[0]) * ics)))
	miny := int(math.Floor(float64((bmin[1] - orig[1]) * ich)))
	minz := int(math.Floor(float64((bmin[2] - orig[2]) * ics)))
	maxx := int(math.Floor(float64((bmax[0] - orig[0]) * ics)))
	maxy := int(math.Floor(float64((bmax[1] - orig[1]) * ich)))
	maxz := int(math.Floor(float64((bmax[2] - orig[2]) * ics)))

	if maxx < 0 || minx >= w || maxz < 0 || minz >= h {
		return nil
	}

	if minx < 0 {
		minx = 0
	}
	if maxx >= w {
		maxx = w - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= h {
		maxz = h - 1
	}

	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			dx := float32(x) + 0.5 - px
			dz := float32(z) + 0.5 - pz
			if dx*dx+dz*dz > r2 {
				continue
			}
			y := int(layer.Heights[x+z*w])
			if y < miny || y > maxy {
				continue
			}
			layer.Areas[x+z*w] = areaId
		}
	}

	return nil
}

// MarkBoxArea marks an axis-aligned box area in the tile cache layer.
func MarkBoxArea(layer *TileCacheLayer, orig *[3]float32, cs, ch float32,
	bmin, bmax *[3]float32, areaId uint8) error {

	w := int(layer.Header.Width)
	h := int(layer.Header.Height)
	ics := 1.0 / cs
	ich := 1.0 / ch

	minx := int(math.Floor(float64((bmin[0] - orig[0]) * ics)))
	miny := int(math.Floor(float64((bmin[1] - orig[1]) * ich)))
	minz := int(math.Floor(float64((bmin[2] - orig[2]) * ics)))
	maxx := int(math.Floor(float64((bmax[0] - orig[0]) * ics)))
	maxy := int(math.Floor(float64((bmax[1] - orig[1]) * ich)))
	maxz := int(math.Floor(float64((bmax[2] - orig[2]) * ics)))

	if maxx < 0 || minx >= w || maxz < 0 || minz >= h {
		return nil
	}

	if minx < 0 {
		minx = 0
	}
	if maxx >= w {
		maxx = w - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= h {
		maxz = h - 1
	}

	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			y := int(layer.Heights[x+z*w])
			if y < miny || y > maxy {
				continue
			}
			layer.Areas[x+z*w] = areaId
		}
	}

	return nil
}

// MarkBoxAreaOriented marks an oriented box area in the tile cache layer.
func MarkBoxAreaOriented(layer *TileCacheLayer, orig *[3]float32, cs, ch float32,
	center, halfExtents *[3]float32, rotAux *[2]float32, areaId uint8) error {

	w := int(layer.Header.Width)
	h := int(layer.Header.Height)
	ics := 1.0 / cs
	ich := 1.0 / ch

	cx := (center[0] - orig[0]) * ics
	cz := (center[2] - orig[2]) * ics

	maxr := float32(1.41) * float32(math.Max(float64(halfExtents[0]), float64(halfExtents[2])))
	minx := int(math.Floor(float64(cx - maxr*ics)))
	maxx := int(math.Floor(float64(cx + maxr*ics)))
	minz := int(math.Floor(float64(cz - maxr*ics)))
	maxz := int(math.Floor(float64(cz + maxr*ics)))
	miny := int(math.Floor(float64((center[1] - halfExtents[1] - orig[1]) * ich)))
	maxy := int(math.Floor(float64((center[1] + halfExtents[1] - orig[1]) * ich)))

	if maxx < 0 || minx >= w || maxz < 0 || minz >= h {
		return nil
	}

	if minx < 0 {
		minx = 0
	}
	if maxx >= w {
		maxx = w - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= h {
		maxz = h - 1
	}

	xhalf := halfExtents[0]*ics + 0.5
	zhalf := halfExtents[2]*ics + 0.5

	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			x2 := 2.0 * (float32(x) - cx)
			z2 := 2.0 * (float32(z) - cz)
			xrot := rotAux[1]*x2 + rotAux[0]*z2
			if xrot > xhalf || xrot < -xhalf {
				continue
			}
			zrot := rotAux[1]*z2 - rotAux[0]*x2
			if zrot > zhalf || zrot < -zhalf {
				continue
			}
			y := int(layer.Heights[x+z*w])
			if y < miny || y > maxy {
				continue
			}
			layer.Areas[x+z*w] = areaId
		}
	}

	return nil
}

// ---------------------------------------------------------------------------
// Header swap endian
// ---------------------------------------------------------------------------

// TileCacheHeaderSwapEndian swaps the endianness of the compressed tile data's header.
func TileCacheHeaderSwapEndian(data []uint8, dataSize int) bool {
	_ = dataSize
	header := (*TileCacheLayerHeader)(unsafe.Pointer(&data[0]))

	swappedMagic := int32(TileCacheMagic)
	swappedVersion := int32(TileCacheVersion)
	swapEndian32(&swappedMagic)
	swapEndian32(&swappedVersion)

	if (header.Magic != TileCacheMagic || header.Version != TileCacheVersion) &&
		(header.Magic != swappedMagic || header.Version != swappedVersion) {
		return false
	}

	swapEndian32(&header.Magic)
	swapEndian32(&header.Version)
	swapEndian32(&header.Tx)
	swapEndian32(&header.Ty)
	swapEndian32(&header.Tlayer)
	swapEndian32((*int32)(unsafe.Pointer(&header.Bmin[0])))
	swapEndian32((*int32)(unsafe.Pointer(&header.Bmin[1])))
	swapEndian32((*int32)(unsafe.Pointer(&header.Bmin[2])))
	swapEndian32((*int32)(unsafe.Pointer(&header.Bmax[0])))
	swapEndian32((*int32)(unsafe.Pointer(&header.Bmax[1])))
	swapEndian32((*int32)(unsafe.Pointer(&header.Bmax[2])))
	swapEndian16(&header.Hmin)
	swapEndian16(&header.Hmax)
	// width, height, minx, maxx, miny, maxy are unsigned char, no need to swap.

	return true
}

func swapEndian32(v *int32) {
	*v = int32(uint32(*v)<<24 | uint32(*v)>>24&0xff | (uint32(*v)&0xff00)<<8 | (uint32(*v)&0xff0000)>>8)
}

func swapEndian16(v *uint16) {
	*v = *v<<8 | *v>>8
}

// ---------------------------------------------------------------------------
// CreateNavMeshData stub
// ---------------------------------------------------------------------------

// CreateNavMeshData creates navigation mesh data from parameters.
// This is a function variable that can be replaced with a real implementation
// that calls into the detour package's CreateNavMeshData.
var CreateNavMeshData func(params *NavMeshCreateParams) ([]uint8, int)

func init() {
	// Default stub - returns nil which means "empty tile"
	CreateNavMeshData = func(params *NavMeshCreateParams) ([]uint8, int) {
		return nil, 0
	}
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

func ptrToSlice[T any](p *T) []uint8 {
	if p == nil {
		return nil
	}
	return unsafe.Slice((*uint8)(unsafe.Pointer(p)), int(unsafe.Sizeof(*p)))
}

func sliceToBytes[T any](s []T) []uint8 {
	if len(s) == 0 {
		return nil
	}
	return unsafe.Slice((*uint8)(unsafe.Pointer(&s[0])), len(s)*int(unsafe.Sizeof(s[0])))
}

func isConnected(layer *TileCacheLayer, ia, ib, walkableClimb int) bool {
	if layer.Areas[ia] != layer.Areas[ib] {
		return false
	}
	if absInt(int(layer.Heights[ia])-int(layer.Heights[ib])) > walkableClimb {
		return false
	}
	return true
}

func absInt(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

func addUniqueLast(a *[dtLayerMaxNeis]uint8, an *uint8, v uint8) {
	n := int(*an)
	if n > 0 && a[n-1] == v {
		return
	}
	a[*an] = v
	*an++
}

func canMerge(oldRegId, newRegId uint8, regs []dtLayerMonotoneRegion) bool {
	count := 0
	for i := range regs {
		reg := &regs[i]
		if reg.regId != oldRegId {
			continue
		}
		nnei := int(reg.nneis)
		for j := 0; j < nnei; j++ {
			if regs[reg.neis[j]].regId == newRegId {
				count++
			}
		}
	}
	return count == 1
}

func getDirOffsetX(dir int) int {
	offset := [4]int{-1, 0, 1, 0}
	return offset[dir&0x03]
}

func getDirOffsetY(dir int) int {
	offset := [4]int{0, 1, 0, -1}
	return offset[dir&0x03]
}

func getNeighbourReg(layer *TileCacheLayer, ax, ay, dir int) uint8 {
	w := int(layer.Header.Width)
	ia := ax + ay*w

	con := layer.Cons[ia] & 0xf
	portal := layer.Cons[ia] >> 4
	mask := uint8(1 << dir)

	if con&mask == 0 {
		if portal&mask != 0 {
			return 0xf8 + uint8(dir)
		}
		return 0xff
	}

	bx := ax + getDirOffsetX(dir)
	by := ay + getDirOffsetY(dir)
	ib := bx + by*w

	return layer.Regs[ib]
}

func walkContour(layer *TileCacheLayer, x, y int, cont *dtTempContour) bool {
	w := int(layer.Header.Width)
	h := int(layer.Header.Height)

	cont.nverts = 0

	startX := x
	startY := y
	startDir := -1

	for i := 0; i < 4; i++ {
		dir := (i + 3) & 3
		rn := getNeighbourReg(layer, x, y, dir)
		if rn != layer.Regs[x+y*w] {
			startDir = dir
			break
		}
	}
	if startDir == -1 {
		return true
	}

	dir := startDir
	maxIter := w * h

	iter := 0
	for iter < maxIter {
		rn := getNeighbourReg(layer, x, y, dir)

		nx := x
		ny := y
		ndir := dir

		if rn != layer.Regs[x+y*w] {
			px := x
			pz := y
			switch dir {
			case 0:
				pz++
			case 1:
				px++
				pz++
			case 2:
				px++
			}

			if !appendVertex(cont, px, int(layer.Heights[x+y*w]), pz, int(rn)) {
				return false
			}

			ndir = (dir + 1) & 0x3 // Rotate CW
		} else {
			nx = x + getDirOffsetX(dir)
			ny = y + getDirOffsetY(dir)
			ndir = (dir + 3) & 0x3 // Rotate CCW
		}

		if iter > 0 && x == startX && y == startY && dir == startDir {
			break
		}

		x = nx
		y = ny
		dir = ndir

		iter++
	}

	// Remove last vertex if it is duplicate of the first one.
	pa := cont.verts[(cont.nverts-1)*4:]
	pb := cont.verts[0:4]
	if pa[0] == pb[0] && pa[2] == pb[2] {
		cont.nverts--
	}

	return true
}

func appendVertex(cont *dtTempContour, x, y, z, r int) bool {
	// Try to merge with existing segments.
	if cont.nverts > 1 {
		pa := cont.verts[(cont.nverts-2)*4:]
		pb := cont.verts[(cont.nverts-1)*4:]
		if int(pb[3]) == r {
			if pa[0] == pb[0] && int(pb[0]) == x {
				pb[1] = uint8(y)
				pb[2] = uint8(z)
				return true
			} else if pa[2] == pb[2] && int(pb[2]) == z {
				pb[0] = uint8(x)
				pb[1] = uint8(y)
				return true
			}
		}
	}

	if cont.nverts+1 > cont.cverts {
		return false
	}

	v := cont.verts[cont.nverts*4:]
	v[0] = uint8(x)
	v[1] = uint8(y)
	v[2] = uint8(z)
	v[3] = uint8(r)
	cont.nverts++

	return true
}

func distancePtSeg(x, z, px, pz, qx, qz int) float32 {
	pqx := float32(qx - px)
	pqz := float32(qz - pz)
	dx := float32(x - px)
	dz := float32(z - pz)
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	dx = float32(px) + t*pqx - float32(x)
	dz = float32(pz) + t*pqz - float32(z)

	return dx*dx + dz*dz
}

func simplifyContour(cont *dtTempContour, maxError float32) {
	cont.npoly = 0

	for i := 0; i < cont.nverts; i++ {
		j := (i + 1) % cont.nverts
		ra := cont.verts[j*4+3]
		rb := cont.verts[i*4+3]
		if ra != rb {
			cont.poly[cont.npoly] = uint16(i)
			cont.npoly++
		}
	}
	if cont.npoly < 2 {
		llx := int(cont.verts[0])
		llz := int(cont.verts[2])
		lli := 0
		urx := int(cont.verts[0])
		urz := int(cont.verts[2])
		uri := 0
		for i := 1; i < cont.nverts; i++ {
			x := int(cont.verts[i*4+0])
			z := int(cont.verts[i*4+2])
			if x < llx || (x == llx && z < llz) {
				llx = x
				llz = z
				lli = i
			}
			if x > urx || (x == urx && z > urz) {
				urx = x
				urz = z
				uri = i
			}
		}
		cont.npoly = 0
		cont.poly[cont.npoly] = uint16(lli)
		cont.npoly++
		cont.poly[cont.npoly] = uint16(uri)
		cont.npoly++
	}

	// Add points until all raw points are within error tolerance.
	for i := 0; i < cont.npoly; {
		ii := (i + 1) % cont.npoly

		ai := int(cont.poly[i])
		ax := int(cont.verts[ai*4+0])
		az := int(cont.verts[ai*4+2])

		bi := int(cont.poly[ii])
		bx := int(cont.verts[bi*4+0])
		bz := int(cont.verts[bi*4+2])

		maxd := float32(0)
		maxi := -1
		var ci, cinc, endi int

		if bx > ax || (bx == ax && bz > az) {
			cinc = 1
			ci = (ai + cinc) % cont.nverts
			endi = bi
		} else {
			cinc = cont.nverts - 1
			ci = (bi + cinc) % cont.nverts
			endi = ai
		}

		for ci != endi {
			d := distancePtSeg(int(cont.verts[ci*4+0]), int(cont.verts[ci*4+2]), ax, az, bx, bz)
			if d > maxd {
				maxd = d
				maxi = ci
			}
			ci = (ci + cinc) % cont.nverts
		}

		if maxi != -1 && maxd > (maxError*maxError) {
			cont.npoly++
			for j := cont.npoly - 1; j > i; j-- {
				cont.poly[j] = cont.poly[j-1]
			}
			cont.poly[i+1] = uint16(maxi)
		} else {
			i++
		}
	}

	// Remap vertices
	start := 0
	for i := 1; i < cont.npoly; i++ {
		if cont.poly[i] < cont.poly[start] {
			start = i
		}
	}

	cont.nverts = 0
	for i := 0; i < cont.npoly; i++ {
		j := (start + i) % cont.npoly
		src := cont.verts[cont.poly[j]*4:]
		dst := cont.verts[cont.nverts*4:]
		dst[0] = src[0]
		dst[1] = src[1]
		dst[2] = src[2]
		dst[3] = src[3]
		cont.nverts++
	}
}

func getCornerHeight(layer *TileCacheLayer, x, y, z, walkableClimb int, shouldRemove *bool) uint8 {
	w := int(layer.Header.Width)
	h := int(layer.Header.Height)

	n := 0

	portal := uint8(0xf)
	height := uint8(0)
	preg := uint8(0xff)
	allSameReg := true

	for dz := -1; dz <= 0; dz++ {
		for dx := -1; dx <= 0; dx++ {
			px := x + dx
			pz := z + dz
			if px >= 0 && pz >= 0 && px < w && pz < h {
				idx := px + pz*w
				lh := int(layer.Heights[idx])
				if absInt(lh-y) <= walkableClimb && layer.Areas[idx] != TileCacheNullArea {
					if uint8(lh) > height {
						height = uint8(lh)
					}
					portal &= (layer.Cons[idx] >> 4)
					if preg != 0xff && preg != layer.Regs[idx] {
						allSameReg = false
					}
					preg = layer.Regs[idx]
					n++
				}
			}
		}
	}

	portalCount := 0
	for dir := 0; dir < 4; dir++ {
		if portal&(1<<dir) != 0 {
			portalCount++
		}
	}

	*shouldRemove = false
	if n > 1 && portalCount == 1 && allSameReg {
		*shouldRemove = true
	}

	return height
}

func buildMeshAdjacency(polys []uint16, npolys int,
	verts []uint16, nverts int, lcset *TileCacheContourSet) bool {

	maxEdgeCount := npolys * maxVertsPerPoly
	firstEdge := make([]uint16, nverts+maxEdgeCount)
	nextEdge := firstEdge[nverts:]
	edgeCount := 0

	edges := make([]rcEdge, maxEdgeCount)

	for i := 0; i < nverts; i++ {
		firstEdge[i] = TileCacheNullIdx
	}

	for i := 0; i < npolys; i++ {
		t := polys[i*maxVertsPerPoly*2:]
		for j := 0; j < maxVertsPerPoly; j++ {
			if t[j] == TileCacheNullIdx {
				break
			}
			v0 := t[j]
			var v1 uint16
			if j+1 >= maxVertsPerPoly || t[j+1] == TileCacheNullIdx {
				v1 = t[0]
			} else {
				v1 = t[j+1]
			}
			if v0 < v1 {
				edge := &edges[edgeCount]
				edge.vert[0] = v0
				edge.vert[1] = v1
				edge.poly[0] = uint16(i)
				edge.polyEdge[0] = uint16(j)
				edge.poly[1] = uint16(i)
				edge.polyEdge[1] = 0xff
				nextEdge[edgeCount] = firstEdge[v0]
				firstEdge[v0] = uint16(edgeCount)
				edgeCount++
			}
		}
	}

	for i := 0; i < npolys; i++ {
		t := polys[i*maxVertsPerPoly*2:]
		for j := 0; j < maxVertsPerPoly; j++ {
			if t[j] == TileCacheNullIdx {
				break
			}
			v0 := t[j]
			var v1 uint16
			if j+1 >= maxVertsPerPoly || t[j+1] == TileCacheNullIdx {
				v1 = t[0]
			} else {
				v1 = t[j+1]
			}
			if v0 > v1 {
				found := false
				for e := firstEdge[v1]; e != TileCacheNullIdx; e = nextEdge[e] {
					edge := &edges[e]
					if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
						edge.poly[1] = uint16(i)
						edge.polyEdge[1] = uint16(j)
						found = true
						break
					}
				}
				if !found {
					edge := &edges[edgeCount]
					edge.vert[0] = v1
					edge.vert[1] = v0
					edge.poly[0] = uint16(i)
					edge.polyEdge[0] = uint16(j)
					edge.poly[1] = uint16(i)
					edge.polyEdge[1] = 0xff
					nextEdge[edgeCount] = firstEdge[v1]
					firstEdge[v1] = uint16(edgeCount)
					edgeCount++
				}
			}
		}
	}

	// Mark portal edges.
	for i := 0; i < lcset.NConts; i++ {
		cont := &lcset.Conts[i]
		if cont.NVerts < 3 {
			continue
		}

		for j, k := 0, cont.NVerts-1; j < cont.NVerts; k, j = j, j+1 {
			va := cont.Verts[k*4:]
			vb := cont.Verts[j*4:]
			dir := va[3] & 0xf
			if dir == 0xf {
				continue
			}

			if dir == 0 || dir == 2 {
				x := uint16(va[0])
				zmin := uint16(va[2])
				zmax := uint16(vb[2])
				if zmin > zmax {
					zmin, zmax = zmax, zmin
				}

				for m := 0; m < edgeCount; m++ {
					e := &edges[m]
					if e.poly[0] != e.poly[1] {
						continue
					}
					eva := verts[e.vert[0]*3:]
					evb := verts[e.vert[1]*3:]
					if eva[0] == x && evb[0] == x {
						ezmin := eva[2]
						ezmax := evb[2]
						if ezmin > ezmax {
							ezmin, ezmax = ezmax, ezmin
						}
						if overlapRangeExl(zmin, zmax, ezmin, ezmax) {
							e.polyEdge[1] = uint16(dir)
						}
					}
				}
			} else {
				z := uint16(va[2])
				xmin := uint16(va[0])
				xmax := uint16(vb[0])
				if xmin > xmax {
					xmin, xmax = xmax, xmin
				}
				for m := 0; m < edgeCount; m++ {
					e := &edges[m]
					if e.poly[0] != e.poly[1] {
						continue
					}
					eva := verts[e.vert[0]*3:]
					evb := verts[e.vert[1]*3:]
					if eva[2] == z && evb[2] == z {
						exmin := eva[0]
						exmax := evb[0]
						if exmin > exmax {
							exmin, exmax = exmax, exmin
						}
						if overlapRangeExl(xmin, xmax, exmin, exmax) {
							e.polyEdge[1] = uint16(dir)
						}
					}
				}
			}
		}
	}

	// Store adjacency
	for i := 0; i < edgeCount; i++ {
		e := &edges[i]
		if e.poly[0] != e.poly[1] {
			p0 := polys[e.poly[0]*maxVertsPerPoly*2:]
			p1 := polys[e.poly[1]*maxVertsPerPoly*2:]
			p0[maxVertsPerPoly+e.polyEdge[0]] = e.poly[1]
			p1[maxVertsPerPoly+e.polyEdge[1]] = e.poly[0]
		} else if e.polyEdge[1] != 0xff {
			p0 := polys[e.poly[0]*maxVertsPerPoly*2:]
			p0[maxVertsPerPoly+e.polyEdge[0]] = 0x8000 | uint16(e.polyEdge[1])
		}
	}

	return true
}

func overlapRangeExl(amin, amax, bmin, bmax uint16) bool {
	return !(amin >= bmax || amax <= bmin)
}

func prev(i, n int) int {
	if i-1 >= 0 {
		return i - 1
	}
	return n - 1
}

func next(i, n int) int {
	if i+1 < n {
		return i + 1
	}
	return 0
}

func area2(a, b, c []uint8) int {
	return (int(b[0])-int(a[0]))*(int(c[2])-int(a[2])) - (int(c[0])-int(a[0]))*(int(b[2])-int(a[2]))
}

func left(a, b, c []uint8) bool {
	return area2(a, b, c) < 0
}

func leftOn(a, b, c []uint8) bool {
	return area2(a, b, c) <= 0
}

func collinear(a, b, c []uint8) bool {
	return area2(a, b, c) == 0
}

func intersectProp(a, b, c, d []uint8) bool {
	if collinear(a, b, c) || collinear(a, b, d) ||
		collinear(c, d, a) || collinear(c, d, b) {
		return false
	}
	return (left(a, b, c) != left(a, b, d)) && (left(c, d, a) != left(c, d, b))
}

func between(a, b, c []uint8) bool {
	if !collinear(a, b, c) {
		return false
	}
	if a[0] != b[0] {
		return (a[0] <= c[0] && c[0] <= b[0]) || (a[0] >= c[0] && c[0] >= b[0])
	}
	return (a[2] <= c[2] && c[2] <= b[2]) || (a[2] >= c[2] && c[2] >= b[2])
}

func intersect(a, b, c, d []uint8) bool {
	if intersectProp(a, b, c, d) {
		return true
	}
	return between(a, b, c) || between(a, b, d) ||
		between(c, d, a) || between(c, d, b)
}

func vequal(a, b []uint8) bool {
	return a[0] == b[0] && a[2] == b[2]
}

func diagonalie(i, j, n int, verts []uint8, indices []uint16) bool {
	d0 := verts[(indices[i]&0x7fff)*4:]
	d1 := verts[(indices[j]&0x7fff)*4:]

	for k := 0; k < n; k++ {
		k1 := next(k, n)
		if !(k == i || k1 == i || k == j || k1 == j) {
			p0 := verts[(indices[k]&0x7fff)*4:]
			p1 := verts[(indices[k1]&0x7fff)*4:]

			if vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1) {
				continue
			}

			if intersect(d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}

func inCone(i, j, n int, verts []uint8, indices []uint16) bool {
	pi := verts[(indices[i]&0x7fff)*4:]
	pj := verts[(indices[j]&0x7fff)*4:]
	pi1 := verts[(indices[next(i, n)]&0x7fff)*4:]
	pin1 := verts[(indices[prev(i, n)]&0x7fff)*4:]

	if leftOn(pin1, pi, pi1) {
		return left(pi, pj, pin1) && left(pj, pi, pi1)
	}
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1))
}

func diagonal(i, j, n int, verts []uint8, indices []uint16) bool {
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices)
}

func triangulate(n int, verts []uint8, indices []uint16, tris []uint16) int {
	ntris := 0
	dst := tris

	// The last bit of the index is used to indicate if the vertex can be removed.
	for i := 0; i < n; i++ {
		i1 := next(i, n)
		i2 := next(i1, n)
		if diagonal(i, i2, n, verts, indices) {
			indices[i1] |= 0x8000
		}
	}

	for n > 3 {
		minLen := -1
		mini := -1
		for i := 0; i < n; i++ {
			i1 := next(i, n)
			if indices[i1]&0x8000 != 0 {
				p0 := verts[(indices[i]&0x7fff)*4:]
				p2 := verts[(indices[next(i1, n)]&0x7fff)*4:]

				dx := int(p2[0]) - int(p0[0])
				dz := int(p2[2]) - int(p0[2])
				length := dx*dx + dz*dz
				if minLen < 0 || length < minLen {
					minLen = length
					mini = i
				}
			}
		}

		if mini == -1 {
			return -ntris
		}

		i := mini
		i1 := next(i, n)
		i2 := next(i1, n)

		dst[0] = indices[i] & 0x7fff
		dst[1] = indices[i1] & 0x7fff
		dst[2] = indices[i2] & 0x7fff
		dst = dst[3:]
		ntris++

		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--
		for k := i1; k < n; k++ {
			indices[k] = indices[k+1]
		}

		if i1 >= n {
			i1 = 0
		}
		i = prev(i1, n)
		// Update diagonal flags.
		if diagonal(prev(i, n), i1, n, verts, indices) {
			indices[i] |= 0x8000
		} else {
			indices[i] &= 0x7fff
		}

		if diagonal(i, next(i1, n), n, verts, indices) {
			indices[i1] |= 0x8000
		} else {
			indices[i1] &= 0x7fff
		}
	}

	// Append the remaining triangle.
	dst[0] = indices[0] & 0x7fff
	dst[1] = indices[1] & 0x7fff
	dst[2] = indices[2] & 0x7fff
	ntris++

	return ntris
}

func countPolyVerts(p []uint16) int {
	for i := 0; i < maxVertsPerPoly; i++ {
		if p[i] == TileCacheNullIdx {
			return i
		}
	}
	return maxVertsPerPoly
}

func uleft(a, b, c []uint16) bool {
	return (int(b[0])-int(a[0]))*(int(c[2])-int(a[2]))-
		(int(c[0])-int(a[0]))*(int(b[2])-int(a[2])) < 0
}

func getPolyMergeValue(pa, pb []uint16, verts []uint16, ea, eb *int) int {
	na := countPolyVerts(pa)
	nb := countPolyVerts(pb)

	if na+nb-2 > maxVertsPerPoly {
		return -1
	}

	*ea = -1
	*eb = -1

	for i := 0; i < na; i++ {
		va0 := pa[i]
		va1 := pa[(i+1)%na]
		if va0 > va1 {
			va0, va1 = va1, va0
		}
		for j := 0; j < nb; j++ {
			vb0 := pb[j]
			vb1 := pb[(j+1)%nb]
			if vb0 > vb1 {
				vb0, vb1 = vb1, vb0
			}
			if va0 == vb0 && va1 == vb1 {
				*ea = i
				*eb = j
				break
			}
		}
	}

	if *ea == -1 || *eb == -1 {
		return -1
	}

	va := pa[(*ea+na-1)%na]
	vb := pa[*ea]
	vc := pb[(*eb+2)%nb]
	if !uleft(verts[va*3:], verts[vb*3:], verts[vc*3:]) {
		return -1
	}

	va = pb[(*eb+nb-1)%nb]
	vb = pb[*eb]
	vc = pa[(*ea+2)%na]
	if !uleft(verts[va*3:], verts[vb*3:], verts[vc*3:]) {
		return -1
	}

	va = pa[*ea]
	vb = pa[(*ea+1)%na]

	dx := int(verts[va*3+0]) - int(verts[vb*3+0])
	dy := int(verts[va*3+2]) - int(verts[vb*3+2])

	return dx*dx + dy*dy
}

func mergePolys(pa, pb []uint16, ea, eb int) {
	tmp := make([]uint16, maxVertsPerPoly*2)

	na := countPolyVerts(pa)
	nb := countPolyVerts(pb)

	for i := range tmp {
		tmp[i] = TileCacheNullIdx
	}
	n := 0
	for i := 0; i < na-1; i++ {
		tmp[n] = pa[(ea+1+i)%na]
		n++
	}
	for i := 0; i < nb-1; i++ {
		tmp[n] = pb[(eb+1+i)%nb]
		n++
	}

	copy(pa, tmp[:maxVertsPerPoly])
}

func pushFront(v uint16, arr []uint16, an *int) {
	*an++
	for i := *an - 1; i > 0; i-- {
		arr[i] = arr[i-1]
	}
	arr[0] = v
}

func pushBack(v uint16, arr []uint16, an *int) {
	arr[*an] = v
	*an++
}

func canRemoveVertex(mesh *TileCachePolyMesh, rem uint16) bool {
	numTouchedVerts := 0
	numRemainingEdges := 0
	for i := 0; i < mesh.NPolys; i++ {
		p := mesh.Polys[i*maxVertsPerPoly*2:]
		nv := countPolyVerts(p)
		numRemoved := 0
		numVerts := 0
		for j := 0; j < nv; j++ {
			if p[j] == rem {
				numTouchedVerts++
				numRemoved++
			}
			numVerts++
		}
		if numRemoved > 0 {
			numRemainingEdges += numVerts - (numRemoved + 1)
		}
	}

	if numRemainingEdges <= 2 {
		return false
	}

	maxEdges := numTouchedVerts * 2
	if maxEdges > maxRemEdges {
		return false
	}

	var edges [maxRemEdges * 3]uint16
	nedges := 0

	for i := 0; i < mesh.NPolys; i++ {
		p := mesh.Polys[i*maxVertsPerPoly*2:]
		nv := countPolyVerts(p)

		for j, k := 0, nv-1; j < nv; k, j = j, j+1 {
			if p[j] == rem || p[k] == rem {
				a := p[j]
				b := p[k]
				if b == rem {
					a, b = b, a
				}

				exists := false
				for m := 0; m < nedges; m++ {
					e := edges[m*3:]
					if e[1] == b {
						e[2]++
						exists = true
					}
				}
				if !exists {
					e := edges[nedges*3:]
					e[0] = a
					e[1] = b
					e[2] = 1
					nedges++
				}
			}
		}
	}

	numOpenEdges := 0
	for i := 0; i < nedges; i++ {
		if edges[i*3+2] < 2 {
			numOpenEdges++
		}
	}
	return numOpenEdges <= 2
}

func removeVertex(mesh *TileCachePolyMesh, rem uint16, maxTris int) error {
	nedges := 0
	var edges [maxRemEdges * 3]uint16
	nhole := 0
	var hole [maxRemEdges]uint16
	nharea := 0
	var harea [maxRemEdges]uint16

	for i := 0; i < mesh.NPolys; i++ {
		p := mesh.Polys[i*maxVertsPerPoly*2:]
		nv := countPolyVerts(p)
		hasRem := false
		for j := 0; j < nv; j++ {
			if p[j] == rem {
				hasRem = true
				break
			}
		}
		if hasRem {
			for j, k := 0, nv-1; j < nv; k, j = j, j+1 {
				if p[j] != rem && p[k] != rem {
					if nedges >= maxRemEdges {
						return detour.ErrBufferTooSmall
					}
					e := edges[nedges*3:]
					e[0] = p[k]
					e[1] = p[j]
					e[2] = uint16(mesh.Areas[i])
					nedges++
				}
			}
			// Remove the polygon.
			p2 := mesh.Polys[(mesh.NPolys-1)*maxVertsPerPoly*2:]
			copy(p, p2)
			for j := maxVertsPerPoly; j < maxVertsPerPoly*2; j++ {
				p[j] = TileCacheNullIdx
			}
			mesh.Areas[i] = mesh.Areas[mesh.NPolys-1]
			mesh.NPolys--
			i--
		}
	}

	// Remove vertex.
	for i := int(rem); i < mesh.NVerts-1; i++ {
		mesh.Verts[i*3+0] = mesh.Verts[(i+1)*3+0]
		mesh.Verts[i*3+1] = mesh.Verts[(i+1)*3+1]
		mesh.Verts[i*3+2] = mesh.Verts[(i+1)*3+2]
	}
	mesh.NVerts--

	// Adjust indices to match the removed vertex layout.
	for i := 0; i < mesh.NPolys; i++ {
		p := mesh.Polys[i*maxVertsPerPoly*2:]
		nv := countPolyVerts(p)
		for j := 0; j < nv; j++ {
			if p[j] > rem {
				p[j]--
			}
		}
	}
	for i := 0; i < nedges; i++ {
		if edges[i*3+0] > rem {
			edges[i*3+0]--
		}
		if edges[i*3+1] > rem {
			edges[i*3+1]--
		}
	}

	if nedges == 0 {
		return nil
	}

	pushBack(edges[0], hole[:], &nhole)
	pushBack(edges[2], harea[:], &nharea)

	for nedges > 0 {
		match := false

		for i := 0; i < nedges; i++ {
			ea := edges[i*3+0]
			eb := edges[i*3+1]
			a := edges[i*3+2]
			add := false
			if hole[0] == eb {
				if nhole >= maxRemEdges {
					return detour.ErrBufferTooSmall
				}
				pushFront(ea, hole[:], &nhole)
				pushFront(a, harea[:], &nharea)
				add = true
			} else if hole[nhole-1] == ea {
				if nhole >= maxRemEdges {
					return detour.ErrBufferTooSmall
				}
				pushBack(eb, hole[:], &nhole)
				pushBack(a, harea[:], &nharea)
				add = true
			}
			if add {
				edges[i*3+0] = edges[(nedges-1)*3+0]
				edges[i*3+1] = edges[(nedges-1)*3+1]
				edges[i*3+2] = edges[(nedges-1)*3+2]
				nedges--
				match = true
				i--
			}
		}

		if !match {
			break
		}
	}

	var tris [maxRemEdges * 3]uint16
	var tverts [maxRemEdges * 4]uint8
	var tpoly [maxRemEdges]uint16

	for i := 0; i < nhole; i++ {
		pi := hole[i]
		tverts[i*4+0] = uint8(mesh.Verts[pi*3+0] & 0xff)
		tverts[i*4+1] = uint8(mesh.Verts[pi*3+1] & 0xff)
		tverts[i*4+2] = uint8(mesh.Verts[pi*3+2] & 0xff)
		tverts[i*4+3] = 0
		tpoly[i] = uint16(i)
	}

	ntris := triangulate(nhole, tverts[:], tpoly[:], tris[:])
	if ntris < 0 {
		ntris = -ntris
	}

	if ntris > maxRemEdges {
		return detour.ErrBufferTooSmall
	}

	var polys [maxRemEdges * maxVertsPerPoly]uint16
	var pareas [maxRemEdges]uint8

	npolys := 0
	for j := 0; j < ntris*maxVertsPerPoly; j++ {
		polys[j] = TileCacheNullIdx
	}
	for j := 0; j < ntris; j++ {
		t := tris[j*3:]
		if t[0] != t[1] && t[0] != t[2] && t[1] != t[2] {
			polys[npolys*maxVertsPerPoly+0] = hole[t[0]]
			polys[npolys*maxVertsPerPoly+1] = hole[t[1]]
			polys[npolys*maxVertsPerPoly+2] = hole[t[2]]
			pareas[npolys] = uint8(harea[t[0]])
			npolys++
		}
	}
	if npolys == 0 {
		return nil
	}

	// Merge polygons.
	if maxVertsPerPoly > 3 {
		for {
			bestMergeVal := 0
			bestPa, bestPb := 0, 0
			bestEa, bestEb := 0, 0

			for j := 0; j < npolys-1; j++ {
				pj := polys[j*maxVertsPerPoly:]
				for k := j + 1; k < npolys; k++ {
					pk := polys[k*maxVertsPerPoly:]
					ea, eb := 0, 0
					v := getPolyMergeValue(pj, pk, mesh.Verts, &ea, &eb)
					if v > bestMergeVal {
						bestMergeVal = v
						bestPa = j
						bestPb = k
						bestEa = ea
						bestEb = eb
					}
				}
			}

			if bestMergeVal > 0 {
				pa := polys[bestPa*maxVertsPerPoly:]
				pb := polys[bestPb*maxVertsPerPoly:]
				mergePolys(pa, pb, bestEa, bestEb)
				copy(pb, polys[(npolys-1)*maxVertsPerPoly:(npolys-1)*maxVertsPerPoly+maxVertsPerPoly])
				pareas[bestPb] = pareas[npolys-1]
				npolys--
			} else {
				break
			}
		}
	}

	// Store polygons.
	for i := 0; i < npolys; i++ {
		if mesh.NPolys >= maxTris {
			break
		}
		p := mesh.Polys[mesh.NPolys*maxVertsPerPoly*2:]
		for j := 0; j < maxVertsPerPoly*2; j++ {
			p[j] = TileCacheNullIdx
		}
		for j := 0; j < maxVertsPerPoly; j++ {
			p[j] = polys[i*maxVertsPerPoly+j]
		}
		mesh.Areas[mesh.NPolys] = pareas[i]
		mesh.NPolys++
		if mesh.NPolys > maxTris {
			return detour.ErrBufferTooSmall
		}
	}

	return nil
}

// Constants from the C++ builder
const (
	maxVertsPerPoly = 6
	maxRemEdges     = 48
)
