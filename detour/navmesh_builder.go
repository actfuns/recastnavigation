package detour

import (
	"encoding/binary"
	"math"
	"sort"
	"unsafe"
)

// NavMeshCreateParams represents the source data used to build a navigation mesh tile.
type NavMeshCreateParams struct {
	// Polygon Mesh Attributes
	Verts     []uint16
	VertCount int
	Polys     []uint16
	PolyFlags []uint16
	PolyAreas []uint8
	PolyCount int
	Nvp       int

	// Height Detail Attributes
	DetailMeshes     []uint32
	DetailVerts      []float32
	DetailVertsCount int
	DetailTris       []uint8
	DetailTriCount   int

	// Off-Mesh Connections Attributes
	OffMeshConVerts  []float32
	OffMeshConRad    []float32
	OffMeshConFlags  []uint16
	OffMeshConAreas  []uint8
	OffMeshConDir    []uint8
	OffMeshConUserID []uint32
	OffMeshConCount  int

	// Tile Attributes
	UserID    uint32
	TileX     int
	TileY     int
	TileLayer int
	Bmin      [3]float32
	Bmax      [3]float32

	// General Configuration Attributes
	WalkableHeight float32
	WalkableRadius float32
	WalkableClimb  float32
	Cs             float32
	Ch             float32
	BuildBvTree    bool
}

type bvItem struct {
	bmin [3]uint16
	bmax [3]uint16
	i    int
}

type byBvItemX []bvItem

func (a byBvItemX) Len() int           { return len(a) }
func (a byBvItemX) Less(i, j int) bool { return a[i].bmin[0] < a[j].bmin[0] }
func (a byBvItemX) Swap(i, j int)      { a[i], a[j] = a[j], a[i] }

type byBvItemY []bvItem

func (a byBvItemY) Len() int           { return len(a) }
func (a byBvItemY) Less(i, j int) bool { return a[i].bmin[1] < a[j].bmin[1] }
func (a byBvItemY) Swap(i, j int)      { a[i], a[j] = a[j], a[i] }

type byBvItemZ []bvItem

func (a byBvItemZ) Len() int           { return len(a) }
func (a byBvItemZ) Less(i, j int) bool { return a[i].bmin[2] < a[j].bmin[2] }
func (a byBvItemZ) Swap(i, j int)      { a[i], a[j] = a[j], a[i] }

func calcExtends(items []bvItem, imin, imax int) (bmin, bmax [3]uint16) {
	bmin = items[imin].bmin
	bmax = items[imin].bmax
	for i := imin + 1; i < imax; i++ {
		it := items[i]
		if it.bmin[0] < bmin[0] {
			bmin[0] = it.bmin[0]
		}
		if it.bmin[1] < bmin[1] {
			bmin[1] = it.bmin[1]
		}
		if it.bmin[2] < bmin[2] {
			bmin[2] = it.bmin[2]
		}
		if it.bmax[0] > bmax[0] {
			bmax[0] = it.bmax[0]
		}
		if it.bmax[1] > bmax[1] {
			bmax[1] = it.bmax[1]
		}
		if it.bmax[2] > bmax[2] {
			bmax[2] = it.bmax[2]
		}
	}
	return bmin, bmax
}

func longestAxis16(x, y, z uint16) int {
	axis := 0
	maxVal := x
	if y > maxVal {
		axis = 1
		maxVal = y
	}
	if z > maxVal {
		axis = 2
	}
	return axis
}

func subdivide(items []bvItem, nitems, imin, imax int, curNode *int, nodes []BVNode) {
	inum := imax - imin
	icur := *curNode
	node := &nodes[*curNode]
	*curNode++

	if inum == 1 {
		node.Bmin = items[imin].bmin
		node.Bmax = items[imin].bmax
		node.I = int32(items[imin].i)
	} else {
		bmin, bmax := calcExtends(items, imin, imax)
		node.Bmin = bmin
		node.Bmax = bmax

		axis := longestAxis16(bmax[0]-bmin[0], bmax[1]-bmin[1], bmax[2]-bmin[2])

		subItems := items[imin:imax]
		switch axis {
		case 0:
			sort.Sort(byBvItemX(subItems))
		case 1:
			sort.Sort(byBvItemY(subItems))
		case 2:
			sort.Sort(byBvItemZ(subItems))
		}

		isplit := imin + inum/2
		subdivide(items, nitems, imin, isplit, curNode, nodes)
		subdivide(items, nitems, isplit, imax, curNode, nodes)

		iescape := *curNode - icur
		node.I = int32(-iescape)
	}
}

func createBVTree(params *NavMeshCreateParams, nodes []BVNode) {
	quantFactor := 1.0 / params.Cs
	items := make([]bvItem, params.PolyCount)
	for i := 0; i < params.PolyCount; i++ {
		it := &items[i]
		it.i = i
		if params.DetailMeshes != nil {
			vb := int(params.DetailMeshes[i*4+0])
			ndv := int(params.DetailMeshes[i*4+1])
			var bmin, bmax [3]float32
			dv := params.DetailVerts[vb*3:]
			bmin[0], bmin[1], bmin[2] = dv[0], dv[1], dv[2]
			bmax[0], bmax[1], bmax[2] = dv[0], dv[1], dv[2]
			for j := 1; j < ndv; j++ {
				bmin = Vmin(bmin, [3]float32{dv[j*3], dv[j*3+1], dv[j*3+2]})
				bmax = Vmax(bmax, [3]float32{dv[j*3], dv[j*3+1], dv[j*3+2]})
			}
			it.bmin[0] = clampInt16(int((bmin[0] - params.Bmin[0]) * quantFactor))
			it.bmin[1] = clampInt16(int((bmin[1] - params.Bmin[1]) * quantFactor))
			it.bmin[2] = clampInt16(int((bmin[2] - params.Bmin[2]) * quantFactor))
			it.bmax[0] = clampInt16(int((bmax[0] - params.Bmin[0]) * quantFactor))
			it.bmax[1] = clampInt16(int((bmax[1] - params.Bmin[1]) * quantFactor))
			it.bmax[2] = clampInt16(int((bmax[2] - params.Bmin[2]) * quantFactor))
		} else {
			p := params.Polys[i*params.Nvp*2:]
			it.bmin[0] = params.Verts[p[0]*3+0]
			it.bmin[1] = params.Verts[p[0]*3+1]
			it.bmin[2] = params.Verts[p[0]*3+2]
			it.bmax[0] = params.Verts[p[0]*3+0]
			it.bmax[1] = params.Verts[p[0]*3+1]
			it.bmax[2] = params.Verts[p[0]*3+2]
			for j := 1; j < params.Nvp; j++ {
				if p[j] == MeshNullIdx {
					break
				}
				x := params.Verts[p[j]*3+0]
				y := params.Verts[p[j]*3+1]
				z := params.Verts[p[j]*3+2]
				if x < it.bmin[0] {
					it.bmin[0] = x
				}
				if y < it.bmin[1] {
					it.bmin[1] = y
				}
				if z < it.bmin[2] {
					it.bmin[2] = z
				}
				if x > it.bmax[0] {
					it.bmax[0] = x
				}
				if y > it.bmax[1] {
					it.bmax[1] = y
				}
				if z > it.bmax[2] {
					it.bmax[2] = z
				}
			}
			it.bmin[1] = uint16(math.Floor(float64(it.bmin[1]) * float64(params.Ch) / float64(params.Cs)))
			it.bmax[1] = uint16(math.Ceil(float64(it.bmax[1]) * float64(params.Ch) / float64(params.Cs)))
		}
	}

	curNode := 0
	subdivide(items, params.PolyCount, 0, params.PolyCount, &curNode, nodes)
}

func classifyOffMeshPoint(pt, bmin, bmax [3]float32) uint8 {
	const (
		xp uint8 = 1 << 0
		zp uint8 = 1 << 1
		xm uint8 = 1 << 2
		zm uint8 = 1 << 3
	)
	var outcode uint8
	if pt[0] >= bmax[0] {
		outcode |= xp
	}
	if pt[2] >= bmax[2] {
		outcode |= zp
	}
	if pt[0] < bmin[0] {
		outcode |= xm
	}
	if pt[2] < bmin[2] {
		outcode |= zm
	}
	switch outcode {
	case xp:
		return 0
	case xp | zp:
		return 1
	case zp:
		return 2
	case xm | zp:
		return 3
	case xm:
		return 4
	case xm | zm:
		return 5
	case zm:
		return 6
	case xp | zm:
		return 7
	}
	return 0xff
}

// CreateNavMeshData builds navigation mesh tile data from the provided parameters.
func CreateNavMeshData(params *NavMeshCreateParams) ([]byte, int, bool) {
	if params.Nvp > VertsPerPolygon {
		return nil, 0, false
	}
	if params.VertCount >= 0xffff {
		return nil, 0, false
	}
	if params.VertCount == 0 || params.Verts == nil {
		return nil, 0, false
	}
	if params.PolyCount == 0 || params.Polys == nil {
		return nil, 0, false
	}

	nvp := params.Nvp

	// Classify off-mesh connection points
	var offMeshConClass []uint8
	storedOffMeshConCount := 0
	offMeshConLinkCount := 0

	if params.OffMeshConCount > 0 {
		offMeshConClass = make([]uint8, params.OffMeshConCount*2)

		hmin := float32(math.MaxFloat32)
		hmax := float32(-math.MaxFloat32)

		if params.DetailVerts != nil && params.DetailVertsCount > 0 {
			for i := 0; i < params.DetailVertsCount; i++ {
				h := params.DetailVerts[i*3+1]
				if h < hmin {
					hmin = h
				}
				if h > hmax {
					hmax = h
				}
			}
		} else {
			for i := 0; i < params.VertCount; i++ {
				iv := params.Verts[i*3:]
				h := params.Bmin[1] + float32(iv[1])*params.Ch
				if h < hmin {
					hmin = h
				}
				if h > hmax {
					hmax = h
				}
			}
		}
		hmin -= params.WalkableClimb
		hmax += params.WalkableClimb
		var bmin, bmax [3]float32
		copy(bmin[:], params.Bmin[:])
		copy(bmax[:], params.Bmax[:])
		bmin[1] = hmin
		bmax[1] = hmax

		for i := 0; i < params.OffMeshConCount; i++ {
			p0 := params.OffMeshConVerts[(i*2+0)*3:]
			p1 := params.OffMeshConVerts[(i*2+1)*3:]
			offMeshConClass[i*2+0] = classifyOffMeshPoint([3]float32{p0[0], p0[1], p0[2]}, bmin, bmax)
			offMeshConClass[i*2+1] = classifyOffMeshPoint([3]float32{p1[0], p1[1], p1[2]}, bmin, bmax)

			if offMeshConClass[i*2+0] == 0xff {
				if p0[1] < bmin[1] || p0[1] > bmax[1] {
					offMeshConClass[i*2+0] = 0
				}
			}

			if offMeshConClass[i*2+0] == 0xff {
				offMeshConLinkCount++
			}
			if offMeshConClass[i*2+1] == 0xff {
				offMeshConLinkCount++
			}
			if offMeshConClass[i*2+0] == 0xff {
				storedOffMeshConCount++
			}
		}
	}

	totPolyCount := params.PolyCount + storedOffMeshConCount
	totVertCount := params.VertCount + storedOffMeshConCount*2

	edgeCount := 0
	portalCount := 0
	for i := 0; i < params.PolyCount; i++ {
		p := params.Polys[i*2*nvp:]
		for j := 0; j < nvp; j++ {
			if p[j] == MeshNullIdx {
				break
			}
			edgeCount++
			if p[nvp+j]&0x8000 != 0 {
				dir := p[nvp+j] & 0xf
				if dir != 0xf {
					portalCount++
				}
			}
		}
	}

	maxLinkCount := edgeCount + portalCount*2 + offMeshConLinkCount*2

	uniqueDetailVertCount := 0
	detailTriCount := 0
	if params.DetailMeshes != nil {
		detailTriCount = params.DetailTriCount
		for i := 0; i < params.PolyCount; i++ {
			p := params.Polys[i*nvp*2:]
			ndv := int(params.DetailMeshes[i*4+1])
			nv := 0
			for j := 0; j < nvp; j++ {
				if p[j] == MeshNullIdx {
					break
				}
				nv++
			}
			ndv -= nv
			uniqueDetailVertCount += ndv
		}
	} else {
		for i := 0; i < params.PolyCount; i++ {
			p := params.Polys[i*nvp*2:]
			nv := 0
			for j := 0; j < nvp; j++ {
				if p[j] == MeshNullIdx {
					break
				}
				nv++
			}
			detailTriCount += nv - 2
		}
	}

	// Calculate data size
	headerSize := Align4(int(unsafeSizeOfMeshHeader()))
	vertsSize := Align4(int(unsafeSizeOfFloat32()) * 3 * totVertCount)
	polysSize := Align4(int(unsafeSizeOfPoly()) * totPolyCount)
	linksSize := Align4(int(unsafeSizeOfLink()) * maxLinkCount)
	detailMeshesSize := Align4(int(unsafeSizeOfPolyDetail()) * params.PolyCount)
	detailVertsSize := Align4(int(unsafeSizeOfFloat32()) * 3 * uniqueDetailVertCount)
	detailTrisSize := Align4(int(unsafeSizeOfUint8()) * 4 * detailTriCount)
	var bvTreeSize int
	if params.BuildBvTree {
		bvTreeSize = Align4(int(unsafeSizeOfBVNode()) * params.PolyCount * 2)
	}
	offMeshConsSize := Align4(int(unsafeSizeOfOffMeshConnection()) * storedOffMeshConCount)

	dataSize := headerSize + vertsSize + polysSize + linksSize +
		detailMeshesSize + detailVertsSize + detailTrisSize +
		bvTreeSize + offMeshConsSize

	data := make([]byte, dataSize)

	d := data

	// Write header
	header := &MeshHeader{
		Magic:           NavMeshMagic,
		Version:         NavMeshVersion,
		X:               int32(params.TileX),
		Y:               int32(params.TileY),
		Layer:           int32(params.TileLayer),
		UserID:          params.UserID,
		PolyCount:       int32(totPolyCount),
		VertCount:       int32(totVertCount),
		MaxLinkCount:    int32(maxLinkCount),
		DetailMeshCount: int32(params.PolyCount),
		DetailVertCount: int32(uniqueDetailVertCount),
		DetailTriCount:  int32(detailTriCount),
		BVQuantFactor:   1.0 / params.Cs,
		OffMeshBase:     int32(params.PolyCount),
		WalkableHeight:  params.WalkableHeight,
		WalkableRadius:  params.WalkableRadius,
		WalkableClimb:   params.WalkableClimb,
		OffMeshConCount: int32(storedOffMeshConCount),
	}
	if params.BuildBvTree {
		header.BVNodeCount = int32(params.PolyCount * 2)
	}
	header.Bmin = params.Bmin
	header.Bmax = params.Bmax

	writeMeshHeaderToBytes(d, header)
	d = d[headerSize:]

	// Write vertices
	navVerts := make([]float32, totVertCount*3)
	for i := 0; i < params.VertCount; i++ {
		iv := params.Verts[i*3:]
		navVerts[i*3+0] = params.Bmin[0] + float32(iv[0])*params.Cs
		navVerts[i*3+1] = params.Bmin[1] + float32(iv[1])*params.Ch
		navVerts[i*3+2] = params.Bmin[2] + float32(iv[2])*params.Cs
	}
	n := 0
	for i := 0; i < params.OffMeshConCount; i++ {
		if offMeshConClass[i*2+0] == 0xff {
			linkv := params.OffMeshConVerts[i*2*3:]
			v := navVerts[(params.VertCount+n*2)*3:]
			copy(v[0:3], linkv[0:3])
			copy(v[3:6], linkv[3:6])
			n++
		}
	}
	for i := 0; i < len(navVerts); i++ {
		binary.LittleEndian.PutUint32(d, math.Float32bits(navVerts[i]))
		d = d[4:]
	}

	// Write polygons
	navPolys := make([]Poly, totPolyCount)
	src := params.Polys
	for i := 0; i < params.PolyCount; i++ {
		p := &navPolys[i]
		p.FirstLink = NullLink
		p.VertCount = 0
		p.Flags = params.PolyFlags[i]
		p.SetArea(params.PolyAreas[i])
		p.SetType(PolyTypeGround)
		for j := 0; j < nvp; j++ {
			if src[j] == MeshNullIdx {
				break
			}
			p.Verts[j] = src[j]
			if src[nvp+j]&0x8000 != 0 {
				dir := src[nvp+j] & 0xf
				switch dir {
				case 0xf:
					p.Neis[j] = 0
				case 0:
					p.Neis[j] = ExtLink | 4
				case 1:
					p.Neis[j] = ExtLink | 2
				case 2:
					p.Neis[j] = ExtLink
				case 3:
					p.Neis[j] = ExtLink | 6
				}
			} else {
				p.Neis[j] = src[nvp+j] + 1
			}
			p.VertCount++
		}
		src = src[nvp*2:]
	}
	n = 0
	for i := 0; i < params.OffMeshConCount; i++ {
		if offMeshConClass[i*2+0] == 0xff {
			p := &navPolys[params.PolyCount+n]
			p.FirstLink = NullLink
			p.VertCount = 2
			p.Verts[0] = uint16(params.VertCount + n*2 + 0)
			p.Verts[1] = uint16(params.VertCount + n*2 + 1)
			p.Flags = params.OffMeshConFlags[i]
			p.SetArea(params.OffMeshConAreas[i])
			p.SetType(PolyTypeOffMeshConnection)
			n++
		}
	}

	for i := 0; i < len(navPolys); i++ {
		writePolyToBytes(d, &navPolys[i])
		d = d[polysSize/totPolyCount:]
	}

	// Skip links
	d = d[linksSize:]

	// Write detail meshes
	navDMeshes := make([]PolyDetail, params.PolyCount)
	if params.DetailMeshes != nil {
		vbase := uint16(0)
		for i := 0; i < params.PolyCount; i++ {
			ndv := int(params.DetailMeshes[i*4+1])
			nv := int(navPolys[i].VertCount)
			navDMeshes[i].VertBase = uint32(vbase)
			navDMeshes[i].VertCount = uint8(ndv - nv)
			navDMeshes[i].TriBase = params.DetailMeshes[i*4+2]
			navDMeshes[i].TriCount = uint8(params.DetailMeshes[i*4+3])
			vbase += uint16(ndv - nv)
		}
	} else {
		tbase := 0
		for i := 0; i < params.PolyCount; i++ {
			nv := int(navPolys[i].VertCount)
			navDMeshes[i].VertBase = 0
			navDMeshes[i].VertCount = 0
			navDMeshes[i].TriBase = uint32(tbase)
			navDMeshes[i].TriCount = uint8(nv - 2)
			tbase += nv - 2
		}
	}

	for i := 0; i < len(navDMeshes); i++ {
		writePolyDetailToBytes(d, &navDMeshes[i])
		d = d[detailMeshesSize/params.PolyCount:]
	}

	// Write detail verts
	if uniqueDetailVertCount > 0 {
		vbase := uint16(0)
		navDVerts := make([]float32, uniqueDetailVertCount*3)
		for i := 0; i < params.PolyCount; i++ {
			vb := int(params.DetailMeshes[i*4+0])
			ndv := int(params.DetailMeshes[i*4+1])
			nv := int(navPolys[i].VertCount)
			if ndv-nv > 0 {
				copy(navDVerts[vbase*3:], params.DetailVerts[(vb+nv)*3:(vb+ndv)*3])
				vbase += uint16(ndv - nv)
			}
		}
		for i := 0; i < len(navDVerts); i++ {
			binary.LittleEndian.PutUint32(d, math.Float32bits(navDVerts[i]))
			d = d[4:]
		}
	} else {
		d = d[detailVertsSize:]
	}

	// Write detail tris
	if params.DetailMeshes != nil {
		for i := 0; i < params.DetailTriCount*4; i++ {
			d[0] = params.DetailTris[i]
			d = d[1:]
		}
	} else {
		for i := 0; i < params.PolyCount; i++ {
			nv := int(navPolys[i].VertCount)
			for j := 2; j < nv; j++ {
				d[0] = 0
				d[1] = uint8(j - 1)
				d[2] = uint8(j)
				d[3] = (1 << 2)
				if j == 2 {
					d[3] |= (1 << 0)
				}
				if j == nv-1 {
					d[3] |= (1 << 4)
				}
				d = d[4:]
			}
		}
	}

	// Write BVTree
	if params.BuildBvTree {
		bvNodes := make([]BVNode, params.PolyCount*2)
		createBVTree(params, bvNodes)
		for i := 0; i < params.PolyCount*2; i++ {
			writeBVNodeToBytes(d, &bvNodes[i])
			d = d[bvTreeSize/(params.PolyCount*2):]
		}
	}

	// Write off-mesh connections
	n = 0
	for i := 0; i < params.OffMeshConCount; i++ {
		if offMeshConClass[i*2+0] == 0xff {
			con := OffMeshConnection{
				Poly:   uint16(params.PolyCount + n),
				Rad:    params.OffMeshConRad[i],
				Flags:  0,
				Side:   offMeshConClass[i*2+1],
				UserID: 0,
			}
			endPts := params.OffMeshConVerts[i*2*3:]
			copy(con.Pos[0:3], endPts[0:3])
			copy(con.Pos[3:6], endPts[3:6])
			if params.OffMeshConDir[i] != 0 {
				con.Flags = uint8(OffMeshConBidir)
			}
			if params.OffMeshConUserID != nil {
				con.UserID = params.OffMeshConUserID[i]
			}
			writeOffMeshConnectionToBytes(d, &con)
			d = d[offMeshConsSize/storedOffMeshConCount:]
			n++
		}
	}

	return data, dataSize, true
}

func writeMeshHeaderToBytes(data []byte, h *MeshHeader) {
	o := 0
	binary.LittleEndian.PutUint32(data[o:], uint32(h.Magic))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.Version))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.X))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.Y))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.Layer))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], h.UserID)
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.PolyCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.VertCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.MaxLinkCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.DetailMeshCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.DetailVertCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.DetailTriCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.BVNodeCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.OffMeshConCount))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], uint32(h.OffMeshBase))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.WalkableHeight))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.WalkableRadius))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.WalkableClimb))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.Bmin[0]))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.Bmin[1]))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.Bmin[2]))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.Bmax[0]))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.Bmax[1]))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.Bmax[2]))
	o += 4
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(h.BVQuantFactor))
	o += 4
}

func writePolyToBytes(data []byte, p *Poly) {
	o := 0
	binary.LittleEndian.PutUint32(data[o:], p.FirstLink)
	o += 4
	for j := 0; j < VertsPerPolygon; j++ {
		binary.LittleEndian.PutUint16(data[o:], p.Verts[j])
		o += 2
	}
	for j := 0; j < VertsPerPolygon; j++ {
		binary.LittleEndian.PutUint16(data[o:], p.Neis[j])
		o += 2
	}
	binary.LittleEndian.PutUint16(data[o:], p.Flags)
	o += 2
	data[o] = p.VertCount
	o++
	data[o] = p.areaAndtype
	o++
}

func writePolyDetailToBytes(data []byte, pd *PolyDetail) {
	o := 0
	binary.LittleEndian.PutUint32(data[o:], pd.VertBase)
	o += 4
	binary.LittleEndian.PutUint32(data[o:], pd.TriBase)
	o += 4
	data[o] = pd.VertCount
	o++
	data[o] = pd.TriCount
	o++
}

func writeBVNodeToBytes(data []byte, n *BVNode) {
	o := 0
	for j := 0; j < 3; j++ {
		binary.LittleEndian.PutUint16(data[o:], n.Bmin[j])
		o += 2
	}
	for j := 0; j < 3; j++ {
		binary.LittleEndian.PutUint16(data[o:], n.Bmax[j])
		o += 2
	}
	binary.LittleEndian.PutUint32(data[o:], uint32(n.I))
	o += 4
}

func writeOffMeshConnectionToBytes(data []byte, c *OffMeshConnection) {
	o := 0
	for j := 0; j < 6; j++ {
		binary.LittleEndian.PutUint32(data[o:], math.Float32bits(c.Pos[j]))
		o += 4
	}
	binary.LittleEndian.PutUint32(data[o:], math.Float32bits(c.Rad))
	o += 4
	binary.LittleEndian.PutUint16(data[o:], c.Poly)
	o += 2
	data[o] = c.Flags
	o++
	data[o] = c.Side
	o++
	binary.LittleEndian.PutUint32(data[o:], c.UserID)
	o += 4
}

// NavMeshHeaderSwapEndian swaps the endianness of the tile data's header.
func NavMeshHeaderSwapEndian(data []byte) bool {
	if len(data) < int(unsafeSizeOfMeshHeader()) {
		return false
	}
	h := (*MeshHeader)(unsafe.Pointer(&data[0]))
	swappedMagic := uint32(NavMeshMagic)
	swappedVersion := uint32(NavMeshVersion)
	swapEndian32(&swappedMagic)
	swapEndian32(&swappedVersion)

	if (uint32(h.Magic) != uint32(NavMeshMagic) || uint32(h.Version) != uint32(NavMeshVersion)) &&
		(uint32(h.Magic) != swappedMagic || uint32(h.Version) != swappedVersion) {
		return false
	}

	swapEndian32((*uint32)(unsafe.Pointer(&h.Magic)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Version)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.X)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Y)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Layer)))
	swapEndian32(&h.UserID)
	swapEndian32((*uint32)(unsafe.Pointer(&h.PolyCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.VertCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.MaxLinkCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.DetailMeshCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.DetailVertCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.DetailTriCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.BVNodeCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.OffMeshConCount)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.OffMeshBase)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.WalkableHeight)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.WalkableRadius)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.WalkableClimb)))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Bmin[0])))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Bmin[1])))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Bmin[2])))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Bmax[0])))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Bmax[1])))
	swapEndian32((*uint32)(unsafe.Pointer(&h.Bmax[2])))
	swapEndian32((*uint32)(unsafe.Pointer(&h.BVQuantFactor)))

	return true
}

// NavMeshDataSwapEndian swaps endianness of the tile data.
func NavMeshDataSwapEndian(data []byte) bool {
	if len(data) < int(unsafeSizeOfMeshHeader()) {
		return false
	}
	h := (*MeshHeader)(unsafe.Pointer(&data[0]))
	if h.Magic != NavMeshMagic {
		return false
	}
	if h.Version != NavMeshVersion {
		return false
	}

	headerSize := Align4(int(unsafeSizeOfMeshHeader()))
	vertsSize := Align4(int(unsafeSizeOfFloat32()) * 3 * int(h.VertCount))
	polysSize := Align4(int(unsafeSizeOfPoly()) * int(h.PolyCount))
	linksSize := Align4(int(unsafeSizeOfLink()) * int(h.MaxLinkCount))
	detailMeshesSize := Align4(int(unsafeSizeOfPolyDetail()) * int(h.DetailMeshCount))
	detailVertsSize := Align4(int(unsafeSizeOfFloat32()) * 3 * int(h.DetailVertCount))
	detailTrisSize := Align4(int(unsafeSizeOfUint8()) * 4 * int(h.DetailTriCount))
	bvtreeSize := Align4(int(unsafeSizeOfBVNode()) * int(h.BVNodeCount))
	Align4(int(unsafeSizeOfOffMeshConnection()) * int(h.OffMeshConCount))

	d := data[headerSize:]

	// Swap verts
	verts := readFloat32Slice(d, vertsSize)
	for i := 0; i < len(verts); i++ {
		swapEndian32((*uint32)(unsafe.Pointer(&verts[i])))
	}
	d = d[vertsSize:]

	// Swap polys
	polys := readPolySlice(d, int(h.PolyCount))
	for i := 0; i < len(polys); i++ {
		p := &polys[i]
		for j := 0; j < VertsPerPolygon; j++ {
			swapEndian16(&p.Verts[j])
			swapEndian16(&p.Neis[j])
		}
		swapEndian16(&p.Flags)
	}
	d = d[polysSize:]

	// Skip links
	d = d[linksSize:]

	// Swap detail meshes
	detailMeshes := readPolyDetailSlice(d, int(h.DetailMeshCount))
	for i := 0; i < len(detailMeshes); i++ {
		pd := &detailMeshes[i]
		swapEndian32(&pd.VertBase)
		swapEndian32(&pd.TriBase)
	}
	d = d[detailMeshesSize:]

	// Swap detail verts
	detailVerts := readFloat32Slice(d, detailVertsSize)
	for i := 0; i < len(detailVerts); i++ {
		swapEndian32((*uint32)(unsafe.Pointer(&detailVerts[i])))
	}
	d = d[detailVertsSize:]

	// Skip detail tris
	d = d[detailTrisSize:]

	// Swap BV tree
	bvTree := readBVNodeSlice(d, int(h.BVNodeCount))
	for i := 0; i < len(bvTree); i++ {
		node := &bvTree[i]
		for j := 0; j < 3; j++ {
			swapEndian16(&node.Bmin[j])
			swapEndian16(&node.Bmax[j])
		}
		swapEndian32((*uint32)(unsafe.Pointer(&node.I)))
	}
	d = d[bvtreeSize:]

	// Swap off-mesh connections
	offMeshCons := readOffMeshConnectionSlice(d, int(h.OffMeshConCount))
	for i := 0; i < len(offMeshCons); i++ {
		con := &offMeshCons[i]
		for j := 0; j < 6; j++ {
			swapEndian32((*uint32)(unsafe.Pointer(&con.Pos[j])))
		}
		swapEndian32((*uint32)(unsafe.Pointer(&con.Rad)))
		swapEndian16(&con.Poly)
	}

	return true
}

func swapEndian16(v *uint16) {
	*v = (*v >> 8) | (*v << 8)
}

func swapEndian32(v *uint32) {
	*v = (*v >> 24) | ((*v >> 8) & 0xff00) | ((*v << 8) & 0xff0000) | (*v << 24)
}
