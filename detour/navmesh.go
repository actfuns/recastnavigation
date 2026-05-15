package detour

import (
	"encoding/binary"
	"math"
	"unsafe"
)

// NavMesh is a navigation mesh based on tiles of convex polygons.
type NavMesh struct {
	Params      NavMeshParams
	Orig        [3]float32
	TileWidth   float32
	TileHeight  float32
	MaxTiles    int32
	TileLutSize int32
	TileLutMask int32
	PosLookup   []*MeshTile
	NextFree    *MeshTile
	Tiles       []MeshTile
	SaltBits    uint32
	TileBits    uint32
	PolyBits    uint32
}

func (m *NavMesh) Init(params *NavMeshParams) error {
	m.Params = *params
	m.Orig = params.Orig
	m.TileWidth = params.TileWidth
	m.TileHeight = params.TileHeight

	// Init tiles
	m.MaxTiles = params.MaxTiles
	m.TileLutSize = int32(NextPow2(uint32(params.MaxTiles / 4)))
	if m.TileLutSize == 0 {
		m.TileLutSize = 1
	}
	m.TileLutMask = m.TileLutSize - 1

	m.Tiles = make([]MeshTile, m.MaxTiles)
	m.PosLookup = make([]*MeshTile, m.TileLutSize)

	m.NextFree = &m.Tiles[0]
	for i := int32(0); i < m.MaxTiles-1; i++ {
		m.Tiles[i].Salt = 1
		m.Tiles[i].Next = &m.Tiles[i+1]
	}
	m.Tiles[m.MaxTiles-1].Salt = 1
	m.Tiles[m.MaxTiles-1].Next = nil

	// Init ID generator values
	m.TileBits = Ilog2(NextPow2(uint32(params.MaxTiles)))
	m.PolyBits = Ilog2(NextPow2(uint32(params.MaxPolys)))

	if int(m.TileBits+m.PolyBits) > 31 {
		return ErrInvalidParam
	}

	m.SaltBits = uint32(Min(31, 32-int(m.TileBits)-int(m.PolyBits)))

	if m.SaltBits < 10 {
		return ErrInvalidParam
	}

	return nil
}

// InitSingleTile initializes the navigation mesh for single tile use.
func (m *NavMesh) InitSingleTile(data []byte, flags int) error {
	hdr, err := decodeMeshHeader(data)
	if err != nil {
		return err
	}

	if hdr.Magic != NavMeshMagic {
		return ErrWrongMagic
	}
	if hdr.Version != NavMeshVersion {
		return ErrWrongVersion
	}

	var params NavMeshParams
	params.Orig = hdr.Bmin
	params.TileWidth = hdr.Bmax[0] - hdr.Bmin[0]
	params.TileHeight = hdr.Bmax[2] - hdr.Bmin[2]
	params.MaxTiles = 1
	params.MaxPolys = hdr.PolyCount

	if err := m.Init(&params); err != nil {
		return err
	}

	_, err = m.AddTile(data, flags, 0)
	return err
}

// GetParams returns the navigation mesh initialization params.
func (m *NavMesh) GetParams() *NavMeshParams {
	return &m.Params
}

func overlapSlabs(amin, amax, bmin, bmax [2]float32, px, py float32) bool {
	minx := Max(amin[0]+px, bmin[0]+px)
	maxx := Min(amax[0]-px, bmax[0]-px)
	if minx > maxx {
		return false
	}

	ad := (amax[1] - amin[1]) / (amax[0] - amin[0])
	ak := amin[1] - ad*amin[0]
	bd := (bmax[1] - bmin[1]) / (bmax[0] - bmin[0])
	bk := bmin[1] - bd*bmin[0]
	aminy := ad*minx + ak
	amaxy := ad*maxx + ak
	bminy := bd*minx + bk
	bmaxy := bd*maxx + bk
	dmin := bminy - aminy
	dmax := bmaxy - amaxy

	if dmin*dmax < 0 {
		return true
	}

	thr := py * py * 4
	if dmin*dmin <= thr || dmax*dmax <= thr {
		return true
	}

	return false
}

func getSlabCoord(va [3]float32, side int32) float32 {
	switch side {
	case 0, 4:
		return va[0]
	case 2, 6:
		return va[2]
	}
	return 0
}

func calcSlabEndPoints(va, vb [3]float32, bmin, bmax []float32, side int32) {
	switch side {
	case 0, 4:
		if va[2] < vb[2] {
			bmin[0] = va[2]
			bmin[1] = va[1]
			bmax[0] = vb[2]
			bmax[1] = vb[1]
		} else {
			bmin[0] = vb[2]
			bmin[1] = vb[1]
			bmax[0] = va[2]
			bmax[1] = va[1]
		}
	case 2, 6:
		if va[0] < vb[0] {
			bmin[0] = va[0]
			bmin[1] = va[1]
			bmax[0] = vb[0]
			bmax[1] = vb[1]
		} else {
			bmin[0] = vb[0]
			bmin[1] = vb[1]
			bmax[0] = va[0]
			bmax[1] = va[1]
		}
	}
}

func computeTileHash(x, y int32, mask int32) int {
	const h1 uint32 = 0x8da6b343
	const h2 uint32 = 0xd8163841
	n := h1*uint32(x) + h2*uint32(y)
	return int(n & uint32(mask))
}

func allocLink(tile *MeshTile) uint32 {
	if tile.LinksFreeList == NullLink {
		return NullLink
	}
	link := tile.LinksFreeList
	tile.LinksFreeList = tile.Links[link].Next
	return link
}

func freeLink(tile *MeshTile, link uint32) {
	tile.Links[link].Next = tile.LinksFreeList
	tile.LinksFreeList = link
}

// FindConnectingPolys finds connecting polygons in a neighbour tile.
func (m *NavMesh) findConnectingPolys(va, vb [3]float32, tile *MeshTile, side int32, con []PolyRef, conarea []float32, maxcon int) int {
	if tile == nil {
		return 0
	}

	var amin, amax [2]float32
	calcSlabEndPoints(va, vb, amin[:], amax[:], side)
	apos := getSlabCoord(va, side)

	var bmin, bmax [2]float32
	medge := uint16(ExtLink) | uint16(side)
	n := 0

	base := m.GetPolyRefBase(tile)

	for i := 0; i < int(tile.Header.PolyCount); i++ {
		poly := &tile.Polys[i]
		nv := int(poly.VertCount)
		for j := 0; j < nv; j++ {
			if poly.Neis[j] != medge {
				continue
			}

			vc := [3]float32{tile.Verts[poly.Verts[j]*3], tile.Verts[poly.Verts[j]*3+1], tile.Verts[poly.Verts[j]*3+2]}
			vd := [3]float32{tile.Verts[poly.Verts[(j+1)%nv]*3], tile.Verts[poly.Verts[(j+1)%nv]*3+1], tile.Verts[poly.Verts[(j+1)%nv]*3+2]}
			bpos := getSlabCoord(vc, side)

			if Abs(apos-bpos) > 0.01 {
				continue
			}

			calcSlabEndPoints(vc, vd, bmin[:], bmax[:], side)

			if !overlapSlabs(amin, amax, bmin, bmax, 0.01, tile.Header.WalkableClimb) {
				continue
			}

			if n < maxcon {
				conarea[n*2+0] = Max(amin[0], bmin[0])
				conarea[n*2+1] = Min(amax[0], bmax[0])
				con[n] = base | PolyRef(i)
				n++
			}
			break
		}
	}
	return n
}

func (m *NavMesh) unconnectLinks(tile, target *MeshTile) {
	if tile == nil || target == nil {
		return
	}

	targetNum := m.DecodePolyIdTile(PolyRef(m.GetTileRef(target)))

	for i := 0; i < int(tile.Header.PolyCount); i++ {
		poly := &tile.Polys[i]
		j := uint32(poly.FirstLink)
		pj := uint32(NullLink)
		for j != NullLink {
			if m.DecodePolyIdTile(tile.Links[j].Ref) == targetNum {
				nj := tile.Links[j].Next
				if pj == NullLink {
					poly.FirstLink = nj
				} else {
					tile.Links[pj].Next = uint32(nj)
				}
				freeLink(tile, j)
				j = nj
			} else {
				pj = uint32(j)
				j = tile.Links[j].Next
			}
		}
	}
}

func (m *NavMesh) connectExtLinks(tile, target *MeshTile, side int32) {
	if tile == nil {
		return
	}

	for i := 0; i < int(tile.Header.PolyCount); i++ {
		poly := &tile.Polys[i]
		nv := int(poly.VertCount)
		for j := 0; j < nv; j++ {
			if (poly.Neis[j] & ExtLink) == 0 {
				continue
			}

			dir := int32(poly.Neis[j] & 0xff)
			if side != -1 && dir != side {
				continue
			}

			va := [3]float32{tile.Verts[poly.Verts[j]*3], tile.Verts[poly.Verts[j]*3+1], tile.Verts[poly.Verts[j]*3+2]}
			vb := [3]float32{tile.Verts[poly.Verts[(j+1)%nv]*3], tile.Verts[poly.Verts[(j+1)%nv]*3+1], tile.Verts[poly.Verts[(j+1)%nv]*3+2]}

			var nei [4]PolyRef
			var neia [8]float32
			nnei := m.findConnectingPolys(va, vb, target, OppositeTile(dir), nei[:], neia[:], 4)
			for k := 0; k < nnei; k++ {
				idx := allocLink(tile)
				if idx != NullLink {
					link := &tile.Links[idx]
					link.Ref = nei[k]
					link.Edge = uint8(j)
					link.Side = uint8(dir)
					link.Next = poly.FirstLink
					poly.FirstLink = idx

					switch dir {
					case 0, 4:
						tmin := (neia[k*2+0] - va[2]) / (vb[2] - va[2])
						tmax := (neia[k*2+1] - va[2]) / (vb[2] - va[2])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.Bmin = uint8(math.Round(float64(Clamp(tmin, 0.0, 1.0) * 255.0)))
						link.Bmax = uint8(math.Round(float64(Clamp(tmax, 0.0, 1.0) * 255.0)))
					case 2, 6:
						tmin := (neia[k*2+0] - va[0]) / (vb[0] - va[0])
						tmax := (neia[k*2+1] - va[0]) / (vb[0] - va[0])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.Bmin = uint8(math.Round(float64(Clamp(tmin, 0.0, 1.0) * 255.0)))
						link.Bmax = uint8(math.Round(float64(Clamp(tmax, 0.0, 1.0) * 255.0)))
					}
				}
			}
		}
	}
}

func (m *NavMesh) connectExtOffMeshLinks(tile, target *MeshTile, side int32) {
	if tile == nil {
		return
	}

	oppositeSide := uint8(0xff)
	if side != -1 {
		oppositeSide = uint8(OppositeTile(side))
	}

	for i := 0; i < int(target.Header.OffMeshConCount); i++ {
		targetCon := &target.OffMeshCons[i]
		if targetCon.Side != oppositeSide {
			continue
		}

		targetPoly := &target.Polys[targetCon.Poly]
		if targetPoly.FirstLink == NullLink {
			continue
		}

		halfExtents := [3]float32{targetCon.Rad, target.Header.WalkableClimb, targetCon.Rad}

		p := [3]float32{targetCon.Pos[3], targetCon.Pos[4], targetCon.Pos[5]}
		ref, nearestPt := m.findNearestPolyInTile(tile, p, halfExtents)
		if ref == 0 {
			continue
		}
		if (nearestPt[0]-p[0])*(nearestPt[0]-p[0])+(nearestPt[2]-p[2])*(nearestPt[2]-p[2]) > targetCon.Rad*targetCon.Rad {
			continue
		}

		v := target.Verts[targetPoly.Verts[1]*3 : targetPoly.Verts[1]*3+3]
		v[0] = nearestPt[0]
		v[1] = nearestPt[1]
		v[2] = nearestPt[2]

		idx := allocLink(target)
		if idx != NullLink {
			link := &target.Links[idx]
			link.Ref = ref
			link.Edge = 1
			link.Side = oppositeSide
			link.Bmin = 0
			link.Bmax = 0
			link.Next = targetPoly.FirstLink
			targetPoly.FirstLink = idx
		}

		if targetCon.Flags&uint8(OffMeshConBidir) != 0 {
			tidx := allocLink(tile)
			if tidx != NullLink {
				landPolyIdx := uint16(m.DecodePolyIdPoly(ref))
				landPoly := &tile.Polys[landPolyIdx]
				link := &tile.Links[tidx]
				link.Ref = m.GetPolyRefBase(target) | PolyRef(targetCon.Poly)
				link.Edge = 0xff
				if side == -1 {
					link.Side = 0xff
				} else {
					link.Side = uint8(side)
				}
				link.Bmin = 0
				link.Bmax = 0
				link.Next = landPoly.FirstLink
				landPoly.FirstLink = tidx
			}
		}
	}
}

func (m *NavMesh) connectIntLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	base := m.GetPolyRefBase(tile)

	for i := 0; i < int(tile.Header.PolyCount); i++ {
		poly := &tile.Polys[i]
		poly.FirstLink = NullLink

		if poly.GetType() == PolyTypeOffMeshConnection {
			continue
		}

		for j := int(poly.VertCount) - 1; j >= 0; j-- {
			if poly.Neis[j] == 0 || (poly.Neis[j]&ExtLink) != 0 {
				continue
			}

			idx := allocLink(tile)
			if idx != NullLink {
				link := &tile.Links[idx]
				link.Ref = base | PolyRef(poly.Neis[j]-1)
				link.Edge = uint8(j)
				link.Side = 0xff
				link.Bmin = 0
				link.Bmax = 0
				link.Next = poly.FirstLink
				poly.FirstLink = idx
			}
		}
	}
}

func (m *NavMesh) baseOffMeshLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	base := m.GetPolyRefBase(tile)

	for i := 0; i < int(tile.Header.OffMeshConCount); i++ {
		con := &tile.OffMeshCons[i]
		poly := &tile.Polys[con.Poly]

		halfExtents := [3]float32{con.Rad, tile.Header.WalkableClimb, con.Rad}

		p := [3]float32{con.Pos[0], con.Pos[1], con.Pos[2]}
		ref, nearestPt := m.findNearestPolyInTile(tile, p, halfExtents)
		if ref == 0 {
			continue
		}
		if (nearestPt[0]-p[0])*(nearestPt[0]-p[0])+(nearestPt[2]-p[2])*(nearestPt[2]-p[2]) > con.Rad*con.Rad {
			continue
		}

		v := tile.Verts[poly.Verts[0]*3 : poly.Verts[0]*3+3]
		v[0] = nearestPt[0]
		v[1] = nearestPt[1]
		v[2] = nearestPt[2]

		idx := allocLink(tile)
		if idx != NullLink {
			link := &tile.Links[idx]
			link.Ref = ref
			link.Edge = 0
			link.Side = 0xff
			link.Bmin = 0
			link.Bmax = 0
			link.Next = poly.FirstLink
			poly.FirstLink = idx
		}

		tidx := allocLink(tile)
		if tidx != NullLink {
			landPolyIdx := uint16(m.DecodePolyIdPoly(ref))
			landPoly := &tile.Polys[landPolyIdx]
			link := &tile.Links[tidx]
			link.Ref = base | PolyRef(con.Poly)
			link.Edge = 0xff
			link.Side = 0xff
			link.Bmin = 0
			link.Bmax = 0
			link.Next = landPoly.FirstLink
			landPoly.FirstLink = tidx
		}
	}
}

func closestPointOnDetailEdges(tile *MeshTile, poly *Poly, pos [3]float32, closest *[3]float32, onlyBoundary bool) {
	ip := uint32(0)
	for i := range tile.Polys {
		if &tile.Polys[i] == poly {
			ip = uint32(i)
			break
		}
	}
	pd := &tile.DetailMeshes[ip]

	dmin := float32(math.MaxFloat32)
	tmin := float32(0)
	var pmin, pmax [3]float32

	for i := 0; i < int(pd.TriCount); i++ {
		tris := tile.DetailTris[(int(pd.TriBase)+i)*4 : (int(pd.TriBase)+i)*4+4]
		anyBoundaryEdge := (DetailEdgeBoundary << 0) | (DetailEdgeBoundary << 2) | (DetailEdgeBoundary << 4)
		if onlyBoundary && (tris[3]&uint8(anyBoundaryEdge)) == 0 {
			continue
		}

		var v [3][]float32
		for j := 0; j < 3; j++ {
			if tris[j] < uint8(poly.VertCount) {
				v[j] = tile.Verts[poly.Verts[tris[j]]*3 : poly.Verts[tris[j]]*3+3]
			} else {
				v[j] = tile.DetailVerts[(int(pd.VertBase)+int(tris[j])-int(poly.VertCount))*3 : (int(pd.VertBase)+int(tris[j])-int(poly.VertCount))*3+3]
			}
		}

		for k, j := 0, 2; k < 3; j, k = k, k+1 {
			if (GetDetailTriEdgeFlags(tris[3], j)&DetailEdgeBoundary) == 0 && (onlyBoundary || tris[j] < tris[k]) {
				continue
			}

			d, t := DistancePtSegSqr2D(
				[3]float32{pos[0], pos[1], pos[2]},
				[3]float32{v[j][0], v[j][1], v[j][2]},
				[3]float32{v[k][0], v[k][1], v[k][2]},
			)
			if d < dmin {
				dmin = d
				tmin = t
				pmin = [3]float32{v[j][0], v[j][1], v[j][2]}
				pmax = [3]float32{v[k][0], v[k][1], v[k][2]}
			}
		}
	}

	result := Vlerp([3]float32{pmin[0], pmin[1], pmin[2]}, [3]float32{pmax[0], pmax[1], pmax[2]}, tmin)
	*closest = result
}

// GetPolyHeight returns the height of the polygon at the given position.
func (m *NavMesh) getPolyHeight(tile *MeshTile, poly *Poly, pos [3]float32) (float32, bool) {
	if poly.GetType() == PolyTypeOffMeshConnection {
		return 0, false
	}

	ip := uint32(0)
	for i := range tile.Polys {
		if &tile.Polys[i] == poly {
			ip = uint32(i)
			break
		}
	}
	pd := &tile.DetailMeshes[ip]

	var verts [VertsPerPolygon * 3]float32
	nv := int(poly.VertCount)
	for i := 0; i < nv; i++ {
		src := tile.Verts[poly.Verts[i]*3 : poly.Verts[i]*3+3]
		verts[i*3+0] = src[0]
		verts[i*3+1] = src[1]
		verts[i*3+2] = src[2]
	}

	if !PointInPolygon(pos, verts[:], nv) {
		return 0, false
	}

	for j := 0; j < int(pd.TriCount); j++ {
		t := tile.DetailTris[(int(pd.TriBase)+j)*4 : (int(pd.TriBase)+j)*4+4]
		var v [3][]float32
		for k := 0; k < 3; k++ {
			if t[k] < uint8(poly.VertCount) {
				v[k] = tile.Verts[poly.Verts[t[k]]*3 : poly.Verts[t[k]]*3+3]
			} else {
				v[k] = tile.DetailVerts[(int(pd.VertBase)+int(t[k])-int(poly.VertCount))*3 : (int(pd.VertBase)+int(t[k])-int(poly.VertCount))*3+3]
			}
		}
		ok, h := ClosestHeightPointTriangle(
			pos,
			[3]float32{v[0][0], v[0][1], v[0][2]},
			[3]float32{v[1][0], v[1][1], v[1][2]},
			[3]float32{v[2][0], v[2][1], v[2][2]},
		)
		if ok {
			return h, true
		}
	}

	var closest [3]float32
	closestPointOnDetailEdges(tile, poly, pos, &closest, false)
	return closest[1], true
}

func (m *NavMesh) closestPointOnPoly(ref PolyRef, pos [3]float32) ([3]float32, bool) {
	tile, poly := m.GetTileAndPolyByRefUnsafe(ref)
	var closest [3]float32

	h, ok := m.getPolyHeight(tile, poly, pos)
	if ok {
		closest = [3]float32{pos[0], h, pos[2]}
		return closest, true
	}

	if poly.GetType() == PolyTypeOffMeshConnection {
		v0 := tile.Verts[poly.Verts[0]*3 : poly.Verts[0]*3+3]
		v1 := tile.Verts[poly.Verts[1]*3 : poly.Verts[1]*3+3]
		_, t := DistancePtSegSqr2D(
			pos,
			[3]float32{v0[0], v0[1], v0[2]},
			[3]float32{v1[0], v1[1], v1[2]},
		)
		r := Vlerp([3]float32{v0[0], v0[1], v0[2]}, [3]float32{v1[0], v1[1], v1[2]}, t)
		return r, false
	}

	closestPointOnDetailEdges(tile, poly, pos, &closest, true)
	return closest, false
}

func (m *NavMesh) findNearestPolyInTile(tile *MeshTile, center, halfExtents [3]float32) (PolyRef, [3]float32) {
	var bmin, bmax [3]float32
	bmin = Vsub(center, halfExtents)
	bmax = Vadd(center, halfExtents)

	var polys [128]PolyRef
	polyCount := m.queryPolygonsInTile(tile, bmin, bmax, polys[:], 128)

	nearest := PolyRef(0)
	var nearestPt [3]float32
	nearestDistanceSqr := float32(math.MaxFloat32)
	for i := 0; i < polyCount; i++ {
		ref := polys[i]
		var diff [3]float32
		var d float32
		closestPtPoly, posOverPoly := m.closestPointOnPoly(ref, center)

		diff = Vsub(center, closestPtPoly)
		if posOverPoly {
			d = Abs(diff[1]) - tile.Header.WalkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = VlenSqr(diff)
		}

		if d < nearestDistanceSqr {
			nearestPt = closestPtPoly
			nearestDistanceSqr = d
			nearest = ref
		}
	}

	return nearest, nearestPt
}

func (m *NavMesh) queryPolygonsInTile(tile *MeshTile, qmin, qmax [3]float32, polys []PolyRef, maxPolys int) int {
	if tile.BVTree != nil {
		node := &tile.BVTree[0]
		bvCount := len(tile.BVTree)
		tbmin := tile.Header.Bmin[:]
		tbmax := tile.Header.Bmax[:]
		qfac := tile.Header.BVQuantFactor

		var bmin, bmax [3]uint16
		minx := Clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := Clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := Clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := Clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := Clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := Clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]

		bmin[0] = uint16(uint32(qfac*minx)) & 0xfffe
		bmin[1] = uint16(uint32(qfac*miny)) & 0xfffe
		bmin[2] = uint16(uint32(qfac*minz)) & 0xfffe
		bmax[0] = uint16(uint32(qfac*maxx+1)) | 1
		bmax[1] = uint16(uint32(qfac*maxy+1)) | 1
		bmax[2] = uint16(uint32(qfac*maxz+1)) | 1

		base := m.GetPolyRefBase(tile)
		n := 0
		for nodeIdx := 0; nodeIdx < bvCount; {
			node = &tile.BVTree[nodeIdx]
			overlap := OverlapQuantBounds(bmin, bmax, node.Bmin, node.Bmax)
			isLeafNode := node.I >= 0

			if isLeafNode && overlap {
				if n < maxPolys {
					polys[n] = base | PolyRef(node.I)
					n++
				}
			}

			if overlap || isLeafNode {
				nodeIdx++
			} else {
				escapeIndex := int(-node.I)
				nodeIdx += escapeIndex
			}
		}

		return n
	} else {
		var bmin, bmax [3]float32
		n := 0
		base := m.GetPolyRefBase(tile)
		for i := 0; i < int(tile.Header.PolyCount); i++ {
			p := &tile.Polys[i]
			if p.GetType() == PolyTypeOffMeshConnection {
				continue
			}
			v := tile.Verts[p.Verts[0]*3 : p.Verts[0]*3+3]
			bmin[0], bmin[1], bmin[2] = v[0], v[1], v[2]
			bmax[0], bmax[1], bmax[2] = v[0], v[1], v[2]
			for j := 1; j < int(p.VertCount); j++ {
				v = tile.Verts[p.Verts[j]*3 : p.Verts[j]*3+3]
				bmin = Vmin(bmin, [3]float32{v[0], v[1], v[2]})
				bmax = Vmax(bmax, [3]float32{v[0], v[1], v[2]})
			}
			if OverlapBounds(qmin, qmax, bmin, bmax) {
				if n < maxPolys {
					polys[n] = base | PolyRef(i)
					n++
				}
			}
		}
		return n
	}
}

// AddTile adds a tile to the navigation mesh.
func (m *NavMesh) AddTile(data []byte, flags int, lastRef TileRef) (TileRef, error) {
	header, err := decodeMeshHeader(data)
	if err != nil {
		return 0, err
	}
	if header.Magic != NavMeshMagic {
		return 0, ErrWrongMagic
	}
	if header.Version != NavMeshVersion {
		return 0, ErrWrongVersion
	}

	if m.PolyBits < Ilog2(NextPow2(uint32(header.PolyCount))) {
		return 0, ErrInvalidParam
	}

	if m.GetTileAt(header.X, header.Y, header.Layer) != nil {
		return 0, ErrAlreadyOccupied
	}

	var tile *MeshTile
	if lastRef == 0 {
		if m.NextFree != nil {
			tile = m.NextFree
			m.NextFree = tile.Next
			tile.Next = nil
		}
	} else {
		tileIndex := int32(m.DecodePolyIdTile(PolyRef(lastRef)))
		if tileIndex >= m.MaxTiles {
			return 0, ErrOutOfMemory
		}
		target := &m.Tiles[tileIndex]
		var prev *MeshTile
		tile = m.NextFree
		for tile != nil && tile != target {
			prev = tile
			tile = tile.Next
		}
		if tile != target {
			return 0, ErrOutOfMemory
		}
		if prev == nil {
			m.NextFree = tile.Next
		} else {
			prev.Next = tile.Next
		}
		tile.Salt = uint32(m.DecodePolyIdSalt(PolyRef(lastRef)))
	}

	if tile == nil {
		return 0, ErrOutOfMemory
	}

	h := computeTileHash(header.X, header.Y, m.TileLutMask)
	tile.Next = m.PosLookup[h]
	m.PosLookup[h] = tile

	// Patch header pointers
	headerSize := Align4(int(unsafeSizeOfMeshHeader()))
	vertsSize := Align4(int(unsafeSizeOfFloat32()) * 3 * int(header.VertCount))
	polysSize := Align4(int(unsafeSizeOfPoly()) * int(header.PolyCount))
	linksSize := Align4(int(unsafeSizeOfLink()) * int(header.MaxLinkCount))
	detailMeshesSize := Align4(int(unsafeSizeOfPolyDetail()) * int(header.DetailMeshCount))
	detailVertsSize := Align4(int(unsafeSizeOfFloat32()) * 3 * int(header.DetailVertCount))
	detailTrisSize := Align4(int(unsafeSizeOfUint8()) * 4 * int(header.DetailTriCount))
	bvtreeSize := Align4(int(unsafeSizeOfBVNode()) * int(header.BVNodeCount))

	d := data[headerSize:]

	tile.Header = &header

	// Zero-copy reads: tile fields reference data byte slice directly.
	tile.Verts = unsafe.Slice((*float32)(unsafe.Pointer(&d[0])), vertsSize/int(unsafe.Sizeof(float32(0))))
	d = d[vertsSize:]
	tile.Polys = unsafe.Slice((*Poly)(unsafe.Pointer(&d[0])), polysSize/int(unsafe.Sizeof(Poly{})))
	d = d[polysSize:]
	tile.Links = unsafe.Slice((*Link)(unsafe.Pointer(&d[0])), linksSize/int(unsafe.Sizeof(Link{})))
	d = d[linksSize:]

	// PolyDetail has struct padding (12B) vs serialized (10B) — must copy.
	tile.DetailMeshes = readPolyDetailSlice(d, int(header.DetailMeshCount))
	d = d[detailMeshesSize:]

	if detailVertsSize > 0 {
		tile.DetailVerts = unsafe.Slice((*float32)(unsafe.Pointer(&d[0])), detailVertsSize/int(unsafe.Sizeof(float32(0))))
		d = d[detailVertsSize:]
	}
	if detailTrisSize > 0 {
		tile.DetailTris = d[:detailTrisSize]
		d = d[detailTrisSize:]
	}
	if bvtreeSize > 0 {
		tile.BVTree = unsafe.Slice((*BVNode)(unsafe.Pointer(&d[0])), bvtreeSize/int(unsafe.Sizeof(BVNode{})))
		d = d[bvtreeSize:]
	}

	tile.Data = data
	tile.Flags = flags

	// Build links freelist
	tile.LinksFreeList = 0
	if len(tile.Links) > 0 {
		tile.Links[len(tile.Links)-1].Next = NullLink
		for i := 0; i < len(tile.Links)-1; i++ {
			tile.Links[i].Next = uint32(i + 1)
		}
	}

	m.connectIntLinks(tile)
	m.baseOffMeshLinks(tile)
	m.connectExtOffMeshLinks(tile, tile, -1)

	const maxNeis = 32
	neis := make([]*MeshTile, maxNeis)

	nneis := m.getTilesAt(header.X, header.Y, neis, maxNeis)
	for j := 0; j < nneis; j++ {
		if neis[j] == tile {
			continue
		}
		m.connectExtLinks(tile, neis[j], -1)
		m.connectExtLinks(neis[j], tile, -1)
		m.connectExtOffMeshLinks(tile, neis[j], -1)
		m.connectExtOffMeshLinks(neis[j], tile, -1)
	}

	for i := int32(0); i < 8; i++ {
		nneis = m.getNeighbourTilesAt(header.X, header.Y, i, neis, maxNeis)
		for j := 0; j < nneis; j++ {
			m.connectExtLinks(tile, neis[j], i)
			m.connectExtLinks(neis[j], tile, OppositeTile(i))
			m.connectExtOffMeshLinks(tile, neis[j], i)
			m.connectExtOffMeshLinks(neis[j], tile, OppositeTile(i))
		}
	}

	result := m.GetTileRef(tile)
	return result, nil
}

// decodeMeshHeader safely decodes a MeshHeader from serialized data.
// Uses field-by-field binary decoding to avoid alignment issues from casting
// a []byte (1-byte aligned) directly to *MeshHeader (4-byte aligned required).
func decodeMeshHeader(data []byte) (h MeshHeader, err error) {
	if len(data) < int(unsafe.Sizeof(h)) {
		return h, ErrInvalidParam
	}
	h.Magic = int32(binary.LittleEndian.Uint32(data[0:4]))
	h.Version = int32(binary.LittleEndian.Uint32(data[4:8]))
	h.X = int32(binary.LittleEndian.Uint32(data[8:12]))
	h.Y = int32(binary.LittleEndian.Uint32(data[12:16]))
	h.Layer = int32(binary.LittleEndian.Uint32(data[16:20]))
	h.UserID = binary.LittleEndian.Uint32(data[20:24])
	h.PolyCount = int32(binary.LittleEndian.Uint32(data[24:28]))
	h.VertCount = int32(binary.LittleEndian.Uint32(data[28:32]))
	h.MaxLinkCount = int32(binary.LittleEndian.Uint32(data[32:36]))
	h.DetailMeshCount = int32(binary.LittleEndian.Uint32(data[36:40]))
	h.DetailVertCount = int32(binary.LittleEndian.Uint32(data[40:44]))
	h.DetailTriCount = int32(binary.LittleEndian.Uint32(data[44:48]))
	h.BVNodeCount = int32(binary.LittleEndian.Uint32(data[48:52]))
	h.OffMeshConCount = int32(binary.LittleEndian.Uint32(data[52:56]))
	h.OffMeshBase = int32(binary.LittleEndian.Uint32(data[56:60]))
	h.WalkableHeight = math.Float32frombits(binary.LittleEndian.Uint32(data[60:64]))
	h.WalkableRadius = math.Float32frombits(binary.LittleEndian.Uint32(data[64:68]))
	h.WalkableClimb = math.Float32frombits(binary.LittleEndian.Uint32(data[68:72]))
	h.Bmin[0] = math.Float32frombits(binary.LittleEndian.Uint32(data[72:76]))
	h.Bmin[1] = math.Float32frombits(binary.LittleEndian.Uint32(data[76:80]))
	h.Bmin[2] = math.Float32frombits(binary.LittleEndian.Uint32(data[80:84]))
	h.Bmax[0] = math.Float32frombits(binary.LittleEndian.Uint32(data[84:88]))
	h.Bmax[1] = math.Float32frombits(binary.LittleEndian.Uint32(data[88:92]))
	h.Bmax[2] = math.Float32frombits(binary.LittleEndian.Uint32(data[92:96]))
	h.BVQuantFactor = math.Float32frombits(binary.LittleEndian.Uint32(data[96:100]))
	return h, nil
}

// binary reader helpers using unsafe.Sizeof
func unsafeSizeOfMeshHeader() uintptr        { return unsafe.Sizeof(MeshHeader{}) }
func unsafeSizeOfFloat32() uintptr           { return unsafe.Sizeof(float32(0)) }
func unsafeSizeOfPoly() uintptr              { return unsafe.Sizeof(Poly{}) }
func unsafeSizeOfLink() uintptr              { return unsafe.Sizeof(Link{}) }
func unsafeSizeOfPolyDetail() uintptr        { return unsafe.Sizeof(PolyDetail{}) }
func unsafeSizeOfUint8() uintptr             { return unsafe.Sizeof(uint8(0)) }
func unsafeSizeOfBVNode() uintptr            { return unsafe.Sizeof(BVNode{}) }
func unsafeSizeOfOffMeshConnection() uintptr { return unsafe.Sizeof(OffMeshConnection{}) }

func readFloat32Slice(data []byte, size int) []float32 {
	count := size / int(unsafe.Sizeof(float32(0)))
	if count == 0 {
		return nil
	}
	src := unsafe.Slice((*float32)(unsafe.Pointer(&data[0])), count)
	result := make([]float32, count)
	copy(result, src)
	return result
}

func readPolySlice(data []byte, count int) []Poly {
	if count == 0 {
		return nil
	}
	src := unsafe.Slice((*Poly)(unsafe.Pointer(&data[0])), count)
	result := make([]Poly, count)
	copy(result, src)
	return result
}

func readPolyDetailSlice(data []byte, count int) []PolyDetail {
	if count == 0 {
		return nil
	}
	result := make([]PolyDetail, count)
	// stride matches Align4(unsafe.Sizeof(PolyDetail{})) = 12 in the serialized data.
	const elemSize = 12
	for i := 0; i < count; i++ {
		off := i * elemSize
		pd := &result[i]
		pd.VertBase = binary.LittleEndian.Uint32(data[off:])
		pd.TriBase = binary.LittleEndian.Uint32(data[off+4:])
		pd.VertCount = data[off+8]
		pd.TriCount = data[off+9]
	}
	return result
}

func readBVNodeSlice(data []byte, count int) []BVNode {
	if count == 0 {
		return nil
	}
	src := unsafe.Slice((*BVNode)(unsafe.Pointer(&data[0])), count)
	result := make([]BVNode, count)
	copy(result, src)
	return result
}

func readOffMeshConnectionSlice(data []byte, count int) []OffMeshConnection {
	if count == 0 {
		return nil
	}
	src := unsafe.Slice((*OffMeshConnection)(unsafe.Pointer(&data[0])), count)
	result := make([]OffMeshConnection, count)
	copy(result, src)
	return result
}

// GetTileAt gets the tile at the specified grid location.
func (m *NavMesh) GetTileAt(x, y, layer int32) *MeshTile {
	h := computeTileHash(x, y, m.TileLutMask)
	tile := m.PosLookup[h]
	for tile != nil {
		if tile.Header != nil && tile.Header.X == x && tile.Header.Y == y && tile.Header.Layer == layer {
			return tile
		}
		tile = tile.Next
	}
	return nil
}

func (m *NavMesh) getNeighbourTilesAt(x, y, side int32, tiles []*MeshTile, maxTiles int) int {
	nx, ny := x, y
	switch side {
	case 0:
		nx++
	case 1:
		nx++
		ny++
	case 2:
		ny++
	case 3:
		nx--
		ny++
	case 4:
		nx--
	case 5:
		nx--
		ny--
	case 6:
		ny--
	case 7:
		nx++
		ny--
	}
	return m.getTilesAt(nx, ny, tiles, maxTiles)
}

func (m *NavMesh) getTilesAt(x, y int32, tiles []*MeshTile, maxTiles int) int {
	n := 0
	h := computeTileHash(x, y, m.TileLutMask)
	tile := m.PosLookup[h]
	for tile != nil {
		if tile.Header != nil && tile.Header.X == x && tile.Header.Y == y {
			if n < maxTiles {
				tiles[n] = tile
				n++
			}
		}
		tile = tile.Next
	}
	return n
}

// GetTilesAt returns all tiles at the specified grid location.
func (m *NavMesh) GetTilesAt(x, y int32, tiles []*MeshTile, maxTiles int) int {
	return m.getTilesAt(x, y, tiles, maxTiles)
}

// GetTileRefAt returns the tile reference for the tile at the specified location.
func (m *NavMesh) GetTileRefAt(x, y, layer int32) TileRef {
	h := computeTileHash(x, y, m.TileLutMask)
	tile := m.PosLookup[h]
	for tile != nil {
		if tile.Header != nil && tile.Header.X == x && tile.Header.Y == y && tile.Header.Layer == layer {
			return m.GetTileRef(tile)
		}
		tile = tile.Next
	}
	return 0
}

// GetTileByRef gets the tile for the specified tile reference.
func (m *NavMesh) GetTileByRef(ref TileRef) *MeshTile {
	if ref == 0 {
		return nil
	}
	tileIndex := int32(m.DecodePolyIdTile(PolyRef(ref)))
	tileSalt := uint32(m.DecodePolyIdSalt(PolyRef(ref)))
	if tileIndex >= m.MaxTiles {
		return nil
	}
	tile := &m.Tiles[tileIndex]
	if tile.Salt != tileSalt {
		return nil
	}
	return tile
}

// GetMaxTiles returns the maximum number of tiles.
func (m *NavMesh) GetMaxTiles() int {
	return int(m.MaxTiles)
}

// GetTile gets the tile at the specified index.
func (m *NavMesh) GetTile(i int) *MeshTile {
	return &m.Tiles[i]
}

// CalcTileLoc calculates the tile grid location for the specified world position.
func (m *NavMesh) CalcTileLoc(pos [3]float32) (int, int) {
	tx := int(math.Floor(float64((pos[0] - m.Orig[0]) / m.TileWidth)))
	ty := int(math.Floor(float64((pos[2] - m.Orig[2]) / m.TileHeight)))
	return tx, ty
}

// GetTileAndPolyByRef gets the tile and polygon for the specified polygon reference.
func (m *NavMesh) GetTileAndPolyByRef(ref PolyRef) (*MeshTile, *Poly, error) {
	if ref == 0 {
		return nil, nil, ErrInvalidParam
	}
	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return nil, nil, ErrInvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return nil, nil, ErrInvalidParam
	}
	if int(ip) >= int(m.Tiles[it].Header.PolyCount) {
		return nil, nil, ErrInvalidParam
	}
	return &m.Tiles[it], &m.Tiles[it].Polys[ip], nil
}

// GetTileAndPolyByRefUnsafe returns the tile and polygon for the specified polygon reference (no validation).
func (m *NavMesh) GetTileAndPolyByRefUnsafe(ref PolyRef) (*MeshTile, *Poly) {
	_, it, ip := m.DecodePolyID(ref)
	return &m.Tiles[it], &m.Tiles[it].Polys[ip]
}

// IsValidPolyRef checks the validity of a polygon reference.
func (m *NavMesh) IsValidPolyRef(ref PolyRef) bool {
	if ref == 0 {
		return false
	}
	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return false
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return false
	}
	if int(ip) >= int(m.Tiles[it].Header.PolyCount) {
		return false
	}
	return true
}

// GetPolyRefBase returns the polygon reference for the tile's base polygon.
func (m *NavMesh) GetPolyRefBase(tile *MeshTile) PolyRef {
	if tile == nil {
		return 0
	}

	// Find the tile index by comparing pointers
	var it uint32
	for i := 0; i < int(m.MaxTiles); i++ {
		if &m.Tiles[i] == tile {
			it = uint32(i)
			break
		}
	}

	return m.EncodePolyID(tile.Salt, it, 0)
}

// GetOffMeshConnectionPolyEndPoints gets the end points of an off-mesh connection.
func (m *NavMesh) GetOffMeshConnectionPolyEndPoints(prevRef, polyRef PolyRef) ([3]float32, [3]float32, error) {
	if polyRef == 0 {
		return [3]float32{}, [3]float32{}, ErrInvalidParam
	}

	salt, it, ip := m.DecodePolyID(polyRef)
	if int32(it) >= m.MaxTiles {
		return [3]float32{}, [3]float32{}, ErrInvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return [3]float32{}, [3]float32{}, ErrInvalidParam
	}
	tile := &m.Tiles[it]
	if int(ip) >= int(tile.Header.PolyCount) {
		return [3]float32{}, [3]float32{}, ErrInvalidParam
	}
	poly := &tile.Polys[ip]

	if poly.GetType() != PolyTypeOffMeshConnection {
		return [3]float32{}, [3]float32{}, ErrNotOffMeshConnection
	}

	idx0, idx1 := 0, 1

	for i := poly.FirstLink; i != NullLink; i = tile.Links[i].Next {
		if tile.Links[i].Edge == 0 {
			if tile.Links[i].Ref != prevRef {
				idx0 = 1
				idx1 = 0
			}
			break
		}
	}

	verts0 := tile.Verts[poly.Verts[idx0]*3 : poly.Verts[idx0]*3+3]
	startPos := Vcopy(verts0)
	verts1 := tile.Verts[poly.Verts[idx1]*3 : poly.Verts[idx1]*3+3]
	endPos := Vcopy(verts1)

	return startPos, endPos, nil
}

// GetOffMeshConnectionByRef gets the specified off-mesh connection.
func (m *NavMesh) GetOffMeshConnectionByRef(ref PolyRef) *OffMeshConnection {
	if ref == 0 {
		return nil
	}

	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return nil
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return nil
	}
	tile := &m.Tiles[it]
	if int(ip) >= int(tile.Header.PolyCount) {
		return nil
	}
	poly := &tile.Polys[ip]

	if poly.GetType() != PolyTypeOffMeshConnection {
		return nil
	}

	idx := ip - uint32(tile.Header.OffMeshBase)
	if int(idx) >= int(tile.Header.OffMeshConCount) {
		return nil
	}
	return &tile.OffMeshCons[idx]
}

// SetPolyFlags sets the user defined flags for the specified polygon.
func (m *NavMesh) SetPolyFlags(ref PolyRef, flags uint16) error {
	if ref == 0 {
		return ErrInvalidParam
	}
	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return ErrInvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return ErrInvalidParam
	}
	tile := &m.Tiles[it]
	if int(ip) >= int(tile.Header.PolyCount) {
		return ErrInvalidParam
	}
	poly := &tile.Polys[ip]
	poly.Flags = flags
	return nil
}

// GetPolyFlags gets the user defined flags for the specified polygon.
func (m *NavMesh) GetPolyFlags(ref PolyRef) (uint16, error) {
	if ref == 0 {
		return 0, ErrInvalidParam
	}
	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return 0, ErrInvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return 0, ErrInvalidParam
	}
	tile := &m.Tiles[it]
	if int(ip) >= int(tile.Header.PolyCount) {
		return 0, ErrInvalidParam
	}
	poly := &tile.Polys[ip]
	return poly.Flags, nil
}

// SetPolyArea sets the user defined area for the specified polygon.
func (m *NavMesh) SetPolyArea(ref PolyRef, area uint8) error {
	if ref == 0 {
		return ErrInvalidParam
	}
	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return ErrInvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return ErrInvalidParam
	}
	tile := &m.Tiles[it]
	if int(ip) >= int(tile.Header.PolyCount) {
		return ErrInvalidParam
	}
	poly := &tile.Polys[ip]
	poly.SetArea(area)
	return nil
}

// GetPolyArea gets the user defined area for the specified polygon.
func (m *NavMesh) GetPolyArea(ref PolyRef) (uint8, error) {
	if ref == 0 {
		return 0, ErrInvalidParam
	}
	salt, it, ip := m.DecodePolyID(ref)
	if int32(it) >= m.MaxTiles {
		return 0, ErrInvalidParam
	}
	if m.Tiles[it].Salt != salt || m.Tiles[it].Header == nil {
		return 0, ErrInvalidParam
	}
	tile := &m.Tiles[it]
	if int(ip) >= int(tile.Header.PolyCount) {
		return 0, ErrInvalidParam
	}
	poly := &tile.Polys[ip]
	return poly.GetArea(), nil
}

// GetTileStateSize returns the size of the buffer required to store the tile's state.
func (m *NavMesh) GetTileStateSize(tile *MeshTile) int {
	if tile == nil {
		return 0
	}
	headerSize := Align4(int(unsafeSizeOfTileState()))
	polyStateSize := Align4(int(unsafeSizeOfPolyState()) * int(tile.Header.PolyCount))
	return headerSize + polyStateSize
}

// TileState stores tile state for serialization.
type TileState struct {
	Magic   int32
	Version int32
	Ref     TileRef
}

// PolyState stores per-polygon state.
type PolyState struct {
	Flags uint16
	Area  uint8
}

func unsafeSizeOfTileState() uintptr { return unsafe.Sizeof(TileState{}) }
func unsafeSizeOfPolyState() uintptr { return unsafe.Sizeof(PolyState{}) }

// StoreTileState stores the non-structural state of the tile.
func (m *NavMesh) StoreTileState(tile *MeshTile, data []byte, maxDataSize int) error {
	sizeReq := m.GetTileStateSize(tile)
	if maxDataSize < sizeReq {
		return ErrBufferTooSmall
	}

	offset := 0
	// Write TileState
	tileState := TileState{
		Magic:   NavMeshStateMagic,
		Version: NavMeshStateVersion,
		Ref:     m.GetTileRef(tile),
	}
	writeTileState(data[offset:], &tileState)
	offset += Align4(int(unsafeSizeOfTileState()))

	// Write per-poly state
	for i := 0; i < int(tile.Header.PolyCount); i++ {
		p := &tile.Polys[i]
		ps := PolyState{Flags: p.Flags, Area: p.GetArea()}
		writePolyState(data[offset:], &ps)
		offset += int(unsafeSizeOfPolyState())
	}

	return nil
}

// RestoreTileState restores the state of the tile.
func (m *NavMesh) RestoreTileState(tile *MeshTile, data []byte, maxDataSize int) error {
	sizeReq := m.GetTileStateSize(tile)
	if maxDataSize < sizeReq {
		return ErrInvalidParam
	}

	offset := 0
	tileState := readTileState(data[offset:])
	offset += Align4(int(unsafeSizeOfTileState()))

	if tileState.Magic != NavMeshStateMagic {
		return ErrWrongMagic
	}
	if tileState.Version != NavMeshStateVersion {
		return ErrWrongVersion
	}
	if tileState.Ref != m.GetTileRef(tile) {
		return ErrInvalidParam
	}

	for i := 0; i < int(tile.Header.PolyCount); i++ {
		ps := readPolyState(data[offset:])
		offset += int(unsafeSizeOfPolyState())
		p := &tile.Polys[i]
		p.Flags = ps.Flags
		p.SetArea(ps.Area)
	}

	return nil
}

func writeTileState(data []byte, ts *TileState) {
	binary.LittleEndian.PutUint32(data[0:], uint32(ts.Magic))
	binary.LittleEndian.PutUint32(data[4:], uint32(ts.Version))
	binary.LittleEndian.PutUint32(data[8:], uint32(ts.Ref))
}

func writePolyState(data []byte, ps *PolyState) {
	binary.LittleEndian.PutUint16(data[0:], ps.Flags)
	data[2] = ps.Area
}

func readTileState(data []byte) TileState {
	return TileState{
		Magic:   int32(binary.LittleEndian.Uint32(data[0:])),
		Version: int32(binary.LittleEndian.Uint32(data[4:])),
		Ref:     TileRef(binary.LittleEndian.Uint32(data[8:])),
	}
}

func readPolyState(data []byte) PolyState {
	return PolyState{
		Flags: binary.LittleEndian.Uint16(data[0:]),
		Area:  data[2],
	}
}

// RemoveTile removes the specified tile.
func (m *NavMesh) RemoveTile(ref TileRef) ([]byte, error) {
	if ref == 0 {
		return nil, ErrInvalidParam
	}
	tileIndex := int32(m.DecodePolyIdTile(PolyRef(ref)))
	tileSalt := uint32(m.DecodePolyIdSalt(PolyRef(ref)))
	if tileIndex >= m.MaxTiles {
		return nil, ErrInvalidParam
	}
	tile := &m.Tiles[tileIndex]
	if tile.Salt != tileSalt {
		return nil, ErrInvalidParam
	}

	// Remove from hash lookup
	h := computeTileHash(tile.Header.X, tile.Header.Y, m.TileLutMask)
	var prev *MeshTile
	cur := m.PosLookup[h]
	for cur != nil {
		if cur == tile {
			if prev != nil {
				prev.Next = cur.Next
			} else {
				m.PosLookup[h] = cur.Next
			}
			break
		}
		prev = cur
		cur = cur.Next
	}

	// Remove connections to neighbour tiles
	const maxNeis = 32
	neis := make([]*MeshTile, maxNeis)

	nneis := m.getTilesAt(tile.Header.X, tile.Header.Y, neis, maxNeis)
	for j := 0; j < nneis; j++ {
		if neis[j] == tile {
			continue
		}
		m.unconnectLinks(neis[j], tile)
	}

	for i := int32(0); i < 8; i++ {
		nneis = m.getNeighbourTilesAt(tile.Header.X, tile.Header.Y, i, neis, maxNeis)
		for j := 0; j < nneis; j++ {
			m.unconnectLinks(neis[j], tile)
		}
	}

	var oldData []byte
	if tile.Flags&TileFreeData != 0 {
		oldData = nil
	} else {
		oldData = tile.Data
	}

	// Reset tile
	tile.Header = nil
	tile.Flags = 0
	tile.LinksFreeList = 0
	tile.Polys = nil
	tile.Verts = nil
	tile.Links = nil
	tile.DetailMeshes = nil
	tile.DetailVerts = nil
	tile.DetailTris = nil
	tile.BVTree = nil
	tile.OffMeshCons = nil

	tile.Salt = (tile.Salt + 1) & ((1 << m.SaltBits) - 1)
	if tile.Salt == 0 {
		tile.Salt++
	}

	tile.Next = m.NextFree
	m.NextFree = tile

	return oldData, nil
}

// GetTileRef returns the tile reference for the given tile.
func (m *NavMesh) GetTileRef(tile *MeshTile) TileRef {
	if tile == nil {
		return 0
	}
	it := uint32(0)
	for i := range m.Tiles {
		if &m.Tiles[i] == tile {
			it = uint32(i)
			break
		}
	}
	return TileRef(m.EncodePolyID(tile.Salt, it, 0))
}

// EncodePolyID encodes a polygon reference.
func (m *NavMesh) EncodePolyID(salt, it, ip uint32) PolyRef {
	return (PolyRef(salt) << (m.PolyBits + m.TileBits)) | (PolyRef(it) << m.PolyBits) | PolyRef(ip)
}

// DecodePolyID decodes a polygon reference.
func (m *NavMesh) DecodePolyID(ref PolyRef) (salt, it, ip uint32) {
	saltMask := PolyRef((1 << m.SaltBits) - 1)
	tileMask := PolyRef((1 << m.TileBits) - 1)
	polyMask := PolyRef((1 << m.PolyBits) - 1)
	salt = uint32((ref >> (m.PolyBits + m.TileBits)) & saltMask)
	it = uint32((ref >> m.PolyBits) & tileMask)
	ip = uint32(ref & polyMask)
	return
}

// DecodePolyIdSalt extracts salt from a polygon reference.
func (m *NavMesh) DecodePolyIdSalt(ref PolyRef) uint32 {
	saltMask := PolyRef((1 << m.SaltBits) - 1)
	return uint32((ref >> (m.PolyBits + m.TileBits)) & saltMask)
}

// DecodePolyIdTile extracts tile index from a polygon reference.
func (m *NavMesh) DecodePolyIdTile(ref PolyRef) uint32 {
	tileMask := PolyRef((1 << m.TileBits) - 1)
	return uint32((ref >> m.PolyBits) & tileMask)
}

// DecodePolyIdPoly extracts polygon index from a polygon reference.
func (m *NavMesh) DecodePolyIdPoly(ref PolyRef) uint32 {
	polyMask := PolyRef((1 << m.PolyBits) - 1)
	return uint32(ref & polyMask)
}
