package debug_utils

import (
	"github.com/actfuns/recastnavigation/detour"
	tc "github.com/actfuns/recastnavigation/detour_tile_cache"
)

// DrawNavMeshFlags defines flags for nav mesh drawing.
type DrawNavMeshFlags int

const (
	DrawNavMeshOffMeshCons DrawNavMeshFlags = 0x01
	DrawNavMeshClosedList  DrawNavMeshFlags = 0x02
	DrawNavMeshColorTiles  DrawNavMeshFlags = 0x04
)

func distancePtLine2d(pt, p, q [3]float32) float32 {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d != 0 {
		t /= d
	}
	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]
	return dx*dx + dz*dz
}

func drawPolyBoundaries(dd DebugDraw, tile *detour.MeshTile, col uint32, linew float32, inner bool) {
	const thr = 0.01 * 0.01

	dd.Begin(DrawLines, linew)

	for i := 0; i < int(tile.Header.PolyCount); i++ {
		p := &tile.Polys[i]

		if p.GetType() == detour.PolyTypeOffMeshConnection {
			continue
		}

		pd := tile.DetailMeshes[i]

		for j := 0; j < int(p.VertCount); j++ {
			c := col
			if inner {
				if p.Neis[j] == 0 {
					continue
				}
				if p.Neis[j]&detour.ExtLink != 0 {
					con := false
					for k := p.FirstLink; k != detour.NullLink; k = tile.Links[k].Next {
						if tile.Links[k].Edge == uint8(j) {
							con = true
							break
						}
					}
					if con {
						c = RGBA(255, 255, 255, 48)
					} else {
						c = RGBA(0, 0, 0, 48)
					}
				} else {
					c = RGBA(0, 48, 64, 32)
				}
			} else {
				if p.Neis[j] != 0 {
					continue
				}
			}

			v0 := tile.Verts[p.Verts[j]*3 : p.Verts[j]*3+3]
			v1 := tile.Verts[p.Verts[(j+1)%int(p.VertCount)]*3 : p.Verts[(j+1)%int(p.VertCount)]*3+3]

			for k := 0; k < int(pd.TriCount); k++ {
				t := tile.DetailTris[(pd.TriBase+uint32(k))*4:]
				tv := make([][]float32, 3)
				for m := 0; m < 3; m++ {
					if t[m] < p.VertCount {
						tv[m] = tile.Verts[p.Verts[t[m]]*3 : p.Verts[t[m]]*3+3]
					} else {
						tv[m] = tile.DetailVerts[(pd.VertBase+uint32(t[m])-uint32(p.VertCount))*3 : (pd.VertBase+uint32(t[m])-uint32(p.VertCount))*3+3]
					}
				}
				for m, n := 0, 2; m < 3; n, m = m, m+1 {
					if detour.GetDetailTriEdgeFlags(t[3], n)&detour.DetailEdgeBoundary == 0 {
						continue
					}
					if distancePtLine2d(
						[3]float32{tv[n][0], tv[n][1], tv[n][2]},
						[3]float32{v0[0], v0[1], v0[2]},
						[3]float32{v1[0], v1[1], v1[2]},
					) < thr &&
						distancePtLine2d(
							[3]float32{tv[m][0], tv[m][1], tv[m][2]},
							[3]float32{v0[0], v0[1], v0[2]},
							[3]float32{v1[0], v1[1], v1[2]},
						) < thr {
						dd.VertexPosColor([3]float32{tv[n][0], tv[n][1], tv[n][2]}, c)
						dd.VertexPosColor([3]float32{tv[m][0], tv[m][1], tv[m][2]}, c)
					}
				}
			}
		}
	}
	dd.End()
}

func drawMeshTile(dd DebugDraw, mesh *detour.NavMesh, query *detour.NavMeshQuery, tile *detour.MeshTile, flags byte) {
	base := mesh.GetPolyRefBase(tile)

	tileNum := mesh.DecodePolyIdTile(base)
	tileColor := IntToCol(int(tileNum), 128)

	dd.DepthMask(false)

	dd.Begin(DrawTris, 1.0)
	for i := 0; i < int(tile.Header.PolyCount); i++ {
		p := &tile.Polys[i]
		if p.GetType() == detour.PolyTypeOffMeshConnection {
			continue
		}

		pd := tile.DetailMeshes[i]

		var col uint32
		if query != nil && query.IsInClosedList(base|detour.PolyRef(i)) {
			col = RGBA(255, 196, 0, 64)
		} else {
			if flags&byte(DrawNavMeshColorTiles) != 0 {
				col = tileColor
			} else {
				col = TransCol(dd.AreaToCol(uint32(p.GetArea())), 64)
			}
		}

		for j := 0; j < int(pd.TriCount); j++ {
			t := tile.DetailTris[(pd.TriBase+uint32(j))*4:]
			for k := 0; k < 3; k++ {
				if t[k] < p.VertCount {
					dd.VertexPosColor([3]float32{tile.Verts[p.Verts[t[k]]*3], tile.Verts[p.Verts[t[k]]*3+1], tile.Verts[p.Verts[t[k]]*3+2]}, col)
				} else {
					dd.VertexPosColor(
							[3]float32{
								tile.DetailVerts[(pd.VertBase+uint32(t[k])-uint32(p.VertCount))*3],
								tile.DetailVerts[(pd.VertBase+uint32(t[k])-uint32(p.VertCount))*3+1],
								tile.DetailVerts[(pd.VertBase+uint32(t[k])-uint32(p.VertCount))*3+2],
							}, col)
				}
			}
		}
	}
	dd.End()

	// Draw inter poly boundaries
	drawPolyBoundaries(dd, tile, RGBA(0, 48, 64, 32), 1.5, true)

	// Draw outer poly boundaries
	drawPolyBoundaries(dd, tile, RGBA(0, 48, 64, 220), 2.5, false)

	if flags&byte(DrawNavMeshOffMeshCons) != 0 {
		dd.Begin(DrawLines, 2.0)
		for i := 0; i < int(tile.Header.PolyCount); i++ {
			p := &tile.Polys[i]
			if p.GetType() != detour.PolyTypeOffMeshConnection {
				continue
			}

			var col, col2 uint32
			if query != nil && query.IsInClosedList(base|detour.PolyRef(i)) {
				col = RGBA(255, 196, 0, 220)
			} else {
				col = DarkenCol(TransCol(dd.AreaToCol(uint32(p.GetArea())), 220))
			}

			con := &tile.OffMeshCons[i-int(tile.Header.OffMeshBase)]
			va := tile.Verts[p.Verts[0]*3 : p.Verts[0]*3+3]
			vb := tile.Verts[p.Verts[1]*3 : p.Verts[1]*3+3]

			startSet := false
			endSet := false
			for k := p.FirstLink; k != detour.NullLink; k = tile.Links[k].Next {
				if tile.Links[k].Edge == 0 {
					startSet = true
				}
				if tile.Links[k].Edge == 1 {
					endSet = true
				}
			}

			dd.VertexPosColor([3]float32{va[0], va[1], va[2]}, col)
			dd.VertexXYZColor(con.Pos[0], con.Pos[1], con.Pos[2], col)
			col2 = col
			if !startSet {
				col2 = RGBA(220, 32, 16, 196)
			}
			AppendCircle(dd, con.Pos[0], con.Pos[1]+0.1, con.Pos[2], con.Rad, col2)

			dd.VertexPosColor([3]float32{vb[0], vb[1], vb[2]}, col)
			dd.VertexXYZColor(con.Pos[3], con.Pos[4], con.Pos[5], col)
			col2 = col
			if !endSet {
				col2 = RGBA(220, 32, 16, 196)
			}
			AppendCircle(dd, con.Pos[3], con.Pos[4]+0.1, con.Pos[5], con.Rad, col2)

			dd.VertexXYZColor(con.Pos[0], con.Pos[1], con.Pos[2], RGBA(0, 48, 64, 196))
			dd.VertexXYZColor(con.Pos[0], con.Pos[1]+0.2, con.Pos[2], RGBA(0, 48, 64, 196))

			dd.VertexXYZColor(con.Pos[3], con.Pos[4], con.Pos[5], RGBA(0, 48, 64, 196))
			dd.VertexXYZColor(con.Pos[3], con.Pos[4]+0.2, con.Pos[5], RGBA(0, 48, 64, 196))

			arcBias := float32(0.6)
			if con.Flags&1 == 0 {
				arcBias = 0
			}
			AppendArc(dd, con.Pos[0], con.Pos[1], con.Pos[2], con.Pos[3], con.Pos[4], con.Pos[5], 0.25, arcBias, 0.6, col)
		}
		dd.End()
	}

	vcol := RGBA(0, 0, 0, 196)
	dd.Begin(DrawPoints, 3.0)
	for i := 0; i < int(tile.Header.VertCount); i++ {
		v := tile.Verts[i*3 : i*3+3]
		dd.VertexPosColor([3]float32{v[0], v[1], v[2]}, vcol)
	}
	dd.End()

	dd.DepthMask(true)
}

// DebugDrawNavMesh draws the navigation mesh.
func DebugDrawNavMesh(dd DebugDraw, mesh *detour.NavMesh, flags byte) {
	if dd == nil {
		return
	}
	for i := 0; i < mesh.GetMaxTiles(); i++ {
		tile := mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		drawMeshTile(dd, mesh, nil, tile, flags)
	}
}

// DebugDrawNavMeshWithClosedList draws the navigation mesh, optionally highlighting the closed list.
func DebugDrawNavMeshWithClosedList(dd DebugDraw, mesh *detour.NavMesh, query *detour.NavMeshQuery, flags byte) {
	if dd == nil {
		return
	}

	var q *detour.NavMeshQuery
	if flags&byte(DrawNavMeshClosedList) != 0 {
		q = query
	}

	for i := 0; i < mesh.GetMaxTiles(); i++ {
		tile := mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		drawMeshTile(dd, mesh, q, tile, flags)
	}
}

// DebugDrawNavMeshNodes draws pathfinding nodes.
func DebugDrawNavMeshNodes(dd DebugDraw, query *detour.NavMeshQuery) {
	if dd == nil {
		return
	}

	pool := query.GetNodePool()
	if pool == nil {
		return
	}

	off := float32(0.5)
	dd.Begin(DrawPoints, 4.0)
	for i := 0; i < pool.GetHashSize(); i++ {
		for j := pool.GetFirst(i); j != detour.NullIdx; j = pool.GetNext(int(j)) {
			node := pool.GetNodeAtIdx(uint32(j) + 1)
			if node == nil {
				continue
			}
			dd.VertexXYZColor(node.Pos[0], node.Pos[1]+off, node.Pos[2], RGBA(255, 192, 0, 255))
		}
	}
	dd.End()

	dd.Begin(DrawLines, 2.0)
	for i := 0; i < pool.GetHashSize(); i++ {
		for j := pool.GetFirst(i); j != detour.NullIdx; j = pool.GetNext(int(j)) {
			node := pool.GetNodeAtIdx(uint32(j) + 1)
			if node == nil {
				continue
			}
			if node.Pidx == 0 {
				continue
			}
			parent := pool.GetNodeAtIdx(node.Pidx)
			if parent == nil {
				continue
			}
			dd.VertexXYZColor(node.Pos[0], node.Pos[1]+off, node.Pos[2], RGBA(255, 192, 0, 128))
			dd.VertexXYZColor(parent.Pos[0], parent.Pos[1]+off, parent.Pos[2], RGBA(255, 192, 0, 128))
		}
	}
	dd.End()
}

func drawMeshTileBVTree(dd DebugDraw, tile *detour.MeshTile) {
	cs := 1.0 / tile.Header.BVQuantFactor
	dd.Begin(DrawLines, 1.0)
	for i := 0; i < int(tile.Header.BVNodeCount); i++ {
		n := tile.BVTree[i]
		if n.I < 0 {
			continue
		}
		AppendBoxWire(dd,
			tile.Header.Bmin[0]+float32(n.Bmin[0])*cs,
			tile.Header.Bmin[1]+float32(n.Bmin[1])*cs,
			tile.Header.Bmin[2]+float32(n.Bmin[2])*cs,
			tile.Header.Bmin[0]+float32(n.Bmax[0])*cs,
			tile.Header.Bmin[1]+float32(n.Bmax[1])*cs,
			tile.Header.Bmin[2]+float32(n.Bmax[2])*cs,
			RGBA(255, 255, 255, 128))
	}
	dd.End()
}

// DebugDrawNavMeshBVTree draws the bounding volume tree of a navigation mesh.
func DebugDrawNavMeshBVTree(dd DebugDraw, mesh *detour.NavMesh) {
	if dd == nil {
		return
	}
	for i := 0; i < mesh.GetMaxTiles(); i++ {
		tile := mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		drawMeshTileBVTree(dd, tile)
	}
}

func drawMeshTilePortal(dd DebugDraw, tile *detour.MeshTile) {
	const padx = 0.04
	pady := tile.Header.WalkableClimb

	dd.Begin(DrawLines, 2.0)

	for side := 0; side < 8; side++ {
		m := detour.ExtLink | uint16(side)

		for i := 0; i < int(tile.Header.PolyCount); i++ {
			poly := &tile.Polys[i]
			nv := int(poly.VertCount)

			for j := 0; j < nv; j++ {
				if poly.Neis[j] != m {
					continue
				}

				va := tile.Verts[poly.Verts[j]*3 : poly.Verts[j]*3+3]
				vb := tile.Verts[poly.Verts[(j+1)%nv]*3 : poly.Verts[(j+1)%nv]*3+3]

				if side == 0 || side == 4 {
					var col uint32
					if side == 0 {
						col = RGBA(128, 0, 0, 128)
					} else {
						col = RGBA(128, 0, 128, 128)
					}

					x := va[0]
					if side == 0 {
						x -= padx
					} else {
						x += padx
					}

					dd.VertexXYZColor(x, va[1]-pady, va[2], col)
					dd.VertexXYZColor(x, va[1]+pady, va[2], col)
					dd.VertexXYZColor(x, va[1]+pady, va[2], col)
					dd.VertexXYZColor(x, vb[1]+pady, vb[2], col)
					dd.VertexXYZColor(x, vb[1]+pady, vb[2], col)
					dd.VertexXYZColor(x, vb[1]-pady, vb[2], col)
					dd.VertexXYZColor(x, vb[1]-pady, vb[2], col)
					dd.VertexXYZColor(x, va[1]-pady, va[2], col)
				} else if side == 2 || side == 6 {
					var col uint32
					if side == 2 {
						col = RGBA(0, 128, 0, 128)
					} else {
						col = RGBA(0, 128, 128, 128)
					}

					z := va[2]
					if side == 2 {
						z -= padx
					} else {
						z += padx
					}

					dd.VertexXYZColor(va[0], va[1]-pady, z, col)
					dd.VertexXYZColor(va[0], va[1]+pady, z, col)
					dd.VertexXYZColor(va[0], va[1]+pady, z, col)
					dd.VertexXYZColor(vb[0], vb[1]+pady, z, col)
					dd.VertexXYZColor(vb[0], vb[1]+pady, z, col)
					dd.VertexXYZColor(vb[0], vb[1]-pady, z, col)
					dd.VertexXYZColor(vb[0], vb[1]-pady, z, col)
					dd.VertexXYZColor(va[0], va[1]-pady, z, col)
				}
			}
		}
	}
	dd.End()
}

// DebugDrawNavMeshPortals draws portal connections between tiles.
func DebugDrawNavMeshPortals(dd DebugDraw, mesh *detour.NavMesh) {
	if dd == nil {
		return
	}
	for i := 0; i < mesh.GetMaxTiles(); i++ {
		tile := mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		drawMeshTilePortal(dd, tile)
	}
}

// DebugDrawNavMeshPolysWithFlags draws polygons with specific flags.
func DebugDrawNavMeshPolysWithFlags(dd DebugDraw, mesh *detour.NavMesh, polyFlags uint16, col uint32) {
	if dd == nil {
		return
	}
	for i := 0; i < mesh.GetMaxTiles(); i++ {
		tile := mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		base := mesh.GetPolyRefBase(tile)
		for j := 0; j < int(tile.Header.PolyCount); j++ {
			p := &tile.Polys[j]
			if p.Flags&polyFlags == 0 {
				continue
			}
			DebugDrawNavMeshPoly(dd, mesh, base|detour.PolyRef(j), col)
		}
	}
}

// DebugDrawNavMeshPoly draws a single polygon of a navigation mesh.
func DebugDrawNavMeshPoly(dd DebugDraw, mesh *detour.NavMesh, ref detour.PolyRef, col uint32) {
	if dd == nil {
		return
	}

	tile, poly, _ := mesh.GetTileAndPolyByRef(ref)
	if tile == nil || poly == nil {
		return
	}

	dd.DepthMask(false)

	c := TransCol(col, 64)
	// Find index of poly within tile
	ip := -1
	for idx := 0; idx < int(tile.Header.PolyCount); idx++ {
		if &tile.Polys[idx] == poly {
			ip = idx
			break
		}
	}
	if ip < 0 {
		return
	}

	if poly.GetType() == detour.PolyTypeOffMeshConnection {
		con := &tile.OffMeshCons[ip-int(tile.Header.OffMeshBase)]

		dd.Begin(DrawLines, 2.0)
		arcBias := float32(0.6)
		if con.Flags&1 == 0 {
			arcBias = 0
		}
		AppendArc(dd, con.Pos[0], con.Pos[1], con.Pos[2], con.Pos[3], con.Pos[4], con.Pos[5], 0.25, arcBias, 0.6, c)
		dd.End()
	} else {
		pd := tile.DetailMeshes[ip]

		dd.Begin(DrawTris, 1.0)
		for i := 0; i < int(pd.TriCount); i++ {
			t := tile.DetailTris[(pd.TriBase+uint32(i))*4:]
			for j := 0; j < 3; j++ {
				if t[j] < poly.VertCount {
					dd.VertexPosColor(
								[3]float32{
									tile.Verts[poly.Verts[t[j]]*3],
									tile.Verts[poly.Verts[t[j]]*3+1],
									tile.Verts[poly.Verts[t[j]]*3+2],
								}, c)
				} else {
					dd.VertexPosColor(
								[3]float32{
									tile.DetailVerts[(pd.VertBase+uint32(t[j])-uint32(poly.VertCount))*3],
									tile.DetailVerts[(pd.VertBase+uint32(t[j])-uint32(poly.VertCount))*3+1],
									tile.DetailVerts[(pd.VertBase+uint32(t[j])-uint32(poly.VertCount))*3+2],
								}, c)
				}
			}
		}
		dd.End()
	}

	dd.DepthMask(true)
}

func debugDrawTileCachePortals(dd DebugDraw, layer *tc.TileCacheLayer, cs, ch float32) {
	w := int(layer.Header.Width)
	h := int(layer.Header.Height)
	bmin := layer.Header.Bmin

	pcol := RGBA(255, 255, 255, 255)
	segs := [16]int{0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0}

	dd.Begin(DrawLines, 2.0)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			idx := x + y*w
			lh := int(layer.Heights[idx])
			if lh == 0xff {
				continue
			}
			for dir := 0; dir < 4; dir++ {
				if layer.Cons[idx]&(1<<(dir+4)) != 0 {
					seg := segs[dir*4 : dir*4+4]
					ax := bmin[0] + (float32(x)+float32(seg[0]))*cs
					ay := bmin[1] + (float32(lh)+2)*ch
					az := bmin[2] + (float32(y)+float32(seg[1]))*cs
					bx := bmin[0] + (float32(x)+float32(seg[2]))*cs
					by := bmin[1] + (float32(lh)+2)*ch
					bz := bmin[2] + (float32(y)+float32(seg[3]))*cs
					dd.VertexXYZColor(ax, ay, az, pcol)
					dd.VertexXYZColor(bx, by, bz, pcol)
				}
			}
		}
	}
	dd.End()
}

// DebugDrawTileCacheLayerAreas draws tile cache layer areas.
func DebugDrawTileCacheLayerAreas(dd DebugDraw, layer *tc.TileCacheLayer, cs, ch float32) {
	w := int(layer.Header.Width)
	h := int(layer.Header.Height)
	bmin := layer.Header.Bmin
	bmax := layer.Header.Bmax
	idx := int(layer.Header.Tlayer)

	color := IntToCol(idx+1, 255)

	lbmin := [3]float32{
		bmin[0] + float32(layer.Header.Minx)*cs,
		bmin[1],
		bmin[2] + float32(layer.Header.Miny)*cs,
	}
	lbmax := [3]float32{
		bmin[0] + float32(layer.Header.Maxx+1)*cs,
		bmax[1],
		bmin[2] + float32(layer.Header.Maxy+1)*cs,
	}
	BoxWire(dd, lbmin[0], lbmin[1], lbmin[2], lbmax[0], lbmax[1], lbmax[2], TransCol(color, 128), 2.0)

	dd.Begin(DrawQuads, 1.0)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			lidx := x + y*w
			lh := int(layer.Heights[lidx])
			if lh == 0xff {
				continue
			}

			area := layer.Areas[lidx]
			var col uint32
			if area == 63 {
				col = LerpCol(color, RGBA(0, 192, 255, 64), 32)
			} else if area == 0 {
				col = LerpCol(color, RGBA(0, 0, 0, 64), 32)
			} else {
				col = LerpCol(color, dd.AreaToCol(uint32(area)), 32)
			}

			fx := bmin[0] + float32(x)*cs
			fy := bmin[1] + (float32(lh)+1)*ch
			fz := bmin[2] + float32(y)*cs

			dd.VertexXYZColor(fx, fy, fz, col)
			dd.VertexXYZColor(fx, fy, fz+cs, col)
			dd.VertexXYZColor(fx+cs, fy, fz+cs, col)
			dd.VertexXYZColor(fx+cs, fy, fz, col)
		}
	}
	dd.End()

	debugDrawTileCachePortals(dd, layer, cs, ch)
}

// DebugDrawTileCacheLayerRegions draws tile cache layer regions.
func DebugDrawTileCacheLayerRegions(dd DebugDraw, layer *tc.TileCacheLayer, cs, ch float32) {
	w := int(layer.Header.Width)
	h := int(layer.Header.Height)
	bmin := layer.Header.Bmin
	bmax := layer.Header.Bmax
	idx := int(layer.Header.Tlayer)

	color := IntToCol(idx+1, 255)

	lbmin := [3]float32{
		bmin[0] + float32(layer.Header.Minx)*cs,
		bmin[1],
		bmin[2] + float32(layer.Header.Miny)*cs,
	}
	lbmax := [3]float32{
		bmin[0] + float32(layer.Header.Maxx+1)*cs,
		bmax[1],
		bmin[2] + float32(layer.Header.Maxy+1)*cs,
	}
	BoxWire(dd, lbmin[0], lbmin[1], lbmin[2], lbmax[0], lbmax[1], lbmax[2], TransCol(color, 128), 2.0)

	dd.Begin(DrawQuads, 1.0)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			lidx := x + y*w
			lh := int(layer.Heights[lidx])
			if lh == 0xff {
				continue
			}
			reg := layer.Regs[lidx]
			col := LerpCol(color, IntToCol(int(reg), 255), 192)

			fx := bmin[0] + float32(x)*cs
			fy := bmin[1] + (float32(lh)+1)*ch
			fz := bmin[2] + float32(y)*cs

			dd.VertexXYZColor(fx, fy, fz, col)
			dd.VertexXYZColor(fx, fy, fz+cs, col)
			dd.VertexXYZColor(fx+cs, fy, fz+cs, col)
			dd.VertexXYZColor(fx+cs, fy, fz, col)
		}
	}
	dd.End()

	debugDrawTileCachePortals(dd, layer, cs, ch)
}

// DebugDrawTileCacheContours draws tile cache contours.
func DebugDrawTileCacheContours(dd DebugDraw, lcset *tc.TileCacheContourSet, orig [3]float32, cs, ch float32) {
	if dd == nil {
		return
	}

	a := uint8(255)
	offs := [8]int{-1, 0, 0, 1, 1, 0, 0, -1}

	dd.Begin(DrawLines, 2.0)
	for i := 0; i < lcset.NConts; i++ {
		c := lcset.Conts[i]
		col := IntToCol(i, int(a))

		for j := 0; j < c.NVerts; j++ {
			k := (j + 1) % c.NVerts
			va := c.Verts[j*4:]
			vb := c.Verts[k*4:]

			ax := orig[0] + float32(va[0])*cs
			ay := orig[1] + (float32(va[1])+1+float32(i&1))*ch
			az := orig[2] + float32(va[2])*cs
			bx := orig[0] + float32(vb[0])*cs
			by := orig[1] + (float32(vb[1])+1+float32(i&1))*ch
			bz := orig[2] + float32(vb[2])*cs

			color := col
			if va[3]&0xf != 0xf {
				color = RGBA(255, 255, 255, 128)
				d := int(va[3] & 0xf)
				cx := (ax + bx) * 0.5
				cy := (ay + by) * 0.5
				cz := (az + bz) * 0.5
				dx := cx + float32(offs[d*2])*2*cs
				dz := cz + float32(offs[d*2+1])*2*cs
				dd.VertexXYZColor(cx, cy, cz, RGBA(255, 0, 0, 255))
				dd.VertexXYZColor(dx, cy, dz, RGBA(255, 0, 0, 255))
			}

			AppendArrow(dd, ax, ay, az, bx, by, bz, 0.0, cs*0.5, color)
		}
	}
	dd.End()

	dd.Begin(DrawPoints, 4.0)
	for i := 0; i < lcset.NConts; i++ {
		c := lcset.Conts[i]
		for j := 0; j < c.NVerts; j++ {
			va := c.Verts[j*4:]
			col := DarkenCol(IntToCol(i, int(a)))
			if va[3]&0x80 != 0 {
				col = RGBA(255, 0, 0, 255)
			}
			fx := orig[0] + float32(va[0])*cs
			fy := orig[1] + (float32(va[1])+1+float32(i&1))*ch
			fz := orig[2] + float32(va[2])*cs
			dd.VertexXYZColor(fx, fy, fz, col)
		}
	}
	dd.End()
}

// DebugDrawTileCachePolyMesh draws tile cache poly mesh.
func DebugDrawTileCachePolyMesh(dd DebugDraw, lmesh *tc.TileCachePolyMesh, orig [3]float32, cs, ch float32) {
	if dd == nil {
		return
	}

	nvp := lmesh.Nvp
	offs := [8]int{-1, 0, 0, 1, 1, 0, 0, -1}

	dd.Begin(DrawTris, 1.0)
	for i := 0; i < lmesh.NPolys; i++ {
		p := lmesh.Polys[i*nvp*2:]
		area := lmesh.Areas[i]

		var col uint32
		if area == tc.TileCacheWalkableArea {
			col = RGBA(0, 192, 255, 64)
		} else if area == tc.TileCacheNullArea {
			col = RGBA(0, 0, 0, 64)
		} else {
			col = dd.AreaToCol(uint32(area))
		}

		var vi [3]uint16
		for j := 2; j < nvp; j++ {
			if p[j] == tc.TileCacheNullIdx {
				break
			}
			vi[0] = p[0]
			vi[1] = p[j-1]
			vi[2] = p[j]
			for k := 0; k < 3; k++ {
				v := lmesh.Verts[vi[k]*3:]
				x := orig[0] + float32(v[0])*cs
				y := orig[1] + (float32(v[1])+1)*ch
				z := orig[2] + float32(v[2])*cs
				dd.VertexXYZColor(x, y, z, col)
			}
		}
	}
	dd.End()

	// Neighbour edges
	coln := RGBA(0, 48, 64, 32)
	dd.Begin(DrawLines, 1.5)
	for i := 0; i < lmesh.NPolys; i++ {
		p := lmesh.Polys[i*nvp*2:]
		for j := 0; j < nvp; j++ {
			if p[j] == tc.TileCacheNullIdx {
				break
			}
			if p[nvp+j]&0x8000 != 0 {
				continue
			}
			nj := j + 1
			if nj >= nvp || p[nj] == tc.TileCacheNullIdx {
				nj = 0
			}
			vi := [2]uint16{p[j], p[nj]}
			for k := 0; k < 2; k++ {
				v := lmesh.Verts[vi[k]*3:]
				x := orig[0] + float32(v[0])*cs
				y := orig[1] + (float32(v[1])+1)*ch + 0.1
				z := orig[2] + float32(v[2])*cs
				dd.VertexXYZColor(x, y, z, coln)
			}
		}
	}
	dd.End()

	// Boundary edges
	colb := RGBA(0, 48, 64, 220)
	dd.Begin(DrawLines, 2.5)
	for i := 0; i < lmesh.NPolys; i++ {
		p := lmesh.Polys[i*nvp*2:]
		for j := 0; j < nvp; j++ {
			if p[j] == tc.TileCacheNullIdx {
				break
			}
			if p[nvp+j]&0x8000 == 0 {
				continue
			}
			nj := j + 1
			if nj >= nvp || p[nj] == tc.TileCacheNullIdx {
				nj = 0
			}
			vi := [2]uint16{p[j], p[nj]}

			col := colb
			if p[nvp+j]&0xf != 0xf {
				va := lmesh.Verts[vi[0]*3:]
				vb := lmesh.Verts[vi[1]*3:]
				ax := orig[0] + float32(va[0])*cs
				ay := orig[1] + (float32(va[1])+1+float32(i&1))*ch
				az := orig[2] + float32(va[2])*cs
				bx := orig[0] + float32(vb[0])*cs
				by := orig[1] + (float32(vb[1])+1+float32(i&1))*ch
				bz := orig[2] + float32(vb[2])*cs
				cx := (ax + bx) * 0.5
				cy := (ay + by) * 0.5
				cz := (az + bz) * 0.5
				d := int(p[nvp+j] & 0xf)
				dx := cx + float32(offs[d*2])*2*cs
				dz := cz + float32(offs[d*2+1])*2*cs
				dd.VertexXYZColor(cx, cy, cz, RGBA(255, 0, 0, 255))
				dd.VertexXYZColor(dx, cy, dz, RGBA(255, 0, 0, 255))
				col = RGBA(255, 255, 255, 128)
			}

			for k := 0; k < 2; k++ {
				v := lmesh.Verts[vi[k]*3:]
				x := orig[0] + float32(v[0])*cs
				y := orig[1] + (float32(v[1])+1)*ch + 0.1
				z := orig[2] + float32(v[2])*cs
				dd.VertexXYZColor(x, y, z, col)
			}
		}
	}
	dd.End()

	// Vertices
	colv := RGBA(0, 0, 0, 220)
	dd.Begin(DrawPoints, 3.0)
	for i := 0; i < lmesh.NVerts; i++ {
		v := lmesh.Verts[i*3:]
		x := orig[0] + float32(v[0])*cs
		y := orig[1] + (float32(v[1])+1)*ch + 0.1
		z := orig[2] + float32(v[2])*cs
		dd.VertexXYZColor(x, y, z, colv)
	}
	dd.End()
}
