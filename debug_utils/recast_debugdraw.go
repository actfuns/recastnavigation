package debug_utils

import (
	"math"

	"github.com/actfuns/recastnavigation/recast"
)

// DebugDrawTriMesh draws a triangle mesh with texturing.
func DebugDrawTriMesh(dd DebugDraw, verts []float32, nverts int, tris []int, normals []float32, ntris int, flags []uint8, texScale float32) {
	if dd == nil || verts == nil || tris == nil || normals == nil {
		return
	}

	unwalkable := RGBA(192, 128, 0, 255)
	dd.Texture(true)
	dd.Begin(DrawTris, 1.0)

	for i := 0; i < ntris*3; i += 3 {
		norm := normals[i : i+3]
		a := uint8(220 * (2 + norm[0] + norm[1]) / 4)
		var color uint32
		if flags != nil && flags[i/3] == 0 {
			color = LerpCol(RGBA(int(a), int(a), int(a), 255), unwalkable, 64)
		} else {
			color = RGBA(int(a), int(a), int(a), 255)
		}

		va := verts[tris[i+0]*3 : tris[i+0]*3+3]
		vb := verts[tris[i+1]*3 : tris[i+1]*3+3]
		vc := verts[tris[i+2]*3 : tris[i+2]*3+3]

		ax := 0
		if float64(norm[1]) > float64(norm[ax]) {
			ax = 1
		}
		if float64(norm[2]) > float64(norm[ax]) {
			ax = 2
		}
		ax = (1 << ax) & 3
		ay := (1 << ax) & 3

		uva := [2]float32{va[ax] * texScale, va[ay] * texScale}
		uvb := [2]float32{vb[ax] * texScale, vb[ay] * texScale}
		uvc := [2]float32{vc[ax] * texScale, vc[ay] * texScale}

		dd.VertexPosColorUV([3]float32{va[0], va[1], va[2]}, color, uva)
		dd.VertexPosColorUV([3]float32{vb[0], vb[1], vb[2]}, color, uvb)
		dd.VertexPosColorUV([3]float32{vc[0], vc[1], vc[2]}, color, uvc)
	}
	dd.End()
	dd.Texture(false)
}

// DebugDrawTriMeshSlope draws a triangle mesh colored by slope.
func DebugDrawTriMeshSlope(dd DebugDraw, verts []float32, nverts int, tris []int, normals []float32, ntris int, walkableSlopeAngle, texScale float32) {
	if dd == nil || verts == nil || tris == nil || normals == nil {
		return
	}

	walkableThr := float32(math.Cos(float64(walkableSlopeAngle / 180.0 * Pi)))
	unwalkable := RGBA(192, 128, 0, 255)

	dd.Texture(true)
	dd.Begin(DrawTris, 1.0)

	for i := 0; i < ntris*3; i += 3 {
		norm := normals[i : i+3]
		a := uint8(220 * (2 + norm[0] + norm[1]) / 4)
		var color uint32
		if norm[1] < walkableThr {
			color = LerpCol(RGBA(int(a), int(a), int(a), 255), unwalkable, 64)
		} else {
			color = RGBA(int(a), int(a), int(a), 255)
		}

		va := verts[tris[i+0]*3 : tris[i+0]*3+3]
		vb := verts[tris[i+1]*3 : tris[i+1]*3+3]
		vc := verts[tris[i+2]*3 : tris[i+2]*3+3]

		ax := 0
		if float64(norm[1]) > float64(norm[ax]) {
			ax = 1
		}
		if float64(norm[2]) > float64(norm[ax]) {
			ax = 2
		}
		ax = (1 << ax) & 3
		ay := (1 << ax) & 3

		uva := [2]float32{va[ax] * texScale, va[ay] * texScale}
		uvb := [2]float32{vb[ax] * texScale, vb[ay] * texScale}
		uvc := [2]float32{vc[ax] * texScale, vc[ay] * texScale}

		dd.VertexPosColorUV([3]float32{va[0], va[1], va[2]}, color, uva)
		dd.VertexPosColorUV([3]float32{vb[0], vb[1], vb[2]}, color, uvb)
		dd.VertexPosColorUV([3]float32{vc[0], vc[1], vc[2]}, color, uvc)
	}
	dd.End()
	dd.Texture(false)
}

// DebugDrawHeightfieldSolid draws a heightfield as solid boxes.
func DebugDrawHeightfieldSolid(dd DebugDraw, hf *recast.Heightfield) {
	if dd == nil {
		return
	}

	orig := hf.Bmin
	cs := hf.Cs
	ch := hf.Ch
	w := hf.Width
	h := hf.Height

	fcol := CalcBoxColors(RGBA(255, 255, 255, 255), RGBA(255, 255, 255, 255))

	dd.Begin(DrawQuads, 1.0)

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			fx := orig[0] + float32(x)*cs
			fz := orig[2] + float32(y)*cs
			s := hf.Spans[x+y*w]
			for s != nil {
				AppendBox(dd, fx, orig[1]+float32(s.Smin)*ch, fz, fx+cs, orig[1]+float32(s.Smax)*ch, fz+cs, fcol[:])
				s = s.Next
			}
		}
	}
	dd.End()
}

// DebugDrawHeightfieldWalkable draws a heightfield colored by walkable area.
func DebugDrawHeightfieldWalkable(dd DebugDraw, hf *recast.Heightfield) {
	if dd == nil {
		return
	}

	orig := hf.Bmin
	cs := hf.Cs
	ch := hf.Ch
	w := hf.Width
	h := hf.Height

	fcol := CalcBoxColors(RGBA(255, 255, 255, 255), RGBA(217, 217, 217, 255))

	dd.Begin(DrawQuads, 1.0)

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			fx := orig[0] + float32(x)*cs
			fz := orig[2] + float32(y)*cs
			s := hf.Spans[x+y*w]
			for s != nil {
				if s.Area == recast.WalkableArea {
					fcol[0] = RGBA(64, 128, 160, 255)
				} else if s.Area == recast.NullArea {
					fcol[0] = RGBA(64, 64, 64, 255)
				} else {
					fcol[0] = MultCol(dd.AreaToCol(s.Area), 200)
				}
				AppendBox(dd, fx, orig[1]+float32(s.Smin)*ch, fz, fx+cs, orig[1]+float32(s.Smax)*ch, fz+cs, fcol[:])
				s = s.Next
			}
		}
	}
	dd.End()
}

// DebugDrawCompactHeightfieldSolid draws a compact heightfield as solid quads.
func DebugDrawCompactHeightfieldSolid(dd DebugDraw, chf *recast.CompactHeightfield) {
	if dd == nil {
		return
	}

	cs := chf.Cs
	ch := chf.Ch

	dd.Begin(DrawQuads, 1.0)

	for y := 0; y < chf.Height; y++ {
		for x := 0; x < chf.Width; x++ {
			fx := chf.Bmin[0] + float32(x)*cs
			fz := chf.Bmin[2] + float32(y)*cs
			c := chf.Cells[x+y*chf.Width]

			for i := c.Index; i < c.Index+c.Count; i++ {
				s := chf.Spans[i]
				area := chf.Areas[i]
				var color uint32
				if area == recast.WalkableArea {
					color = RGBA(0, 192, 255, 64)
				} else if area == recast.NullArea {
					color = RGBA(0, 0, 0, 64)
				} else {
					color = dd.AreaToCol(uint32(area))
				}

				fy := chf.Bmin[1] + float32(s.Y+1)*ch
				dd.VertexXYZColor(fx, fy, fz, color)
				dd.VertexXYZColor(fx, fy, fz+cs, color)
				dd.VertexXYZColor(fx+cs, fy, fz+cs, color)
				dd.VertexXYZColor(fx+cs, fy, fz, color)
			}
		}
	}
	dd.End()
}

// DebugDrawCompactHeightfieldRegions draws a compact heightfield colored by region.
func DebugDrawCompactHeightfieldRegions(dd DebugDraw, chf *recast.CompactHeightfield) {
	if dd == nil {
		return
	}

	cs := chf.Cs
	ch := chf.Ch

	dd.Begin(DrawQuads, 1.0)

	for y := 0; y < chf.Height; y++ {
		for x := 0; x < chf.Width; x++ {
			fx := chf.Bmin[0] + float32(x)*cs
			fz := chf.Bmin[2] + float32(y)*cs
			c := chf.Cells[x+y*chf.Width]

			for i := c.Index; i < c.Index+c.Count; i++ {
				s := chf.Spans[i]
				fy := chf.Bmin[1] + float32(s.Y)*ch
				var color uint32
				if s.Reg != 0 {
					color = IntToCol(int(s.Reg), 192)
				} else {
					color = RGBA(0, 0, 0, 64)
				}

				dd.VertexXYZColor(fx, fy, fz, color)
				dd.VertexXYZColor(fx, fy, fz+cs, color)
				dd.VertexXYZColor(fx+cs, fy, fz+cs, color)
				dd.VertexXYZColor(fx+cs, fy, fz, color)
			}
		}
	}
	dd.End()
}

// DebugDrawCompactHeightfieldDistance draws a compact heightfield colored by distance.
func DebugDrawCompactHeightfieldDistance(dd DebugDraw, chf *recast.CompactHeightfield) {
	if dd == nil {
		return
	}
	if chf.Dist == nil {
		return
	}

	cs := chf.Cs
	ch := chf.Ch

	maxd := float32(chf.MaxDistance)
	if maxd < 1.0 {
		maxd = 1.0
	}
	dscale := 255.0 / maxd

	dd.Begin(DrawQuads, 1.0)

	for y := 0; y < chf.Height; y++ {
		for x := 0; x < chf.Width; x++ {
			fx := chf.Bmin[0] + float32(x)*cs
			fz := chf.Bmin[2] + float32(y)*cs
			c := chf.Cells[x+y*chf.Width]

			for i := c.Index; i < c.Index+c.Count; i++ {
				s := chf.Spans[i]
				fy := chf.Bmin[1] + float32(s.Y+1)*ch
				cd := uint8(float32(chf.Dist[i]) * dscale)
				color := RGBA(int(cd), int(cd), int(cd), 255)
				dd.VertexXYZColor(fx, fy, fz, color)
				dd.VertexXYZColor(fx, fy, fz+cs, color)
				dd.VertexXYZColor(fx+cs, fy, fz+cs, color)
				dd.VertexXYZColor(fx+cs, fy, fz, color)
			}
		}
	}
	dd.End()
}

func drawLayerPortals(dd DebugDraw, layer *recast.HeightfieldLayer) {
	cs := layer.Cs
	ch := layer.Ch
	w := layer.Width
	h := layer.Height

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
					ax := layer.Bmin[0] + (float32(x)+float32(seg[0]))*cs
					ay := layer.Bmin[1] + (float32(lh)+2)*ch
					az := layer.Bmin[2] + (float32(y)+float32(seg[1]))*cs
					bx := layer.Bmin[0] + (float32(x)+float32(seg[2]))*cs
					by := layer.Bmin[1] + (float32(lh)+2)*ch
					bz := layer.Bmin[2] + (float32(y)+float32(seg[3]))*cs
					dd.VertexXYZColor(ax, ay, az, pcol)
					dd.VertexXYZColor(bx, by, bz, pcol)
				}
			}
		}
	}
	dd.End()
}

// DebugDrawHeightfieldLayer draws a single heightfield layer.
func DebugDrawHeightfieldLayer(dd DebugDraw, layer *recast.HeightfieldLayer, idx int) {
	cs := layer.Cs
	ch := layer.Ch
	w := layer.Width
	h := layer.Height

	color := IntToCol(idx+1, 255)

	// Layer bounds
	bmin := [3]float32{
		layer.Bmin[0] + float32(layer.MinX)*cs,
		layer.Bmin[1],
		layer.Bmin[2] + float32(layer.MinY)*cs,
	}
	bmax := [3]float32{
		layer.Bmin[0] + float32(layer.MaxX+1)*cs,
		layer.Bmax[1],
		layer.Bmin[2] + float32(layer.MaxY+1)*cs,
	}
	BoxWire(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], TransCol(color, 128), 2.0)

	// Layer height
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
			if area == recast.WalkableArea {
				col = LerpCol(color, RGBA(0, 192, 255, 64), 32)
			} else if area == recast.NullArea {
				col = LerpCol(color, RGBA(0, 0, 0, 64), 32)
			} else {
				col = LerpCol(color, dd.AreaToCol(uint32(area)), 32)
			}

			fx := layer.Bmin[0] + float32(x)*cs
			fy := layer.Bmin[1] + (float32(lh)+1)*ch
			fz := layer.Bmin[2] + float32(y)*cs

			dd.VertexXYZColor(fx, fy, fz, col)
			dd.VertexXYZColor(fx, fy, fz+cs, col)
			dd.VertexXYZColor(fx+cs, fy, fz+cs, col)
			dd.VertexXYZColor(fx+cs, fy, fz, col)
		}
	}
	dd.End()

	drawLayerPortals(dd, layer)
}

// DebugDrawHeightfieldLayers draws all heightfield layers.
func DebugDrawHeightfieldLayers(dd DebugDraw, lset *recast.HeightfieldLayerSet) {
	if dd == nil {
		return
	}
	for i := 0; i < lset.NLayers; i++ {
		DebugDrawHeightfieldLayer(dd, &lset.Layers[i], i)
	}
}

// DebugDrawHeightfieldLayersRegions draws all heightfield layers colored by region.
func DebugDrawHeightfieldLayersRegions(dd DebugDraw, lset *recast.HeightfieldLayerSet) {
	if dd == nil {
		return
	}
	// Note: C++ version is not implemented either (commented out),
	// so we provide a simple wrapper calling layers.
	for i := 0; i < lset.NLayers; i++ {
		DebugDrawHeightfieldLayer(dd, &lset.Layers[i], i)
	}
}

func getContourCenter(cont *recast.Contour, orig [3]float32, cs, ch float32) [3]float32 {
	var center [3]float32
	if cont.Nverts == 0 {
		return center
	}
	for i := 0; i < cont.Nverts; i++ {
		v := cont.Verts[i*4 : i*4+4]
		center[0] += float32(v[0])
		center[1] += float32(v[1])
		center[2] += float32(v[2])
	}
	s := 1.0 / float32(cont.Nverts)
	center[0] = center[0]*s*cs + orig[0]
	center[1] = center[1]*s*ch + orig[1] + 4*ch
	center[2] = center[2]*s*cs + orig[2]
	return center
}

func findContourFromSet(cset *recast.ContourSet, reg uint16) *recast.Contour {
	for i := 0; i < cset.Nconts; i++ {
		if cset.Conts[i].Reg == reg {
			return &cset.Conts[i]
		}
	}
	return nil
}

// DebugDrawRegionConnections draws region connections as arcs.
func DebugDrawRegionConnections(dd DebugDraw, cset *recast.ContourSet, alpha float32) {
	if dd == nil {
		return
	}

	orig := cset.Bmin
	cs := cset.Cs
	ch := cset.Ch

	color := RGBA(0, 0, 0, 196)

	dd.Begin(DrawLines, 2.0)

	for i := 0; i < cset.Nconts; i++ {
		cont := &cset.Conts[i]
		pos := getContourCenter(cont, orig, cs, ch)
		for j := 0; j < cont.Nverts; j++ {
			v := cont.Verts[j*4 : j*4+4]
			if v[3] == 0 || uint16(v[3]) < cont.Reg {
				continue
			}
			cont2 := findContourFromSet(cset, uint16(v[3]))
			if cont2 != nil {
				pos2 := getContourCenter(cont2, orig, cs, ch)
				AppendArc(dd, pos[0], pos[1], pos[2], pos2[0], pos2[1], pos2[2], 0.25, 0.6, 0.6, color)
			}
		}
	}
	dd.End()

	a := uint8(alpha * 255.0)

	dd.Begin(DrawPoints, 7.0)
	for i := 0; i < cset.Nconts; i++ {
		cont := &cset.Conts[i]
		col := DarkenCol(IntToCol(int(cont.Reg), int(a)))
		pos := getContourCenter(cont, orig, cs, ch)
		dd.VertexPosColor(pos, col)
	}
	dd.End()
}

// DebugDrawRawContours draws raw contours.
func DebugDrawRawContours(dd DebugDraw, cset *recast.ContourSet, alpha float32) {
	if dd == nil {
		return
	}

	orig := cset.Bmin
	cs := cset.Cs
	ch := cset.Ch
	a := uint8(alpha * 255.0)

	dd.Begin(DrawLines, 2.0)
	for i := 0; i < cset.Nconts; i++ {
		c := cset.Conts[i]
		color := IntToCol(int(c.Reg), int(a))
		for j := 0; j < c.Nrvets; j++ {
			v := c.RVerts[j*4 : j*4+4]
			fx := orig[0] + float32(v[0])*cs
			fy := orig[1] + (float32(v[1])+1+float32(i&1))*ch
			fz := orig[2] + float32(v[2])*cs
			dd.VertexXYZColor(fx, fy, fz, color)
			if j > 0 {
				dd.VertexXYZColor(fx, fy, fz, color)
			}
		}
		// Loop last segment
		v := c.RVerts[0:4]
		fx := orig[0] + float32(v[0])*cs
		fy := orig[1] + (float32(v[1])+1+float32(i&1))*ch
		fz := orig[2] + float32(v[2])*cs
		dd.VertexXYZColor(fx, fy, fz, color)
	}
	dd.End()

	dd.Begin(DrawPoints, 2.0)
	for i := 0; i < cset.Nconts; i++ {
		c := cset.Conts[i]
		color := DarkenCol(IntToCol(int(c.Reg), int(a)))
		for j := 0; j < c.Nrvets; j++ {
			v := c.RVerts[j*4 : j*4+4]
			off := float32(0)
			colv := color
			if v[3]&recast.BorderVertex != 0 {
				colv = RGBA(255, 255, 255, int(a))
				off = ch * 2
			}
			fx := orig[0] + float32(v[0])*cs
			fy := orig[1] + (float32(v[1])+1+float32(i&1))*ch + off
			fz := orig[2] + float32(v[2])*cs
			dd.VertexXYZColor(fx, fy, fz, colv)
		}
	}
	dd.End()
}

// DebugDrawContours draws simplified contours.
func DebugDrawContours(dd DebugDraw, cset *recast.ContourSet, alpha float32) {
	if dd == nil {
		return
	}

	orig := cset.Bmin
	cs := cset.Cs
	ch := cset.Ch
	a := uint8(alpha * 255.0)

	dd.Begin(DrawLines, 2.5)
	for i := 0; i < cset.Nconts; i++ {
		c := cset.Conts[i]
		if c.Nverts == 0 {
			continue
		}
		color := IntToCol(int(c.Reg), int(a))
		bcolor := LerpCol(color, RGBA(255, 255, 255, int(a)), 128)
		for j, k := 0, c.Nverts-1; j < c.Nverts; k, j = j, j+1 {
			va := c.Verts[k*4 : k*4+4]
			vb := c.Verts[j*4 : j*4+4]
			col := color
			if va[3]&recast.AreaBorder != 0 {
				col = bcolor
			}
			fx := orig[0] + float32(va[0])*cs
			fy := orig[1] + (float32(va[1])+1+float32(i&1))*ch
			fz := orig[2] + float32(va[2])*cs
			dd.VertexXYZColor(fx, fy, fz, col)
			fx = orig[0] + float32(vb[0])*cs
			fy = orig[1] + (float32(vb[1])+1+float32(i&1))*ch
			fz = orig[2] + float32(vb[2])*cs
			dd.VertexXYZColor(fx, fy, fz, col)
		}
	}
	dd.End()

	dd.Begin(DrawPoints, 3.0)
	for i := 0; i < cset.Nconts; i++ {
		c := cset.Conts[i]
		color := DarkenCol(IntToCol(int(c.Reg), int(a)))
		for j := 0; j < c.Nverts; j++ {
			v := c.Verts[j*4 : j*4+4]
			off := float32(0)
			colv := color
			if v[3]&recast.BorderVertex != 0 {
				colv = RGBA(255, 255, 255, int(a))
				off = ch * 2
			}
			fx := orig[0] + float32(v[0])*cs
			fy := orig[1] + (float32(v[1])+1+float32(i&1))*ch + off
			fz := orig[2] + float32(v[2])*cs
			dd.VertexXYZColor(fx, fy, fz, colv)
		}
	}
	dd.End()
}

// DebugDrawPolyMesh draws a polygon mesh.
func DebugDrawPolyMesh(dd DebugDraw, mesh *recast.PolyMesh) {
	if dd == nil {
		return
	}

	nvp := mesh.Nvp
	cs := mesh.Cs
	ch := mesh.Ch
	orig := mesh.Bmin

	dd.Begin(DrawTris, 1.0)
	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		area := mesh.Areas[i]

		var color uint32
		if area == recast.WalkableArea {
			color = RGBA(0, 192, 255, 120) // Increased alpha for better visibility
		} else if area == recast.NullArea {
			color = RGBA(0, 0, 0, 120)
		} else {
			color = dd.AreaToCol(uint32(area))
		}

		for j := 2; j < nvp; j++ {
			if p[j] == recast.MeshNullIdx {
				break
			}
			vi := [3]uint16{p[0], p[j-1], p[j]}
			for k := 0; k < 3; k++ {
				v := mesh.Verts[vi[k]*3:]
				x := orig[0] + float32(v[0])*cs
				y := orig[1] + (float32(v[1])+1)*ch
				z := orig[2] + float32(v[2])*cs
				dd.VertexXYZColor(x, y, z, color)
			}
		}
	}
	dd.End()

	// Neighbour edges
	coln := RGBA(0, 48, 64, 100) // Increased alpha for edge visibility
	dd.Begin(DrawLines, 1.5)
	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		for j := 0; j < nvp; j++ {
			if p[j] == recast.MeshNullIdx {
				break
			}
			if p[nvp+j]&0x8000 != 0 {
				continue
			}
			nj := j + 1
			if nj >= nvp || p[nj] == recast.MeshNullIdx {
				nj = 0
			}
			vi := [2]uint16{p[j], p[nj]}
			for k := 0; k < 2; k++ {
				v := mesh.Verts[vi[k]*3:]
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
	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		for j := 0; j < nvp; j++ {
			if p[j] == recast.MeshNullIdx {
				break
			}
			if p[nvp+j]&0x8000 == 0 {
				continue
			}
			nj := j + 1
			if nj >= nvp || p[nj] == recast.MeshNullIdx {
				nj = 0
			}
			vi := [2]uint16{p[j], p[nj]}

			col := colb
			if p[nvp+j]&0xf != 0xf {
				col = RGBA(255, 255, 255, 128)
			}
			for k := 0; k < 2; k++ {
				v := mesh.Verts[vi[k]*3:]
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
	for i := 0; i < mesh.Nverts; i++ {
		v := mesh.Verts[i*3:]
		x := orig[0] + float32(v[0])*cs
		y := orig[1] + (float32(v[1])+1)*ch + 0.1
		z := orig[2] + float32(v[2])*cs
		dd.VertexXYZColor(x, y, z, colv)
	}
	dd.End()
}

// DebugDrawPolyMeshDetail draws a detailed polygon mesh.
func DebugDrawPolyMeshDetail(dd DebugDraw, dmesh *recast.PolyMeshDetail) {
	if dd == nil {
		return
	}

	dd.Begin(DrawTris, 1.0)
	for i := 0; i < dmesh.Nmeshes; i++ {
		m := dmesh.Meshes[i*4:]
		bverts := m[0]
		btris := m[2]
		ntris := int(m[3])
		verts := dmesh.Verts[bverts*3:]
		tris := dmesh.Tris[btris*4:]

		color := IntToCol(i, 192)
		for j := 0; j < ntris; j++ {
			t := tris[j*4:]
			dd.VertexPosColor([3]float32{verts[t[0]*3], verts[t[0]*3+1], verts[t[0]*3+2]}, color)
			dd.VertexPosColor([3]float32{verts[t[1]*3], verts[t[1]*3+1], verts[t[1]*3+2]}, color)
			dd.VertexPosColor([3]float32{verts[t[2]*3], verts[t[2]*3+1], verts[t[2]*3+2]}, color)
		}
	}
	dd.End()

	// Internal edges
	coli := RGBA(0, 0, 0, 64)
	dd.Begin(DrawLines, 1.0)
	for i := 0; i < dmesh.Nmeshes; i++ {
		m := dmesh.Meshes[i*4:]
		bverts := m[0]
		btris := m[2]
		ntris := int(m[3])
		verts := dmesh.Verts[bverts*3:]
		tris := dmesh.Tris[btris*4:]

		for j := 0; j < ntris; j++ {
			t := tris[j*4:]
			for k, kp := 0, 2; k < 3; kp, k = k, k+1 {
				ef := (t[3] >> (uint(kp) * 2)) & 0x3
				if ef == 0 {
					if t[kp] < t[k] {
						dd.VertexPosColor([3]float32{verts[t[kp]*3], verts[t[kp]*3+1], verts[t[kp]*3+2]}, coli)
						dd.VertexPosColor([3]float32{verts[t[k]*3], verts[t[k]*3+1], verts[t[k]*3+2]}, coli)
					}
				}
			}
		}
	}
	dd.End()

	// External edges
	cole := RGBA(0, 0, 0, 64)
	dd.Begin(DrawLines, 2.0)
	for i := 0; i < dmesh.Nmeshes; i++ {
		m := dmesh.Meshes[i*4:]
		bverts := m[0]
		btris := m[2]
		ntris := int(m[3])
		verts := dmesh.Verts[bverts*3:]
		tris := dmesh.Tris[btris*4:]

		for j := 0; j < ntris; j++ {
			t := tris[j*4:]
			for k, kp := 0, 2; k < 3; kp, k = k, k+1 {
				ef := (t[3] >> (uint(kp) * 2)) & 0x3
				if ef != 0 {
					dd.VertexPosColor([3]float32{verts[t[kp]*3], verts[t[kp]*3+1], verts[t[kp]*3+2]}, cole)
					dd.VertexPosColor([3]float32{verts[t[k]*3], verts[t[k]*3+1], verts[t[k]*3+2]}, cole)
				}
			}
		}
	}
	dd.End()

	// Points
	colvp := RGBA(0, 0, 0, 64)
	dd.Begin(DrawPoints, 3.0)
	for i := 0; i < dmesh.Nmeshes; i++ {
		m := dmesh.Meshes[i*4:]
		bverts := m[0]
		nverts := int(m[1])
		verts := dmesh.Verts[bverts*3:]
		for j := 0; j < nverts; j++ {
			dd.VertexPosColor([3]float32{verts[j*3], verts[j*3+1], verts[j*3+2]}, colvp)
		}
	}
	dd.End()
}
