package recast

import (
	"math"
)

func getHeightDataSeeds(ctx *Context, chf *CompactHeightfield, poly []uint16, npoly int, verts []uint16, bs int, hp *HeightPatch, queue *[]int) {
	offset := [18]int{
		0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0,
	}

	startCellX := 0
	startCellY := 0
	startSpanIndex := -1
	dmin := int(unsetHeight)
	for j := 0; j < npoly && dmin > 0; j++ {
		for k := 0; k < 9 && dmin > 0; k++ {
			ax := int(verts[poly[j]*3+0]) + offset[k*2+0]
			ay := int(verts[poly[j]*3+1])
			az := int(verts[poly[j]*3+2]) + offset[k*2+1]
			if ax < hp.xmin || ax >= hp.xmin+hp.width ||
				az < hp.ymin || az >= hp.ymin+hp.height {
				continue
			}

			c := chf.Cells[(ax+bs)+(az+bs)*chf.Width]
			for i := int(c.Index); i < int(c.Index+c.Count) && dmin > 0; i++ {
				s := chf.Spans[i]
				d := Abs(ay - int(s.Y))
				if d < dmin {
					startCellX = ax
					startCellY = az
					startSpanIndex = i
					dmin = d
				}
			}
		}
	}

	if startSpanIndex == -1 {
		return
	}

	pcx := 0
	pcy := 0
	for j := 0; j < npoly; j++ {
		pcx += int(verts[poly[j]*3+0])
		pcy += int(verts[poly[j]*3+2])
	}
	pcx /= npoly
	pcy /= npoly

	*queue = (*queue)[:0]
	*queue = append(*queue, startCellX, startCellY, startSpanIndex)

	dirs := [4]int{0, 1, 2, 3}

	for i := 0; i < hp.width*hp.height; i++ {
		hp.data[i] = 0
	}

	cx := -1
	cy := -1
	ci := -1
	for {
		if len(*queue) < 3 {
			ctx.Log(LogWarning, "Walk towards polygon center failed to reach center")
			break
		}

		ci = (*queue)[len(*queue)-1]
		*queue = (*queue)[:len(*queue)-1]
		cy = (*queue)[len(*queue)-1]
		*queue = (*queue)[:len(*queue)-1]
		cx = (*queue)[len(*queue)-1]
		*queue = (*queue)[:len(*queue)-1]

		if cx == pcx && cy == pcy {
			break
		}

		var directDir int
		if cx == pcx {
			if pcy > cy {
				directDir = 1
			} else {
				directDir = 3
			}
		} else {
			if pcx > cx {
				directDir = 2
			} else {
				directDir = 0
			}
		}

		dirs[directDir], dirs[3] = dirs[3], dirs[directDir]

		cs := chf.Spans[ci]
		for i := 0; i < 4; i++ {
			dir := dirs[i]
			if Con(&cs, dir) == notConnected {
				continue
			}

			newX := cx + DirOffsetX(dir)
			newY := cy + DirOffsetZ(dir)

			hpx := newX - hp.xmin
			hpy := newY - hp.ymin
			if hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height {
				continue
			}

			if hp.data[hpx+hpy*hp.width] != 0 {
				continue
			}

			hp.data[hpx+hpy*hp.width] = 1
			*queue = append(*queue, newX, newY,
				int(chf.Cells[(newX+bs)+(newY+bs)*chf.Width].Index)+Con(&cs, dir))
		}

		dirs[directDir], dirs[3] = dirs[3], dirs[directDir]
	}

	*queue = (*queue)[:0]
	*queue = append(*queue, cx+bs, cy+bs, ci)

	for i := 0; i < hp.width*hp.height; i++ {
		hp.data[i] = 0xffff
	}
	cs := chf.Spans[ci]
	hp.data[cx-hp.xmin+(cy-hp.ymin)*hp.width] = cs.Y
}

func push3(queue *[]int, v1, v2, v3 int) {
	*queue = append(*queue, v1, v2, v3)
}

func getHeightData(ctx *Context, chf *CompactHeightfield, poly []uint16, npoly int, verts []uint16, bs int, hp *HeightPatch, queue *[]int, region uint16) {
	*queue = (*queue)[:0]

	for i := 0; i < hp.width*hp.height; i++ {
		hp.data[i] = 0xffff
	}

	empty := true

	if region != multipleRegs {
		for hy := 0; hy < hp.height; hy++ {
			y := hp.ymin + hy + bs
			for hx := 0; hx < hp.width; hx++ {
				x := hp.xmin + hx + bs
				c := chf.Cells[x+y*chf.Width]
				for i := int(c.Index); i < int(c.Index+c.Count); i++ {
					s := chf.Spans[i]
					if s.Reg == region {
						hp.data[hx+hy*hp.width] = s.Y
						empty = false

						border := false
						for dir := 0; dir < 4; dir++ {
							if Con(&s, dir) != notConnected {
								ax := x + DirOffsetX(dir)
								ay := y + DirOffsetZ(dir)
								ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&s, dir)
								as := chf.Spans[ai]
								if as.Reg != region {
									border = true
									break
								}
							}
						}
						if border {
							push3(queue, x, y, i)
						}
						break
					}
				}
			}
		}
	}

	if empty {
		getHeightDataSeeds(ctx, chf, poly, npoly, verts, bs, hp, queue)
	}

	const retractSize = 256
	head := 0

	for head*3 < len(*queue) {
		cx := (*queue)[head*3+0]
		cy := (*queue)[head*3+1]
		ci := (*queue)[head*3+2]
		head++
		if head >= retractSize {
			head = 0
			if len(*queue) > retractSize*3 {
				copy((*queue)[0:], (*queue)[retractSize*3:])
				*queue = (*queue)[:len(*queue)-retractSize*3]
			}
		}

		cs := chf.Spans[ci]
		for dir := 0; dir < 4; dir++ {
			if Con(&cs, dir) == notConnected {
				continue
			}

			ax := cx + DirOffsetX(dir)
			ay := cy + DirOffsetZ(dir)
			hx := ax - hp.xmin - bs
			hy := ay - hp.ymin - bs

			if hx < 0 || hx >= hp.width || hy < 0 || hy >= hp.height {
				continue
			}

			if hp.data[hx+hy*hp.width] != unsetHeight {
				continue
			}

			ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&cs, dir)
			as := chf.Spans[ai]

			hp.data[hx+hy*hp.width] = as.Y

			push3(queue, ax, ay, ai)
		}
	}
}

func buildPolyDetail(ctx *Context, in []float32, nin int,
	sampleDist, sampleMaxError float32,
	heightSearchRadius int, chf *CompactHeightfield,
	hp *HeightPatch, verts []float32,
	nvertsOut *int, trisOut, edgesOut, samplesOut *[]int) {

	const maxVerts = 127
	const maxTris = 255
	const maxVertsPerEdge = 32
	edge := make([]float32, (maxVertsPerEdge+1)*3)
	hull := make([]int, maxVerts)
	nhull := 0

	nverts := nin

	for i := 0; i < nin; i++ {
		verts[i*3+0] = in[i*3+0]
		verts[i*3+1] = in[i*3+1]
		verts[i*3+2] = in[i*3+2]
	}

	tris := make([]int, 0, 512)
	edges := make([]int, 0, 64)

	cs := chf.Cs
	ics := 1.0 / cs

	minExtent := polyMinExtent(verts, nverts)

	if sampleDist > 0 {
		for i, j := 0, nin-1; i < nin; j = i {
			i++

			vj := in[j*3:]
			vi := in[i*3:]
			swapped := false

			if float32(math.Abs(float64(vj[0]-vi[0]))) < 1e-6 {
				if vj[2] > vi[2] {
					vj, vi = vi, vj
					swapped = true
				}
			} else {
				if vj[0] > vi[0] {
					vj, vi = vi, vj
					swapped = true
				}
			}

			dx := vi[0] - vj[0]
			dy := vi[1] - vj[1]
			dz := vi[2] - vj[2]
			d := float32(math.Sqrt(float64(dx*dx + dz*dz)))
			nn := 1 + int(math.Floor(float64(d/sampleDist)))
			if nn >= maxVertsPerEdge {
				nn = maxVertsPerEdge - 1
			}
			if nverts+nn >= maxVerts {
				nn = maxVerts - 1 - nverts
			}

			for k := 0; k <= nn; k++ {
				u := float32(k) / float32(nn)
				pos := edge[k*3:]
				pos[0] = vj[0] + dx*u
				pos[1] = vj[1] + dy*u
				pos[2] = vj[2] + dz*u
				pos[1] = float32(getHeight(pos[0], pos[1], pos[2], cs, ics, chf.Ch, heightSearchRadius, hp)) * chf.Ch
			}

			idx := make([]int, maxVertsPerEdge)
			idx[0] = 0
			idx[1] = nn
			nidx := 2
			for k := 0; k < nidx-1; {
				a := idx[k]
				b := idx[k+1]
				va := edge[a*3:]
				vb := edge[b*3:]
				maxd := float32(0)
				maxi := -1
				for m := a + 1; m < b; m++ {
					dev := distancePtSeg(Vcopy(edge[m*3:]), Vcopy(va), Vcopy(vb))
					if dev > maxd {
						maxd = dev
						maxi = m
					}
				}
				if maxi != -1 && maxd > sampleMaxError*sampleMaxError {
					for m := nidx; m > k; m-- {
						idx[m] = idx[m-1]
					}
					idx[k+1] = maxi
					nidx++
				} else {
					k++
				}
			}

			hull[nhull] = j
			nhull++
			if swapped {
				for k := nidx - 2; k > 0; k-- {
					verts[nverts*3+0] = edge[idx[k]*3+0]
					verts[nverts*3+1] = edge[idx[k]*3+1]
					verts[nverts*3+2] = edge[idx[k]*3+2]
					hull[nhull] = nverts
					nhull++
					nverts++
				}
			} else {
				for k := 1; k < nidx-1; k++ {
					verts[nverts*3+0] = edge[idx[k]*3+0]
					verts[nverts*3+1] = edge[idx[k]*3+1]
					verts[nverts*3+2] = edge[idx[k]*3+2]
					hull[nhull] = nverts
					nhull++
					nverts++
				}
			}
		}
	}

	if minExtent < sampleDist*2 {
		triangulateHull(nverts, verts, nhull, hull[:nhull], nin, &tris)
		setTriFlags(&tris, nhull, hull[:nhull])
		*nvertsOut = nverts
		*trisOut = tris
		*edgesOut = edges
		*samplesOut = nil
		return
	}

	triangulateHull(nverts, verts, nhull, hull[:nhull], nin, &tris)

	if len(tris) == 0 {
		ctx.Log(LogWarning, "buildPolyDetail: Could not triangulate polygon (%d verts).", nverts)
		*nvertsOut = nverts
		*trisOut = tris
		*edgesOut = edges
		*samplesOut = nil
		return
	}

	if sampleDist > 0 {
		bmin := [3]float32{in[0], in[1], in[2]}
		bmax := [3]float32{in[0], in[1], in[2]}
		for i := 1; i < nin; i++ {
			v := [3]float32{in[i*3], in[i*3+1], in[i*3+2]}
			bmin = Vmin(bmin, v)
			bmax = Vmax(bmax, v)
		}
		x0 := int(math.Floor(float64(bmin[0] / sampleDist)))
		x1 := int(math.Ceil(float64(bmax[0] / sampleDist)))
		z0 := int(math.Floor(float64(bmin[2] / sampleDist)))
		z1 := int(math.Ceil(float64(bmax[2] / sampleDist)))

		samples := make([]int, 0, 512)
		for z := z0; z < z1; z++ {
			for x := x0; x < x1; x++ {
				pt := [3]float32{
					float32(x) * sampleDist,
					(bmax[1] + bmin[1]) * 0.5,
					float32(z) * sampleDist,
				}
				if distToPoly(nin, in, pt) > -sampleDist/2 {
					continue
				}
				samples = append(samples, x,
					int(getHeight(pt[0], pt[1], pt[2], cs, ics, chf.Ch, heightSearchRadius, hp)),
					z, 0)
			}
		}

		nsamples := len(samples) / 4
		for iter := 0; iter < nsamples; iter++ {
			if nverts >= maxVerts {
				break
			}

			bestpt := [3]float32{0, 0, 0}
			bestd := float32(0)
			besti := -1

			for i := 0; i < nsamples; i++ {
				s := samples[i*4:]
				if s[3] != 0 {
					continue
				}
				pt := [3]float32{
					float32(s[0])*sampleDist + getJitterX(i)*cs*0.1,
					float32(s[1]) * chf.Ch,
					float32(s[2])*sampleDist + getJitterY(i)*cs*0.1,
				}
				if len(tris) == 0 {
					continue
				}
				d := distToTriMesh(pt, verts, tris, len(tris)/4)
				if d < 0 {
					continue
				}
				if d > bestd {
					bestd = d
					besti = i
					bestpt = pt
				}
			}

			if bestd <= sampleMaxError || besti == -1 {
				break
			}

			samples[besti*4+3] = 1

			verts[nverts*3+0] = bestpt[0]
			verts[nverts*3+1] = bestpt[1]
			verts[nverts*3+2] = bestpt[2]
			nverts++

			edges = edges[:0]
			tris = tris[:0]
			delaunayHull(nverts, verts, nhull, hull[:nhull], &tris, &edges)
		}

		*samplesOut = samples
	}

	ntris := len(tris) / 4
	if ntris > maxTris {
		tris = tris[:maxTris*4]
		ctx.Log(LogError, "BuildPolyMeshDetail: Shrinking triangle count from %d to max %d.", ntris, maxTris)
	}

	setTriFlags(&tris, nhull, hull[:nhull])

	*nvertsOut = nverts
	*trisOut = tris
	*edgesOut = edges
}

// BuildPolyMeshDetail builds the detail mesh for a polygon mesh.
func BuildPolyMeshDetail(ctx *Context, mesh *PolyMesh, chf *CompactHeightfield, sampleDist, sampleMaxError float32, dmesh *PolyMeshDetail) bool {
	if mesh.Nverts == 0 || mesh.Npolys == 0 {
		return true
	}

	defer ctx.ScopedTimer(TimerBuildPolyMeshDetail)()

	nvp := mesh.Nvp
	cs := mesh.Cs
	ch := mesh.Ch
	borderSize := mesh.BorderSize
	heightSearchRadius := max(1, int(math.Ceil(float64(mesh.MaxEdgeError))))

	edges := make([]int, 0, 64)
	tris := make([]int, 0, 512)
	arr := make([]int, 0, 512)
	samples := make([]int, 0, 512)
	verts := make([]float32, 256*3)

	bounds := make([]int, mesh.Npolys*4)
	nPolyVerts := 0
	maxhw := 0
	maxhh := 0

	poly := make([]float32, nvp*3)

	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]
		xmin := chf.Width
		xmax := 0
		ymin := chf.Height
		ymax := 0

		for j := 0; j < nvp; j++ {
			if p[j] == meshNullIdx {
				break
			}
			v := mesh.Verts[p[j]*3:]
			xmin = min(xmin, int(v[0]))
			xmax = max(xmax, int(v[0]))
			ymin = min(ymin, int(v[2]))
			ymax = max(ymax, int(v[2]))
			nPolyVerts++
		}

		xmin = max(0, xmin-1)
		xmax = min(chf.Width, xmax+1)
		ymin = max(0, ymin-1)
		ymax = min(chf.Height, ymax+1)

		bounds[i*4+0] = xmin
		bounds[i*4+1] = xmax
		bounds[i*4+2] = ymin
		bounds[i*4+3] = ymax

		if xmin >= xmax || ymin >= ymax {
			continue
		}
		maxhw = max(maxhw, xmax-xmin)
		maxhh = max(maxhh, ymax-ymin)
	}

	hp := &HeightPatch{
		data:   make([]uint16, maxhw*maxhh),
		width:  maxhw,
		height: maxhh,
	}

	dmesh.Nmeshes = mesh.Npolys
	dmesh.Meshes = make([]uint32, dmesh.Nmeshes*4)
	dmesh.Nverts = 0
	dmesh.Ntris = 0
	dmesh.Verts = make([]float32, 0)
	dmesh.Tris = make([]uint8, 0)

	for i := 0; i < mesh.Npolys; i++ {
		p := mesh.Polys[i*nvp*2:]

		npoly := 0
		for j := 0; j < nvp; j++ {
			if p[j] == meshNullIdx {
				break
			}
			v := mesh.Verts[p[j]*3:]
			poly[j*3+0] = float32(v[0]) * cs
			poly[j*3+1] = float32(v[1]) * ch
			poly[j*3+2] = float32(v[2]) * cs
			npoly++
		}

		hp.xmin = bounds[i*4+0]
		hp.ymin = bounds[i*4+2]
		hp.width = bounds[i*4+1] - bounds[i*4+0]
		hp.height = bounds[i*4+3] - bounds[i*4+2]

		getHeightData(ctx, chf, p, npoly, mesh.Verts, borderSize, hp, &arr, mesh.Regs[i])

		var nverts int
		buildPolyDetail(ctx, poly, npoly,
			sampleDist, sampleMaxError,
			heightSearchRadius, chf, hp,
			verts, &nverts, &tris, &edges, &samples)

		for j := 0; j < nverts; j++ {
			verts[j*3+0] += mesh.Bmin[0]
			verts[j*3+1] += mesh.Bmin[1] + chf.Ch
			verts[j*3+2] += mesh.Bmin[2]
		}
		for j := 0; j < npoly; j++ {
			poly[j*3+0] += mesh.Bmin[0]
			poly[j*3+1] += mesh.Bmin[1]
			poly[j*3+2] += mesh.Bmin[2]
		}

		ntris := len(tris) / 4

		dmesh.Meshes[i*4+0] = uint32(dmesh.Nverts)
		dmesh.Meshes[i*4+1] = uint32(nverts)
		dmesh.Meshes[i*4+2] = uint32(dmesh.Ntris)
		dmesh.Meshes[i*4+3] = uint32(ntris)

		for j := 0; j < nverts; j++ {
			dmesh.Verts = append(dmesh.Verts, verts[j*3+0], verts[j*3+1], verts[j*3+2])
		}
		dmesh.Nverts += nverts

		for j := 0; j < ntris; j++ {
			t := tris[j*4:]
			dmesh.Tris = append(dmesh.Tris, uint8(t[0]), uint8(t[1]), uint8(t[2]), uint8(t[3]))
		}
		dmesh.Ntris += ntris
	}

	return true
}

// MergePolyMeshDetails merges multiple detail meshes into one.
func MergePolyMeshDetails(ctx *Context, meshes []*PolyMeshDetail, mesh *PolyMeshDetail) bool {
	defer ctx.ScopedTimer(TimerMergePolyMeshDetail)()

	maxVerts := 0
	maxTris := 0
	maxMeshes := 0

	for i := 0; i < len(meshes); i++ {
		if meshes[i] == nil {
			continue
		}
		maxVerts += meshes[i].Nverts
		maxTris += meshes[i].Ntris
		maxMeshes += meshes[i].Nmeshes
	}

	mesh.Nmeshes = 0
	mesh.Meshes = make([]uint32, maxMeshes*4)
	mesh.Ntris = 0
	mesh.Tris = make([]uint8, maxTris*4)
	mesh.Nverts = 0
	mesh.Verts = make([]float32, maxVerts*3)

	for i := 0; i < len(meshes); i++ {
		dm := meshes[i]
		if dm == nil {
			continue
		}
		for j := 0; j < dm.Nmeshes; j++ {
			dst := mesh.Meshes[mesh.Nmeshes*4:]
			src := dm.Meshes[j*4:]
			dst[0] = uint32(mesh.Nverts) + src[0]
			dst[1] = src[1]
			dst[2] = uint32(mesh.Ntris) + src[2]
			dst[3] = src[3]
			mesh.Nmeshes++
		}

		for k := 0; k < dm.Nverts; k++ {
			mesh.Verts[mesh.Nverts*3+0] = dm.Verts[k*3+0]
			mesh.Verts[mesh.Nverts*3+1] = dm.Verts[k*3+1]
			mesh.Verts[mesh.Nverts*3+2] = dm.Verts[k*3+2]
			mesh.Nverts++
		}
		for k := 0; k < dm.Ntris; k++ {
			mesh.Tris[mesh.Ntris*4+0] = dm.Tris[k*4+0]
			mesh.Tris[mesh.Ntris*4+1] = dm.Tris[k*4+1]
			mesh.Tris[mesh.Ntris*4+2] = dm.Tris[k*4+2]
			mesh.Tris[mesh.Ntris*4+3] = dm.Tris[k*4+3]
			mesh.Ntris++
		}
	}

	return true
}
