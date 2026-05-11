package recast

// Constants for layer building.
const (
	maxLayersDef = 63
	maxNeisDef   = 16
)

// LayerRegion stores region data for layer building.
type LayerRegion struct {
	layers  [maxLayersDef]uint8
	neis    [maxNeisDef]uint8
	ymin    uint16
	ymax    uint16
	layerID uint8 // Layer ID
	nlayers uint8 // Layer count
	nneis   uint8 // Neighbour count
	base    uint8 // Flag indicating if the region is the base of merged regions.
}

// LayerSweepSpan represents a sweep span used in monotone region building for layers.
type LayerSweepSpan struct {
	ns  uint16 // number samples
	id  uint8  // region id
	nei uint8  // neighbour id
}

func containsArray(a []uint8, an uint8, v uint8) bool {
	n := int(an)
	for i := 0; i < n; i++ {
		if a[i] == v {
			return true
		}
	}
	return false
}

func addUniqueArray(a []uint8, an *uint8, anMax int, v uint8) bool {
	if containsArray(a, *an, v) {
		return true
	}
	if int(*an) >= anMax {
		return false
	}
	a[*an] = v
	*an++
	return true
}

func overlapRange(amin, amax, bmin, bmax uint16) bool {
	return !(amin > bmax || amax < bmin)
}

// BuildHeightfieldLayers builds layer regions from a compact heightfield.
func BuildHeightfieldLayers(ctx *Context, chf *CompactHeightfield, borderSize, walkableHeight int, lset *HeightfieldLayerSet) bool {
	w := chf.Width
	h := chf.Height

	srcReg := make([]uint8, chf.SpanCount)
	for i := range srcReg {
		srcReg[i] = 0xff
	}

	nsweeps := chf.Width
	sweeps := make([]LayerSweepSpan, nsweeps)

	prevCount := make([]int, 256)
	regID := uint8(0)

	for y := borderSize; y < h-borderSize; y++ {
		for i := range prevCount {
			prevCount[i] = 0
		}
		var sweepID uint8 = 0

		for x := borderSize; x < w-borderSize; x++ {
			c := chf.Cells[x+y*w]

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				if chf.Areas[i] == nullArea {
					continue
				}

				var sid uint8 = 0xff

				// -x
				if Con(&s, 0) != notConnected {
					ax := x + DirOffsetX(0)
					ay := y + DirOffsetZ(0)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 0)
					if chf.Areas[ai] != nullArea && srcReg[ai] != 0xff {
						sid = srcReg[ai]
					}
				}

				if sid == 0xff {
					sid = sweepID
					sweepID++
					sweeps[sid].nei = 0xff
					sweeps[sid].ns = 0
				}

				// -y
				if Con(&s, 3) != notConnected {
					ax := x + DirOffsetX(3)
					ay := y + DirOffsetZ(3)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 3)
					nr := srcReg[ai]
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

				srcReg[i] = sid
			}
		}

		for i := 0; i < int(sweepID); i++ {
			if sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == int(sweeps[i].ns) {
				sweeps[i].id = sweeps[i].nei
			} else {
				if regID == 255 {
					ctx.Log(LogError, "BuildHeightfieldLayers: Region ID overflow.")
					return false
				}
				sweeps[i].id = regID
				regID++
			}
		}

		for x := borderSize; x < w-borderSize; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				if srcReg[i] != 0xff {
					srcReg[i] = sweeps[srcReg[i]].id
				}
			}
		}
	}

	nregs := int(regID)
	regs := make([]LayerRegion, nregs)
	for i := 0; i < nregs; i++ {
		regs[i].layerID = 0xff
		regs[i].ymin = 0xffff
		regs[i].ymax = 0
	}

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]

			var lregs [maxLayersDef]uint8
			nlregs := 0

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				ri := srcReg[i]
				if ri == 0xff {
					continue
				}

				regs[ri].ymin = minU16(regs[ri].ymin, s.Y)
				regs[ri].ymax = maxU16(regs[ri].ymax, s.Y)

				if nlregs < maxLayersDef {
					lregs[nlregs] = ri
					nlregs++
				}

				for dir := 0; dir < 4; dir++ {
					if Con(&s, dir) != notConnected {
						ax := x + DirOffsetX(dir)
						ay := y + DirOffsetZ(dir)
						ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
						rai := srcReg[ai]
						if rai != 0xff && rai != ri {
							addUniqueArray(regs[ri].neis[:], &regs[ri].nneis, maxNeisDef, rai)
						}
					}
				}
			}

			for i := 0; i < nlregs-1; i++ {
				for j := i + 1; j < nlregs; j++ {
					if lregs[i] != lregs[j] {
						ri := &regs[lregs[i]]
						rj := &regs[lregs[j]]

						if !addUniqueArray(ri.layers[:], &ri.nlayers, maxLayersDef, lregs[j]) ||
							!addUniqueArray(rj.layers[:], &rj.nlayers, maxLayersDef, lregs[i]) {
							ctx.Log(LogError, "BuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing maxLayersDef.")
							return false
						}
					}
				}
			}
		}
	}

	layerID := uint8(0)

	const maxStack = 64
	stack := make([]uint8, maxStack)
	var nstack int

	for i := 0; i < nregs; i++ {
		root := &regs[i]
		if root.layerID != 0xff {
			continue
		}

		root.layerID = layerID
		root.base = 1

		nstack = 0
		stack[nstack] = uint8(i)
		nstack++

		for nstack > 0 {
			reg := &regs[stack[0]]
			nstack--
			for j := 0; j < nstack; j++ {
				stack[j] = stack[j+1]
			}

			nneis := int(reg.nneis)
			for j := 0; j < nneis; j++ {
				nei := reg.neis[j]
				regn := &regs[nei]
				if regn.layerID != 0xff {
					continue
				}
				if containsArray(root.layers[:], root.nlayers, nei) {
					continue
				}
				ymin := minU16(root.ymin, regn.ymin)
				ymax := maxU16(root.ymax, regn.ymax)
				if int(ymax-ymin) >= 255 {
					continue
				}

				if nstack < maxStack {
					stack[nstack] = nei
					nstack++

					regn.layerID = layerID
					for k := 0; k < int(regn.nlayers); k++ {
						if !addUniqueArray(root.layers[:], &root.nlayers, maxLayersDef, regn.layers[k]) {
							ctx.Log(LogError, "BuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing maxLayersDef.")
							return false
						}
					}
					root.ymin = minU16(root.ymin, regn.ymin)
					root.ymax = maxU16(root.ymax, regn.ymax)
				}
			}
		}

		layerID++
	}

	mergeHeight := uint16(walkableHeight) * 4

	for i := 0; i < nregs; i++ {
		ri := &regs[i]
		if ri.base == 0 {
			continue
		}

		newID := ri.layerID

		for {
			oldID := uint8(0xff)

			for j := 0; j < nregs; j++ {
				if i == j {
					continue
				}
				rj := &regs[j]
				if rj.base == 0 {
					continue
				}

				if !overlapRange(ri.ymin, ri.ymax+mergeHeight, rj.ymin, rj.ymax+mergeHeight) {
					continue
				}
				ymin := minU16(ri.ymin, rj.ymin)
				ymax := maxU16(ri.ymax, rj.ymax)
				if int(ymax-ymin) >= 255 {
					continue
				}

				overlap := false
				for k := 0; k < nregs; k++ {
					if regs[k].layerID != rj.layerID {
						continue
					}
					if containsArray(ri.layers[:], ri.nlayers, uint8(k)) {
						overlap = true
						break
					}
				}
				if overlap {
					continue
				}

				oldID = rj.layerID
				break
			}

			if oldID == 0xff {
				break
			}

			for j := 0; j < nregs; j++ {
				rj := &regs[j]
				if rj.layerID == oldID {
					rj.base = 0
					rj.layerID = newID
					for k := 0; k < int(rj.nlayers); k++ {
						if !addUniqueArray(ri.layers[:], &ri.nlayers, maxLayersDef, rj.layers[k]) {
							ctx.Log(LogError, "BuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing maxLayersDef.")
							return false
						}
					}
					ri.ymin = minU16(ri.ymin, rj.ymin)
					ri.ymax = maxU16(ri.ymax, rj.ymax)
				}
			}
		}
	}

	var remap [256]uint8
	for i := range remap {
		remap[i] = 0
	}

	layerID = 0
	for i := 0; i < nregs; i++ {
		remap[regs[i].layerID] = 1
	}
	for i := 0; i < 256; i++ {
		if remap[i] != 0 {
			remap[i] = layerID
			layerID++
		} else {
			remap[i] = 0xff
		}
	}
	for i := 0; i < nregs; i++ {
		regs[i].layerID = remap[regs[i].layerID]
	}

	if layerID == 0 {
		return true
	}

	lw := w - borderSize*2
	lh := h - borderSize*2

	bmin := chf.Bmin
	bmax := chf.Bmax
	bmin[0] += float32(borderSize) * chf.Cs
	bmin[2] += float32(borderSize) * chf.Cs
	bmax[0] -= float32(borderSize) * chf.Cs
	bmax[2] -= float32(borderSize) * chf.Cs

	lset.NLayers = int(layerID)
	lset.Layers = make([]HeightfieldLayer, lset.NLayers)

	for i := 0; i < lset.NLayers; i++ {
		curID := uint8(i)

		layer := &lset.Layers[i]

		gridSize := lw * lh

		layer.Heights = make([]uint8, gridSize)
		for j := range layer.Heights {
			layer.Heights[j] = 0xff
		}

		layer.Areas = make([]uint8, gridSize)

		layer.Cons = make([]uint8, gridSize)

		var hmin, hmax int
		for j := 0; j < nregs; j++ {
			if regs[j].base != 0 && regs[j].layerID == curID {
				hmin = int(regs[j].ymin)
				hmax = int(regs[j].ymax)
			}
		}

		layer.Width = lw
		layer.Height = lh
		layer.Cs = chf.Cs
		layer.Ch = chf.Ch

		layer.Bmin = bmin
		layer.Bmax = bmax
		layer.Bmin[1] = bmin[1] + float32(hmin)*chf.Ch
		layer.Bmax[1] = bmin[1] + float32(hmax)*chf.Ch
		layer.HMin = hmin
		layer.HMax = hmax

		layer.MinX = layer.Width
		layer.MaxX = 0
		layer.MinY = layer.Height
		layer.MaxY = 0

		for y := 0; y < lh; y++ {
			for x := 0; x < lw; x++ {
				cx := borderSize + x
				cy := borderSize + y
				c := chf.Cells[cx+cy*w]
				for j := int(c.Index); j < int(c.Index+c.Count); j++ {
					s := chf.Spans[j]
					if srcReg[j] == 0xff {
						continue
					}
					lid := regs[srcReg[j]].layerID
					if lid != curID {
						continue
					}

					layer.MinX = min(layer.MinX, x)
					layer.MaxX = max(layer.MaxX, x)
					layer.MinY = min(layer.MinY, y)
					layer.MaxY = max(layer.MaxY, y)

					idx := x + y*lw
					layer.Heights[idx] = uint8(int(s.Y) - hmin)
					layer.Areas[idx] = chf.Areas[j]

					var portal uint8 = 0
					var con uint8 = 0
					for dir := 0; dir < 4; dir++ {
						if Con(&s, dir) != notConnected {
							ax := cx + DirOffsetX(dir)
							ay := cy + DirOffsetZ(dir)
							ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
							alid := uint8(0xff)
							if srcReg[ai] != 0xff {
								alid = regs[srcReg[ai]].layerID
							}
							if chf.Areas[ai] != nullArea && lid != alid {
								portal |= (1 << dir)
								as := chf.Spans[ai]
								if int(as.Y) > hmin {
									layer.Heights[idx] = maxU8(layer.Heights[idx], uint8(int(as.Y)-hmin))
								}
							}
							if chf.Areas[ai] != nullArea && lid == alid {
								nx := ax - borderSize
								ny := ay - borderSize
								if nx >= 0 && ny >= 0 && nx < lw && ny < lh {
									con |= (1 << dir)
								}
							}
						}
					}

					layer.Cons[idx] = (portal << 4) | con
				}
			}
		}

		if layer.MinX > layer.MaxX {
			layer.MinX = 0
			layer.MaxX = 0
		}
		if layer.MinY > layer.MaxY {
			layer.MinY = 0
			layer.MaxY = 0
		}
	}

	return true
}

func minU16(a, b uint16) uint16 {
	if a < b {
		return a
	}
	return b
}

func maxU16(a, b uint16) uint16 {
	if a > b {
		return a
	}
	return b
}

func maxU8(a, b uint8) uint8 {
	if a > b {
		return a
	}
	return b
}
