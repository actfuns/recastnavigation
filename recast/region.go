package recast

import "context"

// LevelStackEntry is an entry in the level stack for watershed partitioning.
type LevelStackEntry struct {
	x, y, index int
}

// DirtyEntry tracks changes in the region table during expansion.
type DirtyEntry struct {
	index     int
	region    uint16
	distance2 uint16
}

// Region represents a connected region in the compact heightfield.
type Region struct {
	spanCount        int
	id               uint16
	areaType         uint8
	remap            bool
	visited          bool
	overlap          bool
	connectsToBorder bool
	ymin, ymax       uint16
	connections      []int
	floors           []int
}

// SweepSpan is used for monotone region building.
type SweepSpan struct {
	rid uint16
	id  uint16
	ns  uint16
	nei uint16
}

const nullNei uint16 = 0xffff

func calculateDistanceField(chf *CompactHeightfield, src []uint16) uint16 {
	w := chf.Width
	h := chf.Height

	// Init distance and points.
	for i := 0; i < chf.SpanCount; i++ {
		src[i] = 0xffff
	}

	// Mark boundary cells.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				area := chf.Areas[i]

				nc := 0
				for dir := 0; dir < 4; dir++ {
					if Con(&s, dir) != notConnected {
						ax := x + DirOffsetX(dir)
						ay := y + DirOffsetZ(dir)
						ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
						if area == chf.Areas[ai] {
							nc++
						}
					}
				}
				if nc != 4 {
					src[i] = 0
				}
			}
		}
	}

	// Pass 1
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]

				if Con(&s, 0) != notConnected {
					// (-1,0)
					ax := x + DirOffsetX(0)
					ay := y + DirOffsetZ(0)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 0)
					as := chf.Spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}

					// (-1,-1)
					if Con(&as, 3) != notConnected {
						aax := ax + DirOffsetX(3)
						aay := ay + DirOffsetZ(3)
						aai := int(chf.Cells[aax+aay*w].Index) + Con(&as, 3)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
				if Con(&s, 3) != notConnected {
					// (0,-1)
					ax := x + DirOffsetX(3)
					ay := y + DirOffsetZ(3)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 3)
					as := chf.Spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}

					// (1,-1)
					if Con(&as, 2) != notConnected {
						aax := ax + DirOffsetX(2)
						aay := ay + DirOffsetZ(2)
						aai := int(chf.Cells[aax+aay*w].Index) + Con(&as, 2)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
			}
		}
	}

	// Pass 2
	for y := h - 1; y >= 0; y-- {
		for x := w - 1; x >= 0; x-- {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]

				if Con(&s, 2) != notConnected {
					// (1,0)
					ax := x + DirOffsetX(2)
					ay := y + DirOffsetZ(2)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 2)
					as := chf.Spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}

					// (1,1)
					if Con(&as, 1) != notConnected {
						aax := ax + DirOffsetX(1)
						aay := ay + DirOffsetZ(1)
						aai := int(chf.Cells[aax+aay*w].Index) + Con(&as, 1)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
				if Con(&s, 1) != notConnected {
					// (0,1)
					ax := x + DirOffsetX(1)
					ay := y + DirOffsetZ(1)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 1)
					as := chf.Spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}

					// (-1,1)
					if Con(&as, 0) != notConnected {
						aax := ax + DirOffsetX(0)
						aay := ay + DirOffsetZ(0)
						aai := int(chf.Cells[aax+aay*w].Index) + Con(&as, 0)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
			}
		}
	}

	maxDist := uint16(0)
	for i := 0; i < chf.SpanCount; i++ {
		if src[i] > maxDist {
			maxDist = src[i]
		}
	}

	return maxDist
}

func boxBlur(chf *CompactHeightfield, thr int, src, dst []uint16) {
	w := chf.Width
	h := chf.Height

	thr *= 2

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				cd := src[i]
				if cd <= uint16(thr) {
					dst[i] = cd
					continue
				}

				d := int(cd)
				for dir := 0; dir < 4; dir++ {
					if Con(&s, dir) != notConnected {
						ax := x + DirOffsetX(dir)
						ay := y + DirOffsetZ(dir)
						ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
						d += int(src[ai])

						as := chf.Spans[ai]
						dir2 := (dir + 1) & 0x3
						if Con(&as, dir2) != notConnected {
							ax2 := ax + DirOffsetX(dir2)
							ay2 := ay + DirOffsetZ(dir2)
							ai2 := int(chf.Cells[ax2+ay2*w].Index) + Con(&as, dir2)
							d += int(src[ai2])
						} else {
							d += int(cd)
						}
					} else {
						d += int(cd) * 2
					}
				}
				dst[i] = uint16((d + 5) / 9)
			}
		}
	}
}

func floodRegion(x, y, i int, level, r uint16, chf *CompactHeightfield, srcReg, srcDist []uint16, stack *[]LevelStackEntry) bool {
	w := chf.Width
	area := chf.Areas[i]

	// Flood fill mark region.
	*stack = (*stack)[:0]
	*stack = append(*stack, LevelStackEntry{x, y, i})
	srcReg[i] = r
	srcDist[i] = 0

	lev := level
	if level >= 2 {
		lev = level - 2
	} else {
		lev = 0
	}
	count := 0

	for len(*stack) > 0 {
		back := (*stack)[len(*stack)-1]
		cx := back.x
		cy := back.y
		ci := back.index
		*stack = (*stack)[:len(*stack)-1]

		cs := chf.Spans[ci]

		// Check if any of the neighbours already have a valid region set.
		ar := uint16(0)
		for dir := 0; dir < 4; dir++ {
			// 8 connected
			if Con(&cs, dir) != notConnected {
				ax := cx + DirOffsetX(dir)
				ay := cy + DirOffsetZ(dir)
				ai := int(chf.Cells[ax+ay*w].Index) + Con(&cs, dir)
				if chf.Areas[ai] != area {
					continue
				}
				nr := srcReg[ai]
				if nr&borderReg != 0 { // Do not take borders into account.
					continue
				}
				if nr != 0 && nr != r {
					ar = nr
					break
				}

				as := chf.Spans[ai]

				dir2 := (dir + 1) & 0x3
				if Con(&as, dir2) != notConnected {
					ax2 := ax + DirOffsetX(dir2)
					ay2 := ay + DirOffsetZ(dir2)
					ai2 := int(chf.Cells[ax2+ay2*w].Index) + Con(&as, dir2)
					if chf.Areas[ai2] != area {
						continue
					}
					nr2 := srcReg[ai2]
					if nr2 != 0 && nr2 != r {
						ar = nr2
						break
					}
				}
			}
		}
		if ar != 0 {
			srcReg[ci] = 0
			continue
		}

		count++

		// Expand neighbours.
		for dir := 0; dir < 4; dir++ {
			if Con(&cs, dir) != notConnected {
				ax := cx + DirOffsetX(dir)
				ay := cy + DirOffsetZ(dir)
				ai := int(chf.Cells[ax+ay*w].Index) + Con(&cs, dir)
				if chf.Areas[ai] != area {
					continue
				}
				if chf.Dist[ai] >= lev && srcReg[ai] == 0 {
					srcReg[ai] = r
					srcDist[ai] = 0
					*stack = append(*stack, LevelStackEntry{ax, ay, ai})
				}
			}
		}
	}

	return count > 0
}

func expandRegions(maxIter int, level uint16, chf *CompactHeightfield, srcReg, srcDist []uint16, stack *[]LevelStackEntry, fillStack bool) {
	w := chf.Width
	h := chf.Height

	if fillStack {
		// Find cells revealed by the raised level.
		*stack = (*stack)[:0]
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				c := chf.Cells[x+y*w]
				for i := int(c.Index); i < int(c.Index+c.Count); i++ {
					if chf.Dist[i] >= level && srcReg[i] == 0 && chf.Areas[i] != nullArea {
						*stack = append(*stack, LevelStackEntry{x, y, i})
					}
				}
			}
		}
	} else {
		// use cells in the input stack
		// mark all cells which already have a region
		for j := 0; j < len(*stack); j++ {
			i := (*stack)[j].index
			if srcReg[i] != 0 {
				(*stack)[j].index = -1
			}
		}
	}

	dirtyEntries := make([]DirtyEntry, 0)
	iter := 0
	for len(*stack) > 0 {
		failed := 0
		dirtyEntries = dirtyEntries[:0]

		for j := 0; j < len(*stack); j++ {
			x := (*stack)[j].x
			y := (*stack)[j].y
			i := (*stack)[j].index
			if i < 0 {
				failed++
				continue
			}

			r := srcReg[i]
			d2 := uint16(0xffff)
			area := chf.Areas[i]
			s := chf.Spans[i]
			for dir := 0; dir < 4; dir++ {
				if Con(&s, dir) == notConnected {
					continue
				}
				ax := x + DirOffsetX(dir)
				ay := y + DirOffsetZ(dir)
				ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
				if chf.Areas[ai] != area {
					continue
				}
				if srcReg[ai] > 0 && (srcReg[ai]&borderReg) == 0 {
					if int(srcDist[ai])+2 < int(d2) {
						r = srcReg[ai]
						d2 = srcDist[ai] + 2
					}
				}
			}
			if r != 0 {
				(*stack)[j].index = -1 // mark as used
				dirtyEntries = append(dirtyEntries, DirtyEntry{i, r, d2})
			} else {
				failed++
			}
		}

		// Copy entries that differ between src and dst to keep them in sync.
		for i := 0; i < len(dirtyEntries); i++ {
			idx := dirtyEntries[i].index
			srcReg[idx] = dirtyEntries[i].region
			srcDist[idx] = dirtyEntries[i].distance2
		}

		if failed == len(*stack) {
			break
		}

		if level > 0 {
			iter++
			if iter >= maxIter {
				break
			}
		}
	}
}

func sortCellsByLevel(startLevel uint16, chf *CompactHeightfield, srcReg []uint16, nbStacks int, stacks [][]LevelStackEntry, loglevelsPerStack uint16) {
	w := chf.Width
	h := chf.Height
	startLevel = startLevel >> loglevelsPerStack

	for j := 0; j < nbStacks; j++ {
		stacks[j] = stacks[j][:0]
	}

	// put all cells in the level range into the appropriate stacks
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				if chf.Areas[i] == nullArea || srcReg[i] != 0 {
					continue
				}

				level := chf.Dist[i] >> loglevelsPerStack
				sId := int(startLevel) - int(level)
				if sId >= nbStacks {
					continue
				}
				if sId < 0 {
					sId = 0
				}

				stacks[sId] = append(stacks[sId], LevelStackEntry{x, y, i})
			}
		}
	}
}

func appendStacks(srcStack []LevelStackEntry, dstStack *[]LevelStackEntry, srcReg []uint16) {
	for j := 0; j < len(srcStack); j++ {
		i := srcStack[j].index
		if i < 0 || srcReg[i] != 0 {
			continue
		}
		*dstStack = append(*dstStack, srcStack[j])
	}
}

func removeAdjacentNeighbours(reg *Region) {
	// Remove adjacent duplicates.
	for i := 0; i < len(reg.connections) && len(reg.connections) > 1; {
		ni := (i + 1) % len(reg.connections)
		if reg.connections[i] == reg.connections[ni] {
			// Remove duplicate
			for j := i; j < len(reg.connections)-1; j++ {
				reg.connections[j] = reg.connections[j+1]
			}
			reg.connections = reg.connections[:len(reg.connections)-1]
		} else {
			i++
		}
	}
}

func replaceNeighbour(reg *Region, oldId, newId uint16) {
	neiChanged := false
	for i := 0; i < len(reg.connections); i++ {
		if reg.connections[i] == int(oldId) {
			reg.connections[i] = int(newId)
			neiChanged = true
		}
	}
	for i := 0; i < len(reg.floors); i++ {
		if reg.floors[i] == int(oldId) {
			reg.floors[i] = int(newId)
		}
	}
	if neiChanged {
		removeAdjacentNeighbours(reg)
	}
}

func canMergeWithRegion(rega, regb *Region) bool {
	if rega.areaType != regb.areaType {
		return false
	}
	n := 0
	for i := 0; i < len(rega.connections); i++ {
		if rega.connections[i] == int(regb.id) {
			n++
		}
	}
	if n > 1 {
		return false
	}
	for i := 0; i < len(rega.floors); i++ {
		if rega.floors[i] == int(regb.id) {
			return false
		}
	}
	return true
}

func addUniqueFloorRegion(reg *Region, n int) {
	for i := 0; i < len(reg.floors); i++ {
		if reg.floors[i] == n {
			return
		}
	}
	reg.floors = append(reg.floors, n)
}

func mergeRegions(rega, regb *Region) bool {
	aid := rega.id
	bid := regb.id

	// Duplicate current neighbourhood.
	acon := make([]int, len(rega.connections))
	copy(acon, rega.connections)
	bcon := regb.connections

	// Find insertion point on A.
	insa := -1
	for i := 0; i < len(acon); i++ {
		if acon[i] == int(bid) {
			insa = i
			break
		}
	}
	if insa == -1 {
		return false
	}

	// Find insertion point on B.
	insb := -1
	for i := 0; i < len(bcon); i++ {
		if bcon[i] == int(aid) {
			insb = i
			break
		}
	}
	if insb == -1 {
		return false
	}

	// Merge neighbours.
	rega.connections = rega.connections[:0]
	for i := 0; i < len(acon)-1; i++ {
		rega.connections = append(rega.connections, acon[(insa+1+i)%len(acon)])
	}

	for i := 0; i < len(bcon)-1; i++ {
		rega.connections = append(rega.connections, bcon[(insb+1+i)%len(bcon)])
	}

	removeAdjacentNeighbours(rega)

	for j := 0; j < len(regb.floors); j++ {
		addUniqueFloorRegion(rega, regb.floors[j])
	}
	rega.spanCount += regb.spanCount
	regb.spanCount = 0
	regb.connections = regb.connections[:0]

	return true
}

func isRegionConnectedToBorder(reg *Region) bool {
	// Region is connected to border if one of the neighbours is null id.
	for i := 0; i < len(reg.connections); i++ {
		if reg.connections[i] == 0 {
			return true
		}
	}
	return false
}

func isSolidEdge(chf *CompactHeightfield, srcReg []uint16, x, y, i, dir int) bool {
	s := chf.Spans[i]
	r := uint16(0)
	if Con(&s, dir) != notConnected {
		ax := x + DirOffsetX(dir)
		ay := y + DirOffsetZ(dir)
		ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&s, dir)
		r = srcReg[ai]
	}
	if r == srcReg[i] {
		return false
	}
	return true
}

func walkContourRegion(x, y, i, dir int, chf *CompactHeightfield, srcReg []uint16, cont *[]int) {
	startDir := dir
	starti := i

	ss := chf.Spans[i]
	curReg := uint16(0)
	if Con(&ss, dir) != notConnected {
		ax := x + DirOffsetX(dir)
		ay := y + DirOffsetZ(dir)
		ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&ss, dir)
		curReg = srcReg[ai]
	}
	*cont = append(*cont, int(curReg))

	iter := 0
	for {
		iter++
		if iter >= 40000 {
			break
		}

		s := chf.Spans[i]

		if isSolidEdge(chf, srcReg, x, y, i, dir) {
			// Choose the edge corner
			r := uint16(0)
			if Con(&s, dir) != notConnected {
				ax := x + DirOffsetX(dir)
				ay := y + DirOffsetZ(dir)
				ai := int(chf.Cells[ax+ay*chf.Width].Index) + Con(&s, dir)
				r = srcReg[ai]
			}
			if r != curReg {
				curReg = r
				*cont = append(*cont, int(curReg))
			}

			dir = (dir + 1) & 0x3 // Rotate CW
		} else {
			ni := -1
			nx := x + DirOffsetX(dir)
			ny := y + DirOffsetZ(dir)
			if Con(&s, dir) != notConnected {
				nc := chf.Cells[nx+ny*chf.Width]
				ni = int(nc.Index) + Con(&s, dir)
			}
			if ni == -1 {
				// Should not happen.
				return
			}
			x = nx
			y = ny
			i = ni
			dir = (dir + 3) & 0x3 // Rotate CCW
		}

		if starti == i && startDir == dir {
			break
		}
	}

	// Remove adjacent duplicates.
	if len(*cont) > 1 {
		for j := 0; j < len(*cont); {
			nj := (j + 1) % len(*cont)
			if (*cont)[j] == (*cont)[nj] {
				for k := j; k < len(*cont)-1; k++ {
					(*cont)[k] = (*cont)[k+1]
				}
				*cont = (*cont)[:len(*cont)-1]
			} else {
				j++
			}
		}
	}
}

func addUniqueConnection(reg *Region, n int) {
	for i := 0; i < len(reg.connections); i++ {
		if reg.connections[i] == n {
			return
		}
	}
	reg.connections = append(reg.connections, n)
}

func mergeAndFilterRegions(minRegionArea, mergeRegionSize int, maxRegionId *uint16, chf *CompactHeightfield, srcReg []uint16, overlaps *[]int) {
	w := chf.Width
	h := chf.Height

	nreg := int(*maxRegionId) + 1
	regions := make([]Region, nreg)
	for i := 0; i < nreg; i++ {
		regions[i] = Region{id: uint16(i)}
	}

	// Find edge of a region and find connections around the contour.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				r := srcReg[i]
				if r == 0 || int(r) >= nreg {
					continue
				}

				reg := &regions[r]
				reg.spanCount++

				// Update floors.
				for j := int(c.Index); j < int(c.Index+c.Count); j++ {
					if i == j {
						continue
					}
					floorId := srcReg[j]
					if floorId == 0 || int(floorId) >= nreg {
						continue
					}
					if floorId == r {
						reg.overlap = true
					}
					addUniqueFloorRegion(reg, int(floorId))
				}

				// Have found contour
				if len(reg.connections) > 0 {
					continue
				}

				reg.areaType = chf.Areas[i]

				// Check if this cell is next to a border.
				ndir := -1
				for dir := 0; dir < 4; dir++ {
					if isSolidEdge(chf, srcReg, x, y, i, dir) {
						ndir = dir
						break
					}
				}

				if ndir != -1 {
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContourRegion(x, y, i, ndir, chf, srcReg, &reg.connections)
				}
			}
		}
	}

	// Remove too small regions.
	stack := make([]int, 0, 32)
	trace := make([]int, 0, 32)
	for i := 0; i < nreg; i++ {
		reg := &regions[i]
		if reg.id == 0 || (reg.id&borderReg) != 0 {
			continue
		}
		if reg.spanCount == 0 {
			continue
		}
		if reg.visited {
			continue
		}

		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		connectsToBorder := false
		spanCount := 0
		stack = stack[:0]
		trace = trace[:0]

		reg.visited = true
		stack = append(stack, i)

		for len(stack) > 0 {
			// Pop
			ri := stack[len(stack)-1]
			stack = stack[:len(stack)-1]

			creg := &regions[ri]

			spanCount += creg.spanCount
			trace = append(trace, ri)

			for j := 0; j < len(creg.connections); j++ {
				if creg.connections[j]&int(borderReg) != 0 {
					connectsToBorder = true
					continue
				}
				neireg := &regions[creg.connections[j]]
				if neireg.visited {
					continue
				}
				if neireg.id == 0 || (neireg.id&borderReg) != 0 {
					continue
				}
				// Visit
				stack = append(stack, int(neireg.id))
				neireg.visited = true
			}
		}

		// If the accumulated regions size is too small, remove it.
		if spanCount < minRegionArea && !connectsToBorder {
			// Kill all visited regions.
			for j := 0; j < len(trace); j++ {
				regions[trace[j]].spanCount = 0
				regions[trace[j]].id = 0
			}
		}
	}

	// Merge too small regions to neighbour regions.
	mergeCount := 0
	for {
		mergeCount = 0
		for i := 0; i < nreg; i++ {
			reg := &regions[i]
			if reg.id == 0 || (reg.id&borderReg) != 0 {
				continue
			}
			if reg.overlap {
				continue
			}
			if reg.spanCount == 0 {
				continue
			}

			// Check to see if the region should be merged.
			if reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg) {
				continue
			}

			// Find smallest neighbour region that connects to this one.
			smallest := 0x7fffffff
			mergeId := reg.id
			for j := 0; j < len(reg.connections); j++ {
				if reg.connections[j]&int(borderReg) != 0 {
					continue
				}
				mreg := &regions[reg.connections[j]]
				if mreg.id == 0 || (mreg.id&borderReg) != 0 || mreg.overlap {
					continue
				}
				if mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg) {
					smallest = mreg.spanCount
					mergeId = mreg.id
				}
			}
			// Found new id.
			if mergeId != reg.id {
				oldId := reg.id
				target := &regions[mergeId]

				// Merge neighbours.
				if mergeRegions(target, reg) {
					// Fixup regions pointing to current region.
					for j := 0; j < nreg; j++ {
						if regions[j].id == 0 || (regions[j].id&borderReg) != 0 {
							continue
						}
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if regions[j].id == oldId {
							regions[j].id = mergeId
						}
						// Replace the current region with the new one if the
						// current regions is neighbour.
						replaceNeighbour(&regions[j], oldId, mergeId)
					}
					mergeCount++
				}
			}
		}
		if mergeCount == 0 {
			break
		}
	}

	// Compress region Ids.
	for i := 0; i < nreg; i++ {
		regions[i].remap = false
		if regions[i].id == 0 {
			continue
		}
		if regions[i].id&borderReg != 0 {
			continue
		}
		regions[i].remap = true
	}

	regIdGen := uint16(0)
	for i := 0; i < nreg; i++ {
		if !regions[i].remap {
			continue
		}
		oldId := regions[i].id
		regIdGen++
		newId := regIdGen
		for j := i; j < nreg; j++ {
			if regions[j].id == oldId {
				regions[j].id = newId
				regions[j].remap = false
			}
		}
	}
	*maxRegionId = regIdGen

	// Remap regions.
	for i := 0; i < chf.SpanCount; i++ {
		if srcReg[i]&borderReg == 0 {
			srcReg[i] = regions[srcReg[i]].id
		}
	}

	// Return regions that we found to be overlapping.
	for i := 0; i < nreg; i++ {
		if regions[i].overlap {
			*overlaps = append(*overlaps, int(regions[i].id))
		}
	}
}

func mergeAndFilterLayerRegions(minRegionArea int, maxRegionId *uint16, chf *CompactHeightfield, srcReg []uint16) {
	w := chf.Width
	h := chf.Height

	nreg := int(*maxRegionId) + 1
	regions := make([]Region, nreg)
	for i := 0; i < nreg; i++ {
		regions[i] = Region{id: uint16(i)}
	}

	// Find region neighbours and overlapping regions.
	lregs := make([]int, 0, 32)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := chf.Cells[x+y*w]

			lregs = lregs[:0]

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				area := chf.Areas[i]
				ri := srcReg[i]
				if ri == 0 || int(ri) >= nreg {
					continue
				}
				reg := &regions[ri]

				reg.spanCount++
				reg.areaType = area

				if s.Y < reg.ymin {
					reg.ymin = s.Y
				}
				if s.Y > reg.ymax {
					reg.ymax = s.Y
				}

				// Collect all region layers.
				lregs = append(lregs, int(ri))

				// Update neighbours
				for dir := 0; dir < 4; dir++ {
					if Con(&s, dir) != notConnected {
						ax := x + DirOffsetX(dir)
						ay := y + DirOffsetZ(dir)
						ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, dir)
						rai := srcReg[ai]
						if rai > 0 && int(rai) < nreg && rai != ri {
							addUniqueConnection(reg, int(rai))
						}
						if rai&borderReg != 0 {
							reg.connectsToBorder = true
						}
					}
				}
			}

			// Update overlapping regions.
			for i := 0; i < len(lregs)-1; i++ {
				for j := i + 1; j < len(lregs); j++ {
					if lregs[i] != lregs[j] {
						ri := &regions[lregs[i]]
						rj := &regions[lregs[j]]
						addUniqueFloorRegion(ri, lregs[j])
						addUniqueFloorRegion(rj, lregs[i])
					}
				}
			}
		}
	}

	// Create 2D layers from regions.
	layerId := uint16(1)

	for i := 0; i < nreg; i++ {
		regions[i].id = 0
	}

	// Merge montone regions to create non-overlapping areas.
	stack := make([]int, 0, 32)
	for i := 1; i < nreg; i++ {
		root := &regions[i]
		// Skip already visited.
		if root.id != 0 {
			continue
		}

		// Start search.
		root.id = layerId

		stack = stack[:0]
		stack = append(stack, i)

		for len(stack) > 0 {
			// Pop front
			reg := &regions[stack[0]]
			for j := 0; j < len(stack)-1; j++ {
				stack[j] = stack[j+1]
			}
			stack = stack[:len(stack)-1]

			ncons := len(reg.connections)
			for j := 0; j < ncons; j++ {
				nei := reg.connections[j]
				regn := &regions[nei]
				// Skip already visited.
				if regn.id != 0 {
					continue
				}
				// Skip if different area type.
				if reg.areaType != regn.areaType {
					continue
				}
				// Skip if the neighbour is overlapping root region.
				overlap := false
				for k := 0; k < len(root.floors); k++ {
					if root.floors[k] == nei {
						overlap = true
						break
					}
				}
				if overlap {
					continue
				}

				// Deepen
				stack = append(stack, nei)

				// Mark layer id
				regn.id = layerId
				// Merge current layers to root.
				for k := 0; k < len(regn.floors); k++ {
					addUniqueFloorRegion(root, regn.floors[k])
				}
				if regn.ymin < root.ymin {
					root.ymin = regn.ymin
				}
				if regn.ymax > root.ymax {
					root.ymax = regn.ymax
				}
				root.spanCount += regn.spanCount
				regn.spanCount = 0
				root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder
			}
		}

		layerId++
	}

	// Remove small regions
	for i := 0; i < nreg; i++ {
		if regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder {
			reg := regions[i].id
			for j := 0; j < nreg; j++ {
				if regions[j].id == reg {
					regions[j].id = 0
				}
			}
		}
	}

	// Compress region Ids.
	for i := 0; i < nreg; i++ {
		regions[i].remap = false
		if regions[i].id == 0 {
			continue
		}
		if regions[i].id&borderReg != 0 {
			continue
		}
		regions[i].remap = true
	}

	regIdGen := uint16(0)
	for i := 0; i < nreg; i++ {
		if !regions[i].remap {
			continue
		}
		oldId := regions[i].id
		regIdGen++
		newId := regIdGen
		for j := i; j < nreg; j++ {
			if regions[j].id == oldId {
				regions[j].id = newId
				regions[j].remap = false
			}
		}
	}
	*maxRegionId = regIdGen

	// Remap regions.
	for i := 0; i < chf.SpanCount; i++ {
		if srcReg[i]&borderReg == 0 {
			srcReg[i] = regions[srcReg[i]].id
		}
	}
}

func paintRectRegion(minx, maxx, miny, maxy int, regId uint16, chf *CompactHeightfield, srcReg []uint16) {
	w := chf.Width
	for y := miny; y < maxy; y++ {
		for x := minx; x < maxx; x++ {
			c := chf.Cells[x+y*w]
			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				if chf.Areas[i] != nullArea {
					srcReg[i] = regId
				}
			}
		}
	}
}

// BuildDistanceField builds a distance field from the compact heightfield.
// This is usually the second to the last step before building regions.
func BuildDistanceField(ctx context.Context, chf *CompactHeightfield) bool {
	defer ScopedTimer(ctx, TimerBuildDistanceField)()

	if chf.Dist != nil {
		chf.Dist = nil
	}

	src := make([]uint16, chf.SpanCount)
	dst := make([]uint16, chf.SpanCount)

	maxDist := calculateDistanceField(chf, src)
	chf.MaxDistance = maxDist

	// Blur
	boxBlur(chf, 1, src, dst)

	// Store distance.
	chf.Dist = src

	return true
}

// BuildRegionsMonotone partitions the heightfield into monotone regions.
func BuildRegionsMonotone(ctx context.Context, chf *CompactHeightfield, borderSize, minRegionArea, mergeRegionArea int) bool {
	w := chf.Width
	h := chf.Height
	id := uint16(1)

	srcReg := make([]uint16, chf.SpanCount)

	nsweeps := w
	if h > nsweeps {
		nsweeps = h
	}
	sweeps := make([]SweepSpan, nsweeps+1)

	// Mark border regions.
	if borderSize > 0 {
		bw := borderSize
		if w < bw {
			bw = w
		}
		bh := borderSize
		if h < bh {
			bh = h
		}
		paintRectRegion(0, bw, 0, h, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(w-bw, w, 0, h, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(0, w, 0, bh, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(0, w, h-bh, h, id|borderReg, chf, srcReg)
		id++
	}

	chf.BorderSize = borderSize

	prev := make([]int, 256)

	// Sweep one line at a time.
	for y := borderSize; y < h-borderSize; y++ {
		// Collect spans from this row.
		for i := range prev {
			prev[i] = 0
		}
		if int(id)+1 > len(prev) {
			newPrev := make([]int, int(id)+1)
			copy(newPrev, prev)
			prev = newPrev
		}
		rid := uint16(1)

		for x := borderSize; x < w-borderSize; x++ {
			c := chf.Cells[x+y*w]

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				if chf.Areas[i] == nullArea {
					continue
				}

				// -x
				previd := uint16(0)
				if Con(&s, 0) != notConnected {
					ax := x + DirOffsetX(0)
					ay := y + DirOffsetZ(0)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 0)
					if srcReg[ai]&borderReg == 0 && chf.Areas[i] == chf.Areas[ai] {
						previd = srcReg[ai]
					}
				}

				if previd == 0 {
					previd = rid
					rid++
					sweeps[previd].rid = previd
					sweeps[previd].ns = 0
					sweeps[previd].nei = 0
				}

				// -y
				if Con(&s, 3) != notConnected {
					ax := x + DirOffsetX(3)
					ay := y + DirOffsetZ(3)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 3)
					if srcReg[ai] != 0 && srcReg[ai]&borderReg == 0 && chf.Areas[i] == chf.Areas[ai] {
						nr := srcReg[ai]
						if sweeps[previd].nei == 0 || sweeps[previd].nei == nr {
							sweeps[previd].nei = nr
							sweeps[previd].ns++
							prev[nr]++
						} else {
							sweeps[previd].nei = nullNei
						}
					}
				}

				srcReg[i] = previd
			}
		}

		// Create unique ID.
		for i := 1; i < int(rid); i++ {
			if sweeps[i].nei != nullNei && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == int(sweeps[i].ns) {
				sweeps[i].id = sweeps[i].nei
			} else {
				sweeps[i].id = id
				id++
			}
		}

		// Remap IDs
		for x := borderSize; x < w-borderSize; x++ {
			c := chf.Cells[x+y*w]

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				if srcReg[i] > 0 && srcReg[i] < rid {
					srcReg[i] = sweeps[srcReg[i]].id
				}
			}
		}
	}

	// Merge regions and filter out small regions.
	overlaps := make([]int, 0)
	chf.MaxRegions = id
	mergeAndFilterRegions(minRegionArea, mergeRegionArea, &chf.MaxRegions, chf, srcReg, &overlaps)

	// Store the result out.
	for i := 0; i < chf.SpanCount; i++ {
		chf.Spans[i].Reg = srcReg[i]
	}

	return true
}

// BuildRegions partitions the compact heightfield into regions using the watershed algorithm.
func BuildRegions(ctx context.Context, chf *CompactHeightfield, borderSize, minRegionArea, mergeRegionArea int) bool {
	w := chf.Width
	h := chf.Height

	buf := make([]uint16, chf.SpanCount*2)
	srcReg := buf[:chf.SpanCount]
	srcDist := buf[chf.SpanCount:]

	regionId := uint16(1)
	level := (chf.MaxDistance + 1) & ^uint16(1)

	const logNbStacks = 3
	const nbStacks = 1 << logNbStacks
	lvlStacks := make([][]LevelStackEntry, nbStacks)
	for i := 0; i < nbStacks; i++ {
		lvlStacks[i] = make([]LevelStackEntry, 0, 256)
	}

	stack := make([]LevelStackEntry, 0, 256)

	if borderSize > 0 {
		bw := borderSize
		if w < bw {
			bw = w
		}
		bh := borderSize
		if h < bh {
			bh = h
		}

		paintRectRegion(0, bw, 0, h, regionId|borderReg, chf, srcReg)
		regionId++
		paintRectRegion(w-bw, w, 0, h, regionId|borderReg, chf, srcReg)
		regionId++
		paintRectRegion(0, w, 0, bh, regionId|borderReg, chf, srcReg)
		regionId++
		paintRectRegion(0, w, h-bh, h, regionId|borderReg, chf, srcReg)
		regionId++
	}

	chf.BorderSize = borderSize

	sId := -1
	for level > 0 {
		if level >= 2 {
			level = level - 2
		} else {
			level = 0
		}
		sId = (sId + 1) & (nbStacks - 1)

		if sId == 0 {
			sortCellsByLevel(level, chf, srcReg, nbStacks, lvlStacks, 1)
		} else {
			appendStacks(lvlStacks[sId-1], &lvlStacks[sId], srcReg)
		}

		// Expand current regions until no empty connected cells found.
		expandRegions(8, level, chf, srcReg, srcDist, &lvlStacks[sId], false)

		// Mark new regions with IDs.
		for j := 0; j < len(lvlStacks[sId]); j++ {
			current := lvlStacks[sId][j]
			x := current.x
			y := current.y
			i := current.index
			if i >= 0 && srcReg[i] == 0 {
				if floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, &stack) {
					if regionId == 0xFFFF {
						return false
					}
					regionId++
				}
			}
		}
	}

	// Expand current regions until no empty connected cells found.
	expandRegions(8*8, 0, chf, srcReg, srcDist, &stack, true)

	// Merge regions and filter out small regions.
	overlaps := make([]int, 0)
	chf.MaxRegions = regionId
	mergeAndFilterRegions(minRegionArea, mergeRegionArea, &chf.MaxRegions, chf, srcReg, &overlaps)

	// Write the result out.
	for i := 0; i < chf.SpanCount; i++ {
		chf.Spans[i].Reg = srcReg[i]
	}

	return true
}

// BuildLayerRegions builds regions using layer-based approach (for layered navigation mesh).
func BuildLayerRegions(ctx context.Context, chf *CompactHeightfield, borderSize, minRegionArea int) bool {
	w := chf.Width
	h := chf.Height
	id := uint16(1)

	srcReg := make([]uint16, chf.SpanCount)

	nsweeps := w
	if h > nsweeps {
		nsweeps = h
	}
	sweeps := make([]SweepSpan, nsweeps+1)

	// Mark border regions.
	if borderSize > 0 {
		bw := borderSize
		if w < bw {
			bw = w
		}
		bh := borderSize
		if h < bh {
			bh = h
		}
		paintRectRegion(0, bw, 0, h, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(w-bw, w, 0, h, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(0, w, 0, bh, id|borderReg, chf, srcReg)
		id++
		paintRectRegion(0, w, h-bh, h, id|borderReg, chf, srcReg)
		id++
	}

	chf.BorderSize = borderSize

	prev := make([]int, 256)

	// Sweep one line at a time.
	for y := borderSize; y < h-borderSize; y++ {
		for i := range prev {
			prev[i] = 0
		}
		if int(id)+1 > len(prev) {
			newPrev := make([]int, int(id)+1)
			copy(newPrev, prev)
			prev = newPrev
		}
		rid := uint16(1)

		for x := borderSize; x < w-borderSize; x++ {
			c := chf.Cells[x+y*w]

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				s := chf.Spans[i]
				if chf.Areas[i] == nullArea {
					continue
				}

				// -x
				previd := uint16(0)
				if Con(&s, 0) != notConnected {
					ax := x + DirOffsetX(0)
					ay := y + DirOffsetZ(0)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 0)
					if srcReg[ai]&borderReg == 0 && chf.Areas[i] == chf.Areas[ai] {
						previd = srcReg[ai]
					}
				}

				if previd == 0 {
					previd = rid
					rid++
					sweeps[previd].rid = previd
					sweeps[previd].ns = 0
					sweeps[previd].nei = 0
				}

				// -y
				if Con(&s, 3) != notConnected {
					ax := x + DirOffsetX(3)
					ay := y + DirOffsetZ(3)
					ai := int(chf.Cells[ax+ay*w].Index) + Con(&s, 3)
					if srcReg[ai] != 0 && srcReg[ai]&borderReg == 0 && chf.Areas[i] == chf.Areas[ai] {
						nr := srcReg[ai]
						if sweeps[previd].nei == 0 || sweeps[previd].nei == nr {
							sweeps[previd].nei = nr
							sweeps[previd].ns++
							prev[nr]++
						} else {
							sweeps[previd].nei = nullNei
						}
					}
				}

				srcReg[i] = previd
			}
		}

		// Create unique ID.
		for i := 1; i < int(rid); i++ {
			if sweeps[i].nei != nullNei && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == int(sweeps[i].ns) {
				sweeps[i].id = sweeps[i].nei
			} else {
				sweeps[i].id = id
				id++
			}
		}

		// Remap IDs
		for x := borderSize; x < w-borderSize; x++ {
			c := chf.Cells[x+y*w]

			for i := int(c.Index); i < int(c.Index+c.Count); i++ {
				if srcReg[i] > 0 && srcReg[i] < rid {
					srcReg[i] = sweeps[srcReg[i]].id
				}
			}
		}
	}

	// Merge monotone regions to layers and remove small regions.
	chf.MaxRegions = id
	mergeAndFilterLayerRegions(minRegionArea, &chf.MaxRegions, chf, srcReg)

	// Store the result out.
	for i := 0; i < chf.SpanCount; i++ {
		chf.Spans[i].Reg = srcReg[i]
	}

	return true
}
