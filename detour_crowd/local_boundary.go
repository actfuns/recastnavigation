package detour_crowd

import "github.com/actfuns/recastnavigation/detour"

// LocalBoundary stores local boundary data for an agent.
type LocalBoundary struct {
	center [3]float32
	segs   [maxLocalSegs]Segment
	nsegs  int
	polys  [maxLocalPolys]PolyRef
	npolys int
}

// Segment represents a segment of the local boundary.
type Segment struct {
	s [6]float32 // Segment start/end (3 floats each)
	d float32    // Distance for pruning
}

const (
	maxLocalSegs  = 8
	maxLocalPolys = 16
)

// NewLocalBoundary creates a new local boundary.
func NewLocalBoundary() *LocalBoundary {
	b := &LocalBoundary{}
	b.Reset()
	return b
}

// Reset resets the local boundary data.
func (b *LocalBoundary) Reset() {
	b.center = [3]float32{mathMaxFloat32, mathMaxFloat32, mathMaxFloat32}
	b.npolys = 0
	b.nsegs = 0
}

func (b *LocalBoundary) addSegment(dist float32, s *[6]float32) {
	var seg *Segment

	if b.nsegs == 0 {
		// First, trivial accept.
		seg = &b.segs[0]
	} else if dist >= b.segs[b.nsegs-1].d {
		// Further than the last segment, skip.
		if b.nsegs >= maxLocalSegs {
			return
		}
		// Last, trivial accept.
		seg = &b.segs[b.nsegs]
	} else {
		// Insert inbetween.
		i := 0
		for i < b.nsegs {
			if dist <= b.segs[i].d {
				break
			}
			i++
		}
		tgt := i + 1
		n := recastMin(b.nsegs-i, maxLocalSegs-tgt)
		if n > 0 {
			copy(b.segs[tgt:tgt+n], b.segs[i:i+n])
		}
		seg = &b.segs[i]
	}

	seg.d = dist
	copy(seg.s[:], s[:])

	if b.nsegs < maxLocalSegs {
		b.nsegs++
	}
}

// Update updates the local boundary using the current position and navigation mesh query.
func (b *LocalBoundary) Update(ref PolyRef, pos [3]float32, collisionQueryRange float32,
	navquery *detour.NavMeshQuery, filter *QueryFilter) {

	if ref == 0 {
		b.center = [3]float32{mathMaxFloat32, mathMaxFloat32, mathMaxFloat32}
		b.nsegs = 0
		b.npolys = 0
		return
	}

	b.center = pos

	// First query non-overlapping polygons.
	var nresult int
	nresult, _ = navquery.FindLocalNeighbourhood(ref, pos, collisionQueryRange,
		filter, b.polys[:], nil, maxLocalPolys)
	b.npolys = nresult

	// Secondly, store all polygon edges.
	b.nsegs = 0
	maxSegsPerPoly := recastVerstsPerPolygon * 3
	segs := make([]NeighbourSeg, maxSegsPerPoly)

	for j := 0; j < b.npolys; j++ {
		nsegs, _ := navquery.GetPolyWallSegments(b.polys[j], filter, segs, maxSegsPerPoly)
		for k := 0; k < nsegs; k++ {
			sArr := segs[k].Seg
			// Skip too distant segments.
			distSqr, _ := recastDistancePtSegSqr2D(pos, [3]float32{sArr[0], sArr[1], sArr[2]}, [3]float32{sArr[3], sArr[4], sArr[5]})
			if distSqr > collisionQueryRange*collisionQueryRange {
				continue
			}
			b.addSegment(distSqr, &sArr)
		}
	}
}

// IsValid checks whether the local boundary is still valid.
func (b *LocalBoundary) IsValid(navquery *detour.NavMeshQuery, filter *QueryFilter) bool {
	if b.npolys == 0 {
		return false
	}

	// Check that all polygons still pass query filter.
	for i := 0; i < b.npolys; i++ {
		if !navquery.IsValidPolyRef(b.polys[i], filter) {
			return false
		}
	}

	return true
}

// GetCenter returns the center position.
func (b *LocalBoundary) GetCenter() *[3]float32 {
	return &b.center
}

// GetSegmentCount returns the number of boundary segments.
func (b *LocalBoundary) GetSegmentCount() int {
	return b.nsegs
}

// GetSegment returns the segment data at index i.
func (b *LocalBoundary) GetSegment(i int) *[6]float32 {
	return &b.segs[i].s
}

// Constants used internally
const mathMaxFloat32 = float32(^uint32(0) >> 1)

// Placeholder constants - these should be imported from recast
const recastVerstsPerPolygon = 6

func recastDistancePtSegSqr2D(pt, p, q [3]float32) (float32, float32) {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqz*pqz
	t := float32(0)
	if d > 0 {
		t = (pqx*dx + pqz*dz) / d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]
	return dx*dx + dz*dz, t
}
