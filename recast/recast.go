// Package recast implements navigation mesh generation.
package recast

import "math"

// Internal constants used by region, contour, and mesh building.
const (
	borderReg            = BorderReg
	notConnected         = NotConnected
	nullArea             = NullArea
	meshNullIdx          = MeshNullIdx
	multipleRegs         = MultipleRegs
	borderVertex         = BorderVertex
	areaBorder           = AreaBorder
	contourRegMask       = ContourRegMask
	contourTessWallEdges = ContourTessWallEdges
	contourTessAreaEdges = ContourTessAreaEdges
)

// CalcBounds calculates the bounding box of an array of vertices.
func CalcBounds(verts []float32, numVerts int) (minBounds, maxBounds [3]float32) {
	Vcopy(&minBounds, &[3]float32{verts[0], verts[1], verts[2]})
	Vcopy(&maxBounds, &[3]float32{verts[0], verts[1], verts[2]})
	for i := 1; i < numVerts; i++ {
		v := &[3]float32{verts[i*3], verts[i*3+1], verts[i*3+2]}
		Vmin(&minBounds, v)
		Vmax(&maxBounds, v)
	}
	return
}

// CalcGridSize calculates the grid size based on the bounding box and grid cell size.
func CalcGridSize(minBounds, maxBounds *[3]float32, cellSize float32) (int, int) {
	sizeX := int((maxBounds[0]-minBounds[0])/cellSize + 0.5)
	sizeZ := int((maxBounds[2]-minBounds[2])/cellSize + 0.5)
	return sizeX, sizeZ
}

// CreateHeightfield initializes a new heightfield.
func CreateHeightfield(ctx *Context, hf *Heightfield, sizeX, sizeZ int, minBounds, maxBounds *[3]float32, cellSize, cellHeight float32) bool {
	hf.Width = sizeX
	hf.Height = sizeZ
	Vcopy(&hf.Bmin, minBounds)
	Vcopy(&hf.Bmax, maxBounds)
	hf.Cs = cellSize
	hf.Ch = cellHeight
	hf.Spans = make([]*Span, hf.Width*hf.Height)
	return true
}

// MarkWalkableTriangles sets the area id of all triangles with a slope below the specified value to WalkableArea.
func MarkWalkableTriangles(ctx *Context, walkableSlopeAngle float32, verts []float32, numVerts int, tris []int, numTris int, triAreaIDs []uint8) {
	walkableThr := float32(math.Cos(float64(walkableSlopeAngle / 180.0 * Pi)))

	var norm [3]float32
	for i := 0; i < numTris; i++ {
		tri := tris[i*3 : i*3+3]
		v0 := &[3]float32{verts[tri[0]*3], verts[tri[0]*3+1], verts[tri[0]*3+2]}
		v1 := &[3]float32{verts[tri[1]*3], verts[tri[1]*3+1], verts[tri[1]*3+2]}
		v2 := &[3]float32{verts[tri[2]*3], verts[tri[2]*3+1], verts[tri[2]*3+2]}
		CalcTriNormal(v0, v1, v2, &norm)
		if norm[1] > walkableThr {
			triAreaIDs[i] = WalkableArea
		}
	}
}

// ClearUnwalkableTriangles sets the area id of all triangles with a slope greater than or equal to the specified value to NullArea.
func ClearUnwalkableTriangles(ctx *Context, walkableSlopeAngle float32, verts []float32, numVerts int, tris []int, numTris int, triAreaIDs []uint8) {
	walkableLimitY := float32(math.Cos(float64(walkableSlopeAngle / 180.0 * Pi)))

	var faceNormal [3]float32
	for i := 0; i < numTris; i++ {
		tri := tris[i*3 : i*3+3]
		v0 := &[3]float32{verts[tri[0]*3], verts[tri[0]*3+1], verts[tri[0]*3+2]}
		v1 := &[3]float32{verts[tri[1]*3], verts[tri[1]*3+1], verts[tri[1]*3+2]}
		v2 := &[3]float32{verts[tri[2]*3], verts[tri[2]*3+1], verts[tri[2]*3+2]}
		CalcTriNormal(v0, v1, v2, &faceNormal)
		if faceNormal[1] <= walkableLimitY {
			triAreaIDs[i] = NullArea
		}
	}
}

// GetHeightFieldSpanCount returns the number of non-null spans contained in the specified heightfield.
func GetHeightFieldSpanCount(ctx *Context, hf *Heightfield) int {
	numCols := hf.Width * hf.Height
	spanCount := 0
	for columnIndex := 0; columnIndex < numCols; columnIndex++ {
		for span := hf.Spans[columnIndex]; span != nil; span = span.Next {
			if span.Area != NullArea {
				spanCount++
			}
		}
	}
	return spanCount
}

// BuildCompactHeightfield builds a compact heightfield representing open space, from a heightfield representing solid space.
func BuildCompactHeightfield(ctx *Context, walkableHeight, walkableClimb int, hf *Heightfield, chf *CompactHeightfield) bool {
	Assert(ctx != nil)

	defer ctx.ScopedTimer(TimerBuildCompactHeightfield)()

	xSize := hf.Width
	zSize := hf.Height
	spanCount := GetHeightFieldSpanCount(ctx, hf)

	// Fill in header.
	chf.Width = xSize
	chf.Height = zSize
	chf.SpanCount = spanCount
	chf.WalkableHeight = walkableHeight
	chf.WalkableClimb = walkableClimb
	chf.MaxRegions = 0
	Vcopy(&chf.Bmin, &hf.Bmin)
	Vcopy(&chf.Bmax, &hf.Bmax)
	chf.Bmax[1] += float32(walkableHeight) * hf.Ch
	chf.Cs = hf.Cs
	chf.Ch = hf.Ch

	chf.Cells = make([]CompactCell, xSize*zSize)
	chf.Spans = make([]CompactSpan, spanCount)
	chf.Areas = make([]uint8, spanCount)
	for i := range chf.Areas {
		chf.Areas[i] = NullArea
	}

	const maxHeight = 0xffff

	// Fill in cells and spans.
	currentCellIndex := 0
	numColumns := xSize * zSize
	for columnIndex := 0; columnIndex < numColumns; columnIndex++ {
		span := hf.Spans[columnIndex]

		// If there are no spans at this cell, just leave the data to index=0, count=0.
		if span == nil {
			continue
		}

		cell := &chf.Cells[columnIndex]
		cell.Index = uint32(currentCellIndex)
		cell.Count = 0

		for ; span != nil; span = span.Next {
			if span.Area != NullArea {
				bot := int(span.Smax)
				top := maxHeight
				if span.Next != nil {
					top = int(span.Next.Smin)
				}
				chf.Spans[currentCellIndex].Y = uint16(Clamp(bot, 0, 0xffff))
				chf.Spans[currentCellIndex].H = uint8(Clamp(top-bot, 0, 0xff))
				chf.Areas[currentCellIndex] = uint8(span.Area)
				currentCellIndex++
				cell.Count++
			}
		}
	}

	// Find neighbour connections.
	const maxLayers = NotConnected - 1
	maxLayerIndex := 0
	zStride := xSize // for readability

	for z := 0; z < zSize; z++ {
		for x := 0; x < xSize; x++ {
			cell := chf.Cells[x+z*zStride]
			for i := int(cell.Index); i < int(cell.Index+cell.Count); i++ {
				span := &chf.Spans[i]

				for dir := 0; dir < 4; dir++ {
					SetCon(span, dir, NotConnected)
					neighborX := x + GetDirOffsetX(dir)
					neighborZ := z + GetDirOffsetZ(dir)

					// First check that the neighbor cell is in bounds.
					if neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize {
						continue
					}

					// Iterate over all neighbor spans and check if any of them is
					// accessible from current cell.
					neighborCell := chf.Cells[neighborX+neighborZ*zStride]
					for k := int(neighborCell.Index); k < int(neighborCell.Index+neighborCell.Count); k++ {
						neighborSpan := chf.Spans[k]
						bot := Max(int(span.Y), int(neighborSpan.Y))
						top := Min(int(span.Y)+int(span.H), int(neighborSpan.Y)+int(neighborSpan.H))

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if (top-bot) >= walkableHeight && Abs(int(neighborSpan.Y)-int(span.Y)) <= walkableClimb {
							// Mark direction as walkable.
							layerIndex := k - int(neighborCell.Index)
							if layerIndex < 0 || layerIndex > maxLayers {
								if layerIndex > maxLayerIndex {
									maxLayerIndex = layerIndex
								}
								continue
							}
							SetCon(span, dir, layerIndex)
							break
						}
					}
				}
			}
		}
	}

	if maxLayerIndex > maxLayers {
		ctx.Log(LogError, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)", maxLayerIndex, maxLayers)
	}

	return true
}
