// Package recast implements navigation mesh generation.
package recast

import (
	"context"
)

// InsertSort sorts the given data in-place using insertion sort.
func insertSort(data []uint8) {
	for valueIndex := 1; valueIndex < len(data); valueIndex++ {
		value := data[valueIndex]
		insertionIndex := valueIndex - 1
		for insertionIndex >= 0 && data[insertionIndex] > value {
			data[insertionIndex+1] = data[insertionIndex]
			insertionIndex--
		}
		data[insertionIndex+1] = value
	}
}

// ErodeWalkableArea erodes the walkable area of a compact heightfield.
func ErodeWalkableArea(ctx context.Context, erosionRadius int, chf *CompactHeightfield) error {
	xSize := chf.Width
	zSize := chf.Height
	zStride := xSize // For readability

	defer ScopedTimer(ctx, TimerErodeArea)()

	distanceToBoundary := make([]uint8, chf.SpanCount)
	for i := range distanceToBoundary {
		distanceToBoundary[i] = 0xff
	}

	// Mark boundary cells.
	for z := 0; z < zSize; z++ {
		for x := 0; x < xSize; x++ {
			cell := chf.Cells[x+z*zStride]
			for spanIndex := int(cell.Index); spanIndex < int(cell.Index+cell.Count); spanIndex++ {
				if chf.Areas[spanIndex] == NullArea {
					distanceToBoundary[spanIndex] = 0
					continue
				}
				span := chf.Spans[spanIndex]

				// Check that there is a non-null adjacent span in each of the 4 cardinal directions.
				neighborCount := 0
				for direction := 0; direction < 4; direction++ {
					neighborConnection := Con(&span, direction)
					if neighborConnection == NotConnected {
						break
					}

					neighborX := x + DirOffsetX(direction)
					neighborZ := z + DirOffsetZ(direction)
					neighborSpanIndex := int(chf.Cells[neighborX+neighborZ*zStride].Index) + neighborConnection

					if chf.Areas[neighborSpanIndex] == NullArea {
						break
					}
					neighborCount++
				}

				// At least one missing neighbour, so this is a boundary cell.
				if neighborCount != 4 {
					distanceToBoundary[spanIndex] = 0
				}
			}
		}
	}

	var newDistance uint8

	// Pass 1
	for z := 0; z < zSize; z++ {
		for x := 0; x < xSize; x++ {
			cell := chf.Cells[x+z*zStride]
			maxSpanIndex := int(cell.Index + cell.Count)
			for spanIndex := int(cell.Index); spanIndex < maxSpanIndex; spanIndex++ {
				span := chf.Spans[spanIndex]

				if Con(&span, 0) != NotConnected {
					// (-1,0)
					aX := x + DirOffsetX(0)
					aY := z + DirOffsetZ(0)
					aIndex := int(chf.Cells[aX+aY*xSize].Index) + Con(&span, 0)
					aSpan := chf.Spans[aIndex]
					newDistance = uint8(Min(int(distanceToBoundary[aIndex])+2, 255))
					if newDistance < distanceToBoundary[spanIndex] {
						distanceToBoundary[spanIndex] = newDistance
					}

					// (-1,-1)
					if Con(&aSpan, 3) != NotConnected {
						bX := aX + DirOffsetX(3)
						bY := aY + DirOffsetZ(3)
						bIndex := int(chf.Cells[bX+bY*xSize].Index) + Con(&aSpan, 3)
						newDistance = uint8(Min(int(distanceToBoundary[bIndex])+3, 255))
						if newDistance < distanceToBoundary[spanIndex] {
							distanceToBoundary[spanIndex] = newDistance
						}
					}
				}
				if Con(&span, 3) != NotConnected {
					// (0,-1)
					aX := x + DirOffsetX(3)
					aY := z + DirOffsetZ(3)
					aIndex := int(chf.Cells[aX+aY*xSize].Index) + Con(&span, 3)
					aSpan := chf.Spans[aIndex]
					newDistance = uint8(Min(int(distanceToBoundary[aIndex])+2, 255))
					if newDistance < distanceToBoundary[spanIndex] {
						distanceToBoundary[spanIndex] = newDistance
					}

					// (1,-1)
					if Con(&aSpan, 2) != NotConnected {
						bX := aX + DirOffsetX(2)
						bY := aY + DirOffsetZ(2)
						bIndex := int(chf.Cells[bX+bY*xSize].Index) + Con(&aSpan, 2)
						newDistance = uint8(Min(int(distanceToBoundary[bIndex])+3, 255))
						if newDistance < distanceToBoundary[spanIndex] {
							distanceToBoundary[spanIndex] = newDistance
						}
					}
				}
			}
		}
	}

	// Pass 2
	for z := zSize - 1; z >= 0; z-- {
		for x := xSize - 1; x >= 0; x-- {
			cell := chf.Cells[x+z*zStride]
			maxSpanIndex := int(cell.Index + cell.Count)
			for spanIndex := int(cell.Index); spanIndex < maxSpanIndex; spanIndex++ {
				span := chf.Spans[spanIndex]

				if Con(&span, 2) != NotConnected {
					// (1,0)
					aX := x + DirOffsetX(2)
					aY := z + DirOffsetZ(2)
					aIndex := int(chf.Cells[aX+aY*xSize].Index) + Con(&span, 2)
					aSpan := chf.Spans[aIndex]
					newDistance = uint8(Min(int(distanceToBoundary[aIndex])+2, 255))
					if newDistance < distanceToBoundary[spanIndex] {
						distanceToBoundary[spanIndex] = newDistance
					}

					// (1,1)
					if Con(&aSpan, 1) != NotConnected {
						bX := aX + DirOffsetX(1)
						bY := aY + DirOffsetZ(1)
						bIndex := int(chf.Cells[bX+bY*xSize].Index) + Con(&aSpan, 1)
						newDistance = uint8(Min(int(distanceToBoundary[bIndex])+3, 255))
						if newDistance < distanceToBoundary[spanIndex] {
							distanceToBoundary[spanIndex] = newDistance
						}
					}
				}
				if Con(&span, 1) != NotConnected {
					// (0,1)
					aX := x + DirOffsetX(1)
					aY := z + DirOffsetZ(1)
					aIndex := int(chf.Cells[aX+aY*xSize].Index) + Con(&span, 1)
					aSpan := chf.Spans[aIndex]
					newDistance = uint8(Min(int(distanceToBoundary[aIndex])+2, 255))
					if newDistance < distanceToBoundary[spanIndex] {
						distanceToBoundary[spanIndex] = newDistance
					}

					// (-1,1)
					if Con(&aSpan, 0) != NotConnected {
						bX := aX + DirOffsetX(0)
						bY := aY + DirOffsetZ(0)
						bIndex := int(chf.Cells[bX+bY*xSize].Index) + Con(&aSpan, 0)
						newDistance = uint8(Min(int(distanceToBoundary[bIndex])+3, 255))
						if newDistance < distanceToBoundary[spanIndex] {
							distanceToBoundary[spanIndex] = newDistance
						}
					}
				}
			}
		}
	}

	minBoundaryDistance := uint8(erosionRadius * 2)
	for spanIndex := 0; spanIndex < chf.SpanCount; spanIndex++ {
		if distanceToBoundary[spanIndex] < minBoundaryDistance {
			chf.Areas[spanIndex] = NullArea
		}
	}

	return nil
}

// MedianFilterWalkableArea applies a median filter to the walkable area of a compact heightfield.
func MedianFilterWalkableArea(ctx context.Context, chf *CompactHeightfield) error {
	xSize := chf.Width
	zSize := chf.Height
	zStride := xSize // For readability

	defer ScopedTimer(ctx, TimerMedianArea)()

	areas := make([]uint8, chf.SpanCount)
	for i := range areas {
		areas[i] = 0xff
	}

	for z := 0; z < zSize; z++ {
		for x := 0; x < xSize; x++ {
			cell := chf.Cells[x+z*zStride]
			maxSpanIndex := int(cell.Index + cell.Count)
			for spanIndex := int(cell.Index); spanIndex < maxSpanIndex; spanIndex++ {
				span := chf.Spans[spanIndex]
				if chf.Areas[spanIndex] == NullArea {
					areas[spanIndex] = chf.Areas[spanIndex]
					continue
				}

				neighborAreas := [9]uint8{}
				for neighborIndex := 0; neighborIndex < 9; neighborIndex++ {
					neighborAreas[neighborIndex] = chf.Areas[spanIndex]
				}

				for dir := 0; dir < 4; dir++ {
					if Con(&span, dir) == NotConnected {
						continue
					}

					aX := x + DirOffsetX(dir)
					aZ := z + DirOffsetZ(dir)
					aIndex := int(chf.Cells[aX+aZ*zStride].Index) + Con(&span, dir)
					if chf.Areas[aIndex] != NullArea {
						neighborAreas[dir*2+0] = chf.Areas[aIndex]
					}

					aSpan := chf.Spans[aIndex]
					dir2 := (dir + 1) & 0x3
					neighborConnection2 := Con(&aSpan, dir2)
					if neighborConnection2 != NotConnected {
						bX := aX + DirOffsetX(dir2)
						bZ := aZ + DirOffsetZ(dir2)
						bIndex := int(chf.Cells[bX+bZ*zStride].Index) + neighborConnection2
						if chf.Areas[bIndex] != NullArea {
							neighborAreas[dir*2+1] = chf.Areas[bIndex]
						}
					}
				}
				insertSort(neighborAreas[:])
				areas[spanIndex] = neighborAreas[4]
			}
		}
	}

	copy(chf.Areas, areas)

	return nil
}

// MarkBoxArea applies an area id to all spans within the specified bounding box (AABB).
func MarkBoxArea(ctx context.Context, boxMinBounds, boxMaxBounds [3]float32, areaID uint8, chf *CompactHeightfield) error {
	defer ScopedTimer(ctx, TimerMarkBoxArea)()

	xSize := chf.Width
	zSize := chf.Height
	zStride := xSize // For readability

	// Find the footprint of the box area in grid cell coordinates.
	minX := int((boxMinBounds[0] - chf.Bmin[0]) / chf.Cs)
	minY := int((boxMinBounds[1] - chf.Bmin[1]) / chf.Ch)
	minZ := int((boxMinBounds[2] - chf.Bmin[2]) / chf.Cs)
	maxX := int((boxMaxBounds[0] - chf.Bmin[0]) / chf.Cs)
	maxY := int((boxMaxBounds[1] - chf.Bmin[1]) / chf.Ch)
	maxZ := int((boxMaxBounds[2] - chf.Bmin[2]) / chf.Cs)

	// Early-out if the box is outside the bounds of the grid.
	if maxX < 0 || minX >= xSize || maxZ < 0 || minZ >= zSize {
		return nil
	}

	// Clamp relevant bound coordinates to the grid.
	if minX < 0 {
		minX = 0
	}
	if maxX >= xSize {
		maxX = xSize - 1
	}
	if minZ < 0 {
		minZ = 0
	}
	if maxZ >= zSize {
		maxZ = zSize - 1
	}

	// Mark relevant cells.
	for z := minZ; z <= maxZ; z++ {
		for x := minX; x <= maxX; x++ {
			cell := chf.Cells[x+z*zStride]
			maxSpanIndex := int(cell.Index + cell.Count)
			for spanIndex := int(cell.Index); spanIndex < maxSpanIndex; spanIndex++ {
				span := chf.Spans[spanIndex]

				// Skip if the span is outside the box extents.
				if int(span.Y) < minY || int(span.Y) > maxY {
					continue
				}

				// Skip if the span has been removed.
				if chf.Areas[spanIndex] == NullArea {
					continue
				}

				// Mark the span.
				chf.Areas[spanIndex] = areaID
			}
		}
	}
	return nil
}

// MarkConvexPolyArea applies the area id to all spans within the specified convex polygon.
func MarkConvexPolyArea(ctx context.Context, verts []float32, numVerts int, minY, maxY float32, areaID uint8, chf *CompactHeightfield) error {
	defer ScopedTimer(ctx, TimerMarkConvexPolyArea)()

	xSize := chf.Width
	zSize := chf.Height
	zStride := xSize // For readability

	// Compute the bounding box of the polygon
	var bmin, bmax [3]float32
	bmin = [3]float32{verts[0], verts[1], verts[2]}
	bmax = [3]float32{verts[0], verts[1], verts[2]}
	for i := 1; i < numVerts; i++ {
		v := [3]float32{verts[i*3], verts[i*3+1], verts[i*3+2]}
		bmin = Vmin(bmin, v)
		bmax = Vmax(bmax, v)
	}
	bmin[1] = minY
	bmax[1] = maxY

	// Compute the grid footprint of the polygon
	minx := int((bmin[0] - chf.Bmin[0]) / chf.Cs)
	miny := int((bmin[1] - chf.Bmin[1]) / chf.Ch)
	minz := int((bmin[2] - chf.Bmin[2]) / chf.Cs)
	maxx := int((bmax[0] - chf.Bmin[0]) / chf.Cs)
	maxy := int((bmax[1] - chf.Bmin[1]) / chf.Ch)
	maxz := int((bmax[2] - chf.Bmin[2]) / chf.Cs)

	// Early-out if the polygon lies entirely outside the grid.
	if maxx < 0 || minx >= xSize || maxz < 0 || minz >= zSize {
		return nil
	}

	// Clamp the polygon footprint to the grid
	if minx < 0 {
		minx = 0
	}
	if maxx >= xSize {
		maxx = xSize - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= zSize {
		maxz = zSize - 1
	}

	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			cell := chf.Cells[x+z*zStride]
			maxSpanIndex := int(cell.Index + cell.Count)
			for spanIndex := int(cell.Index); spanIndex < maxSpanIndex; spanIndex++ {
				span := chf.Spans[spanIndex]

				// Skip if span is removed.
				if chf.Areas[spanIndex] == NullArea {
					continue
				}

				// Skip if y extents don't overlap.
				if int(span.Y) < miny || int(span.Y) > maxy {
					continue
				}

				point := [3]float32{
					chf.Bmin[0] + (float32(x)+0.5)*chf.Cs,
					0,
					chf.Bmin[2] + (float32(z)+0.5)*chf.Cs,
				}

				if PointInPoly(numVerts, verts, point) {
					chf.Areas[spanIndex] = areaID
				}
			}
		}
	}
	return nil
}

// OffsetPoly expands a convex polygon along its vertex normals by the given offset amount.
func OffsetPoly(verts []float32, numVerts int, offset float32, outVerts []float32, maxOutVerts int) int {
	numOutVerts := 0

	for vertIndex := 0; vertIndex < numVerts; vertIndex++ {
		// Grab three vertices of the polygon.
		vertIndexA := (vertIndex + numVerts - 1) % numVerts
		vertIndexB := vertIndex
		vertIndexC := (vertIndex + 1) % numVerts
		vertA := [3]float32{verts[vertIndexA*3], verts[vertIndexA*3+1], verts[vertIndexA*3+2]}
		vertB := [3]float32{verts[vertIndexB*3], verts[vertIndexB*3+1], verts[vertIndexB*3+2]}
		vertC := [3]float32{verts[vertIndexC*3], verts[vertIndexC*3+1], verts[vertIndexC*3+2]}

		// From A to B on the x/z plane
		prevSegmentDir := Vsub(vertB, vertA)
		prevSegmentDir[1] = 0 // Squash onto x/z plane
		prevSegmentDir = VsafeNormalize(prevSegmentDir)

		// From B to C on the x/z plane
		currSegmentDir := Vsub(vertC, vertB)
		currSegmentDir[1] = 0 // Squash onto x/z plane
		currSegmentDir = VsafeNormalize(currSegmentDir)

		// The y component of the cross product of the two normalized segment directions.
		cross := currSegmentDir[0]*prevSegmentDir[2] - prevSegmentDir[0]*currSegmentDir[2]

		// CCW perpendicular vector to AB. The segment normal.
		prevSegmentNormX := -prevSegmentDir[2]
		prevSegmentNormZ := prevSegmentDir[0]

		// CCW perpendicular vector to BC. The segment normal.
		currSegmentNormX := -currSegmentDir[2]
		currSegmentNormZ := currSegmentDir[0]

		// Average the two segment normals to get the proportional miter offset for B.
		cornerMiterX := (prevSegmentNormX + currSegmentNormX) * 0.5
		cornerMiterZ := (prevSegmentNormZ + currSegmentNormZ) * 0.5
		cornerMiterSqMag := cornerMiterX*cornerMiterX + cornerMiterZ*cornerMiterZ

		// If the magnitude of the segment normal average is less than about .69444,
		// the corner is an acute enough angle that the result should be beveled.
		bevel := cornerMiterSqMag*MiterLimit*MiterLimit < 1.0

		// Scale the corner miter so it's proportional to how much the corner should be offset compared to the edges.
		if cornerMiterSqMag > Epsilon {
			scale := 1.0 / cornerMiterSqMag
			cornerMiterX *= scale
			cornerMiterZ *= scale
		}

		if bevel && cross < 0.0 { // If the corner is convex and an acute enough angle, generate a bevel.
			if numOutVerts+2 > maxOutVerts {
				return 0
			}

			// Generate two bevel vertices at a distances from B proportional to the angle between the two segments.
			d := (1.0 - (prevSegmentDir[0]*currSegmentDir[0] + prevSegmentDir[2]*currSegmentDir[2])) * 0.5

			outVerts[numOutVerts*3+0] = vertB[0] + (-prevSegmentNormX+prevSegmentDir[0]*d)*offset
			outVerts[numOutVerts*3+1] = vertB[1]
			outVerts[numOutVerts*3+2] = vertB[2] + (-prevSegmentNormZ+prevSegmentDir[2]*d)*offset
			numOutVerts++

			outVerts[numOutVerts*3+0] = vertB[0] + (-currSegmentNormX-currSegmentDir[0]*d)*offset
			outVerts[numOutVerts*3+1] = vertB[1]
			outVerts[numOutVerts*3+2] = vertB[2] + (-currSegmentNormZ-currSegmentDir[2]*d)*offset
			numOutVerts++
		} else {
			if numOutVerts+1 > maxOutVerts {
				return 0
			}

			// Move B along the miter direction by the specified offset.
			outVerts[numOutVerts*3+0] = vertB[0] - cornerMiterX*offset
			outVerts[numOutVerts*3+1] = vertB[1]
			outVerts[numOutVerts*3+2] = vertB[2] - cornerMiterZ*offset
			numOutVerts++
		}
	}

	return numOutVerts
}

// MarkCylinderArea applies the area id to all spans within the specified y-axis-aligned cylinder.
func MarkCylinderArea(ctx context.Context, position [3]float32, radius, height float32, areaID uint8, chf *CompactHeightfield) error {
	defer ScopedTimer(ctx, TimerMarkCylinderArea)()

	xSize := chf.Width
	zSize := chf.Height
	zStride := xSize // For readability

	// Compute the bounding box of the cylinder
	cylinderBBMin := [3]float32{
		position[0] - radius,
		position[1],
		position[2] - radius,
	}
	cylinderBBMax := [3]float32{
		position[0] + radius,
		position[1] + height,
		position[2] + radius,
	}

	// Compute the grid footprint of the cylinder
	minx := int((cylinderBBMin[0] - chf.Bmin[0]) / chf.Cs)
	miny := int((cylinderBBMin[1] - chf.Bmin[1]) / chf.Ch)
	minz := int((cylinderBBMin[2] - chf.Bmin[2]) / chf.Cs)
	maxx := int((cylinderBBMax[0] - chf.Bmin[0]) / chf.Cs)
	maxy := int((cylinderBBMax[1] - chf.Bmin[1]) / chf.Ch)
	maxz := int((cylinderBBMax[2] - chf.Bmin[2]) / chf.Cs)

	// Early-out if the cylinder is completely outside the grid bounds.
	if maxx < 0 || minx >= xSize || maxz < 0 || minz >= zSize {
		return nil
	}

	// Clamp the cylinder bounds to the grid.
	if minx < 0 {
		minx = 0
	}
	if maxx >= xSize {
		maxx = xSize - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= zSize {
		maxz = zSize - 1
	}

	radiusSq := radius * radius

	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			cell := chf.Cells[x+z*zStride]
			maxSpanIndex := int(cell.Index + cell.Count)

			cellX := chf.Bmin[0] + (float32(x)+0.5)*chf.Cs
			cellZ := chf.Bmin[2] + (float32(z)+0.5)*chf.Cs
			deltaX := cellX - position[0]
			deltaZ := cellZ - position[2]

			// Skip this column if it's too far from the center point of the cylinder.
			if deltaX*deltaX+deltaZ*deltaZ >= radiusSq {
				continue
			}

			// Mark all overlapping spans
			for spanIndex := int(cell.Index); spanIndex < maxSpanIndex; spanIndex++ {
				span := chf.Spans[spanIndex]

				// Skip if span is removed.
				if chf.Areas[spanIndex] == NullArea {
					continue
				}

				// Mark if y extents overlap.
				if int(span.Y) >= miny && int(span.Y) <= maxy {
					chf.Areas[spanIndex] = areaID
				}
			}
		}
	}
	return nil
}
