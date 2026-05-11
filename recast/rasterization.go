// Package recast implements navigation mesh generation.
package recast

import (
	"fmt"
	"math"
)

// allocSpan allocates a new span in the heightfield.
// Uses a memory pool and free list to minimize actual allocations.
func allocSpan(hf *Heightfield) *Span {
	// If necessary, allocate new page and update the freelist.
	if hf.FreeList == nil || hf.FreeList.Next == nil {
		// Create new page.
		// Allocate memory for the new pool.
		spanPool := &SpanPool{}
		spanPool.Next = hf.Pools
		hf.Pools = spanPool

		// Add new spans to the free list.
		freeList := hf.FreeList
		// Add spans from the pool to the free list in reverse order
		for i := SpansPerPool - 1; i >= 0; i-- {
			spanPool.Items[i].Next = freeList
			freeList = &spanPool.Items[i]
		}
		hf.FreeList = &spanPool.Items[0]
	}

	// Pop item from the front of the free list.
	newSpan := hf.FreeList
	hf.FreeList = hf.FreeList.Next
	newSpan.Next = nil
	return newSpan
}

// freeSpan releases the memory used by the span back to the heightfield.
func freeSpan(hf *Heightfield, span *Span) {
	if span == nil {
		return
	}
	span.Next = hf.FreeList
	hf.FreeList = span
}

// addSpan adds a span to the heightfield. If the new span overlaps existing spans,
// it will merge the new span with the existing ones.
func addSpan(hf *Heightfield, x, z int, min, max uint16, areaID uint8, flagMergeThreshold int) bool {
	// Create the new span.
	newSpan := allocSpan(hf)
	if newSpan == nil {
		return false
	}
	newSpan.Smin = uint32(min)
	newSpan.Smax = uint32(max)
	newSpan.Area = uint32(areaID)
	newSpan.Next = nil

	columnIndex := x + z*hf.Width
	var previousSpan *Span
	currentSpan := hf.Spans[columnIndex]

	// Insert the new span, possibly merging it with existing spans.
	for currentSpan != nil {
		if currentSpan.Smin > newSpan.Smax {
			// Current span is completely after the new span, break.
			break
		}

		if currentSpan.Smax < newSpan.Smin {
			// Current span is completely before the new span. Keep going.
			previousSpan = currentSpan
			currentSpan = currentSpan.Next
		} else {
			// The new span overlaps with an existing span. Merge them.
			if currentSpan.Smin < newSpan.Smin {
				newSpan.Smin = currentSpan.Smin
			}
			if currentSpan.Smax > newSpan.Smax {
				newSpan.Smax = currentSpan.Smax
			}

			// Merge flags.
			if Abs(int(newSpan.Smax)-int(currentSpan.Smax)) <= flagMergeThreshold {
				// Higher area ID numbers indicate higher resolution priority.
				if currentSpan.Area > newSpan.Area {
					newSpan.Area = currentSpan.Area
				}
			}

			// Remove the current span since it's now merged with newSpan.
			next := currentSpan.Next
			freeSpan(hf, currentSpan)
			if previousSpan != nil {
				previousSpan.Next = next
			} else {
				hf.Spans[columnIndex] = next
			}
			currentSpan = next
		}
	}

	// Insert new span after prev
	if previousSpan != nil {
		newSpan.Next = previousSpan.Next
		previousSpan.Next = newSpan
	} else {
		// This span should go before the others in the list
		newSpan.Next = hf.Spans[columnIndex]
		hf.Spans[columnIndex] = newSpan
	}

	return true
}

// AddSpan adds a span to the specified heightfield. This is the public API.
func AddSpan(ctx *Context, hf *Heightfield, x, z int, spanMin, spanMax uint16, areaID uint8, flagMergeThreshold int) (bool, error) {
	if ctx == nil {
		return false, fmt.Errorf("recast: ctx must not be nil")
	}

	// Span is zero size or inverted size. Ignore.
	if spanMin >= spanMax {
		ctx.Log(LogWarning, "AddSpan: Adding a span with zero or negative size. Ignored.")
		return true, nil
	}

	if !addSpan(hf, x, z, spanMin, spanMax, areaID, flagMergeThreshold) {
		ctx.Log(LogError, "AddSpan: Out of memory.")
		return false, nil
	}

	return true, nil
}

// dividePoly divides a convex polygon of max 12 vertices into two convex polygons
// across a separating axis.
func dividePoly(inVerts []float32, inVertsCount int,
	outVerts1 []float32, outVerts1Count *int,
	outVerts2 []float32, outVerts2Count *int,
	axisOffset float32, axis Axis) {

	// How far positive or negative away from the separating axis is each vertex.
	inVertAxisDelta := make([]float32, inVertsCount)
	for inVert := 0; inVert < inVertsCount; inVert++ {
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert*3+int(axis)]
	}

	poly1Vert := 0
	poly2Vert := 0
	for inVertA, inVertB := 0, inVertsCount-1; inVertA < inVertsCount; inVertB, inVertA = inVertA, inVertA+1 {
		// If the two vertices are on the same side of the separating axis
		sameSide := (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0)

		if !sameSide {
			s := inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA])
			outVerts1[poly1Vert*3+0] = inVerts[inVertB*3+0] + (inVerts[inVertA*3+0]-inVerts[inVertB*3+0])*s
			outVerts1[poly1Vert*3+1] = inVerts[inVertB*3+1] + (inVerts[inVertA*3+1]-inVerts[inVertB*3+1])*s
			outVerts1[poly1Vert*3+2] = inVerts[inVertB*3+2] + (inVerts[inVertA*3+2]-inVerts[inVertB*3+2])*s
			copy(outVerts2[poly2Vert*3:poly2Vert*3+3], outVerts1[poly1Vert*3:poly1Vert*3+3])
			poly1Vert++
			poly2Vert++

			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			if inVertAxisDelta[inVertA] > 0 {
				copy(outVerts1[poly1Vert*3:poly1Vert*3+3], inVerts[inVertA*3:inVertA*3+3])
				poly1Vert++
			} else if inVertAxisDelta[inVertA] < 0 {
				copy(outVerts2[poly2Vert*3:poly2Vert*3+3], inVerts[inVertA*3:inVertA*3+3])
				poly2Vert++
			}
		} else {
			// add the inVertA point to the right polygon.
			if inVertAxisDelta[inVertA] >= 0 {
				copy(outVerts1[poly1Vert*3:poly1Vert*3+3], inVerts[inVertA*3:inVertA*3+3])
				poly1Vert++
				if inVertAxisDelta[inVertA] != 0 {
					continue
				}
			}
			copy(outVerts2[poly2Vert*3:poly2Vert*3+3], inVerts[inVertA*3:inVertA*3+3])
			poly2Vert++
		}
	}

	*outVerts1Count = poly1Vert
	*outVerts2Count = poly2Vert
}

// rasterizeTri rasterizes a single triangle to the heightfield.
func rasterizeTri(v0, v1, v2 *[3]float32, areaID uint8, hf *Heightfield,
	hfBBMin, hfBBMax *[3]float32,
	cellSize, inverseCellSize, inverseCellHeight float32,
	flagMergeThreshold int) bool {

	// Calculate the bounding box of the triangle.
	triBBMin := *v0
	triBBMin = Vmin(triBBMin, *v1)
	triBBMin = Vmin(triBBMin, *v2)

	triBBMax := *v0
	triBBMax = Vmax(triBBMax, *v1)
	triBBMax = Vmax(triBBMax, *v2)

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if !OverlapBounds(&triBBMin, &triBBMax, hfBBMin, hfBBMax) {
		return true
	}

	w := hf.Width
	h := hf.Height
	by := hfBBMax[1] - hfBBMin[1]

	// Calculate the footprint of the triangle on the grid's z-axis
	z0 := int((triBBMin[2] - hfBBMin[2]) * inverseCellSize)
	z1 := int((triBBMax[2] - hfBBMin[2]) * inverseCellSize)

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = Clamp(z0, -1, h-1)
	z1 = Clamp(z1, 0, h-1)

	// Clip the triangle into all grid cells it touches.
	buf := make([]float32, 7*3*4)
	in := buf[0 : 7*3]
	inRow := buf[7*3 : 7*3*2]
	p1 := buf[7*3*2 : 7*3*3]
	p2 := buf[7*3*3 : 7*3*4]

	copy(in[0:3], v0[:])
	copy(in[3:6], v1[:])
	copy(in[6:9], v2[:])
	var nvRow int
	nvIn := 3

	for z := z0; z <= z1; z++ {
		// Clip polygon to row. Store the remaining polygon as well
		cellZ := hfBBMin[2] + float32(z)*cellSize
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ+cellSize, AxisZ)
		in, p1 = p1, in

		if nvRow < 3 {
			continue
		}
		if z < 0 {
			continue
		}

		// find X-axis bounds of the row
		minX := inRow[0]
		maxX := inRow[0]
		for vert := 1; vert < nvRow; vert++ {
			if minX > inRow[vert*3] {
				minX = inRow[vert*3]
			}
			if maxX < inRow[vert*3] {
				maxX = inRow[vert*3]
			}
		}
		x0 := int((minX - hfBBMin[0]) * inverseCellSize)
		x1 := int((maxX - hfBBMin[0]) * inverseCellSize)
		if x1 < 0 || x0 >= w {
			continue
		}
		x0 = Clamp(x0, -1, w-1)
		x1 = Clamp(x1, 0, w-1)

		var nv int
		nv2 := nvRow

		for x := x0; x <= x1; x++ {
			// Clip polygon to column. store the remaining polygon as well
			cx := hfBBMin[0] + float32(x)*cellSize
			dividePoly(inRow, nv2, p1, &nv, p2, &nv2, cx+cellSize, AxisX)
			inRow, p2 = p2, inRow

			if nv < 3 {
				continue
			}
			if x < 0 {
				continue
			}

			// Calculate min and max of the span.
			spanMin := p1[1]
			spanMax := p1[1]
			for vert := 1; vert < nv; vert++ {
				if p1[vert*3+1] < spanMin {
					spanMin = p1[vert*3+1]
				}
				if p1[vert*3+1] > spanMax {
					spanMax = p1[vert*3+1]
				}
			}
			spanMin -= hfBBMin[1]
			spanMax -= hfBBMin[1]

			// Skip the span if it's completely outside the heightfield bounding box
			if spanMax < 0.0 {
				continue
			}
			if spanMin > by {
				continue
			}

			// Clamp the span to the heightfield bounding box.
			if spanMin < 0.0 {
				spanMin = 0
			}
			if spanMax > by {
				spanMax = by
			}

			// Snap the span to the heightfield height grid.
			spanMinCellIndex := uint16(Clamp(int(math.Floor(float64(spanMin*inverseCellHeight))), 0, SpanMaxHeight))
			spanMaxCellIndex := uint16(Clamp(int(math.Ceil(float64(spanMax*inverseCellHeight))), int(spanMinCellIndex)+1, SpanMaxHeight))

			if !addSpan(hf, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold) {
				return false
			}
		}
	}

	return true
}

// RasterizeTriangle rasterizes a single triangle into the specified heightfield.
func RasterizeTriangle(ctx *Context, v0, v1, v2 *[3]float32, areaID uint8, hf *Heightfield, flagMergeThreshold int) (bool, error) {
	if ctx == nil {
		return false, fmt.Errorf("recast: ctx must not be nil")
	}

	defer ctx.ScopedTimer(TimerRasterizeTriangles)()

	// Rasterize the single triangle.
	inverseCellSize := 1.0 / hf.Cs
	inverseCellHeight := 1.0 / hf.Ch
	if !rasterizeTri(v0, v1, v2, areaID, hf, &hf.Bmin, &hf.Bmax, hf.Cs, inverseCellSize, inverseCellHeight, flagMergeThreshold) {
		ctx.Log(LogError, "RasterizeTriangle: Out of memory.")
		return false, nil
	}

	return true, nil
}

// RasterizeTriangles rasterizes an indexed triangle mesh (int indices) into the specified heightfield.
func RasterizeTriangles(ctx *Context, verts []float32, numVerts int, tris []int, triAreaIDs []uint8, numTris int, hf *Heightfield, flagMergeThreshold int) (bool, error) {
	if ctx == nil {
		return false, fmt.Errorf("recast: ctx must not be nil")
	}

	defer ctx.ScopedTimer(TimerRasterizeTriangles)()

	// Rasterize the triangles.
	inverseCellSize := 1.0 / hf.Cs
	inverseCellHeight := 1.0 / hf.Ch
	for triIndex := 0; triIndex < numTris; triIndex++ {
		v0 := &[3]float32{verts[tris[triIndex*3+0]*3], verts[tris[triIndex*3+0]*3+1], verts[tris[triIndex*3+0]*3+2]}
		v1 := &[3]float32{verts[tris[triIndex*3+1]*3], verts[tris[triIndex*3+1]*3+1], verts[tris[triIndex*3+1]*3+2]}
		v2 := &[3]float32{verts[tris[triIndex*3+2]*3], verts[tris[triIndex*3+2]*3+1], verts[tris[triIndex*3+2]*3+2]}
		if !rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], hf, &hf.Bmin, &hf.Bmax, hf.Cs, inverseCellSize, inverseCellHeight, flagMergeThreshold) {
			ctx.Log(LogError, "RasterizeTriangles: Out of memory.")
			return false, nil
		}
	}

	return true, nil
}

// RasterizeTrianglesUShort rasterizes an indexed triangle mesh (uint16 indices) into the specified heightfield.
func RasterizeTrianglesUShort(ctx *Context, verts []float32, numVerts int, tris []uint16, triAreaIDs []uint8, numTris int, hf *Heightfield, flagMergeThreshold int) (bool, error) {
	if ctx == nil {
		return false, fmt.Errorf("recast: ctx must not be nil")
	}

	defer ctx.ScopedTimer(TimerRasterizeTriangles)()

	// Rasterize the triangles.
	inverseCellSize := 1.0 / hf.Cs
	inverseCellHeight := 1.0 / hf.Ch
	for triIndex := 0; triIndex < numTris; triIndex++ {
		v0 := &[3]float32{verts[tris[triIndex*3+0]*3], verts[tris[triIndex*3+0]*3+1], verts[tris[triIndex*3+0]*3+2]}
		v1 := &[3]float32{verts[tris[triIndex*3+1]*3], verts[tris[triIndex*3+1]*3+1], verts[tris[triIndex*3+1]*3+2]}
		v2 := &[3]float32{verts[tris[triIndex*3+2]*3], verts[tris[triIndex*3+2]*3+1], verts[tris[triIndex*3+2]*3+2]}
		if !rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], hf, &hf.Bmin, &hf.Bmax, hf.Cs, inverseCellSize, inverseCellHeight, flagMergeThreshold) {
			ctx.Log(LogError, "RasterizeTriangles: Out of memory.")
			return false, nil
		}
	}

	return true, nil
}

// RasterizeTrianglesVerts rasterizes a triangle list (sequential vertices) into the specified heightfield.
func RasterizeTrianglesVerts(ctx *Context, verts []float32, triAreaIDs []uint8, numTris int, hf *Heightfield, flagMergeThreshold int) (bool, error) {
	if ctx == nil {
		return false, fmt.Errorf("recast: ctx must not be nil")
	}

	defer ctx.ScopedTimer(TimerRasterizeTriangles)()

	// Rasterize the triangles.
	inverseCellSize := 1.0 / hf.Cs
	inverseCellHeight := 1.0 / hf.Ch
	for triIndex := 0; triIndex < numTris; triIndex++ {
		v0 := &[3]float32{verts[(triIndex*3+0)*3], verts[(triIndex*3+0)*3+1], verts[(triIndex*3+0)*3+2]}
		v1 := &[3]float32{verts[(triIndex*3+1)*3], verts[(triIndex*3+1)*3+1], verts[(triIndex*3+1)*3+2]}
		v2 := &[3]float32{verts[(triIndex*3+2)*3], verts[(triIndex*3+2)*3+1], verts[(triIndex*3+2)*3+2]}
		if !rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], hf, &hf.Bmin, &hf.Bmax, hf.Cs, inverseCellSize, inverseCellHeight, flagMergeThreshold) {
			ctx.Log(LogError, "RasterizeTriangles: Out of memory.")
			return false, nil
		}
	}

	return true, nil
}
