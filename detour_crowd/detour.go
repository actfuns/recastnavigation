package detour_crowd

import "github.com/actfuns/recastnavigation/detour"

// NavMeshQueryInterface defines the subset of NavMeshQuery used by the crowd.
// This allows the crowd module to work with any implementation of the navigation
// mesh query.
type NavMeshQueryInterface interface {
	FindNearestPoly(pos [3]float32, halfExtents [3]float32, filter *QueryFilter) (PolyRef, [3]float32, error)
	IsValidPolyRef(ref PolyRef, filter *QueryFilter) bool
	MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result []float32, visited []PolyRef, maxVisitedSize int) (int, error)
	GetPolyHeight(ref PolyRef, pos [3]float32) (float32, error)
	ClosestPointOnPoly(ref PolyRef, pos [3]float32) ([3]float32, bool, error)
	FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, maxStraightPath int, options int) ([]float32, []uint8, []PolyRef, int, error)
	Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32, prevRef PolyRef, hit *RaycastHit) error
	FindPathSliced(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error
	UpdateSlicedPath(maxIter int) error
	GetPathFromSlicedPath(path []PolyRef, maxPath int) (int, error)
	GetAttachedNavMesh() *NavMesh
	ClosestPointOnPolyBoundary(ref PolyRef, pos [3]float32) ([3]float32, error)
	FindPolysAroundCircle(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error)
	GetPolyWallSegments(ref PolyRef, filter *QueryFilter, segs []NeighbourSeg, maxSegs int) (int, error)
}

// PolyRef is a polygon reference, re-exported from the detour package.
type PolyRef = detour.PolyRef

// NavMesh is a navigation mesh, re-exported from the detour package.
type NavMesh = detour.NavMesh

// QueryFilter is a query filter, re-exported from the detour package.
type QueryFilter = detour.QueryFilter

// NeighbourSeg stores a segment from a polygon's neighbours.
type NeighbourSeg = detour.NeighbourSeg

// RaycastHit stores the result of a raycast operation.
type RaycastHit = detour.RaycastHit
