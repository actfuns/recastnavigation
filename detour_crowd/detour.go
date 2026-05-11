package detour_crowd

import "github.com/actfuns/recastnavigation/detour"

// NavMeshQueryInterface defines the subset of NavMeshQuery used by the crowd.
// This allows the crowd module to work with any implementation of the navigation
// mesh query.
type NavMeshQueryInterface interface {
	FindNearestPoly(pos [3]float32, halfExtents [3]float32, filter *QueryFilter) (PolyRef, [3]float32, error)
	IsValidPolyRef(ref PolyRef, filter *QueryFilter) bool
	MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result *[3]float32, visited []PolyRef, nvisited *int, maxVisited int) error
	GetPolyHeight(ref PolyRef, pos [3]float32, height *float32) error
	ClosestPointOnPoly(ref PolyRef, pos [3]float32, result *[3]float32, posOverPoly *bool) error
	FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, cornerVerts []float32, cornerFlags []uint8, cornerPolys []PolyRef, ncorners *int, maxCorners int) error
	Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, t *float32, hitNormal *[3]float32, path []PolyRef, npath *int, maxPath int) error
	InitSlicedFindPath(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter) error
	UpdateSlicedFindPath(maxIter int, doneIters *int) error
	FinalizeSlicedFindPath(path []PolyRef, npath *int, maxPath int) error
	FinalizeSlicedFindPathPartial(existing []PolyRef, existingSize int, path []PolyRef, npath *int, maxPath int) error
	GetAttachedNavMesh() *NavMesh
	ClosestPointOnPolyBoundary(ref PolyRef, pos [3]float32, result *[3]float32) error
	FindLocalNeighbourhood(startRef PolyRef, pos [3]float32, radius float32, filter *QueryFilter, resultRefs []PolyRef, resultParent []PolyRef, nresult *int, maxResult int) error
	GetPolyWallSegments(ref PolyRef, filter *QueryFilter, segments []float32, segRefs []PolyRef, nsegments *int, maxSegments int) error
}

// PolyRef is a polygon reference, re-exported from the detour package.
type PolyRef = detour.PolyRef

// NavMesh is a navigation mesh, re-exported from the detour package.
type NavMesh = detour.NavMesh

// QueryFilter is a query filter, re-exported from the detour package.
type QueryFilter = detour.QueryFilter
