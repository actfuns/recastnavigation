package detour_crowd

import "github.com/actfuns/recastnavigation/detour"

// NavMeshQueryInterface defines the subset of dtNavMeshQuery used by the crowd.
// This allows the crowd module to work with any implementation of the navigation
// mesh query.
type NavMeshQueryInterface interface {
	FindNearestPoly(pos [3]float32, halfExtents [3]float32, filter *QueryFilter) (PolyRef, [3]float32, error)
	IsValidPolyRef(ref PolyRef, filter *QueryFilter) bool
	MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result *[3]float32, visited []PolyRef, nvisited *int, maxVisited int) Status
	GetPolyHeight(ref PolyRef, pos [3]float32, height *float32) Status
	ClosestPointOnPoly(ref PolyRef, pos [3]float32, result *[3]float32, posOverPoly *bool) Status
	FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, cornerVerts []float32, cornerFlags []uint8, cornerPolys []PolyRef, ncorners *int, maxCorners int) Status
	Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, t *float32, hitNormal *[3]float32, path []PolyRef, npath *int, maxPath int) Status
	InitSlicedFindPath(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter) Status
	UpdateSlicedFindPath(maxIter int, doneIters *int) Status
	FinalizeSlicedFindPath(path []PolyRef, npath *int, maxPath int) Status
	FinalizeSlicedFindPathPartial(existing []PolyRef, existingSize int, path []PolyRef, npath *int, maxPath int) Status
	GetAttachedNavMesh() *NavMesh
	ClosestPointOnPolyBoundary(ref PolyRef, pos [3]float32, result *[3]float32) Status
	FindLocalNeighbourhood(startRef PolyRef, pos [3]float32, radius float32, filter *QueryFilter, resultRefs []PolyRef, resultParent []PolyRef, nresult *int, maxResult int) Status
	GetPolyWallSegments(ref PolyRef, filter *QueryFilter, segments []float32, segRefs []PolyRef, nsegments *int, maxSegments int) Status
}

// PolyRef is a polygon reference, re-exported from the detour package.
type PolyRef = detour.PolyRef

// NavMesh is a navigation mesh, re-exported from the detour package.
type NavMesh = detour.NavMesh

// QueryFilter is a query filter, re-exported from the detour package.
type QueryFilter = detour.QueryFilter

// Status is a status value, re-exported from the detour package.
type Status = detour.Status

// NavMeshStatus constants
const (
	StatusFailure          Status = 1 << 31
	StatusSuccess          Status = 1 << 30
	StatusInProgress       Status = 1 << 29
	StatusDetailMask       Status = 0x0ffffff
	StatusWrongMagic       Status = 1 << 0
	StatusWrongVersion     Status = 1 << 1
	StatusOutOfMemory      Status = 1 << 2
	StatusInvalidParam     Status = 1 << 3
	StatusBufferTooSmall   Status = 1 << 4
	StatusOutOfNodes       Status = 1 << 5
	StatusPartialResult    Status = 1 << 6
	StatusAlreadInProgress Status = 1 << 7
	StatusDetailNone       Status = 0
)

// NavMeshStatus helper functions
func StatusSucceed(status Status) bool {
	return (status & StatusSuccess) != 0
}

func StatusFailed(status Status) bool {
	return (status & StatusFailure) != 0
}

func StatusInProgressFlag(status Status) bool {
	return (status & StatusInProgress) != 0
}

func StatusDetail(status Status, detail Status) bool {
	return (status & StatusDetailMask) == detail
}

// Vec3 is a 3D vector type used throughout the crowd module.
type Vec3 = [3]float32
