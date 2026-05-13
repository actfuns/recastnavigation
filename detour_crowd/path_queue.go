package detour_crowd

import (
	"errors"

	"github.com/actfuns/recastnavigation/detour"
)

const pathQueueMaxQueue = 8

// PathQueueRef is a reference to a path query in the queue.
type PathQueueRef uint32

// PathQueue manages asynchronous pathfinding requests.
type PathQueue struct {
	queue       [pathQueueMaxQueue]pathQuery
	nextHandle  PathQueueRef
	maxPathSize int
	queueHead   int
	navquery    *detour.NavMeshQuery
}

type pathQuery struct {
	ref              PathQueueRef
	startPos, endPos [3]float32
	startRef, endRef PolyRef
	path             []PolyRef
	npath            int
	err              error
	keepAlive        int
	filter           *QueryFilter
}

// NewPathQueue creates a new path queue.
func NewPathQueue() *PathQueue {
	q := &PathQueue{}
	q.nextHandle = 1
	for i := 0; i < pathQueueMaxQueue; i++ {
		q.queue[i].ref = 0
	}
	return q
}

// Init initializes the path queue.
func (q *PathQueue) Init(maxPathSize int, maxSearchNodeCount int, nav *detour.NavMeshQuery) bool {
	q.maxPathSize = maxPathSize
	for i := 0; i < pathQueueMaxQueue; i++ {
		q.queue[i].ref = 0
		q.queue[i].path = make([]PolyRef, maxPathSize)
	}
	q.queueHead = 0
	q.navquery = nav
	return true
}

// Update updates path requests in the queue.
func (q *PathQueue) Update(maxIters int) {
	const maxKeepAlive = 2

	iterCount := 0
	for i := 0; i < pathQueueMaxQueue; i++ {
		pq := &q.queue[q.queueHead%pathQueueMaxQueue]

		// Skip inactive requests.
		if pq.ref == 0 {
			q.queueHead++
			continue
		}

		// Handle completed/failed request (terminal state, not "in progress").
		// Terminal states: non-ErrInProgress error (failure/partial), or
		// nil error with npath > 0 (successfully completed).
		if (pq.err != nil && pq.err != detour.ErrInProgress) || (pq.err == nil && pq.npath > 0) {
			pq.keepAlive++
			if pq.keepAlive > maxKeepAlive {
				pq.ref = 0
				pq.err = nil
				pq.npath = 0
			}
			q.queueHead++
			continue
		}

		// Handle query start.
		if pq.err == nil {
			pq.err = q.navquery.FindPathSliced(pq.startRef, pq.endRef, pq.startPos, pq.endPos, pq.filter, 0)
		}

		// Handle query in progress.
		if errors.Is(pq.err, detour.ErrInProgress) {
			itersLeft := maxIters - iterCount
			if itersLeft <= 0 {
				break
			}
			pq.err = q.navquery.UpdateSlicedPath(itersLeft)
			if errors.Is(pq.err, detour.ErrInProgress) {
				iterCount++
			}
		}

		// Handle query complete.
		if pq.err == nil {
			pq.npath, pq.err = q.navquery.GetPathFromSlicedPath(pq.path, q.maxPathSize)
		}

		q.queueHead++
	}
}

// Request submits a new pathfinding request.
func (q *PathQueue) Request(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter) PathQueueRef {
	// Find empty slot
	slot := -1
	for i := 0; i < pathQueueMaxQueue; i++ {
		if q.queue[i].ref == 0 {
			slot = i
			break
		}
	}
	if slot == -1 {
		return 0
	}

	ref := q.nextHandle
	q.nextHandle++
	if q.nextHandle == 0 {
		q.nextHandle++
	}

	pq := &q.queue[slot]
	pq.ref = ref
	pq.startPos = startPos
	pq.startRef = startRef
	pq.endPos = endPos
	pq.endRef = endRef
	pq.err = nil
	pq.npath = 0
	pq.filter = filter
	pq.keepAlive = 0

	return ref
}

// GetRequestErr gets the err of a pathfinding request.
func (q *PathQueue) GetRequestErr(ref PathQueueRef) error {
	for i := 0; i < pathQueueMaxQueue; i++ {
		if q.queue[i].ref == ref {
			return q.queue[i].err
		}
	}
	return detour.ErrFailure
}

// GetPathResult gets the result of a completed pathfinding request.
func (q *PathQueue) GetPathResult(ref PathQueueRef, path []PolyRef, maxPath int) (int, error) {
	for i := 0; i < pathQueueMaxQueue; i++ {
		if q.queue[i].ref == ref {
			pq := &q.queue[i]
			details := errors.Is(pq.err, detour.ErrPartialResult)
			// Free request for reuse.
			pq.ref = 0
			pq.err = nil
			// Copy path
			n := recastMin(pq.npath, maxPath)
			copy(path[:n], pq.path[:n])
			if details {
				return n, detour.ErrPartialResult
			}
			return n, nil
		}
	}
	return 0, detour.ErrFailure
}

// GetNavQuery returns the navigation mesh query used by the path queue.
func (q *PathQueue) GetNavQuery() *detour.NavMeshQuery {
	return q.navquery
}
