package detour_crowd

const pathQueueMaxQueue = 8

// PathQueueRef is a reference to a path query in the queue.
type PathQueueRef uint32

// PathQueue manages asynchronous pathfinding requests.
type PathQueue struct {
	queue       [pathQueueMaxQueue]pathQuery
	nextHandle  PathQueueRef
	maxPathSize int
	queueHead   int
	navquery    NavMeshQueryInterface
}

type pathQuery struct {
	ref              PathQueueRef
	startPos, endPos [3]float32
	startRef, endRef PolyRef
	path             []PolyRef
	npath            int
	status           Status
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
func (q *PathQueue) Init(maxPathSize int, maxSearchNodeCount int, nav NavMeshQueryInterface) bool {
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
	iterCount := maxIters

	for i := 0; i < pathQueueMaxQueue; i++ {
		pq := &q.queue[q.queueHead%pathQueueMaxQueue]

		// Skip inactive requests.
		if pq.ref == 0 {
			q.queueHead++
			continue
		}

		// Handle completed request.
		if StatusSucceed(pq.status) || StatusFailed(pq.status) {
			pq.keepAlive++
			if pq.keepAlive > maxKeepAlive {
				pq.ref = 0
				pq.status = 0
			}
			q.queueHead++
			continue
		}

		// Handle query start.
		if pq.status == 0 {
			pq.status = q.navquery.InitSlicedFindPath(pq.startRef, pq.endRef, pq.startPos, pq.endPos, pq.filter)
		}

		// Handle query in progress.
		if StatusInProgressFlag(pq.status) {
			var iters int
			pq.status = q.navquery.UpdateSlicedFindPath(iterCount, &iters)
			iterCount -= iters
		}

		if StatusSucceed(pq.status) {
			pq.status = q.navquery.FinalizeSlicedFindPath(pq.path, &pq.npath, q.maxPathSize)
		}

		if iterCount <= 0 {
			break
		}

		q.queueHead++
	}
}

// Request submits a new pathfinding request.
func (q *PathQueue) Request(startRef, endRef PolyRef, startPos, endPos *[3]float32, filter *QueryFilter) PathQueueRef {
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
	pq.startPos = *startPos
	pq.startRef = startRef
	pq.endPos = *endPos
	pq.endRef = endRef
	pq.status = 0
	pq.npath = 0
	pq.filter = filter
	pq.keepAlive = 0

	return ref
}

// GetRequestStatus gets the status of a pathfinding request.
func (q *PathQueue) GetRequestStatus(ref PathQueueRef) Status {
	for i := 0; i < pathQueueMaxQueue; i++ {
		if q.queue[i].ref == ref {
			return q.queue[i].status
		}
	}
	return StatusFailure
}

// GetPathResult gets the result of a completed pathfinding request.
func (q *PathQueue) GetPathResult(ref PathQueueRef, path []PolyRef, pathSize *int, maxPath int) Status {
	for i := 0; i < pathQueueMaxQueue; i++ {
		if q.queue[i].ref == ref {
			pq := &q.queue[i]
			details := pq.status & StatusDetailMask
			// Free request for reuse.
			pq.ref = 0
			pq.status = 0
			// Copy path
			n := recastMin(pq.npath, maxPath)
			copy(path[:n], pq.path[:n])
			*pathSize = n
			return details | StatusSuccess
		}
	}
	return StatusFailure
}

// GetNavQuery returns the navigation mesh query used by the path queue.
func (q *PathQueue) GetNavQuery() NavMeshQueryInterface {
	return q.navquery
}
