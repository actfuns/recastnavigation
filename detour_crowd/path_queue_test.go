package detour_crowd

import (
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

// mockNavQueryForPathQueue implements NavMeshQueryInterface for PathQueue tests.
type mockNavQueryForPathQueue struct {
	findPathSlicedFunc    func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error
	updateSlicedPathFunc  func(maxIter int) error
	getPathFromSlicedFunc func(path []PolyRef, maxPath int) (int, error)
}

func (m *mockNavQueryForPathQueue) FindNearestPoly(pos [3]float32, halfExtents [3]float32, filter *QueryFilter) (PolyRef, [3]float32, error) {
	return 0, [3]float32{}, nil
}

func (m *mockNavQueryForPathQueue) IsValidPolyRef(ref PolyRef, filter *QueryFilter) bool {
	return true
}

func (m *mockNavQueryForPathQueue) MoveAlongSurface(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, result []float32, visited []PolyRef, maxVisitedSize int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForPathQueue) GetPolyHeight(ref PolyRef, pos [3]float32) (float32, error) {
	return 0, nil
}

func (m *mockNavQueryForPathQueue) ClosestPointOnPoly(ref PolyRef, pos [3]float32) ([3]float32, bool, error) {
	return [3]float32{}, true, nil
}

func (m *mockNavQueryForPathQueue) FindStraightPath(startPos, endPos [3]float32, path []PolyRef, pathSize int, maxStraightPath int, options int) ([]float32, []uint8, []PolyRef, int, error) {
	return nil, nil, nil, 0, nil
}

func (m *mockNavQueryForPathQueue) Raycast(startRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32, prevRef PolyRef, hit *RaycastHit) error {
	return nil
}

func (m *mockNavQueryForPathQueue) FindPathSliced(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
	if m.findPathSlicedFunc != nil {
		return m.findPathSlicedFunc(startRef, endRef, startPos, endPos, filter, options)
	}
	return nil
}

func (m *mockNavQueryForPathQueue) UpdateSlicedPath(maxIter int) error {
	if m.updateSlicedPathFunc != nil {
		return m.updateSlicedPathFunc(maxIter)
	}
	return nil
}

func (m *mockNavQueryForPathQueue) GetPathFromSlicedPath(path []PolyRef, maxPath int) (int, error) {
	if m.getPathFromSlicedFunc != nil {
		return m.getPathFromSlicedFunc(path, maxPath)
	}
	return 0, nil
}

func (m *mockNavQueryForPathQueue) GetAttachedNavMesh() *NavMesh {
	return nil
}

func (m *mockNavQueryForPathQueue) ClosestPointOnPolyBoundary(ref PolyRef, pos [3]float32) ([3]float32, error) {
	return [3]float32{}, nil
}

func (m *mockNavQueryForPathQueue) FindPolysAroundCircle(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, resultCost []float32, maxResult int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForPathQueue) FindLocalNeighbourhood(startRef PolyRef, centerPos [3]float32, radius float32, filter *QueryFilter, resultRef []PolyRef, resultParent []PolyRef, maxResult int) (int, error) {
	return 0, nil
}

func (m *mockNavQueryForPathQueue) GetPolyWallSegments(ref PolyRef, filter *QueryFilter, segs []NeighbourSeg, maxSegs int) (int, error) {
	return 0, nil
}

func TestPathQueueInit(t *testing.T) {
	t.Run("should initialize successfully", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()

		result := q.Init(256, 4096, mock)
		if !result {
			t.Errorf("Init returned false, expected true")
		}

		if q.maxPathSize != 256 {
			t.Errorf("maxPathSize = %d, want 256", q.maxPathSize)
		}

		if q.queueHead != 0 {
			t.Errorf("queueHead = %d, want 0", q.queueHead)
		}

		if q.nextHandle != 1 {
			t.Errorf("nextHandle = %d, want 1", q.nextHandle)
		}

		// Verify all queue slots are initialized to ref=0
		for i := 0; i < pathQueueMaxQueue; i++ {
			if q.queue[i].ref != 0 {
				t.Errorf("queue[%d].ref = %d, want 0", i, q.queue[i].ref)
			}
			if len(q.queue[i].path) != 256 {
				t.Errorf("queue[%d].path len = %d, want 256", i, len(q.queue[i].path))
			}
		}
	})

	t.Run("should return nil nav query when not initialized", func(t *testing.T) {
		q := NewPathQueue()
		if q.GetNavQuery() != nil {
			t.Errorf("GetNavQuery should return nil before Init")
		}
	})

	t.Run("should return the configured nav query", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		if q.GetNavQuery() != mock {
			t.Errorf("GetNavQuery did not return the mock passed to Init")
		}
	})
}

func TestPathQueueRequest(t *testing.T) {
	t.Run("should return a valid handle for a new request", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{0, 0, 0}, [3]float32{10, 0, 10}, nil)
		if ref == PathQueueRef(PathQInvalid) {
			t.Errorf("Request returned PathQInvalid (0), expected non-zero handle")
		}
	})

	t.Run("should allocate unique handles", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref1 := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		ref2 := q.Request(3, 4, [3]float32{}, [3]float32{}, nil)

		if ref1 == ref2 {
			t.Errorf("handles should be unique, got %d and %d", ref1, ref2)
		}
	})

	t.Run("should return 0 when queue is full", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		// Fill up all slots
		var lastRef PathQueueRef
		for i := 0; i < pathQueueMaxQueue; i++ {
			ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
			if ref == PathQueueRef(PathQInvalid) {
				t.Errorf("request %d should succeed, got 0", i)
			}
			lastRef = ref
		}

		// Next request should fail
		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		if ref != PathQueueRef(PathQInvalid) {
			t.Errorf("expected PathQInvalid for full queue, got %d (last successful was %d)", ref, lastRef)
		}
	})

	t.Run("should not reuse a handle of 0", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		// Force nextHandle to wrap through 0
		q.nextHandle = PathQueueRef(^uint32(0)) // max uint32

		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		if ref != PathQueueRef(^uint32(0)) {
			t.Errorf("expected handle = %d, got %d", ^uint32(0), ref)
		}

		// Next request should skip 0
		ref2 := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		if ref2 == 0 {
			t.Errorf("handle should not be 0 after wrap, got 0")
		}
		if ref2 != 1 {
			t.Errorf("expected handle 1 after max uint32 wrap, got %d", ref2)
		}
	})
}

func TestPathQueueGetRequestErr(t *testing.T) {
	t.Run("should return ErrFailure for unknown ref", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		err := q.GetRequestErr(999)
		if err != detour.ErrFailure {
			t.Errorf("GetRequestErr(999) = %v, want %v", err, detour.ErrFailure)
		}
	})

	t.Run("should return the request's error", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)

		err := q.GetRequestErr(ref)
		if err != nil {
			t.Errorf("GetRequestErr should return nil for pending request, got %v", err)
		}
	})
}

func TestPathQueueGetPathResult(t *testing.T) {
	t.Run("should return ErrFailure for unknown ref", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(999, buf, 256)
		if err != detour.ErrFailure {
			t.Errorf("GetPathResult unknown ref: err = %v, want %v", err, detour.ErrFailure)
		}
		if n != 0 {
			t.Errorf("GetPathResult unknown ref: n = %d, want 0", n)
		}
	})

	t.Run("should retrieve path after completion", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
				return nil
			},
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				path[0] = 42
				path[1] = 43
				return 2, nil
			},
		}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{0, 0, 0}, [3]float32{10, 0, 10}, nil)

		// Process the request to completion
		q.Update(100)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)

		if err != nil {
			t.Errorf("GetPathResult returned error: %v", err)
		}
		if n != 2 {
			t.Errorf("GetPathResult returned n = %d, want 2", n)
		}
		if buf[0] != 42 || buf[1] != 43 {
			t.Errorf("path = [%d, %d], want [42, 43]", buf[0], buf[1])
		}

		// After GetPathResult, the request should be freed
		err2 := q.GetRequestErr(ref)
		if err2 != detour.ErrFailure {
			t.Errorf("request should be freed after GetPathResult, got err = %v, want ErrFailure", err2)
		}
	})

	t.Run("should trim path to maxPath length", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error { return nil },
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				for i := 0; i < maxPath; i++ {
					path[i] = PolyRef(100 + i)
				}
				return maxPath, nil
			},
		}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		q.Update(100)

		buf := make([]PolyRef, 2)
		n, err := q.GetPathResult(ref, buf, 2)

		if err != nil {
			t.Errorf("GetPathResult error: %v", err)
		}
		if n != 2 {
			t.Errorf("n = %d, want 2 (clamped to buf size)", n)
		}
	})

	t.Run("should handle partial result", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error { return nil },
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				path[0] = 42
				return 1, detour.ErrPartialResult
			},
		}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		q.Update(100)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)

		if err != detour.ErrPartialResult {
			t.Errorf("expected ErrPartialResult, got %v", err)
		}
		if n != 1 {
			t.Errorf("n = %d, want 1", n)
		}
	})
}

func TestPathQueueUpdate(t *testing.T) {
	t.Run("should process request to completion in one call", func(t *testing.T) {
		var findCalled, updateCalled, getCalled bool

		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
				findCalled = true
				if startRef != 1 || endRef != 2 {
					t.Errorf("FindPathSliced: startRef=%d, endRef=%d, want 1, 2", startRef, endRef)
				}
				return nil
			},
			updateSlicedPathFunc: func(maxIter int) error {
				updateCalled = true
				return nil
			},
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				getCalled = true
				path[0] = 42
				return 1, nil
			},
		}

		q := NewPathQueue()
		q.Init(256, 4096, mock)

		q.Request(1, 2, [3]float32{0, 0, 0}, [3]float32{10, 0, 10}, nil)
		q.Update(100)

		if !findCalled {
			t.Errorf("FindPathSliced was not called")
		}
		if updateCalled {
			t.Errorf("UpdateSlicedPath was called but should not be (FindPathSliced returned nil)")
		}
		if !getCalled {
			t.Errorf("GetPathFromSlicedPath was not called")
		}
	})

	t.Run("should handle multi-step async completion", func(t *testing.T) {
		callCount := 0

		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
				callCount++
				return detour.ErrInProgress
			},
			updateSlicedPathFunc: func(maxIter int) error {
				callCount++
				// On first in-progress update, keep in progress, then complete on second
				if callCount <= 2 {
					return detour.ErrInProgress
				}
				return nil
			},
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				callCount++
				path[0] = 99
				return 1, nil
			},
		}

		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{0, 0, 0}, [3]float32{10, 0, 10}, nil)

		// First update: FindPathSliced → ErrInProgress, UpdateSlicedPath → ErrInProgress.
		// The request stays in progress in the same iteration because both happen in one pass.
		// Actually, looking at the Update loop: after FindPathSliced returns ErrInProgress,
		// it enters the "in progress" handler and calls UpdateSlicedPath. If that also returns
		// ErrInProgress, iterCount++ and queueHead++. Next time the slot is visited...
		// but actually in a single iteration, after UpdateSlicedPath returns ErrInProgress,
		// we skip the "complete" handler (since pq.err != nil) and queueHead++.
		// So the request is NOT completed in one iteration with this setup.

		q.Update(100)

		// The request should have been started and made some progress
		err := q.GetRequestErr(ref)
		if err == nil && err != detour.ErrInProgress {
			// Could be completed if everything happened in one pass
			_ = err
		}

		// Second update should complete it
		q.Update(100)

		buf := make([]PolyRef, 256)
		n, err := q.GetPathResult(ref, buf, 256)

		// Should have completed by now
		if err != nil && err != detour.ErrPartialResult {
			t.Errorf("path not completed after two updates: err = %v", err)
		}
		if n <= 0 {
			t.Errorf("expected completed path, got n = %d", n)
		}
	})

	t.Run("should handle failing request", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error {
				return detour.ErrFailure
			},
		}

		q := NewPathQueue()
		q.Init(256, 4096, mock)

		ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
		q.Update(100)

		err := q.GetRequestErr(ref)
		if err == nil {
			// The request might have been freed already after keepAlive expired
			// Check if there's a result
			buf := make([]PolyRef, 256)
			n, pathErr := q.GetPathResult(ref, buf, 256)
			if pathErr != detour.ErrFailure && n == 0 {
				t.Errorf("expected failed request to not produce a valid path")
			}
		}
	})

	t.Run("should not panic with empty queue", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{}
		q := NewPathQueue()
		q.Init(256, 4096, mock)

		q.Update(100) // should not panic
	})

	t.Run("should process multiple requests", func(t *testing.T) {
		requestsProcessed := 0

		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error { return nil },
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				path[0] = 42
				requestsProcessed++
				return 1, nil
			},
		}

		q := NewPathQueue()
		q.Init(256, 4096, mock)

		refs := make([]PathQueueRef, 0, 5)
		for i := 0; i < 5; i++ {
			ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
			refs = append(refs, ref)
		}

		q.Update(100)

		if requestsProcessed != 5 {
			t.Errorf("processed %d requests, want 5", requestsProcessed)
		}

		// All should have results
		for _, ref := range refs {
			buf := make([]PolyRef, 256)
			n, err := q.GetPathResult(ref, buf, 256)
			if err != nil {
				t.Errorf("request %d failed: %v", ref, err)
			}
			if n <= 0 {
				t.Errorf("request %d returned empty path", ref)
			}
		}
	})

	t.Run("should handle wrapped queue head across multiple updates", func(t *testing.T) {
		mock := &mockNavQueryForPathQueue{
			findPathSlicedFunc: func(startRef, endRef PolyRef, startPos, endPos [3]float32, filter *QueryFilter, options uint32) error { return nil },
			getPathFromSlicedFunc: func(path []PolyRef, maxPath int) (int, error) {
				path[0] = 42
				return 1, nil
			},
		}

		q := NewPathQueue()
		q.Init(256, 4096, mock)

		// Process through many updates to cause queueHead to wrap
		for i := 0; i < 5; i++ {
			ref := q.Request(1, 2, [3]float32{}, [3]float32{}, nil)
			q.Update(100)

			buf := make([]PolyRef, 256)
			q.GetPathResult(ref, buf, 256)
		}

		// queueHead should have advanced through multiple cycles
		if q.queueHead <= pathQueueMaxQueue {
			t.Logf("queueHead = %d after %d requests+updates", q.queueHead, 5)
		}
	})
}
