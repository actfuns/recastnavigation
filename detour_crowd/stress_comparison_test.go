package detour_crowd

import (
	"math/rand"
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

// expectedResults holds the C++ reference values for comparison.
type expectedResult struct {
	endRef    detour.PolyRef
	pathCount int
	corrCount int
}

func captureExpected() expectedResult {
	return expectedResult{
		endRef:    4173, // poly 77
		pathCount: 15,
		corrCount: 13,
	}
}

func TestStressDetailedComparison(t *testing.T) {
	expected := captureExpected()
	const iterations = 1000

	for i := 0; i < iterations; i++ {
		navQuery := createTestNavMeshQueryEx(t, 1, 512)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		for j := 0; j < detour.MaxAreas; j++ {
			filter.AreaCost[j] = 1.0
		}

		halfExtents := [3]float32{2, 4, 2}
		startPos := [3]float32{1, 0, 1}
		endPos := [3]float32{8, 0, 8}

		startRef, startNearest, err := navQuery.FindNearestPoly(startPos, halfExtents, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("iter %d: FindNearestPoly start failed: ref=%d err=%v", i, startRef, err)
		}
		endRef, endNearest, err := navQuery.FindNearestPoly(endPos, halfExtents, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("iter %d: FindNearestPoly end failed: ref=%d err=%v", i, endRef, err)
		}

		if endRef != expected.endRef {
			t.Fatalf("iter %d: endRef mismatch: got %d, want %d", i, endRef, expected.endRef)
		}

		pathBuf := make([]detour.PolyRef, 256)
		npath, err := navQuery.FindPath(startRef, endRef, startNearest, endNearest, filter, pathBuf)
		if err != nil || npath == 0 {
			t.Fatalf("iter %d: FindPath failed err=%v npath=%d", i, err, npath)
		}
		path := pathBuf[:npath]
		if len(path) != expected.pathCount {
			t.Fatalf("iter %d: path count mismatch: got %d, want %d\n  got: %v", i, len(path), expected.pathCount, path)
		}
		if path[len(path)-1] != endRef {
			t.Fatalf("iter %d: last path poly mismatch: got %d, want %d", i, path[len(path)-1], endRef)
		}

		spBuf := make([]detour.PolyRef, 256)
		spCnt, _ := navQuery.FindPath(startRef, endRef, startNearest, endNearest, filter, spBuf)
		straightPath := make([]float32, 256*3)
		straightPathFlags := make([]uint8, 256)
		straightPathRefs := make([]detour.PolyRef, 256)
		nstraight, _ := navQuery.FindStraightPath(startNearest, endNearest, spBuf, spCnt,
			straightPath, straightPathFlags, straightPathRefs, 256, 0)
		if nstraight < 2 {
			t.Fatalf("iter %d: straight path too short: %d", i, nstraight)
		}

		c := NewCrowd()
		if !c.Init(16, 0.6, navQuery) {
			t.Fatalf("iter %d: Crowd.Init failed", i)
		}

		cf := detour.NewQueryFilter()
		cf.IncludeFlags = 0xffff
		for j := 0; j < detour.MaxAreas; j++ {
			cf.AreaCost[j] = 1.0
		}
		c.SetFilter(0, cf)

		params := &CrowdAgentParams{
			Radius:                0.5,
			Height:                2.0,
			MaxAcceleration:       8.0,
			MaxSpeed:              3.5,
			CollisionQueryRange:   6,
			PathOptimizationRange: 10,
			SeparationWeight:      3.0,
			UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo,
			ObstacleAvoidanceType: 0,
			QueryFilterType:       0,
		}

		idx := c.AddAgent(startNearest, params)
		if idx < 0 {
			t.Fatalf("iter %d: AddAgent failed", i)
		}

		if !c.RequestMoveTarget(idx, endRef, endNearest) {
			t.Fatalf("iter %d: RequestMoveTarget failed", i)
		}

		const dt = 1.0 / 60.0
		for f := 1; f <= 30; f++ {
			c.Update(dt, nil)
		}

		ag := c.GetAgent(idx)
		corrCount := ag.corridor.GetPathCount()
		if corrCount != expected.corrCount {
			t.Fatalf("iter %d: corridor count mismatch: got %d, want %d", i, corrCount, expected.corrCount)
		}

		corrPath := ag.corridor.GetPath()
		if corrPath[corrCount-1] != endRef {
			t.Fatalf("iter %d: last corridor poly mismatch: got %d, want %d", i, corrPath[corrCount-1], endRef)
		}
	}
}

func TestStressRandomPositions(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	const iterations = 500

	for i := 0; i < iterations; i++ {
		navQuery := createTestNavMeshQueryEx(t, 1, 512)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		for j := 0; j < detour.MaxAreas; j++ {
			filter.AreaCost[j] = 1.0
		}

		halfExtents := [3]float32{2, 4, 2}

		sx := float32(rng.Float64()*8 + 0.5)
		sz := float32(rng.Float64()*8 + 0.5)
		ex := float32(rng.Float64()*8 + 0.5)
		ez := float32(rng.Float64()*8 + 0.5)

		startPos := [3]float32{sx, 0, sz}
		endPos := [3]float32{ex, 0, ez}

		startRef, startNearest, err := navQuery.FindNearestPoly(startPos, halfExtents, filter)
		if err != nil || startRef == 0 {
			continue
		}
		endRef, endNearest, err := navQuery.FindNearestPoly(endPos, halfExtents, filter)
		if err != nil || endRef == 0 {
			continue
		}

		if startRef == endRef {
			continue
		}

		pathBuf := make([]detour.PolyRef, 256)
		npath, err := navQuery.FindPath(startRef, endRef, startNearest, endNearest, filter, pathBuf)
		if err != nil || npath == 0 {
			t.Fatalf("iter %d pos (%.1f,%.1f)->(%.1f,%.1f): FindPath failed err=%v npath=%d",
				i, sx, sz, ex, ez, err, npath)
		}
		path := pathBuf[:npath]

		if path[0] != startRef {
			t.Fatalf("iter %d: first path poly %d != startRef %d", i, path[0], startRef)
		}
		if path[npath-1] != endRef {
			t.Fatalf("iter %d: last path poly %d != endRef %d", i, path[npath-1], endRef)
		}

		c := NewCrowd()
		if !c.Init(16, 0.6, navQuery) {
			t.Fatalf("iter %d: Crowd.Init failed", i)
		}

		cf := detour.NewQueryFilter()
		cf.IncludeFlags = 0xffff
		for j := 0; j < detour.MaxAreas; j++ {
			cf.AreaCost[j] = 1.0
		}
		c.SetFilter(0, cf)

		params := &CrowdAgentParams{
			Radius:                0.5,
			Height:                2.0,
			MaxAcceleration:       8.0,
			MaxSpeed:              3.5,
			CollisionQueryRange:   6,
			PathOptimizationRange: 10,
			SeparationWeight:      3.0,
			UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo,
			ObstacleAvoidanceType: 0,
			QueryFilterType:       0,
		}

		idx := c.AddAgent(startNearest, params)
		if idx < 0 {
			t.Fatalf("iter %d: AddAgent failed", i)
		}

		if !c.RequestMoveTarget(idx, endRef, endNearest) {
			t.Fatalf("iter %d: RequestMoveTarget failed", i)
		}

		const dt = 1.0 / 60.0
		for f := 1; f <= 30; f++ {
			c.Update(dt, nil)
		}
	}
}

func TestStressMultiAgent(t *testing.T) {
	rng := rand.New(rand.NewSource(123))
	const iterations = 200

	for i := 0; i < iterations; i++ {
		navQuery := createTestNavMeshQueryEx(t, 1, 512)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		for j := 0; j < detour.MaxAreas; j++ {
			filter.AreaCost[j] = 1.0
		}
		halfExtents := [3]float32{2, 4, 2}

		c := NewCrowd()
		if !c.Init(16, 0.6, navQuery) {
			t.Fatalf("iter %d: Crowd.Init failed", i)
		}

		cf := detour.NewQueryFilter()
		cf.IncludeFlags = 0xffff
		for j := 0; j < detour.MaxAreas; j++ {
			cf.AreaCost[j] = 1.0
		}
		c.SetFilter(0, cf)

		params := &CrowdAgentParams{
			Radius:                0.5,
			Height:                2.0,
			MaxAcceleration:       8.0,
			MaxSpeed:              3.5,
			CollisionQueryRange:   6,
			PathOptimizationRange: 10,
			SeparationWeight:      3.0,
			UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo,
			ObstacleAvoidanceType: 0,
			QueryFilterType:       0,
		}

		nagents := 0
		for a := 0; a < 3; a++ {
			sx := float32(rng.Float64()*8 + 0.5)
			sz := float32(rng.Float64()*8 + 0.5)
			ex := float32(rng.Float64()*8 + 0.5)
			ez := float32(rng.Float64()*8 + 0.5)

			startPos := [3]float32{sx, 0, sz}
			endPos := [3]float32{ex, 0, ez}

			sRef, sNear, err := navQuery.FindNearestPoly(startPos, halfExtents, filter)
			if err != nil || sRef == 0 {
				continue
			}
			eRef, eNear, err := navQuery.FindNearestPoly(endPos, halfExtents, filter)
			if err != nil || eRef == 0 {
				continue
			}
			if sRef == eRef {
				continue
			}

			idx := c.AddAgent(sNear, params)
			if idx < 0 {
				continue
			}
			if !c.RequestMoveTarget(idx, eRef, eNear) {
				continue
			}
			nagents++
		}

		if nagents == 0 {
			continue
		}

		const dt = 1.0 / 60.0
		for f := 1; f <= 60; f++ {
			c.Update(dt, nil)
		}
	}
}

func BenchmarkGoCrowdUpdate(b *testing.B) {
	navQuery := createTestNavMeshQueryEx(b, 1, 65535)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	for j := 0; j < detour.MaxAreas; j++ {
		filter.AreaCost[j] = 1.0
	}

	c := NewCrowd()
	c.Init(128, 0.6, navQuery)

	cf := detour.NewQueryFilter()
	cf.IncludeFlags = 0xffff
	for j := 0; j < detour.MaxAreas; j++ {
		cf.AreaCost[j] = 1.0
	}
	c.SetFilter(0, cf)

	params := &CrowdAgentParams{
		Radius:                0.5,
		Height:                2.0,
		MaxAcceleration:       8.0,
		MaxSpeed:              3.5,
		CollisionQueryRange:   6,
		PathOptimizationRange: 10,
		SeparationWeight:      3.0,
		UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo | CrowdObstacleAvoidance | CrowdSeparation,
		ObstacleAvoidanceType: 0,
		QueryFilterType:       0,
	}

	halfExt := [3]float32{2, 4, 2}
	added := 0
	for i := 0; i < 10; i++ {
		x := 1.0 + float32(i)
		z := 1.0 + float32(i%9)
		startPos := [3]float32{x, 0, z}
		sRef, sNear, _ := navQuery.FindNearestPoly(startPos, halfExt, filter)
		if sRef == 0 {
			continue
		}
		idx := c.AddAgent(sNear, params)
		if idx >= 0 {
			added++
			tx := 9.0 - float32(i)
			tz := 9.0 - float32(i%9)
			endPos := [3]float32{tx, 0, tz}
			eRef, eNear, _ := navQuery.FindNearestPoly(endPos, halfExt, filter)
			if eRef != 0 {
				c.RequestMoveTarget(idx, eRef, eNear)
			}
		}
	}
	_ = added

	b.ResetTimer()
	const dt = 1.0 / 60.0
	for i := 0; i < b.N; i++ {
		c.Update(dt, nil)
	}
}

func BenchmarkGoAddRemoveAgent(b *testing.B) {
	navQuery := createTestNavMeshQueryEx(b, 1, 65535)

	c := NewCrowd()
	c.Init(128, 0.6, navQuery)
	cf := detour.NewQueryFilter()
	cf.IncludeFlags = 0xffff
	c.SetFilter(0, cf)

	params := &CrowdAgentParams{
		Radius:          0.5,
		Height:          2.0,
		MaxAcceleration: 8.0,
		MaxSpeed:        3.5,
		UpdateFlags:     CrowdOptimizeVis | CrowdOptimizeTopo,
	}

	pos := [3]float32{1, 0, 1}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		idx := c.AddAgent(pos, params)
		if idx >= 0 {
			c.RemoveAgent(idx)
		}
	}
}

func BenchmarkGoRequestAndPath(b *testing.B) {
	navQuery := createTestNavMeshQueryEx(b, 1, 65535)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	for j := 0; j < detour.MaxAreas; j++ {
		filter.AreaCost[j] = 1.0
	}

	halfExt := [3]float32{2, 4, 2}
	endPos := [3]float32{8, 0, 8}
	_, sNear, _ := navQuery.FindNearestPoly([3]float32{1, 0, 1}, halfExt, filter)
	eRef, eNear, _ := navQuery.FindNearestPoly(endPos, halfExt, filter)

	c := NewCrowd()
	c.Init(16, 0.6, navQuery)
	cf := detour.NewQueryFilter()
	cf.IncludeFlags = 0xffff
	for j := 0; j < detour.MaxAreas; j++ {
		cf.AreaCost[j] = 1.0
	}
	c.SetFilter(0, cf)

	params := &CrowdAgentParams{
		Radius:                0.5,
		Height:                2.0,
		MaxAcceleration:       8.0,
		MaxSpeed:              3.5,
		CollisionQueryRange:   6,
		PathOptimizationRange: 10,
		SeparationWeight:      3.0,
		UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo,
		ObstacleAvoidanceType: 0,
		QueryFilterType:       0,
	}

	b.ResetTimer()
	const dt = 1.0 / 60.0
	for i := 0; i < b.N; i++ {
		idx := c.AddAgent(sNear, params)
		if idx < 0 {
			continue
		}
		c.RequestMoveTarget(idx, eRef, eNear)
		for j := 0; j < 5; j++ {
			c.Update(dt, nil)
		}
		c.RemoveAgent(idx)
	}
}
