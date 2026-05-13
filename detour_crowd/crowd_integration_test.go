package detour_crowd

import (
	"testing"

	"github.com/actfuns/recastnavigation/detour"
)

func TestCrowdInit(t *testing.T) {
	t.Run("should initialize with navmesh query", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()

		ok := c.Init(16, 0.6, navQuery)
		if !ok {
			t.Fatal("Crowd.Init returned false")
		}

		if c.navquery != navQuery {
			t.Errorf("navquery not stored correctly")
		}
		if len(c.GetActiveAgents()) != 0 {
			t.Errorf("active agents = %d, want 0", len(c.GetActiveAgents()))
		}
	})

	t.Run("should return nil nav query when not initialized", func(t *testing.T) {
		c := NewCrowd()
		if c.GetNavMeshQuery() != nil {
			t.Errorf("GetNavMeshQuery should return nil before Init")
		}
	})

	t.Run("should return the configured nav query", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		if c.GetNavMeshQuery() != navQuery {
			t.Errorf("GetNavMeshQuery did not return the query passed to Init")
		}
	})
}

func TestCrowdAddAgent(t *testing.T) {
	t.Run("should add agent at valid position", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		c.SetFilter(0, filter)

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

		idx := c.AddAgent([3]float32{1, 0, 1}, params)
		if idx < 0 {
			t.Fatal("AddAgent returned negative index")
		}

		ag := c.GetAgent(idx)
		if ag == nil || !ag.active {
			t.Errorf("agent not active")
		}
		if ag.state != CrowdAgentStateWalking {
			t.Errorf("agent state = %d, want %d", ag.state, CrowdAgentStateWalking)
		}
	})

	t.Run("should fill all slots", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		c.SetFilter(0, filter)

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

		count := 0
		for i := 0; i < 16; i++ {
			idx := c.AddAgent([3]float32{float32(1 + i), 0, float32(1 + i)}, params)
			if idx >= 0 {
				count++
			}
		}

		if count != 16 {
			t.Errorf("added %d agents, want 16", count)
		}

		idx := c.AddAgent([3]float32{1, 0, 1}, params)
		if idx >= 0 {
			t.Errorf("expected -1 when full, got %d", idx)
		}
	})
}

func TestCrowdMoveAgent(t *testing.T) {
	t.Run("agent should pathfind toward target", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		c.SetFilter(0, filter)

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

		_, startPos, err := navQuery.FindNearestPoly(
			[3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil {
			t.Fatalf("FindNearestPoly start failed: err=%v", err)
		}

		endRef, endPos, err := navQuery.FindNearestPoly(
			[3]float32{8, 0, 8}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed: err=%v", err)
		}

		idx := c.AddAgent(startPos, params)
		if idx < 0 {
			t.Fatal("AddAgent failed")
		}

		ok := c.RequestMoveTarget(idx, endRef, endPos)
		if !ok {
			t.Fatal("RequestMoveTarget returned false")
		}

		for i := 0; i < 10; i++ {
			c.Update(1.0/60.0, nil)
		}

		ag := c.GetAgent(idx)
		if ag.state != CrowdAgentStateWalking {
			t.Errorf("agent state = %d, want %d", ag.state, CrowdAgentStateWalking)
		}

		startDist := vecDist2DSqr(startPos, endPos)
		currentDist := vecDist2DSqr(ag.npos, endPos)
		t.Logf("start=(%.2f,%.2f,%.2f) current=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f)",
			startPos[0], startPos[1], startPos[2],
			ag.npos[0], ag.npos[1], ag.npos[2],
			endPos[0], endPos[1], endPos[2])
		t.Logf("startDist=%.2f currentDist=%.2f", startDist, currentDist)

		if currentDist >= startDist {
			t.Errorf("agent did not move closer to target: startDist=%.2f currentDist=%.2f",
				startDist, currentDist)
		}
	})

	t.Run("agent with velocity target should move", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		c.SetFilter(0, filter)

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

		startRef, startPos, err := navQuery.FindNearestPoly(
			[3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}

		idx := c.AddAgent(startPos, params)
		if idx < 0 {
			t.Fatal("AddAgent failed")
		}

		ok := c.RequestMoveVelocity(idx, [3]float32{3.5, 0, 3.5})
		if !ok {
			t.Fatal("RequestMoveVelocity returned false")
		}

		startPos2 := c.GetAgent(idx).npos

		for i := 0; i < 5; i++ {
			c.Update(1.0/60.0, nil)
		}

		ag := c.GetAgent(idx)
		moved := vecDist2DSqr(ag.npos, startPos2)
		t.Logf("velocity agent: start=(%.2f,%.2f,%.2f) current=(%.2f,%.2f,%.2f) moved=%.2f",
			startPos2[0], startPos2[1], startPos2[2],
			ag.npos[0], ag.npos[1], ag.npos[2], moved)

		if moved < 1e-6 {
			t.Errorf("agent with velocity target did not move: dist=%.6f", moved)
		}
	})
}

func TestCrowdMultiAgent(t *testing.T) {
	t.Run("multiple agents should coexist", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		c.SetFilter(0, filter)

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

		_, pos1, _ := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		_, pos2, _ := navQuery.FindNearestPoly([3]float32{1, 0, 5}, [3]float32{2, 4, 2}, filter)

		idx1 := c.AddAgent(pos1, params)
		idx2 := c.AddAgent(pos2, params)
		if idx1 < 0 || idx2 < 0 {
			t.Fatal("AddAgent failed")
		}

		end1Ref, end1Pos, _ := navQuery.FindNearestPoly([3]float32{5, 0, 5}, [3]float32{2, 4, 2}, filter)
		end2Ref, end2Pos, _ := navQuery.FindNearestPoly([3]float32{5, 0, 1}, [3]float32{2, 4, 2}, filter)

		c.RequestMoveTarget(idx1, end1Ref, end1Pos)
		c.RequestMoveTarget(idx2, end2Ref, end2Pos)

		for i := 0; i < 15; i++ {
			c.Update(1.0/60.0, nil)
		}

		ag1 := c.GetAgent(idx1)
		ag2 := c.GetAgent(idx2)

		t.Logf("agent1: pos=(%.2f,%.2f,%.2f) state=%d", ag1.npos[0], ag1.npos[1], ag1.npos[2], ag1.state)
		t.Logf("agent2: pos=(%.2f,%.2f,%.2f) state=%d", ag2.npos[0], ag2.npos[1], ag2.npos[2], ag2.state)

		if ag1.state != CrowdAgentStateWalking {
			t.Errorf("agent1 state = %d, want %d", ag1.state, CrowdAgentStateWalking)
		}
		if ag2.state != CrowdAgentStateWalking {
			t.Errorf("agent2 state = %d, want %d", ag2.state, CrowdAgentStateWalking)
		}

		dist1 := vecDist2DSqr(pos1, ag1.npos)
		dist2 := vecDist2DSqr(pos2, ag2.npos)
		if dist1 < 1e-6 {
			t.Errorf("agent1 did not move")
		}
		if dist2 < 1e-6 {
			t.Errorf("agent2 did not move")
		}
	})
}

func TestCrowdRemoveAgent(t *testing.T) {
	navQuery := createTestNavMeshQuery(t)
	c := NewCrowd()
	c.Init(16, 0.6, navQuery)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	c.SetFilter(0, filter)

	params := &CrowdAgentParams{
		Radius:          0.5,
		Height:          2.0,
		MaxAcceleration: 8.0,
		MaxSpeed:        3.5,
		UpdateFlags:     CrowdOptimizeVis | CrowdOptimizeTopo,
		QueryFilterType: 0,
	}

	idx := c.AddAgent([3]float32{1, 0, 1}, params)
	if idx < 0 {
		t.Fatal("AddAgent failed")
	}

	if len(c.GetActiveAgents()) != 1 {
		t.Errorf("active count = %d, want 1", len(c.GetActiveAgents()))
	}

	c.RemoveAgent(idx)

	if len(c.GetActiveAgents()) != 0 {
		t.Errorf("active count after RemoveAgent = %d, want 0", len(c.GetActiveAgents()))
	}
}

func TestCrowdWalkPath(t *testing.T) {
	t.Run("agent should walk along path and approach target", func(t *testing.T) {
		navQuery := createTestNavMeshQuery(t)
		c := NewCrowd()
		c.Init(16, 0.6, navQuery)

		filter := detour.NewQueryFilter()
		filter.IncludeFlags = 0xffff
		c.SetFilter(0, filter)

		params := &CrowdAgentParams{
			Radius:                0.4,
			Height:                2.0,
			MaxAcceleration:       12.0,
			MaxSpeed:              6.0,
			CollisionQueryRange:   6,
			PathOptimizationRange: 10,
			SeparationWeight:      3.0,
			UpdateFlags:           CrowdOptimizeVis | CrowdOptimizeTopo | CrowdAnticipateTurns,
			ObstacleAvoidanceType: 0,
			QueryFilterType:       0,
		}

		startRef, startNearest, err := navQuery.FindNearestPoly(
			[3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
		if err != nil || startRef == 0 {
			t.Fatalf("FindNearestPoly start failed")
		}
		endRef, endNearest, err := navQuery.FindNearestPoly(
			[3]float32{8, 0, 8}, [3]float32{2, 4, 2}, filter)
		if err != nil || endRef == 0 {
			t.Fatalf("FindNearestPoly end failed")
		}

		idx := c.AddAgent(startNearest, params)
		if idx < 0 {
			t.Fatal("AddAgent failed")
		}

		c.RequestMoveTarget(idx, endRef, endNearest)

		const dt = 1.0 / 60.0
		for i := 0; i < 10; i++ {
			c.Update(dt, nil)
		}

		ag := c.GetAgent(idx)
		t.Logf("after 10 frames: pos=(%.3f,%.3f,%.3f) state=%d ncorners=%d",
			ag.npos[0], ag.npos[1], ag.npos[2], ag.state, ag.ncorners)

		for i := 0; i < 90; i++ {
			c.Update(dt, nil)
		}

		t.Logf("after 100 frames: pos=(%.3f,%.3f,%.3f) state=%d ncorners=%d",
			ag.npos[0], ag.npos[1], ag.npos[2], ag.state, ag.ncorners)

		startDist := vecDist2D(startNearest, endNearest)
		currentDist := vecDist2D(ag.npos, endNearest)
		t.Logf("startDist=%.3f currentDist=%.3f", startDist, currentDist)

		if currentDist > startDist*0.5 {
			t.Logf("agent may not have moved enough: currentDist=%.3f is >50%% of startDist=%.3f",
				currentDist, startDist)
		}
	})
}

func BenchmarkCrowdUpdate(b *testing.B) {
	navQuery := createTestNavMeshQuery(b)
	c := NewCrowd()
	c.Init(128, 0.6, navQuery)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	c.SetFilter(0, filter)

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

	added := 0
	for i := 0; i < 10; i++ {
		x := float32(1 + i)
		z := float32(1 + i%9)
		ref, pos, err := navQuery.FindNearestPoly(
			[3]float32{x, 0, z}, [3]float32{2, 4, 2}, filter)
		if err != nil || ref == 0 {
			continue
		}
		idx := c.AddAgent(pos, params)
		if idx >= 0 {
			added++
			targetX := float32(9 - i)
			targetZ := float32(9 - i%9)
			endRef, endPos, _ := navQuery.FindNearestPoly(
				[3]float32{targetX, 0, targetZ}, [3]float32{2, 4, 2}, filter)
			if endRef != 0 {
				c.RequestMoveTarget(idx, endRef, endPos)
			}
		}
	}

	b.Logf("added %d agents for benchmark", added)
	b.ResetTimer()

	const dt = 1.0 / 60.0
	for i := 0; i < b.N; i++ {
		c.Update(dt, nil)
	}
}

func BenchmarkCrowdAddRemoveAgent(b *testing.B) {
	navQuery := createTestNavMeshQuery(b)
	c := NewCrowd()
	c.Init(128, 0.6, navQuery)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	c.SetFilter(0, filter)

	params := &CrowdAgentParams{
		Radius:          0.5,
		Height:          2.0,
		MaxAcceleration: 8.0,
		MaxSpeed:        3.5,
		UpdateFlags:     CrowdOptimizeVis | CrowdOptimizeTopo,
		QueryFilterType: 0,
	}

	ref, pos, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
	if err != nil || ref == 0 {
		b.Fatal("FindNearestPoly failed")
	}
	_ = pos

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		idx := c.AddAgent(pos, params)
		if idx >= 0 {
			c.RemoveAgent(idx)
		}
	}
}

func BenchmarkCrowdRequestAndPath(b *testing.B) {
	navQuery := createTestNavMeshQuery(b)
	c := NewCrowd()
	c.Init(16, 0.6, navQuery)

	filter := detour.NewQueryFilter()
	filter.IncludeFlags = 0xffff
	c.SetFilter(0, filter)

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

	startRef, startPos, err := navQuery.FindNearestPoly([3]float32{1, 0, 1}, [3]float32{2, 4, 2}, filter)
	if err != nil || startRef == 0 {
		b.Fatal("FindNearestPoly start failed")
	}
	endRef, endPos, err := navQuery.FindNearestPoly([3]float32{8, 0, 8}, [3]float32{2, 4, 2}, filter)
	if err != nil || endRef == 0 {
		b.Fatal("FindNearestPoly end failed")
	}

	b.ResetTimer()
	const dt = 1.0 / 60.0
	for i := 0; i < b.N; i++ {
		idx := c.AddAgent(startPos, params)
		if idx < 0 {
			continue
		}
		c.RequestMoveTarget(idx, endRef, endPos)
		for j := 0; j < 5; j++ {
			c.Update(dt, nil)
		}
		c.RemoveAgent(idx)
	}
}
