package detour_crowd

import (
	"errors"
	"math"

	"github.com/actfuns/recastnavigation/detour"
)

const (
	maxItersPerUpdate = 100
	maxPathqueueNodes = 4096
)

// Crowd manages local steering and dynamic avoidance for a group of agents.
type Crowd struct {
	maxAgents      int
	maxAgentRadius float32
	agents         []CrowdAgent
	activeAgents   []*CrowdAgent
	agentAnims     []CrowdAgentAnimation

	pathQueue PathQueue

	obstacleQueryParams [CrowdMaxObstAvoidanceParams]ObstacleAvoidanceParams
	obstacleQuery       *ObstacleAvoidanceQuery

	grid *ProximityGrid

	pathResult    []PolyRef
	maxPathResult int

	agentPlacementHalfExtents [3]float32

	filters             [CrowdMaxQueryFilterType]interface{}
	velocitySampleCount int

	navquery NavMeshQueryInterface
}

// NewCrowd creates a new crowd object.
func NewCrowd() *Crowd {
	return &Crowd{}
}

// Init initializes the crowd manager.
func (c *Crowd) Init(maxAgents int, maxAgentRadius float32, nav NavMeshQueryInterface) bool {
	c.maxAgents = maxAgents
	c.maxAgentRadius = maxAgentRadius

	// Larger than agent radius because it is also used for agent recovery.
	c.agentPlacementHalfExtents = [3]float32{
		maxAgentRadius * 2.0,
		maxAgentRadius * 1.5,
		maxAgentRadius * 2.0,
	}

	c.grid = NewProximityGrid()
	if !c.grid.Init(maxAgents*4, maxAgentRadius*3) {
		return false
	}

	c.obstacleQuery = NewObstacleAvoidanceQuery()
	if !c.obstacleQuery.Init(6, 8) {
		return false
	}

	// Init obstacle query params.
	for i := 0; i < CrowdMaxObstAvoidanceParams; i++ {
		params := &c.obstacleQueryParams[i]
		params.VelBias = 0.4
		params.WeightDesVel = 2.0
		params.WeightCurVel = 0.75
		params.WeightSide = 0.75
		params.WeightToi = 2.5
		params.HorizTime = 2.5
		params.GridSize = 33
		params.AdaptiveDivs = 7
		params.AdaptiveRings = 2
		params.AdaptiveDepth = 5
	}

	// Allocate temp buffer for merging paths.
	c.maxPathResult = 256
	c.pathResult = make([]PolyRef, c.maxPathResult)

	if !c.pathQueue.Init(c.maxPathResult, maxPathqueueNodes, nav) {
		return false
	}

	c.agents = make([]CrowdAgent, maxAgents)
	c.activeAgents = make([]*CrowdAgent, maxAgents)
	c.agentAnims = make([]CrowdAgentAnimation, maxAgents)

	for i := 0; i < maxAgents; i++ {
		c.agents[i].corridor = NewPathCorridor()
		c.agents[i].corridor.Init(c.maxPathResult)
		c.agents[i].active = false
	}

	for i := 0; i < maxAgents; i++ {
		c.agentAnims[i].Active = false
	}

	c.navquery = nav

	return true
}

// SetObstacleAvoidanceParams sets obstacle avoidance params for a given index.
func (c *Crowd) SetObstacleAvoidanceParams(idx int, params *ObstacleAvoidanceParams) {
	if idx >= 0 && idx < CrowdMaxObstAvoidanceParams {
		c.obstacleQueryParams[idx] = *params
	}
}

// GetObstacleAvoidanceParams returns the obstacle avoidance params for a given index.
func (c *Crowd) GetObstacleAvoidanceParams(idx int) *ObstacleAvoidanceParams {
	if idx >= 0 && idx < CrowdMaxObstAvoidanceParams {
		return &c.obstacleQueryParams[idx]
	}
	return nil
}

// GetAgentCount returns the maximum number of agents.
func (c *Crowd) GetAgentCount() int {
	return c.maxAgents
}

// GetAgent returns the agent at the given index.
func (c *Crowd) GetAgent(idx int) *CrowdAgent {
	if idx < 0 || idx >= c.maxAgents {
		return nil
	}
	return &c.agents[idx]
}

// GetEditableAgent returns the agent at the given index for modification.
func (c *Crowd) GetEditableAgent(idx int) *CrowdAgent {
	if idx < 0 || idx >= c.maxAgents {
		return nil
	}
	return &c.agents[idx]
}

// UpdateAgentParameters updates the specified agent's configuration.
func (c *Crowd) UpdateAgentParameters(idx int, params *CrowdAgentParams) {
	if idx < 0 || idx >= c.maxAgents {
		return
	}
	c.agents[idx].params = *params
}

// AddAgent adds a new agent to the crowd.
func (c *Crowd) AddAgent(pos [3]float32, params *CrowdAgentParams) int {
	// Find empty slot
	idx := -1
	for i := 0; i < c.maxAgents; i++ {
		if !c.agents[i].active {
			idx = i
			break
		}
	}
	if idx == -1 {
		return -1
	}

	ag := &c.agents[idx]

	c.UpdateAgentParameters(idx, params)

	// Find nearest position on navmesh and place the agent there.
	var nearest [3]float32
	var ref PolyRef
	nearest = pos

	queryFilter := c.getFilter(ag.params.QueryFilterType)
	if queryFilter != nil {
		r, nearestPt, _ := c.navquery.FindNearestPoly(pos, c.agentPlacementHalfExtents, queryFilter)
		ref = r
		if ref != 0 {
			nearest = nearestPt
		}
	}

	ag.corridor.Reset(ref, nearest)
	ag.boundary.Reset()
	ag.partial = false

	ag.topologyOptTime = 0
	ag.targetReplanTime = 0
	ag.nneis = 0

	ag.dvel = [3]float32{}
	ag.nvel = [3]float32{}
	ag.vel = [3]float32{}
	ag.npos = nearest

	ag.desiredSpeed = 0

	if ref != 0 {
		ag.state = CrowdAgentStateWalking
	} else {
		ag.state = CrowdAgentStateInvalid
	}

	ag.targetState = CrowdAgentTargetNone
	ag.active = true

	return idx
}

// RemoveAgent removes an agent from the crowd.
func (c *Crowd) RemoveAgent(idx int) {
	if idx >= 0 && idx < c.maxAgents {
		c.agents[idx].active = false
	}
}

// RequestMoveTarget submits a new move target request for the specified agent.
func (c *Crowd) RequestMoveTarget(idx int, ref PolyRef, pos [3]float32) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}
	if ref == 0 {
		return false
	}

	ag := &c.agents[idx]

	ag.targetRef = ref
	ag.targetPos = pos
	ag.targetPathqRef = 0
	ag.targetReplan = false
	if ag.targetRef != 0 {
		ag.targetState = CrowdAgentTargetRequesting
	} else {
		ag.targetState = CrowdAgentTargetFailed
	}

	return true
}

func (c *Crowd) RequestMoveTargetReplan(idx int, ref PolyRef, pos [3]float32) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}

	ag := &c.agents[idx]

	ag.targetRef = ref
	ag.targetPos = pos
	ag.targetPathqRef = 0
	ag.targetReplan = true
	if ag.targetRef != 0 {
		ag.targetState = CrowdAgentTargetRequesting
	} else {
		ag.targetState = CrowdAgentTargetFailed
	}

	return true
}

// RequestMoveVelocity submits a velocity-based move request.
func (c *Crowd) RequestMoveVelocity(idx int, vel [3]float32) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}

	ag := &c.agents[idx]

	ag.targetRef = 0
	ag.targetPos = vel
	ag.targetPathqRef = 0
	ag.targetReplan = false
	ag.targetState = CrowdAgentTargetVelocity

	return true
}

// ResetMoveTarget resets the move target for the specified agent.
func (c *Crowd) ResetMoveTarget(idx int) bool {
	if idx < 0 || idx >= c.maxAgents {
		return false
	}

	ag := &c.agents[idx]

	ag.targetRef = 0
	ag.targetPos = [3]float32{}
	ag.dvel = [3]float32{}
	ag.targetPathqRef = 0
	ag.targetReplan = false
	ag.targetState = CrowdAgentTargetNone

	return true
}

// GetActiveAgents returns the list of active agents.
func (c *Crowd) GetActiveAgents() []*CrowdAgent {
	n := 0
	for i := 0; i < c.maxAgents; i++ {
		if c.agents[i].active {
			if n < len(c.activeAgents) {
				c.activeAgents[n] = &c.agents[i]
				n++
			}
		}
	}
	return c.activeAgents[:n]
}

// Update updates the steering and positions of all agents.
func (c *Crowd) Update(dt float32, debug *CrowdAgentDebugInfo) {
	c.velocitySampleCount = 0

	debugIdx := -1
	if debug != nil {
		debugIdx = debug.Idx
	}

	agents := c.GetActiveAgents()
	nagents := len(agents)

	// Check that all agents still have valid paths.
	c.checkPathValidity(agents, nagents, dt)

	// Update async move request and path finder.
	c.updateMoveRequest(dt)

	// Optimize path topology.
	c.updateTopologyOptimization(agents, nagents, dt)

	// Register agents to proximity grid.
	c.grid.Clear()
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		p := &ag.npos
		r := ag.params.Radius
		c.grid.AddItem(uint16(i), p[0]-r, p[2]-r, p[0]+r, p[2]+r)
	}

	// Get nearby navmesh segments and agents to collide with.
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != CrowdAgentStateWalking {
			continue
		}

		// Update the collision boundary after certain distance has been passed.
		updateThr := ag.params.CollisionQueryRange * 0.25
		queryFilter := c.getFilter(ag.params.QueryFilterType)
		if queryFilter != nil {
			if vecDist2DSqr(ag.npos, *ag.boundary.GetCenter()) > updateThr*updateThr ||
				!ag.boundary.IsValid(c.navquery, queryFilter) {
				ag.boundary.Update(ag.corridor.GetFirstPoly(), ag.npos,
					ag.params.CollisionQueryRange, c.navquery, queryFilter)
			}
		}

		// Query neighbour agents
		ag.nneis = c.getNeighbours(ag.npos, ag.params.Height, ag.params.CollisionQueryRange,
			ag, ag.neis[:], CrowdAgentMaxNeighbours, agents, nagents, c.grid)
		for j := 0; j < ag.nneis; j++ {
			ag.neis[j].Idx = c.getAgentIndex(agents[ag.neis[j].Idx])
		}
	}

	// Find next corner to steer to.
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != CrowdAgentStateWalking {
			continue
		}
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			continue
		}

		queryFilter := c.getFilter(ag.params.QueryFilterType)
		if queryFilter == nil {
			continue
		}

		// Find corners for steering
		ag.ncorners = ag.corridor.FindCorners(ag.cornerVerts[:], ag.cornerFlags[:],
			ag.cornerPolys[:], CrowdAgentMaxCorners, c.navquery, queryFilter)

		// Check to see if the corner after the next corner is directly visible.
		if ag.params.UpdateFlags&CrowdOptimizeVis != 0 && ag.ncorners > 0 {
			target := cornerVert(ag.cornerVerts[:], recastMin(1, ag.ncorners-1))
			ag.corridor.OptimizePathVisibility(target, ag.params.PathOptimizationRange, c.navquery, queryFilter)

			if debugIdx == i {
				debug.OptStart = ag.corridor.GetPos()
				debug.OptEnd = target
			}
		} else {
			if debugIdx == i {
				debug.OptStart = [3]float32{}
				debug.OptEnd = [3]float32{}
			}
		}
	}

	// Trigger off-mesh connections (depends on corners).
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != CrowdAgentStateWalking {
			continue
		}
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			continue
		}

		// Check for off-mesh connection
		triggerRadius := ag.params.Radius * 2.25
		if overOffmeshConnection(ag, triggerRadius) {
			// Prepare to off-mesh connection.
			idx := c.getAgentIndex(ag)
			anim := &c.agentAnims[idx]

			// Adjust the path over the off-mesh connection.
			var refs [2]PolyRef
			startPos, endPos, ok := ag.corridor.MoveOverOffmeshConnection(ag.cornerPolys[ag.ncorners-1], &refs, c.navquery)
			if ok {
				anim.StartPos = startPos
				anim.EndPos = endPos
				anim.InitPos = ag.npos
				anim.PolyRef = int64(refs[1])
				anim.Active = true
				anim.T = 0
				anim.TMax = (vecDist2D(anim.StartPos, anim.EndPos) / ag.params.MaxSpeed) * 0.5

				ag.state = CrowdAgentStateOffMesh
				ag.ncorners = 0
				ag.nneis = 0
				continue
			}
		}
	}

	// Calculate steering.
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != CrowdAgentStateWalking {
			continue
		}
		if ag.targetState == CrowdAgentTargetNone {
			continue
		}

		var dvel [3]float32

		if ag.targetState == CrowdAgentTargetVelocity {
			dvel = ag.targetPos
			ag.desiredSpeed = vecLen(ag.targetPos)
		} else {
			// Calculate steering direction.
			if ag.params.UpdateFlags&CrowdAnticipateTurns != 0 {
				dvel = calcSmoothSteerDirection(ag)
			} else {
				dvel = calcStraightSteerDirection(ag)
			}

			// Calculate speed scale
			slowDownRadius := ag.params.Radius * 2
			speedScale := getDistanceToGoal(ag, slowDownRadius) / slowDownRadius

			ag.desiredSpeed = ag.params.MaxSpeed
			dvel = vecScale(dvel, ag.desiredSpeed*speedScale)
		}

		// Separation
		if ag.params.UpdateFlags&CrowdSeparation != 0 {
			separationDist := ag.params.CollisionQueryRange
			invSeparationDist := 1.0 / separationDist
			separationWeight := ag.params.SeparationWeight

			var disp [3]float32
			w := float32(0)

			for j := 0; j < ag.nneis; j++ {
				nei := &c.agents[ag.neis[j].Idx]

				diff := vecSub(ag.npos, nei.npos)
				diff[1] = 0

				distSqr := vecLenSqr(diff)
				if distSqr < 0.00001 {
					continue
				}
				if distSqr > separationDist*separationDist {
					continue
				}
				dist := float32(math.Sqrt(float64(distSqr)))
				weight := separationWeight * (1.0 - (dist*invSeparationDist)*(dist*invSeparationDist))

				disp = vecMad(disp, diff, weight/dist)
				w += 1.0
			}

			if w > 0.0001 {
				// Adjust desired velocity.
				dvel = vecMad(dvel, disp, 1.0/w)
				// Clamp desired velocity to desired speed.
				speedSqr := vecLenSqr(dvel)
				desiredSqr := ag.desiredSpeed * ag.desiredSpeed
				if speedSqr > desiredSqr {
					dvel = vecScale(dvel, desiredSqr/speedSqr)
				}
			}
		}

		// Set the desired velocity.
		ag.dvel = dvel
	}

	// Velocity planning.
	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != CrowdAgentStateWalking {
			continue
		}

		if ag.params.UpdateFlags&CrowdObstacleAvoidance != 0 {
			c.obstacleQuery.Reset()

			// Add neighbours as obstacles.
			for j := 0; j < ag.nneis; j++ {
				nei := &c.agents[ag.neis[j].Idx]
				c.obstacleQuery.AddCircle(nei.npos, nei.params.Radius, nei.vel, nei.dvel)
			}

			// Append neighbour segments as obstacles.
			for j := 0; j < ag.boundary.GetSegmentCount(); j++ {
				s := ag.boundary.GetSegment(j)
				if triArea2D(ag.npos, [3]float32{s[0], s[1], s[2]}, [3]float32{s[3], s[4], s[5]}) < 0 {
					continue
				}
				c.obstacleQuery.AddSegment([3]float32{s[0], s[1], s[2]}, [3]float32{s[3], s[4], s[5]})
			}

			var vod *ObstacleAvoidanceDebugData
			if debugIdx == i && debug != nil {
				vod = debug.Vod
			}

			// Sample new safe velocity.
			params := &c.obstacleQueryParams[ag.params.ObstacleAvoidanceType]

			nvel, ns := c.obstacleQuery.SampleVelocityAdaptive(ag.npos, ag.params.Radius, ag.desiredSpeed,
				ag.vel, ag.dvel, params, vod)
			ag.nvel = nvel
			c.velocitySampleCount += ns
		} else {
			ag.nvel = ag.dvel
		}
	}

	// Integrate.
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != CrowdAgentStateWalking {
			continue
		}
		integrate(ag, dt)
	}

	// Handle collisions.
	const collisionResolveFactor = 0.7

	for iter := 0; iter < 4; iter++ {
		for i := 0; i < nagents; i++ {
			ag := agents[i]
			idx0 := c.getAgentIndex(ag)

			if ag.state != CrowdAgentStateWalking {
				continue
			}

			ag.disp = [3]float32{}
			w := float32(0)

			for j := 0; j < ag.nneis; j++ {
				nei := &c.agents[ag.neis[j].Idx]
				idx1 := c.getAgentIndex(nei)

				diff := vecSub(ag.npos, nei.npos)
				diff[1] = 0

				distSq := vecLenSqr(diff)
				if distSq > (ag.params.Radius+nei.params.Radius)*(ag.params.Radius+nei.params.Radius) {
					continue
				}
				dist := float32(math.Sqrt(float64(distSq)))
				pen := (ag.params.Radius + nei.params.Radius) - dist
				if dist < 0.0001 {
					if idx0 > idx1 {
						diff = [3]float32{-ag.dvel[2], 0, ag.dvel[0]}
					} else {
						diff = [3]float32{ag.dvel[2], 0, -ag.dvel[0]}
					}
					pen = 0.01
				} else {
					pen = (1.0 / dist) * (pen * 0.5) * collisionResolveFactor
				}

				ag.disp = vecMad(ag.disp, diff, pen)
				w += 1.0
			}

			if w > 0.0001 {
				ag.disp = vecScale(ag.disp, 1.0/w)
			}
		}

		for i := 0; i < nagents; i++ {
			ag := agents[i]
			if ag.state != CrowdAgentStateWalking {
				continue
			}
			ag.npos = vecAdd(ag.npos, ag.disp)
		}
	}

	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != CrowdAgentStateWalking {
			continue
		}

		// Move along navmesh.
		ag.corridor.MovePosition(ag.npos, c.navquery, c.getFilter(ag.params.QueryFilterType))
		// Get valid constrained position back.
		ag.npos = ag.corridor.GetPos()

		// If not using path, truncate the corridor to just one poly.
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			ag.corridor.Reset(ag.corridor.GetFirstPoly(), ag.npos)
			ag.partial = false
		}
	}

	// Update agents using off-mesh connection.
	for i := 0; i < nagents; i++ {
		ag := agents[i]
		idx := c.getAgentIndex(ag)
		anim := &c.agentAnims[idx]
		if !anim.Active {
			continue
		}

		anim.T += dt
		if anim.T > anim.TMax {
			// Reset animation
			anim.Active = false
			// Prepare agent for walking.
			ag.state = CrowdAgentStateWalking
			continue
		}

		// Update position
		ta := anim.TMax * 0.15
		tb := anim.TMax
		if anim.T < ta {
			u := tween(anim.T, 0, ta)
			ag.npos = vecLerp(anim.InitPos, anim.StartPos, u)
		} else {
			u := tween(anim.T, ta, tb)
			ag.npos = vecLerp(anim.StartPos, anim.EndPos, u)
		}

		// Update velocity.
		ag.vel = [3]float32{}
		ag.dvel = [3]float32{}
	}
}

// GetFilter returns the filter for the given index.
func (c *Crowd) GetFilter(i int) interface{} {
	if i >= 0 && i < CrowdMaxQueryFilterType {
		return c.filters[i]
	}
	return nil
}

// GetEditableFilter returns the filter for the given index for modification.
func (c *Crowd) GetEditableFilter(i int) interface{} {
	if i >= 0 && i < CrowdMaxQueryFilterType {
		return c.filters[i]
	}
	return nil
}

// GetQueryHalfExtents returns the search half extents used by the crowd.
func (c *Crowd) GetQueryHalfExtents() *[3]float32 {
	return &c.agentPlacementHalfExtents
}

// GetQueryExtents returns the search extents (same as half extents).
func (c *Crowd) GetQueryExtents() *[3]float32 {
	return &c.agentPlacementHalfExtents
}

// GetVelocitySampleCount returns the velocity sample count.
func (c *Crowd) GetVelocitySampleCount() int {
	return c.velocitySampleCount
}

// GetGrid returns the proximity grid.
func (c *Crowd) GetGrid() *ProximityGrid {
	return c.grid
}

// GetPathQueue returns the path queue.
func (c *Crowd) GetPathQueue() *PathQueue {
	return &c.pathQueue
}

// GetNavMeshQuery returns the navigation mesh query.
func (c *Crowd) GetNavMeshQuery() NavMeshQueryInterface {
	return c.navquery
}

// SetFilter sets a filter at a given index.
func (c *Crowd) SetFilter(i int, filter interface{}) {
	if i >= 0 && i < CrowdMaxQueryFilterType {
		c.filters[i] = filter
	}
}

// --- Internal methods ---

func (c *Crowd) getAgentIndex(agent *CrowdAgent) int {
	// Calculate offset from the base of the agents array
	for i := 0; i < c.maxAgents; i++ {
		if &c.agents[i] == agent {
			return i
		}
	}
	return -1
}

func (c *Crowd) getFilter(queryFilterType uint8) *QueryFilter {
	if int(queryFilterType) < CrowdMaxQueryFilterType {
		if f, ok := c.filters[queryFilterType].(*QueryFilter); ok {
			return f
		}
		// Also try *detour.QueryFilter that was set via SetFilter
		return c.filters[queryFilterType].(*QueryFilter)
	}
	return nil
}

func (c *Crowd) updateMoveRequest(_ float32) {
	const pathMaxAgents = 8
	queue := make([]*CrowdAgent, 0, pathMaxAgents)

	// Fire off new requests.
	for i := 0; i < c.maxAgents; i++ {
		ag := &c.agents[i]
		if !ag.active {
			continue
		}
		if ag.state == CrowdAgentStateInvalid {
			continue
		}
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			continue
		}

		if ag.targetState == CrowdAgentTargetRequesting {
			path := ag.corridor.GetPath()

			const maxRes = 32
			var reqPos [3]float32
			reqPath := make([]PolyRef, maxRes)
			reqPathCount := 0

			// Quick search towards the goal.
			const maxIter = 20
			queryFilter := c.getFilter(ag.params.QueryFilterType)

			c.navquery.FindPathSliced(path[0], ag.targetRef, ag.npos, ag.targetPos, queryFilter, 0)
			c.navquery.UpdateSlicedPath(maxIter)

			var err error
			reqPathCount, err = c.navquery.GetPathFromSlicedPath(reqPath, maxRes)

			if err == nil && reqPathCount > 0 {
				if reqPath[reqPathCount-1] != ag.targetRef {
					reqPos, _, err = c.navquery.ClosestPointOnPoly(reqPath[reqPathCount-1], ag.targetPos)
					if err != nil {
						reqPathCount = 0
					}
				} else {
					reqPos = ag.targetPos
				}
			} else {
				reqPathCount = 0
			}

			if reqPathCount == 0 {
				reqPos = ag.npos
				reqPath[0] = path[0]
				reqPathCount = 1
			}

			ag.corridor.SetCorridor(reqPos, reqPath, reqPathCount)
			ag.boundary.Reset()
			ag.partial = false

			if reqPath[reqPathCount-1] == ag.targetRef {
				ag.targetState = CrowdAgentTargetValid
				ag.targetReplanTime = 0
			} else {
				ag.targetState = CrowdAgentTargetWaitingForQueue
			}
		}

		if ag.targetState == CrowdAgentTargetWaitingForQueue {
			queue = append(queue, ag)
		}
	}

	// Submit path requests to the path queue.
	for _, ag := range queue {
		queryFilter := c.getFilter(ag.params.QueryFilterType)
		ag.targetPathqRef = c.pathQueue.Request(ag.corridor.GetLastPoly(), ag.targetRef,
			ag.corridor.GetTarget(), ag.targetPos, queryFilter)
		if ag.targetPathqRef != 0 {
			ag.targetState = CrowdAgentTargetWaitingForPath
		}
	}

	// Update requests.
	c.pathQueue.Update(maxItersPerUpdate)

	// Process path results.
	for i := 0; i < c.maxAgents; i++ {
		ag := &c.agents[i]
		if !ag.active {
			continue
		}
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			continue
		}

		if ag.targetState == CrowdAgentTargetWaitingForPath {
			err := c.pathQueue.GetRequestErr(ag.targetPathqRef)
			if err != nil {
				ag.targetPathqRef = 0
				if ag.targetRef != 0 {
					ag.targetState = CrowdAgentTargetRequesting
				} else {
					ag.targetState = CrowdAgentTargetFailed
				}
				ag.targetReplanTime = 0
			} else {
				path := ag.corridor.GetPath()
				npath := ag.corridor.GetPathCount()

				targetPos := ag.targetPos
				res := c.pathResult
				valid := true
				nres := 0
				nres, err := c.pathQueue.GetPathResult(ag.targetPathqRef, res, c.maxPathResult)
				if err != nil || nres == 0 {
					valid = false
				}

				if errors.Is(err, detour.ErrPartialResult) {
					ag.partial = true
				} else {
					ag.partial = false
				}

				if valid && npath > 0 && path[npath-1] != res[0] {
					valid = false
				}

				if valid {
					if npath > 1 {
						if (npath-1)+nres > c.maxPathResult {
							nres = c.maxPathResult - (npath - 1)
						}

						// Make space for the old path.
						copy(res[npath-1:npath-1+nres], res[:nres])
						// Copy old path in the beginning.
						copy(res[:npath-1], path[:npath-1])
						nres += npath - 1

						// Remove trackbacks
						for j := 0; j < nres; j++ {
							if j-1 >= 0 && j+1 < nres {
								if res[j-1] == res[j+1] {
									copy(res[j-1:], res[j+1:nres])
									nres -= 2
									j -= 2
								}
							}
						}
					}

					// Check for partial path.
					if res[nres-1] != ag.targetRef {
						var nearest [3]float32
						nearest, _, err = c.navquery.ClosestPointOnPoly(res[nres-1], targetPos)
						if err == nil {
							targetPos = nearest
						} else {
							valid = false
						}
					}
				}

				if valid {
					ag.corridor.SetCorridor(targetPos, res, nres)
					ag.boundary.Reset()
					ag.targetState = CrowdAgentTargetValid
				} else {
					ag.targetState = CrowdAgentTargetFailed
				}

				ag.targetReplanTime = 0
			}
		}
	}
}

func (c *Crowd) updateTopologyOptimization(agents []*CrowdAgent, nagents int, dt float32) {
	if nagents == 0 {
		return
	}

	const optTimeThr = 0.5
	const optMaxAgents = 1
	queue := make([]*CrowdAgent, 0, optMaxAgents)

	for i := 0; i < nagents; i++ {
		ag := agents[i]
		if ag.state != CrowdAgentStateWalking {
			continue
		}
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			continue
		}
		if ag.params.UpdateFlags&CrowdOptimizeTopo == 0 {
			continue
		}
		ag.topologyOptTime += dt
		if ag.topologyOptTime >= optTimeThr {
			queue = append(queue, ag)
		}
	}

	for _, ag := range queue {
		queryFilter := c.getFilter(ag.params.QueryFilterType)
		ag.corridor.OptimizePathTopology(c.navquery, queryFilter)
		ag.topologyOptTime = 0
	}
}

func (c *Crowd) checkPathValidity(agents []*CrowdAgent, nagents int, dt float32) {
	const checkLookahead = 10
	const targetReplanDelay = 1.0

	for i := 0; i < nagents; i++ {
		ag := agents[i]

		if ag.state != CrowdAgentStateWalking {
			continue
		}

		ag.targetReplanTime += dt

		replan := false

		// First check that the current location is valid.
		idx := c.getAgentIndex(ag)
		queryFilter := c.getFilter(ag.params.QueryFilterType)
		agentRef := ag.corridor.GetFirstPoly()

		if !c.navquery.IsValidPolyRef(agentRef, queryFilter) {
			// Current location is not valid, try to reposition.
			agentPos := ag.npos
			nearest := agentPos
			agentRef = 0

			r, nearestPt, _ := c.navquery.FindNearestPoly(ag.npos, c.agentPlacementHalfExtents, queryFilter)
			agentRef = r
			if agentRef != 0 {
				nearest = nearestPt
			} else {
				// Could not find location in navmesh, set state to invalid.
				ag.corridor.Reset(0, agentPos)
				ag.partial = false
				ag.boundary.Reset()
				ag.state = CrowdAgentStateInvalid
				continue
			}

			ag.corridor.FixPathStart(agentRef, nearest)
			ag.boundary.Reset()
			ag.npos = nearest
			replan = true
		}

		// If the agent does not have move target or is controlled by velocity, no need to recover.
		if ag.targetState == CrowdAgentTargetNone || ag.targetState == CrowdAgentTargetVelocity {
			continue
		}

		// Try to recover move request position.
		if ag.targetState != CrowdAgentTargetNone && ag.targetState != CrowdAgentTargetFailed {
			if !c.navquery.IsValidPolyRef(ag.targetRef, queryFilter) {
				nearest := ag.targetPos
				ag.targetRef = 0
				r, nearestPt, _ := c.navquery.FindNearestPoly(ag.targetPos, c.agentPlacementHalfExtents, queryFilter)
				ag.targetRef = r
				if ag.targetRef != 0 {
					nearest = nearestPt
				}
				ag.targetPos = nearest
				replan = true
			}
			if ag.targetRef == 0 {
				ag.corridor.Reset(agentRef, ag.npos)
				ag.partial = false
				ag.targetState = CrowdAgentTargetNone
			}
		}

		// If nearby corridor is not valid, replan.
		if !ag.corridor.IsValid(checkLookahead, c.navquery, queryFilter) {
			replan = true
		}

		// If the end of the path is near and it is not the requested location, replan.
		if ag.targetState == CrowdAgentTargetValid {
			if ag.targetReplanTime > targetReplanDelay &&
				ag.corridor.GetPathCount() < checkLookahead &&
				ag.corridor.GetLastPoly() != ag.targetRef {
				replan = true
			}
		}

		// Try to replan path to goal.
		if replan {
			if ag.targetState != CrowdAgentTargetNone {
				c.RequestMoveTargetReplan(idx, ag.targetRef, ag.targetPos)
			}
		}
	}
}

func (c *Crowd) getNeighbours(pos [3]float32, height, range_ float32, skip *CrowdAgent, result []CrowdNeighbour, maxResult int, agents []*CrowdAgent, _ int, grid *ProximityGrid) int {
	n := 0
	const maxNeis = 32
	ids := make([]uint16, maxNeis)
	nids := grid.QueryItems(pos[0]-range_, pos[2]-range_,
		pos[0]+range_, pos[2]+range_, ids)

	for i := 0; i < nids; i++ {
		ag := agents[ids[i]]
		if ag == skip {
			continue
		}

		// Check for overlap.
		diff := vecSub(pos, ag.npos)
		if float32(math.Abs(float64(diff[1]))) >= (height+ag.params.Height)/2.0 {
			continue
		}
		diff[1] = 0
		distSqr := vecLenSqr(diff)
		if distSqr > range_*range_ {
			continue
		}

		n = addNeighbour(int(ids[i]), distSqr, result, n, maxResult)
	}
	return n
}

// --- Static helper functions ---

func integrate(ag *CrowdAgent, dt float32) {
	// Fake dynamic constraint.
	maxDelta := ag.params.MaxAcceleration * dt
	dv := vecSub(ag.nvel, ag.vel)
	ds := vecLen(dv)
	if ds > maxDelta {
		dv = vecScale(dv, maxDelta/ds)
	}
	ag.vel = vecAdd(ag.vel, dv)

	// Integrate
	if vecLen(ag.vel) > 0.0001 {
		ag.npos = vecMad(ag.npos, ag.vel, dt)
	} else {
		ag.vel = [3]float32{}
	}
}

func overOffmeshConnection(ag *CrowdAgent, radius float32) bool {
	if ag.ncorners == 0 {
		return false
	}

	offMeshConnection := (ag.cornerFlags[ag.ncorners-1] & StraightPathOffMeshConnection) != 0
	if offMeshConnection {
		distSq := vecDist2DSqr(ag.npos, cornerVert(ag.cornerVerts[:], ag.ncorners-1))
		if distSq < radius*radius {
			return true
		}
	}

	return false
}

func getDistanceToGoal(ag *CrowdAgent, range_ float32) float32 {
	if ag.ncorners == 0 {
		return range_
	}

	endOfPath := (ag.cornerFlags[ag.ncorners-1] & StraightPathEnd) != 0
	if endOfPath {
		return float32(math.Min(
			float64(vecDist2D(ag.npos, cornerVert(ag.cornerVerts[:], ag.ncorners-1))),
			float64(range_)))
	}

	return range_
}

func calcSmoothSteerDirection(ag *CrowdAgent) [3]float32 {
	if ag.ncorners == 0 {
		return [3]float32{}
	}

	ip0 := 0
	ip1 := recastMin(1, ag.ncorners-1)
	p0 := cornerVert(ag.cornerVerts[:], ip0)
	p1 := cornerVert(ag.cornerVerts[:], ip1)

	dir0 := vecSub(p0, ag.npos)
	dir1 := vecSub(p1, ag.npos)
	dir0[1] = 0
	dir1[1] = 0

	len0 := vecLen(dir0)
	len1 := vecLen(dir1)
	if len1 > 0.001 {
		dir1 = vecScale(dir1, 1.0/len1)
	}

	var dir [3]float32
	dir[0] = dir0[0] - dir1[0]*len0*0.5
	dir[1] = 0
	dir[2] = dir0[2] - dir1[2]*len0*0.5

	return vecNormalize(dir)
}

func calcStraightSteerDirection(ag *CrowdAgent) [3]float32 {
	if ag.ncorners == 0 {
		return [3]float32{}
	}
	d := vecSub(cornerVert(ag.cornerVerts[:], 0), ag.npos)
	d[1] = 0
	return vecNormalize(d)
}

func addNeighbour(idx int, dist float32, neis []CrowdNeighbour, nneis, maxNeis int) int {
	// Insert neighbour based on the distance.
	if nneis == 0 {
		neis[0] = CrowdNeighbour{Idx: idx, Dist: dist}
		return 1
	}

	last := &neis[nneis-1]
	if dist >= last.Dist {
		if nneis >= maxNeis {
			return nneis
		}
		neis[nneis] = CrowdNeighbour{Idx: idx, Dist: dist}
		return nneis + 1
	}

	// Find insertion point
	i := 0
	for i < nneis {
		if dist <= neis[i].Dist {
			break
		}
		i++
	}

	tgt := i + 1
	n := recastMin(nneis-i, maxNeis-tgt)
	if n > 0 {
		copy(neis[tgt:tgt+n], neis[i:i+n])
	}
	neis[i] = CrowdNeighbour{Idx: idx, Dist: dist}

	return recastMin(nneis+1, maxNeis)
}
