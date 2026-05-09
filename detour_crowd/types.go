// Package detour_crowd implements local steering and dynamic avoidance features for groups of agents.
package detour_crowd

import (
	"github.com/actfuns/recastnavigation/recast"
)

// Constants

// CrowdAgentMaxNeighbours is the maximum number of neighbors that a crowd agent
// can take into account for steering decisions.
const CrowdAgentMaxNeighbours = 6

// CrowdAgentMaxCorners is the maximum number of corners a crowd agent will look
// ahead in the path.
const CrowdAgentMaxCorners = 4

// CrowdMaxObstAvoidanceParams is the maximum number of crowd avoidance
// configurations supported by the crowd manager.
const CrowdMaxObstAvoidanceParams = 8

// CrowdMaxQueryFilterType is the maximum number of query filter types supported
// by the crowd manager.
const CrowdMaxQueryFilterType = 16

// PathQInvalid is an invalid path queue reference.
const PathQInvalid uint32 = 0

// Update flags for crowd agents.
const (
	CrowdAnticipateTurns   = 1 << 0
	CrowdObstacleAvoidance = 1 << 1
	CrowdSeparation        = 1 << 2
	CrowdOptimizeVis       = 1 << 3
	CrowdOptimizeTopo      = 1 << 4
)

// CrowdAgentState represents the navigation mesh polygon type the agent
// is currently traversing.
type CrowdAgentState int

const (
	CrowdAgentStateInvalid CrowdAgentState = iota // The agent is not in a valid state.
	CrowdAgentStateWalking                        // The agent is traversing a normal navigation mesh polygon.
	CrowdAgentStateOffMesh                        // The agent is traversing an off-mesh connection.
)

// MoveRequestState represents the state of a move request.
type MoveRequestState int

const (
	CrowdAgentTargetNone MoveRequestState = iota
	CrowdAgentTargetFailed
	CrowdAgentTargetValid
	CrowdAgentTargetRequesting
	CrowdAgentTargetWaitingForQueue
	CrowdAgentTargetWaitingForPath
	CrowdAgentTargetVelocity
)

// CrowdNeighbour provides neighbor data for agents managed by the crowd.
type CrowdNeighbour struct {
	Idx  int     // The index of the neighbor in the crowd.
	Dist float32 // The distance between the current agent and the neighbor.
}

// CrowdAgentParams holds configuration parameters for a crowd agent.
type CrowdAgentParams struct {
	Radius                float32 // Agent radius. [Limit: >= 0]
	Height                float32 // Agent height. [Limit: > 0]
	MaxAcceleration       float32 // Maximum allowed acceleration. [Limit: >= 0]
	MaxSpeed              float32 // Maximum allowed speed. [Limit: >= 0]
	CollisionQueryRange   float32 // How close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	PathOptimizationRange float32 // The path visibility optimization range. [Limit: > 0]
	SeparationWeight      float32 // How aggressive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	UpdateFlags           uint8   // Flags that impact steering behavior.
	ObstacleAvoidanceType uint8   // The index of the avoidance configuration to use for the agent.
	QueryFilterType       uint8   // The index of the query filter used by this agent.
}

// CrowdAgentAnimation represents an off-mesh connection animation for an agent.
type CrowdAgentAnimation struct {
	Active   bool
	InitPos  [3]float32
	StartPos [3]float32
	EndPos   [3]float32
	PolyRef  int64
	T        float32
	TMax     float32
}

// CrowdAgentDebugInfo holds debug information for a specific agent.
type CrowdAgentDebugInfo struct {
	Idx      int
	OptStart [3]float32
	OptEnd   [3]float32
	Vod      *ObstacleAvoidanceDebugData
}

// Helper functions used across the crowd package.

func tween(t, t0, t1 float32) float32 {
	return float32(recast.Clamp(int((t-t0)/(t1-t0)), 0, 1))
}

func clampF32(v, min, max float32) float32 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}
