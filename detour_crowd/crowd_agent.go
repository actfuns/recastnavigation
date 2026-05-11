package detour_crowd

// CrowdAgent represents an agent managed by a Crowd object.
type CrowdAgent struct {
	// True if the agent is active, false if in an unused slot.
	active bool

	// The type of mesh polygon the agent is traversing.
	state CrowdAgentState

	// True if the agent has valid path and the path does not lead to the requested position.
	partial bool

	// The path corridor the agent is using.
	corridor *PathCorridor

	// The local boundary data for the agent.
	boundary LocalBoundary

	// Time since the agent's path corridor was optimized.
	topologyOptTime float32

	// The known neighbors of the agent.
	neis [CrowdAgentMaxNeighbours]CrowdNeighbour

	// The number of neighbors.
	nneis int

	// The desired speed.
	desiredSpeed float32

	npos [3]float32 // The current agent position. [(x, y, z)]
	disp [3]float32 // A temporary value used to accumulate agent disp
	dvel [3]float32 // The desired velocity of the agent.
	nvel [3]float32 // The desired velocity adjusted by obstacle avoidance.
	vel  [3]float32 // The actual velocity of the agent.

	params CrowdAgentParams

	// The local path corridor corners for the agent. (Straight path.)
	cornerVerts [CrowdAgentMaxCorners * 3]float32

	// The local path corridor corner flags.
	cornerFlags [CrowdAgentMaxCorners]uint8

	// The reference id of the polygon being entered at the corner.
	cornerPolys [CrowdAgentMaxCorners]PolyRef

	// The number of corners.
	ncorners int

	targetState      MoveRequestState // State of the movement request.
	targetRef        PolyRef          // Target polyref of the movement request.
	targetPos        [3]float32       // Target position of the movement request.
	targetPathqRef   PathQueueRef     // Path finder ref.
	targetReplan     bool             // Flag indicating that the current path is being
	targetReplanTime float32          // Time since the agent's target was replanned.
}

// IsActive returns whether the agent is active.
func (a *CrowdAgent) IsActive() bool {
	return a.active
}
