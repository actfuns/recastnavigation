package detour_crowd

import (
	"math"
)

const pi = 3.14159265

// ObstacleCircle represents a circular obstacle for avoidance.
type ObstacleCircle struct {
	P      [3]float32 // Position of the obstacle
	Vel    [3]float32 // Velocity of the obstacle
	DVel   [3]float32 // Desired velocity of the obstacle
	Rad    float32    // Radius of the obstacle
	Dp, Np [3]float32 // Used for side selection during sampling
}

// ObstacleSegment represents a line segment obstacle for avoidance.
type ObstacleSegment struct {
	P, Q  [3]float32 // End points of the obstacle segment
	Touch bool
}

// MaxPatternDivs is the max number of adaptive divisions.
const MaxPatternDivs = 32

// MaxPatternRings is the max number of adaptive rings.
const MaxPatternRings = 4

// ObstacleAvoidanceParams holds configuration parameters for obstacle avoidance.
type ObstacleAvoidanceParams struct {
	VelBias       float32
	WeightDesVel  float32
	WeightCurVel  float32
	WeightSide    float32
	WeightToi     float32
	HorizTime     float32
	GridSize      uint8 // grid
	AdaptiveDivs  uint8 // adaptive
	AdaptiveRings uint8 // adaptive
	AdaptiveDepth uint8 // adaptive
}

// ObstacleAvoidanceDebugData stores debug samples for obstacle avoidance.
type ObstacleAvoidanceDebugData struct {
	nsamples   int
	maxSamples int
	vel        []float32
	ssize      []float32
	pen        []float32
	vpen       []float32
	vcpen      []float32
	spen       []float32
	tpen       []float32
}

// NewObstacleAvoidanceDebugData creates a new debug data object.
func NewObstacleAvoidanceDebugData() *ObstacleAvoidanceDebugData {
	return &ObstacleAvoidanceDebugData{}
}

// Init initializes the debug data with the maximum number of samples.
func (d *ObstacleAvoidanceDebugData) Init(maxSamples int) bool {
	if maxSamples <= 0 {
		return false
	}
	d.maxSamples = maxSamples
	d.vel = make([]float32, 3*maxSamples)
	d.ssize = make([]float32, maxSamples)
	d.pen = make([]float32, maxSamples)
	d.vpen = make([]float32, maxSamples)
	d.vcpen = make([]float32, maxSamples)
	d.spen = make([]float32, maxSamples)
	d.tpen = make([]float32, maxSamples)
	return true
}

// Reset clears the debug data.
func (d *ObstacleAvoidanceDebugData) Reset() {
	d.nsamples = 0
}

// AddSample adds a sample to the debug data.
func (d *ObstacleAvoidanceDebugData) AddSample(vel *[3]float32, ssize, pen, vpen, vcpen, spen, tpen float32) {
	if d.nsamples >= d.maxSamples {
		return
	}
	copy(d.vel[d.nsamples*3:d.nsamples*3+3], vel[:])
	d.ssize[d.nsamples] = ssize
	d.pen[d.nsamples] = pen
	d.vpen[d.nsamples] = vpen
	d.vcpen[d.nsamples] = vcpen
	d.spen[d.nsamples] = spen
	d.tpen[d.nsamples] = tpen
	d.nsamples++
}

// NormalizeSamples normalizes the penalty range for all samples.
func (d *ObstacleAvoidanceDebugData) NormalizeSamples() {
	normalizeArray(d.pen, d.nsamples)
	normalizeArray(d.vpen, d.nsamples)
	normalizeArray(d.vcpen, d.nsamples)
	normalizeArray(d.spen, d.nsamples)
	normalizeArray(d.tpen, d.nsamples)
}

// GetSampleCount returns the number of samples.
func (d *ObstacleAvoidanceDebugData) GetSampleCount() int {
	return d.nsamples
}

// GetSampleVelocity returns the velocity of a sample.
func (d *ObstacleAvoidanceDebugData) GetSampleVelocity(i int) *[3]float32 {
	return &[3]float32{d.vel[i*3], d.vel[i*3+1], d.vel[i*3+2]}
}

// GetSampleSize returns the size of a sample.
func (d *ObstacleAvoidanceDebugData) GetSampleSize(i int) float32 {
	return d.ssize[i]
}

// GetSamplePenalty returns the total penalty of a sample.
func (d *ObstacleAvoidanceDebugData) GetSamplePenalty(i int) float32 {
	return d.pen[i]
}

// GetSampleDesiredVelocityPenalty returns the desired velocity penalty.
func (d *ObstacleAvoidanceDebugData) GetSampleDesiredVelocityPenalty(i int) float32 {
	return d.vpen[i]
}

// GetSampleCurrentVelocityPenalty returns the current velocity penalty.
func (d *ObstacleAvoidanceDebugData) GetSampleCurrentVelocityPenalty(i int) float32 {
	return d.vcpen[i]
}

// GetSamplePreferredSidePenalty returns the preferred side penalty.
func (d *ObstacleAvoidanceDebugData) GetSamplePreferredSidePenalty(i int) float32 {
	return d.spen[i]
}

// GetSampleCollisionTimePenalty returns the collision time penalty.
func (d *ObstacleAvoidanceDebugData) GetSampleCollisionTimePenalty(i int) float32 {
	return d.tpen[i]
}

func normalizeArray(arr []float32, n int) {
	if n <= 0 {
		return
	}
	minPen := float32(math.MaxFloat32)
	maxPen := float32(-math.MaxFloat32)
	for i := 0; i < n; i++ {
		if arr[i] < minPen {
			minPen = arr[i]
		}
		if arr[i] > maxPen {
			maxPen = arr[i]
		}
	}
	penRange := maxPen - minPen
	s := float32(1.0)
	if penRange > 0.001 {
		s = 1.0 / penRange
	}
	for i := 0; i < n; i++ {
		arr[i] = clampF32((arr[i]-minPen)*s, 0, 1)
	}
}

// ObstacleAvoidanceQuery performs obstacle avoidance velocity sampling.
type ObstacleAvoidanceQuery struct {
	params       ObstacleAvoidanceParams
	invHorizTime float32
	vmax         float32
	invVmax      float32

	maxCircles int
	circles    []ObstacleCircle
	ncircles   int

	maxSegments int
	segments    []ObstacleSegment
	nsegments   int
}

// NewObstacleAvoidanceQuery creates a new obstacle avoidance query.
func NewObstacleAvoidanceQuery() *ObstacleAvoidanceQuery {
	return &ObstacleAvoidanceQuery{}
}

// Init initializes the query with max number of circles and segments.
func (q *ObstacleAvoidanceQuery) Init(maxCircles, maxSegments int) bool {
	q.maxCircles = maxCircles
	q.ncircles = 0
	q.circles = make([]ObstacleCircle, maxCircles)

	q.maxSegments = maxSegments
	q.nsegments = 0
	q.segments = make([]ObstacleSegment, maxSegments)

	return true
}

// Reset clears all obstacles.
func (q *ObstacleAvoidanceQuery) Reset() {
	q.ncircles = 0
	q.nsegments = 0
}

// AddCircle adds a circular obstacle.
func (q *ObstacleAvoidanceQuery) AddCircle(pos [3]float32, rad float32, vel, dvel [3]float32) {
	if q.ncircles >= q.maxCircles {
		return
	}
	cir := &q.circles[q.ncircles]
	q.ncircles++
	cir.P = pos
	cir.Rad = rad
	cir.Vel = vel
	cir.DVel = dvel
}

// AddSegment adds a segment obstacle.
func (q *ObstacleAvoidanceQuery) AddSegment(p, qp [3]float32) {
	if q.nsegments >= q.maxSegments {
		return
	}
	seg := &q.segments[q.nsegments]
	q.nsegments++
	seg.P = p
	seg.Q = qp
}

func (q *ObstacleAvoidanceQuery) prepare(pos, dvel [3]float32) {
	// Prepare obstacles
	for i := 0; i < q.ncircles; i++ {
		cir := &q.circles[i]

		// Side
		pb := cir.P

		var orig [3]float32
		cir.Dp = vecSub(pb, pos)
		cir.Dp = vecNormalize(cir.Dp)
		dv := vecSub(cir.DVel, dvel)

		a := triArea2D(orig, cir.Dp, dv)
		if a < 0.01 {
			cir.Np[0] = -cir.Dp[2]
			cir.Np[2] = cir.Dp[0]
		} else {
			cir.Np[0] = cir.Dp[2]
			cir.Np[2] = -cir.Dp[0]
		}
	}

	for i := 0; i < q.nsegments; i++ {
		seg := &q.segments[i]

		// Precalc if the agent is really close to the segment.
		const r = 0.01
		distSqr, _ := recastDistancePtSegSqr2D(pos, seg.P, seg.Q)
		seg.Touch = distSqr < r*r
	}
}

// ProcessSample calculates the penalty for a given velocity vector.
func (q *ObstacleAvoidanceQuery) ProcessSample(vcand [3]float32, cs float32,
	pos [3]float32, rad float32, vel, dvel [3]float32,
	minPenalty float32, debug *ObstacleAvoidanceDebugData) float32 {

	// Penalty for straying away from the desired and current velocities
	dx0 := vcand[0] - dvel[0]
	dz0 := vcand[2] - dvel[2]
	vpen := q.params.WeightDesVel * (float32(math.Sqrt(float64(dx0*dx0+dz0*dz0))) * q.invVmax)
	dx1 := vcand[0] - vel[0]
	dz1 := vcand[2] - vel[2]
	vcpen := q.params.WeightCurVel * (float32(math.Sqrt(float64(dx1*dx1+dz1*dz1))) * q.invVmax)

	// Find the threshold hit time to bail out based on the early out penalty
	minPen := minPenalty - vpen - vcpen
	tThresold := (q.params.WeightToi/minPen - 0.1) * q.params.HorizTime
	if tThresold-q.params.HorizTime > -1e-6 {
		return minPenalty
	}

	// Find min time of impact and exit amongst all obstacles.
	tmin := q.params.HorizTime
	side := float32(0)
	nside := 0

	for i := 0; i < q.ncircles; i++ {
		cir := &q.circles[i]

		// RVO — fully inlined to avoid [3]float32 temp allocations
		vabX := vcand[0]*2 - vel[0] - cir.Vel[0]
		_ = vcand[1]*2 - vel[1] - cir.Vel[1]
		vabZ := vcand[2]*2 - vel[2] - cir.Vel[2]

		// Side — inline vecDot2D, replace math.Min + float64 conversions with branch
		d0 := cir.Dp[0]*vabX + cir.Dp[2]*vabZ
		d1 := cir.Np[0]*vabX + cir.Np[2]*vabZ
		s := d0*0.5 + 0.5
		if d1*2 < s {
			s = d1 * 2
		}
		if s < 0 {
			s = 0
		} else if s > 1 {
			s = 1
		}
		side += s
		nside++

		// Inline sweepCircleCircle
		{
			const eps = 0.0001
			sx := cir.P[0] - pos[0]
			_ = cir.P[1] - pos[1]
			sz := cir.P[2] - pos[2]
			rr := rad + cir.Rad
			cc := sx*sx + sz*sz - rr*rr
			aa := vabX*vabX + vabZ*vabZ
			if aa >= eps {
				bb := vabX*sx + vabZ*sz
				dd := bb*bb - aa*cc
				if dd >= 0 {
					aaInv := 1.0 / aa
					rd := float32(math.Sqrt(float64(dd)))
					htmin := (bb - rd) * aaInv
					htmax := (bb + rd) * aaInv

					if htmin < 0 && htmax > 0 {
						htmin = -htmin * 0.5
					}
					if htmin >= 0 {
						if htmin < tmin {
							tmin = htmin
							if tmin < tThresold {
								return minPenalty
							}
						}
					}
				}
			}
		}
	}

	for i := 0; i < q.nsegments; i++ {
		seg := &q.segments[i]
		var htmin float32

		if seg.Touch {
			// Special case when the agent is very close to the segment.
			sdir := vecSub(seg.Q, seg.P)
			var snorm [3]float32
			snorm[0] = -sdir[2]
			snorm[2] = sdir[0]
			// If the velocity is pointing towards the segment, no collision.
			if vecDot2D(snorm, vcand) < 0 {
				continue
			}
			// Else immediate collision.
			htmin = 0
		} else {
			// Inline isectRaySeg — fully inlined to avoid [3]float32 temps
			vx := seg.Q[0] - seg.P[0]
			vz := seg.Q[2] - seg.P[2]
			wx := pos[0] - seg.P[0]
			wz := pos[2] - seg.P[2]
			d := vcand[0]*vz - vcand[2]*vx
			if float32(math.Abs(float64(d))) < 1e-6 {
				continue
			}
			nx := vcand[0]*wz - vcand[2]*wx
			htmin = -nx / d
			if htmin < 0 {
				continue
			}
		}

		// Avoid less when facing walls.
		htmin *= 2.0

		if htmin < tmin {
			tmin = htmin
			if tmin < tThresold {
				return minPenalty
			}
		}
	}

	// Normalize side bias, to prevent it dominating too much.
	if nside > 0 {
		side /= float32(nside)
	}

	spen := q.params.WeightSide * side
	tpen := q.params.WeightToi * (1.0 / (0.1 + tmin*q.invHorizTime))

	penalty := vpen + vcpen + spen + tpen

	// Store different penalties for debug viewing
	if debug != nil {
		debug.AddSample(&vcand, cs, penalty, vpen, vcpen, spen, tpen)
	}

	return penalty
}

// SampleVelocityGrid samples velocities on a grid pattern.
func (q *ObstacleAvoidanceQuery) SampleVelocityGrid(pos [3]float32, rad, vmax float32,
	vel, dvel [3]float32,
	params *ObstacleAvoidanceParams, debug *ObstacleAvoidanceDebugData) ([3]float32, int) {

	q.prepare(pos, dvel)

	q.params = *params
	q.invHorizTime = 1.0 / q.params.HorizTime
	q.vmax = vmax
	if vmax > 0 {
		q.invVmax = 1.0 / vmax
	} else {
		q.invVmax = float32(math.MaxFloat32)
	}

	nvel := [3]float32{}

	if debug != nil {
		debug.Reset()
	}

	cvx := dvel[0] * q.params.VelBias
	cvz := dvel[2] * q.params.VelBias
	cs := vmax * 2 * (1 - q.params.VelBias) / float32(q.params.GridSize-1)
	half := float32(q.params.GridSize-1) * cs * 0.5

	minPenalty := float32(math.MaxFloat32)
	ns := 0

	for y := uint8(0); y < q.params.GridSize; y++ {
		for x := uint8(0); x < q.params.GridSize; x++ {
			var vcand [3]float32
			vcand[0] = cvx + float32(x)*cs - half
			vcand[1] = 0
			vcand[2] = cvz + float32(y)*cs - half

			if vcand[0]*vcand[0]+vcand[2]*vcand[2] > (vmax+cs*0.5)*(vmax+cs*0.5) {
				continue
			}

			penalty := q.ProcessSample(vcand, cs, pos, rad, vel, dvel, minPenalty, debug)
			ns++
			if penalty < minPenalty {
				minPenalty = penalty
				nvel = vcand
			}
		}
	}

	return nvel, ns
}

// SampleVelocityAdaptive samples velocities using an adaptive pattern.
func (q *ObstacleAvoidanceQuery) SampleVelocityAdaptive(pos [3]float32, rad, vmax float32,
	vel, dvel [3]float32,
	params *ObstacleAvoidanceParams, debug *ObstacleAvoidanceDebugData) ([3]float32, int) {

	q.prepare(pos, dvel)

	q.params = *params
	q.invHorizTime = 1.0 / q.params.HorizTime
	q.vmax = vmax
	if vmax > 0 {
		q.invVmax = 1.0 / vmax
	} else {
		q.invVmax = float32(math.MaxFloat32)
	}

	nvel := [3]float32{}

	if debug != nil {
		debug.Reset()
	}

	// Build sampling pattern aligned to desired velocity.
	pat := make([]float32, (MaxPatternDivs*MaxPatternRings+1)*2)
	npat := 0

	ndivs := int(q.params.AdaptiveDivs)
	nrings := int(q.params.AdaptiveRings)
	depth := int(q.params.AdaptiveDepth)

	nd := clampInt(ndivs, 1, MaxPatternDivs)
	nr := clampInt(nrings, 1, MaxPatternRings)
	da := (1.0 / float32(nd)) * pi * 2
	ca := float32(math.Cos(float64(da)))
	sa := float32(math.Sin(float64(da)))

	// Desired direction
	var ddir0, ddir1 [3]float32
	ddir0[0] = dvel[0]
	ddir0[1] = dvel[1]
	ddir0[2] = dvel[2]
	ddir0 = vecNormalize2D(ddir0)
	ddir1 = rotate2D(ddir0, da*0.5)

	// Always add sample at zero
	pat[npat*2+0] = 0
	pat[npat*2+1] = 0
	npat++

	for j := 0; j < nr; j++ {
		r := float32(nr-j) / float32(nr)
		dir := [2][3]float32{ddir0, ddir1}
		pat[npat*2+0] = dir[j%2][0] * r
		pat[npat*2+1] = dir[j%2][2] * r
		last1 := pat[npat*2:]
		last2 := last1
		npat++

		for i := 1; i < nd-1; i += 2 {
			// Get next point on the "right" (rotate CW)
			pat[npat*2+0] = last1[0]*ca + last1[1]*sa
			pat[npat*2+1] = -last1[0]*sa + last1[1]*ca
			// Get next point on the "left" (rotate CCW)
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca

			last1 = pat[npat*2:]
			last2 = pat[npat*2+2:]
			npat += 2
		}

		if nd&1 == 0 {
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca
			npat++
		}
	}

	// Start sampling.
	cr := vmax * (1.0 - q.params.VelBias)
	var res [3]float32
	res[0] = dvel[0] * q.params.VelBias
	res[1] = 0
	res[2] = dvel[2] * q.params.VelBias
	ns := 0

	for k := 0; k < depth; k++ {
		minPenalty := float32(math.MaxFloat32)
		var bvel [3]float32

		for i := 0; i < npat; i++ {
			var vcand [3]float32
			vcand[0] = res[0] + pat[i*2+0]*cr
			vcand[1] = 0
			vcand[2] = res[2] + pat[i*2+1]*cr

			if vcand[0]*vcand[0]+vcand[2]*vcand[2] > (vmax+0.001)*(vmax+0.001) {
				continue
			}

			penalty := q.ProcessSample(vcand, cr/10, pos, rad, vel, dvel, minPenalty, debug)
			ns++
			if penalty < minPenalty {
				minPenalty = penalty
				bvel = vcand
			}
		}

		res = bvel
		cr *= 0.5
	}

	nvel = res

	return nvel, ns
}

// GetObstacleCircleCount returns the number of obstacle circles.
func (q *ObstacleAvoidanceQuery) GetObstacleCircleCount() int {
	return q.ncircles
}

// GetObstacleCircle returns the obstacle circle at the given index.
func (q *ObstacleAvoidanceQuery) GetObstacleCircle(i int) *ObstacleCircle {
	return &q.circles[i]
}

// GetObstacleSegmentCount returns the number of obstacle segments.
func (q *ObstacleAvoidanceQuery) GetObstacleSegmentCount() int {
	return q.nsegments
}

// GetObstacleSegment returns the obstacle segment at the given index.
func (q *ObstacleAvoidanceQuery) GetObstacleSegment(i int) *ObstacleSegment {
	return &q.segments[i]
}

// --- Internal helper functions ---

func sweepCircleCircle(c0 [3]float32, r0 float32, v, c1 [3]float32, r1 float32) (bool, float32, float32) {
	const eps = 0.0001
	s := vecSub(c1, c0)
	r := r0 + r1
	c := vecDot2D(s, s) - r*r
	a := vecDot2D(v, v)
	if a < eps {
		return false, 0, 0
	}

	b := vecDot2D(v, s)
	d := b*b - a*c
	if d < 0 {
		return false, 0, 0
	}
	a = 1.0 / a
	rd := float32(math.Sqrt(float64(d)))
	tmin := (b - rd) * a
	tmax := (b + rd) * a
	return true, tmin, tmax
}

func isectRaySeg(ap, u, bp, bq [3]float32) (bool, float32) {
	v := vecSub(bq, bp)
	w := vecSub(ap, bp)
	d := vecPerp2D(u, v)
	if math.Abs(float64(d)) < 1e-6 {
		return false, 0
	}
	d = 1.0 / d
	t := vecPerp2D(v, w) * d
	if t < 0 || t > 1 {
		return false, 0
	}
	s := vecPerp2D(u, w) * d
	if s < 0 || s > 1 {
		return false, 0
	}
	return true, t
}

func vecNormalize2D(v [3]float32) [3]float32 {
	// v has x and z components at indices 0 and 2
	d := float32(math.Sqrt(float64(v[0]*v[0] + v[2]*v[2])))
	if d == 0 {
		return v
	}
	d = 1.0 / d
	return [3]float32{v[0] * d, v[1], v[2] * d}
}

func rotate2D(v [3]float32, ang float32) [3]float32 {
	c := float32(math.Cos(float64(ang)))
	s := float32(math.Sin(float64(ang)))
	return [3]float32{v[0]*c - v[2]*s, v[1], v[0]*s + v[2]*c}
}

func clampInt(v, min, max int) int {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}
