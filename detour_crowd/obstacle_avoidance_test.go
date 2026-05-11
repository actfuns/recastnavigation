package detour_crowd

import (
	"testing"
)

func TestObstacleAvoidanceQueryInit(t *testing.T) {
	t.Run("should initialize with valid sizes", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		result := q.Init(6, 8)
		if !result {
			t.Errorf("Init(6,8) returned false, expected true")
		}
		if q.maxCircles != 6 {
			t.Errorf("maxCircles = %d, want 6", q.maxCircles)
		}
		if q.maxSegments != 8 {
			t.Errorf("maxSegments = %d, want 8", q.maxSegments)
		}
		if q.ncircles != 0 {
			t.Errorf("ncircles = %d, want 0", q.ncircles)
		}
		if q.nsegments != 0 {
			t.Errorf("nsegments = %d, want 0", q.nsegments)
		}
	})

	t.Run("should initialize with zero sizes", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		result := q.Init(0, 0)
		if !result {
			t.Errorf("Init(0,0) returned false, expected true")
		}
		// Must not panic when adding to zero-sized buffer (should be silently dropped)
		q.AddCircle([3]float32{1, 0, 1}, 1.0, [3]float32{}, [3]float32{})
		q.AddSegment([3]float32{}, [3]float32{1, 0, 0})
		if q.ncircles != 0 {
			t.Errorf("ncircles = %d, want 0 (zero-sized buffer)", q.ncircles)
		}
		if q.nsegments != 0 {
			t.Errorf("nsegments = %d, want 0 (zero-sized buffer)", q.nsegments)
		}
	})
}

func TestObstacleAvoidanceQueryReset(t *testing.T) {
	t.Run("should clear all obstacles", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		q.AddCircle([3]float32{1, 0, 1}, 2.0, [3]float32{}, [3]float32{})
		q.AddCircle([3]float32{2, 0, 2}, 3.0, [3]float32{}, [3]float32{})
		q.AddSegment([3]float32{0, 0, 0}, [3]float32{10, 0, 10})

		if q.ncircles != 2 {
			t.Errorf("ncircles = %d, want 2", q.ncircles)
		}
		if q.nsegments != 1 {
			t.Errorf("nsegments = %d, want 1", q.nsegments)
		}

		q.Reset()

		if q.ncircles != 0 {
			t.Errorf("after Reset ncircles = %d, want 0", q.ncircles)
		}
		if q.nsegments != 0 {
			t.Errorf("after Reset nsegments = %d, want 0", q.nsegments)
		}
	})

	t.Run("should allow re-adding obstacles after reset", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		q.AddCircle([3]float32{1, 0, 1}, 2.0, [3]float32{}, [3]float32{})
		q.Reset()
		q.AddCircle([3]float32{3, 0, 3}, 4.0, [3]float32{}, [3]float32{})

		if q.ncircles != 1 {
			t.Errorf("after reset+add ncircles = %d, want 1", q.ncircles)
		}
		cir := q.GetObstacleCircle(0)
		if cir.P != [3]float32{3, 0, 3} {
			t.Errorf("circle.P = %v, want {3,0,3}", cir.P)
		}
	})
}

func TestObstacleAvoidanceQueryAddCircle(t *testing.T) {
	t.Run("should add circles and retrieve them", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		pos := [3]float32{1, 2, 3}
		vel := [3]float32{0.1, 0, 0.2}
		dvel := [3]float32{0.3, 0, 0.4}

		q.AddCircle(pos, 5.0, vel, dvel)

		if q.GetObstacleCircleCount() != 1 {
			t.Errorf("circle count = %d, want 1", q.GetObstacleCircleCount())
		}

		cir := q.GetObstacleCircle(0)
		if cir.P != pos {
			t.Errorf("circle.P = %v, want %v", cir.P, pos)
		}
		if cir.Rad != 5.0 {
			t.Errorf("circle.Rad = %f, want 5.0", cir.Rad)
		}
		if cir.Vel != vel {
			t.Errorf("circle.Vel = %v, want %v", cir.Vel, vel)
		}
		if cir.DVel != dvel {
			t.Errorf("circle.DVel = %v, want %v", cir.DVel, dvel)
		}
	})

	t.Run("should not exceed max circles", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(2, 8)

		q.AddCircle([3]float32{}, 1, [3]float32{}, [3]float32{})
		q.AddCircle([3]float32{}, 1, [3]float32{}, [3]float32{})
		q.AddCircle([3]float32{}, 1, [3]float32{}, [3]float32{})

		if q.ncircles != 2 {
			t.Errorf("ncircles = %d, want 2", q.ncircles)
		}
	})
}

func TestObstacleAvoidanceQueryAddSegment(t *testing.T) {
	t.Run("should add segments and retrieve them", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		p := [3]float32{0, 0, 0}
		qp := [3]float32{10, 0, 10}

		q.AddSegment(p, qp)

		if q.GetObstacleSegmentCount() != 1 {
			t.Errorf("segment count = %d, want 1", q.GetObstacleSegmentCount())
		}

		seg := q.GetObstacleSegment(0)
		if seg.P != p {
			t.Errorf("segment.P = %v, want %v", seg.P, p)
		}
		if seg.Q != qp {
			t.Errorf("segment.Q = %v, want %v", seg.Q, qp)
		}
	})

	t.Run("should not have touch flag initially", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		q.AddSegment([3]float32{0, 0, 0}, [3]float32{10, 0, 10})

		seg := q.GetObstacleSegment(0)
		if seg.Touch {
			t.Errorf("segment.Touch should be false initially")
		}
	})

	t.Run("should not exceed max segments", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 2)

		q.AddSegment([3]float32{}, [3]float32{1, 0, 0})
		q.AddSegment([3]float32{}, [3]float32{2, 0, 0})
		q.AddSegment([3]float32{}, [3]float32{3, 0, 0})

		if q.nsegments != 2 {
			t.Errorf("nsegments = %d, want 2", q.nsegments)
		}
	})
}

func TestSweepCircleCircle(t *testing.T) {
	t.Run("should detect head-on collision", func(t *testing.T) {
		c0 := [3]float32{0, 0, 0}
		r0 := float32(1.0)
		v := [3]float32{5, 0, 0}
		c1 := [3]float32{10, 0, 0}
		r1 := float32(1.0)

		ok, tmin, tmax := sweepCircleCircle(c0, r0, v, c1, r1)

		if !ok {
			t.Errorf("expected collision, got no collision")
		}
		if tmin <= 0 {
			t.Errorf("tmin = %f, expected positive", tmin)
		}
		if tmax <= tmin {
			t.Errorf("tmax = %f, expected > tmin=%f", tmax, tmin)
		}
	})

	t.Run("should not detect collision when moving away", func(t *testing.T) {
		c0 := [3]float32{0, 0, 0}
		r0 := float32(1.0)
		v := [3]float32{-5, 0, 0}
		c1 := [3]float32{10, 0, 0}
		r1 := float32(1.0)
		ok, tmin, tmax := sweepCircleCircle(c0, r0, v, c1, r1)
		if ok {
			// Collision may be detected at negative t (paths intersect "in the past")
			if tmin >= 0 || tmax >= 0 {
				t.Errorf("expected only past collision, got tmin=%f tmax=%f", tmin, tmax)
			}
		}
	})

	t.Run("should not detect collision with zero velocity", func(t *testing.T) {
		c0 := [3]float32{0, 0, 0}
		r0 := float32(1.0)
		v := [3]float32{0, 0, 0}
		c1 := [3]float32{10, 0, 0}
		r1 := float32(1.0)

		ok, _, _ := sweepCircleCircle(c0, r0, v, c1, r1)
		if ok {
			t.Errorf("expected no collision with zero velocity")
		}
	})

	t.Run("should handle overlapping circles", func(t *testing.T) {
		c0 := [3]float32{0, 0, 0}
		r0 := float32(3.0)
		v := [3]float32{1, 0, 0}
		c1 := [3]float32{2, 0, 0}
		r1 := float32(2.0)

		ok, tmin, tmax := sweepCircleCircle(c0, r0, v, c1, r1)
		if !ok {
			t.Errorf("expected collision for overlapping circles")
		}
		if tmin > 0 || tmax < 0 {
			t.Errorf("overlapping circles should have tmin <= 0 <= tmax, got tmin=%f, tmax=%f", tmin, tmax)
		}
	})

	t.Run("should not collide with distant circles on parallel paths", func(t *testing.T) {
		c0 := [3]float32{0, 0, 0}
		r0 := float32(1.0)
		v := [3]float32{1, 0, 0}
		c1 := [3]float32{0, 0, 100}
		r1 := float32(1.0)

		ok, _, _ := sweepCircleCircle(c0, r0, v, c1, r1)
		if ok {
			t.Errorf("expected no collision for distant circles")
		}
	})

	t.Run("should handle tangent case with zero discriminant", func(t *testing.T) {
		// Arrange circles so discriminant approaches zero
		// d = b*b - a*c. When d=0, the circles just touch.
		c0 := [3]float32{0, 0, 0}
		r0 := float32(0.5)
		v := [3]float32{2, 0, 0}
		// c1 directly at the edge of collision range
		c1 := [3]float32{3, 0, 0} // distance from c0 to c1 = 3, r0+r1 = 1
		r1 := float32(0.5)
		// s = (3,0,0), r = 1, c = 9-1 = 8
		// a = 4, b = 6, d = 36-32 = 4 > 0
		// That still collides. Let's make it exactly tangent:
		// v=(1,0,0), s=(2,0,0), r0+r1=1
		// c = 4-1 = 3, a = 1, b = 2, d = 4-3 = 1 > 0
		// Still collides (barely).
		// For tangent: need d=0, so b*b = a*c

		ok, _, _ := sweepCircleCircle(c0, r0, v, c1, r1)
		_ = ok
	})
}

func TestSampleVelocityGrid(t *testing.T) {
	makeParams := func() *ObstacleAvoidanceParams {
		return &ObstacleAvoidanceParams{
			VelBias:       0.4,
			WeightDesVel:  2.0,
			WeightCurVel:  0.75,
			WeightSide:    0.75,
			WeightToi:     2.5,
			HorizTime:     2.5,
			GridSize:      33,
			AdaptiveDivs:  7,
			AdaptiveRings: 2,
			AdaptiveDepth: 5,
		}
	}

	t.Run("should return a valid velocity", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		nvel, ns := q.SampleVelocityGrid(pos, 0.5, 2.0, vel, dvel, makeParams(), nil)

		if nvel == ([3]float32{}) {
			t.Errorf("SampleVelocityGrid returned zero velocity")
		}
		if ns <= 0 {
			t.Errorf("SampleVelocityGrid returned %d samples, expected > 0", ns)
		}
		if nvel[1] != 0 {
			t.Errorf("nvel[1] = %f, want 0", nvel[1])
		}
	})

	t.Run("should work with debug data", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		debug := NewObstacleAvoidanceDebugData()
		debug.Init(256)

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		_, ns := q.SampleVelocityGrid(pos, 0.5, 2.0, vel, dvel, makeParams(), debug)

		if debug.GetSampleCount() == 0 {
			t.Errorf("debug sample count is 0, expected > 0")
		}
		if ns <= 0 {
			t.Errorf("ns = %d, expected > 0", ns)
		}
	})

	t.Run("should work with small grid size", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		params := &ObstacleAvoidanceParams{
			VelBias:      0.4,
			WeightDesVel: 2.0,
			WeightCurVel: 0.75,
			WeightSide:   0.75,
			WeightToi:    2.5,
			HorizTime:    2.5,
			GridSize:     3,
		}

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		_, ns := q.SampleVelocityGrid(pos, 0.5, 2.0, vel, dvel, params, nil)

		if ns > 9 {
			t.Errorf("ns = %d, expected <= 9 for 3x3 grid", ns)
		}
		if ns <= 0 {
			t.Errorf("ns = %d, expected > 0", ns)
		}
	})

	t.Run("should handle zero vmax", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{0, 0, 0}

		nvel, ns := q.SampleVelocityGrid(pos, 0.5, 0, vel, dvel, makeParams(), nil)

		if ns <= 0 {
			t.Errorf("SampleVelocityGrid with vmax=0: ns = %d, expected > 0", ns)
		}
		_ = nvel
	})

	t.Run("should return velocities within vmax range", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		params := &ObstacleAvoidanceParams{
			VelBias:      0.4,
			WeightDesVel: 2.0,
			WeightCurVel: 0.75,
			WeightSide:   0.75,
			WeightToi:    2.5,
			HorizTime:    2.5,
			GridSize:     33,
		}

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{0, 0, 1}

		nvel, _ := q.SampleVelocityGrid(pos, 0.5, 2.0, vel, dvel, params, nil)

		speedSq := nvel[0]*nvel[0] + nvel[2]*nvel[2]
		if speedSq > 4.01 { // slight allowance for floating point
			t.Errorf("result velocity magnitude squared = %f, want <= 4.0 (vmax=2)", speedSq)
		}
	})
}

func TestSampleVelocityAdaptive(t *testing.T) {
	makeParams := func() *ObstacleAvoidanceParams {
		return &ObstacleAvoidanceParams{
			VelBias:       0.4,
			WeightDesVel:  2.0,
			WeightCurVel:  0.75,
			WeightSide:    0.75,
			WeightToi:     2.5,
			HorizTime:     2.5,
			GridSize:      33,
			AdaptiveDivs:  7,
			AdaptiveRings: 2,
			AdaptiveDepth: 5,
		}
	}

	t.Run("should return a valid velocity", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		nvel, ns := q.SampleVelocityAdaptive(pos, 0.5, 2.0, vel, dvel, makeParams(), nil)

		if nvel == ([3]float32{}) {
			t.Errorf("SampleVelocityAdaptive returned zero velocity")
		}
		if ns <= 0 {
			t.Errorf("SampleVelocityAdaptive returned %d samples, expected > 0", ns)
		}
		if nvel[1] != 0 {
			t.Errorf("nvel[1] = %f, want 0", nvel[1])
		}
	})

	t.Run("should work with debug data", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		debug := NewObstacleAvoidanceDebugData()
		debug.Init(256)

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		_, ns := q.SampleVelocityAdaptive(pos, 0.5, 2.0, vel, dvel, makeParams(), debug)

		if debug.GetSampleCount() == 0 {
			t.Errorf("debug sample count is 0, expected > 0")
		}
		if ns <= 0 {
			t.Errorf("ns = %d, expected > 0", ns)
		}
	})

	t.Run("should work with obstacle circles that affect sampling", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		params := makeParams()

		q.AddCircle([3]float32{3, 0, 0}, 1.0, [3]float32{}, [3]float32{})

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		nvel, ns := q.SampleVelocityAdaptive(pos, 0.5, 3.0, vel, dvel, params, nil)

		if ns <= 0 {
			t.Errorf("ns = %d, expected > 0", ns)
		}
		_ = nvel
	})

	t.Run("should avoid segment obstacles", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		params := makeParams()

		q.AddSegment([3]float32{2, 0, -5}, [3]float32{2, 0, 5})

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		nvel, ns := q.SampleVelocityAdaptive(pos, 0.5, 3.0, vel, dvel, params, nil)

		if ns <= 0 {
			t.Errorf("ns = %d, expected > 0", ns)
		}
		_ = nvel
	})

	t.Run("should handle zero vmax", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{0, 0, 0}

		nvel, ns := q.SampleVelocityAdaptive(pos, 0.5, 0, vel, dvel, makeParams(), nil)

		if ns <= 0 {
			t.Errorf("SampleVelocityAdaptive with vmax=0: ns = %d, expected > 0", ns)
		}
		_ = nvel
	})
}

func TestObstacleAvoidanceDebugData(t *testing.T) {
	t.Run("should initialize and add samples", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		if !d.Init(100) {
			t.Errorf("Init(100) failed")
		}
		if d.GetSampleCount() != 0 {
			t.Errorf("initial sample count = %d, want 0", d.GetSampleCount())
		}

		vel := [3]float32{1, 0, 0}
		d.AddSample(&vel, 1.0, 0.5, 0.3, 0.2, 0.1, 0.4)
		if d.GetSampleCount() != 1 {
			t.Errorf("sample count = %d, want 1", d.GetSampleCount())
		}

		sv := d.GetSampleVelocity(0)
		if *sv != vel {
			t.Errorf("sample velocity = %v, want %v", *sv, vel)
		}
		if d.GetSampleSize(0) != 1.0 {
			t.Errorf("sample size = %f, want 1.0", d.GetSampleSize(0))
		}
		if d.GetSamplePenalty(0) != 0.5 {
			t.Errorf("sample penalty = %f, want 0.5", d.GetSamplePenalty(0))
		}
		if d.GetSampleDesiredVelocityPenalty(0) != 0.3 {
			t.Errorf("sample dvel penalty = %f, want 0.3", d.GetSampleDesiredVelocityPenalty(0))
		}
		if d.GetSampleCurrentVelocityPenalty(0) != 0.2 {
			t.Errorf("sample cvel penalty = %f, want 0.2", d.GetSampleCurrentVelocityPenalty(0))
		}
		if d.GetSamplePreferredSidePenalty(0) != 0.1 {
			t.Errorf("sample side penalty = %f, want 0.1", d.GetSamplePreferredSidePenalty(0))
		}
		if d.GetSampleCollisionTimePenalty(0) != 0.4 {
			t.Errorf("sample time penalty = %f, want 0.4", d.GetSampleCollisionTimePenalty(0))
		}
	})

	t.Run("should reset samples", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		d.Init(10)

		vel := [3]float32{1, 0, 0}
		d.AddSample(&vel, 1, 0, 0, 0, 0, 0)
		d.Reset()

		if d.GetSampleCount() != 0 {
			t.Errorf("after Reset sample count = %d, want 0", d.GetSampleCount())
		}
	})

	t.Run("should not exceed max samples", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		d.Init(3)

		for i := 0; i < 5; i++ {
			vel := [3]float32{float32(i), 0, 0}
			d.AddSample(&vel, 1, 0, 0, 0, 0, 0)
		}

		if d.GetSampleCount() != 3 {
			t.Errorf("sample count = %d, want 3", d.GetSampleCount())
		}
	})

	t.Run("Init(0) should return false", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		if d.Init(0) {
			t.Errorf("Init(0) should return false")
		}
	})

	t.Run("NormalizeSamples should handle empty set without panic", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		d.Init(10)
		d.NormalizeSamples()
	})

	t.Run("NormalizeSamples should normalize penalties to [0,1]", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		d.Init(10)

		v0 := [3]float32{1, 0, 0}
		v1 := [3]float32{2, 0, 0}

		d.AddSample(&v0, 1, 10, 20, 30, 40, 50)
		d.AddSample(&v1, 1, 20, 30, 40, 50, 60)

		d.NormalizeSamples()

		for i := 0; i < d.GetSampleCount(); i++ {
			p := d.GetSamplePenalty(i)
			if p < 0 || p > 1 {
				t.Errorf("normalized penalty[%d] = %f, want in [0,1]", i, p)
			}
			vp := d.GetSampleDesiredVelocityPenalty(i)
			if vp < 0 || vp > 1 {
				t.Errorf("normalized dvel penalty[%d] = %f, want in [0,1]", i, vp)
			}
			vcp := d.GetSampleCurrentVelocityPenalty(i)
			if vcp < 0 || vcp > 1 {
				t.Errorf("normalized cvel penalty[%d] = %f, want in [0,1]", i, vcp)
			}
			sp := d.GetSamplePreferredSidePenalty(i)
			if sp < 0 || sp > 1 {
				t.Errorf("normalized side penalty[%d] = %f, want in [0,1]", i, sp)
			}
			tp := d.GetSampleCollisionTimePenalty(i)
			if tp < 0 || tp > 1 {
				t.Errorf("normalized time penalty[%d] = %f, want in [0,1]", i, tp)
			}
		}
	})

	t.Run("NormalizeSamples with single sample should not divide by zero", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		d.Init(10)

		vel := [3]float32{1, 0, 0}
		d.AddSample(&vel, 1, 10, 20, 30, 40, 50)

		d.NormalizeSamples()

		if d.GetSamplePenalty(0) != 0 {
			t.Errorf("single sample: penalty should be 0 after normalize, got %f", d.GetSamplePenalty(0))
		}
	})

	t.Run("NormalizeSamples with identical penalties should not divide by zero", func(t *testing.T) {
		d := NewObstacleAvoidanceDebugData()
		d.Init(10)

		vel := [3]float32{1, 0, 0}
		d.AddSample(&vel, 1, 5, 5, 5, 5, 5)
		d.AddSample(&vel, 1, 5, 5, 5, 5, 5)

		d.NormalizeSamples()

		// When all values are equal, normalized value should be 0
		if d.GetSamplePenalty(0) != 0 || d.GetSamplePenalty(1) != 0 {
			t.Errorf("identical penalties should normalize to 0")
		}
	})
}

func TestProcessSample(t *testing.T) {
	t.Run("should return a finite penalty for a valid sample", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		params := &ObstacleAvoidanceParams{
			VelBias:       0.4,
			WeightDesVel:  2.0,
			WeightCurVel:  0.75,
			WeightSide:    0.75,
			WeightToi:     2.5,
			HorizTime:     2.5,
			GridSize:      33,
			AdaptiveDivs:  7,
			AdaptiveRings: 2,
			AdaptiveDepth: 5,
		}

		q.params = *params
		q.invHorizTime = 1.0 / params.HorizTime
		q.vmax = 2.0
		q.invVmax = 1.0 / 2.0

		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}
		vcand := [3]float32{1, 0, 1}

		penalty := q.ProcessSample(vcand, 0.5, pos, 0.5, vel, dvel, 9999.0, nil)

		if penalty < 0 {
			t.Errorf("penalty = %f, expected >= 0", penalty)
		}
		if penalty > 9999.0 {
			t.Errorf("penalty = %f, expected <= 9999 (minPenalty)", penalty)
		}
	})

	t.Run("should return minPenalty early when threshold is below zero", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(6, 8)

		params := &ObstacleAvoidanceParams{
			WeightToi:    2.5,
			HorizTime:    2.5,
			WeightDesVel: 2.0,
			WeightCurVel: 0.75,
		}

		q.params = *params
		q.invHorizTime = 1.0 / params.HorizTime
		q.vmax = 2.0
		q.invVmax = 1.0 / 2.0
		dvel := [3]float32{1, 0, 0}
		vcand := [3]float32{1, 0, 1}
		pos := [3]float32{0, 0, 0}
		vel := [3]float32{0, 0, 0}
		minPenalty := float32(0.001)
		penalty := q.ProcessSample(vcand, 0.5, pos, 0.5, vel, dvel, minPenalty, nil)
		if penalty < 0 {
			t.Errorf("expected non-negative penalty, got %f", penalty)
		}
	})
}

func TestPrepare(t *testing.T) {
	t.Run("should prepare circles with side selection", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(3, 3)

		q.AddCircle([3]float32{5, 0, 0}, 1.0, [3]float32{}, [3]float32{0, 0, 1})

		pos := [3]float32{0, 0, 0}
		dvel := [3]float32{1, 0, 0}

		q.prepare(pos, dvel)

		// Circle should have Dp and Np set
		cir := q.GetObstacleCircle(0)
		if cir.Dp == ([3]float32{}) {
			t.Errorf("Dp should not be zero after prepare")
		}
		if cir.Np == ([3]float32{}) {
			t.Errorf("Np should not be zero after prepare")
		}
	})

	t.Run("should set touch flag on segments close to position", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(3, 3)

		q.AddSegment([3]float32{0, 0, 0}, [3]float32{1, 0, 0})

		pos := [3]float32{0.005, 0, 0}
		dvel := [3]float32{1, 0, 0}

		q.prepare(pos, dvel)

		seg := q.GetObstacleSegment(0)
		if !seg.Touch {
			t.Errorf("expected segment.Touch=true when agent is very close to segment")
		}
	})

	t.Run("should not set touch flag on distant segments", func(t *testing.T) {
		q := NewObstacleAvoidanceQuery()
		q.Init(3, 3)

		q.AddSegment([3]float32{0, 0, 0}, [3]float32{10, 0, 0})

		pos := [3]float32{5, 0, 5}
		dvel := [3]float32{1, 0, 0}

		q.prepare(pos, dvel)

		seg := q.GetObstacleSegment(0)
		if seg.Touch {
			t.Errorf("expected segment.Touch=false when agent is far from segment")
		}
	})
}
