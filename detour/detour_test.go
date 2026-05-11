package detour

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

// --- Status helpers ---
func TestVcross(t *testing.T) {
	v1 := []float32{3, -3, 1}
	v2 := []float32{4, 9, 2}
	dest := make([]float32, 3)
	Vcross(dest, v1, v2)
	assert.InDelta(t, -15.0, dest[0], 0.0001)
	assert.InDelta(t, -2.0, dest[1], 0.0001)
	assert.InDelta(t, 39.0, dest[2], 0.0001)

	Vcross(dest, v1, v1)
	assert.InDelta(t, 0.0, dest[0], 0.0001)
	assert.InDelta(t, 0.0, dest[1], 0.0001)
	assert.InDelta(t, 0.0, dest[2], 0.0001)
}

func TestVdot(t *testing.T) {
	v1 := []float32{1, 0, 0}
	assert.InDelta(t, 1.0, Vdot(v1, v1), 0.0001)

	v2 := []float32{0, 0, 0}
	assert.InDelta(t, 0.0, Vdot(v1, v2), 0.0001)

	v3 := []float32{1, 2, 3}
	v4 := []float32{4, 5, 6}
	assert.InDelta(t, 32.0, Vdot(v3, v4), 0.0001) // 1*4 + 2*5 + 3*6 = 32
}

func TestVmad(t *testing.T) {
	v1 := []float32{1, 2, 3}
	v2 := []float32{0, 2, 4}
	dest := make([]float32, 3)
	Vmad(dest, v1, v2, 2)
	assert.InDelta(t, 1.0, dest[0], 0.0001)
	assert.InDelta(t, 6.0, dest[1], 0.0001)
	assert.InDelta(t, 11.0, dest[2], 0.0001)

	Vmad(dest, v1, v2, 0)
	assert.InDelta(t, 1.0, dest[0], 0.0001)
	assert.InDelta(t, 2.0, dest[1], 0.0001)
	assert.InDelta(t, 3.0, dest[2], 0.0001)
}

func TestVlerp(t *testing.T) {
	v1 := []float32{0, 0, 0}
	v2 := []float32{10, 10, 10}
	dest := make([]float32, 3)

	Vlerp(dest, v1, v2, 0.5)
	assert.InDelta(t, 5.0, dest[0], 0.0001)
	assert.InDelta(t, 5.0, dest[1], 0.0001)
	assert.InDelta(t, 5.0, dest[2], 0.0001)

	Vlerp(dest, v1, v2, 0)
	assert.InDelta(t, 0.0, dest[0], 0.0001)

	Vlerp(dest, v1, v2, 1)
	assert.InDelta(t, 10.0, dest[0], 0.0001)
}

func TestVadd(t *testing.T) {
	v1 := []float32{1, 2, 3}
	v2 := []float32{5, 6, 7}
	dest := make([]float32, 3)
	Vadd(dest, v1, v2)
	assert.InDelta(t, 6.0, dest[0], 0.0001)
	assert.InDelta(t, 8.0, dest[1], 0.0001)
	assert.InDelta(t, 10.0, dest[2], 0.0001)
}

func TestVsub(t *testing.T) {
	v1 := []float32{5, 4, 3}
	v2 := []float32{1, 2, 3}
	dest := make([]float32, 3)
	Vsub(dest, v1, v2)
	assert.InDelta(t, 4.0, dest[0], 0.0001)
	assert.InDelta(t, 2.0, dest[1], 0.0001)
	assert.InDelta(t, 0.0, dest[2], 0.0001)
}

func TestVscale(t *testing.T) {
	v := []float32{1, 2, 3}
	dest := make([]float32, 3)
	Vscale(dest, v, 2)
	assert.InDelta(t, 2.0, dest[0], 0.0001)
	assert.InDelta(t, 4.0, dest[1], 0.0001)
	assert.InDelta(t, 6.0, dest[2], 0.0001)
}

func TestVmin(t *testing.T) {
	v1 := []float32{5, 4, 0}
	v2 := []float32{1, 2, 9}
	Vmin(v1, v2)
	assert.InDelta(t, 1.0, v1[0], 0.0001)
	assert.InDelta(t, 2.0, v1[1], 0.0001)
	assert.InDelta(t, 0.0, v1[2], 0.0001)
}

func TestVmax(t *testing.T) {
	v1 := []float32{1, 2, 3}
	v2 := []float32{4, 5, 6}
	Vmax(v1, v2)
	assert.InDelta(t, 4.0, v1[0], 0.0001)
	assert.InDelta(t, 5.0, v1[1], 0.0001)
	assert.InDelta(t, 6.0, v1[2], 0.0001)
}

func TestVset(t *testing.T) {
	dest := make([]float32, 3)
	Vset(dest, 1, 2, 3)
	assert.InDelta(t, 1.0, dest[0], 0.0001)
	assert.InDelta(t, 2.0, dest[1], 0.0001)
	assert.InDelta(t, 3.0, dest[2], 0.0001)
}

func TestVcopy(t *testing.T) {
	a := []float32{5, 4, 0}
	dest := make([]float32, 3)
	Vcopy(dest, a)
	assert.InDelta(t, 5.0, dest[0], 0.0001)
	assert.InDelta(t, 4.0, dest[1], 0.0001)
	assert.InDelta(t, 0.0, dest[2], 0.0001)
}

func TestVlen(t *testing.T) {
	v := []float32{3, 4, 0}
	assert.InDelta(t, 5.0, Vlen(v), 0.0001)
	assert.InDelta(t, 0.0, Vlen([]float32{0, 0, 0}), 0.0001)
}

func TestVlenSqr(t *testing.T) {
	assert.InDelta(t, 25.0, VlenSqr([]float32{3, 4, 0}), 0.0001)
	assert.InDelta(t, 0.0, VlenSqr([]float32{0, 0, 0}), 0.0001)
}

func TestVdist(t *testing.T) {
	v1 := []float32{3, 1, 3}
	v2 := []float32{1, 3, 1}
	assert.InDelta(t, 3.4641, Vdist(v1, v2), 0.0001)

	v3 := []float32{0, 0, 0}
	mag := Vlen(v1)
	assert.InDelta(t, mag, Vdist(v1, v3), 0.0001)
}

func TestVdistSqr(t *testing.T) {
	v1 := []float32{3, 1, 3}
	v2 := []float32{1, 3, 1}
	assert.InDelta(t, 12.0, VdistSqr(v1, v2), 0.0001)

	assert.InDelta(t, 0.0, VdistSqr(v1, v1), 0.0001)
}

func TestVdist2D(t *testing.T) {
	v1 := []float32{0, 99, 0}
	v2 := []float32{3, -5, 4}
	assert.InDelta(t, 5.0, Vdist2D(v1, v2), 0.0001)
}

func TestVdist2DSqr(t *testing.T) {
	v1 := []float32{0, 0, 0}
	v2 := []float32{3, 0, 4}
	assert.InDelta(t, 25.0, Vdist2DSqr(v1, v2), 0.0001)
}

func TestVnormalize(t *testing.T) {
	v := []float32{3, 3, 3}
	Vnormalize(v)
	expected := 1.0 / float32(math.Sqrt(3))
	assert.InDelta(t, expected, v[0], 0.0001)
	assert.InDelta(t, expected, v[1], 0.0001)
	assert.InDelta(t, expected, v[2], 0.0001)
	mag := Vlen(v)
	assert.InDelta(t, 1.0, mag, 0.0001)
}

func TestVequal(t *testing.T) {
	v1 := []float32{1, 2, 3}
	v2 := []float32{1, 2, 3}
	assert.True(t, Vequal(v1, v2))

	v3 := []float32{1, 2, 3.1}
	assert.False(t, Vequal(v1, v3))
}

func TestVisfinite(t *testing.T) {
	assert.True(t, Visfinite([]float32{1, 2, 3}))
	assert.False(t, Visfinite([]float32{float32(math.Inf(1)), 2, 3}))
	assert.False(t, Visfinite([]float32{1, float32(math.NaN()), 3}))
	assert.False(t, Visfinite([]float32{1, 2, float32(math.Inf(-1))}))
}

func TestVisfinite2D(t *testing.T) {
	// Only checks x and z (indices 0 and 2)
	assert.True(t, Visfinite2D([]float32{1, float32(math.NaN()), 3}))
	assert.False(t, Visfinite2D([]float32{float32(math.Inf(1)), 2, 3}))
}

func TestVdot2D(t *testing.T) {
	u := []float32{1, 99, 0}
	v := []float32{0, -5, 1}
	assert.InDelta(t, 0.0, Vdot2D(u, v), 0.0001)

	u2 := []float32{3, 0, 4}
	v2 := []float32{3, 0, 4}
	assert.InDelta(t, 25.0, Vdot2D(u2, v2), 0.0001)
}

func TestVperp2D(t *testing.T) {
	u := []float32{1, 0, 0}
	v := []float32{0, 0, 1}
	assert.InDelta(t, -1.0, Vperp2D(u, v), 0.0001) // u[2]*v[0] - u[0]*v[2] = 0*0 - 1*1 = -1

	assert.InDelta(t, -1.0, Vperp2D(u, v), 0.0001)
	assert.InDelta(t, 1.0, Vperp2D(v, u), 0.0001)
}

func TestTriArea2D(t *testing.T) {
	a := []float32{0, 0, 0}
	b := []float32{1, 0, 0}
	c := []float32{0, 0, 1}
	assert.InDelta(t, -1.0, TriArea2D(a, b, c), 0.0001) // acx*abz - abx*acz = 0*0 - 1*1 = -1
}

// --- Bounds overlap ---

func TestOverlapQuantBounds(t *testing.T) {
	assert.True(t, OverlapQuantBounds(
		[]uint16{0, 0, 0}, []uint16{10, 10, 10},
		[]uint16{5, 5, 5}, []uint16{15, 15, 15},
	))
	assert.False(t, OverlapQuantBounds(
		[]uint16{0, 0, 0}, []uint16{10, 10, 10},
		[]uint16{11, 5, 5}, []uint16{15, 15, 15},
	))
	assert.False(t, OverlapQuantBounds(
		[]uint16{0, 0, 0}, []uint16{10, 10, 10},
		[]uint16{5, 5, 11}, []uint16{15, 9, 15},
	))
}

func TestOverlapBounds(t *testing.T) {
	assert.True(t, OverlapBounds(
		[]float32{0, 0, 0}, []float32{10, 10, 10},
		[]float32{5, 5, 5}, []float32{15, 15, 15},
	))
	assert.False(t, OverlapBounds(
		[]float32{0, 0, 0}, []float32{10, 10, 10},
		[]float32{11, 5, 5}, []float32{15, 15, 15},
	))
	assert.True(t, OverlapBounds(
		[]float32{0, 0, 0}, []float32{10, 10, 10},
		[]float32{10, 5, 5}, []float32{15, 15, 15}, // touching edges overlap
	))
}

// --- Geometry queries ---

func TestClosestPtPointTriangle(t *testing.T) {
	a := []float32{0, 0, 0}
	b := []float32{10, 0, 0}
	c := []float32{0, 0, 10}
	closest := make([]float32, 3)

	t.Run("point at A", func(t *testing.T) {
		ClosestPtPointTriangle(closest, []float32{-1, 0, -1}, a, b, c)
		assert.InDelta(t, 0, closest[0], 0.0001)
		assert.InDelta(t, 0, closest[1], 0.0001)
		assert.InDelta(t, 0, closest[2], 0.0001)
	})

	t.Run("point inside triangle", func(t *testing.T) {
		ClosestPtPointTriangle(closest, []float32{2, 0, 2}, a, b, c)
		assert.InDelta(t, 2, closest[0], 0.0001)
		assert.InDelta(t, 0, closest[1], 0.0001)
		assert.InDelta(t, 2, closest[2], 0.0001)
	})
}

func TestDistancePtSegSqr2D(t *testing.T) {
	p := []float32{0, 0, 0}
	q := []float32{10, 0, 0}

	d, td := DistancePtSegSqr2D([]float32{5, 0, 0}, p, q)
	assert.InDelta(t, 0.0, d, 0.0001)
	assert.InDelta(t, 0.5, td, 0.0001)

	d, _ = DistancePtSegSqr2D([]float32{5, 0, 5}, p, q)
	assert.InDelta(t, 25.0, d, 0.0001)
}

func TestClosestHeightPointTriangle(t *testing.T) {
	a := []float32{0, 0, 0}
	b := []float32{10, 0, 0}
	c := []float32{0, 10, 0}

	// Degenerate triangle (collinear)
	found, h := ClosestHeightPointTriangle([]float32{0, 0, 0}, a, b, c)
	assert.False(t, found)
	assert.InDelta(t, 0.0, h, 0.0001)

	// Non-degenerate
	a2 := []float32{0, 5, 0}
	b2 := []float32{10, 0, 0}
	c2 := []float32{0, 0, 10}
	found, h = ClosestHeightPointTriangle([]float32{1, 0, 1}, a2, b2, c2)
	assert.True(t, found)
}

func TestPointInPolygon(t *testing.T) {
	verts := []float32{
		0, 0, 0,
		10, 0, 0,
		10, 0, 10,
		0, 0, 10,
	}

	assert.True(t, PointInPolygon([]float32{5, 0, 5}, verts, 4))
	assert.False(t, PointInPolygon([]float32{-1, 0, 5}, verts, 4))
	assert.False(t, PointInPolygon([]float32{5, 0, 15}, verts, 4))
}

func TestDistancePtPolyEdgesSqr(t *testing.T) {
	verts := []float32{
		0, 0, 0,
		10, 0, 0,
		10, 0, 10,
		0, 0, 10,
	}
	ed := make([]float32, 4)
	et := make([]float32, 4)

	inside := DistancePtPolyEdgesSqr([]float32{5, 0, 5}, verts, 4, ed, et)
	assert.True(t, inside)

	inside2 := DistancePtPolyEdgesSqr([]float32{-5, 0, 5}, verts, 4, ed, et)
	assert.False(t, inside2)
}

func TestIntersectSegmentPoly2D(t *testing.T) {
	verts := []float32{
		0, 0, 0,
		0, 0, 10,
		10, 0, 10,
		10, 0, 0,
	}

	found, tmin, tmax, _, _ := IntersectSegmentPoly2D(
		[]float32{-5, 0, 5}, []float32{15, 0, 5}, verts, 4,
	)
	assert.True(t, found)
	assert.InDelta(t, 0.25, tmin, 0.0001)
	assert.InDelta(t, 0.75, tmax, 0.0001)
}

func TestIntersectSegSeg2D(t *testing.T) {
	ap := []float32{0, 0, 0}
	aq := []float32{10, 0, 10}
	bp := []float32{0, 0, 10}
	bq := []float32{10, 0, 0}

	found, s, _ := IntersectSegSeg2D(ap, aq, bp, bq)
	assert.True(t, found)
	assert.InDelta(t, 0.5, s, 0.0001)

	// Parallel segments
	bp2 := []float32{0, 0, 1}
	bq2 := []float32{10, 0, 11}
	found, _, _ = IntersectSegSeg2D(ap, aq, bp2, bq2)
	assert.False(t, found)
}

// --- Polygon utilities ---

func TestCalcPolyCenter(t *testing.T) {
	verts := []float32{
		0, 0, 0,
		10, 0, 0,
		10, 0, 10,
		0, 0, 10,
	}
	idx := []uint16{0, 1, 2, 3}
	tc := make([]float32, 3)
	CalcPolyCenter(tc, idx, verts)
	assert.InDelta(t, 5.0, tc[0], 0.0001)
	assert.InDelta(t, 0.0, tc[1], 0.0001)
	assert.InDelta(t, 5.0, tc[2], 0.0001)
}

func TestOverlapPolyPoly2D(t *testing.T) {
	polyA := []float32{
		0, 0, 0,
		10, 0, 0,
		10, 0, 10,
		0, 0, 10,
	}
	polyB := []float32{
		5, 0, 5,
		15, 0, 5,
		15, 0, 15,
		5, 0, 15,
	}
	assert.True(t, OverlapPolyPoly2D(polyA, 4, polyB, 4))

	polyC := []float32{
		20, 0, 20,
		30, 0, 20,
		30, 0, 30,
		20, 0, 30,
	}
	assert.False(t, OverlapPolyPoly2D(polyA, 4, polyC, 4))
}

// --- Utility functions ---

func TestNextPow2(t *testing.T) {
	assert.Equal(t, uint32(1), NextPow2(1))
	assert.Equal(t, uint32(2), NextPow2(2))
	assert.Equal(t, uint32(4), NextPow2(3))
	assert.Equal(t, uint32(8), NextPow2(5))
	assert.Equal(t, uint32(16), NextPow2(9))
	assert.Equal(t, uint32(0x10000), NextPow2(0xffff))
	assert.Equal(t, uint32(0x80000000), NextPow2(0x40000001))
}

func TestIlog2(t *testing.T) {
	assert.Equal(t, uint32(0), Ilog2(1))
	assert.Equal(t, uint32(1), Ilog2(2))
	assert.Equal(t, uint32(1), Ilog2(3))
	assert.Equal(t, uint32(4), Ilog2(16))
	assert.Equal(t, uint32(9), Ilog2(512))
	assert.Equal(t, uint32(9), Ilog2(1023))
	assert.Equal(t, uint32(31), Ilog2(0xffffffff))
}

func TestAlign4(t *testing.T) {
	assert.Equal(t, 0, Align4(0))
	assert.Equal(t, 4, Align4(1))
	assert.Equal(t, 4, Align4(3))
	assert.Equal(t, 4, Align4(4))
	assert.Equal(t, 8, Align4(5))
}

func TestOppositeTile(t *testing.T) {
	assert.Equal(t, 4, OppositeTile(0))
	assert.Equal(t, 5, OppositeTile(1))
	assert.Equal(t, 0, OppositeTile(4))
	assert.Equal(t, 1, OppositeTile(5))
}

func TestSwap(t *testing.T) {
	a, b := 1, 2
	Swap(&a, &b)
	assert.Equal(t, 2, a)
	assert.Equal(t, 1, b)
}

func TestMin(t *testing.T) {
	assert.Equal(t, 1, Min(1, 2))
	assert.Equal(t, 1, Min(2, 1))
	assert.Equal(t, 1, Min(1, 1))
	assert.InDelta(t, float32(1.5), Min(float32(1.5), float32(2.0)), 0.0001)
}

func TestMax(t *testing.T) {
	assert.Equal(t, 2, Max(1, 2))
	assert.Equal(t, 2, Max(2, 1))
	assert.Equal(t, 1, Max(1, 1))
	assert.InDelta(t, float32(2.0), Max(float32(1.5), float32(2.0)), 0.0001)
}

func TestAbs(t *testing.T) {
	assert.Equal(t, 1, Abs(-1))
	assert.Equal(t, 1, Abs(1))
	assert.Equal(t, 0, Abs(0))
	assert.InDelta(t, float32(3.5), Abs(float32(-3.5)), 0.0001)
}

func TestSqr(t *testing.T) {
	assert.Equal(t, 4, Sqr(2))
	assert.Equal(t, 16, Sqr(-4))
	assert.Equal(t, uint32(9), Sqr(uint32(3)))
	assert.InDelta(t, float32(2.25), Sqr(float32(1.5)), 0.0001)
}

func TestClamp(t *testing.T) {
	assert.Equal(t, 5, Clamp(10, 0, 5))
	assert.Equal(t, 0, Clamp(-1, 0, 5))
	assert.Equal(t, 3, Clamp(3, 0, 5))
	assert.InDelta(t, float32(1.0), Clamp(float32(-5), float32(1.0), float32(3.0)), 0.0001)
}

// --- RandomPointInConvexPoly ---

func TestRandomPointInConvexPoly(t *testing.T) {
	pts := []float32{
		0, 0, 0,
		0, 0, 1,
		1, 0, 0,
	}
	npts := 3
	areas := make([]float32, 6)
	out := make([]float32, 3)

	t.Run("s=0, t=1 returns vertex B (z=1)", func(t *testing.T) {
		RandomPointInConvexPoly(pts, npts, areas, 0.0, 1.0, out)
		assert.InDelta(t, 0.0, out[0], 0.0001)
		assert.InDelta(t, 0.0, out[1], 0.0001)
		assert.InDelta(t, 1.0, out[2], 0.0001)
	})

	t.Run("s=0.5, t=1 returns midpoint of edge", func(t *testing.T) {
		RandomPointInConvexPoly(pts, npts, areas, 0.5, 1.0, out)
		assert.InDelta(t, 0.5, out[0], 0.0001)
		assert.InDelta(t, 0.0, out[1], 0.0001)
		assert.InDelta(t, 0.5, out[2], 0.0001)
	})
}

// --- HashRef ---

func TestHashRef(t *testing.T) {
	// Hash function should be deterministic and non-zero for non-zero inputs
	h1 := HashRef(42)
	h2 := HashRef(42)
	assert.Equal(t, h1, h2)
	assert.NotEqual(t, 0, HashRef(1))

	// Different inputs should generally produce different hashes
	h3 := HashRef(43)
	assert.NotEqual(t, h1, h3)
}

// --- NodePool ---

func TestNewNodePool(t *testing.T) {
	t.Run("valid params", func(t *testing.T) {
		pool, err := NewNodePool(100, 16)
		assert.NoError(t, err)
		assert.NotNil(t, pool)
		assert.Equal(t, 100, pool.GetMaxNodes())
		assert.Equal(t, 16, pool.GetHashSize())
		assert.Equal(t, 0, pool.GetNodeCount())
	})

	t.Run("maxNodes = 0 returns error", func(t *testing.T) {
		pool, err := NewNodePool(0, 16)
		assert.Error(t, err)
		assert.Nil(t, pool)
	})

	t.Run("maxNodes too large returns error", func(t *testing.T) {
		pool, err := NewNodePool(1<<NodeParentBits, 16)
		assert.Error(t, err)
		assert.Nil(t, pool)
	})
}

func TestNodePoolGetNode(t *testing.T) {
	pool, err := NewNodePool(100, 16)
	assert.NoError(t, err)

	node := pool.GetNode(42, 0)
	assert.NotNil(t, node)
	assert.Equal(t, PolyRef(42), node.ID)
	assert.Equal(t, uint8(0), node.State)
	assert.Equal(t, 1, pool.GetNodeCount())

	// Getting the same one should return existing
	node2 := pool.GetNode(42, 0)
	assert.Equal(t, node, node2)
	assert.Equal(t, 1, pool.GetNodeCount())

	// Different state should create new
	node3 := pool.GetNode(42, 1)
	assert.NotEqual(t, node, node3)
	assert.Equal(t, 2, pool.GetNodeCount())
}

func TestNodePoolFindNode(t *testing.T) {
	pool, _ := NewNodePool(100, 16)
	pool.GetNode(42, 0)
	pool.GetNode(42, 1)
	pool.GetNode(99, 0)

	found := pool.FindNode(42, 0)
	assert.NotNil(t, found)
	assert.Equal(t, PolyRef(42), found.ID)

	found2 := pool.FindNode(42, 1)
	assert.NotNil(t, found2)

	notFound := pool.FindNode(42, 2)
	assert.Nil(t, notFound)

	notFound2 := pool.FindNode(999, 0)
	assert.Nil(t, notFound2)
}

func TestNodePoolFindNodes(t *testing.T) {
	pool, _ := NewNodePool(100, 16)
	pool.GetNode(42, 0)
	pool.GetNode(42, 1)
	pool.GetNode(99, 0)

	nodes := make([]*Node, 10)
	n := pool.FindNodes(42, nodes, 10)
	assert.Equal(t, 2, n)
	assert.NotNil(t, nodes[0])
	assert.NotNil(t, nodes[1])

	// Max nodes limit
	nodes2 := make([]*Node, 1)
	n2 := pool.FindNodes(42, nodes2, 1)
	assert.Equal(t, 1, n2)
}

func TestNodePoolClear(t *testing.T) {
	pool, _ := NewNodePool(100, 16)
	pool.GetNode(42, 0)
	assert.Equal(t, 1, pool.GetNodeCount())

	pool.Clear()
	assert.Equal(t, 0, pool.GetNodeCount())
	assert.Nil(t, pool.FindNode(42, 0))
}

func TestNodePoolGetNodeAtIdx(t *testing.T) {
	pool, _ := NewNodePool(100, 16)
	node := pool.GetNode(42, 0)
	idx := pool.GetNodeIdx(node)
	assert.NotEqual(t, uint32(0), idx)

	same := pool.GetNodeAtIdx(idx)
	assert.Equal(t, node, same)

	nilNode := pool.GetNodeAtIdx(0)
	assert.Nil(t, nilNode)
}

// --- NodeQueue ---

func TestNewNodeQueue(t *testing.T) {
	q := NewNodeQueue(10)
	assert.NotNil(t, q)
	assert.Equal(t, 10, q.GetCapacity())
	assert.True(t, q.Empty())
}

func TestNodeQueuePushPop(t *testing.T) {
	q := NewNodeQueue(10)

	// Push nodes with different costs
	cheap := &Node{Total: 1}
	mid := &Node{Total: 5}
	expensive := &Node{Total: 10}

	q.Push(expensive)
	q.Push(mid)
	q.Push(cheap)

	assert.False(t, q.Empty())

	// Should pop in order of total cost (min-heap)
	first := q.Pop()
	assert.Equal(t, float32(1), first.Total)

	second := q.Pop()
	assert.Equal(t, float32(5), second.Total)

	third := q.Pop()
	assert.Equal(t, float32(10), third.Total)

	assert.True(t, q.Empty())
}

func TestNodeQueueTop(t *testing.T) {
	q := NewNodeQueue(10)
	assert.Nil(t, q.Top())

	cheap := &Node{Total: 1}
	expensive := &Node{Total: 10}
	q.Push(expensive)
	q.Push(cheap)

	top := q.Top()
	assert.Equal(t, float32(1), top.Total)
	assert.Equal(t, 2, q.size) // Top doesn't remove
}

func TestNodeQueueClear(t *testing.T) {
	q := NewNodeQueue(10)
	q.Push(&Node{Total: 1})
	q.Push(&Node{Total: 2})
	assert.False(t, q.Empty())

	q.Clear()
	assert.True(t, q.Empty())
	assert.Nil(t, q.Pop())
}

func TestNodeQueueModify(t *testing.T) {
	q := NewNodeQueue(10)

	node1 := &Node{Total: 10}
	node2 := &Node{Total: 5}

	q.Push(node1)
	q.Push(node2)

	// Modify node1 to have lowest cost
	node1.Total = 1
	q.Modify(node1)

	first := q.Pop()
	assert.Equal(t, float32(1), first.Total)
}
