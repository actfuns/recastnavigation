package detour

import (
	"fmt"
	"testing"
)

// Benchmark grid navmesh configuration.
// 20x20 grid = 800 polygons, 441 vertices.
const (
	benchRows     = 20
	benchCols     = 20
	benchCellSize = 10.0
)

// benchSetup builds the benchmark navmesh and query once.
func benchSetup(b *testing.B) (*NavMesh, *NavMeshQuery, *QueryFilter, [3]float32) {
	b.Helper()
	m := buildTestGridNavmesh(b, benchRows, benchCols, benchCellSize)
	q := NewNavMeshQuery()
	if err := q.Init(m, 65535); err != nil {
		b.Fatalf("NavMeshQuery.Init: %v", err)
	}

	filter := &QueryFilter{IncludeFlags: 0xffff, ExcludeFlags: 0}
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0
	}
	halfExtents := [3]float32{benchCellSize, 2, benchCellSize}
	return m, q, filter, halfExtents
}

// benchRef finds the nearest poly ref, skipping benchmark on failure.
func benchRef(b *testing.B, q *NavMeshQuery, pos [3]float32, filter *QueryFilter, halfExtents [3]float32) PolyRef {
	b.Helper()
	ref, _, err := q.FindNearestPoly(pos, halfExtents, filter)
	if err != nil || ref == 0 {
		b.Skipf("no poly near (%.0f,%.0f,%.0f)", pos[0], pos[1], pos[2])
	}
	return ref
}

// benchPath pre-computes a reference path, skipping benchmark on failure.
func benchPath(b *testing.B, q *NavMeshQuery, startRef, endRef PolyRef, startPos, endPos [3]float32) ([]PolyRef, int) {
	b.Helper()
	path, n, err := q.FindPath(startRef, endRef, startPos, endPos, filter, 4096)
	if err != nil || n == 0 {
		b.Skip("no path found")
	}
	return path, n
}

var (
	filter      *QueryFilter
	halfExtents [3]float32
	query       *NavMeshQuery
)

// ---------------------------------------------------------------------------
// FindPath: near, mid, far
// ---------------------------------------------------------------------------

func BenchmarkFindPath(b *testing.B) {
	_, q, f, he := benchSetup(b)
	filter = f
	halfExtents = he
	query = q

	type benchCase struct {
		name       string
		start, end [3]float32
	}
	cases := []benchCase{
		{"near_5cells", [3]float32{2, 0, 2}, [3]float32{42, 0, 2}},
		{"mid_10cells", [3]float32{2, 0, 2}, [3]float32{102, 0, 2}},
		{"far_19cells", [3]float32{2, 0, 2}, [3]float32{192, 0, 2}},
	}

	for _, c := range cases {
		startRef := benchRef(b, q, c.start, f, he)
		endRef := benchRef(b, q, c.end, f, he)

		b.Run(c.name, func(b *testing.B) {
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				path, n, err := q.FindPath(startRef, endRef, c.start, c.end, f, 4096)
				if err != nil {
					b.Fatalf("FindPath: %v", err)
				}
				if n == 0 {
					b.Fatal("empty path")
				}
				_ = path
			}
		})
	}
}

// ---------------------------------------------------------------------------
// FindStraightPath on the long horizontal path
// ---------------------------------------------------------------------------

func BenchmarkFindStraightPath(b *testing.B) {
	_, q, f, he := benchSetup(b)

	start := [3]float32{2, 0, 2}
	end := [3]float32{192, 0, 2}

	startRef := benchRef(b, q, start, f, he)
	endRef := benchRef(b, q, end, f, he)
	path, pathCount := benchPath(b, q, startRef, endRef, start, end)
	b.Logf("reference path: %d polys", pathCount)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _, _, n, err := q.FindStraightPath(start, end, path, pathCount, 256, 0)
		if err != nil {
			b.Fatalf("FindStraightPath: %v", err)
		}
		if n < 2 {
			b.Fatal("too short")
		}
	}
}

// ---------------------------------------------------------------------------
// Raycast: short, mid, long, diagonal
// ---------------------------------------------------------------------------

func BenchmarkRaycast(b *testing.B) {
	_, q, f, he := benchSetup(b)

	type rayCase struct {
		name       string
		start, end [3]float32
	}
	cases := []rayCase{
		{"short_5cells", [3]float32{2, 0, 2}, [3]float32{42, 0, 2}},
		{"mid_10cells", [3]float32{2, 0, 2}, [3]float32{102, 0, 2}},
		{"long_19cells", [3]float32{2, 0, 2}, [3]float32{192, 0, 2}},
	}

	for _, c := range cases {
		startRef := benchRef(b, q, c.start, f, he)

		b.Run(c.name, func(b *testing.B) {
			hit := &RaycastHit{}
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				err := q.Raycast(startRef, c.start, c.end, f, 0, 0, hit)
				if err != nil {
					b.Fatalf("Raycast: %v", err)
				}
			}
		})
	}
}

// ---------------------------------------------------------------------------
// FindPolysAroundCircle: increasing radii
// ---------------------------------------------------------------------------

func BenchmarkFindPolysAroundCircle(b *testing.B) {
	_, q, f, he := benchSetup(b)

	maxResult := 4096
	resultRef := make([]PolyRef, maxResult)
	resultParent := make([]PolyRef, maxResult)
	resultCost := make([]float32, maxResult)

	center := [3]float32{102, 0, 102}
	ref := benchRef(b, q, center, f, he)

	for _, radius := range []float32{50, 100, 200} {
		b.Run(fmt.Sprintf("r%.0f", radius), func(b *testing.B) {
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				n, err := q.FindPolysAroundCircle(ref, center, radius, f,
					resultRef, resultParent, resultCost, maxResult)
				if err != nil {
					b.Fatalf("FindPolysAroundCircle: %v", err)
				}
				_ = n
			}
		})
	}
}

// ---------------------------------------------------------------------------
// Navmesh build stress: 10×10, 50×50, 100×100 grids
// ---------------------------------------------------------------------------

func BenchmarkBuildNavmesh(b *testing.B) {
	for _, s := range []struct {
		name      string
		rows, cols int
	}{
		{"10x10_200polys", 10, 10},
		{"50x50_5000polys", 50, 50},
		{"100x100_20000polys", 100, 100},
	} {
		b.Run(s.name, func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				data := buildGridNavmeshBytes(b, s.rows, s.cols, 10)
				m := &NavMesh{}
				err := m.InitSingleTile(data, 0)
				if err != nil {
					b.Fatalf("InitSingleTile: %v", err)
				}
				_ = m
			}
		})
	}
}
