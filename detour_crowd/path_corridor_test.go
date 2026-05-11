package detour_crowd

import (
	"testing"
)

func TestMergeCorridorStartMoved(t *testing.T) {
	t.Run("Should handle empty input", func(t *testing.T) {
		path := []PolyRef(nil)
		visited := []PolyRef(nil)
		result := mergeCorridorStartMoved(path, 0, 0, visited, 0)
		if result != 0 {
			t.Errorf("expected 0, got %d", result)
		}
	})

	t.Run("Should handle empty visited", func(t *testing.T) {
		path := []PolyRef{1}
		visited := []PolyRef(nil)
		result := mergeCorridorStartMoved(path, 1, 1, visited, 0)
		if result != 1 {
			t.Errorf("expected 1, got %d", result)
		}
		if path[0] != 1 {
			t.Errorf("expected path[0]=1, got %d", path[0])
		}
	})

	t.Run("Should handle empty path", func(t *testing.T) {
		path := []PolyRef(nil)
		visited := []PolyRef{1}
		result := mergeCorridorStartMoved(path, 0, 0, visited, 1)
		if result != 0 {
			t.Errorf("expected 0, got %d", result)
		}
	})

	t.Run("Should strip visited points from path except last", func(t *testing.T) {
		path := []PolyRef{1, 2}
		visited := []PolyRef{1, 2}
		result := mergeCorridorStartMoved(path, 2, 2, visited, 2)
		if result != 1 {
			t.Errorf("expected 1, got %d", result)
		}
		if path[0] != 2 {
			t.Errorf("expected path[0]=2, got %d", path[0])
		}
	})

	t.Run("Should add visited points not present in path in reverse order", func(t *testing.T) {
		path := []PolyRef{1, 2, 0}
		visited := []PolyRef{1, 2, 3, 4}
		result := mergeCorridorStartMoved(path, 2, 3, visited, 4)
		if result != 3 {
			t.Errorf("expected 3, got %d", result)
		}
		expected := []PolyRef{4, 3, 2}
		for i := range expected {
			if path[i] != expected[i] {
				t.Errorf("path[%d] = %d, want %d", i, path[i], expected[i])
			}
		}
	})

	t.Run("Should add visited points not present in path up to the path capacity", func(t *testing.T) {
		path := []PolyRef{1, 2, 0}
		visited := []PolyRef{1, 2, 3, 4, 5}
		result := mergeCorridorStartMoved(path, 2, 3, visited, 5)
		if result != 3 {
			t.Errorf("expected 3, got %d", result)
		}
		expected := []PolyRef{5, 4, 3}
		for i := range expected {
			if path[i] != expected[i] {
				t.Errorf("path[%d] = %d, want %d", i, path[i], expected[i])
			}
		}
	})

	t.Run("Should not change path if there is no intersection with visited", func(t *testing.T) {
		path := []PolyRef{1, 2}
		visited := []PolyRef{3, 4}
		result := mergeCorridorStartMoved(path, 2, 2, visited, 2)
		if result != 2 {
			t.Errorf("expected 2, got %d", result)
		}
		expected := []PolyRef{1, 2}
		for i := range expected {
			if path[i] != expected[i] {
				t.Errorf("path[%d] = %d, want %d", i, path[i], expected[i])
			}
		}
	})

	t.Run("Should save unvisited path points", func(t *testing.T) {
		path := []PolyRef{1, 2, 0}
		visited := []PolyRef{1, 3}
		result := mergeCorridorStartMoved(path, 2, 3, visited, 2)
		if result != 3 {
			t.Errorf("expected 3, got %d", result)
		}
		expected := []PolyRef{3, 1, 2}
		for i := range expected {
			if path[i] != expected[i] {
				t.Errorf("path[%d] = %d, want %d", i, path[i], expected[i])
			}
		}
	})

	t.Run("Should save unvisited path points up to the path capacity", func(t *testing.T) {
		path := []PolyRef{1, 2}
		visited := []PolyRef{1, 3}
		result := mergeCorridorStartMoved(path, 2, 2, visited, 2)
		if result != 2 {
			t.Errorf("expected 2, got %d", result)
		}
		expected := []PolyRef{3, 1}
		for i := range expected {
			if path[i] != expected[i] {
				t.Errorf("path[%d] = %d, want %d", i, path[i], expected[i])
			}
		}
	})
}
