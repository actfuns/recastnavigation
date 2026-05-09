package detour_crowd

import (
	"math"
)

// ProximityGrid is a spatial hash grid used for proximity queries among agents.
type ProximityGrid struct {
	cellSize    float32
	invCellSize float32

	pool     []ProximityItem
	poolHead int
	poolSize int

	buckets     []uint16
	bucketsSize int

	bounds [4]int
}

// ProximityItem represents an item in the proximity grid's pool.
type ProximityItem struct {
	Id   uint16
	X, Y int16
	Next uint16
}

// NewProximityGrid creates and initializes a proximity grid.
func NewProximityGrid() *ProximityGrid {
	return &ProximityGrid{}
}

// Init initializes the proximity grid.
func (g *ProximityGrid) Init(poolSize int, cellSize float32) bool {
	if poolSize <= 0 || cellSize <= 0 {
		return false
	}

	g.cellSize = cellSize
	g.invCellSize = 1.0 / cellSize

	// Allocate hash buckets
	g.bucketsSize = nextPow2(poolSize)
	g.buckets = make([]uint16, g.bucketsSize)

	// Allocate pool of items
	g.poolSize = poolSize
	g.poolHead = 0
	g.pool = make([]ProximityItem, g.poolSize)

	g.Clear()

	return true
}

// Clear clears the grid.
func (g *ProximityGrid) Clear() {
	for i := range g.buckets {
		g.buckets[i] = 0xffff
	}
	g.poolHead = 0
	g.bounds[0] = 0xffff
	g.bounds[1] = 0xffff
	g.bounds[2] = -0xffff
	g.bounds[3] = -0xffff
}

// AddItem adds an item to the grid.
func (g *ProximityGrid) AddItem(id uint16, minx, miny, maxx, maxy float32) {
	iminx := int(math.Floor(float64(minx * g.invCellSize)))
	iminy := int(math.Floor(float64(miny * g.invCellSize)))
	imaxx := int(math.Floor(float64(maxx * g.invCellSize)))
	imaxy := int(math.Floor(float64(maxy * g.invCellSize)))

	if iminx < g.bounds[0] {
		g.bounds[0] = iminx
	}
	if iminy < g.bounds[1] {
		g.bounds[1] = iminy
	}
	if imaxx > g.bounds[2] {
		g.bounds[2] = imaxx
	}
	if imaxy > g.bounds[3] {
		g.bounds[3] = imaxy
	}

	for y := iminy; y <= imaxy; y++ {
		for x := iminx; x <= imaxx; x++ {
			if g.poolHead < g.poolSize {
				h := hashPos2(x, y, g.bucketsSize)
				idx := uint16(g.poolHead)
				g.poolHead++
				item := &g.pool[idx]
				item.X = int16(x)
				item.Y = int16(y)
				item.Id = id
				item.Next = g.buckets[h]
				g.buckets[h] = idx
			}
		}
	}
}

// QueryItems queries items in the given rectangular region.
func (g *ProximityGrid) QueryItems(minx, miny, maxx, maxy float32, ids []uint16) int {
	iminx := int(math.Floor(float64(minx * g.invCellSize)))
	iminy := int(math.Floor(float64(miny * g.invCellSize)))
	imaxx := int(math.Floor(float64(maxx * g.invCellSize)))
	imaxy := int(math.Floor(float64(maxy * g.invCellSize)))

	n := 0
	maxIds := len(ids)

	for y := iminy; y <= imaxy; y++ {
		for x := iminx; x <= imaxx; x++ {
			h := hashPos2(x, y, g.bucketsSize)
			idx := g.buckets[h]
			for idx != 0xffff {
				item := &g.pool[idx]
				if int(item.X) == x && int(item.Y) == y {
					// Check if the id exists already
					found := false
					for i := 0; i < n; i++ {
						if ids[i] == item.Id {
							found = true
							break
						}
					}
					if !found {
						if n >= maxIds {
							return n
						}
						ids[n] = item.Id
						n++
					}
				}
				idx = item.Next
			}
		}
	}

	return n
}

// GetItemCountAt returns the number of items at the given cell coordinates.
func (g *ProximityGrid) GetItemCountAt(x, y int) int {
	n := 0
	h := hashPos2(x, y, g.bucketsSize)
	idx := g.buckets[h]
	for idx != 0xffff {
		item := &g.pool[idx]
		if int(item.X) == x && int(item.Y) == y {
			n++
		}
		idx = item.Next
	}
	return n
}

// GetBounds returns the grid bounds.
func (g *ProximityGrid) GetBounds() *[4]int {
	return &g.bounds
}

// GetCellSize returns the cell size.
func (g *ProximityGrid) GetCellSize() float32 {
	return g.cellSize
}

func hashPos2(x, y, n int) int {
	return ((x * 73856093) ^ (y * 19349663)) & (n - 1)
}

func nextPow2(v int) int {
	r := 1
	for r < v {
		r <<= 1
	}
	return r
}
