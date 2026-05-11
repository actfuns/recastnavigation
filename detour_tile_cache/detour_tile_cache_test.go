package detour_tile_cache

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestTileCacheLayerHeaderSize(t *testing.T) {
	assert.Equal(t, 56, TileCacheLayerHeaderSize())
	assert.Equal(t, Align4(56), TileCacheLayerHeaderSize())
}
func TestAlign4(t *testing.T) {
	assert.Equal(t, 0, Align4(0))
	assert.Equal(t, 4, Align4(1))
	assert.Equal(t, 4, Align4(3))
	assert.Equal(t, 4, Align4(4))
	assert.Equal(t, 8, Align4(5))
}

func TestNextPow2(t *testing.T) {
	assert.Equal(t, uint32(1), NextPow2(1))
	assert.Equal(t, uint32(2), NextPow2(2))
	assert.Equal(t, uint32(4), NextPow2(3))
	assert.Equal(t, uint32(8), NextPow2(5))
	assert.Equal(t, uint32(16), NextPow2(9))
	assert.Equal(t, uint32(0x10000), NextPow2(0xffff))
}

func TestIlog2(t *testing.T) {
	assert.Equal(t, 0, Ilog2(1))
	assert.Equal(t, 1, Ilog2(2))
	assert.Equal(t, 1, Ilog2(3))
	assert.Equal(t, 4, Ilog2(16))
	assert.Equal(t, 9, Ilog2(512))
	assert.Equal(t, 9, Ilog2(1023))
	assert.Equal(t, 31, Ilog2(0xffffffff))
}
