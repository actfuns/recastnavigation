// Package detour implements navigation mesh pathfinding.
package detour

import "errors"

// Sentinel errors for Detour operations.
var (
	ErrInvalidParam           = errors.New("detour: invalid parameter")
	ErrOutOfMemory            = errors.New("detour: out of memory")
	ErrBufferTooSmall         = errors.New("detour: buffer too small")
	ErrOutOfNodes             = errors.New("detour: out of nodes")
	ErrPartialResult          = errors.New("detour: partial result")
	ErrWrongMagic             = errors.New("detour: wrong magic")
	ErrWrongVersion           = errors.New("detour: wrong version")
	ErrAlreadyOccupied        = errors.New("detour: already occupied")
	ErrInProgress             = errors.New("detour: in progress")
	ErrPolyNotFound           = errors.New("detour: polygon not found")
	ErrTileNotFound           = errors.New("detour: tile not found")
	ErrPointNotOnPoly         = errors.New("detour: point not on polygon")
	ErrStartPolyNotPassFilter = errors.New("detour: start polygon does not pass filter")
	ErrRequestNotFound        = errors.New("detour: request not found")
	ErrTileAlreadyExists      = errors.New("detour: tile already exists")
	ErrNavMeshDataBuildFailed = errors.New("detour: navmesh data build failed")
	ErrNotOffMeshConnection   = errors.New("detour: not an off-mesh connection")
)
