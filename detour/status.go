// Package detour implements navigation mesh pathfinding.
package detour

import "errors"

// Sentinel errors for Detour operations.
var (
	ErrFailure         = errors.New("detour: failure")
	ErrInvalidParam    = errors.New("detour: invalid parameter")
	ErrOutOfMemory     = errors.New("detour: out of memory")
	ErrBufferTooSmall  = errors.New("detour: buffer too small")
	ErrOutOfNodes      = errors.New("detour: out of nodes")
	ErrPartialResult   = errors.New("detour: partial result")
	ErrWrongMagic      = errors.New("detour: wrong magic")
	ErrWrongVersion    = errors.New("detour: wrong version")
	ErrAlreadyOccupied = errors.New("detour: already occupied")
	ErrInProgress      = errors.New("detour: in progress")
)
