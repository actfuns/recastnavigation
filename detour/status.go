package detour

// Status type for Detour operations.
type Status uint32

// High level status.
const (
	Failure    Status = 1 << 31
	Success    Status = 1 << 30
	InProgress Status = 1 << 29
)

// Detail information for status.
const (
	WrongMagic      Status = 1 << 0
	WrongVersion    Status = 1 << 1
	OutOfMemory     Status = 1 << 2
	InvalidParam    Status = 1 << 3
	BufferTooSmall  Status = 1 << 4
	OutOfNodes      Status = 1 << 5
	PartialResult   Status = 1 << 6
	AlreadyOccupied Status = 1 << 7

	StatusDetailMask Status = 0x0ffffff
)

// StatusSucceed returns true if status is success.
func StatusSucceed(status Status) bool {
	return (status & Success) != 0
}

// StatusFailed returns true if status is failure.
func StatusFailed(status Status) bool {
	return (status & Failure) != 0
}

// StatusInProgress returns true if status is in progress.
func StatusInProgress(status Status) bool {
	return (status & InProgress) != 0
}

// StatusDetail returns true if specific detail is set.
func StatusDetail(status Status, detail Status) bool {
	return (status & detail) != 0
}
