// Package recast implements navigation mesh generation.
package recast

import (
	"fmt"
	"time"
)

// Context provides an interface for optional logging and performance tracking
// of the Recast build process.
type Context struct {
	logEnabled   bool
	timerEnabled bool
	logFunc      func(category LogCategory, msg string)
	timers       map[TimerLabel]time.Duration
	timerStart   map[TimerLabel]time.Time
}

// NewContext creates a new Context.
func NewContext(state bool) *Context {
	return &Context{
		logEnabled:   state,
		timerEnabled: state,
		timers:       make(map[TimerLabel]time.Duration),
		timerStart:   make(map[TimerLabel]time.Time),
	}
}

// EnableLog enables or disables logging.
func (ctx *Context) EnableLog(state bool) {
	ctx.logEnabled = state
}

// ResetLog clears all log entries.
func (ctx *Context) ResetLog() {
	// Log entries are not stored; they are passed through the callback.
}

// Log logs a message.
func (ctx *Context) Log(category LogCategory, format string, args ...interface{}) {
	if !ctx.logEnabled {
		return
	}
	if ctx.logFunc != nil {
		msg := fmt.Sprintf(format, args...)
		ctx.logFunc(category, msg)
	}
}

// SetLogFunc sets the log callback function.
func (ctx *Context) SetLogFunc(f func(category LogCategory, msg string)) {
	ctx.logFunc = f
}

// EnableTimer enables or disables the performance timers.
func (ctx *Context) EnableTimer(state bool) {
	ctx.timerEnabled = state
}

// ResetTimers clears all performance timers.
func (ctx *Context) ResetTimers() {
	if ctx.timerEnabled {
		ctx.timers = make(map[TimerLabel]time.Duration)
		ctx.timerStart = make(map[TimerLabel]time.Time)
	}
}

// StartTimer starts the specified performance timer.
func (ctx *Context) StartTimer(label TimerLabel) {
	if ctx.timerEnabled {
		ctx.timerStart[label] = time.Now()
	}
}

// StopTimer stops the specified performance timer.
func (ctx *Context) StopTimer(label TimerLabel) {
	if ctx.timerEnabled {
		if start, ok := ctx.timerStart[label]; ok {
			elapsed := time.Since(start)
			ctx.timers[label] += elapsed
			delete(ctx.timerStart, label)
		}
	}
}

// GetAccumulatedTime returns the total accumulated time of the specified performance timer.
func (ctx *Context) GetAccumulatedTime(label TimerLabel) time.Duration {
	if !ctx.timerEnabled {
		return -1
	}
	if d, ok := ctx.timers[label]; ok {
		return d
	}
	return -1
}

// ScopedTimer is a helper to start and stop a timer.
// Usage: defer ctx.ScopedTimer(label)()
func (ctx *Context) ScopedTimer(label TimerLabel) func() {
	ctx.StartTimer(label)
	return func() {
		ctx.StopTimer(label)
	}
}
