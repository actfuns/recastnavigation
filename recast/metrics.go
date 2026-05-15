package recast

import (
	"context"
	"sync"
	"time"
)

type metricsContextKeyType struct{}

var metricsContextKey = metricsContextKeyType{}

type PerfMetrics struct {
	mu         sync.Mutex
	Timers     map[TimerLabel]time.Duration
	timerStart map[TimerLabel]time.Time
}

func NewPerfMetrics() *PerfMetrics {
	return &PerfMetrics{
		Timers:     make(map[TimerLabel]time.Duration),
		timerStart: make(map[TimerLabel]time.Time),
	}
}

func WithMetrics(ctx context.Context, metrics *PerfMetrics) context.Context {
	return context.WithValue(ctx, metricsContextKey, metrics)
}

func GetMetrics(ctx context.Context) (*PerfMetrics, bool) {
	m, ok := ctx.Value(metricsContextKey).(*PerfMetrics)
	return m, ok
}

func ScopedTimer(ctx context.Context, label TimerLabel) func() {
	metrics, ok := ctx.Value(metricsContextKey).(*PerfMetrics)
	if !ok || metrics == nil {
		return func() {}
	}

	start := time.Now()

	return func() {
		cost := time.Since(start)

		metrics.mu.Lock()
		metrics.Timers[label] += cost
		metrics.mu.Unlock()
	}
}

func StartTimer(ctx context.Context, label TimerLabel) {
	metrics, ok := ctx.Value(metricsContextKey).(*PerfMetrics)
	if !ok || metrics == nil {
		return
	}

	metrics.mu.Lock()
	metrics.timerStart[label] = time.Now()
	metrics.mu.Unlock()
}

func StopTimer(ctx context.Context, label TimerLabel) {
	metrics, ok := ctx.Value(metricsContextKey).(*PerfMetrics)
	if !ok || metrics == nil {
		return
	}

	metrics.mu.Lock()
	if start, ok := metrics.timerStart[label]; ok {
		metrics.Timers[label] += time.Since(start)
		delete(metrics.timerStart, label)
	}
	metrics.mu.Unlock()
}

func GetAccumulatedTime(ctx context.Context, label TimerLabel) (time.Duration, bool) {
	metrics, ok := ctx.Value(metricsContextKey).(*PerfMetrics)
	if !ok || metrics == nil {
		return 0, false
	}

	metrics.mu.Lock()
	defer metrics.mu.Unlock()

	d, ok := metrics.Timers[label]
	return d, ok
}
