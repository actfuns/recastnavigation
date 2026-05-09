// Package recast implements navigation mesh generation.
package recast

import (
	"fmt"
	"os"
)

// AssertFailFunc is the type for assertion failure callback functions.
type AssertFailFunc func(expression string, file string, line int)

var sRecastAssertFailFunc AssertFailFunc

// AssertFailSetCustom sets the custom assertion failure callback function.
func AssertFailSetCustom(assertFailFunc AssertFailFunc) {
	sRecastAssertFailFunc = assertFailFunc
}

// AssertFailGetCustom gets the custom assertion failure function.
func AssertFailGetCustom() AssertFailFunc {
	return sRecastAssertFailFunc
}

// Assert performs an assertion check.
// If the custom assertion fail function is set, it calls it; otherwise it panics.
func Assert(expression bool) {
	if !expression {
		if sRecastAssertFailFunc != nil {
			sRecastAssertFailFunc("assertion failed", "unknown", 0)
		} else {
			_, _ = fmt.Fprintf(os.Stderr, "Assertion failed\n")
			panic("recast assertion failed")
		}
	}
}
