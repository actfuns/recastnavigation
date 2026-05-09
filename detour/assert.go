package detour

// AssertFailFunc is a custom assertion failure function.
type AssertFailFunc func(expression string, file string, line int)

var sAssertFailFunc AssertFailFunc

// AssertFailSetCustom sets a custom assertion failure function.
func AssertFailSetCustom(assertFailFunc AssertFailFunc) {
	sAssertFailFunc = assertFailFunc
}

// AssertFailGetCustom gets the custom assertion failure function.
func AssertFailGetCustom() AssertFailFunc {
	return sAssertFailFunc
}

// Assert checks the condition and calls the failure function if it's false.
func Assert(expression bool) {
	if sAssertFailFunc != nil && !expression {
		sAssertFailFunc("assertion failed", "detour", 0)
	}
}
