// newlib compatibility shim for the precompiled libmicroros.a archive.
//
// The cortex-m0plus libmicroros.a in micro_ros_arduino was built against an
// older newlib that exported __locale_ctype_ptr(). arduino-pico 4.x ships
// GCC 14 with a newer newlib in which that symbol is gone, so the linker
// fails on the rmw/rcl validate_*_name() functions.
//
// We re-export the symbol by returning newlib's underlying ctype table.
// _ctype_ has one prefix entry for EOF, so the canonical pointer is
// _ctype_ + 1 — that's what the validate_* functions then index with an
// unsigned char to probe isalpha/isalnum/etc.

extern "C" {

extern const char _ctype_[];

const char* __locale_ctype_ptr(void) {
  return _ctype_ + 1;
}

}  // extern "C"
