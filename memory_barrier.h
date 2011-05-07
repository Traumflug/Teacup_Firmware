#ifndef _MEMORY_BARRIER_H_
#define _MEMORY_BARRIER_H_

#include <util/atomic.h>
#include <avr/version.h>

// Provide a memory barrier to the compiler. This informs
// the compiler that is should write any cached values that
// are destined for a global variable and discard any other
// cached values from global variables.
//
// Note that this behavior does apply to all global variables,
// not just volatile ones. However, cached local variables
// are not affected as they are not externally visible.

#define MEMORY_BARRIER() __asm volatile( "" ::: "memory" )

#endif