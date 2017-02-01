#ifndef	DDA_PLANNER_H
#define	DDA_PLANNER_H

#include	<stdint.h>

#include	"config_wrapper.h"
#include "dda.h"

// Period over which we calculate new velocity; should be at least TICK_TIME/2
#define QUANTUM (TICK_TIME*2)

// Acceleration per QUANTUM.
// Normalized to q8.24; allows up to 2^8=256 in mantissa (steps per quantum)
#define ACCEL_P_SHIFT 16 // 24
extern const axes_uint32_t PROGMEM accel_P ;

#define PLANNER_QUEUE_SIZE 4
/**
  \struct MOVE_PLANNER
  \brief path planner for future movements

  This struct holds the plan fast-axis steps for the next several steps.  It
  incorporates the current DDA as well as future DDAs before they become active
  on the hardware. There is only one of this structure. It is updated in dda_plan
  and consumed in dda_step.
*/
typedef struct {
  uint32_t          position;     // Calculated fast-axis position planned
  uint32_t          velocity;     // fast axis velocity updated on each dda_clock call
  uint32_t          remainder;    // calculated fractional position between dda_clock intervals

  DDA *             dda;          // Pointer to currently active dda (HACK)

  uint32_t          accel_per_tick;                 // fast axis acceleration per TICK_TIME, 8.24 fixed point
  uint32_t          curr_c;                         // Current speed
  uint32_t          end_c;                          // Planned speed at end of current queue
  int32_t           next_dc[PLANNER_QUEUE_SIZE];    // delta speed of next steps
  uint32_t          next_n[PLANNER_QUEUE_SIZE];     // Number of steps in the next movement; 0 when taken by dda
  uint8_t           head;                           // Index of next movement queue
  uint8_t           tail;                           // Index of last movement queue
  uint8_t           accel    :1 ;                   // bool: accel or decel/cruise
} MOVE_PLANNER;

extern MOVE_PLANNER BSS planner;


// Initialize movement planner structures
void planner_init(void);

// Get the next step time from the movement planner
uint32_t planner_get(bool clip_cruise);

// Put a movement into the movement planner
void planner_put(uint32_t steps, uint32_t speed);

// Check for no more moves in planner queue
inline bool planner_empty(void) {
  return planner.next_n[planner.head] == 0;
}

// Check for planner queue full
inline bool planner_full(void) {
  return planner.next_n[planner.tail] != 0;
}

// Incorporate next dda into the movement planner
void planner_begin_dda(DDA *dda);

// Fill planner queue with calculated step motions
void planner_fill_queue(void);

#endif	/* DDA_PLANNER_H */
