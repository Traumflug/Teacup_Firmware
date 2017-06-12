#include	"dda_queue.h"

/** \file
	\brief DDA Queue - manage the move queue
*/

#include	<string.h>

#include	"config_wrapper.h"
#include	"timer.h"
#include	"serial.h"
#include	"temp.h"
#include	"delay.h"
#include	"sersendf.h"
#include	"debug.h"
#include	"clock.h"
#include "cpu.h"
#include	"memory_barrier.h"
#include "math.h"
#include "dda_maths.h"
#include "gcode_parse.h"
/**
  Movebuffer head pointer. Points to the last move in the queue. This variable
  is used both in and out of interrupts, but is only written outside of
  interrupts.
*/
static uint8_t mb_head = 0;

/// movebuffer tail pointer. Points to the currently executing move
/// this variable is read/written both in and out of interrupts.
uint8_t	mb_tail = 0;

/// move buffer.
/// holds move queue
/// contents are read/written both in and out of interrupts, but
/// once writing starts in interrupts on a specific slot, the
/// slot will only be modified in interrupts until the slot is
/// is no longer live.
/// The size does not need to be a power of 2 anymore!
DDA BSS movebuffer[MOVEBUFFER_SIZE];

/**
  Pointer to the currently ongoing movement, or NULL, if there's no movement
  ongoing. Actually a cache of movebuffer[mb_tail].
*/
DDA *mb_tail_dda;

/// Find the next DDA index after 'x', where 0 <= x < MOVEBUFFER_SIZE
#define MB_NEXT(x) ((x) < MOVEBUFFER_SIZE - 1 ? (x) + 1 : 0)

/// check if the queue is completely full
uint8_t queue_full() {
	MEMORY_BARRIER();
  return MB_NEXT(mb_head) == mb_tail;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
void queue_step() {

  if (mb_tail_dda != NULL) {
    dda_step(mb_tail_dda);
	}

  /**
    Start the next move if this one is done and another one is available.

    This needs no atomic protection, because we're in an interrupt already.
  */
  if (mb_tail_dda == NULL || ! mb_tail_dda->live) {
    if (mb_tail != mb_head) {
      mb_tail = MB_NEXT(mb_tail);
      mb_tail_dda = &(movebuffer[mb_tail]);
      dda_start(mb_tail_dda);
    }
    else {
      mb_tail_dda = NULL;
    }
  }
}

/// add a move to the movebuffer
/// \note this function waits for space to be available if necessary, check queue_full() first if waiting is a problem
/// This is the only function that modifies mb_head and it always called from outside an interrupt.
void enqueue_home(TARGET *t, uint8_t endstop_check, uint8_t endstop_stop_cond) {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full())
		delay_us(10);
    //sersendf_P(PSTR("F%lu X%lq Y%lq Z%lq E%lq\n"),t->F,t->axis[X], t->axis[Y],t->axis[Z],t->axis[E]  );      

  uint8_t h = MB_NEXT(mb_head);

	DDA* new_movebuffer = &(movebuffer[h]);

  // Initialise queue entry to a known state. This also clears flags like
  // dda->live, dda->done and dda->wait_for_temp.
  new_movebuffer->allflags = 0;

  new_movebuffer->endstop_check = endstop_check;
  new_movebuffer->endstop_stop_cond = endstop_stop_cond;
  dda_create(new_movebuffer, t);

  /**
    It's pointless to queue up movements which don't actually move the stepper,
    e.g. pure velocity changes or movements shorter than a single motor step.

    That said, accept movements which do move the steppers by forwarding
    mb_head. Also kick off movements if it's the first movement after a pause.
  */
  if ( ! new_movebuffer->nullmove) {
    // make certain all writes to global memory
    // are flushed before modifying mb_head.
    MEMORY_BARRIER();

    mb_head = h;

    if (mb_tail_dda == NULL) {
      /**
        Go to the next move.

        This is the version used from outside interrupts. The in-interrupt
        version is inlined (and simplified) in queue_step().
      */
      timer_reset();
      mb_tail = mb_head;  // Valid ONLY if the queue was empty before!
      mb_tail_dda = new_movebuffer; // Dito!
      
      dda_start(mb_tail_dda);
      // Compensate for the cli() in timer_set().
      sei();
    }
	}
}

/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void print_queue() {
  sersendf_P(PSTR("Queue: %d/%d%c\n"), mb_tail, mb_head,
             (queue_full() ? 'F' : (mb_tail_dda ? ' ' : 'E')));
}

/// dump queue for emergency stop.
/// Make sure to have all timers stopped with timer_stop() or
/// unexpected things might happen.
/// \todo effect on startpoint is undefined!
void queue_flush() {

  // if the timer were running, this would require
  // wrapping in ATOMIC_START ... ATOMIC_END.
  mb_tail = mb_head;
  mb_tail_dda = NULL;
}

/// wait for queue to empty
void queue_wait() {
  while (mb_tail_dda)
		clock();
}

#define MM_PER_ARC_SEGMENT 1
#define MM_PER_ARC_SEGMENT_BIG 3
#define N_ARC_CORRECTION 25

#define ARCFLOAT
#ifdef ARC_SUPPORT

#ifdef ARCFLOAT
/*
 * 
 *  ARC using Float implementation 
 * 
 */
void draw_arc(TARGET *target, uint32_t fI,uint32_t fJ, uint8_t isclockwise)
{
    #define debug0
    uint32_t radius=approx_distance(labs(fI),labs(fJ));
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
                
    TARGET move;
    move=startpoint;
    move.F=target->F;
    move.axis[E] = startpoint.axis[E];
    int32_t cx = startpoint.axis[X] + (int32_t)fI ;
    int32_t cy = startpoint.axis[Y] + (int32_t)fJ ;  
        #ifdef debug1 
        sersendf_P(PSTR("Arc  I, J,R:%lq,%lq,%lq\n"),fI,fJ,radius);
        sersendf_P(PSTR("go cX cY:%lq %lq\n"), cx,cy);
        #endif
    
    //uint32_t linear_travel = 0; //target[axis_linear] - position[axis_linear];
    uint32_t extruder_travel = target->axis[E]-move.axis[E];
    float I=0.001f*(int32_t)fI;
    float J=0.001f*(int32_t)fJ;
    
    float r_axis0 = -I;  // Radius vector from center to current location
    float r_axis1 = -J;
    float rt_axis0 = 0.001f*((int32_t)target->axis[X] - cx);
    float rt_axis1 = 0.001f*((int32_t)target->axis[Y] - cy);
    
    #ifdef debug1 
        sersendf_P(PSTR("go rX rY:%lq %lq\n"), r_axis0*1000,r_axis1*1000);
        sersendf_P(PSTR("go rtX rtY:%lq %lq\n"), rt_axis0*1000,rt_axis1*1000);
        #endif
    /*long xtarget = Printer::destinationSteps[X_AXIS];
    long ytarget = Printer::destinationSteps[Y_AXIS];
    long ztarget = Printer::destinationSteps[Z_AXIS];
    long etarget = Printer::destinationSteps[E_AXIS];
    */
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001))
    {
        angular_travel += 2.0f * M_PI;
    }
    if (isclockwise)
    {
        angular_travel -= 2.0f * M_PI;
    }

    uint32_t millimeters_of_travel = fabs(angular_travel)*radius; //hypot(angular_travel*radius, fabs(linear_travel));
    #ifdef debug1 
    sersendf_P(PSTR("radius:%lq Distance:%lq\n"),
                        radius,millimeters_of_travel);
    #endif
    if (millimeters_of_travel < 1)
    {
        return;// treat as succes because there is nothing to do;
    }
    //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint32_t segments = millimeters_of_travel >>11;
    uint32_t ang= angular_travel*1000;
    #ifdef debug1 
    sersendf_P(PSTR("Segmen arc:%lu %lq\n"),
                        segments,ang);
    #endif
    if(segments == 0) segments = 1;
    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel / segments;
    //uint32_t linear_per_segment = linear_travel/segments;
    uint32_t extruder_per_segment = (extruder_travel) / segments;
    
    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. This requires only two cos() and sin() computations to form the rotation
       matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       all double numbers are single precision on the Arduino. (True double precision will not have
       round off issues for CNC applications.) Single precision error can accumulate to be greater than
       tool precision in some cases. Therefore, arc path correction is implemented.

       Small angle approximation may be used to reduce computation overhead further. This approximation
       holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
       theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       issue for CNC machines with the single precision Arduino calculations.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    //float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
    //float sin_T = theta_per_segment;
    
    float cos_T = 2.0 - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667*(cos_T + 4.0);
    cos_T *= 0.5;
    
    #ifdef debug1 
        sersendf_P(PSTR("go cost sint:%lq %lq\n"), cos_T*1000,sin_T*1000);
        #endif
    
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;
 
    // Initialize the linear axis
    //arc_target[axis_linear] = position[axis_linear];

    // Initialize the extruder axis

    
    for (i = 1; i < segments; i++)
    {
        // Increment (segments-1)

        

        if (count < N_ARC_CORRECTION)  //25 pieces
        {
            // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        }
        else
        {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti  = cos(i*theta_per_segment);
            sin_Ti  = sin(i*theta_per_segment);
            r_axis0 = -I * cos_Ti + J * sin_Ti;
            r_axis1 = -I * sin_Ti - J * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        move.axis[X] = cx+r_axis0*1000.0f;
        move.axis[Y] = cy+r_axis1*1000.0f;
        //arc_target[axis_linear] += linear_per_segment;
        #ifdef debug1 
        sersendf_P(PSTR("go X Y:%lq %lq\n"), move.axis[X],move.axis[Y]);
        #endif
        move.axis[E]+=extruder_per_segment;
        //move.axis[E] = extscaled>>4;
        enqueue(&move);
    }
    // Ensure last segment arrives at target location.
    enqueue(target);
}
#endif
#endif