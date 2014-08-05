/*
	This file contains defines that change the way the step/dir pins work, such
	that they become interchangable graycode signaling pins. If your stepper 
	driver expects coil polarity signals instead of step/dir, this will allow
	them to work with reprap.

	By default, X Y and Z are graycode, while E remains step/dir. If this isn't what
	you want, edit the lines just below.
*/
// #define U_GREYCODE
// #define V_GREYCODE
// #define X_GREYCODE
// #define Y_GREYCODE
// #define Z_GREYCODE
// #define E_GREYCODE


/*
	U Stepper
*/
#ifdef U_GREYCODE
	#undef u_step
	#undef _u_step
	#undef u_direction
	#define u_step()                        {\
												u_greycode+=stored_u_direction;\
												WRITE(U_STEP_PIN,(u_greycode>>1)&1);\
												WRITE(U_DIR_PIN,((u_greycode>>1)^u_greycode)&1);\
											}
	#define _u_step(st)                    
	#define u_direction(dir)                stored_u_direction=(dir)?1:-1
	int8_t stored_u_direction;
	int8_t u_greycode;
#endif

/*
	V Stepper
*/

#ifdef V_GREYCODE
	#undef v_step
	#undef _v_step
	#undef v_direction
	#define v_step()                        {\
												v_greycode+=stored_v_direction;\
												WRITE(V_STEP_PIN,(v_greycode>>1)&1);\
												WRITE(V_DIR_PIN,((v_greycode>>1)^v_greycode)&1);\
											}
	#define _v_step(st)                     
	#define v_direction(dir)                stored_v_direction=(dir)?1:-1
	int8_t stored_v_direction;
	int8_t v_greycode;
#endif

/*
	X Stepper
*/
#ifdef X_GREYCODE
	#undef x_step
	#undef _x_step
	#undef x_direction
	#define x_step()                        {\
												x_greycode+=stored_x_direction;\
												WRITE(X_STEP_PIN,(x_greycode>>1)&1);\
												WRITE(X_DIR_PIN,((x_greycode>>1)^x_greycode)&1);\
											}
	#define _x_step(st)                    
	#define x_direction(dir)                stored_x_direction=(dir)?1:-1
	int8_t stored_x_direction;
	int8_t x_greycode;
#endif

/*
	Y Stepper
*/

#ifdef Y_GREYCODE
	#undef y_step
	#undef _y_step
	#undef y_direction
	#define y_step()                        {\
												y_greycode+=stored_y_direction;\
												WRITE(Y_STEP_PIN,(y_greycode>>1)&1);\
												WRITE(Y_DIR_PIN,((y_greycode>>1)^y_greycode)&1);\
											}
	#define _y_step(st)                     
	#define y_direction(dir)                stored_y_direction=(dir)?1:-1
	int8_t stored_y_direction;
	int8_t y_greycode;
#endif

/*
	Z Stepper
*/

#ifdef Z_GREYCODE
	#undef z_step
	#undef _z_step
	#undef z_direction
	#define z_step()                        {\
												z_greycode+=stored_z_direction;\
												WRITE(Z_STEP_PIN,(z_greycode>>1)&1);\
												WRITE(Z_DIR_PIN,((z_greycode>>1)^z_greycode)&1);\
											}
	#define _z_step(st)                    
	#define z_direction(dir)                stored_z_direction=(dir)?1:-1
	int8_t stored_z_direction;
	int8_t z_greycode;
#endif

/*
	Extruder
*/

#ifdef E_GREYCODE
	#undef e_step
	#undef _e_step
	#undef e_direction
	#define e_step()                        {\
												e_greycode+=stored_e_direction;\
												WRITE(E_STEP_PIN,(e_greycode>>1)&1);\
												WRITE(E_DIR_PIN,((e_greycode>>1)^e_greycode)&1);\
											}
	#define _e_step(st)                     
	#define e_direction(dir)                stored_e_direction=(dir)?1:-1
	int8_t stored_e_direction;
	int8_t e_greycode;
#endif


