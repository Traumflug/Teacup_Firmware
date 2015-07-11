
/** \file
  \brief CPU initialisation, ARM specific part.

  To be included from cpu.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__


/** SystemInit will be called before main from startup assembly code.
*/
void SystemInit(void) {
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
