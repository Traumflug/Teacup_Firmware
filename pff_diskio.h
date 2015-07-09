/*-----------------------------------------------------------------------
/  PFF - Low level disk interface modlue include file    (C)ChaN, 2014
/-----------------------------------------------------------------------*/
/* Changes for Teacup see pff.c. */

#include "sd.h"

#ifdef SD

#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "pff_conf.h"


/* Status of Disk Functions */
typedef BYTE    DSTATUS;


/* Results of Disk Functions */
typedef enum {
    RES_OK = 0,     /* 0: Function succeeded */
    RES_ERROR,      /* 1: Disk error */
    RES_NOTRDY,     /* 2: Not ready */
    RES_PARERR,     /* 3: Invalid parameter */
    RES_EOL_FOUND,  /* 4: Function succeeded and also found an EOL. */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

DSTATUS disk_initialize (void);
#if _USE_READ
  DRESULT disk_readp (BYTE* buffer, DWORD sector, UINT offset, UINT count);
  DRESULT disk_parsep (DWORD sector, UINT offset, UINT* count,
                       uint8_t (*parser)(uint8_t));
#endif
#if _USE_WRITE
  DRESULT disk_writep (const BYTE* buffer, DWORD sc);
#endif

#define STA_NOINIT      0x01    /* Drive not initialized */
#define STA_NODISK      0x02    /* No medium in the drive */

#ifdef __cplusplus
}
#endif

#endif  /* _DISKIO_DEFINED */

#endif /* SD */
