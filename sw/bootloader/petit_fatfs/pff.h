/*---------------------------------------------------------------------------/
/  Petit FatFs - FAT file system module include file  R0.03a
/----------------------------------------------------------------------------/
/ Petit FatFs module is an open source software to implement FAT file system to
/ small embedded systems. This is a free software and is opened for education,
/ research and commercial developments under license policy of following trems.
/
/  Copyright (C) 2019, ChaN, all right reserved.
/
/ * The Petit FatFs module is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/----------------------------------------------------------------------------*/

#ifndef PF_DEFINED
#define PF_DEFINED  8088  /* Revision ID */

#ifdef __cplusplus
extern "C" {
#endif

#include "pffconf.h"

#if PF_DEFINED != PFCONF_DEF
#error Wrong configuration file (pffconf.h).
#endif


/* Integer types used for FatFs API */

#if defined(_WIN32)  /* Main development platform */
#include <windows.h>
#elif (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L) || defined(__cplusplus)  /* C99 or later */
#include <stdint.h>
typedef unsigned int  UINT;  /* int must be 16-bit or 32-bit */
typedef unsigned char  BYTE;  /* char must be 8-bit */
typedef uint16_t    WORD;  /* 16-bit unsigned integer */
typedef uint16_t    WCHAR;  /* 16-bit unsigned integer */
typedef uint32_t    DWORD;  /* 32-bit unsigned integer */
#else    /* Earlier than C99 */
typedef unsigned int  UINT;  /* int must be 16-bit or 32-bit */
typedef unsigned char  BYTE;  /* char must be 8-bit */
typedef unsigned short  WORD;  /* 16-bit unsigned integer */
typedef unsigned short  WCHAR;  /* 16-bit unsigned integer */
typedef unsigned long  DWORD;  /* 32-bit unsigned integer */
#endif
#define PF_INTDEF 1


#if PF_FS_FAT32
#define  CLUST  DWORD
#else
#define  CLUST  WORD
#endif


/* File system object structure */

typedef struct {
  BYTE  fs_type;  /* FAT sub type */
  BYTE  flag;    /* File status flags */
  BYTE  csize;    /* Number of sectors per cluster */
  BYTE  pad1;
  WORD  n_rootdir;  /* Number of root directory entries (0 on FAT32) */
  CLUST  n_fatent;  /* Number of FAT entries (= number of clusters + 2) */
  DWORD  fatbase;  /* FAT start sector */
  DWORD  dirbase;  /* Root directory start sector (Cluster# on FAT32) */
  DWORD  database;  /* Data start sector */
  DWORD  fptr;    /* File R/W pointer */
  DWORD  fsize;    /* File size */
  CLUST  org_clust;  /* File start cluster */
  CLUST  curr_clust;  /* File current cluster */
  DWORD  dsect;    /* File current data sector */
} FATFS;



/* Directory object structure */

typedef struct {
  WORD  index;    /* Current read/write index number */
  BYTE*  fn;      /* Pointer to the SFN (in/out) {file[8],ext[3],status[1]} */
  CLUST  sclust;    /* Table start cluster (0:Static table) */
  CLUST  clust;    /* Current cluster */
  DWORD  sect;    /* Current sector */
} DIR;



/* File status structure */

typedef struct {
  DWORD  fsize;    /* File size */
  WORD  fdate;    /* Last modified date */
  WORD  ftime;    /* Last modified time */
  BYTE  fattrib;  /* Attribute */
  char  fname[13];  /* File name */
} FILINFO;



/* File function return code (FRESULT) */

typedef enum {
  FR_OK = 0,      /* 0 */
  FR_DISK_ERR,    /* 1 */
  FR_NOT_READY,    /* 2 */
  FR_NO_FILE,      /* 3 */
  FR_NOT_OPENED,    /* 4 */
  FR_NOT_ENABLED,    /* 5 */
  FR_NO_FILESYSTEM  /* 6 */
} FRESULT;



/*--------------------------------------------------------------*/
/* Petit FatFs module application interface                     */

FRESULT pf_mount (FATFS* fs);                /* Mount/Unmount a logical drive */
FRESULT pf_open (const char* path);              /* Open a file */
FRESULT pf_read (void* buff, UINT btr, UINT* br);      /* Read data from the open file */
FRESULT pf_write (const void* buff, UINT btw, UINT* bw);  /* Write data to the open file */
FRESULT pf_lseek (DWORD ofs);                /* Move file pointer of the open file */
FRESULT pf_opendir (DIR* dj, const char* path);        /* Open a directory */
FRESULT pf_readdir (DIR* dj, FILINFO* fno);          /* Read a directory item from the open directory */



/*--------------------------------------------------------------*/
/* Flags and offset address                                     */


/* File status flag (FATFS.flag) */
#define  FA_OPENED  0x01
#define  FA_WPRT    0x02
#define  FA__WIP    0x40


/* FAT sub type (FATFS.fs_type) */
#define FS_FAT12  1
#define FS_FAT16  2
#define FS_FAT32  3


/* File attribute bits for directory entry */

#define  AM_RDO  0x01  /* Read only */
#define  AM_HID  0x02  /* Hidden */
#define  AM_SYS  0x04  /* System */
#define  AM_VOL  0x08  /* Volume label */
#define AM_LFN  0x0F  /* LFN entry */
#define AM_DIR  0x10  /* Directory */
#define AM_ARC  0x20  /* Archive */
#define AM_MASK  0x3F  /* Mask of defined bits */


#ifdef __cplusplus
}
#endif

#endif /* _PFATFS */
