/*---------------------------------------------------------------------------/
/  Petit FatFs - Configuration file
/---------------------------------------------------------------------------*/

#ifndef PFCONF_DEF
#define PFCONF_DEF 8088  /* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations (0:Disable, 1:Enable)
/---------------------------------------------------------------------------*/

#define  PF_USE_READ   1  /* pf_read() function */
#define  PF_USE_DIR    0  /* pf_opendir() and pf_readdir() function */
#define  PF_USE_LSEEK  0  /* pf_lseek() function */
#define  PF_USE_WRITE  0  /* pf_write() function */

#define PF_FS_FAT12    0  /* FAT12 */
#define PF_FS_FAT16    0  /* FAT16 */
#define PF_FS_FAT32    1  /* FAT32 */


/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define PF_USE_LCC    1  /* Allow lower case ASCII and non-ASCII chars */

#define  PF_CODE_PAGE  437
/* The PF_CODE_PAGE specifies the code page to be used on the target system.
/  SBCS code pages with PF_USE_LCC == 1 requiers a 128 byte of case conversion
/  table. It might occupy RAM on some platforms, e.g. avr-gcc.
/  When PF_USE_LCC == 0, PF_CODE_PAGE has no effect.
/
/   437 - U.S.
/   720 - Arabic
/   737 - Greek
/   771 - KBL
/   775 - Baltic
/   850 - Latin 1
/   852 - Latin 2
/   855 - Cyrillic
/   857 - Turkish
/   860 - Portuguese
/   861 - Icelandic
/   862 - Hebrew
/   863 - Canadian French
/   864 - Arabic
/   865 - Nordic
/   866 - Russian
/   869 - Greek 2
/   932 - Japanese (DBCS)
/   936 - Simplified Chinese (DBCS)
/   949 - Korean (DBCS)
/   950 - Traditional Chinese (DBCS)
*/

#endif /* PF_CONF */
