/*---------------------------------------------------------------------------/
/  FatFs Functional Configurations
/---------------------------------------------------------------------------*/

#ifndef _FFCONF
#define _FFCONF 68300

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define _FS_READONLY	0		/* 0:Read/Write or 1:Read only */
#define _FS_MINIMIZE	0		/* 0:Normal or 1:Minimize code size or 2:Minimize data size */
#define _FS_EXFAT		0		/* 0:Disable or 1:Enable exFAT file system */

#define _USE_STRFUNC	2		/* 0:Disable, 1:f_gets only, 2:f_gets and f_puts */
#define _USE_FIND		0		/* 0:Disable, 1:Enable */
#define _USE_MKFS		1		/* 0:Disable, 1:Enable — needed for DataLogger.c's
									   auto-format fallback when the card has no FAT
									   filesystem FatFs recognizes (FR_NO_FILESYSTEM) */
#define _USE_FASTSEEK	0		/* 0:Disable, 1:Enable */
#define _USE_LABEL		0		/* 0:Disable, 1:Enable */
#define _USE_FORWARD	0		/* 0:Disable, 1:Enable */
#define _USE_EXPAND		0		/* 0:Disable, 1:Enable */


/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define _CODE_PAGE	850
/* 0:SBCS CP437, 1:SBCS CP720, 2:SBCS CP737, 3:SBCS CP771, 4:SBCS CP775
 * 5:SBCS CP850, 6:SBCS CP852, 7:SBCS CP855, 8:SBCS CP857, 9:SBCS CP858
 * 10:SBCS CP862, 11:SBCS CP866, 12:SBCS CP869, 13:SBCS CP932, 14:SBCS CP936
 * 15:SBCS CP949, 16:SBCS CP950, 17:SBCS CP1252, 18:BIG5, 19:GB2312, 20:GB18030
 * 21:UTF-16BE, 22:UTF-16LE, 23:UTF-8 */

#define _USE_LFN	0		/* 0:Disable or 1:Enable LFN with static working buffer or 2:Enable LFN with dynamic working buffer */
#define _MAX_LFN	255		/* Max. length of LFN (12 to 255) */

#define _LFN_UNICODE	0		/* 0:ANSI/OEM or 1:Unicode */
#define _STRF_ENCODE	3		/* 0:ANSI/OEM, 1:UTF-16LE, 2:UTF-16BE, 3:UTF-8 */

#define _FS_RPATH	0		/* 0:Disable or 1:Enable relative path feature */


/*---------------------------------------------------------------------------/
/ Drive/Physical Drive Configurations
/---------------------------------------------------------------------------*/

#define _VOLUMES	1		/* Number of volumes (1-10) */

#define _STR_VOLUME_ID	0		/* 0:Use only numeric ID, 1:Include volume label */
#define _VOLUME_STRS	"RAM","NAND","CF","SD","SD2","USB","USB2","USB3"

#define _MULTI_PARTITION	0	/* 0:Single partition, 1:Enable multi-partition */

#define _MIN_SS		512		/* Min sector size (512, 1024, 2048 or 4096) */
#define _MAX_SS		512		/* Max sector size (512, 1024, 2048 or 4096) */
#define _USE_TRIM	0		/* 0:Disable or 1:Enable sector erase feature */

#define _FS_NOFSINFO	2		/* 0:Leave FSINFO, 1:Remove FSINFO, 2:Auto detect */


/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define _FS_TINY	1		/* 0:Normal or 1:Tiny (TCB=366 bytes in default config, unavailable with exFAT) */
#define _FS_NORTC	1		/* 0:Enable RTC or 1:Disable RTC */

#define _NORTC_MON	1
#define _NORTC_MDAY	1
#define _NORTC_YEAR	2017

#define _FS_LOCK	0		/* 0:Disable, >0:Enable (number of files that can be opened simultaneously) */
#define _FS_REENTRANT	0	/* 0:Disable, 1:Enable (not thread-safe) */
#define _FS_TIMEOUT		1000	/* Timeout period in unit of ms */
#define _SYNC_t	HANDLE		/* O/S dependent sync object type. e.g. HANDLE, ID, OS_EVENT*, SemaphoreHandle_t and etc. */



#define _FS_NOLIB		1		/* 0:Enable creation of chain of singly-linked list, 1:Disable */
#define _FS_DAT		20		/* Date SFN 8.3 feature */


#endif /* _FFCONF */