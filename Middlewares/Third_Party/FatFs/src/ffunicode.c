/* Minimal stub for ffunicode.c
 * This project config disables LFN/unicode in Core/Inc/ffconf.h,
 * but the Makefile expects ffunicode.c next to ff.c. Provide a
 * small stub so builds succeed without pulling the full FatFs
 * Unicode implementation.
 */

#include "ff.h"

/* No Unicode functions are needed when LFN/Unicode is disabled.
 * Provide weak stubs to avoid linker or compile issues if any
 * configuration expects these symbols to exist.
 */

#if _USE_LFN != 0 && _LFN_UNICODE
WCHAR ff_convert (WCHAR chr, UINT dir) { return chr; }
WCHAR ff_wtoupper (WCHAR chr) { return chr; }
#if _USE_LFN == 3
void* ff_memalloc (UINT msize) { return 0; }
void ff_memfree (void* mblock) { (void)mblock; }
#endif
#endif
