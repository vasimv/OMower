// Configuration for *printf() library
// https://github.com/weiss/c99-snprintf

#ifndef SYSTEM_H
#define SYSTEM_H

#define HAVE_STDARG_H 1
#define HAVE_STDARG_H 1
#define HAVE_STDDEF_H 1
#define HAVE_STDINT_H 1
#define HAVE_STDLIB_H 1
#define HAVE_FLOAT_H 1
#define HAVE_INTTYPES_H 1



// HAVE_VSNPRINTF                                                                                        
//#define HAVE_SNPRINTF 1
#define HAVE_VASPRINTF 1
#define HAVE_ASPRINTF 1

#if HAVE_STDARG_H
#include <stdarg.h>
#if HAVE_STDDEF_H
#include <stddef.h>
#endif
#if !HAVE_VSNPRINTF
int rpl_vsnprintf(char *, size_t, const char *, va_list);
#endif  /* !HAVE_VSNPRINTF */
#if !HAVE_SNPRINTF
int rpl_snprintf(char *, size_t, const char *, ...);
#endif  /* !HAVE_SNPRINTF */
#if !HAVE_VASPRINTF
int rpl_vasprintf(char **, const char *, va_list);
#endif  /* !HAVE_VASPRINTF */
#if !HAVE_ASPRINTF
int rpl_asprintf(char **, const char *, ...);
#endif  /* !HAVE_ASPRINTF */
#endif  /* HAVE_STDARG_H */
#endif  /* SYSTEM_H */
