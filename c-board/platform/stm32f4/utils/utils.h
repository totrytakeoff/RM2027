#ifndef RM_FORMAT_H
#define RM_FORMAT_H

#include <stdarg.h>
#include <stddef.h>

/** Bounded formatter that does not use stdio or dynamic memory. */
int RmFormat_Vsnprintf(char *buffer,
                       size_t buffer_size,
                       const char *format,
                       va_list args);

/** Variadic wrapper for RmFormat_Vsnprintf. */
int RmFormat_Snprintf(char *buffer,
                      size_t buffer_size,
                      const char *format,
                      ...);

/* Transitional aliases retained for existing diagnostics demos. */
int safe_vsnprintf(char *buffer,
                   size_t buffer_size,
                   const char *format,
                   va_list args);
int safe_snprintf(char *buffer,
                  size_t buffer_size,
                  const char *format,
                  ...);

#endif /* RM_FORMAT_H */
