#include <errno.h>
#include <stddef.h>
/* Startup explicitly runs __libc_init_array without pulling in a second CRT. */
void _init(void) {}
void _fini(void) {}
/* No firmware heap: library allocations fail instead of growing into static RAM. */
void *_sbrk(ptrdiff_t increment)
{
    (void)increment; errno=ENOMEM; return (void *)-1;
}
