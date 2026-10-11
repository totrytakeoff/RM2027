#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>

#include "SEGGER_RTT.h"
#include "stm32f4xx.h"

/*
 * Minimal newlib process I/O boundary for bare-metal firmware.
 *
 * stdout/stderr are deliberately lossy RTT diagnostic streams.  Unsupported
 * process and file operations fail explicitly instead of being supplied by
 * libnosys, whose warning stubs otherwise hide accidental hosted assumptions.
 * Heap growth is intentionally not implemented: the formal firmware's
 * post-link audit rejects _sbrk and every allocator entry point.
 */

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    if ((status == NULL) || ((file != 0) && (file != 1) && (file != 2))) {
        errno = EBADF;
        return -1;
    }

    *status = (struct stat){0};
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    if ((file == 0) || (file == 1) || (file == 2)) {
        return 1;
    }

    errno = EBADF;
    return 0;
}

int _lseek(int file, int offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    errno = ESPIPE;
    return -1;
}

int _read(int file, char *buffer, int length)
{
    (void)buffer;
    (void)length;

    errno = (file == 0) ? EAGAIN : EBADF;
    return -1;
}

int _write(int file, char *buffer, int length)
{
    if (((file != 1) && (file != 2)) || (buffer == NULL) || (length < 0)) {
        errno = EBADF;
        return -1;
    }
    if (length == 0) {
        return 0;
    }

    /* RTT channel 0 is non-blocking; dropped diagnostics must not stall control. */
    (void)SEGGER_RTT_Write(0U, buffer, (unsigned)length);
    return length;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int signal)
{
    (void)pid;
    (void)signal;
    errno = EINVAL;
    return -1;
}

__attribute__((noreturn)) void _exit(int status)
{
    (void)status;
    __disable_irq();
    for (;;) {
        __WFI();
    }
}
