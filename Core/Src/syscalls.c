#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include "board.h"
extern char end, _estack;
void *_sbrk(ptrdiff_t increment) {
    static char *heap;
    if (!heap) heap = &end;
    char *old = heap;
    if (increment < 0 || (uintptr_t)heap + (uintptr_t)increment > (uintptr_t)&_estack - 4096) {
        errno = ENOMEM; return (void *)-1;
    }
    heap += increment;
    return old;
}
int _write(int file, char *data, int length) {
    if (file != 1 && file != 2) { errno = EBADF; return -1; }
    for (int offset = 0; offset < length; offset += 250) {
        int n = length - offset;
        if (n > 250) n = 250;
        board_log("%.*s", n, data + offset);
    }
    return length;
}
int _read(int file, char *data, int length) {
    if (file != 0) { errno = EBADF; return -1; }
    int count = 0;
    while (count < length) {
        int c = board_uart_getchar();
        if (c < 0) break;
        data[count++] = (char)c;
    }
    if (!count) { errno = EAGAIN; return -1; }
    return count;
}
int _close(int file) { (void)file; errno = EBADF; return -1; }
int _fstat(int file, struct stat *st) {
    if (file < 0 || file > 2) { errno = EBADF; return -1; }
    st->st_mode = S_IFCHR; return 0;
}
int _isatty(int file) { return file >= 0 && file <= 2; }
int _lseek(int file, int offset, int origin) {
    (void)file; (void)offset; (void)origin; errno = ESPIPE; return -1;
}
int _getpid(void) { return 1; }
int _kill(int pid, int signal) { (void)pid; (void)signal; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; board_emergency_off(); for (;;) {} }
