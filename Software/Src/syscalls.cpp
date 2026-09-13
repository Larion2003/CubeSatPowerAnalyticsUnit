/**
 * @file syscalls.cpp
 * @brief Minimal system call implementation for bare-metal STM32.
 * @details Provides the necessary stubs for the standard C library (libc_nano),
 * specifically for memory allocation (sbrk) required by sprintf.
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <cstddef>

/**
 * Symbol defined in the linker script (usually .ld file).
 * It marks the start of the free RAM area (the heap).
 */
extern "C" char _end;

extern "C" {
    /**
     * @brief Low-level memory allocation.
     * @param incr Number of bytes to increment the heap.
     * @return Pointer to the previous end of the heap.
     * Used by malloc() and formatted string functions like sprintf().
     */
    void* _sbrk(ptrdiff_t incr) {
        static char* heap_end = nullptr;
        char* prev_heap_end;

        if (heap_end == nullptr) {
            heap_end = &_end;
        }

        prev_heap_end = heap_end;

        /* Note: In production, one should check if heap_end + incr
         * exceeds the stack pointer to prevent memory corruption.
         */
        heap_end += incr;

        return (void*)prev_heap_end;
    }

    /* Minimal system stubs required for successful linking with nano.specs */

    int _close(int file) { (void)file; return -1; }

    int _fstat(int file, struct stat *st) {
        (void)file;
        st->st_mode = S_IFCHR;
        return 0;
    }

    int _isatty(int file) { (void)file; return 1; }

    int _lseek(int file, int ptr, int dir) {
        (void)file; (void)ptr; (void)dir;
        return 0;
    }

    int _read(int file, char *ptr, int len) {
        (void)file; (void)ptr; (void)len;
        return 0;
    }

    int _write(int file, char *ptr, int len) {
        (void)file; (void)ptr; (void)len;
        return 0;
    }

    void _exit(int status) {
        (void)status;
        while(1);
    }

    int _getpid(void) { return 1; }

    int _kill(int pid, int sig) {
        (void)pid; (void)sig;
        errno = EINVAL;
        return -1;
    }
}
