/*
 * myscratch_test.c - userspace test program for /dev/myscratch
 *
 * Opens the device, asks it (via ioctl) how big the mmap-able region
 * is, maps it, writes a pattern directly through the mapped pointer
 * (no read()/write() syscalls at all - this is the whole point), reads
 * it back to confirm, then exercises the RESET ioctl and confirms the
 * buffer actually went back to zero.
 *
 * Build: gcc -o myscratch_test myscratch_test.c
 * Run inside the guest (needs to run where /dev/myscratch exists,
 * i.e. after insmod'ing the driver).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define MYSCRATCH_IOC_MAGIC 'k'
#define MYSCRATCH_IOC_GET_SIZE _IOR(MYSCRATCH_IOC_MAGIC, 1, uint32_t)
#define MYSCRATCH_IOC_RESET    _IO(MYSCRATCH_IOC_MAGIC, 2)

int main(void)
{
    int fd;
    uint32_t size;
    void *map;
    unsigned char *buf;
    size_t i;
    int ok = 1;

    fd = open("/dev/myscratch", O_RDWR);
    if (fd < 0) {
        perror("open /dev/myscratch");
        return 1;
    }

    if (ioctl(fd, MYSCRATCH_IOC_GET_SIZE, &size) < 0) {
        perror("ioctl GET_SIZE");
        close(fd);
        return 1;
    }
    printf("device reports scratch size: %u bytes\n", size);

    map = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }
    buf = (unsigned char *)map;

    printf("writing pattern directly through the mapped pointer...\n");
    for (i = 0; i < size; i++) {
        buf[i] = (unsigned char)(i & 0xff);
    }

    printf("reading it back...\n");
    for (i = 0; i < size; i++) {
        if (buf[i] != (unsigned char)(i & 0xff)) {
            printf("MISMATCH at offset %zu: got 0x%02x, want 0x%02x\n",
                   i, buf[i], (unsigned char)(i & 0xff));
            ok = 0;
            break;
        }
    }
    if (ok) {
        printf("PASS: pattern read back correctly (%u bytes, pure mmap access)\n",
               size);
    }

    printf("issuing RESET ioctl...\n");
    if (ioctl(fd, MYSCRATCH_IOC_RESET) < 0) {
        perror("ioctl RESET");
        ok = 0;
    } else {
        int all_zero = 1;
        for (i = 0; i < size; i++) {
            if (buf[i] != 0) {
                all_zero = 0;
                break;
            }
        }
        printf("%s: buffer is %s after reset\n",
               all_zero ? "PASS" : "FAIL",
               all_zero ? "all zero" : "NOT all zero");
        ok = ok && all_zero;
    }

    munmap(map, size);
    close(fd);

    return ok ? 0 : 1;
}
