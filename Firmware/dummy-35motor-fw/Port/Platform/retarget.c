#include <errno.h>
#include <sys/time.h>
#include <Platform/retarget.h>
#include "usart.h"

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

#ifndef MOTOR_DEBUG_UART
#define MOTOR_DEBUG_UART 0
#endif

#define MOTOR_DEBUG_UART_TIMEOUT_MS 20U

UART_HandleTypeDef *gHuart;

void RetargetInit(UART_HandleTypeDef *huart)
{
    gHuart = huart;

    /* Disable I/O buffering for STDOUT stream, so that
     * chars are sent out as soon as they are printed. */
    setvbuf(stdout, NULL, _IONBF, 0);
}

int _isatty(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

int _write(int fd, char *ptr, int len)
{
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
    {
#if MOTOR_DEBUG_UART
        if (__get_IPSR() != 0U || gHuart == NULL || ptr == NULL || len < 0)
        {
            errno = EWOULDBLOCK;
            return -1;
        }
        if (HAL_UART_Transmit(gHuart, (uint8_t *) ptr, (uint16_t) len,
                             MOTOR_DEBUG_UART_TIMEOUT_MS) != HAL_OK)
        {
            errno = EIO;
            return -1;
        }
#else
        (void) ptr;
        (void) len;
#endif
        return len;
    } else
        return -1;
}

int _close(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    errno = EBADF;
    return -1;
}

int _lseek(int fd, int ptr, int dir)
{
    (void) fd;
    (void) ptr;
    (void) dir;

    errno = EBADF;
    return -1;
}

int _read(int fd, char *ptr, int len)
{
    if (fd == STDIN_FILENO)
    {
#if MOTOR_DEBUG_UART
        if (__get_IPSR() != 0U || gHuart == NULL || ptr == NULL || len <= 0)
        {
            errno = EWOULDBLOCK;
            return -1;
        }
        const HAL_StatusTypeDef hstatus = HAL_UART_Receive(
            gHuart, (uint8_t *) ptr, 1, MOTOR_DEBUG_UART_TIMEOUT_MS);
        if (hstatus == HAL_OK)
            return 1;
        errno = EIO;
#else
        (void) ptr;
        (void) len;
        errno = EWOULDBLOCK;
#endif
        return -1;
    }
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return 0;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)
