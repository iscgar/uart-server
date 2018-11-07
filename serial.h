/************************************************************************************
 * serial.h -
 *  A wrapper over platform-specific serial port functionality.
 *
 *   Copyright (c) 2018 Isaac Garzon
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 ************************************************************************************/

#ifndef INCLUDE_GUARD_7AF18658_33EE_48A2_999F_02F5D03895F9
#define INCLUDE_GUARD_7AF18658_33EE_48A2_999F_02F5D03895F9

#include <stddef.h>

/**
 * Windows/POSIX compatibility glue.
 */
#if defined(_WIN32)
#   include <Windows.h>

#   define SERIAL_INVALID_FD INVALID_HANDLE_VALUE

typedef HANDLE SerialFd;
#else /* if !defined(_WIN32) */
#   define SERIAL_INVALID_FD (-1)

typedef int SerialFd;
#endif /* !defined(_WIN32) */

#ifndef NULL
#   define NULL 0
#endif /* !NULL */

#ifndef FALSE
#   define FALSE 0
#endif /* !FALSE */

#ifndef TRUE
#   define TRUE (!FALSE)
#endif /* !TRUE */

enum SerialParity
{
    e_parity_none,
    e_parity_odd,
    e_parity_even
};

enum SerialStopBits
{
    e_stop_bits_one,
    e_stop_bits_one_half,
    e_stop_bits_two
};

struct SerialConfig
{
    unsigned            baudrate;
    enum SerialParity   parity;
    int                 data_bits;
    enum SerialStopBits stop_bits;
};

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Opens a serial port.
 *
 * @param port_s    The name of the serial port to open (e.g. "COM1" on Windows, or "/dev/ttyS0" on Linux)
 * @param cfg       Pointer to serial port configuration info.
 * @param timeout   The read timeout (in milliseconds) to set on the port. A negative value for infinity.
 *
 * @return          An OS handle to the open serial port if successful. `SERIAL_INVALID_FD` otherwise.
 */
extern SerialFd serial_open(const char *port_s, const struct SerialConfig *cfg, int timeout);

/**
 * Reads data from a serial port.
 *
 * @param fd    A handle to an open serial port returned from `serial_open`.
 * @param o_buf Pointer to an output memory buffer to read data into.
 * @param size  The amount of bytes to read.
 *
 * @return      The amount of bytes written into `o_buf` on success. -1 on error.
 */
extern long serial_read(SerialFd fd, unsigned char *o_buf, size_t size);

/**
 * Writes data into a serial port.
 *
 * @param fd    A handle to an open serial port returned from `serial_open`.
 * @param buf   Pointer to a memory buffer containing data to write into the serial port.
 * @param size  The amount of bytes to write.
 *
 * @return      TRUE if all the bytes were successfully written into the serial port. FALSE otherwise.
 */
extern int serial_write(SerialFd fd, const unsigned char *buf, size_t size);

/**
 * Closes an open serial port.
 *
 * @param fd    A handle to an open serial port returned from `serial_open`.
 */
extern void serial_close(SerialFd fd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !INCLUDE_GUARD_7AF18658_33EE_48A2_999F_02F5D03895F9 */
