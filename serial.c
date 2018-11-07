/************************************************************************************
 * serial.c -
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

#include <string.h>
#include "serial.h"

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined(_WIN32)
#   include <tchar.h>

    SerialFd serial_open(const char *port_s, const struct SerialConfig *cfg, int timeout)
    {
        DCB comstate;
        SerialFd fd;
        TCHAR portname[sizeof(_T("\\\\.\\COM256")) / sizeof(TCHAR)];

        if (_stprintf_s(portname, ARRAY_SIZE(portname) - 1, _T("\\\\.\\%s"), port_s) >= (int)ARRAY_SIZE(portname))
        {
            return SERIAL_INVALID_FD;
        }

        fd = CreateFile(portname, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

        if (fd != SERIAL_INVALID_FD)
        {
            COMMTIMEOUTS timeouts;

            if (!GetCommState(fd, &comstate))
            {
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }

            comstate.BaudRate = cfg->baudrate; /* Set the baud rate */
            comstate.fBinary = 1; /* Operate in binary mode */
            comstate.fParity = cfg->parity != e_parity_none; /* Set if parity should be active */

            switch (cfg->parity)
            {
            case e_parity_none:
                comstate.Parity = NOPARITY;
                break;

            case e_parity_odd:
                comstate.Parity = ODDPARITY;
                break;

            case e_parity_even:
                comstate.Parity = EVENPARITY;
                break;

            default:
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }

            comstate.ByteSize = cfg->data_bits; /* Set byte size */

            switch (cfg->stop_bits)
            {
            case e_stop_bits_one:
                comstate.StopBits = ONESTOPBIT;
                break;

            case e_stop_bits_one_half:
                comstate.StopBits = ONE5STOPBITS;
                break;

            case e_stop_bits_two:
                comstate.StopBits = TWOSTOPBITS;
                break;

            default:
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }

            /* Interval between bytes */
            timeouts.ReadIntervalTimeout = timeout < 0 ? 0 : 50;
            /* The amount of bytes multiplied by this is the timeout for the entire read */
            timeouts.ReadTotalTimeoutMultiplier = 0;
            /* Added to the result of the previous memeber to get the timeout for read */
            timeouts.ReadTotalTimeoutConstant = timeout < 0 ? 0 : timeout;

            /* The amount of bytes multiplied by this is the timeout for the entire write */
            timeouts.WriteTotalTimeoutMultiplier = 10;
            /* Added to the result of the previous memeber to get the timeout for write */
            timeouts.WriteTotalTimeoutConstant = 20;

            if ((!SetCommState(fd, &comstate)) ||
                (!SetCommTimeouts(fd, &timeouts)) ||
                (!SetCommMask(fd, EV_RXCHAR)))
            {
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }
        }

        return fd;
    }

    void serial_close(SerialFd fd)
    {
        if (fd != SERIAL_INVALID_FD)
        {
            CloseHandle(fd);
        }
    }

    long serial_read(SerialFd fd, unsigned char *o_buf, size_t size)
    {
        DWORD n;
        OVERLAPPED ov = { 0 };

        if ((ov.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL)) == NULL)
        {
            return -1;
        }

        if ((!ReadFile(fd, o_buf, size, NULL, &ov)) && (GetLastError() != ERROR_IO_PENDING))
        {
            goto cleanup;
        }

        switch (WaitForSingleObject(ov.hEvent, INFINITE))
        {
        case WAIT_OBJECT_0:
            if (!GetOverlappedResult(fd, &ov, &n, TRUE))
            {
                goto cleanup;
            }

            break;

        default:
            goto cleanup;
        }

        CloseHandle(ov.hEvent);

        return (int)n;

    cleanup:
        CloseHandle(ov.hEvent);
        return -1;
    }

    int serial_write(SerialFd fd, const unsigned char *buf, size_t size)
    {
        size_t written = 0;
        OVERLAPPED ov = { 0 };

        if ((ov.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL)) == NULL)
        {
            return FALSE;
        }

        for (;;)
        {
            DWORD n;
            if ((!WriteFile(fd, buf + written, size - written, NULL, &ov)) && (GetLastError() != ERROR_IO_PENDING))
            {
                goto cleanup;
            }

            switch (WaitForSingleObject(ov.hEvent, INFINITE))
            {
            case WAIT_OBJECT_0:
                if (!GetOverlappedResult(fd, &ov, &n, TRUE))
                {
                    goto cleanup;
                }

                written += (size_t)n;

                if (written == size)
                {
                    CloseHandle(ov.hEvent);
                    return TRUE;
                }

                break;

            default:
                goto cleanup;
            }
        }

    cleanup:
        CloseHandle(ov.hEvent);
        return FALSE;
    }
#else /* if !defined(_WIN32) */
#   include <fcntl.h>
#   include <unistd.h>
#   include <termios.h>
#   include <stdio.h>

    static int convert_baudrate(int baudrate)
    {
        switch (baudrate)
        {
        case 50:
            return B50;

        case 75:
            return B75;

        case 110:
            return B110;

        case 134:
            return B134;

        case 150:
            return B150;

        case 200:
            return B200;

        case 300:
            return B300;

        case 600:
            return B600;

        case 1200:
            return B1200;

        case 1800:
            return B1800;

        case 2400:
            return B2400;

        case 4800:
            return B4800;

        case 9600:
            return B9600;

        case 19200:
            return B19200;

        case 38400:
            return B38400;

        case 57600:
            return B57600;

        case 115200:
            return B115200;

        case 230400:
            return B230400;

        default:
            return B0;
        }
    }

    void serial_close(SerialFd fd)
    {
        if (fd != SERIAL_INVALID_FD)
        {
            close(fd);
        }
    }

    SerialFd serial_open(const char *port_s, const struct SerialConfig *cfg, int timeout)
    {
        SerialFd fd;
        int br = convert_baudrate(cfg->baudrate);

        if (br == B0)
        {
            fprintf(stderr, "baudrate %d is not supported\n", cfg->baudrate);
            return SERIAL_INVALID_FD;
        }

        fd = open(port_s, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd != SERIAL_INVALID_FD)
        {
            struct termios tio;

            memset(&tio, 0, sizeof(tio));

            if ((cfsetispeed(&tio, br) < 0) ||
                (cfsetospeed(&tio, br) < 0))
            {
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }

            tio.c_iflag &= ~(IXON | IXOFF);
            tio.c_cflag |= (CLOCAL | CREAD);

            switch (cfg->stop_bits)
            {
            case e_stop_bits_one:
                tio.c_cflag &= ~CSTOPB;  /* Set one stop bit. */
                break;

            case e_stop_bits_two:
                tio.c_cflag |= CSTOPB;  /* Set two stop bits. */
                break;

            /* One and a half stop bits aren't supported with termios? */
            case e_stop_bits_one_half:
            default:
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }

            if (cfg->parity == e_parity_none)
            {
                tio.c_cflag &= ~PARENB;  /* Disable parity. */
            }
            else
            {
                tio.c_cflag |= PARENB;  /* Enable parity. */

                switch (cfg->parity)
                {
                case e_parity_odd:
                    tio.c_cflag |= PARODD;
                    break;

                case e_parity_even:
                    tio.c_cflag &= ~PARODD;
                    break;

                default:
                    serial_close(fd);
                    return SERIAL_INVALID_FD;
                }
            }

            tio.c_cflag &= ~CRTSCTS; /* Disable flow control. */
            tio.c_cflag &= ~CSIZE;

            /* Set data bits */
            switch (cfg->data_bits)
            {
            case 5:
                tio.c_cflag |= CS5;
                break;

            case 6:
                tio.c_cflag |= CS6;
                break;

            case 7:
                tio.c_cflag |= CS7;
                break;

            case 8:
                tio.c_cflag |= CS8;
                break;

            default:
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }

            tio.c_cc[VMIN] = 1; /* Non-cannonical. */
            tio.c_cc[VTIME] = timeout < 0 ? 0 : (timeout + 99) / 100;

            if (tcsetattr(fd, TCSANOW, &tio) < 0)
            {
                serial_close(fd);
                return SERIAL_INVALID_FD;
            }
        }

        return fd;
    }

    long serial_read(SerialFd fd, unsigned char *o_buf, size_t size)
    {
        long count = read(fd, o_buf, size);

        if (count < 0)
        {
            return -1;
        }

        return count;
    }

    int serial_write(SerialFd fd, const unsigned char *buf, size_t size)
    {
        size_t written = 0;

        for (;;)
        {
            long w = write(fd, buf + written, size - written);

            if (w < 0)
            {
                return FALSE;
            }

            written += (size_t)w;

            if (written == size)
            {
                return TRUE;
            }
        }
    }
#endif /* !_WIN32 */

#ifdef __cplusplus
}
#endif /* __cplusplus */
