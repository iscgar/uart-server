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

#include <stdio.h>
#include <string.h>
#include "serial.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined(_WIN32)
#   include <stdlib.h>
#   include <tchar.h>

    SerialFd serial_open(const char *port_s, const struct SerialConfig *cfg, int timeout)
    {
        DCB comstate;
        SerialFd fd;
        TCHAR *portname;
        size_t portname_size = 0;

        if (!port_s)
        {
            return SERIAL_INVALID_FD;
        }

        portname_size = sizeof("\\\\.\\") + strlen(port_s);
        portname = (TCHAR *)malloc(portname_size * sizeof(TCHAR));

        if (!portname)
        {
            return SERIAL_INVALID_FD;
        }

        /* Convert to TCHAR (just in case), and prefix with the \\.\ namespace in case the port name
         * does not contain a backslash */
        if (_sntprintf_s(portname, portname_size, _TRUNCATE,
                         strchr(port_s, '\\') ? _T("%hs") : _T("\\\\.\\%hs"), port_s) >= portname_size)
        {
            free(portname);
            return SERIAL_INVALID_FD;
        }

        fd = CreateFile(portname, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

        /* No need for the port name anymore */
        free(portname);
        portname = NULL;
        portname_size = 0;

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
            comstate.fOutX = comstate.fInX = FALSE;
            comstate.fTXContinueOnXoff = FALSE;
            comstate.fNull = FALSE;
            comstate.fErrorChar = FALSE;
            comstate.fAbortOnError = FALSE;
            comstate.fOutxCtsFlow = FALSE;
            comstate.fOutxDsrFlow = FALSE;
            comstate.fDsrSensitivity = FALSE;

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

            case e_parity_mark:
                comstate.Parity = MARKPARITY;
                break;

            case e_parity_space:
                comstate.Parity = SPACEPARITY;
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

            if ((!SetCommState(fd, &comstate)) || (!SetCommTimeouts(fd, &timeouts)))
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

        return (long)n;

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

#if defined(B19200)
        case 19200:
            return B19200;
#endif /* defined(B19200) */

#if defined(B38400)
        case 38400:
            return B38400;
#endif /* defined(B38400) */

#if defined(B57600)
        case 57600:
            return B57600;
#endif /* defined(B57600) */

#if defined(B115200)
        case 115200:
            return B115200;
#endif /* defined(B115200) */

#if defined(B230400)
        case 230400:
            return B230400;
#endif /* defined(B230400) */

#if defined(B460800)
        case 460800:
            return B460800;
#endif /* defined(B460800) */

#if defined(B500000)
        case 500000:
            return B500000;
#endif /* defined(B500000) */

#if defined(B576000)
        case 576000:
            return B576000;
#endif /* defined(B576000) */

#if defined(B921600)
        case 921600:
            return B921600;
#endif /* defined(B921600) */

#if defined(B1000000)
        case 1000000:
            return B1000000;
#endif /* defined(B1000000) */

#if defined(B1152000)
        case 1152000:
            return B1152000;
#endif /* defined(B1152000) */

#if defined(B1500000)
        case 1500000:
            return B1500000;
#endif /* defined(B1500000) */

#if defined(B2000000)
        case 2000000:
            return B2000000;
#endif /* defined(B2000000) */

#if defined(B2500000)
        case 2500000:
            return B2500000;
#endif /* defined(B2500000) */

#if defined(B3000000)
        case 3000000:
            return B3000000;
#endif /* defined(B3000000) */

#if defined(B3500000)
        case 3500000:
            return B3500000;
#endif /* defined(B3500000) */

#if defined(B4000000)
        case 4000000:
            return B4000000;
#endif /* defined(B4000000) */

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

        if (!port_s)
        {
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

                /* Whatever those are, termios can't handle them */
                case e_parity_mark:
                case e_parity_space:
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
