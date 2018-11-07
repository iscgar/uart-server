/************************************************************************************
 * UART Server -
 *  A simple program that serves a serial port over TCP to multiple clients.
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
#include <ctype.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#if (defined(__unix__) || defined(unix)) && !defined(USG)
#   include <sys/param.h>
#endif /* (defined(__unix__) || defined(unix)) && !defined(USG) */

#ifdef _WIN32
#   ifdef _MSC_VER
#       pragma comment(lib, "ws2_32.lib")
#       pragma warning(disable : 4996)
#   endif /* _MSC_VER */

#   define WIN32_LEAN_AND_MEAN
#   include <WinSock2.h>
#   include <WS2tcpip.h>
#   include <Windows.h>

typedef SOCKET Socket;
typedef HANDLE Waiter;

static WSADATA g_wsadata;
#else /* if !_WIN32 */
#   include <errno.h>
#   include <sys/types.h>
#   include <sys/socket.h>
#   include <netinet/in.h>
#   include <netdb.h>
#   include <fcntl.h>
#   include <poll.h>
#   include <arpa/inet.h>
#   include <unistd.h>

#   define closesocket     close
#   define INVALID_SOCKET  (-1)

typedef int Socket;
typedef uint32_t socklen_t;
typedef struct pollfd Waiter;
#endif /* !_WIN32 */

/**
 * Need to include this here so that on Windows the inclusion of `Windows.h`
 * won't pull in Winsock which conflicts with Winsock2. The `WIN32_LEAN_AND_MEAN`
 * definition above takes care of it.
 */
#include "serial.h"

#define DEFAULT_PORT        8278
#define SERIAL_TIMEOUT      1000

#define ARGS_SERIAL_PORT    1
#define ARGS_SERIAL_CFG     2
#define ARGS_TCP_PORT       3

#define SERIAL_CFG_BAUDRATE_IDX     0
#define SERIAL_CFG_PARITY_IDX       1
#define SERIAL_CFG_DATA_BITS_IDX    2
#define SERIAL_CFG_STOP_BITS_IDX    3
#define SERIAL_CFG_COUNT_MAX        4

#define SERIAL_CFG_DEFAULT_PARITY    e_parity_none
#define SERIAL_CFG_DEFAULT_DATA_BITS 8
#define SERIAL_CFG_DEFAULT_STOP_BITS e_stop_bits_one

#define EVENT_CLOSE_INDEX  0
#define EVENT_SERIAL_INDEX 1
#define EVENT_SERVER_INDEX 2
#define EVENT_CLIENT_INDEX 3
#define EVENT_INDEX_COUNT_MAX 4

/**
 * Simple utility macro to get the biggest of two integers.
 */
#define IMAX(a, b)  ((a) > (b) ? (a) : (b))

/**
 * Helper type definitions to make the code a bit clearer.
 */
typedef uint32_t Ipv4Addr;
typedef uint16_t InetPort;

/**
 * A structure that holds connected client information.
 */
struct ClientNode
{
    int id; /* Used for quickly identifying a client */
    Ipv4Addr addr; /* Client address for info pretty printing */
    InetPort port; /* Client port for info pretty printing */
    Socket client; /* The client socket */
    struct ClientNode *next; /* Pointer to the next client node in the clients list */
};

/**
 * A utility function that parses a string as an unsigned integer.
 *
 * @param s
 * @param len
 * @param o_result
 *
 * @return
 */
static int get_unsigned(const char *s, size_t len, unsigned *o_result)
{
    unsigned helper;

    if ((!s) || (!len) || (!o_result))
    {
        return 0;
    }

    helper = *o_result = 0;

    do
    {
        if (!isdigit(*s))
        {
            return 0;
        }

        *o_result *= 10;
        *o_result += *s++ - '0';

        /* Make sure we didn't overflow */
        if (*o_result < helper)
        {
            return FALSE;
        }

        helper = *o_result;
    } while (--len);

    return TRUE;
}

/**
 * A utility function that parses a serial configuration string.
 *
 * @param config_str
 * @param o_config
 *
 * @return
 */
static const char* parse_serial_config(const char *config_str, struct SerialConfig *o_config)
{
    int cfg_idx = 0;
    const char *start = config_str;

    if ((!config_str) || (!o_config))
    {
        return "Internal program error";
    }

    /* Initialise with default values */
    o_config->parity = SERIAL_CFG_DEFAULT_PARITY;
    o_config->data_bits = SERIAL_CFG_DEFAULT_DATA_BITS;
    o_config->stop_bits = SERIAL_CFG_DEFAULT_STOP_BITS;

    /* Parse config elements */
    do
    {
        const char *next, *end;

        /* Trim start */
        while (isspace(*start))
        {
            ++start;
        }

        next = strchr(start, ',');

        if (!next)
        {
            next = start + strlen(start);
        }

        end = next;

        /* Trim end */
        while ((end > start) && (isspace(end[-1])))
        {
            --end;
        }

        /* Parse specific config elements */
        switch (cfg_idx)
        {
        case SERIAL_CFG_BAUDRATE_IDX:
            if ((!get_unsigned(start, end - start, &o_config->baudrate)) ||
                (!o_config->baudrate))
            {
                return "baud rate must be a positive integer";
            }

            break;

        case SERIAL_CFG_PARITY_IDX:
            if (end - start == 1)
            {
                switch (*start)
                {
                case 'N':
                    o_config->parity = e_parity_none;
                    break;

                case 'O':
                    o_config->parity = e_parity_odd;
                    break;

                case 'E':
                    o_config->parity = e_parity_even;
                    break;

                default:
                    /* Increment end to reach the error condition below */
                    ++end;
                    break;
                }
            }

            if (end - start > 1)
            {
                return "Parity configuration must be either empty or one of N, O, or E";
            }

            break;

        case SERIAL_CFG_DATA_BITS_IDX:
            if (end - start == 1)
            {
                switch (*start)
                {
                case '5':
                case '6':
                case '7':
                case '8':
                    o_config->data_bits = *start - '0';
                    break;

                default:
                    /* Increment end to reach the error condition below */
                    ++end;
                    break;
                }
            }

            if (end - start > 1)
            {
                return "Data bits must be either empty or one of 5, 6, 7, or 8";
            }

            break;

        case SERIAL_CFG_STOP_BITS_IDX:
            if (end - start > 0)
            {
                if (memcmp("1.5", start, IMAX(3, end - start)) == 0)
                {
                    o_config->stop_bits = e_stop_bits_one_half;
                }
                else if (memcmp("1", start, IMAX(1, end - start)) == 0)
                {
                    o_config->stop_bits = e_stop_bits_one;
                }
                else if (memcmp("2", start, IMAX(1, end - start)) == 0)
                {
                    o_config->stop_bits = e_stop_bits_two;
                }
                else
                {
                    return "Stop bits must be either empty or one of 1, 1.5, or 2";
                }
            }

            break;

        default:
            return "Invalid serial port configuration string";
        }

        ++cfg_idx;
        start = next + 1;
    } while (start[-1]);

    return NULL;
}

/**
 * A utility function that converts a parity enumeration value to a human readable string.
 */
static const char* parity_to_string(enum SerialParity p)
{
    switch (p)
    {
    case e_parity_none:
        return "None";

    case e_parity_odd:
        return "Odd";

    case e_parity_even:
        return "Even";

    default:
        return "Unknown";
    }
}

/**
 * A utility function that converts a stop bits enumeration value to a human readable string.
 */
static const char* stop_bits_to_string(enum SerialStopBits sb)
{
    switch (sb)
    {
    case e_stop_bits_one:
        return "1";

    case e_stop_bits_one_half:
        return "1.5";

    case e_stop_bits_two:
        return "2";

    default:
        return "Unknown";
    }
}

/**
 * A utility function that converts an IPv4 address to a human readable string.
 */
static const char* ipaddr_to_string(Ipv4Addr addr)
{
    static char ip[sizeof("255.255.255.255")];

    /* Format the IP address nicely */
    if (snprintf(ip, sizeof(ip), "%d.%d.%d.%d",
            (addr >> 24) & 0xff, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff) >= (int)sizeof(ip))
    {
        return "Unknown";
    }

    return ip;
}

/**
 * A utility function that returns the current timestamp as a string.
 */
static const char* current_timestamp_as_string(void)
{
    static char current_time[sizeof("DD/mm/YYYY HH:MM:SS")];

    time_t ct = time(NULL);
    struct tm *sm = localtime(&ct);

    /* Return an empty timestamp string if something failed */
    if ((!sm) || (strftime(current_time, sizeof(current_time), "%d/%m/%Y %H:%M:%S", sm) >= sizeof(current_time)))
    {
        return "";
    }

    return current_time;
}

/**
 * A utility function that prints a printf-formatted string to STDOUT with a timestamp.
 */
static void status(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    fprintf(stdout, "[%s] ", current_timestamp_as_string());
    vfprintf(stdout, fmt, args);
    fprintf(stdout, "\n");

    va_end(args);
}

/**
 * A utility function that prints a printf-formatted string to STDERR.
 */
static void error(const char *s, ...)
{
    va_list args;
    va_start(args, s);

    vfprintf(stderr, s, args);
    fprintf(stderr, "\n");

    va_end(args);
}

/**
 * A utility function that prints usage information with an optional error message.
 */
static void help(const char *s, ...)
{
    if (s)
    {
        va_list args;
        va_start(args, s);

        vfprintf(stderr, s, args);
        fprintf(stderr, "\n");

        va_end(args);
    }

    fputs("usage: uart-server [-h] serial_port config_str [tcp_port]\n", stderr);

    /* Print only brief usage info if an error message was provided */
    if (!s)
    {
        fputs("\nUART TCP Server\n\n", stderr);
        fputs("positional arguments:\n", stderr);
        fputs("  serial_port  The serial port to serve\n", stderr);
        fputs("  config_str   The configuration of the serial port: baudrate[,parity[,data_bits[,stop_bits]]]\n", stderr);
        fputs("                 baudrate    The baud rate to be used for the communication\n", stderr);
        fprintf(stderr, "                 parity      N for none, O for odd, E for even [%c]\n", parity_to_string(SERIAL_CFG_DEFAULT_PARITY)[0]);
        fprintf(stderr, "                 data_bits   The amount of data bits to use [%d]\n", SERIAL_CFG_DEFAULT_DATA_BITS);
        fprintf(stderr, "                 stop_bits   1, 1.5, or 2 [%s]\n", stop_bits_to_string(SERIAL_CFG_DEFAULT_STOP_BITS));
        fprintf(stderr, "  tcp_port     The port to serve on [%d]\n\n", DEFAULT_PORT);
        fputs("optional arguments:\n", stderr);
        fputs("  -h, --help   show this help message and exit\n", stderr);
    }
}

/* Global state variables */
static unsigned char g_cache[1024];
static int g_last_id = -1, g_commanding_client = -1;
static struct ClientNode *g_clients = NULL;
static Waiter g_waiters[EVENT_INDEX_COUNT_MAX];

/**
 * A utility function that handles client connections.
 */
static int accept_client(Socket server)
{
    struct sockaddr_in remote;
    socklen_t remlen = sizeof(remote);
    struct ClientNode *new_client = (struct ClientNode *)malloc(sizeof(struct ClientNode));

    if (!new_client)
    {
        goto cleanup;
    }

    memset(new_client, 0, sizeof(*new_client));

    if (((new_client->client = accept(server, (struct sockaddr *)&remote, &remlen)) == INVALID_SOCKET) ||
        (remlen != sizeof(remote)) || (remote.sin_family != AF_INET))
    {
        goto cleanup;
    }

    memcpy(&new_client->addr, &remote.sin_addr, sizeof(Ipv4Addr));
    new_client->addr = ntohl(new_client->addr);
    new_client->port = ntohs(remote.sin_port);

    new_client->id = ++g_last_id;
    status("Accepted a connection from %s:%u on client ID %d",
        ipaddr_to_string(new_client->addr), new_client->port,
        new_client->id);

    if (!g_clients)
    {
        g_clients = new_client;
    }
    else
    {
        struct ClientNode *current = g_clients;

        while (current->next)
        {
            current = current->next;
        }

        current->next = new_client;
    }

    return TRUE;

cleanup:
    if (new_client)
    {
        if (new_client->client != 0)
        {
            closesocket(new_client->client);
        }

        free(new_client);
        new_client = NULL;
    }

    return FALSE;
}

/**
 * A utility function that handles client termination.
 */
static struct ClientNode* terminate_client(struct ClientNode *node, struct ClientNode **pointer)
{
    struct ClientNode *next;

    if ((!node) || (!pointer) || (*pointer != node))
    {
        return NULL;
    }

    status("Removing client %d...", node->id);

    /* Clear commanding client event wait */
    if (node->id == g_commanding_client)
    {
#if defined(_WIN32)
        WSAEventSelect(node->client, g_waiters[EVENT_CLIENT_INDEX], 0);
#else /* if !defined(_WIN32) */
        g_waiters[EVENT_CLIENT_INDEX].fd = -1;
#endif /* !defined(_WIN32) */
    }

    /* Change the pointer of `pointer` before terminating the client
     * to make sure it never points to an invalid client */
    *pointer = next = node->next;

    /* Gracefully shut down the socket */
    shutdown(node->client, 0);
    closesocket(node->client);
    node->client = INVALID_SOCKET;

    free(node);

    return next;
}

/**
 * A utility function that handles serial receive events.
 */
static void send_data_to_clients(SerialFd sport)
{
    struct ClientNode *current = g_clients, **previous = &g_clients;
    long rbytes = serial_read(sport, g_cache, sizeof(g_cache));

    if (rbytes <= 0)
    {
        return;
    }

    while (current)
    {
        size_t sent = 0;

        for (;;)
        {
            long sbytes = send(current->client, (char *)g_cache + sent, (size_t)rbytes - sent, 0);

            if (sbytes < 0)
            {
                current = terminate_client(current, previous);
                break;
            }

            sent += (size_t)sbytes;

            if (sent == (size_t)rbytes)
            {
                previous = &(*previous)->next;
                current = current->next;
                break;
            }
        }
    }
}

/**
 * A utility function that handles commanding client data.
 */
static void handle_commanding_client(SerialFd sport)
{
    long rbytes;
    static int last_commander = -1, tryout = 0;

    if (!g_clients)
    {
        return;
    }

    if (last_commander != g_commanding_client)
    {
        last_commander = g_commanding_client;
        tryout = 0;
    }

    rbytes = recv(g_clients->client, (char *)g_cache, sizeof(g_cache), 0);

    if (rbytes <= 0)
    {
        if (tryout++ == 3)
        {
            terminate_client(g_clients, &g_clients);
        }

        return;
    }

    serial_write(sport, g_cache, (size_t)rbytes);
}

#if defined(_WIN32)
static BOOL WINAPI sig_handler(DWORD sig)
{
    switch (sig)
    {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
        SetEvent(g_waiters[EVENT_CLOSE_INDEX]);
        break;

    default:
        break;
    }

    return TRUE;
}
#else /* if !defined(_WIN32) */
/* Pipe ends used to signal that we should gracefully shut down on POSIX systems */
static int g_close[2] = { -1, -1 };

static void sig_handler(int sig)
{
    switch (sig)
    {
    case SIGINT:
    case SIGABRT:
    case SIGTERM:
        {
            char b = '\0';
            write(g_close[1], &b, 1);
        }
        break;

    default:
        break;
    }
}
#endif /* !defined(_WIN32) */

int main(int argc, char const *argv[])
{
    int ret_val = EXIT_FAILURE;

    if ((argc == 2) && ((strcmp(argv[1], "-h") == 0) || (strcmp(argv[1], "--help") == 0)))
    {
        help(NULL);
    }
    else if (argc < 3)
    {
        help("Too few arguments");
    }
    else if (argc > 4)
    {
        help("Too many arguments");
    }
    else
    {
        unsigned port = DEFAULT_PORT;
        struct SerialConfig cfg;

        const char *parse_err = parse_serial_config(argv[ARGS_SERIAL_CFG], &cfg);

        if (parse_err)
        {
            help(parse_err);
        }
        /* Parse the TCP port if provided */
        else if ((argc == 4) &&
                 ((!get_unsigned(argv[ARGS_TCP_PORT], strlen(argv[ARGS_TCP_PORT]), &port) ||
                  (port <= 0) || (port > 0xffff))))
        {
            help("port must be in the range 1-65535");
        }
        else
#if defined(_WIN32)
        /* On Windows we should initialise Winsock2 */
        if (WSAStartup(0x101, &g_wsadata) != 0)
        {
            error("Failed to initialise WinSock");
        }
        else
#endif /* defined(_WIN32) */
        {
            Socket server;

            if ((server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET)
            {
                error("Failed to open a TCP socket");
            }
            else
            {
                struct sockaddr_in bind_addr;

                memset(&bind_addr, 0, sizeof(bind_addr));
                bind_addr.sin_family = AF_INET;
                bind_addr.sin_port = htons(port);

                if (bind(server, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0)
                {
                    error("Failed to bind to port %u", port);
                }
                else if (listen(server, 10) != 0)
                {
                    error("Failed to start TCP listen");
                }
                else
                {
                    SerialFd serial = serial_open(argv[ARGS_SERIAL_PORT], &cfg, SERIAL_TIMEOUT);

                    if (serial == SERIAL_INVALID_FD)
                    {
                        error("Failed to open the requested serial port");
                    }
                    else
                    {
                        size_t event_idx = 0;

#ifdef _WIN32
                        DWORD serial_events = 0;
                        OVERLAPPED ov_serial = { 0 };

                        for (event_idx = 0; event_idx < EVENT_INDEX_COUNT_MAX; ++event_idx)
                        {
                            if ((g_waiters[event_idx] = CreateEvent(NULL, FALSE, FALSE, NULL)) == NULL)
                            {
                                break;
                            }
                        }

                        ov_serial.hEvent = g_waiters[EVENT_SERIAL_INDEX];

                        if (event_idx < EVENT_INDEX_COUNT_MAX)
                        {
                            error("Failed to create wait event %d/%d (%s)",
                                (int)event_idx, EVENT_INDEX_COUNT_MAX, strerror(GetLastError()));
                        }
                        else if (WSAEventSelect(server, g_waiters[EVENT_SERVER_INDEX], FD_ACCEPT) != 0)
                        {
                            /* This is so rare that we shouldn't really bother printing an error
                             * message. The error code should be enough. */
                            error("Failed to register server for accept events: %08x", WSAGetLastError());
                        }
                        /* Break the serial abstraction and listen for serial events because there's no
                         * real way to get serial events in a portable manner that makes sense. */
                        else if ((!WaitCommEvent(serial, &serial_events, &ov_serial)) &&
                                 (GetLastError() != ERROR_IO_PENDING))
                        {
                            error("Failed to register for serial events: %s", strerror(GetLastError()));
                        }
                        else
                        {
                            /* Register for application termination requests to allow graceful shut down */
                            SetConsoleCtrlHandler(sig_handler, TRUE);
#else /* if !defined(_WIN32) */
                        if (pipe(g_close) < 0)
                        {
                            error("Failed to create shut down event: %s", strerror(errno));
                        }
                        else
                        {
                            g_waiters[EVENT_CLOSE_INDEX].fd = g_close[0];
                            g_waiters[EVENT_SERIAL_INDEX].fd = serial;
                            g_waiters[EVENT_SERVER_INDEX].fd = server;
                            g_waiters[EVENT_CLIENT_INDEX].fd = -1;

                            for (event_idx = 0; event_idx < EVENT_INDEX_COUNT_MAX; ++event_idx)
                            {
                                g_waiters[event_idx].events = POLLIN;
                            }
                        }

                        if (event_idx == EVENT_INDEX_COUNT_MAX)
                        {
                            /* Register for application termination requests to allow graceful shut down */
                            signal(SIGINT, sig_handler);
                            signal(SIGABRT, sig_handler);
                            signal(SIGTERM, sig_handler);
#endif /* !defined(_WIN32) */

                            /* Workaround for running in mintty (doesn't really matter everywhere else because
                             * we don't print that much) */
                            setbuf(stdout, NULL);

                            status("Serving %s @ %u bps (parity %s, %d data bits, and %s stop bits) on port %u",
                                argv[ARGS_SERIAL_PORT], cfg.baudrate, parity_to_string(cfg.parity), cfg.data_bits,
                                stop_bits_to_string(cfg.stop_bits), port);

                            /* Main server loop */
                            for (;;)
                            {
#ifdef _WIN32
                                DWORD result = WaitForMultipleObjects(EVENT_INDEX_COUNT_MAX, g_waiters, FALSE, INFINITE);

                                if ((result < WAIT_OBJECT_0) &&
                                    (result >= (WAIT_OBJECT_0 + EVENT_INDEX_COUNT_MAX)))
                                {
                                    error("Failed to wait on event: %d (%s)",
                                            (int)result, strerror(GetLastError()));
                                    break;
                                }

                                result -= WAIT_OBJECT_0;

                                /* Break if we were requested to shut down */
                                if (result == EVENT_CLOSE_INDEX)
                                {
                                    /* Exit with 0 exit code on graceful shut down */
                                    ret_val = EXIT_SUCCESS;
                                    break;
                                }

                                if (result == EVENT_SERIAL_INDEX)
                                {
                                    if ((serial_events & EV_RXCHAR) == 0)
                                    {
                                        error("Spurious serial event %08x", serial_events);
                                        break;
                                    }

                                    send_data_to_clients(serial);
                                    memset(&ov_serial, 0, sizeof(ov_serial));
                                    ov_serial.hEvent = g_waiters[EVENT_SERIAL_INDEX];

                                    if ((!WaitCommEvent(serial, &serial_events, &ov_serial))&&
                                        (GetLastError() != ERROR_IO_PENDING))
                                    {
                                        error("Failed to register for next serial events: %s", strerror(GetLastError()));
                                        break;
                                    }
                                }
                                else if (result == EVENT_SERVER_INDEX)
                                {
                                    accept_client(server);
                                }
                                else if (result == EVENT_CLIENT_INDEX)
                                {
                                    WSANETWORKEVENTS events;

                                    /* Try to check if a commanding client closed the connection */
                                    if ((WSAEnumNetworkEvents(g_clients->client, NULL, &events) == 0) &&
                                        (events.lNetworkEvents & FD_CLOSE))
                                    {
                                        terminate_client(g_clients, &g_clients);
                                    }
                                    else
                                    {
                                        handle_commanding_client(serial);
                                    }
                                }
#else /* if !defined(_WIN32) */
                                int result = poll(g_waiters, EVENT_INDEX_COUNT_MAX, -1);

                                if ((result < 0) && (errno != EINTR))
                                {
                                    error("Failed to wait on event: %d (%s)", errno, strerror(errno));
                                    break;
                                }

                                /* Break if we were requested to shut down */
                                if (g_waiters[EVENT_CLOSE_INDEX].revents & (POLLIN | POLLHUP | POLLERR))
                                {
                                    /* Exit with 0 exit code on graceful shut down */
                                    ret_val = EXIT_SUCCESS;
                                    break;
                                }

                                if (g_waiters[EVENT_CLIENT_INDEX].revents & (POLLHUP | POLLERR))
                                {
                                    terminate_client(g_clients, &g_clients);
                                }
                                else if (g_waiters[EVENT_CLIENT_INDEX].revents & POLLIN)
                                {
                                    handle_commanding_client(serial);
                                    g_waiters[EVENT_CLIENT_INDEX].revents = 0;
                                }

                                if (g_waiters[EVENT_SERVER_INDEX].revents & POLLIN)
                                {
                                    accept_client(server);
                                    g_waiters[EVENT_SERVER_INDEX].revents = 0;
                                }

                                if (g_waiters[EVENT_SERIAL_INDEX].revents & POLLIN)
                                {
                                    send_data_to_clients(serial);
                                    g_waiters[EVENT_SERIAL_INDEX].revents = 0;
                                }
#endif /* !defined(_WIN32) */

                                /* Check if we need to listen for a new commanding client */
                                if ((g_clients) && (g_commanding_client != g_clients->id))
                                {
#if defined(_WIN32)
                                    if (WSAEventSelect(g_clients->client,
                                            g_waiters[EVENT_CLIENT_INDEX], FD_READ | FD_CLOSE) != 0)
                                    {
                                        error("Failed to attach event to commanding client");
                                        break;
                                    }
#else /* if !defined(_WIN32) */
                                    g_waiters[EVENT_CLIENT_INDEX].fd = g_clients->client;
                                    g_waiters[EVENT_CLIENT_INDEX].revents = 0;
#endif /* !defined(_WIN32) */

                                    g_commanding_client = g_clients->id;

                                    status("Client %d @ %s:%u is now in command of the serial port",
                                        g_clients->id, ipaddr_to_string(g_clients->addr), g_clients->port);
                                }
                            }

                            /* Gracefully shut down all clients */
                            while (g_clients)
                            {
                                terminate_client(g_clients, &g_clients);
                            }
                        }

#if defined(_WIN32)
                        /* Clear server events */
                        WSAEventSelect(server, g_waiters[EVENT_SERVER_INDEX], 0);

                        /* Close all wait event handles */
                        for (event_idx = 0; event_idx < EVENT_INDEX_COUNT_MAX; ++event_idx)
                        {
                            if (g_waiters[event_idx] != NULL)
                            {
                                CloseHandle(g_waiters[event_idx]);
                            }
                        }
#else /* if !defined(_WIN32) */
                        /* Close both ends of shut down FIFO */
                        close(g_close[0]);
                        close(g_close[1]);
#endif /* !defined(_WIN32) */

                        serial_close(serial);
                    }
                }

                closesocket(server);
            }

#if defined(_WIN32)
            WSACleanup();
#endif /* defined(_WIN32) */
        }
    }

    return ret_val;
}
