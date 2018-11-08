# UART Server

A simple program that serves a serial port over TCP to multiple clients. The first connected
client (in a FIFO fashion) can also send data to the serial port.

## Dependencies

Windows, or a POSIX-compliant system with a functional `poll(2)` implementation.

## Building

Just build and link `main.c` and `serial.c` using your favourite compiler. For GCC on a Linux
system the following command will do:

```bash
$ gcc *.c -o uart-server
```

On Windows you'll need to add `-lws2_32` to the command line:

```bash
$ gcc *.c -o uart-server -lws2_32
```

MSVC will link Winsock2 automatically using an embedded `#pragma comment` directive, so the
following command line will be enough (assuming the proper environment has been set up):

```batch
> cl *.c /Feuart-server
```

## Usage

```bash
$ ./uart-server serial_port config_str [tcp_port]
```

### Arguments

* Serial Port -- The name of the serial port to use (e.g. `COM1` on Windows, or `/dev/ttyS0` on \*nix).

* Serial Port Configuration -- A string specifying how to configure the serial port. The format of
    the serial port configuration string is `baudrate[,parity[,data-bits[,stop-bits]]]`.

    * **baudrate** - The baud rate to use, e.g. 115200 (required)
    * **parity** - N for none, O for odd, E for even, M for mark, S for space (optional, default is N)
    * **data-bits** - 5, 6, 7, or 8 (optional, default is 8)
    * **stop-bits** - 1, 1.5, or 2 (optional, default is 1)

    Optional parameters can be omitted entirely if they're at the end, or left empty if you want to use
    the default and specify a parameter after them. For example, to configure a baud rate of 19200 with 7
    data bits you can simply pass `19200,,7` as the configuration string.

* TCP Port -- The TCP to accept connections on (optional, the default is 8278).

## License

This program is licensed under the MIT license. See [LICENSE](LICENSE) for details.
