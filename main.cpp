/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 *
 * simple udp-lirc daemon for yaUsbIr
 *
 * Compile:
 * 	g++ -o yausbir_lirc main.cpp -lusb-1.0
 *
 * start lircd:
 *  lircd --driver=udp [config-file]
 *  yausbir_lirc [-f]
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <netdb.h>
#include <errno.h>
#include "libusb-1.0/libusb.h"

#define LIRCD_UDP_PORT 8765

#define CMD_NONE       0x00 // Kommando = 0 -> Keine Kommando, nur Dummydaten
#define CMD_IRDATA     0x01 // Kommando = 1 -> IR-Daten senden/ empfangen
#define CMD_COMDATA    0x02 // Kommando = 2 -> serielle Daten senden/ empfangen
#define CMD_SETCOMBAUD 0x03 // Kommando = 3 -> Baudrate CommPort setzen
#define CMD_GETCOMBAUD 0x04 // Kommando = 4 -> Baudrate CommPort abfragen
#define CMD_GETIOS     0x05 // Kommando = 5 -> alle Input/Output Port lesen
#define CMD_GETIO      0x06 // Kommando = 6 -> einen Input/Output Port lesen
#define CMD_SETIOS     0x07 // Kommando = 7 -> alle Outputports setzen/löschen
#define CMD_SETIO      0x08 // Kommando = 8 -> einen Outputport setzen/löschen

#define IRRX_NODATA 0x0000

extern char *optarg;
extern int optind, opterr, optopt;
char *prog;
int daemonized;
int debug = 0;

void logprintf(int prio, const char *format_str, ...);

//*** raw hid interface ******************************************************

typedef struct {
    libusb_device_handle *handle;
    int kernelattached;
    int iface;
    int nb_ifaces;
    uint8_t ep_in;
    uint8_t ep_out;
} raw_hid;

//****************************************************************************

static int usb_vendor = 0x10c4;
static int usb_product = 0x876c;

//  rawhidrecv - receive a packet
//    Inputs:
//  hid = device to receive from
//  buf = buffer to receive packet
//  len = buffer's size
//  timeout = time to wait, in milliseconds
//    Output:
//  number of bytes received, or -1 on error
//
int rawhidrecv(raw_hid *hid, void *buf, int len, int timeout)
{
    int r;
    int lenret;
    if ((hid == NULL) || (hid->handle == NULL)) return -1;

    r = libusb_interrupt_transfer(hid->handle, hid->ep_in, (unsigned char*)buf, len, &lenret, timeout);
    if (r >= 0) return lenret;
    if ((r == LIBUSB_ERROR_TIMEOUT) || (r == -110)) return 0;// timeout
    logprintf(LOG_ERR,"yaUsbIr: interrupt read error %d, %s", r, libusb_error_name(r));
    return -1;
}

//  rawhidsend - send a packet
//    Inputs:
//  hid = device to transmit to
//  buf = buffer containing packet to send
//  len = number of bytes to transmit
//  timeout = time to wait, in milliseconds
//    Output:
//  number of bytes sent, or -1 on error
//
int rawhidsend(raw_hid *hid, void *buf, int len, int timeout)
{
    int r;
    int lenret;
    if ((hid == NULL) || (hid->handle == NULL)) return -1;

    r = libusb_interrupt_transfer(hid->handle, hid->ep_out, (unsigned char*)buf, len, &lenret, timeout);
    if (r < 0) {
        logprintf(LOG_ERR,"yaUsbIr: interrupt write error %d, %s", r, libusb_error_name(r));
        return -1;
    }

    return lenret;
}

//  rawhidClose - close a device
//
//    Inputs:
//  hid = device to close
//    Output
//  (nothing)
//
void rawhidclose(raw_hid *hid)
{
    int iface;

//    logprintf(LOG_NOTICE,"yaUsbIr: close device A, %d, %d",hid,hid->handle);

    if ((hid == NULL) || (hid->handle == NULL)) return;

    logprintf(LOG_NOTICE,"yaUsbIr: close device");

    for (iface = 0; iface<hid->nb_ifaces; iface++)
        libusb_release_interface(hid->handle, iface);

    //if we detached kernel driver, reattach.
    if (hid->kernelattached == 1)
        libusb_attach_kernel_driver( hid->handle, hid->iface);

    libusb_close(hid->handle);
    hid->handle = NULL;

    libusb_exit(NULL);
}

//  rawhidOpen - open first device
//
//    Inputs:
//  hid = device struct
//  vid = Vendor ID
//  pid = Product ID
//    Output:
//  first openend device
//
raw_hid *rawhidopen(raw_hid *hid, int vid, int pid, int info)
{
    libusb_device *dev;
    int r;
    char stringManufacturer[128] = {0};
    char stringProduct[128] = {0};
    int iface;
    struct libusb_device_descriptor dev_desc;
    struct libusb_config_descriptor *conf_desc;

    if (hid == NULL) return NULL;

    rawhidclose(hid);

    logprintf(LOG_INFO, "yaUsbIr: Initializing yaUsbIr (libusb-1.0 API)");
    if (libusb_init(NULL) < 0) {
        logprintf(LOG_ERR, "yaUsbIr: unable to init libusb");
        return 0;
    }

#ifdef LIBUSBX_API_VERSION
    logprintf(LOG_NOTICE, "yaUsbIr: libusb version %d.%d.%d.%d %s %s", libusb_get_version()->major, libusb_get_version()->minor,
        libusb_get_version()->micro, libusb_get_version()->nano, libusb_get_version()->rc, libusb_get_version()->describe);
#endif

    hid->handle = libusb_open_device_with_vid_pid(NULL, vid, pid);

    if (hid->handle == NULL) {
        logprintf(LOG_ERR,"yaUsbIr: unable to open device");
        return NULL;
    }

    dev = libusb_get_device(hid->handle);

    r = libusb_get_device_descriptor(dev, &dev_desc);
    if (r < 0) {
        logprintf(LOG_ERR,"yaUsbIr: failed to get device descriptor");
        rawhidclose(hid);
        return NULL;
    }

    r = libusb_get_config_descriptor(dev, 0, &conf_desc);
    if (r < 0) {
        logprintf(LOG_ERR,"yaUsbIr: failed to get config descriptor");
        rawhidclose(hid);
        return NULL;
    }

    hid->kernelattached = 0;
    hid->iface = 0;
    hid->ep_in = 0x81;
    hid->ep_out = 0x01;
    hid->nb_ifaces = conf_desc->bNumInterfaces;
    libusb_free_config_descriptor(conf_desc);

    if (libusb_kernel_driver_active(hid->handle, hid->iface) == 1) {
        logprintf(LOG_NOTICE,"yaUsbIr: device busy...detaching from kernel...");
        r = libusb_detach_kernel_driver(hid->handle, hid->iface);
        if (r < LIBUSB_SUCCESS) {
            logprintf(LOG_ERR,"yaUsbIr: unable to detach from kernel, error %d, %s",r , libusb_error_name(r));
            return NULL;
        }
        hid->kernelattached = 1;
    }

    for (iface = 0; iface < hid->nb_ifaces; iface++) {
        r = libusb_claim_interface(hid->handle, iface);
        if (r < LIBUSB_SUCCESS) {
            logprintf(LOG_ERR,"yaUsbIr: unable claim interface %d, error %d, %s", iface, r, libusb_error_name(r));
            rawhidclose(hid);
            return NULL;
        }
    }

    if (info) {
        if (dev_desc.iManufacturer != 0)
            libusb_get_string_descriptor_ascii(hid->handle, dev_desc.iManufacturer, (unsigned char*)stringManufacturer, sizeof(stringManufacturer)-1);
        if (dev_desc.iProduct != 0)
            libusb_get_string_descriptor_ascii(hid->handle, dev_desc.iProduct, (unsigned char*)stringProduct, sizeof(stringProduct)-1);
        logprintf(LOG_NOTICE,  "yaUsbIr: idVendor=%04X, idProduct=%04X\n"
                "                Manufacturer: %s\n"
                "                Product: %s\n"
                "                hid interface (generic)\n"
                "                SerialNumber: %04d",
            dev_desc.idVendor, dev_desc.idProduct, stringManufacturer, stringProduct, dev_desc.iSerialNumber);
    }

    return hid;
}

//*****************************************************************************

void usage(void)
{
    fprintf(stderr,
        "usage: %s [options] [-h <lircd_host>] [-p <lircd_port>]\n"
        "   lircd_host defaults to 127.0.0.1\n"
        "   lircd_port defaults to 8765.\n"
        "   options:\n"
        "   -t      to make a TCP connection rather than UDP\n"
        "   -D      display pulse/pause time for debugging\n"
        "   -f      to keep program in foreground\n"
        "   -w S    to poll the creation of hiddev at S second intervals (default 3 sec)\n"
        , prog);
    exit(1);
}

void logprintf(int prio, const char *format_str, ...)
{
    va_list ap;
    va_start(ap, format_str);

    if (daemonized)
        vsyslog(prio, format_str, ap);
    else {
        vfprintf(stderr, format_str, ap);
        fprintf(stderr, "\n");
    }

    va_end(ap);
}

int socket_init(int tcp, char *host, int port)
{
    struct hostent *hent;
    int s;
    sockaddr_in si_other[1];

    hent = gethostbyname(host);
    if (!hent) {
        fprintf(stderr, "%s: gethostbyname: ", prog);
        herror(host);
        exit(1);
    }

    if (tcp) {
        if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            logprintf(LOG_ERR, "socket");
            exit(1);
        }
    } else {
        if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
            logprintf(LOG_ERR, "socket");
            exit(1);
        }
    }

    memset((char *) si_other, 0 ,sizeof(struct sockaddr_in));
    si_other->sin_family = AF_INET;
    si_other->sin_port = htons(port);
    si_other->sin_addr = *((struct in_addr *)hent->h_addr);

    if (connect(s, (struct sockaddr *)si_other, sizeof(struct sockaddr_in)) < 0)
        return -1;

    return s;
}

void hiddev_restore(void)
{
    //
}

void sighandler(int sig)
{
    hiddev_restore();
    exit(1);
    logprintf(LOG_ERR, "signal");
}

void data_loop(raw_hid *dev, int tcp, char *host, int port)
{
    uint8_t ya_usbir_rxbuf[64];
    uint8_t bufUdp[64];
    int bufUdpLen;
    int num;
    unsigned int n;
    static int to = -1;
    int64_t rcvdata = 0;
    char text[256];

    while (dev->handle!=NULL) {
        num = rawhidrecv(dev, ya_usbir_rxbuf, sizeof(ya_usbir_rxbuf), 25);
        if (num < 0) {
            logprintf(LOG_ERR,"yaUsbIr: error reading, device went offline");
            break;
        }

        if ((num > 2) && (ya_usbir_rxbuf[0] == CMD_IRDATA)) {
            if (to < 0)
                to = socket_init(tcp, host, port);
            if (to >= 0) {
                // lirc-udp daemon received UDP packets consist of some number
                // of LE 16-bit integers. The high bit signifies whether the
                // received signal was high or low; the low 15 bits specify the
                // number of 1/16384-second intervals the signal lasted.
                bufUdpLen = 0;
                for (n=2; n < sizeof(ya_usbir_rxbuf) ;n += 2) {
                    rcvdata = ((int)ya_usbir_rxbuf[n]&0x7F)<<8;// MSB
                    rcvdata |= ya_usbir_rxbuf[n+1];// LSB
                    rcvdata *= ya_usbir_rxbuf[1];// us step
                    if (rcvdata==IRRX_NODATA) break;// end of data

                    if (debug)
                        sprintf(text,"%s %ldus",ya_usbir_rxbuf[n] & 0x80?"pulse":"space",(long)rcvdata);

                    // Convert microseconds to 1/16384-seconds
                    rcvdata *=  1000L;
                    rcvdata /= 61035L;

                    if (debug)
                        logprintf(LOG_DEBUG, "%s (upd %ld)",text,(long)rcvdata);

                    if (rcvdata > 0x7FFF) rcvdata = 0x7FFF;
                    if ((ya_usbir_rxbuf[n] & 0x80)==0)
                        rcvdata |= 0x8000;// PULSE_BIT (invers!)

                    bufUdp[bufUdpLen++] = rcvdata & 0xFF;// LSB
                    bufUdp[bufUdpLen++] = (rcvdata & 0xFF00)>>8;// MSB
                }

                if (bufUdpLen>0) {
                    if (write(to,bufUdp,bufUdpLen) < 0) {
                        if (errno != ECONNREFUSED) {
                            logprintf(LOG_ERR, "write");
                            exit(1);
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    int c;
    char *p;
    char *host = 0;
    int port = LIRCD_UDP_PORT;
    int foreground = 0;
    int wait_term = 3;
    int tcp = 0;
    int logged = 0;
    int first = 1;
    raw_hid dev = {NULL, 0, 0, 0, 0, 0};

    host = (char *)"127.0.0.1\0";

    prog = argv[0];
    p = strrchr(argv[0], '/');
    if (p) prog = p + 1;

    while ((c = getopt(argc, argv, "Dtfw:h:p:")) != EOF) {
        switch (c) {
        case 'D':
            debug = 1;
            break;
        case 't':
            tcp = 1;
            break;
        case 'f':
            foreground = 1;
            break;
        case 'w':
            wait_term = atoi(optarg);
            if (wait_term == 0)
                usage();
            break;
        case 'h':
            host = optarg;
            break;
        case 'p':
            port = atoi(optarg);
            break;
        default:
            usage();
            break;
        }
    }

    if ((debug == 0 && !host) || optind != argc) {
        usage();
    }

    while(1) {
        rawhidopen(&dev, usb_vendor, usb_product, first);

        if ((wait_term==0)&&(dev.handle==NULL)) {
          logprintf(LOG_ERR, "can't open yaUsbIr device %04X:%04X", usb_vendor, usb_product);
            exit(1);
        }

        if ((first)&&(dev.handle!=NULL)) {
            first = 0;
            atexit(hiddev_restore);
            signal(SIGTERM, sighandler);
            signal(SIGHUP, sighandler);
        }

        if (!daemonized && !foreground && !debug) {
            if (daemon(0, 0) < 0) {
                logprintf(LOG_ERR, "daemon");
                exit(1);
            }
            daemonized = 1;
        }

        if (dev.handle!=NULL)
            logprintf(LOG_NOTICE, "connect to yaUsbIr");

        data_loop(&dev, tcp, host, port);
        rawhidclose(&dev);

        // we'll only ever return from data_loop() if our read()
        // returns 0, which usually means our HID-based IR-Transceiver has
        // gone away.  loop if we were told to wait (-w) for it.
        if (!wait_term) {
            logprintf(LOG_ERR, "end-of-dataloop");
            exit(1);
        }

        if (!logged) {
            logprintf(LOG_NOTICE, "waiting for hiddev creation");
            logged = 1;
        }

        sleep(wait_term);
    }

    return 0;
}
