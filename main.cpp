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
 * 	gcc -o yausbir_lirc main.cpp -lusb
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
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <unistd.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <usb.h>

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
    usb_dev_handle *usb;
    int iface;
    int ep_in;
    int ep_out;
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
    if (hid==NULL) return -1;
    r = usb_interrupt_read(hid->usb, hid->ep_in, (char*)buf, len, timeout);
    if (r >= 0) return r;
    if (r == -110) return 0;// timeout
    //logprintf(LOG_NOTICE,"yaUsbIr: Interrupt read (%d: %m) (%s).\n", r, usb_strerror());
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
    if (hid==NULL) return -1;
    if (hid->ep_out) {
        return usb_interrupt_write(hid->usb, hid->ep_out, (char*)buf, len, timeout);
    } else {
        return usb_control_msg(hid->usb, 0x21, 9, 0, hid->iface, (char*)buf, len, timeout);
    }
}

//  rawhidOpen - open first device
//
//    Inputs:
//  vid = Vendor ID
//  pid = Product ID
//    Output:
//  first openend device
//
raw_hid *rawhidopen(int vid, int pid, int info)
{
    struct usb_bus *bus;
    struct usb_device *dev;
    struct usb_interface *iface;
    struct usb_interface_descriptor *desc;
    struct usb_endpoint_descriptor *ep;
    usb_dev_handle *usb;
    uint8_t buf[1024];
    int ifacenum, n, len, ep_in, ep_out;
    raw_hid *hid;
    char text[512];

    usb_init();
    usb_find_busses();
    usb_find_devices();
    for (bus = usb_get_busses(); bus; bus = bus->next) {
        for (dev = bus->devices; dev; dev = dev->next) {
            if (dev->descriptor.idVendor != vid) continue;
            if (dev->descriptor.idProduct != pid) continue;
            if (!dev->config) continue;
            if (dev->config->bNumInterfaces < 1) continue;
            if (info)
                logprintf(LOG_NOTICE,"yaUsbIr: device: vid=%04X, pic=%04X, with %d interface",
                    dev->descriptor.idVendor,dev->descriptor.idProduct,dev->config->bNumInterfaces);

            iface = dev->config->interface;
            usb = NULL;
            for (ifacenum=0; ifacenum<dev->config->bNumInterfaces && iface; ifacenum++, iface++) {
                desc = iface->altsetting;
                if (!desc) continue;
                //logprintf(LOG_NOTICE,"yaUsbIr:   type %d, %d, %d\n", desc->bInterfaceClass, desc->bInterfaceSubClass, desc->bInterfaceProtocol);
                if (desc->bInterfaceClass != 3) continue;
                if (desc->bInterfaceSubClass != 0) continue;
                if (desc->bInterfaceProtocol != 0) continue;
                ep = desc->endpoint;
                ep_in = ep_out = 0;
                for (n = 0; n < desc->bNumEndpoints; n++, ep++) {
                    if (ep->bEndpointAddress & 0x80) {
                        if (!ep_in) ep_in = ep->bEndpointAddress & 0x7F;
                        //logprintf(LOG_NOTICE,"yaUsbIr:     IN endpoint %d\n", ep_in);
                    } else {
                        if (!ep_out) ep_out = ep->bEndpointAddress;
                        //logprintf(LOG_NOTICE,"yaUsbIr:     OUT endpoint %d\n", ep_out);
                    }
                }
                if (!ep_in) continue;
                if (!usb) {
                    usb = usb_open(dev);
                    if (!usb) {
                        logprintf(LOG_ERR,"yaUsbIr: unable to open device");
                        break;
                    }
                }

                usb_get_string_simple(usb, 1,(char *)buf, sizeof(buf));
                usb_get_string_simple(usb, 2,text, sizeof(text));
                if (info)
                    logprintf(LOG_NOTICE,"         Manufacturer: %s\n                Product: %s\n                hid interface (generic)", buf,text);

                if (usb_get_driver_np(usb, ifacenum, (char *)buf, sizeof(buf)) >= 0) {
                    if (info)
                        logprintf(LOG_NOTICE,"yaUsbIr: in use by driver \"%s\"", buf);

                    if (usb_detach_kernel_driver_np(usb, ifacenum) < 0) {
                        logprintf(LOG_ERR,"yaUsbIr: unable to detach from kernel");
                        continue;
                    }
                }
                if (usb_claim_interface(usb, ifacenum) < 0) {
                    logprintf(LOG_ERR,"yaUsbIr: unable claim interface %d", ifacenum);
                    continue;
                }
                len = usb_control_msg(usb, 0x81, 6, 0x2200, ifacenum, (char *)buf, sizeof(buf), 250);
                //logprintf(LOG_NOTICE,"descriptor, len=%d\n", len);
                if (len < 2) {
                    usb_release_interface(usb, ifacenum);
                    continue;
                }
                hid = (raw_hid *)malloc(sizeof(raw_hid));
                if (!hid) {
                    usb_release_interface(usb, ifacenum);
                    continue;
                }
                hid->usb = usb;
                hid->iface = ifacenum;
                hid->ep_in = ep_in;
                hid->ep_out = ep_out;
                return hid;
            }
            if (usb) usb_close(usb);
        }
    }
    return NULL;
}

//  rawhidClose - close a device
//
//    Inputs:
//  hid = device to close
//    Output
//  (nothing)
//
void rawhidclose(raw_hid **hid)
{
    if ((hid==NULL)||(*hid==NULL)) return;
    usb_release_interface((*hid)->usb, (*hid)->iface);
    usb_close((*hid)->usb);
    free(*hid);
    *hid = NULL;
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
    int num, n;
    static int to = -1;
    int64_t rcvdata = 0;
    char text[256];

    while (dev!=NULL) {
        num = rawhidrecv(dev, ya_usbir_rxbuf, sizeof(ya_usbir_rxbuf), 220);
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
        raw_hid *dev = rawhidopen(0x10c4, 0x876c, first);

        if ((wait_term==0)&&(dev==NULL)) {
            logprintf(LOG_ERR, "can't open yaUsbIr device 0x10c4:0x876c");
            exit(1);
        }

        if ((first)&&(dev!=NULL)) {
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

        if (dev!=NULL)
            logprintf(LOG_NOTICE, "connect to yaUsbIr");

        data_loop(dev, tcp, host, port);
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
