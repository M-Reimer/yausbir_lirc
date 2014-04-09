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

#define CMD_NONE               0x00 // Kommando = 0 -> Keine Kommando, nur Dummydaten
#define CMD_IRDATA             0x01 // Kommando = 1 -> IR-Daten senden/ empfangen
#define CMD_COMDATA            0x02 // Kommando = 2 -> serielle Daten vom CommPort senden/ empfangen
#define CMD_SETCOMBAUD         0x03 // Kommando = 3 -> Baudrate CommPort setzen
#define CMD_GETCOMBAUD         0x04 // Kommando = 4 -> Baudrate CommPort abfragen
#define CMD_GETIOS             0x05 // Kommando = 5 -> alle Input/Output Port lesen
#define CMD_GETIO              0x06 // Kommando = 6 -> einen Input/Output Port lesen
#define CMD_SETIOS             0x07 // Kommando = 7 -> alle Outputports setzen/löschen
#define CMD_SETIO              0x08 // Kommando = 8 -> einen Outputport setzen/löschen

#define IRRX_NODATA 0x0000

extern char *optarg;
extern int optind, opterr, optopt;
char *prog;
int daemonized;

// debugging can either replace, or augment, normal operation
#define DEBUG_HEX 1
#define DEBUG_PULSE 1
int debug;

void report(char *s);
void error(char *s);

//*** raw hid interface **************************************************************************

typedef struct {
    usb_dev_handle *usb;
    int iface;
    int ep_in;
    int ep_out;
} raw_hid;

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
    //printf("Interrupt read (%d: %m) (%s).\n", r, usb_strerror());
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
    char text2[100];

    usb_init();
    usb_find_busses();
    usb_find_devices();
    for (bus = usb_get_busses(); bus; bus = bus->next) {
        for (dev = bus->devices; dev; dev = dev->next) {
            if (dev->descriptor.idVendor != vid) continue;
            if (dev->descriptor.idProduct != pid) continue;
            if (!dev->config) continue;
            if (dev->config->bNumInterfaces < 1) continue;
            if (info) {
                sprintf(text,"device: vid=%04X, pic=%04X, with %d interface",
                    dev->descriptor.idVendor,dev->descriptor.idProduct,dev->config->bNumInterfaces);
                report(text);
            }
            iface = dev->config->interface;
            usb = NULL;
            for (ifacenum=0; ifacenum<dev->config->bNumInterfaces && iface; ifacenum++, iface++) {
                desc = iface->altsetting;
                if (!desc) continue;
                //printf("  type %d, %d, %d\n", desc->bInterfaceClass, desc->bInterfaceSubClass, desc->bInterfaceProtocol);
                if (desc->bInterfaceClass != 3) continue;
                if (desc->bInterfaceSubClass != 0) continue;
                if (desc->bInterfaceProtocol != 0) continue;
                ep = desc->endpoint;
                ep_in = ep_out = 0;
                for (n = 0; n < desc->bNumEndpoints; n++, ep++) {
                    if (ep->bEndpointAddress & 0x80) {
                        if (!ep_in) ep_in = ep->bEndpointAddress & 0x7F;
                        //printf("    IN endpoint %d\n", ep_in);
                    } else {
                        if (!ep_out) ep_out = ep->bEndpointAddress;
                        //printf("    OUT endpoint %d\n", ep_out);
                    }
                }
                if (!ep_in) continue;
                if (!usb) {
                    usb = usb_open(dev);
                    if (!usb) {
                        report("unable to open device");
                        break;
                    }
                }
                
                usb_get_string_simple(usb, 1,(char *)buf, sizeof(buf));
                usb_get_string_simple(usb, 2,text2, sizeof(text2));
                if (info) {
                    sprintf(text,"Manufacturer: %s\nProduct: %s\nhid interface (generic)", buf,text2);
                    report(text);
                }
                
                if (usb_get_driver_np(usb, ifacenum, (char *)buf, sizeof(buf)) >= 0) {
                    if (info) {
                        sprintf(text,"in use by driver \"%s\"", buf);
                        report(text);
                    }
                    if (usb_detach_kernel_driver_np(usb, ifacenum) < 0) {
                        error("unable to detach from kernel");
                        continue;
                    }
                }
                if (usb_claim_interface(usb, ifacenum) < 0) {
                    printf("unable claim interface %d", ifacenum);
                    continue;
                }
                len = usb_control_msg(usb, 0x81, 6, 0x2200, ifacenum, (char *)buf, sizeof(buf), 250);
                //printf("descriptor, len=%d\n", len);
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

//************************************************************************************************

void usage(void)
{
    fprintf(stderr,
        "usage: %s [options] [-h <lircd_host>] [-p <lircd_port>]\n"
        "   lircd_host defaults to 127.0.0.1\n"
        "   lircd_port defaults to 8765.\n"
        "   options:\n"
        "   -t      to make a TCP connection rather than UDP\n"
        "   -d      for debugging (without socket connection)\n"
        "   -D      display pulse/pause time for debugging\n"
        "   -f      to keep program in foreground\n"        
        "   -w S    to poll the creation of hiddev at S second intervals (default 3 sec)\n"
        , prog);
    exit(1);
}

void report(char *s)
{
    if (daemonized)
        syslog(LOG_NOTICE, s);
    else
        fprintf(stderr, "%s\n", s);
}

void error(char *s)
{
    if (daemonized) {
        syslog(LOG_ERR, "%s: %m", s);
        syslog(LOG_ERR, "exiting");
    } else {
        perror(s);
    }
    exit(1);
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
        if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0)
            error("socket");
    } else {
        if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
            error("socket");
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
    error("signal");
}

void data_loop(raw_hid *dev, int tcp, char *host, int port)
{
    uint8_t buf[64];
    uint8_t bufUdp[64];
    int bufUdpLen;
    int num , i;
    static int to = -1;
    int64_t irsig;
    char text[256];
    
    while (dev!=NULL) {
        num = rawhidrecv(dev, buf, sizeof(buf), 220);
        if (num < 0) {
            report("error reading, device went offline");
            break;
        }
        if (num > 0) {
            if ((debug==0)||(debug==DEBUG_PULSE)) {// sending to hostent
                if (buf[0]==CMD_IRDATA) {
                    if (to < 0)
                        to = socket_init(tcp, host, port);
                    if (to >= 0) {
                        // lirc-udp daemon received UDP packets consist of some number of LE 16-bit integers.
                        // The high bit signifies whether the received signal was high or low;
                        // the low 15 bits specify the number of 1/16384-second intervals the signal lasted.

                        bufUdpLen = 0;
                        for (i=2; i<num; i++) {
                            irsig = ((int)buf[i++]&0x7F)<<8;// MSB
                            irsig |= buf[i];// LSB
                            if (irsig==IRRX_NODATA) break;// end of data
                            irsig *= buf[1];// us step
                            
                            if (debug==DEBUG_PULSE)
                                sprintf(text,"%s %ldus",buf[i-1] & 0x80?"pulse":"space",(long)irsig);
                            
                            // Convert microseconds to 1/16384-seconds
                            irsig *=  1000L;
                            irsig /= 61035L;

                            if (debug==DEBUG_PULSE) {
                                sprintf(text+strlen(text)," (upd %ld)",(long)irsig);
                                report(text);
                            }
                           
                            if (irsig > 0x7FFF) irsig = 0x7FFF;
                            if ((buf[i-1] & 0x80)==0) irsig |= 0x8000;// PULSE_BIT (invers!)
                            bufUdp[bufUdpLen++] = irsig & 0xFF;// LSB
                            bufUdp[bufUdpLen++] = (irsig & 0xFF00)>>8;// MSB
                        }

                        if (bufUdpLen>0) {
                            if (write(to,bufUdp,bufUdpLen) < 0) {
                                if (errno != ECONNREFUSED)
                                    error("write");
                            }
                        }
                    }
                }
            } else {
                sprintf(text,"recv %d bytes:", num);
                report(text);
                memset(text,' ',sizeof(text));
                text[sizeof(text)-1] = 0;
                for (int i=0; i<num; i++) 
                    sprintf(text+(i*3),"%02X ", buf[i]);
                text[3*32-1] = '\n';
                report(text);
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

    host = "127.0.0.1\0";
    debug = 0;
    
    prog = argv[0];
    p = strrchr(argv[0], '/');
    if (p) prog = p + 1;

    while ((c = getopt(argc, argv, "dDtfw:h:p:")) != EOF) {
        switch (c) {
        case 'd':
            debug = DEBUG_HEX;
            break;
        case 'D':
            debug = DEBUG_PULSE;
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
            error("can't open yaUsbIr device 0x10c4:0x876c");
            break;
        }

        if ((first)&&(dev!=NULL)) {
            first = 0;
            atexit(hiddev_restore);
            signal(SIGTERM, sighandler);
            signal(SIGHUP, sighandler);
        }

        if (!daemonized && !foreground && !debug) {
            if (daemon(0, 0) < 0)
                error("daemon");
            daemonized = 1;
        }

        if (dev!=NULL)
            report("connect to yaUsbIr");
        
        data_loop(dev, tcp, host, port);
        rawhidclose(&dev);

        // we'll only ever return from data_loop() if our read()
        // returns 0, which usually means our HID-based IR-Transceiver has
        // gone away.  loop if we were told to wait (-w) for it.
        if (!wait_term)
            error("end-of-dataloop");
       
        if (!logged) {
            report("waiting for hiddev creation");
            logged = 1;
        }
        
        sleep(wait_term);
    }

    return 0;
}
