/*
 * usb.h
 *
 *  Created on: 29.04.2018
 *      Author: DF4IAH
 */

#ifndef USB_H_
#define USB_H_

#include <stddef.h>
#include <sys/_stdint.h>


/* Called from the application */
void usbToHost(const uint8_t* buf, uint32_t len);
void usbToHostWait(const uint8_t* buf, uint32_t len);

void usbLogLen(const char* str, int len);
void usbLog(const char* str);


/* Called from the USB CDC backend */
void usbFromHostFromIRQ(const uint8_t* buf, uint32_t len);

void usbUsbToHostTaskInit(void);
void usbUsbToHostTaskLoop(void);

void usbUsbFromHostTaskInit(void);
void usbUsbFromHostTaskLoop(void);

#endif /* USB_H_ */
