/* comms.h - USB CDC + BLE UART line transport */
#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { LINK_USB = 0, LINK_BLE = 1, LINK_ALL = 2 } comms_link_t;
typedef void (*comms_line_cb)(comms_link_t link, char *line);

void comms_init(void);
void comms_poll(comms_line_cb cb);
void comms_write(comms_link_t link, const char *s, uint32_t n);
bool comms_ble_connected(void);

#endif
