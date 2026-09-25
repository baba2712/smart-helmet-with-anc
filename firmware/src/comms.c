/*
 * comms.c - two byte-stream links carrying the same line protocol (cli.c):
 *   USB  : TinyUSB CDC-ACM on USB2_OTG_FS (PA11/PA12), clocked from HSI48+CRS
 *   BLE  : USART3 (PD8/PD9, 115200 8N1) to an optional BLE-UART module
 *          (any transparent-UART module: e.g. HM-10/JDY-23 class, or an nRF52 running a NUS bridge)
 */
#include "comms.h"
#include "hw.h"
#include "tusb.h"
#include <string.h>

/* ================================================================ USB descriptors */
/* pid.codes open-source VID. 0x0001 is their *test* PID - request a free PID for production. */
#define USB_VID  0x1209
#define USB_PID  0x0001

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE, .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC, .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, .idVendor = USB_VID, .idProduct = USB_PID, .bcdDevice = 0x0100,
    .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3, .bNumConfigurations = 1
};

enum { ITF_CDC = 0, ITF_CDC_DATA, ITF_TOTAL };
#define EP_CDC_NOTIF 0x81
#define EP_CDC_OUT   0x02
#define EP_CDC_IN    0x82
#define CONFIG_LEN   (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 4, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

static char serial_str[25];
static const char *const strings[] = { "", "Open ANC Helmet", "ANC Helmet main board", serial_str, "ANC Helmet CLI" };

const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&desc_device; }
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) { (void)index; return desc_config; }

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t buf[33];
    uint8_t n;
    if (index == 0) { buf[1] = 0x0409; n = 1; }
    else {
        if (index >= sizeof strings / sizeof strings[0]) return NULL;
        const char *s = strings[index];
        n = (uint8_t)strlen(s);
        if (n > 32) n = 32;
        for (uint8_t i = 0; i < n; i++) buf[1 + i] = (uint8_t)s[i];
    }
    buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * n + 2u));
    return buf;
}

uint32_t tusb_time_millis_api(void) { return hw_millis(); }
void tusb_time_delay_ms_api(uint32_t ms) { hw_delay_ms(ms); }

void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

static void usb_init(void)
{
    /* serial number from the 96-bit unique ID */
    const uint32_t *uid = (const uint32_t *)UID_BASE;
    static const char hex[] = "0123456789ABCDEF";
    for (int w = 0; w < 3; w++)
        for (int k = 0; k < 8; k++) serial_str[w * 8 + k] = hex[(uid[w] >> (28 - 4 * k)) & 0xFu];
    serial_str[24] = 0;

    hw_gpio(PIN_USB_DM_PORT, PIN_USB_DM_PIN, GPIO_AF, PULL_NONE, 10);
    hw_gpio(PIN_USB_DP_PORT, PIN_USB_DP_PIN, GPIO_AF, PULL_NONE, 10);
    PWR->CR3 |= PWR_CR3_USB33DEN;
    while (!(PWR->CR3 & PWR_CR3_USB33RDY)) {}
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(OTG_FS_IRQn, 5);
    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
    tusb_init(0, &dev_init);
}

/* ================================================================ BLE UART (USART3) */
#define BLE_RXQ 256u
static volatile uint8_t ble_rx[BLE_RXQ];
static volatile uint32_t ble_rx_h, ble_rx_t;

static void ble_init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
    (void)RCC->APB1LENR;
    hw_gpio(PIN_BLE_TX_PORT, PIN_BLE_TX_PIN, GPIO_AF, PULL_NONE, 7);
    hw_gpio(PIN_BLE_RX_PORT, PIN_BLE_RX_PIN, GPIO_AF, PULL_UP, 7);
    hw_gpio(PIN_BLE_EN_PORT, PIN_BLE_EN_PIN, GPIO_OUT, PULL_NONE, 0);
    hw_gpio(PIN_BLE_STATE_PORT, PIN_BLE_STATE_PIN, GPIO_IN, PULL_DOWN, 0);
    hw_pin_write(PIN_BLE_EN_PORT, PIN_BLE_EN_PIN, true);
    USART3->CR1 = 0;
    USART3->BRR = 100000000u / 115200u;          /* PCLK1 = 100 MHz */
    USART3->CR1 = USART_CR1_RXNEIE_RXFNEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    NVIC_SetPriority(USART3_IRQn, 7);
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{
    const uint32_t isr = USART3->ISR;
    if (isr & USART_ISR_RXNE_RXFNE) {
        const uint8_t c = (uint8_t)USART3->RDR;
        const uint32_t n = (ble_rx_h + 1u) % BLE_RXQ;
        if (n != ble_rx_t) { ble_rx[ble_rx_h] = c; ble_rx_h = n; }
    }
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
}

static void ble_write(const char *s, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint32_t t = 20000u;
        while (!(USART3->ISR & USART_ISR_TXE_TXFNF) && --t) {}
        USART3->TDR = (uint8_t)s[i];
    }
}

bool comms_ble_connected(void) { return hw_pin_read(PIN_BLE_STATE_PORT, PIN_BLE_STATE_PIN); }

/* ================================================================ common */
static char line[2][160];
static uint32_t line_len[2];

void comms_init(void)
{
    usb_init();
    ble_init();
}

static void usb_write(const char *s, uint32_t n)
{
    if (!tud_cdc_connected()) return;
    while (n) {
        uint32_t w = tud_cdc_write(s, n);
        s += w; n -= w;
        tud_task();
        if (!tud_cdc_connected()) return;
    }
    tud_cdc_write_flush();
}

void comms_write(comms_link_t link, const char *s, uint32_t n)
{
    if (link == LINK_USB || link == LINK_ALL) usb_write(s, n);
    if (link == LINK_BLE || link == LINK_ALL) ble_write(s, n);
}

static void feed(comms_link_t link, char c, comms_line_cb cb)
{
    char *l = line[link];
    if (c == '\r' || c == '\n') {
        if (line_len[link]) { l[line_len[link]] = 0; cb(link, l); line_len[link] = 0; }
    } else if (line_len[link] < sizeof line[0] - 1u) {
        l[line_len[link]++] = c;
    }
}

void comms_poll(comms_line_cb cb)
{
    tud_task();
    while (tud_cdc_available()) {
        char buf[64];
        const uint32_t n = tud_cdc_read(buf, sizeof buf);
        for (uint32_t i = 0; i < n; i++) feed(LINK_USB, buf[i], cb);
    }
    while (ble_rx_t != ble_rx_h) {
        const char c = (char)ble_rx[ble_rx_t];
        ble_rx_t = (ble_rx_t + 1u) % BLE_RXQ;
        feed(LINK_BLE, c, cb);
    }
}
