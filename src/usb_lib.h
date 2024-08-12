//24.07.2022
#ifndef __USB_LIB_H__
#define __USB_LIB_H__

/*************************************************************************************
 *************************************************************************************
 ****   SETTINGS  ********************************************************************
 *************************************************************************************
 *************************************************************************************
 */

#define USB_EP0_BUFSZ   8
//#define USBLIB_SOF_ENABLE


/*************************************************************************************
 *************************************************************************************
 ****   AVAIBLE FUNCIONS (from core)  ************************************************
 *************************************************************************************
 *************************************************************************************
 */
#include <cstdint>
#include <cstring>
#define USB_ENDP_CTRL 0x00
#define USB_ENDP_ISO  0x01
#define USB_ENDP_BULK 0x02
#define USB_ENDP_INTR 0x03

//usb_lib.c
#define USB_ALIGN __attribute__ ((aligned (2)))

#include <functional>

void usb_ep_init(uint8_t epnum, uint8_t ep_type, uint16_t size, std::function<void(uint8_t)> func);
void usb_ep_init_double(uint8_t epnum, uint8_t ep_type, uint16_t size, std::function<void(uint8_t)> func);
static void usb_ep_write(uint8_t epnum, const uint16_t *buf, uint16_t size);
static void usb_ep_write_double(uint8_t epnum, const uint16_t *buf, uint16_t size);
static int usb_ep_read(uint8_t epnum, uint16_t *buf);
static int usb_ep_read_double(uint8_t epnum, uint16_t *buf);
#define usb_ep_ready(epnum)

class USBDevice {
    static USBDevice* exemplar;

public:
    struct config_pack_t {
        uint8_t bmRequestType;
        uint8_t bRequest;
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;

        config_pack_t () :
            bmRequestType(0),
            bRequest(0),
            wValue(0),
            wIndex(0),
            wLength(0) {}
    };

    USBDevice();

    // удалить конструкторы

    void start ();
    //void stop  ();

    //init (optional)
    //Funcation callen on usb connect
    virtual void init       ();

    //disconnect (optional)
    //Function called on usb disconnect
    virtual void disconnect ();

    //poll (optional)
    //Function called periodically from main()
    virtual void poll       ();

    //sof (optional, enabled by USBLIB_SOF_ENABLE macro)
    //Function called periodically by USB SOF event (every 1 ms)
    virtual void sof        ();

    //stddescr (required!)
    //Return standard descriptors
    //  descr - descriptor index (USB_DESCR_DEVICE, USB_DESCR_CONFIG, etc)
    //  data - return data
    //  size - descriptor length
    virtual void stddescr   (uint16_t descr, const void** data, uint16_t* size) = 0;

    //usb_class_ep0_in (optional)
    //IN request of endpoint 0
    //  req - request
    //  data - requrn data
    //  size - size of return data
    //return value: 0 - request not processed
    //              1 - request processed
    //char usb_class_ep0_in(config_pack_t *req, void **data, uint16_t *size);
    virtual char ep0_in  (config_pack_t* req, void** data, uint16_t *size);

    //OUT request of endpoint 0
    //  req - request
    //  offset - offset of received payload
    //  size - size of received payload
    //return value: 0 - request not processed
    //              1 - request processed (usb_ep_read(0, &data) called)
    virtual char ep0_out (config_pack_t* req, uint16_t offset, uint16_t rx_size);
    static USBDevice& device ();


    config_pack_t setup_packet;
    const uint8_t* ep0_buf = nullptr;
    uint16_t ep0_count = 0;
};


/*************************************************************************************
 *************************************************************************************
 ****  REQUIRED FUNCTIONS  ***********************************************************
 *************************************************************************************
 *************************************************************************************
*/

/*************************************************************************************
 *************************************************************************************
 ****  AUXILIARY FUNTIONS AND CONSTANTS   ********************************************
 *************************************************************************************
 *************************************************************************************
 */

#define USB_DESCR_DEVICE    0x01
#define USB_DESCR_CONFIG    0x02
#define USB_DESCR_STRING    0x03
#define USB_DESCR_INTERFACE 0x04
#define USB_DESCR_ENDPOINT  0x05
#define USB_DESCR_QUALIFIER 0x06
#define USB_DESCR_OTHER_SPEED 0x07
#define USB_DESCR_INTERFACE_POWER 0x08
#define USB_DESCR_HID       0x21
#define USB_DESCR_HID_REPORT  0x22
#define USB_DESCR_CS_INTERFACE 0x24
#define USB_DESCR_ENDP_ISO  0x25

#define USB_STRING(name, str)                                                       \
USB_ALIGN static const struct name##_t{                                             \
        uint8_t  bLength;                                                           \
        uint8_t  bDescriptorType;                                                   \
        uint16_t bString[(sizeof(str) - 2) / 2];                                    \
                                                                                    \
        name##_t (uint8_t length, uint8_t descriptorType, const char16_t* descr) :  \
            bLength(length), bDescriptorType(descriptorType) {                      \
                memcpy(bString, descr, (sizeof(str) - 2) / 2);                      \
            }                                                                       \
} name = {sizeof(name), USB_DESCR_STRING, str}

#define USB_U16(x) ((x)&0xFF), (((x)>>8)&0xFF)
#define USB_U24(x) ((x)&0xFF), (((x)>>8)&0xFF), (((x)>>16)&0xFF)

#define USB_REQ_STANDARD  0x00
#define USB_REQ_CLASS     0x20
#define USB_REQ_VENDOR    0x40

#define USB_REQ_DEVICE    0x00
#define USB_REQ_INTERFACE 0x01
#define USB_REQ_ENDPOINT  0x02
#define USB_REQ_OTHER     0x03
// bRequest, standard; for bmRequestType == 0x80
#define GET_STATUS                      0x00
#define GET_DESCRIPTOR                  0x06
#define GET_CONFIGURATION               0x08
#define SET_FEAUTRE                     0x09
#define SET_IDLE_REQUEST                0x0a

// for bmRequestType == 0
#define CLEAR_FEATURE                   0x01
#define SET_FEATURE                     0x03 
#define SET_ADDRESS                     0x05
#define SET_DESCRIPTOR                  0x07
#define SET_CONFIGURATION               0x09
// for bmRequestType == 0x81, 1 or 0xB2
#define GET_INTERFACE                   0x0A
#define SET_INTERFACE                   0x0B
#define SYNC_FRAME                      0x0C
#define VENDOR_REQUEST                  0x01

// standard descriptors (wValue = DESCR_TYPE<<8 | DESCR_INDEX)
#define DEVICE_DESCRIPTOR               0x0100
#define CONFIGURATION_DESCRIPTOR        0x0200
#define STRING_DESCRIPTOR               0x0300
#define DEVICE_QUALIFIER_DESCRIPTOR     0x0600
#define HID_REPORT_DESCRIPTOR           0x2200

#include <stm32l151xc.h>
#define USB_EPx(num) (((volatile uint16_t*)USB)[(num)*2])

#undef usb_ep_ready
#define usb_ep_ready(epnum) (                                                               \
    (((epnum) & 0x80) && ((USB_EPx((epnum) & 0x0F) & USB_EPTX_STAT) != USB_EP_TX_VALID) ) ||\
    (!((epnum)& 0x80) && ((USB_EPx((epnum) & 0x0F) & USB_EPRX_STAT) == USB_EP_RX_NAK) )     \
)


#define _ARRLEN1(ign, x...) (1+sizeof((uint8_t[]){x})), x
#define __ARRLEN34(x1, x2, i3, i4, x...) x1, x2, USB_U16(4+sizeof((uint8_t[]){x})), x
#define _ARRLEN34(x1, x2, i3, i4, x...) __ARRLEN34(x1, x2, i3, i4, x)
#define __ARRLEN67(x1, x2, x3, x4, x5, i6, i7, x...) x1, x2, x3, x4, x5, USB_U16(7+sizeof((uint8_t[]){x})), x
#define _ARRLEN67(x1, x2, x3, x4, x5, i6, i7, x...) __ARRLEN67(x1, x2, x3, x4, x5, i6, i7, x)
#define ARRLEN1(ign, x...) _ARRLEN1(ign, x)
#define ARRLEN34(x) _ARRLEN34(x)
#define ARRLEN67(x) _ARRLEN67(x)

#define bLENGTH 0
#define wTOTALLENGTH 0,0

#define ENDP_TOG(num, tog) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) | tog; }while(0)

void _usb_ep_write(uint8_t idx, const uint16_t *buf, uint16_t size);
static inline void usb_ep_write(uint8_t epnum, const uint16_t *buf, uint16_t size){
    _usb_ep_write((epnum & 0x0F)*2, buf, size);
}

static inline void usb_ep_write_double(uint8_t epnum, const uint16_t *buf, uint16_t size){
    epnum &= 0x0F;
    uint8_t idx = !!( USB_EPx(epnum) & USB_EP_DTOG_RX );
    idx += 2*epnum;
    ENDP_TOG( epnum, USB_EP_DTOG_RX );
    _usb_ep_write(idx, buf, size);
}

int _usb_ep_read(uint8_t idx, uint16_t *buf);
static inline int usb_ep_read(uint8_t epnum, uint16_t *buf){
    return _usb_ep_read((epnum & 0x0F)*2 + 1, buf);
}

static inline int usb_ep_read_double(uint8_t epnum, uint16_t *buf){
    uint8_t idx = !(USB_EPx(epnum) & USB_EP_DTOG_RX);
    int res = _usb_ep_read((epnum & 0x0F)*2 + idx, buf);
    ENDP_TOG( (epnum & 0x0F), USB_EP_DTOG_TX );
    return res;
}

#endif // __USB_LIB_H__
