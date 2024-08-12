//12.11.2022: add default EP callbacks
//24.07.2022: remove USB_Addr global variable; change usb types (u8->u16); add USB_ALIGN macro
//21.07.2022: Add default EPn callback; change sub-interrupt order (EPn -> SOF -> other)

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <functional>

#include <usb_lib.h>
#include "pinmacro.h"
#include "hardware.h"

//спасибо ST за наркоманскую работу с регистром!
#define ENDP_STAT_RX(num, stat) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ (stat); }while(0)
#define ENDP_STAT_TX(num, stat) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ (stat); }while(0)
#define ENDP_DTOG_RX(num, dtog) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ (dtog); }while(0)
#define ENDP_DTOG_TX(num, dtog) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EPRX_STAT | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ (dtog); }while(0)
#define ENDP_CTR_RX_CLR(num) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_CTR_RX)) | USB_EP_CTR_TX); }while(0)
#define ENDP_CTR_TX_CLR(num) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_CTR_TX)) | USB_EP_CTR_RX); }while(0)

struct usb_epdata_t {
    volatile uint32_t usb_tx_addr;
    volatile union{
        uint32_t usb_tx_count; //SINGLE mode, TX count
        struct{                //DOUBLE mode, RX struct
            uint32_t tx_count:10;
            uint32_t tx_num_blocks:5;
            uint32_t tx_blocksize:1;
        };
    };
    volatile uint32_t usb_rx_addr;
    volatile union{
        uint32_t usb_rx_count; //DOUBLE mode, TX count
        struct{                //SINGLE mode, RX struct
            uint32_t rx_count:10;
            uint32_t rx_num_blocks:5;
            uint32_t rx_blocksize:1;
        };
    };
};

constexpr size_t STM32ENDPOINTS = 8;
constexpr size_t LASTADDR_DEFAULT = STM32ENDPOINTS * 8;
#define usb_epdata (reinterpret_cast<volatile usb_epdata_t*>(USB_PMAADDR))

static void endp_callback_default (uint8_t epnum) {}

std::function<void(uint8_t)> epfunc_in [STM32ENDPOINTS];
std::function<void(uint8_t)> epfunc_out[STM32ENDPOINTS];

USBDevice* USBDevice::exemplar = nullptr;

USBDevice::USBDevice () {
    if (exemplar != nullptr) exit(1);

    exemplar = this;
}

void USBDevice::start () {
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->PMC  &= ~SYSCFG_PMC_USB_PU;

    USB->CNTR = USB_CNTR_FRES; // Force USB Reset
    //Initialization of callback functions (thx Олег Свиридов, my_xfiles@mail.ru)
    for (uint8_t i = 0; i < STM32ENDPOINTS; i++) {
        epfunc_in[i]  = endp_callback_default;
        epfunc_out[i] = endp_callback_default;
    }
    for (uint32_t ctr = 0; ctr < 100000; ctr++) asm volatile("nop"); // wait >1ms
    USB->CNTR   = 0;
    USB->BTABLE = 0;
    USB->DADDR  = 0;
    USB->ISTR   = 0;
    USB->CNTR   = USB_CNTR_RESETM | USB_CNTR_WKUPM;
    NVIC_EnableIRQ(USB_LP_IRQn);

    SYSCFG->PMC |= SYSCFG_PMC_USB_PU;
}

USBDevice& USBDevice::device () {
    return *exemplar;
}

void USBDevice::init       () {}
void USBDevice::poll       () {}
void USBDevice::disconnect () {}
void USBDevice::sof        () {}

char USBDevice::ep0_in  (config_pack_t *req, void **data, uint16_t *size)       { return 0; }
char USBDevice::ep0_out (config_pack_t *req, uint16_t offset, uint16_t rx_size) { return 0; }

//#define ep0_buf      (USBDevice::device().ep0_buf)
//#define ep0_count    (USBDevice::device().ep0_count)
//#define setup_packet (USBDevice::device().setup_packet)

static void ep0_in (uint8_t epnum) {
    auto& ep0_buf   = USBDevice::device().ep0_buf;
    auto& ep0_count = USBDevice::device().ep0_count;

    if (ep0_buf == nullptr) return;
    uint16_t left = ep0_count;
    if (left > USB_EP0_BUFSZ) left = USB_EP0_BUFSZ;
    usb_ep_write(0, (uint16_t*)ep0_buf, left);
    
    ep0_count -= left;
    ep0_buf   += left;
    
    if (left < USB_EP0_BUFSZ) ep0_buf = nullptr;
}

inline static void ep0_send (const uint16_t *buf, uint16_t size) {
    auto& ep0_buf   = USBDevice::device().ep0_buf;
    auto& ep0_count = USBDevice::device().ep0_count;

    ep0_count = size;
    ep0_buf = (uint8_t*)buf;
    ep0_in(0x80);
}

static uint8_t configuration = 0;

static void ep0_out (uint8_t epnum) {
    auto& setup_packet = USBDevice::device().setup_packet;

    static uint16_t bytesread = 0;
    uint16_t rxcount = usb_epdata[0].rx_count;
    uint8_t setup = !!(USB_EPx(0) & USB_EP_SETUP);

    if (setup) {
        if (setup_packet.bmRequestType & 0x80) { //предыдущий пакет=IN, этот точно будет запросом
        usb_ep_read(0, (uint16_t*)&setup_packet);
        bytesread = 0;
        rxcount = 0;
        } else { //предыдущий пакет=OUT, этот будет либо данными к нему, либо новым запросом
            if (bytesread >= setup_packet.wLength) {
                usb_ep_read(0, (uint16_t*)&setup_packet);
                bytesread = 0;
                rxcount = 0;
            }
        }
    }

    uint8_t req = setup_packet.bmRequestType & 0x7F;
  
    if (setup_packet.bmRequestType & 0x80) {
        //---IN---
        if (!setup) return; //какой-то костыль. Иначе вызывается два раза
        if (req == (USB_REQ_STANDARD | USB_REQ_DEVICE)) {
            switch (setup_packet.bRequest) {
            case GET_DESCRIPTOR:{
                const void *data = nullptr;
                uint16_t size = 0;
                USBDevice::device().stddescr(setup_packet.wValue, &data, &size);
                size = std::min(size, setup_packet.wLength);
                
                ep0_send((const uint16_t*)data, size);
                return;
            }
            case GET_STATUS:
                usb_ep_write(0, (uint16_t*)"\0", 2); // send status: Bus Powered
                return;
            case GET_CONFIGURATION:
                usb_ep_write(0, (uint16_t*)&configuration, 1);
                return;
            }
        }

        void* data = nullptr;
        uint16_t size = 0;
        //usb_class_ep0_in(&setup_packet, &data, &size);
        USBDevice::device().ep0_in(&setup_packet, &data, &size);
        size = std::min(size, setup_packet.wLength);

        ep0_send((const uint16_t*)data, size);
    } else {
        //---OUT---
        if (req == (USB_REQ_STANDARD | USB_REQ_DEVICE)) {
            if (setup_packet.bRequest == SET_ADDRESS) {
                uint8_t USB_Addr = setup_packet.wValue;
                usb_ep_write(0, nullptr, 0);
                while((USB_EPx(0) & USB_EPTX_STAT) == USB_EP_TX_VALID);
                USB->DADDR = USB_DADDR_EF | USB_Addr;
                return;
            } else if(setup_packet.bRequest == SET_CONFIGURATION) {
                configuration = setup_packet.wValue;
                usb_ep_write(0, nullptr, 0);
                return;
            }
        }
        if (!USBDevice::device().ep0_out(&setup_packet, bytesread, rxcount)) {
            ENDP_STAT_RX(0, USB_EP_RX_VALID);
        }

        bytesread += rxcount;
        if (bytesread >= setup_packet.wLength) usb_ep_write(0, nullptr, 0);
    }
}

static uint16_t lastaddr = LASTADDR_DEFAULT;
void usb_ep_init (uint8_t epnum, uint8_t ep_type, uint16_t size, std::function<void(uint8_t)> func) {
    if (func == nullptr) func = endp_callback_default;
    uint8_t dir_in = (epnum & 0x80);
    epnum &= 0x0F;
    
    ENDP_STAT_TX(epnum, dir_in ? USB_EP_TX_DIS : USB_EP_RX_DIS);
    
    uint16_t buf = USB_EPx(epnum);
    buf = (buf & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
    buf = (buf & ~USB_EPADDR_FIELD) | epnum;
    //buf = (buf & ~USB_EP_T_FIELD) | ep_type;
    buf &= ~USB_EP_T_FIELD;
    
    switch (ep_type) {
    case USB_ENDP_CTRL: 
        buf |= USB_EP_CONTROL;
        break;
    case USB_ENDP_BULK: 
        buf |= USB_EP_BULK;
        break;
    case USB_ENDP_INTR: 
        buf |= USB_EP_INTERRUPT;
        break;
    default: 
        buf |= USB_EP_ISOCHRONOUS; //в дескрипторах изохронные точки могут иметь расширенные настройки
        break;
    }
    USB_EPx(epnum) = buf;
    
    if (dir_in) {
        usb_epdata[epnum].usb_tx_addr = lastaddr;
        epfunc_in[epnum] = func;

        ENDP_STAT_TX(epnum, ((ep_type & 0x03) == USB_ENDP_ISO) ? USB_EP_TX_VALID : USB_EP_TX_NAK);
    } else {
        usb_epdata[epnum].usb_rx_addr = lastaddr;
        if (size < 64) {
            usb_epdata[epnum].rx_blocksize  = 0;
            usb_epdata[epnum].rx_num_blocks = size / 2;
        } else {
            size = std::max<uint16_t>(size, 32);

            usb_epdata[epnum].rx_blocksize  = 1;
            usb_epdata[epnum].rx_num_blocks = size / 32 - 1;
        }

        epfunc_out[epnum] = func;
        ENDP_STAT_RX(epnum, USB_EP_RX_VALID);
    }

    lastaddr += size;
}

void usb_ep_init_double(uint8_t epnum, uint8_t ep_type, uint16_t size, std::function<void(uint8_t)> func){
    if (func == nullptr) func = endp_callback_default;
    uint8_t dir_in = (epnum & 0x80);
    epnum &= 0x0F;
    
    ENDP_STAT_TX(epnum, USB_EP_TX_DIS);
    ENDP_STAT_RX(epnum, USB_EP_RX_DIS);
    
    uint16_t buf = USB_EPx(epnum);
    buf = (buf & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
    buf = (buf & ~USB_EPADDR_FIELD) | epnum;
    
    buf &= ~USB_EP_T_FIELD;
    switch (ep_type) {
    case USB_ENDP_CTRL:
        buf |= USB_EP_CONTROL; 
        break;
    case USB_ENDP_BULK: 
        buf |= USB_EP_BULK | USB_EP_KIND; 
        break;
    case USB_ENDP_INTR: 
        buf |= USB_EP_INTERRUPT;
        break;
    default: 
        buf |= USB_EP_ISOCHRONOUS; //в дескрипторах изохронные точки могут иметь расширенные настройки
        break;
    }
    USB_EPx(epnum) = buf;
  
#define USB_EP_SWBUF_TX     USB_EP_DTOG_RX
#define USB_EP_SWBUF_RX     USB_EP_DTOG_TX
    if (dir_in) {
        usb_epdata[epnum].usb_tx_addr  = lastaddr;
        usb_epdata[epnum].usb_tx_count = 0;
        usb_epdata[epnum].usb_rx_addr  = lastaddr + size;
        usb_epdata[epnum].usb_rx_count = 0;
        
        buf = USB_EPx(epnum);
        USB_EPx(epnum) = (buf ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT | USB_EP_DTOG_TX | USB_EP_SWBUF_TX);
    } else {
        usb_epdata[epnum].usb_rx_addr = lastaddr;
        usb_epdata[epnum].usb_tx_addr = lastaddr + size;

        if (size < 64) {
            usb_epdata[epnum].rx_blocksize  = usb_epdata[epnum].tx_blocksize = 0;
            usb_epdata[epnum].rx_num_blocks = usb_epdata[epnum].tx_num_blocks = size / 2;
        }else{
            size = std::max<uint16_t>(size, 32);

            usb_epdata[epnum].rx_blocksize  = usb_epdata[epnum].tx_blocksize = 1;
            usb_epdata[epnum].rx_num_blocks = usb_epdata[epnum].tx_num_blocks = size / 32 - 1;
        }

        ENDP_STAT_RX(epnum, USB_EP_RX_VALID);
        ENDP_STAT_TX(epnum, USB_EP_TX_VALID);
    }
    epfunc_in[epnum]  = func;
    epfunc_out[epnum] = func;
        
    lastaddr += 2 * size;
}
//-----------------------------------------------------------------------
//--------- USB IRQ handler----------------------------------------------
//-----------------------------------------------------------------------
extern "C" void USB_LP_IRQHandler () {
    if (USB->ISTR & USB_ISTR_CTR) {
        while (USB->ISTR & USB_ISTR_CTR) {
            uint8_t epnum = USB->ISTR & USB_ISTR_EP_ID;
            if (USB_EPx(epnum) & USB_EP_CTR_RX) { //OUT
                epfunc_out[epnum](epnum);
                ENDP_CTR_RX_CLR(epnum);
            }
            if (USB_EPx(epnum) & USB_EP_CTR_TX) { //IN
                epfunc_in[epnum](epnum | 0x80);
                ENDP_CTR_TX_CLR(epnum);
            }
        }

        return;
    }
    
    if (USB->ISTR & USB_ISTR_SOF) {
        USBDevice::device().sof();
        USB->ISTR = (uint16_t)~USB_ISTR_SOF;
        return;
    }
    
    if (USB->ISTR & USB_ISTR_RESET) {
        USBDevice::device().disconnect();
        #ifdef USBLIB_SOF_ENABLE
        USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SOFM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
        #else
        USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
        #endif
        lastaddr = LASTADDR_DEFAULT;
        USB->DADDR = USB_DADDR_EF;
        for (uint8_t i = 0; i < STM32ENDPOINTS; i++) {
            epfunc_in[i]  = endp_callback_default;
            epfunc_out[i] = endp_callback_default;
        }
        
        // state is default - wait for enumeration
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        usb_ep_init(0x00, USB_ENDP_CTRL, USB_EP0_BUFSZ, ep0_out);
        usb_ep_init(0x80, USB_ENDP_CTRL, USB_EP0_BUFSZ, ep0_in);

        USBDevice::device().ep0_buf = nullptr;
        USBDevice::device().init();
    }
    
    if (USB->ISTR & USB_ISTR_SUSP) { // suspend -> still no connection, may sleep
        USBDevice::device().disconnect();
        USB->CNTR |= USB_CNTR_FSUSP | USB_CNTR_LPMODE;
        USB->ISTR = (uint16_t)~USB_ISTR_SUSP;
    }
    
    if (USB->ISTR & USB_ISTR_WKUP) { // wakeup
        USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LPMODE); // clear suspend flags
        USB->ISTR = (uint16_t)~USB_ISTR_WKUP;
    }
}

struct pma_descr_t {
    volatile uint32_t addr;
    volatile union {
        uint32_t count; //SINGLE mode, TX count
        struct {        //DOUBLE mode, RX struct
            uint32_t rx_count:10;
            uint32_t rx_num_blocks:5;
            uint32_t rx_blocksize:1;
        };
    };
};

void _usb_ep_write (uint8_t idx, const uint16_t *buf, uint16_t size) {
    pma_descr_t *descr = &((pma_descr_t*)usb_epdata)[idx];
    uint16_t N2 = (size + 1) >> 1;
    // the buffer is 16-bit, so we should copy data as it would be uint16_t
    uint16_t *buf16 = (uint16_t *)buf;
    volatile uint32_t *out = (volatile uint32_t*)((uint16_t *)(USB_PMAADDR + descr->addr*2));
    for (uint16_t i = 0; i < N2; i++, out++) {
        *out = buf16[i];
    }

    descr->count = size;
    
    ENDP_STAT_TX(idx / 2, USB_EP_TX_VALID);
}

int _usb_ep_read (uint8_t idx, uint16_t *buf) {
    pma_descr_t *descr = &((pma_descr_t*)usb_epdata)[idx];
    int sz = descr->rx_count;
    if (!sz) return 0;
    int n = (sz + 1) >> 1;
    volatile uint32_t *in = (volatile uint32_t*)((uint16_t *)(USB_PMAADDR + descr->addr*2));
    for (int i = 0; i < n; ++i, ++in) buf[i] = *(uint16_t*)in;
    
    ENDP_STAT_RX((idx/2), USB_EP_RX_VALID);
    return sz;
}

