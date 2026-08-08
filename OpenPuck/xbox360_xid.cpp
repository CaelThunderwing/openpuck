#include "xbox360_xid.h"
#include <Adafruit_TinyUSB.h>


// Xbox 360 XID descriptor
static const uint8_t X360_XID_DESCRIPTOR[] =
{
    0x10, 0x42,
    0x00, 0x01,
    0x01, 0x01,
    0x14, 0x06,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF
};


// XID input capabilities
static const uint8_t X360_CAPABILITIES_IN[] =
{
    0x00, 0x14,
    0xFF, 0x00,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF
};


// XID output capabilities
static const uint8_t X360_CAPABILITIES_OUT[] =
{
    0x00, 0x06,
    0xFF, 0xFF,
    0xFF, 0xFF
};


bool xbox360VendorControlXfer(
    uint8_t rhport,
    uint8_t stage,
    const tusb_control_request_t *request)
{

    if(stage != CONTROL_STAGE_SETUP)
        return true;


    // XID requests are device-to-host vendor requests
    if(request->bmRequestType != 0xC1)
        return false;


    /*
       GET_DESCRIPTOR
       wValue = 0x4200
    */
    if(request->bRequest == 0x06 &&
       request->wValue == 0x4200)
    {
        return tud_control_xfer(
            rhport,
            request,
            (void*)X360_XID_DESCRIPTOR,
            sizeof(X360_XID_DESCRIPTOR));
    }


    /*
       GET_CAPABILITIES
       0x0100 = input
       0x0200 = output
    */
    if(request->bRequest == 0x01)
    {

        if(request->wValue == 0x0100)
        {
            return tud_control_xfer(
                rhport,
                request,
                (void*)X360_CAPABILITIES_IN,
                sizeof(X360_CAPABILITIES_IN));
        }


        if(request->wValue == 0x0200)
        {
            return tud_control_xfer(
                rhport,
                request,
                (void*)X360_CAPABILITIES_OUT,
                sizeof(X360_CAPABILITIES_OUT));
        }
    }


    return false;
}