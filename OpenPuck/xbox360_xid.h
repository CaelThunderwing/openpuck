#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <Adafruit_TinyUSB.h>

bool xbox360VendorControlXfer(
    uint8_t rhport,
    uint8_t stage,
    const tusb_control_request_t *request);