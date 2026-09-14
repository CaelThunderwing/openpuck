#include "controllers.h"
#include "config.h"
#include "puck_hid.h"
#include "mode_xinput.h"
#include "mode_switch_hori.h"
#include "mode_switch_pro.h"
#include "mode_ps5.h"
#include "mode_hidgyro.h"
#include "mode_ps3.h"
#include "mode_dinput.h"
#include "mode_sinput.h"
#include "mode_xbox_og.h"

IController *g_active = nullptr;

extern bool g_xbox360ConsoleMode;

// Map a USB-presentation mode to its singleton controller. Steam and Lizard share the puck controller.
IController *controllerFor(uint8_t mode)
{
    switch (mode) {
    case MODE_STEAM:
    case MODE_LIZARD:
        return &g_steamPuck;
    case MODE_XBOX:
        g_xbox360ConsoleMode = false;
        return &g_xboxCtl;
    case MODE_XBOX360_CONSOLE:
        g_xbox360ConsoleMode = true;
        return &g_xboxCtl;
    case MODE_XBOX_OG:
        return &g_xboxOgCtl;
    case MODE_SW_HORI:
        return &g_switchHori;
    case MODE_SW_PRO:
        return &g_switchPro;
    case MODE_PS5:
        return &g_ps5Ctl;
    case MODE_HIDGYRO:
    case MODE_DS4_GAME:
        // Same DS4 controller; setup() drops wake/WebUSB for the clean enum.
        return &g_hidGyroCtl;
    case MODE_PS5_GAME:
        // Same DualSense controller; setup() drops wake/WebUSB for the clean enum.
        return &g_ps5Ctl;
    case MODE_PS3:
        return &g_ps3Ctl;
    case MODE_DINPUT:
        return &g_dinputCtl;
    case MODE_SINPUT:
        return &g_sinputCtl;
    default:
        return &g_steamPuck;
    }
}
