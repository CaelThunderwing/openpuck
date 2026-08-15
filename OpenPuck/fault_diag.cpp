#include "fault_diag.h"
#include <Arduino.h> // readResetReason(), NRF_POWER, NVIC_SystemReset, POWER_RESETREAS_*_Msk
#include <FreeRTOS.h>
#include <task.h> // uxTaskGetSystemState -> per-task stack high-water (overflow hypothesis check)
#include <string.h>
#include "rf_link.h" // g_pollsps/g_relayps/g_crcps/g_norxps/g_rfStallRecover -- flight-recorder vitals
#include "haptics.h" // g_ringFault, relayPending() -- relay-load vitals
#include "config.h" // g_usbMode / MODE_XBOX360_CONSOLE -- persistent Xbox USB boot trace
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

// GPREGRET2 markers. GPREGRET (id 0) is reserved by the Adafruit bootloader for the DFU magic; GPREGRET2 is
// free for the application and is retained across soft/watchdog/pin reset, cleared only on power-on/brownout.
// Arbitrary non-zero tags -- any value distinct from each other and from the power-on default (0) works.
#define G2_INTENT 0xB1u
#define G2_X360_RECOVERY 0xB2u
#define G2_FAULT 0xFAu

static uint8_t g_reason = RR_UNKNOWN;
static uint32_t g_resetReas = 0;
static uint8_t g_hangStage = 0xFF;

// ---- watchdog pre-reset PC capture ("software SWD") --------------------------------------------------------
// The nRF52 WDT raises a TIMEOUT interrupt ~2 LFCLK cycles BEFORE it resets the chip. We take that interrupt at
// the highest NVIC priority (0) -- above FreeRTOS/TinyUSB BASEPRI critical sections -- read the stacked PC of
// whatever code was wedged, and stash it in a .noinit RAM struct that survives the reset. Next boot reports it
// on the panel; map it with addr2line to name the stuck function. Caveat: this fires only if interrupts aren't
// hard-masked (PRIMASK/CPSID). If a watchdog reset reports NO pc, the hang was a PRIMASK-off / flash-stall type
// (a different, narrower class) -- itself a useful signal. .noinit isn't zeroed by the C runtime, so it rides
// through the watchdog reset (unlike GPREGRET2, which this board's bootloader wipes).
struct HangFrame {
	uint32_t pc, lr, magic;
};
#define HANGPC_MAGIC 0x48414E47u // "HANG"
__attribute__((section(".noinit"))) volatile struct HangFrame g_hangFrame;
static uint32_t g_reportHangPC = 0, g_reportHangLR = 0;

extern "C" __attribute__((naked)) void WDT_IRQHandler(void)
{
	__asm volatile(
		"tst lr, #4            \n" // EXC_RETURN bit2: 0=frame on MSP, 1=on PSP
		"ite eq                \n"
		"mrseq r0, msp         \n"
		"mrsne r0, psp         \n"
		"ldr r1, [r0, #24]     \n" // stacked PC  (frame[6])
		"ldr r3, [r0, #20]     \n" // stacked LR  (frame[5])
		"ldr r2, =g_hangFrame  \n"
		"str r1, [r2, #0]      \n"
		"str r3, [r2, #4]      \n"
		"ldr r1, =0x48414E47   \n"
		"str r1, [r2, #8]      \n"
		"b .                   \n" // hold until the WDT reset lands (microseconds away)
		".ltorg                \n");
}

// Enable the WDT TIMEOUT interrupt at top priority. Call once, right after NRF_WDT->TASKS_START in setup().
void faultDiagArmHangCapture()
{
	NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Msk;
	NVIC_SetPriority(
		WDT_IRQn,
		0); // above FreeRTOS configMAX_SYSCALL priority -> fires through BASEPRI guards
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);
}
uint32_t faultDiagHangPC()
{
	return g_reportHangPC;
}
uint32_t faultDiagHangLR()
{
	return g_reportHangLR;
}

// Hang breadcrumb: loop() stage written to GPREGRET2 as 0x80|stage. 0x80..0x9F can never collide with the
// intentional-reboot (0xB1) / HardFault (0xFA) markers or the power-on default (0), so the same register
// disambiguates all of them at boot.
#define G2_STAGE_FLAG 0x80u
static const char *const STAGE_STR[] = { "webusb", "ctrl.task", "serial",
					 "rfdiag", "rflink",	"haptic",
					 "led",	   "usbmount",	"usbtx" };
#define STAGE_COUNT ((uint8_t)(sizeof STAGE_STR / sizeof STAGE_STR[0]))

static const char *const REASON_STR[RR_COUNT] = {
	"unknown",   "power-on", "pin/replug", "WATCHDOG (hang)", "CPU lockup",
	"HARDFAULT", "reboot",	 "soft reset", "wake-from-off",
};

// HardFault override. The Adafruit core's default handler (cores/.../debug.cpp) already does NVIC_SystemReset();
// we replace it to first stamp the fault marker so the NEXT boot classifies this SREQ as RR_HARDFAULT rather
// than an intentional reboot. The core's strong symbol is only linked to satisfy the vector table, so our own
// strong definition takes its place. Keep this MINIMAL: we are in fault context on a possibly-corrupt stack --
// no Serial, no allocations, just stamp and reset.
extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
        __asm volatile(
                // Select the exception frame.
                "tst lr, #4                \n"
                "ite eq                    \n"
                "mrseq r0, msp             \n"
                "mrsne r0, psp             \n"

                // Core LR/PC remain at +20/+24 in basic and extended frames.

                // Save evidence in registers; this handler never returns.
                "ldr r4, [r0, #24]         \n" // PC
                "ldr r5, [r0, #20]         \n" // LR

                "ldr r0, =0xE000ED28       \n"
                "ldr r6, [r0]              \n" // CFSR
                "ldr r0, =0xE000ED2C       \n"
                "ldr r7, [r0]              \n" // HFSR

                // HFF words 48..52 must ALL still be erased.
                // If not, preserve the older evidence rather than corrupt it.
                "ldr r2, =0x000E80C0       \n"
                "movs r1, #0               \n"
                "mvns r1, r1               \n" // r1 = 0xFFFFFFFF

                "ldr r0, [r2, #0]          \n"
                "cmp r0, r1                \n"
                "bne 9f                    \n"
                "ldr r0, [r2, #4]          \n"
                "cmp r0, r1                \n"
                "bne 9f                    \n"
                "ldr r0, [r2, #8]          \n"
                "cmp r0, r1                \n"
                "bne 9f                    \n"
                "ldr r0, [r2, #12]         \n"
                "cmp r0, r1                \n"
                "bne 9f                    \n"
                "ldr r0, [r2, #16]         \n"
                "cmp r0, r1                \n"
                "bne 9f                    \n"

                // NVMC CONFIG = write-enable.
                "ldr r0, =0x4001E504       \n"
                "movs r1, #1               \n"
                "str r1, [r0]              \n"
                "dsb                        \n"

                // NVMC READY.
                "ldr r3, =0x4001E400       \n"

                // word 49 = PC
                "str r4, [r2, #4]          \n"
                "1: ldr r1, [r3]           \n"
                "cmp r1, #0                \n"
                "beq 1b                    \n"

                // word 50 = LR
                "str r5, [r2, #8]          \n"
                "2: ldr r1, [r3]           \n"
                "cmp r1, #0                \n"
                "beq 2b                    \n"

                // word 51 = CFSR
                "str r6, [r2, #12]         \n"
                "3: ldr r1, [r3]           \n"
                "cmp r1, #0                \n"
                "beq 3b                    \n"

                // word 52 = HFSR
                "str r7, [r2, #16]         \n"
                "4: ldr r1, [r3]           \n"
                "cmp r1, #0                \n"
                "beq 4b                    \n"

                // Commit HFF1 LAST at word 48.
                "ldr r1, =0x48464631       \n"
                "str r1, [r2, #0]          \n"
                "5: ldr r1, [r3]           \n"
                "cmp r1, #0                \n"
                "beq 5b                    \n"

                // Return NVMC to read mode.
                "movs r1, #0               \n"
                "str r1, [r0]              \n"
                "dsb                        \n"

                // Whether flash capture succeeded or the slot was occupied,
                // retain the existing GPREGRET2 HardFault classification.
                "9:                         \n"
                "ldr r2, =0x40000520       \n"
                "movs r1, #0xFA            \n"
                "str r1, [r2]              \n"

                // SYSRESETREQ.
                "dsb                        \n"
                "ldr r2, =0xE000ED0C       \n"
                "ldr r1, =0x05FA0004       \n"
                "str r1, [r2]              \n"
                "dsb                        \n"
                "b .                        \n"
                ".ltorg                    \n");
}

void faultDiagArmIntentionalReset()
{
	NRF_POWER->GPREGRET2 = G2_INTENT;
}

void faultDiagArmXbox360RecoveryReset()
{
    NRF_POWER->GPREGRET2 = G2_X360_RECOVERY;
}

bool faultDiagConsumeXbox360RecoveryReset()
{
    if ((uint8_t)NRF_POWER->GPREGRET2 != G2_X360_RECOVERY)
        return false;
    NRF_POWER->GPREGRET2 = 0;
    return true;
}


// Live stage + loop heartbeat. The post-reset GPREGRET2 breadcrumb only works if the bootloader preserves
// GPREGRET2 across a watchdog reset -- some clone boards clear it, so it reads blank. These LIVE values don't
// depend on surviving a reset: the WebUSB blob send runs on the USB SOF interrupt (independent of loop()), so
// when loop() wedges, the SOF drain can still report the current stage + how long loop() has been stuck,
// straight to the panel during the ~8s before the watchdog fires.
static volatile uint8_t g_curStage = 0xFF;
static volatile uint32_t g_loopBeatMs = 0;

void faultDiagSetStage(uint8_t stage)
{
	g_curStage = stage;
	NRF_POWER->GPREGRET2 = (uint8_t)(G2_STAGE_FLAG | (stage & 0x1Fu));
}

void faultDiagBeat()
{
	g_loopBeatMs = millis();
}
uint8_t faultDiagCurStage()
{
	return g_curStage;
}
uint32_t faultDiagStallMs()
{
	// ms since loop() last beat. ~0 when healthy (loop beats ~250x/s); grows while loop() is stuck.
	return (uint32_t)(millis() - g_loopBeatMs);
}
const char *faultDiagStageStr(uint8_t s)
{
	return s < STAGE_COUNT ? STAGE_STR[s] : "?";
}

uint8_t faultDiagHangStage()
{
	return g_hangStage;
}
const char *faultDiagHangStageStr()
{
	return g_hangStage < STAGE_COUNT ? STAGE_STR[g_hangStage] : "n/a";
}

// ---- flight recorder (data) --------------------------------------------------------------------------------
// The live ring lives in .noinit so it survives a watchdog reset (like g_hangFrame). At boot we copy the
// pre-reset trail into g_flightSaved (ordinary RAM -- no need to survive twice) so the live session can start a
// fresh ring while the console can still re-dump the old trail on demand. Declared here (above faultDiagBoot)
// because the boot classifier snapshots + dumps it; the push/tick/dump functions are implemented further down.
struct FRRec {
	uint32_t ms; // millis() at the push
	uint16_t arg; // event-specific
	uint8_t evt; // FR_*
	uint8_t stage; // loop stage current at the push (g_curStage)
};
#define FR_RING 96u
#define FR_MAGIC 0x464C4954u // "FLIT"
struct FlightRec {
	uint32_t magic;
	uint16_t head; // next write slot
	uint16_t count; // total pushed this session (saturating) -> "showing last N of M"
	FRRec ring[FR_RING];
	// live vitals, refreshed ~4x/s -- the last values captured before a wedge
	uint32_t vMs; // millis() at last refresh
	uint32_t loopPerSec; // loop() iterations in the last full second
	uint32_t heapUsed; // mallinfo().uordblks (bytes) -- trend up = leak/fragmentation
	uint16_t usbdStackFree, loopStackFree; // words (0 = overflowed)
	uint16_t pollsps, relayps, crcps, norxps;
	uint16_t rfHeal, ringFault;
	uint8_t curStage;
	uint8_t stallMs; // ms since last loop beat, capped 255
};
__attribute__((section(".noinit"))) static volatile struct FlightRec g_flight;
static struct FlightRec
	g_flightSaved; // boot-time copy of the pre-reset trail (BSS)
static bool g_haveSaved = false;
bool g_vitals = true; // live per-second CDC vitals line (console "VIT" toggles)

static const char *const FR_STR[] = { "none",  "beat",	  "SET",    "GET",
				      "relay", "rf-up",	  "rf-DN",  "HEAL!",
				      "mount", "SUSPEND", "resume", "OFF",
				      "RINGF", "save" };
#define FR_STR_COUNT ((uint8_t)(sizeof FR_STR / sizeof FR_STR[0]))
static const char *frEvtStr(uint8_t e)
{
	return e < FR_STR_COUNT ? FR_STR[e] : "?";
}

// ---- persistent Xbox 360 USB boot trace ------------------------------------------------------------------
#define X360BT_FILE "/x360boot.bin"
#define X360BT_FILE_A "/x360bta.bin"
#define X360BT_FILE_B "/x360btb.bin"
#define X360BT_TXN_MAGIC 0x58335458u /* "X3TX" */

#define X360BT_MAGIC 0x54423358u // "X3BT"
#define X360BT_VERSION 1u
#define X360BT_RING 128u
#define X360BT_QUIET_MS 10000u

struct X360BootRec { uint32_t ms; uint16_t arg; uint8_t stage; uint8_t reserved; };
struct X360BootFile { uint32_t magic; uint16_t version; uint16_t count; uint32_t total; X360BootRec rec[X360BT_RING]; };
static X360BootRec g_x360bt[X360BT_RING];
static volatile uint16_t g_x360btHead = 0, g_x360btCount = 0;
static volatile uint32_t g_x360btTotal = 0, g_x360btLastMs = 0;
static volatile uint32_t g_x360btFirstMs = 0;
static volatile bool g_x360btActive = false, g_x360btDirty = false;
static bool g_x360btEarlyCommitted = false;
static volatile bool g_x360btMilestonePending = false;
static bool g_x360btMilestoneCommitted = false;

// Diagnostic persistence override: while Xbox traffic never becomes quiet,
// take exactly one early snapshot in each MCU boot. The normal 10 s quiet
// commit remains available afterward if capture continues, but the early
// snapshot latches so it cannot churn InternalFS continuously in one boot.
#define X360BT_EARLY_SNAPSHOT_MS 350u


// Dual-slot transactional wrapper for the persistent Xbox boot trace.
// We never destroy the previously-valid slot before a replacement has been
// written and verified. A reset during remove/write can therefore damage at
// most the slot currently being replaced.
struct X360BootTxn {
    uint32_t magic;
    uint32_t generation;
    X360BootFile payload;
    uint32_t checksum;
};

// These InternalFS operations run only from setup()/loop(). USB/ISR callers
// append to g_x360bt but never enter the transaction helpers, so fixed
// file-scope workspaces avoid multi-kilobyte loop-stack frames safely.
static X360BootFile g_x360FileScratch;
static X360BootTxn g_x360TxnScratch;

static uint32_t x360TxnChecksum(const X360BootTxn &t)
{
    // FNV-1a over generation + payload. The wrapper is memset(0) before use,
    // so any padding inside X360BootFile is deterministic on write/read.
    const uint8_t *b = (const uint8_t *)&t.generation;
    const size_t n = sizeof(t.generation) + sizeof(t.payload);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

static bool x360TxnRead(const char *path, X360BootTxn &out)
{
    File f(InternalFS);
    if (!f.open(path, FILE_O_READ))
        return false;
    memset(&out, 0, sizeof out);
    int got = f.read((uint8_t *)&out, sizeof out);
    f.close();
    if (got != (int)sizeof out)
        return false;
    if (out.magic != X360BT_TXN_MAGIC)
        return false;
    if (out.payload.magic != X360BT_MAGIC ||
        out.payload.version != X360BT_VERSION ||
        out.payload.count > X360BT_RING)
        return false;
    return out.checksum == x360TxnChecksum(out);
}

static bool x360TxnProbe(const char *path, uint32_t &generation)
{
    if (!x360TxnRead(path, g_x360TxnScratch))
        return false;
    generation = g_x360TxnScratch.generation;
    return true;
}

static const char *x360TxnNewestPath(uint32_t &generation)
{
    uint32_t ga = 0, gb = 0;
    bool va = x360TxnProbe(X360BT_FILE_A, ga);
    bool vb = x360TxnProbe(X360BT_FILE_B, gb);
    if (!va && !vb) {
        generation = 0;
        return NULL;
    }
    if (va && (!vb || ga >= gb)) {
        generation = ga;
        return X360BT_FILE_A;
    }
    generation = gb;
    return X360BT_FILE_B;
}

static bool x360TxnCommit(const X360BootFile &payload)
{
    uint32_t ga = 0, gb = 0;
    bool va = x360TxnProbe(X360BT_FILE_A, ga);
    bool vb = x360TxnProbe(X360BT_FILE_B, gb);

    uint32_t newest = 0;
    if (va && ga > newest) newest = ga;
    if (vb && gb > newest) newest = gb;

    // Replace the older/invalid slot. The other valid slot is never touched.
    const char *target;
    if (!va)
        target = X360BT_FILE_A;
    else if (!vb)
        target = X360BT_FILE_B;
    else
        target = (ga <= gb) ? X360BT_FILE_A : X360BT_FILE_B;

    X360BootTxn &tx = g_x360TxnScratch;
    memset(&tx, 0, sizeof tx);
    tx.magic = X360BT_TXN_MAGIC;
    tx.generation = newest + 1u;
    if (tx.generation == 0) tx.generation = 1u;
    tx.payload = payload;
    tx.checksum = x360TxnChecksum(tx);

    const uint32_t expectedGeneration = tx.generation;
    const uint32_t expectedChecksum = tx.checksum;

    // Safe because the other slot still contains the previous valid record.
    InternalFS.remove(target);
    File f(InternalFS);
    if (!f.open(target, FILE_O_WRITE))
        return false;
    int wrote = f.write((const uint8_t *)&tx, sizeof tx);
    f.close();
    if (wrote != (int)sizeof tx)
        return false;

    // Verify from storage before declaring the snapshot committed.
    if (!x360TxnRead(target, tx))
        return false;
    return tx.generation == expectedGeneration &&
           tx.checksum == expectedChecksum;
}


// ---- mode-independent persistent MCU boot history --------------------------------
//
// Unlike the X360 USB trace, this recorder is active in EVERY USB mode.
// It preserves the actual sequence of MCU boots across a failure followed
// by any necessary replug/mode-recovery steps.
//
// Two transactional InternalFS files are alternated. A new write never
// destroys the newest previously-valid copy before its replacement verifies.

#define BOOTHIST_MAGIC       0x3148424Fu  // "OBH1"
#define BOOTHIST_TXN_MAGIC   0x31544842u  // "BHT1"
#define BOOTHIST_VERSION     2u
#define BOOTHIST_RING        8u
#define BOOTHIST_FILE_A      "/opk_bhist_a.bin"
#define BOOTHIST_FILE_B      "/opk_bhist_b.bin"

struct BootHistRec {
    uint32_t seq;
    uint32_t resetReas;
    uint8_t mode;
    uint8_t reason;
    uint8_t gpregret2;
    uint8_t reserved;
    uint32_t faultPC;
    uint32_t faultLR;
    uint32_t faultCFSR;
    uint32_t faultHFSR;
};

struct BootHistFile {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t nextSeq;
    BootHistRec rec[BOOTHIST_RING];
};

struct BootHistTxn {
    uint32_t magic;
    uint32_t generation;
    BootHistFile payload;
    uint32_t checksum;
};

static uint32_t bootHistChecksum(const BootHistTxn &tx)
{
    const uint8_t *b = (const uint8_t *)&tx;
    const size_t n = offsetof(BootHistTxn, checksum);

    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

static bool bootHistRead(const char *path, BootHistTxn &tx)
{
    File f(InternalFS);
    if (!f.open(path, FILE_O_READ))
        return false;

    memset(&tx, 0, sizeof tx);
    int got = f.read((uint8_t *)&tx, sizeof tx);
    f.close();

    if (got != (int)sizeof tx ||
        tx.magic != BOOTHIST_TXN_MAGIC ||
        tx.payload.magic != BOOTHIST_MAGIC ||
        tx.payload.version != BOOTHIST_VERSION ||
        tx.payload.count > BOOTHIST_RING)
        return false;

    return tx.checksum == bootHistChecksum(tx);
}

static bool bootHistProbe(const char *path, uint32_t &generation)
{
    BootHistTxn tx;
    if (!bootHistRead(path, tx))
        return false;

    generation = tx.generation;
    return true;
}

static bool bootHistNewest(BootHistFile &out, uint32_t &generation)
{
    BootHistTxn a, b;
    bool va = bootHistRead(BOOTHIST_FILE_A, a);
    bool vb = bootHistRead(BOOTHIST_FILE_B, b);

    if (!va && !vb) {
        generation = 0;
        return false;
    }

    const BootHistTxn *best =
        (va && (!vb || a.generation >= b.generation)) ? &a : &b;

    out = best->payload;
    generation = best->generation;
    return true;
}

static bool bootHistCommit(const BootHistFile &payload)
{
    uint32_t ga = 0, gb = 0;
    bool va = bootHistProbe(BOOTHIST_FILE_A, ga);
    bool vb = bootHistProbe(BOOTHIST_FILE_B, gb);

    uint32_t newest = 0;
    if (va && ga > newest)
        newest = ga;
    if (vb && gb > newest)
        newest = gb;

    const char *target;
    if (!va)
        target = BOOTHIST_FILE_A;
    else if (!vb)
        target = BOOTHIST_FILE_B;
    else
        target = (ga <= gb) ? BOOTHIST_FILE_A : BOOTHIST_FILE_B;

    BootHistTxn tx;
    memset(&tx, 0, sizeof tx);
    tx.magic = BOOTHIST_TXN_MAGIC;
    tx.generation = newest + 1u;
    if (tx.generation == 0)
        tx.generation = 1u;
    tx.payload = payload;
    tx.checksum = bootHistChecksum(tx);

    InternalFS.remove(target);

    File f(InternalFS);
    if (!f.open(target, FILE_O_WRITE))
        return false;

    int wrote = f.write((const uint8_t *)&tx, sizeof tx);
    f.close();

    if (wrote != (int)sizeof tx)
        return false;

    BootHistTxn verify;
    return bootHistRead(target, verify) &&
           verify.generation == tx.generation &&
           verify.checksum == tx.checksum;
}

static bool bootHistRecord(uint8_t mode,
                           uint8_t reason,
                           uint32_t resetReas,
                           uint8_t gpregret2,
                           const uint32_t *flashFault)
{
    BootHistFile h;
    uint32_t generation = 0;

    if (!bootHistNewest(h, generation)) {
        memset(&h, 0, sizeof h);
        h.magic = BOOTHIST_MAGIC;
        h.version = BOOTHIST_VERSION;
        h.nextSeq = 1;
    }

    if (h.nextSeq == 0)
        h.nextSeq = 1;

    BootHistRec r;
    memset(&r, 0, sizeof r);
    r.seq = h.nextSeq++;
    r.resetReas = resetReas;
    r.mode = mode;
    r.reason = reason;
    r.gpregret2 = gpregret2;

    // Raw flash is authoritative across reset/startup.
    if (flashFault) {
        r.faultPC = flashFault[0];
        r.faultLR = flashFault[1];
        r.faultCFSR = flashFault[2];
        r.faultHFSR = flashFault[3];
    }

    if (h.count < BOOTHIST_RING) {
        h.rec[h.count++] = r;
    } else {
        for (unsigned i = 1; i < BOOTHIST_RING; ++i)
            h.rec[i - 1] = h.rec[i];
        h.rec[BOOTHIST_RING - 1] = r;
    }

    // Failure does not alter normal boot behavior. The caller uses this
    // result solely to decide whether the raw HFF1 record may be recycled.
    return bootHistCommit(h);
}


bool faultDiagBootHistSnapshot(struct FaultBootHistSnapshot *out)
{
    if (!out)
        return false;

    memset(out, 0, sizeof *out);
    out->version = BOOTHIST_VERSION;

    BootHistFile h;
    uint32_t generation = 0;

    if (!bootHistNewest(h, generation))
        return false;

    out->valid = 1;
    out->count = (uint8_t)h.count;
    out->generation = generation;
    out->nextSeq = h.nextSeq;

    for (uint8_t i = 0; i < out->count && i < BOOTHIST_RING; ++i) {
        out->rec[i].seq = h.rec[i].seq;
        out->rec[i].resetReas = h.rec[i].resetReas;
        out->rec[i].mode = h.rec[i].mode;
        out->rec[i].reason = h.rec[i].reason;
        out->rec[i].gpregret2 = h.rec[i].gpregret2;
        out->rec[i].reserved = h.rec[i].reserved;
        out->rec[i].faultPC = h.rec[i].faultPC;
        out->rec[i].faultLR = h.rec[i].faultLR;
        out->rec[i].faultCFSR = h.rec[i].faultCFSR;
        out->rec[i].faultHFSR = h.rec[i].faultHFSR;
    }

    return true;
}


void faultDiagUsbBootTraceBegin(bool capture)
{
    // Start a fresh RAM capture for this boot, but DO NOT delete the previously
    // committed InternalFS trace here. Mode 11 can reboot (including entering
    // and leaving the UF2 bootloader) before the user has a chance to retrieve
    // the capture. The next successful quiet-period commit atomically replaces
    // the old file instead.
    g_x360btActive = capture;
    g_x360btDirty = false;
    g_x360btHead = g_x360btCount = 0;
    g_x360btTotal = 0;
    g_x360btLastMs = 0;
    g_x360btFirstMs = 0;
    g_x360btEarlyCommitted = false;
    g_x360btMilestonePending = false;
    g_x360btMilestoneCommitted = false;
}

void faultDiagUsbBootTrace(uint16_t arg)
{
    if (!g_x360btActive) return;
    uint32_t pm = __get_PRIMASK(); __disable_irq();
    uint16_t h = g_x360btHead;
    g_x360bt[h].ms = millis(); g_x360bt[h].arg = arg; g_x360bt[h].stage = g_curStage; g_x360bt[h].reserved = 0;
    g_x360btHead = (uint16_t)((h + 1u) % X360BT_RING);
    if (g_x360btCount < X360BT_RING) g_x360btCount++;
    if (g_x360btTotal != 0xFFFFFFFFu) g_x360btTotal++;
    if (g_x360btFirstMs == 0) g_x360btFirstMs = g_x360bt[h].ms ? g_x360bt[h].ms : 1u;
    g_x360btLastMs = g_x360bt[h].ms; g_x360btDirty = true;
    if (arg == 0x00B6u && !g_x360btMilestoneCommitted)
        g_x360btMilestonePending = true;

    // C0xx is immediately followed by an MCU mode-switch reset. Force a new
    // transactional milestone even if an earlier B6 milestone already committed.
    if ((arg & 0xFF00u) == 0xC000u) {
        g_x360btMilestoneCommitted = false;
        g_x360btMilestonePending = true;
    }
    if (!pm) __enable_irq();
}

void faultDiagUsbBootTraceTask(void)
{
    if (!g_x360btActive || !g_x360btDirty) return;

    uint32_t now = millis();
    bool early = !g_x360btEarlyCommitted && g_x360btFirstMs &&
                 (uint32_t)(now - g_x360btFirstMs) >= X360BT_EARLY_SNAPSHOT_MS;
    bool quiet = (uint32_t)(now - g_x360btLastMs) >= X360BT_QUIET_MS;
    bool milestone = g_x360btMilestonePending && !g_x360btMilestoneCommitted;
    if (!early && !quiet && !milestone) return;

    // If an early write fails, retry at a bounded cadence rather than on every
    // loop iteration. This protects flash while still allowing transient FS
    // failures to recover within the same MCU boot.
    static uint32_t lastAttemptMs = 0;
    static uint8_t earlyAttempts = 0;
    if (early && !quiet && !milestone) {
        if (earlyAttempts >= 3u) return;
        if (lastAttemptMs && (uint32_t)(now - lastAttemptMs) < 250u) return;
        lastAttemptMs = now;
        earlyAttempts++;
    }

    X360BootFile &out = g_x360FileScratch;
    memset(&out, 0, sizeof out);
    out.magic = X360BT_MAGIC;
    out.version = X360BT_VERSION;

    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    uint16_t cnt = g_x360btCount;
    uint16_t start = (uint16_t)((g_x360btHead + X360BT_RING - cnt) % X360BT_RING);
    out.count = cnt;
    out.total = g_x360btTotal;
    for (uint16_t i = 0; i < cnt; i++)
        out.rec[i] = g_x360bt[(start + i) % X360BT_RING];
    if (!pm) __enable_irq();

    bool committed = x360TxnCommit(out);
    if (!committed)
        return;

    // Latch only AFTER verified persistence.
    if (early)
        g_x360btEarlyCommitted = true;
    if (milestone) {
        g_x360btMilestoneCommitted = true;
        g_x360btMilestonePending = false;
    }
    if (quiet)
        g_x360btDirty = false;
}

static void x360BootTraceLoadIntoFlight(void)
{
    X360BootFile &in = g_x360FileScratch;
    bool have = false;

    uint32_t gen = 0;
    const char *newest = x360TxnNewestPath(gen);
    if (newest) {
        X360BootTxn &tx = g_x360TxnScratch;
        if (x360TxnRead(newest, tx)) {
            in = tx.payload;
            have = true;
        }
    }

    // Migration/read fallback for traces created by older firmware.
    if (!have) {
        File f(InternalFS);
        if (f.open(X360BT_FILE, FILE_O_READ)) {
            memset(&in, 0, sizeof in);
            int got = f.read((uint8_t *)&in, sizeof in);
            f.close();
            have = got >= 12 &&
                   in.magic == X360BT_MAGIC &&
                   in.version == X360BT_VERSION &&
                   in.count <= X360BT_RING;
        }
    }

    if (!have)
        return;

    memset(&g_flightSaved, 0, sizeof g_flightSaved);
    g_flightSaved.magic = FR_MAGIC;
    uint16_t n = in.count < FR_RING ? in.count : (uint16_t)FR_RING;
    uint16_t src0 = (uint16_t)(in.count - n);
    for (uint16_t i = 0; i < n; i++) {
        const X360BootRec &r = in.rec[src0 + i];
        g_flightSaved.ring[i].ms = r.ms;
        g_flightSaved.ring[i].arg = r.arg;
        g_flightSaved.ring[i].evt = FR_SAVE;
        g_flightSaved.ring[i].stage = r.stage;
    }
    g_flightSaved.head = n % FR_RING;
    g_flightSaved.count =
        (uint16_t)(in.total > 0xFFFFu ? 0xFFFFu : in.total);
    g_haveSaved = true;
}

// ---- flash black box (pre-watchdog dump) -------------------------------------------------------------------
// On boards whose bootloader wipes .noinit + GPREGRET2 across a watchdog reset (Teyleten-class), EVERY
// post-mortem channel above is destroyed -- hangs on those boards were unforensicable. Flash survives any
// reset, so: a 4 Hz TIMER4 interrupt (priority 1, above FreeRTOS BASEPRI masking, below the WDT capture's 0)
// watches the loop heartbeat; when loop() has been frozen ~6 s (the WDT resets at 8 s), it walks the loop and
// usbd tasks' TCBs for their stacked PCs (both tasks BLOCKED = the suspected mutual-deadlock class) plus the
// PC it interrupted (covers the one-task-spinning class), and writes one record to a reserved raw flash page.
// Next boot reports it on CDC and feeds the existing hangPC/hangStage panel channels. Caveats: fires only if
// priority-1 interrupts still run (PRIMASK-off hangs leave no record -- same caveat as the WDT capture, and
// itself a signal); a task that is RUNNING (not blocked) has a stale TCB pxTopOfStack, so cross-check its PC
// against irqPC.
#define BB_ADDR \
	0xE8000UL // raw page: app image (~170 KB, ends < 0x60000) < here < InternalFS (0xED000)
#define BB_MAGIC 0x62627831u // "bbx1"
#define BB_WORDS 12
#define BB_DESTRUCT_MAGIC_ADDR (BB_ADDR + 4u * 13u)
#define BB_DESTRUCT_MAGIC      0x44535431u /* DST1 */
#define BB_DESTRUCT_ADDR       (BB_ADDR + 4u * 15u)
#define BB_DESTRUCT_BITS       0x0000000Fu
#define BBT_MAGIC 0x42545431u /* BTT1 */

// Auxiliary scheduler / re-enumeration snapshot.
//
// Existing BB-page ownership:
//   words 0..11 = normal black box
//   word 12     = seen marker
//   word 13     = destructive magic
//   word 15     = destructive bits
//
// Keep the independent snapshot well away at word 32.
#define BBS_WORD_BASE 32u
#define BBS_MAGIC     0x42535331u /* "BSS1" */

// Independent HardFault record in the already-reserved BB page.
//
//   word 48 = HFF1 commit magic -- written LAST
//   word 49 = stacked PC
//   word 50 = stacked LR
//   word 51 = CFSR
//   word 52 = HFSR
//
// A valid HFF1 record survives unrelated bbErase() operations until
// boot history has transactionally persisted it.
#define BBHF_WORD_BASE   48u
#define BBHF_MAGIC       0x48464631u /* "HFF1" */
#define BBHF_MAGIC_ADDR  (BB_ADDR + 4u * BBHF_WORD_BASE)

struct BbTimerTrail {
    uint32_t magic;
    uint32_t irqTicks;
    uint32_t beatChanges;
    uint16_t maxStuck;
    uint16_t flags;
};
__attribute__((section(".noinit"))) static volatile struct BbTimerTrail g_bbTimerTrail;
static uint8_t g_bbReportMaxStuck;
static uint8_t g_bbReportIrqTicks;
static uint8_t g_bbReportFlags;
static uint8_t g_bbReportBeatChanges;
uint8_t faultDiagBbTimerMaxStuck() { return g_bbReportMaxStuck; }
uint8_t faultDiagBbTimerIrqTicks() { return g_bbReportIrqTicks; }
uint8_t faultDiagBbTimerFlags() { return g_bbReportFlags; }
uint8_t faultDiagBbTimerBeatChanges() { return g_bbReportBeatChanges; }

static volatile uint8_t g_usbReenumPhase = 0;

void faultDiagUsbReenumPhase(uint8_t phase)
{
        g_usbReenumPhase = phase;
}

bool faultDiagBssSnapshot(uint32_t out[6])
{
        if (!out)
                return false;

        const volatile uint32_t *bs =
                (const volatile uint32_t *)(BB_ADDR +
                                            4u * BBS_WORD_BASE);

        if (bs[0] != BBS_MAGIC)
                return false;

        for (int i = 0; i < 6; ++i)
                out[i] = bs[1 + i];

        return true;
}

static TaskHandle_t g_hLoop,
	g_hUsbd; // captured by faultDiagStackTick's 1 Hz sweep
// non-static (like g_hangFrame): the naked ISR's `ldr =symbol` needs external linkage to resolve
extern "C" {
volatile uint32_t g_bbIrqPC,
	g_bbIrqLR; // stacked frame of whatever TIMER4 preempted
}
static void bbBootReport(
	bool isHang); // defined below faultDiagBlackBoxArm; called from faultDiagBoot
static void bbErase(void); // dump-time lazy erase (see bbTimerBody)

static void bbFlashWord(uint32_t addr, uint32_t v)
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	__DMB();
	*(volatile uint32_t *)addr = v;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	__DMB();
}

void faultDiagMarkDestructive(uint8_t code)
{
    if (code < 1u || code > 4u)
        return;

    if (*(volatile const uint32_t *)BB_DESTRUCT_MAGIC_ADDR != BB_DESTRUCT_MAGIC) {
        bbErase();
        bbFlashWord(BB_DESTRUCT_MAGIC_ADDR, BB_DESTRUCT_MAGIC);
    }

    uint32_t cur = *(volatile const uint32_t *)BB_DESTRUCT_ADDR;
    uint32_t next = cur & ~(1u << (code - 1u));
    if (next != cur)
        bbFlashWord(BB_DESTRUCT_ADDR, next);
}

uint8_t faultDiagDestructiveMask()
{
    if (*(volatile const uint32_t *)BB_DESTRUCT_MAGIC_ADDR != BB_DESTRUCT_MAGIC)
        return 0;
    uint32_t v = *(volatile const uint32_t *)BB_DESTRUCT_ADDR;
    return (uint8_t)((~v) & BB_DESTRUCT_BITS);
}

// Stacked PC/LR of a BLOCKED FreeRTOS task. pxTopOfStack is the first TCB member (portable-layer invariant);
// the CM4F PendSV push order is [r4-r11, EXC_RETURN] then (s16-s31 iff the FPU frame was live) then the
// hardware frame [r0-r3, r12, lr, pc, xpsr]. EXC_RETURN bit4 = 0 means the FPU frame is present.
static void bbTaskPc(TaskHandle_t h, uint32_t *pc, uint32_t *lr)
{
	*pc = *lr = 0;
	if (!h)
		return;
	uint32_t *tos = *(uint32_t **)h;
	uint32_t a = (uint32_t)tos;
	if (a < 0x20000000u || a >= 0x20040000u - (26 + 8) * 4)
		return; // corrupt TCB/stack -> don't fault inside the black box
	uint32_t excret = tos[8];
	uint32_t hw = 9 + ((excret & 0x10) ? 0 : 16);
	*pc = tos[hw + 6];
	*lr = tos[hw + 5];
}
extern "C" void bbTimerBody(void)
{
	NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	if (g_bbTimerTrail.magic == BBT_MAGIC)
	        g_bbTimerTrail.irqTicks++;
	(void)NRF_TIMER4->EVENTS_COMPARE[0]; // readback: event write is posted
	static uint32_t lastBeat;
	static uint16_t stuck;
	static bool dumped;
	uint32_t b = g_loopBeatMs;
	if (b != lastBeat) {
		lastBeat = b;
		stuck = 0;
		if (g_bbTimerTrail.magic == BBT_MAGIC)
		        g_bbTimerTrail.beatChanges++;
		return;
	}
	if (dumped)
		return;
	++stuck;
	if (g_bbTimerTrail.magic == BBT_MAGIC) {
	        if (stuck > g_bbTimerTrail.maxStuck)
	                g_bbTimerTrail.maxStuck = stuck;
	        if (stuck >= 2)
	                g_bbTimerTrail.flags |= 0x02u;
	}
	if (stuck <
	    24) { // 24 ticks @ 4 Hz = 6 s frozen; WDT reset lands at 8 s
		// USBD self-heal kick before giving up. The black-boxed wedge (2026-07-03, loopPC=start_dma,
		// usbdPC=idle xQueueReceive) is a USB EasyDMA whose END event/interrupt got swallowed (clone
		// USBD silicon under concurrent RADIO EasyDMA): TinyUSB's dma_running never clears, every next
		// transfer re-defers on the TASK_PRIO_HIGH usbd task (livelock), and TASK_PRIO_LOW loop starves.
		// If the END EVENT is latched but its IRQ was lost, re-pending the USBD IRQ makes the ISR run
		// edpt_dma_end -> the defer chain unwinds -> FULL recovery, no reset (the panel trail shows a
		// recovered stall episode). Harmless when nothing is pending: a spurious USBD ISR just returns.
		// >=500ms of frozen loop heartbeat is never legitimate (bond flash saves are ms-class).
		if (stuck >= 2) {
			NVIC_SetPendingIRQ(USBD_IRQn);
			faultDiagTrace(
				FR_HEAL,
				0x05Bu); // arg 0x5B = USBD kick (radio heal passes its own count)
		}
		return;
	}
	dumped = true;
	if (g_bbTimerTrail.magic == BBT_MAGIC)
	        g_bbTimerTrail.flags |= 0x04u;
	// Lazy erase: the page still holds the PREVIOUS hang's (already-reported) record -- boot no longer
	// erases it, so its CDC banner stays re-printable on any later debug boot. An in-ISR page erase is
	// ~85ms of CPU stall, irrelevant here: the system is already dead and the WDT reset is ~2s away.
	bbErase();
	if (g_bbTimerTrail.magic == BBT_MAGIC)
	        g_bbTimerTrail.flags |= 0x08u;
	// Record v2, written in TWO PHASES: phase 1 (PCs + vitals) commits BEFORE any USBD register is
	// touched -- if the USBD peripheral has wedged the AHB matrix, reading its registers could bus-stall
	// this ISR forever (the WDT still resets: its reset path is hardware, not CPU). A record with phase 1
	// present and phase 2 erased (0xFFFFFFFF) is therefore itself the proof of an AHB/USBD bus wedge.
	uint32_t lpc, llr, upc, ulr;
	bbTaskPc(g_hLoop, &lpc, &llr);
	bbTaskPc(g_hUsbd, &upc, &ulr);
	uint32_t w1[8] = {
		BB_MAGIC,
		(2u << 24) | ((uint32_t)g_curStage << 16), // ver 2
		lpc, // w2: loop task stacked PC (where it starved/blocked)
		g_bbIrqPC, // w3: what TIMER4 preempted = the code RUNNING at dump time
		upc, // w4: usbd task stacked PC
		((uint32_t)g_flight.usbdStackFree << 16) |
			g_flight.loopStackFree, // w5
		((uint32_t)g_flight.relayps << 16) | g_flight.pollsps, // w6
		g_flight.vMs, // w7: millis at last healthy vitals ~= wedge time
	};
	(void)llr;
	(void)ulr;
	for (int i = 0; i < 8; i++)
		bbFlashWord(BB_ADDR + 4u * i, w1[i]);

    // Independent scheduler / RTC snapshot.
    //
    // TIMER4 is NVIC priority 1, above normal FreeRTOS BASEPRI masking.
    // Capture RTC1 hardware/IRQ state and CPU mask state BEFORE reading
    // NRF_USBD registers.  Do not call FreeRTOS APIs from priority-1 TIMER4.
    uint32_t rtcState =
            (NRF_RTC1->INTENSET & 0x00FFFFFFu) |
            (NVIC_GetPendingIRQ(RTC1_IRQn) ? 0x40000000u : 0u) |
            (NVIC_GetEnableIRQ(RTC1_IRQn)  ? 0x80000000u : 0u);

    uint32_t cpuMask =
            (__get_BASEPRI() & 0xFFu) |
            ((__get_PRIMASK() & 1u) << 8);

    uint32_t bss[6] = {
            (uint32_t)g_usbReenumPhase,
            NRF_RTC1->COUNTER,
            0xFFFFFFFFu, // reserved: no FreeRTOS API from priority-1 TIMER4
            rtcState,
            cpuMask,
            SCB->ICSR,
    };

    // Payload first.
    for (int j = 0; j < 6; j++) {
            bbFlashWord(
                    BB_ADDR + 4u * (BBS_WORD_BASE + 1u + (uint32_t)j),
                    bss[j]);
    }

    // Commit magic LAST, so a partial write cannot look valid.
    bbFlashWord(BB_ADDR + 4u * BBS_WORD_BASE, BBS_MAGIC);

if (g_bbTimerTrail.magic == BBT_MAGIC)
        g_bbTimerTrail.flags |= 0x10u;
	// phase 2: USBD peripheral state -- did the DMA END event ever latch? are its interrupts enabled?
	uint32_t endmask = 0;
	for (int i = 0; i < 8; i++) {
		if (NRF_USBD->EVENTS_ENDEPIN[i])
			endmask |= (1u << i);
		if (NRF_USBD->EVENTS_ENDEPOUT[i])
			endmask |= (1u << (8 + i));
	}
	if (NRF_USBD->EVENTS_EPDATA)
		endmask |= (1u << 16);
	if (NRF_USBD->EVENTS_USBEVENT)
		endmask |= (1u << 17);
	if (NRF_USBD->EVENTS_EP0DATADONE)
		endmask |= (1u << 18);
	if (NRF_USBD->EVENTS_SOF)
		endmask |= (1u << 19);
	if (NVIC_GetPendingIRQ(USBD_IRQn))
		endmask |= (1u << 20); // IRQ pending but not being serviced
	if (NVIC_GetEnableIRQ(USBD_IRQn))
		endmask |= (1u << 21);
	uint32_t w2[4] = {
		endmask, // w8: latched-event bitmap (ENDEPIN 0-7, ENDEPOUT 8-15, EPDATA/USBEVENT/EP0DD/SOF, NVIC)
		NRF_USBD->INTEN, // w9: which USBD interrupts are enabled
		NRF_USBD->EPDATASTATUS, // w10: per-endpoint data-ready flags
		((uint32_t)NRF_USBD->EPINEN << 16) |
			(NRF_USBD->EPOUTEN & 0xFFFF), // w11
	};
	for (int i = 0; i < 4; i++)
		bbFlashWord(BB_ADDR + 4u * (8 + i), w2[i]);
}
// Naked shim: grab the interrupted context's stacked PC/LR (like the WDT capture), then tail-branch to the C
// body -- LR still holds EXC_RETURN, so the body's return IS the exception return.
extern "C" __attribute__((naked)) void TIMER4_IRQHandler(void)
{
	__asm volatile("tst lr, #4            \n"
		       "ite eq                \n"
		       "mrseq r0, msp         \n"
		       "mrsne r0, psp         \n"
		       "ldr r1, [r0, #24]     \n"
		       "ldr r2, =g_bbIrqPC    \n"
		       "str r1, [r2]          \n"
		       "ldr r1, [r0, #20]     \n"
		       "ldr r2, =g_bbIrqLR    \n"
		       "str r1, [r2]          \n"
		       "b bbTimerBody         \n"
		       ".ltorg                \n");
}

bool faultDiagBlackBoxSnapshot(struct FaultBlackBox *out)
{
        if (!out)
                return false;

        memset(out, 0, sizeof *out);
        const volatile uint32_t *w = (const volatile uint32_t *)BB_ADDR;
        if (w[0] != BB_MAGIC)
                return false;

        out->valid = 1;
        out->version = (uint8_t)(w[1] >> 24);
        out->stage = (uint8_t)(w[1] >> 16);
        out->loopPC = w[2];
        out->irqPC = w[3];
        out->usbdPC = w[4];
        out->usbdStackFree = (uint16_t)(w[5] >> 16);
        out->loopStackFree = (uint16_t)(w[5] & 0xFFFFu);
        out->pollsps = (uint16_t)(w[6] & 0xFFFFu);
        out->relayps = (uint16_t)(w[6] >> 16);
        out->wedgeMs = w[7];
        out->usbdRegsReadable =
                !((w[8] == 0xFFFFFFFFu) && (w[9] == 0xFFFFFFFFu));
        out->usbdEvents = w[8];
        out->usbdInten = w[9];
        out->epDataStatus = w[10];
        out->epEnable = w[11];
        return true;
}

void faultDiagBlackBoxArm()
{
    g_bbTimerTrail.magic = BBT_MAGIC;
    g_bbTimerTrail.irqTicks = 0;
    g_bbTimerTrail.beatChanges = 0;
    g_bbTimerTrail.maxStuck = 0;
    g_bbTimerTrail.flags = 0x01u;

	NRF_TIMER4->TASKS_STOP = 1;
	NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit
			      << TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER4->PRESCALER = 9; // 16 MHz / 512 = 31250 Hz
	NRF_TIMER4->CC[0] = 31250u / 4u; // 4 Hz
	NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	NVIC_SetPriority(TIMER4_IRQn,
			 1); // above BASEPRI masking, below the WDT capture
	NVIC_ClearPendingIRQ(TIMER4_IRQn);
	NVIC_EnableIRQ(TIMER4_IRQn);
	NRF_TIMER4->TASKS_START = 1;
}
static void bbErase()
{
    uint32_t destructMagicKeep =
            *(volatile const uint32_t *)BB_DESTRUCT_MAGIC_ADDR;
    uint32_t destructKeep =
            *(volatile const uint32_t *)BB_DESTRUCT_ADDR;

    uint32_t hfKeep[5] = {0, 0, 0, 0, 0};
    const volatile uint32_t *hf =
            (const volatile uint32_t *)(BB_ADDR +
                                        4u * BBHF_WORD_BASE);
    bool keepHardFault = (hf[0] == BBHF_MAGIC);

    if (keepHardFault) {
        for (int i = 0; i < 5; ++i)
            hfKeep[i] = hf[i];
    }
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
	__DMB();
	NRF_NVMC->ERASEPAGE = BB_ADDR;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	__DMB();

    if (destructMagicKeep == BB_DESTRUCT_MAGIC) {
        bbFlashWord(BB_DESTRUCT_MAGIC_ADDR, BB_DESTRUCT_MAGIC);
        if (destructKeep != 0xFFFFFFFFu)
            bbFlashWord(BB_DESTRUCT_ADDR, destructKeep);
    }

    if (keepHardFault) {
        // Restore payload first; commit magic last.
        for (int i = 1; i < 5; ++i)
            bbFlashWord(
                    BB_ADDR +
                            4u * (BBHF_WORD_BASE + (uint32_t)i),
                    hfKeep[i]);

        bbFlashWord(BBHF_MAGIC_ADDR, BBHF_MAGIC);
    }
}
// Boot-side: report the record. Runs from faultDiagBoot AFTER the .noinit hangPC/hangStage recovery, so on
// boards where those survive the black box only fills the gaps; on RAM-wiping boards it IS the evidence.
// Lifecycle: the record is NOT erased here -- a normal boot often has no CDC attached, so erasing on first
// boot would destroy the register words unread. Instead it persists (the banner re-prints on ANY later boot,
// e.g. a debug-CDC reboot) and the NEXT dump erases the page itself. A "seen" marker at word 12 keeps the
// panel adoption one-shot so a stale record can't masquerade as a later, unrelated hang's evidence.
static void bbBootReport(bool isHang)
{
	const volatile uint32_t *w = (const volatile uint32_t *)BB_ADDR;
	bool fresh = (w[12] == 0xFFFFFFFFu);
	if (w[0] != BB_MAGIC) {
		// garbage/partial page (interrupted dump or pre-feature content): clean it so the next dump
		// writes onto erased flash. All-blank needs no erase.
		bool blank = true;
		for (int i = 0; i < BB_WORDS; i++)
			if (w[i] != 0xFFFFFFFFu)
				blank = false;
		// keep a half-written phase-1 record (magic present handled below); anything else non-blank
		// without magic is garbage
		if (!blank)
			bbErase();
		return;
	}
	{
		uint8_t stage = (uint8_t)(w[1] >> 16);
		Serial.printf(
			"# BLACK BOX v%lu (flash dump written ~6s into the last hang):\n",
			(unsigned long)(w[1] >> 24));
		Serial.printf(
			"#   stage=%s  loopPC=%08lX  irqPC=%08lX (running at dump)  usbdPC=%08lX\n",
			faultDiagStageStr(stage), (unsigned long)w[2],
			(unsigned long)w[3], (unsigned long)w[4]);
		Serial.printf(
			"#   stkFree usbd=%luw loop=%luw  polls/s=%lu relay/s=%lu  tWedge~%lums\n",
			(unsigned long)(w[5] >> 16),
			(unsigned long)(w[5] & 0xFFFF),
			(unsigned long)(w[6] & 0xFFFF),
			(unsigned long)(w[6] >> 16), (unsigned long)w[7]);
		// Independent scheduler / re-enumeration snapshot.
		const volatile uint32_t *bs =
		        (const volatile uint32_t *)(BB_ADDR +
		                                   4u * BBS_WORD_BASE);

		if (bs[0] == BBS_MAGIC) {
		        uint32_t rs = bs[4];
		        uint32_t cm = bs[5];

		        Serial.printf(
		                "#   BSS1: reenumPhase=%lu RTC1=%08lX\n",
		                (unsigned long)bs[1],
		                (unsigned long)bs[2]);

		        Serial.printf(
		                "#   BSS1: RTC1 INTEN=%06lX NVICpend=%u NVICen=%u"
		                " BASEPRI=%02lX PRIMASK=%u ICSR=%08lX\n",
		                (unsigned long)(rs & 0x00FFFFFFu),
		                (unsigned)((rs >> 30) & 1u),
		                (unsigned)((rs >> 31) & 1u),
		                (unsigned long)(cm & 0xFFu),
		                (unsigned)((cm >> 8) & 1u),
		                (unsigned long)bs[6]);
		}

		// phase 2 = USBD peripheral state; all-FF here with phase 1 present = reading USBD registers
		// bus-stalled the dump ISR = the peripheral wedged the AHB matrix (decisive by itself).
		if (w[8] == 0xFFFFFFFFu && w[9] == 0xFFFFFFFFu)
			Serial.printf(
				"#   USBD regs: UNREADABLE (dump died reading them -> AHB/USBD bus wedge)\n");
		else
			Serial.printf(
				"#   USBD: events=%08lX (ENDEPIN0-7,ENDEPOUT8-15,EPDATA16,USBEVT17,EP0DD18,SOF19,NVICpend20,NVICen21)"
				" INTEN=%08lX EPDATASTATUS=%08lX EPINEN=%04lX EPOUTEN=%04lX\n",
				(unsigned long)w[8], (unsigned long)w[9],
				(unsigned long)w[10],
				(unsigned long)(w[11] >> 16),
				(unsigned long)(w[11] & 0xFFFF));
		// feed the existing panel channels (hangLog / Last-reset tile) when the .noinit path came back
		// blank -- i.e. exactly the RAM-wiping boards this exists for. loopPC is the primary suspect
		// location; irqPC (the code actually RUNNING while loop starved) rides the LR slot. One-shot:
		// only a FRESH record (seen-marker still erased) may claim this boot's hang.
		if (isHang && fresh) {
			if (!g_reportHangPC && w[2]) {
				g_reportHangPC = w[2];
				g_reportHangLR = w[3];
			}
			if (g_hangStage == 0xFF && stage < STAGE_COUNT)
				g_hangStage = stage;
		}
		if (fresh)
			bbFlashWord(BB_ADDR + 4u * 12, 1u); // stamp "seen"
	}
}

void faultDiagBoot()
{
	uint32_t rr =
		readResetReason(); // latched + cleared by the core's init()
	uint8_t g2 = (uint8_t)NRF_POWER->GPREGRET2;
	NRF_POWER->GPREGRET2 = 0; // consume the marker for this boot cycle
	g_resetReas = rr;


	// Read committed raw HardFault evidence before BB-page lifecycle work.
	const volatile uint32_t *hf =
	        (const volatile uint32_t *)(BB_ADDR +
	                                    4u * BBHF_WORD_BASE);
	bool haveFlashFault = (hf[0] == BBHF_MAGIC);
	uint32_t flashFault[4] = {0, 0, 0, 0};

	if (haveFlashFault) {
	        flashFault[0] = hf[1];
	        flashFault[1] = hf[2];
	        flashFault[2] = hf[3];
	        flashFault[3] = hf[4];
	}
	// If this boot follows a watchdog/lockup reset and GPREGRET2 holds a stage breadcrumb (0x80|stage), recover
	// which loop stage was stuck. Only meaningful for hangs -- an intentional reboot/HardFault stamps its own
	// marker over the breadcrumb, and a clean power-on zeroes it.
	bool isHang = (rr & POWER_RESETREAS_DOG_Msk) ||
		      (rr & POWER_RESETREAS_LOCKUP_Msk);

        g_bbReportMaxStuck = 0;
        g_bbReportIrqTicks = 0;
        g_bbReportFlags = 0;
        g_bbReportBeatChanges = 0;
        if (isHang && g_bbTimerTrail.magic == BBT_MAGIC) {
                g_bbReportMaxStuck = (uint8_t)(g_bbTimerTrail.maxStuck > 255u ? 255u : g_bbTimerTrail.maxStuck);
                g_bbReportIrqTicks = (uint8_t)(g_bbTimerTrail.irqTicks > 255u ? 255u : g_bbTimerTrail.irqTicks);
                g_bbReportFlags = (uint8_t)(g_bbTimerTrail.flags & 0xFFu);
                g_bbReportBeatChanges = (uint8_t)(g_bbTimerTrail.beatChanges > 255u ? 255u : g_bbTimerTrail.beatChanges);
        }
        g_bbTimerTrail.magic = 0;

	if (isHang && (g2 & G2_STAGE_FLAG) && g2 < G2_INTENT)
		g_hangStage = (uint8_t)(g2 & 0x1Fu);
	else
		g_hangStage = 0xFF;

	// Recover the PC the WDT pre-reset ISR captured (if it fired -- i.e. the hang wasn't PRIMASK-off). Consume
	// the magic so a later watchdog reset that DIDN'T capture (PRIMASK-off) can't report a stale address.
	if (isHang && g_hangFrame.magic == HANGPC_MAGIC) {
		g_reportHangPC = g_hangFrame.pc;
		g_reportHangLR = g_hangFrame.lr;
	} else {
		g_reportHangPC = 0;
		g_reportHangLR = 0;
	}
	g_hangFrame.magic = 0;

	uint8_t reason;
	// Precedence: a physical pin reset / watchdog / lockup is unambiguous from RESETREAS; only a SREQ needs the
	// GPREGRET2 marker to split intentional reboot vs HardFault.
	if (rr & POWER_RESETREAS_RESETPIN_Msk)
		reason = RR_PIN;
	else if (rr & POWER_RESETREAS_DOG_Msk)
		reason = RR_WATCHDOG;
	else if (rr & POWER_RESETREAS_LOCKUP_Msk)
		reason = RR_LOCKUP;
	else if (rr & POWER_RESETREAS_SREQ_Msk)
           reason = (g2 == G2_FAULT || haveFlashFault) ? RR_HARDFAULT :
                    (g2 == G2_INTENT)                  ? RR_REBOOT :
                                                        RR_SOFT;
	else if (rr & POWER_RESETREAS_OFF_Msk)
		reason = RR_WAKE;
	else if (rr == 0)
		reason = RR_POWERON; // all bits clear == power-on / brownout
	else
		reason = RR_UNKNOWN;
	g_reason = reason;

	// Preserve this MCU boot independently of the active USB mode.
	// rr/g2 are the genuine values captured before GPREGRET2 was consumed.
	bool histCommitted =
           bootHistRecord(g_usbMode,
                          reason,
                          rr,
                          g2,
                          haveFlashFault ? flashFault : nullptr);

   if (haveFlashFault && histCommitted) {
           // Transaction is safely in InternalFS now. Consume HFF1, then
           // recycle/re-arm the whole shared page for the NEXT HardFault.
           bbFlashWord(BBHF_MAGIC_ADDR, BBHF_MAGIC & ~1u);
           bbErase();
   }

	Serial.printf(
		"# reset cause: %s (RESETREAS=0x%08lX gpregret2=0x%02X)\n",
		REASON_STR[reason], (unsigned long)rr, g2);
	if (g_hangStage != 0xFF)
		Serial.printf("# hang stage: %s (%u)\n",
			      faultDiagHangStageStr(), g_hangStage);
	if (g_reportHangPC)
		Serial.printf("# hang PC=0x%08lX LR=0x%08lX\n",
			      (unsigned long)g_reportHangPC,
			      (unsigned long)g_reportHangLR);

	// Flash black box: report + consume last hang's pre-watchdog dump (fills hangPC/stage on boards whose
	// bootloader wiped the .noinit/GPREGRET2 evidence above).
	bbBootReport(isHang);

	// Flight recorder: if the pre-reset trail survived (.noinit), snapshot it into ordinary RAM so the live
	// session can start a fresh ring while the console can still re-dump the old trail ("FR"). On a hang, dump
	// it now -- this is the post-mortem of what the firmware was doing in the seconds before it wedged.
	if (g_flight.magic == FR_MAGIC) {
		memcpy(&g_flightSaved, (const void *)&g_flight,
		       sizeof g_flightSaved);
		g_haveSaved = true;
	}
	g_flight.magic = FR_MAGIC; // start this session's trail fresh
	g_flight.head = 0;
	g_flight.count = 0;
	if (isHang)
		faultDiagDumpFlight();

	x360BootTraceLoadIntoFlight();
}

// ---- flight recorder (implementation; data declared above faultDiagBoot) ----------------------------------
void faultDiagTrace(uint8_t evt, uint16_t arg)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_flight.magic !=
	    FR_MAGIC) { // lazy (re)init: power-on garbage or first push this session
		g_flight.magic = FR_MAGIC;
		g_flight.head = 0;
		g_flight.count = 0;
	}
	uint16_t h = g_flight.head;
	g_flight.ring[h].ms = millis();
	g_flight.ring[h].arg = arg;
	g_flight.ring[h].evt = evt;
	g_flight.ring[h].stage = g_curStage;
	g_flight.head = (uint16_t)((h + 1u) % FR_RING);
	if (g_flight.count < 0xFFFFu)
		g_flight.count++;
	if (!pm)
		__enable_irq();
}

void faultDiagFlightTick(void)
{
	static unsigned long beatMs = 0, secMs = 0;
	static uint32_t loops = 0;
	loops++;
	unsigned long now = millis();

	// ~4x/s: refresh the persistent vitals snapshot + drop a heartbeat crumb (its cadence in the trail shows
	// exactly when loop() stopped advancing).
	if ((uint32_t)(now - beatMs) >= 250u) {
		beatMs = now;
		uint32_t stall = faultDiagStallMs();
		g_flight.vMs = now;
		g_flight.usbdStackFree = faultDiagUsbdStackFree();
		g_flight.loopStackFree = faultDiagLoopStackFree();
		// NOTE: mallinfo() removed here -- it suspended the FreeRTOS scheduler + walked the heap free list 4x/s,
		// which is new loop latency and a new wedge surface on a slow clone, for zero payoff (heap was flat). If
		// a heap-usage trend is ever needed, sample it lazily from loop, not on this hot path.
		g_flight.heapUsed = 0;
		g_flight.pollsps = g_pollsps;
		g_flight.relayps = g_relayps;
		g_flight.crcps = g_crcps;
		g_flight.norxps = g_norxps;
		g_flight.rfHeal = g_rfStallRecover;
		g_flight.ringFault = g_ringFault;
		g_flight.curStage = g_curStage;
		g_flight.stallMs = (uint8_t)(stall > 255 ? 255 : stall);
		faultDiagTrace(FR_BEAT, (uint16_t)g_flight.stallMs);
	}

	// once per second: latch loop rate + emit the live CDC vitals line (loop context -> Serial is safe here,
	// unlike the usbd task). Reads as a trend on a flaky board: usbd stack falling toward 0 = the overflow;
	// heapUsed climbing = a leak; poll==0 = the RF loop stalled; relay high = Steam flooding the link.
	if ((uint32_t)(now - secMs) >= 1000u) {
		g_flight.loopPerSec = loops;
		loops = 0;
		secMs = now;
		// Guard on FIFO space: a CDC write to a host that has the port open but isn't draining blocks the loop --
		// itself a watchdog-hang source (same failure the WebUSB blob send hit). If there's no room this second,
		// skip the line rather than stall; the persistent trail still captured the vitals via the beat above.
		if (g_vitals && Serial && Serial.availableForWrite() >= 200) {
			// Clock fingerprint on every line: the radio waits time out on micros() (HFCLK-derived). If a clone's
			// HFCLK drops to the internal RC (or micros() freezes), usPerMs drifts off 1000 and a "bounded" RX/
			// disable wait can spin forever -- the prime suspect for a healthy-then-sudden hard lock. lf/hf: R=RC,
			// X=xtal, S=synth, -=stopped.
			const char clkc[] = { '-', 'R', 'X', 'S' };
			uint8_t lf = clockLfSrc(), hf = clockHfSrc();
			Serial.printf(
				"# vit up=%lus loop=%lu/s stall=%ums stage=%s usbdStk=%u loopStk=%u heapUsed=%lu "
				"poll=%u relay=%u crc=%u norx=%u heal=%u ringF=%u clk=%c/%c usPerMs=%u\n",
				(unsigned long)(now / 1000),
				(unsigned long)g_flight.loopPerSec,
				(unsigned)g_flight.stallMs,
				faultDiagStageStr(g_flight.curStage),
				(unsigned)g_flight.usbdStackFree,
				(unsigned)g_flight.loopStackFree,
				(unsigned long)g_flight.heapUsed,
				(unsigned)g_flight.pollsps,
				(unsigned)g_flight.relayps,
				(unsigned)g_flight.crcps,
				(unsigned)g_flight.norxps,
				(unsigned)g_flight.rfHeal,
				(unsigned)g_flight.ringFault,
				lf < 4 ? clkc[lf] : '?',
				hf < 4 ? clkc[hf] : '?',
				(unsigned)clockUsPerMs());
		}
	}
}

// Wait (bounded) for room in the CDC TX FIFO so a multi-line dump to a slow host neither drops lines nor
// blocks forever. Caps each line's wait at ~20ms -> the whole ~100-line dump can't approach the ~8s watchdog.
static void frWaitTx(uint16_t need)
{
	for (uint8_t w = 0;
	     w < 20 && Serial && Serial.availableForWrite() < need; w++)
		delay(1);
}

void faultDiagDumpFlight(void)
{
	if (!Serial) // no host attached (e.g. the automatic boot dump before the panel connects) -- use "FR" later
		return;
	if (!g_haveSaved || g_flightSaved.magic != FR_MAGIC) {
		Serial.println(
			"# flight: no pre-reset trail (recorder lost across this reset, or last boot wasn't a hang)");
		return;
	}
	const struct FlightRec *f = &g_flightSaved;
	Serial.printf(
		"# ---- FLIGHT RECORDER (trail up to the last hang) ----\n"
		"# vitals@wedge: loop=%lu/s stall=%ums stage=%s usbdStkFree=%u loopStkFree=%u heapUsed=%lu\n"
		"#               poll=%u relay=%u crc=%u norx=%u heal=%u ringFault=%u  (age %lums before reset)\n",
		(unsigned long)f->loopPerSec, (unsigned)f->stallMs,
		faultDiagStageStr(f->curStage), (unsigned)f->usbdStackFree,
		(unsigned)f->loopStackFree, (unsigned long)f->heapUsed,
		(unsigned)f->pollsps, (unsigned)f->relayps, (unsigned)f->crcps,
		(unsigned)f->norxps, (unsigned)f->rfHeal,
		(unsigned)f->ringFault, (unsigned long)0);

	uint16_t total = f->count;
	uint16_t cnt = total < FR_RING ? total : (uint16_t)FR_RING;
	if (!cnt) {
		Serial.println("# flight: trail empty");
		return;
	}
	uint16_t start = (uint16_t)((f->head + FR_RING - cnt) % FR_RING);
	uint16_t lastIdx = (uint16_t)((f->head + FR_RING - 1u) % FR_RING);
	uint32_t last = f->ring[lastIdx].ms;
	Serial.printf(
		"# %u events (of %u total); t = ms BEFORE the last recorded crumb:\n",
		cnt, total);
	for (uint16_t i = 0; i < cnt; i++) {
		uint16_t idx = (uint16_t)((start + i) % FR_RING);
		const FRRec *r = &f->ring[idx];
		frWaitTx(64);
		Serial.printf("#  -%6lu  %-8s stage=%-9s arg=0x%04X\n",
			      (unsigned long)(last - r->ms), frEvtStr(r->evt),
			      faultDiagStageStr(r->stage), r->arg);
	}
	Serial.println("# ---- end flight recorder ----");
}

// ---- WebUSB panel access to the saved trail ----------------------------------------------------------------
static uint16_t g_frDrain =
	0; // pull cursor into g_flightSaved, 0 = oldest saved event

bool faultDiagHaveFlight(void)
{
	return g_haveSaved && g_flightSaved.magic == FR_MAGIC;
}
uint16_t faultDiagFlightCount(void)
{
	if (!faultDiagHaveFlight())
		return 0;
	uint16_t t = g_flightSaved.count;
	return t < FR_RING ? t : (uint16_t)FR_RING;
}
uint16_t faultDiagFlightTotal(void)
{
	return faultDiagHaveFlight() ? g_flightSaved.count : 0;
}
void faultDiagFlightDrainReset(void)
{
	g_frDrain = 0;
}
bool faultDiagFlightPull(uint32_t *dtMs, uint8_t *evt, uint8_t *stage,
			 uint16_t *arg)
{
	uint16_t cnt = faultDiagFlightCount();
	if (g_frDrain >= cnt)
		return false;
	const struct FlightRec *f = &g_flightSaved;
	uint16_t start = (uint16_t)((f->head + FR_RING - cnt) % FR_RING);
	uint16_t idx = (uint16_t)((start + g_frDrain) % FR_RING);
	uint16_t lastIdx = (uint16_t)((f->head + FR_RING - 1u) % FR_RING);
	*dtMs = (uint32_t)(f->ring[lastIdx].ms - f->ring[idx].ms);
	*evt = f->ring[idx].evt;
	*stage = f->ring[idx].stage;
	*arg = f->ring[idx].arg;
	g_frDrain++;
	return true;
}
bool faultDiagFlightVitals(struct FaultFlightVitals *out)
{
	if (!faultDiagHaveFlight())
		return false;
	const struct FlightRec *f = &g_flightSaved;
	out->loopPerSec =
		(uint16_t)(f->loopPerSec > 0xFFFFu ? 0xFFFFu : f->loopPerSec);
	out->usbdStkFree = f->usbdStackFree;
	out->loopStkFree = f->loopStackFree;
	out->heapUsed = f->heapUsed;
	out->pollsps = f->pollsps;
	out->relayps = f->relayps;
	out->crcps = f->crcps;
	out->norxps = f->norxps;
	out->rfHeal = f->rfHeal;
	out->ringFault = f->ringFault;
	out->curStage = f->curStage;
	out->stallMs = f->stallMs;
	return true;
}

// ---- clock fingerprint -------------------------------------------------------------------------------------
static uint8_t g_clkLf = 0, g_clkHf = 0;
static uint16_t g_clkUsPerMs = 0;

void clockDiagBoot()
{
	// LFCLKSTAT: bit16 STATE(running), bits1:0 SRC (0=RC,1=Xtal,2=Synth). HFCLKSTAT: bit16 STATE, bit0 SRC
	// (0=RC,1=Xtal). The bare-metal radio needs HFXO; if HF shows RC here, RF runs on the 64 MHz internal RC.
	uint32_t lf = NRF_CLOCK->LFCLKSTAT;
	uint32_t hf = NRF_CLOCK->HFCLKSTAT;
	bool lfrun = lf & CLOCK_LFCLKSTAT_STATE_Msk;
	uint8_t lfsrc = (uint8_t)(lf & CLOCK_LFCLKSTAT_SRC_Msk);
	g_clkLf = lfrun ? (uint8_t)(lfsrc + 1) :
			  0; // 1=RC,2=Xtal,3=Synth, 0=stopped
	bool hfxtal = hf & CLOCK_HFCLKSTAT_SRC_Msk;
	g_clkHf = hfxtal ? 2 : 0; // 2=crystal, 0=RC
	Serial.printf("# clock: LFCLK=%s HFCLK=%s\n",
		      g_clkLf == 2 ? "xtal" :
		      g_clkLf == 1 ? "RC" :
		      g_clkLf == 3 ? "synth" :
				     "stopped",
		      g_clkHf == 2 ? "xtal" : "RC");
}

void clockDiagTick()
{
	// Cross-check the two time bases: count micros() (HFCLK-derived) elapsed over a ~1s millis() (LFCLK/RTC)
	// window. usPerMs = micros-delta / millis-delta; 1000 = the clocks agree. A clone whose HFCLK runs fast vs
	// its LFCLK reads >1000 here -- the exact signature behind "poll rate too high, delivered too low".
	static uint32_t u0 = 0;
	static unsigned long m0 = 0;
	static bool init = false;
	unsigned long m = millis();
	if (!init) {
		u0 = micros();
		m0 = m;
		init = true;
		return;
	}
	unsigned long dm = m - m0;
	if (dm >= 1000) {
		uint32_t du = (uint32_t)(micros() - u0);
		g_clkUsPerMs = (uint16_t)(du / dm);
		u0 = micros();
		m0 = m;
	}
}

uint8_t clockLfSrc()
{
	return g_clkLf;
}
uint8_t clockHfSrc()
{
	return g_clkHf;
}
uint16_t clockUsPerMs()
{
	return g_clkUsPerMs;
}

// ---- per-task stack headroom (usbd-overflow hypothesis check) ---------------------------------------------
// uxTaskGetSystemState (configUSE_TRACE_FACILITY=1) reports each task's stack high-water mark = the LEAST free
// stack it ever had, in words. The repo xTaskCreate wrapper raises the "usbd" task to 1024 words / 4096 B;
// if this trends toward 0 under haptic load, the stack overflow is confirmed. Repo-scoped, read-only, no core
// changes -- just observing FreeRTOS.
static uint16_t g_usbdStackMin = 0xFFFF;
static uint16_t g_loopStackMin = 0xFFFF;
void faultDiagStackTick()
{
	static unsigned long ms = 0;
	if ((uint32_t)(millis() - ms) < 1000)
		return;
	ms = millis();
	static TaskStatus_t st[12];
	UBaseType_t n = uxTaskGetSystemState(st, 12, NULL);
	for (UBaseType_t i = 0; i < n; i++) {
		uint16_t hw = (uint16_t)st[i].usStackHighWaterMark;
		if (!strcmp(st[i].pcTaskName, "usbd")) {
			g_hUsbd =
				st[i].xHandle; // black box: TCB to walk at dump time
			if (hw < g_usbdStackMin)
				g_usbdStackMin = hw;
		} else if (!strcmp(st[i].pcTaskName, "loop")) {
			g_hLoop = st[i].xHandle;
			if (hw < g_loopStackMin)
				g_loopStackMin = hw;
		}
	}
}
uint16_t faultDiagUsbdStackFree()
{
	return g_usbdStackMin == 0xFFFF ? 0 : g_usbdStackMin;
}
uint16_t faultDiagLoopStackFree()
{
	return g_loopStackMin == 0xFFFF ? 0 : g_loopStackMin;
}

bool faultDiagCaptureLoopStack(FaultDiagStackMap *out)
{
    if (!out || !g_hLoop)
        return false;

    // TCB member zero is pxTopOfStack on this FreeRTOS port; the existing
    // black-box task-PC walker relies on the same invariant.
    uint8_t *sp = (uint8_t *)(*(StackType_t **)g_hLoop);

    // The repo xTaskCreate wrapper raises only the core loop task to
    // 2048 StackType_t units (8192 bytes); keep this scan synchronized.
    const uint32_t stackBytes = 2048u * sizeof(StackType_t);

    // Obtain pxStackBase/HWM from FreeRTOS rather than guessing TCB layout
    // beyond pxTopOfStack.
    static TaskStatus_t st[12];
    UBaseType_t n = uxTaskGetSystemState(st, 12, NULL);

    TaskStatus_t *lp = NULL;
    for (UBaseType_t i = 0; i < n; ++i) {
        if (st[i].xHandle == g_hLoop ||
            (st[i].pcTaskName && !strcmp(st[i].pcTaskName, "loop"))) {
            lp = &st[i];
            break;
        }
    }
    if (!lp || !lp->pxStackBase)
        return false;

    uint8_t *base = (uint8_t *)lp->pxStackBase;

    uint32_t prefix = 0;
    while (prefix < stackBytes && base[prefix] == 0xA5u)
        ++prefix;

    uint32_t laterA5 = stackBytes;
    if (prefix < stackBytes) {
        for (uint32_t i = prefix + 1; i + 16u <= stackBytes; ++i) {
            bool run = true;
            for (uint32_t j = 0; j < 16u; ++j) {
                if (base[i + j] != 0xA5u) {
                    run = false;
                    break;
                }
            }
            if (run) {
                laterA5 = i;
                break;
            }
        }
    }

    out->base = (uint32_t)base;
    out->sp = (uint32_t)sp;
    out->hwm = (uint16_t)lp->usStackHighWaterMark;
    out->prefix = (uint16_t)(prefix > 0xFFFFu ? 0xFFFFu : prefix);
    out->laterA5 =
        (uint16_t)(laterA5 >= stackBytes ? 0xFFFFu : laterA5);
    out->stackBytes =
        (uint16_t)(stackBytes > 0xFFFFu ? 0xFFFFu : stackBytes);

    memcpy(out->raw, base, sizeof out->raw);
    return true;
}

void faultDiagDumpLoopStack()
{
    static TaskStatus_t st[12];
    UBaseType_t n = uxTaskGetSystemState(st, 12, NULL);

    TaskStatus_t *lp = NULL;
    for (UBaseType_t i = 0; i < n; i++) {
        if (!strcmp(st[i].pcTaskName, "loop")) {
            lp = &st[i];
            break;
        }
    }

    if (!lp) {
        Serial.println("STACKMAP loop task not found");
        return;
    }

    uint8_t *base = (uint8_t *)lp->pxStackBase;

    // pxTopOfStack is TCB member zero on this FreeRTOS build;
    // bbTaskPc() already relies on the same invariant.
    uint8_t *sp = (uint8_t *)(*(StackType_t **)lp->xHandle);

    // The repo xTaskCreate wrapper raises only the core loop task to
    // 2048 StackType_t units (8192 bytes); keep this scan synchronized.
    const uint32_t stackBytes = 2048u * sizeof(StackType_t);

    uint32_t prefix = 0;
    while (prefix < stackBytes && base[prefix] == 0xA5u)
        ++prefix;

    // Find the first later run of 16 untouched A5 bytes.
    uint32_t laterA5 = stackBytes;
    if (prefix < stackBytes) {
        for (uint32_t i = prefix + 1; i + 16u <= stackBytes; ++i) {
            bool run = true;
            for (uint32_t j = 0; j < 16u; ++j) {
                if (base[i + j] != 0xA5u) {
                    run = false;
                    break;
                }
            }
            if (run) {
                laterA5 = i;
                break;
            }
        }
    }

    Serial.print("STACKMAP base=0x");
    Serial.print((uint32_t)base, HEX);
    Serial.print(" sp=0x");
    Serial.print((uint32_t)sp, HEX);
    Serial.print(" spoff=");

    if (sp >= base && sp <= base + stackBytes)
        Serial.print((uint32_t)(sp - base));
    else
        Serial.print("OUT");

    Serial.print(" hwm=");
    Serial.print((uint32_t)lp->usStackHighWaterMark);
    Serial.print("w prefix=");
    Serial.print(prefix);
    Serial.print("B laterA5=");

    if (laterA5 < stackBytes)
        Serial.print(laterA5);
    else
        Serial.print("none");

    Serial.println();

    // Dump the low 128 bytes: this is the region that determines whether
    // HWM=0 is true contiguous stack use or isolated corruption at the base.
    for (uint32_t off = 0; off < 128u; off += 16u) {
        Serial.print("  +");
        if (off < 0x10) Serial.print("00");
        else if (off < 0x100) Serial.print("0");
        Serial.print(off, HEX);
        Serial.print(":");

        for (uint32_t j = 0; j < 16u; ++j) {
            uint8_t v = base[off + j];
            Serial.print(' ');
            if (v < 0x10) Serial.print('0');
            Serial.print(v, HEX);
        }
        Serial.println();
    }
}


uint8_t faultDiagReason()
{
	return g_reason;
}
uint32_t faultDiagResetReas()
{
	return g_resetReas;
}
const char *faultDiagReasonStr()
{
	return REASON_STR[g_reason < RR_COUNT ? g_reason : 0];
}
