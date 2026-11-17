//
//  MtkLog.cpp
//  SimpleMtkWlan
//

#include "MtkLog.hpp"
#include <IOKit/IOLib.h>
#include <libkern/libkern.h>
#include <libkern/OSAtomic.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>

/*
 * Persistent debug log. Early boot messages are lost from the unified log
 * (no uuidtext for OpenCore-injected kexts, small msgbuf), so every message
 * is also kept in a ring buffer and published as the "MtkDebugLog" property
 * on the PCI provider, where it survives a failed probe/start.
 */
#define MTK_DBG_BUF_SIZE (128 * 1024)
static char gMtkDbgBuf[MTK_DBG_BUF_SIZE];
static size_t gMtkDbgLen;
static volatile SInt32 gMtkDbgLock;
static IOService *gMtkDbgNub;

/*
 * The lines logged while mtk_dbg_keep() is on, kept apart: the reports of a reception stall, which are seldom read
 * before the ring above has moved past them. Published as "MtkStallLog"; when full, the oldest half goes, as in the
 * ring.
 */
#define MTK_DBG_KEEP_SIZE (16 * 1024)
static char gMtkDbgKeep[MTK_DBG_KEEP_SIZE];
static size_t gMtkDbgKeepLen;
static uint32_t gMtkDbgKeepGen;
static volatile int gMtkDbgKeepOn;

static void mtkDbgLock()
{
    while (!OSCompareAndSwap(0, 1, &gMtkDbgLock)) {}
}

static void mtkDbgUnlock()
{
    OSCompareAndSwap(1, 0, &gMtkDbgLock);
}

extern "C" void mtk_dbg_log(const char *fmt, ...)
{
    char line[512];
    uint64_t ns;
    va_list ap;
    int prefix;
    size_t len;

    absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
    prefix = snprintf(line, sizeof(line), "[%llu.%03llu] ",
        ns / 1000000000ULL, (ns / 1000000ULL) % 1000ULL);
    va_start(ap, fmt);
    vsnprintf(line + prefix, sizeof(line) - prefix, fmt, ap);
    va_end(ap);

    len = strnlen(line, sizeof(line));
    mtkDbgLock();
    if (gMtkDbgLen + len >= MTK_DBG_BUF_SIZE) {
        size_t drop = MTK_DBG_BUF_SIZE / 2;
        memmove(gMtkDbgBuf, gMtkDbgBuf + drop, gMtkDbgLen - drop);
        gMtkDbgLen -= drop;
    }
    memcpy(gMtkDbgBuf + gMtkDbgLen, line, len);
    gMtkDbgLen += len;
    gMtkDbgBuf[gMtkDbgLen] = 0;
    if (gMtkDbgKeepOn) {
        if (gMtkDbgKeepLen + len >= MTK_DBG_KEEP_SIZE) {
            size_t drop = MTK_DBG_KEEP_SIZE / 2;
            memmove(gMtkDbgKeep, gMtkDbgKeep + drop, gMtkDbgKeepLen - drop);
            gMtkDbgKeepLen -= drop;
        }
        memcpy(gMtkDbgKeep + gMtkDbgKeepLen, line, len);
        gMtkDbgKeepLen += len;
        gMtkDbgKeep[gMtkDbgKeepLen] = 0;
        gMtkDbgKeepGen++;
    }
    mtkDbgUnlock();
}

extern "C" void mtk_dbg_keep(int on)
{
    gMtkDbgKeepOn = on != 0;
}

/*
 * The log holds raw kernel state (object pointers, interface addresses) that is
 * only useful while debugging an attach failure. Publishing it as an IORegistry
 * property makes it world-readable (any user, "ioreg -l"), which defeats KASLR,
 * so it is published only when asked for with the -mtkdbglog boot argument. AWDL
 * is on without an argument; a test that reads its counters asks for the log too.
 */
static bool mtkDbgPublishAllowed()
{
    static int allowed = -1;
    if (allowed < 0) {
        char flag[16];
        allowed = PE_parse_boot_argn("-mtkdbglog", flag, sizeof(flag)) ? 1 : 0;
    }
    return allowed > 0;
}

/* "MtkStallLog", only when something was added since it was last published. */
static void mtkDbgPublishKeep(IOService *nub)
{
    static uint32_t published;
    char *copy;
    size_t len;
    uint32_t gen;

    if (gMtkDbgKeepGen == published)
        return;
    copy = (char *)IOMalloc(MTK_DBG_KEEP_SIZE);
    if (copy == NULL)
        return;
    mtkDbgLock();
    len = gMtkDbgKeepLen;
    gen = gMtkDbgKeepGen;
    memcpy(copy, gMtkDbgKeep, len);
    mtkDbgUnlock();
    copy[len] = 0;
    OSString *str = OSString::withCString(copy);
    IOFree(copy, MTK_DBG_KEEP_SIZE);
    if (str != NULL) {
        nub->setProperty("MtkStallLog", str);
        str->release();
        published = gen;
    }
}

void mtk_dbg_publish()
{
    IOService *nub = gMtkDbgNub;
    char *copy;
    size_t len;

    if (nub == NULL || !mtkDbgPublishAllowed())
        return;
    copy = (char *)IOMalloc(MTK_DBG_BUF_SIZE);
    if (copy == NULL)
        return;
    mtkDbgLock();
    len = gMtkDbgLen;
    memcpy(copy, gMtkDbgBuf, len);
    mtkDbgUnlock();
    copy[len] = 0;
    OSString *str = OSString::withCString(copy);
    IOFree(copy, MTK_DBG_BUF_SIZE);
    if (str != NULL) {
        nub->setProperty("MtkDebugLog", str);
        str->release();
    }
    mtkDbgPublishKeep(nub);
}

void mtk_dbg_set_nub(IOService *nub)
{
    gMtkDbgNub = nub;
}
