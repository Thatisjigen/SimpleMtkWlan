//
//  MtkLog.hpp
//  SimpleMtkWlan
//
//  Persistent driver log shared by SimpleMtkWlan and AirportMtkWlan.
//

#ifndef MtkLog_hpp
#define MtkLog_hpp

#include <IOKit/IOService.h>

extern "C" void mtk_dbg_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/*
 * While on, each line also goes to the "MtkStallLog" property, which keeps the reports of reception stalls until the
 * next boot: the ring of "MtkDebugLog" holds about ten minutes while AWDL runs.
 */
extern "C" void mtk_dbg_keep(int on);

/* Where mtk_dbg_publish() puts the "MtkDebugLog" property (the PCI device). */
void mtk_dbg_set_nub(IOService *nub);
void mtk_dbg_publish();

#endif /* MtkLog_hpp */
