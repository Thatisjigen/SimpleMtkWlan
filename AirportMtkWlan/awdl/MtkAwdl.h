// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (c) 2026 thanhnndev and contributors
// The AWDL virtual interface (awdl0), adapted from thanhnndev/mt7925-macos-wifi to this driver's Tahoe front end.
//
// The Family creates its peer-to-peer interfaces through APPLE80211_IOC_VIRTUAL_IF_CREATE (0x5e), with role 6 for AWDL, and
// the driver answers with the BSD name it created. MtkAwdlInterface.cpp is compiled on its own against slot-exact layouts of
// Apple's classes (io80211/), which are not the headers the rest of the front end uses; only these plain functions cross.
//
// Everything is behind the boot-arg -mtkawdl: without it the card reports no AWDL capability and the ioctl is refused.
#pragma once
#include <IOKit/IOReturn.h>
#include <stddef.h>
#include <stdint.h>

class IOService;

bool mtkAwdlEnabled();

// requestedMac may be all zero, then the address is derived from primaryMac. bsdName receives awdl0, or the name in use.
IOReturn mtkAwdlCreateInterface(IOService *controller, const uint8_t *requestedMac, const uint8_t *primaryMac,
                                char *bsdName, size_t bsdNameLen);
