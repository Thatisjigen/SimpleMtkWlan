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

// ---------------------------------------------------------------------------------------------------------------------
// The AWDL radio, implemented by the MediaTek hardware layer (SimpleMtkWlan/hal_mwx/MtkMwx.cpp) for the AWDL interface.
//
// The interface keeps the AWDL timeline and decides where the radio has to be; the hardware layer only executes it. A
// window puts the radio on an AWDL channel from startUs for dwellMs: on the station's own channel nothing moves and only
// the reception of other devices' frames is opened, anywhere else the firmware is asked for a remain-on-channel period.
// While a window is open, the frames queued for its channel are sent and the AWDL frames heard are kept for the
// interface. All times are host uptime in microseconds (mtkAwdlUptimeUs).
// ---------------------------------------------------------------------------------------------------------------------
struct MtkAwdlWindow {
    uint8_t channel;        // primary channel; 20 MHz, unless it is the station's own channel
    uint8_t reserved;
    uint16_t dwellMs;       // how long the radio stays, from startUs
    uint64_t startUs;       // when the window has to be open; 0 = as soon as possible
};

uint64_t mtkAwdlUptimeUs();
// Allocates the radio's queues and its thread, and remembers awdl0's address. 0, or an errno.
int mtkAwdlRadioStart(const uint8_t *address);
// No more windows: the window in progress ends at its time and the radio goes back to the station's settings.
void mtkAwdlRadioStop();
// Replaces the next window. Called ahead of the window's start, at most one window ahead.
int mtkAwdlRadioWindow(const MtkAwdlWindow *window);
// Queues a complete 802.11 frame for the next window on `channel` (0: the next window, whatever its channel). A frame
// not sent within maxAgeMs is dropped. ENOBUFS when the queue is full: the caller keeps the frame and asks again later.
int mtkAwdlRadioQueue(const uint8_t *frame, uint32_t length, uint8_t channel, uint32_t maxAgeMs);
// Free places in that queue.
uint32_t mtkAwdlRadioRoom();
// True for two minutes after the firmware was reloaded for a reception stall: the Family's channel sequence and new
// templates then do not start the timeline, only a frame or traffic request, a peer or data do.
bool mtkAwdlRadioResting();
// The station's channel while it is associated, else 0.
uint8_t mtkAwdlRadioHomeChannel();
// True when the radio may use `channel` (in the chip's channel list, not passive).
bool mtkAwdlRadioChannelAllowed(uint8_t channel);
// The station's data frames with its access point, either way, counted since the radio was started: packets and their
// bytes (an A-MSDU counts once). The interface rates the access point's traffic from them.
void mtkAwdlRadioStationTraffic(uint64_t *packets, uint64_t *bytes);
// Counters for the driver log.
void mtkAwdlRadioSummary(char *buffer, size_t length);
// Windows, AWDL frames and other frames heard, by social channel (6, 44, 149, others).
void mtkAwdlRadioChannels(char *buffer, size_t length);

// Block Ack agreements with the peers, for the driver log.
void mtkAwdlRadioBlockAck(char *buffer, size_t length);

// How a received data frame arrived: whole, or as one subframe of an A-MSDU the hardware split, which it does for a peer
// with a station record (the receive descriptor's payload format: first 3, middle 2, last 1). The radio removes the two
// alignment bytes the hardware leaves after the 802.11 header, so a split subframe reads as an A-MSDU of one subframe;
// all the subframes of one A-MSDU carry its sequence number.
enum : uint8_t { kMtkAwdlWhole = 0, kMtkAwdlLastPart = 1, kMtkAwdlMiddlePart = 2, kMtkAwdlFirstPart = 3 };

// Provided by the AWDL interface. Both are called on the main work loop with its command gate held, never with a lock
// of the radio held, and must not wait.
// Right before a frame leaves: stamps the fields that only hold at that moment (the synchronization parameters'
// counters and the action header's transmit times).
void mtkAwdlStampFrame(uint8_t *frame, uint32_t length, uint64_t nowUs);
// One AWDL frame heard (an action frame or a data frame), 802.11 header first, no FCS. rssi is in dBm, 0 when the
// hardware gave none; part is one of kMtkAwdl*Part for a data frame. The bytes are only valid during the call.
void mtkAwdlInput(const uint8_t *frame, uint32_t length, uint32_t channel, int32_t rssi, uint64_t timeUs, uint8_t part);
// Room was made in the frame queue after it had been full: the interface can hand over more.
void mtkAwdlRadioRoomFreed();
