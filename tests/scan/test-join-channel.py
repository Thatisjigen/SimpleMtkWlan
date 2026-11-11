"""The first scan of a join listens only on the channel macOS named.

Compiles the real mt7921_mcu_hw_scan() of the native Wi-Fi build
(AIRPORT_WCL) with the real AirportWCL helpers against stubs, and checks
the scan request of a join, that its channel is used once, and that the
other scans keep their channels and dwell times.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()
header = (root / 'SimpleMtkWlan/hal_mwx/if_mwxreg.h').read_text()


def block(text, pattern):
    match = re.search(pattern, text, re.M | re.S)
    assert match, pattern
    return match.group(0)


structs = '\n'.join(block(header, r'^struct %s \{.*?^\} __packed;' % name)
                    for name in ('mt76_connac_mcu_scan_ssid',
                                 'mt76_connac_mcu_scan_channel',
                                 'mt76_connac_hw_scan_req'))
scan = block(source, r'^int\nmt7921_mcu_hw_scan\(.*?^\}\n')

pre = r'''
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include "WCL.hpp"
#define AIRPORT_WCL 1
#define __packed __attribute__((packed))
#define IEEE80211_NWID_LEN 32
#define IEEE80211_ADDR_LEN 6
#define ETHER_ADDR_LEN 6
#define IEEE80211_CHAN_MAX 255
#define MT76_HW_SCAN_IE_LEN 600
#define SCAN_FUNC_SPLIT_SCAN 0x20
#define SCAN_FUNC_RANDOM_MAC 0x1
#define MWX_FLAG_SCANNING 1
#define MWX_FLAG_BGSCAN 2
#define MCU_CE_CMD_START_HW_SCAN 1
#define htole16(x) (x)
#define htole32(x) (x)
#define le16toh(x) (x)
#define le32toh(x) (x)
#define MWX_DEV_LOG(...) ((void)0)
'''

stubs = r'''
struct ieee80211_channel { unsigned ic_flags, ic_freq; };
struct ieee80211com {
 ieee80211_channel ic_channels[256];
 bool ic_wcl_scan_requested, ic_wcl_scan_active, ic_wcl_scan_restricted;
 bool ic_wcl_join_requested;
 uint8_t ic_wcl_scan_ssid_length, ic_wcl_scan_ssid[32], ic_wcl_scan_channels[32];
 uint8_t ic_wcl_join_channel;
};
struct mwx_softc { ieee80211com sc_ic; unsigned sc_flags; uint8_t sc_scan_seq_num; };
struct mbuf { unsigned char bytes[sizeof(mt76_connac_hw_scan_req)]; };
#define mtod(m, T) reinterpret_cast<T>((m)->bytes)
#define IEEE80211_IS_CHAN_2GHZ(c) ((c)->ic_freq < 3000)
static unsigned ieee80211_mhz2ieee(unsigned freq, int)
{ return freq < 3000 ? (freq == 2484 ? 14 : (freq - 2407) / 5) : (freq - 5000) / 5; }
static mt76_connac_hw_scan_req captured;
static mbuf storage;
static mbuf *mwx_mcu_alloc_msg(size_t n) { memset(storage.bytes, 0xbd, n); return &storage; }
static int mwx_mcu_send_mbuf(mwx_softc *, unsigned, mbuf *m, int *)
{ memcpy(&captured, m->bytes, sizeof(captured)); return 0; }
'''

post = r'''
static void check(bool ok, const char *reason)
{ if (!ok) { fprintf(stderr, "FAIL: %s\n", reason); exit(1); } }

// The card's 41 channels: 1-14 on 2.4 GHz, 27 on 5 GHz.
static const unsigned five[] = {36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120,
 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173};

static void card(mwx_softc &sc)
{
 memset(&sc, 0, sizeof(sc));
 for (unsigned ch = 1; ch <= 14; ch++)
  sc.sc_ic.ic_channels[ch] = {1, ch == 14 ? 2484u : 2407 + 5 * ch};
 for (unsigned ch : five)
  sc.sc_ic.ic_channels[ch] = {1, 5000 + 5 * ch};
}

static unsigned count() { return captured.channels_num + captured.ext_channels_num; }

int main()
{
 mwx_softc sc;

 // The first scan of a join: one channel, passive, two beacon intervals.
 card(sc);
 sc.sc_ic.ic_wcl_join_requested = true;
 sc.sc_ic.ic_wcl_join_channel = 56;
 check(mt7921_mcu_hw_scan(&sc, 0) == 0, "join scan return");
 check(count() == 1 && captured.channels[0].channel_num == 56 && captured.channels[0].band == 2,
       "join scan channel");
 check(captured.channel_dwell_time == 220 && captured.channel_min_dwell_time == 220 &&
       captured.timeout_value == 220, "join scan dwell");
 check(captured.channel_type == 4 && captured.scan_type == 0 && captured.ssids_num == 0,
       "join scan stays passive");
 check(sc.sc_ic.ic_wcl_join_channel == 0, "join channel used once");

 // Not heard there: the next scan of the same join sweeps every channel.
 check(mt7921_mcu_hw_scan(&sc, 0) == 0, "fallback return");
 check(count() == 41 && captured.channels_num == 32 && captured.ext_channels_num == 9,
       "fallback sweeps every channel");
 check(captured.channel_dwell_time == 120 && captured.timeout_value == 41 * 120, "fallback dwell");

 // A 2.4 GHz network.
 card(sc);
 sc.sc_ic.ic_wcl_join_requested = true;
 sc.sc_ic.ic_wcl_join_channel = 11;
 check(mt7921_mcu_hw_scan(&sc, 0) == 0 && count() == 1 &&
       captured.channels[0].channel_num == 11 && captured.channels[0].band == 1,
       "2.4 GHz join scan");

 // A background scan in RUN neither uses nor spends the channel.
 card(sc);
 sc.sc_ic.ic_wcl_join_requested = true;
 sc.sc_ic.ic_wcl_join_channel = 56;
 check(mt7921_mcu_hw_scan(&sc, 1) == 0 && count() == 41 && captured.channel_dwell_time == 0,
       "background scan unchanged");
 check(sc.sc_ic.ic_wcl_join_channel == 56, "background scan keeps the join channel");

 // A channel left from an ended join does nothing without a join.
 card(sc);
 sc.sc_ic.ic_wcl_join_channel = 56;
 check(mt7921_mcu_hw_scan(&sc, 0) == 0 && count() == 41 && captured.channel_dwell_time == 120,
       "no join: every channel");
 check(sc.sc_ic.ic_wcl_join_channel == 56, "no join: channel not spent");

 // A scan macOS asked for keeps its own channels, probes and dwell.
 card(sc);
 sc.sc_ic.ic_wcl_scan_requested = true;
 sc.sc_ic.ic_wcl_scan_active = true;
 sc.sc_ic.ic_wcl_scan_restricted = true;
 sc.sc_ic.ic_wcl_scan_channels[36 / 8] |= 1u << (36 % 8);
 sc.sc_ic.ic_wcl_scan_channels[149 / 8] |= 1u << (149 % 8);
 check(mt7921_mcu_hw_scan(&sc, 0) == 0 && count() == 2 && captured.scan_type == 1 &&
       captured.channels[0].channel_num == 36 && captured.channels[1].channel_num == 149 &&
       captured.channel_dwell_time == 120, "macOS scan unchanged");

 // Chanspecs: 20 MHz as this driver reports them; anything else sweeps.
 struct { uint16_t spec; unsigned channel; } specs[] = {
  {0xd038, 56}, {0xd024, 36}, {0xd0c4, 196}, {0x1001, 1}, {0x100b, 11}, {0x100e, 14},
  {0x1000, 0}, {0x100f, 0}, {0xd000, 0}, {0xd0c8, 0}, {0xd023, 0}, {0xd82a, 0}, {0xe02a, 0},
  {0x1803, 0}, {0x0006, 0}, {0x5025, 0}, {0xc038, 0}};
 for (auto &s : specs)
  check(AirportWCL::joinChannel(s.spec) == s.channel, "chanspec decoding");

 printf("PASS join scan on the named channel, used once, sweep fallback, other scans unchanged, chanspecs\n");
}
'''

with tempfile.TemporaryDirectory(prefix='mtk-join-channel-') as tmp:
    path = Path(tmp) / 'test.cpp'
    path.write_text(pre + structs + '\n' + stubs + scan + post)
    binary = Path(tmp) / 'test'
    subprocess.run(['clang++', '-std=c++14', '-fsanitize=undefined',
                    '-I', str(root / 'AirportMtkWlan/WCL'), str(path), '-o', str(binary)],
                   check=True)
    subprocess.run([str(binary)], check=True)
