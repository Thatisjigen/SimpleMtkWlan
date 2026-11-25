"""Exercise the real background-scan admission function with a fake MCU/clock."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()
start = source.index('int\nmwx_bgscan(struct ieee80211com *ic)')
function = source[start:source.index('\nvoid\nmwx_bgscan_done(', start)]
pre = r'''
#include <cstdint>
#include <cerrno>
#include <cassert>
#include "WCL.hpp"
#define IEEE80211_S_RUN 4
#define IEEE80211_CHAN_MAX 255
#define MWX_FLAG_SCANNING 1
#define MWX_FLAG_BGSCAN 2
#define DEVNAME(sc) "test"
#define printf(...) ((void)0)
struct ieee80211com { void *ic_softc; int ic_state;
 bool ic_wcl_scan_requested, ic_wcl_scan_restricted;
 uint8_t ic_wcl_scan_channels[32]; struct { unsigned ic_flags; } ic_channels[256]; };
struct mwx_softc { uint64_t sc_last_bgscan_ns; unsigned sc_flags; int calls; int error; uint64_t sc_last_bgscan_band_ns[2]; };
static uint64_t clock_ns;
static uint64_t mach_absolute_time() { return clock_ns; }
static void absolutetime_to_nanoseconds(uint64_t n, uint64_t *out) { *out=n; }
static int mt7921_mcu_hw_scan(mwx_softc *sc, int bg) {
 assert(bg==1); ++sc->calls;
 if (!sc->error) sc->sc_flags |= MWX_FLAG_SCANNING;
 return sc->error;
}
'''
post = r'''
static constexpr uint64_t second=1000000000ULL;
// The card's channels; a restricted scan of macOS asks for some of them.
static void card(ieee80211com &ic) {
 for (unsigned ch = 1; ch <= 14; ++ch) ic.ic_channels[ch].ic_flags = 1;
 for (unsigned ch : {36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
                     149, 153, 157, 161, 165}) ic.ic_channels[ch].ic_flags = 1;
}
static void ask(ieee80211com &ic, unsigned first, unsigned last) {
 memset(ic.ic_wcl_scan_channels, 0, sizeof(ic.ic_wcl_scan_channels));
 for (unsigned ch = first; ch <= last; ++ch) ic.ic_wcl_scan_channels[ch / 8] |= 1u << (ch % 8);
}
int main() {
 mwx_softc a={}, b={};
 ieee80211com ia={&a,IEEE80211_S_RUN,true,true};
 ieee80211com ib={&b,IEEE80211_S_RUN,true,true};
 card(ia); card(ib); ask(ia, 1, 14); ask(ib, 1, 14);
 // Allocation or MCU send failures do not consume admission quota.
 clock_ns=200*second;
 for (int error : {ENOMEM,EIO}) {
  a.error=error; assert(mwx_bgscan(&ia)==error);
  assert(a.sc_last_bgscan_ns==0 && a.sc_flags==0);
 }
 a.error=0; assert(mwx_bgscan(&ia)==0 && a.calls==3);
 assert(a.sc_last_bgscan_ns==clock_ns);
 // Duplicate calls while hardware is already scanning do not resubmit.
 assert(mwx_bgscan(&ia)==0 && a.calls==3);
 a.sc_flags=0;
 // Another adapter's first request is independent of this adapter's quota.
 assert(mwx_bgscan(&ib)==0 && b.calls==1);
#ifdef AIRPORT_WCL
 clock_ns=210*second-1;
 assert(mwx_bgscan(&ia)==EBUSY && a.calls==3);
 clock_ns=210*second;
 assert(mwx_bgscan(&ia)==0 && a.calls==4);
 a.sc_flags=0;
 // Full sweeps retain the 120 second protection after a short scan.
 ia.ic_wcl_scan_restricted=false;
 clock_ns=330*second-1;
 assert(mwx_bgscan(&ia)==EBUSY && a.calls==4);
 clock_ns=330*second;
 assert(mwx_bgscan(&ia)==0 && a.calls==5);
 a.sc_flags=0;
 ia.ic_wcl_scan_restricted=true;
 clock_ns=340*second;
 a.error=EIO;
 assert(mwx_bgscan(&ia)==EIO && a.sc_last_bgscan_ns==330*second);
 a.error=0;
 assert(mwx_bgscan(&ia)==0 && a.calls==7);
 a.sc_flags=0;
 // macOS asks for 5 GHz as soon as its 2.4 GHz scan answered: the 10 s count
 // per band, so that half is not refused as busy.
 clock_ns=343*second; ask(ia, 36, 165);
 assert(mwx_bgscan(&ia)==0 && a.calls==8 && a.sc_last_bgscan_band_ns[1]==343*second);
 a.sc_flags=0;
 // Each band waits 10 s after its own last scan; a scan of both waits for both.
 clock_ns=350*second-1; ask(ia, 1, 14); assert(mwx_bgscan(&ia)==EBUSY && a.calls==8);
 clock_ns=350*second; assert(mwx_bgscan(&ia)==0 && a.calls==9);
 a.sc_flags=0;
 clock_ns=353*second-1; ask(ia, 36, 48); assert(mwx_bgscan(&ia)==EBUSY && a.calls==9);
 clock_ns=353*second; assert(mwx_bgscan(&ia)==0 && a.calls==10);
 a.sc_flags=0;
 clock_ns=362*second; ask(ia, 1, 165); assert(mwx_bgscan(&ia)==EBUSY && a.calls==10);
 clock_ns=363*second; assert(mwx_bgscan(&ia)==0 && a.calls==11);
 assert(a.sc_last_bgscan_band_ns[0]==363*second && a.sc_last_bgscan_band_ns[1]==363*second);
 a.sc_flags=0;
 // A channel the card does not have does not count: 2.4 GHz only.
 clock_ns=374*second; ask(ia, 1, 14); ia.ic_wcl_scan_channels[140 / 8] |= 1u << (140 % 8);
 ia.ic_channels[140].ic_flags = 0;
 assert(mwx_bgscan(&ia)==0 && a.calls==12 && a.sc_last_bgscan_band_ns[1]==363*second);
 a.sc_flags=0;
 // Full sweeps still wait 120 s after any scan.
 ia.ic_wcl_scan_restricted=false;
 clock_ns=494*second-1; assert(mwx_bgscan(&ia)==EBUSY && a.calls==12);
 clock_ns=494*second; assert(mwx_bgscan(&ia)==0 && a.calls==13);
 ia.ic_wcl_scan_restricted=true;
#else
 clock_ns=320*second-1;
 assert(mwx_bgscan(&ia)==EBUSY && a.calls==3);
 clock_ns=320*second;
 assert(mwx_bgscan(&ia)==0 && a.calls==4);
#endif
 // Unassociated scans are not throttled and don't overwrite the quota.
 a.sc_flags=0; ia.ic_state=0; clock_ns+=second;
 uint64_t saved=a.sc_last_bgscan_ns;
 assert(mwx_bgscan(&ia)==0 && a.sc_last_bgscan_ns==saved);
}
'''
# The development Hackintosh cannot execute ASan binaries; CI may opt in.
sanitizers = os.environ.get('MTK_TEST_SANITIZERS', 'undefined')
with tempfile.TemporaryDirectory(prefix='mtk-bgscan-') as directory:
    path = Path(directory) / 'test.cpp'
    path.write_text('#include <initializer_list>\n#include <cstring>\n' + pre + function + post)
    for wcl in (False, True):
        binary = Path(directory) / ('wcl' if wcl else 'simple')
        command = ['clang++', '-std=c++14', '-fsanitize=' + sanitizers, '-I', str(root / 'AirportMtkWlan/WCL')]
        if wcl:
            command.append('-DAIRPORT_WCL')
        subprocess.run(command + [str(path), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS real bgscan failure retry, success throttling (10 s per band for macOS restricted scans, 120 s for full sweeps), adapter isolation and non-RUN admission (WCL + Simple)')
