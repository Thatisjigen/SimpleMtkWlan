"""Leaving a network without a sweep: the real mwx_leave_unasked() and mwx_scan_idle() under UBSan.

A leave with nothing asked for by macOS and nothing scanning enters the scan
state idle, as mwx_init() does; anything else (a join or a scan macOS asked
for, a background scan, a scan already running, a SCAN -> SCAN iteration)
still goes to mwx_scan(). The state task takes the idle path before it
starts a hardware scan.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
hal = (root / 'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()


def function(name):
    m = re.search(r'(?m)^(?:static )?(?:bool|void|int)\n' + name + r'\([^;]+?\)\s*\{', hal)
    assert m, name
    end, depth = hal.index('{', m.start()) + 1, 1
    while depth:
        depth += (hal[end] == '{') - (hal[end] == '}')
        end += 1
    return hal[m.start():end]


unasked = function('mwx_leave_unasked')
idle = function('mwx_scan_idle')
task = function('mwx_newstate_task')
# In the state task the idle path comes first in the SCAN case, and only in the native Wi-Fi build.
scan_case = task[task.index('\tswitch (nstate) {'):]
scan_case = scan_case[scan_case.index('\tcase IEEE80211_S_SCAN:'):]
assert scan_case.index('#ifdef AIRPORT_WCL\n\t\tif (mwx_leave_unasked(sc, ostate)) {\n'
                       '\t\t\tmwx_scan_idle(sc);\n\t\t\tbreak;\n\t\t}\n#endif\n') < scan_case.index('rv = mwx_scan(sc);')

test = r'''
#include <cassert>
#include <cstdio>
#include <initializer_list>
#define MWX_DEV_LOG(sc, ...) (++logs)
#define IEEE80211_S_INIT 0
#define IEEE80211_S_SCAN 1
#define IEEE80211_S_AUTH 2
#define IEEE80211_S_ASSOC 3
#define IEEE80211_S_RUN 4
#define MWX_FLAG_SCANNING 0x01
#define MWX_FLAG_BGSCAN 0x02
#define IEEE80211_F_BGSCAN 0x100
#define IEEE80211_MODE_AUTO 0
#define IEEE80211_MODE_11AC 5
#define IFM_AUTO 0
#define IFM_MODE(m) (m)
#define LINK_STATE_DOWN 2
typedef int ieee80211_state;
struct ifnet {int unused;};
struct ieee80211_node {int unused;};
struct ifmedia_entry {int ifm_media;};
struct ifmedia {ifmedia_entry *ifm_cur;};
struct ieee80211com {ifnet ic_if; ifmedia ic_media; ieee80211_node *ic_bss; int ic_state;
    unsigned ic_flags; bool ic_wcl_scan_requested, ic_wcl_join_requested, ic_wcl_scan_idle;};
struct mwx_softc {ieee80211com sc_ic; unsigned sc_flags;};
static int logs, resets, cleanups, setmodes, link_state = -1, mode = -1;
static ieee80211_node *cleaned;
static void ieee80211_setmode(ieee80211com *, int m) {++setmodes; mode = m;}
static void ieee80211_reset_scan(ifnet *) {++resets;}
static void ieee80211_set_link_state(ieee80211com *, int s) {link_state = s;}
static void ieee80211_node_cleanup(ieee80211com *, ieee80211_node *ni) {++cleanups; cleaned = ni;}
#define enum
#define AIRPORT_WCL 1
''' + unasked + '\n' + idle + r'''
#undef enum

static void reset(mwx_softc &sc, ieee80211_node &bss, ifmedia_entry &media)
{
    sc = mwx_softc();
    sc.sc_ic.ic_bss = &bss;
    sc.sc_ic.ic_media.ifm_cur = &media;
    sc.sc_ic.ic_state = IEEE80211_S_RUN;
    logs = resets = cleanups = setmodes = 0; link_state = mode = -1; cleaned = nullptr;
}

int main()
{
    mwx_softc sc; ieee80211_node bss; ifmedia_entry media = {IFM_AUTO};
    reset(sc, bss, media);
    // A leave from RUN, AUTH or ASSOC with nothing asked for and nothing scanning: idle.
    for (int ostate : {IEEE80211_S_RUN, IEEE80211_S_ASSOC, IEEE80211_S_AUTH})
        assert(mwx_leave_unasked(&sc, ostate));
    // A SCAN -> SCAN iteration and INIT -> SCAN keep scanning.
    assert(!mwx_leave_unasked(&sc, IEEE80211_S_SCAN));
    assert(!mwx_leave_unasked(&sc, IEEE80211_S_INIT));
    // Anything asked for or running keeps the hardware scan.
    sc.sc_ic.ic_wcl_join_requested = true; assert(!mwx_leave_unasked(&sc, IEEE80211_S_RUN));
    sc.sc_ic.ic_wcl_join_requested = false; sc.sc_ic.ic_wcl_scan_requested = true;
    assert(!mwx_leave_unasked(&sc, IEEE80211_S_RUN));
    sc.sc_ic.ic_wcl_scan_requested = false;
    for (unsigned f : {MWX_FLAG_SCANNING, MWX_FLAG_BGSCAN}) {
        sc.sc_flags = f; assert(!mwx_leave_unasked(&sc, IEEE80211_S_RUN));
    }
    sc.sc_flags = 0; sc.sc_ic.ic_flags = IEEE80211_F_BGSCAN; assert(!mwx_leave_unasked(&sc, IEEE80211_S_RUN));

    // The idle state: scan state, fresh channel list, link down, BSS node cleaned, no mode change on auto media.
    reset(sc, bss, media);
    mwx_scan_idle(&sc);
    assert(sc.sc_ic.ic_state == IEEE80211_S_SCAN && sc.sc_ic.ic_wcl_scan_idle);
    assert(resets == 1 && link_state == LINK_STATE_DOWN && cleanups == 1 && cleaned == &bss);
    assert(setmodes == 0 && logs == 1 && sc.sc_flags == 0);
    // A mode fixed during the association goes back to auto, as mwx_scan() does.
    media.ifm_media = IEEE80211_MODE_11AC; reset(sc, bss, media);
    mwx_scan_idle(&sc);
    assert(setmodes == 1 && mode == IEEE80211_MODE_AUTO && resets == 1);
    puts("PASS leave without a sweep: idle only for a leave with nothing asked for and nothing scanning; scan state, fresh channel list, link down, BSS cleaned; taken before mwx_scan() in the state task");
}
'''

with tempfile.TemporaryDirectory(prefix='mtk-leave-idle-') as d:
    path = Path(d) / 'test.cpp'
    binary = Path(d) / 'test'
    path.write_text(test)
    subprocess.run(['clang++', '-std=c++14', '-fsanitize=undefined', '-fno-sanitize-recover=all', '-g',
                    str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)
