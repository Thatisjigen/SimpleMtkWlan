#!/usr/bin/env python3
"""LQM updates to WCL on Tahoe: the beacon snapshot and how often it goes out.

Compiles the real AirportWCL::makeBeaconUpdate() and lqmBeaconUpdateDue()
(WCL.hpp): an update carries the beacons received since the last one and
goes out at most every 5 s, the first of a join at once. The beacon path
checks every 8 beacons, so an update is never later than one such step
after 5 s, and never later than before when beacons are sparse. In WCL.cpp
the beacon path posts only when an update is due and records when, a join
and a leave start over, the Sequoia update and the link quality keep their
cadence.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
wcl = (root / 'AirportMtkWlan/WCL/WCL.cpp').read_text()

beacon = wcl[wcl.index('case IEEE80211_EVT_BEACON: {'):]
beacon = beacon[:beacon.index('publishWCLBeacon(')]
gate = beacon.index('if (portAuthorized && (wclLqmBeacons % 8 == 0)) {')
sequoia = beacon[beacon.index('#if __IO80211_TARGET == __MAC_15_2', gate):beacon.index('#else', gate)]
tahoe = beacon[beacon.index('#else', gate):beacon.index('#endif', gate)]
assert 'lqmBeaconUpdateDue' not in sequoia and 'SequoiaLqmUpdate' in sequoia
assert 'AirportWCL::lqmBeaconUpdateDue(now, wclLqmReportedAt) &&' in tahoe
assert tahoe.index('lqmBeaconUpdateDue') < tahoe.index('postMessage') < tahoe.index('wclLqmReportedAt = now;')
assert beacon.index('#endif', gate) < beacon.index('updateWCLLinkQuality();', gate)
assert wcl.count('    wclAssociatedBeacons = 0;\n    wclLqmReportedAt = 0;\n') == 2
for name in ('IOReturn AirportItlwmSkywalkInterface::startWCLJoin(', 'IOReturn AirportItlwmSkywalkInterface::leaveWCLNetwork('):
    body = wcl[wcl.index(name):]
    body = body[:body.index('\n}\n')]
    assert 'wclLqmReportedAt = 0;' in body, name

test = r'''
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "WCL.hpp"
using namespace AirportWCL;

// Beacons every period_ns, the counter seen every 8 as the beacon path does; returns the posts and the longest gap.
static void run(uint64_t period_ns, uint32_t beacons, uint32_t &posts, uint64_t &longest, uint32_t &maxDelta)
{
    const uint64_t t0 = 100000000000ULL;
    uint64_t last = 0, previous = 0;
    uint32_t reported = 0;
    posts = 0; longest = 0; maxDelta = 0;
    for (uint32_t n = 1; n <= beacons; ++n) {
        if (n % 8)
            continue;
        const uint64_t now = t0 + uint64_t(n) * period_ns;
        LqmBeaconUpdate u;
        if (lqmBeaconUpdateDue(now, last) && makeBeaconUpdate(n - reported, u)) {
            if (posts) {
                longest = now - previous > longest ? now - previous : longest;
                maxDelta = u.receivedDelta > maxDelta ? u.receivedDelta : maxDelta;
            }
            ++posts; reported = n; last = previous = now;
        }
    }
}

int main()
{
    LqmBeaconUpdate u;
    assert(!makeBeaconUpdate(0, u));
    assert(makeBeaconUpdate(8, u) && u.receivedDelta == 8 && u.scheduledDelta == 0 &&
           u.beaconValid == 1 && u.snapshotValid == 1 && u.countersValid == 1);
    // The first update of a join goes out at once, then at most one every 5 s.
    const uint64_t s = 1000000000ULL;
    assert(lqmBeaconUpdateDue(1, 0));
    assert(!lqmBeaconUpdateDue(105 * s - 1, 100 * s) && lqmBeaconUpdateDue(105 * s, 100 * s));
    uint32_t posts, maxDelta; uint64_t longest;
    // Every beacon of a 100 TU access point, 10 minutes: before, an update every 8 beacons (0.8 s, 750 of them);
    // now one every 56 (5.7 s), far inside WCL's 60 s without beacons.
    run(102400000ULL, 6000, posts, longest, maxDelta);
    assert(posts == 108 && maxDelta == 56 && longest == 56 * 102400000ULL);
    // Only every 10th beacon delivered (a long DTIM in power save): every check posts, as before.
    run(1024000000ULL, 600, posts, longest, maxDelta);
    assert(posts == 75 && maxDelta == 8 && longest == 8 * 1024000000ULL);
    puts("PASS WCL LQM on Tahoe: beacon snapshot, at most one update every 5 s (every 56 beacons at 100 TU instead of 8), first of a join at once, never later than before with sparse beacons; Sequoia update and link quality untouched, join and leave start over");
}
'''

with tempfile.TemporaryDirectory(prefix='mtk-wcl-lqm-') as d:
    path = Path(d) / 'test.cpp'
    binary = Path(d) / 'test'
    path.write_text(test)
    subprocess.run(['clang++', '-std=c++14', '-fsanitize=undefined', '-fno-sanitize-recover=all', '-g',
                    '-I', str(root / 'AirportMtkWlan/WCL'), str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)
