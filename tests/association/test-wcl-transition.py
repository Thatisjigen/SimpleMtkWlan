#!/usr/bin/env python3
"""WPA2/WPA3 transition networks as the front end shows them to macOS.

Compiles the real AirportWCL::selectWPA2TransitionMode() (WCL.hpp): a network
that offers plain PSK beside SAE (and PSK-SHA256), with 802.11w capable but
not required, is shown with the PSK suite alone and without its RSN
Extension element; everything else is left exactly as it is, and the list is
never read or written past its end.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]

test = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "WCL.hpp"
using AirportWCL::selectWPA2TransitionMode;
using Bytes = std::vector<uint8_t>;

// One RSN element: version 1, CCMP group and pairwise, the given AKM suite types, RSN capabilities.
static Bytes rsn(std::initializer_list<uint8_t> akms, uint16_t caps) {
    Bytes e = {48, 0, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, uint8_t(akms.size()), 0};
    for (uint8_t a : akms) { e.push_back(0); e.push_back(0x0f); e.push_back(0xac); e.push_back(a); }
    e.push_back(uint8_t(caps)); e.push_back(uint8_t(caps >> 8));
    e[1] = uint8_t(e.size() - 2);
    return e;
}
static Bytes join(std::initializer_list<Bytes> parts) {
    Bytes out;
    for (const auto &p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}
// Runs the conversion; guard bytes after the list are never read as elements nor written.
static Bytes shown(Bytes ies) {
    ies.resize(ies.size() + 8, 0xee);
    const size_t length = selectWPA2TransitionMode(ies.data(), ies.size() - 8);
    for (size_t i = ies.size() - 8; i < ies.size(); ++i) assert(ies[i] == 0xee);
    ies.resize(length);
    return ies;
}
static const Bytes ssid = {0, 3, 'n', 'e', 't'};
static const Bytes rsnxe = {244, 1, 0x20};
static const Bytes vendor = {221, 4, 0x00, 0x50, 0xf2, 2};

int main() {
    const auto psk = rsn({2}, 0x0080);
    // PSK, PSK-SHA256 and SAE, 802.11w capable: shown as plain PSK.
    assert(shown(join({ssid, rsn({2, 6, 8}, 0x0080), vendor})) == join({ssid, psk, vendor}));
    // PSK and SAE, as before.
    assert(shown(join({ssid, rsn({2, 8}, 0x0080)})) == join({ssid, psk}));
    // SAE with the group-dependent hash (24) counts as SAE.
    assert(shown(join({ssid, rsn({2, 24}, 0x0080)})) == join({ssid, psk}));
    // After a conversion the RSN Extension element goes too, wherever it is; other elements stay.
    assert(shown(join({ssid, rsnxe, rsn({2, 6, 8}, 0x0080), rsnxe, vendor})) == join({ssid, psk, vendor}));
    // Left exactly as they are:
    for (const auto &ies : {
             join({ssid, rsn({2, 8}, 0x00c0), rsnxe}),   // 802.11w required: WPA3 only in practice
             join({ssid, rsn({8}, 0x0080), rsnxe}),      // SAE only
             join({ssid, rsn({6, 8}, 0x0080)}),          // no plain PSK to fall back on
             join({ssid, rsn({2, 6}, 0x0080)}),          // no SAE: not a transition network
             join({ssid, rsn({2, 1, 8}, 0x0080)}),       // 802.1X mixed in
             join({ssid, rsn({2}, 0x0000), rsnxe}),      // plain WPA2: its RSN Extension element stays
             join({ssid, vendor}),                       // open
         })
        assert(shown(ies) == ies);
    // A malformed element list is returned unchanged.
    auto broken = join({ssid, rsn({2, 6, 8}, 0x0080)});
    broken[1] = 40;
    assert(shown(broken) == broken);
    puts("PASS WCL WPA2/WPA3 transition networks: PSK alone shown also beside PSK-SHA256, SAE-EXT counted as SAE, RSN Extension element removed after a conversion, other networks and malformed lists untouched");
}
'''

with tempfile.TemporaryDirectory(prefix='mtk-wcl-transition-') as d:
    path = Path(d) / 'test.cpp'
    binary = Path(d) / 'test'
    path.write_text(test)
    subprocess.run(['clang++', '-std=c++14', '-fsanitize=undefined', '-fno-sanitize-recover=all', '-g',
                    '-I', str(root / 'AirportMtkWlan/WCL'), str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)
