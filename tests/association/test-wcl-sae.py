#!/usr/bin/env python3
"""WPA3-Personal (SAE) joins as the front end reads them from a WCL request.

Compiles the real AirportWCL helpers (WCL.hpp) and checks how a join request
is classified (SAE with the password itself, cipher 10, alone and with a
CCMP group cipher; WPA2 with a passphrase or a PMK unchanged), that the
enlarged key field of a request decodes, and that only an RSN Extension
element with the hash-to-element bit lets macOS see a network's SAE AKM.
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
using AirportWCL::LegacyAuth;
using AirportWCL::classifyLegacyAuth;
using AirportWCL::JoinRequest;

// A minimal RSN element: CCMP (or TKIP) group cipher, CCMP pairwise, SAE.
static void makeRSN(JoinRequest &j, bool ccmp = true) {
    const uint8_t ie[] = {48, 20, 1, 0, 0, 0x0f, 0xac, uint8_t(ccmp ? 4 : 2),
                          1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 8, 0x00, 0x80};
    static_assert(sizeof(ie) == 22, "RSN element");
    memcpy(j.rsn, ie, sizeof(ie));
    j.rsnLength = sizeof(ie);
}

static JoinRequest base() {
    JoinRequest j = {};
    j.lowerAuth = 1;
    j.ssidLength = 3;
    memcpy(j.ssid, "net", 3);
    makeRSN(j);
    return j;
}

static JoinRequest sae(uint32_t cipher, uint32_t length) {
    JoinRequest j = base();
    j.upperAuth = 1u << 12;
    j.keyCipher = cipher;
    j.keyLength = length;
    memset(j.key, 'a', length < sizeof(j.key) ? length : sizeof(j.key));
    return j;
}

int main() {
    // SAE: the password itself, cipher 10, from 1 to 63 bytes.
    assert(classifyLegacyAuth(sae(10, 8)) == LegacyAuth::PersonalSAE);
    assert(classifyLegacyAuth(sae(10, 1)) == LegacyAuth::PersonalSAE);
    assert(classifyLegacyAuth(sae(10, 63)) == LegacyAuth::PersonalSAE);
    assert(classifyLegacyAuth(sae(10, 0)) == LegacyAuth::Invalid);
    assert(classifyLegacyAuth(sae(10, 64)) == LegacyAuth::Invalid);
    // ...not a PMK, and never with a TKIP group cipher or a WPA element.
    assert(classifyLegacyAuth(sae(6, 32)) == LegacyAuth::Invalid);
    {
        JoinRequest j = sae(10, 8);
        makeRSN(j, false);
        assert(classifyLegacyAuth(j) == LegacyAuth::Invalid);
        const uint8_t wpa[] = {221, 22, 0, 0x50, 0xf2, 1, 1, 0, 0, 0x50, 0xf2, 4, 1, 0, 0, 0x50, 0xf2, 4, 1, 0, 0, 0x50, 0xf2, 2};
        memcpy(j.rsn, wpa, sizeof(wpa));
        j.rsnLength = sizeof(wpa);
        assert(classifyLegacyAuth(j) == LegacyAuth::Invalid);
        j.rsnLength = 0;
        assert(classifyLegacyAuth(j) == LegacyAuth::PersonalSAE);
    }
    // SAE comes alone: mixed with a WPA2 type it is not a join WCL sends.
    {
        JoinRequest j = sae(10, 8);
        j.upperAuth |= 1u << 1;
        assert(classifyLegacyAuth(j) == LegacyAuth::Invalid);
        j.upperAuth = (1u << 12) | (1u << 13);
        assert(classifyLegacyAuth(j) == LegacyAuth::Invalid);
    }
    // WPA2 as before: a PMK, or a passphrase of printable characters up to 63.
    {
        JoinRequest j = base();
        j.upperAuth = 1u << 1;
        j.keyCipher = 6;
        j.keyLength = 32;
        assert(classifyLegacyAuth(j) == LegacyAuth::Personal);
        j.keyCipher = 10;
        j.keyLength = 8;
        memcpy(j.key, "password", 8);
        assert(AirportWCL::isPassphrase(j));
        assert(classifyLegacyAuth(j) == LegacyAuth::Personal);
        j.keyLength = 63;
        memset(j.key, 'p', 63);
        assert(classifyLegacyAuth(j) == LegacyAuth::Personal && AirportWCL::isPassphrase(j));
        memcpy(j.key, "pass\0ord", 8);
        j.keyLength = 8;
        assert(classifyLegacyAuth(j) == LegacyAuth::Invalid);
        j = base();
        j.upperAuth = 1u << 0;
        assert(classifyLegacyAuth(j) == LegacyAuth::Enterprise);
    }
    assert(strcmp(AirportWCL::legacyAuthName(LegacyAuth::PersonalSAE), "personal SAE") == 0);

    // The request's key field holds 64 bytes: a 63-byte password decodes.
    {
        std::vector<uint8_t> p(0x218 + 18, 0);
        const uint32_t count = 1, ssidLength = 3, keyLength = 63;
        memcpy(&p[0x214], &count, 4);
        memcpy(&p[0x1c], &ssidLength, 4);
        memcpy(&p[0x44], &keyLength, 4);
        memcpy(&p[0x20], "net", 3);
        memset(&p[0x50], 'a', keyLength);
        p[0x21c] = 0x02;
        JoinRequest j;
        assert(AirportWCL::decodeJoin(p.data(), p.size(), j));
        assert(j.keyLength == keyLength && memcmp(j.key, p.data() + 0x50, keyLength) == 0);
        const uint32_t tooLong = 65;
        memcpy(&p[0x44], &tooLong, 4);
        assert(!AirportWCL::decodeJoin(p.data(), p.size(), j));
    }

    // Only an RSN Extension element (244) with the hash-to-element bit shows macOS the SAE AKM.
    {
        std::vector<uint8_t> ies = {0, 3, 'n', 'e', 't'};
        assert(!AirportWCL::advertisesSAEH2E(ies.data(), ies.size()));
        std::vector<uint8_t> hnp = ies;
        hnp.insert(hnp.end(), {244, 1, 0x00});
        assert(!AirportWCL::advertisesSAEH2E(hnp.data(), hnp.size()));
        std::vector<uint8_t> h2e = ies;
        h2e.insert(h2e.end(), {244, 1, 0x20});
        assert(AirportWCL::advertisesSAEH2E(h2e.data(), h2e.size()));
        std::vector<uint8_t> empty = ies;
        empty.insert(empty.end(), {244, 0});
        assert(!AirportWCL::advertisesSAEH2E(empty.data(), empty.size()));
        std::vector<uint8_t> cut = ies;
        cut.insert(cut.end(), {244, 2, 0x20});
        assert(!AirportWCL::advertisesSAEH2E(cut.data(), cut.size()));
    }
    puts("PASS WCL WPA3-Personal: SAE requests (password, cipher 10, 1-63 bytes, alone, CCMP group) classified, WPA2 and enterprise as before, 64-byte key field decoded, SAE AKM shown only with hash-to-element");
}
'''

with tempfile.TemporaryDirectory(prefix='mtk-wcl-sae-') as d:
    path = Path(d) / 'test.cpp'
    binary = Path(d) / 'test'
    path.write_text(test)
    subprocess.run(['clang++', '-std=c++14', '-fsanitize=undefined', '-fno-sanitize-recover=all', '-g',
                    '-I', str(root / 'AirportMtkWlan/WCL'), str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)
