#!/usr/bin/env python3
"""BIP on receive: production key selection, CMAC and AES.

With management frame protection the access point authenticates its
group-addressed Deauthentication, Disassociation and Action frames with a MIC
in a trailing Management MIC element, keyed by the IGTK. The production
ieee80211_decrypt() path (key lookup by the MMIE key id, ieee80211_bip_decap,
AES_CMAC_* and the AES behind it) runs on frames protected by a reference
written from the standard: a genuine frame is accepted once, and nothing an
attacker without the key can build is.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

bip = (ref.NET / 'ieee80211_crypto_bip.c').read_text()
crypto = (ref.NET / 'ieee80211_crypto.c').read_text()
code = (ref.unincluded(ref.CRYPTO / 'aes.h') + ref.unincluded(ref.CRYPTO / 'aes.c') +
        ref.unincluded(ref.CRYPTO / 'cmac.h') + ref.unincluded(ref.CRYPTO / 'cmac.c') +
        ref.between(bip, '/* BIP software crypto context */', 'mbuf_t\nieee80211_bip_encap(') +
        ref.extract(bip, 'ieee80211_bip_decap') + '\n' +
        ref.extract(crypto, 'ieee80211_get_rxkey') + '\n' + ref.extract(crypto, 'ieee80211_decrypt'))

h = bytes.fromhex
igtk = h('4ea9543e09cf2b1eca66ffc58bdecbcf')
other = h('000102030405060708090a0b0c0d0e0f')
ap, bcast = h('020000000001'), h('ffffffffffff')


def mgmt(subtype, body, fc1=0, sa=ap, da=bcast):
    return bytes([subtype, fc1, 0, 0]) + da + sa + ap + bytes([0x90, 0x12]) + body


deauth = mgmt(0xc0, h('0200'))
frames = {
    'deauth': ref.bip_protect(igtk, deauth, 4, 5),
    'higher': ref.bip_protect(igtk, deauth, 4, 6),
    'disassoc': ref.bip_protect(igtk, mgmt(0xa0, h('0800')), 4, 7),
    # the AAD ignores Retry, Power Management and More Data, set after the MIC was computed
    'flags': bytearray(ref.bip_protect(igtk, mgmt(0xc0, h('0300')), 4, 8)),
    # an Action frame long enough for several CMAC blocks, and one ending on a block boundary
    'action': ref.bip_protect(igtk, mgmt(0xd0, bytes([0, 4]) + bytes(range(60))), 4, 9),
    'aligned': ref.bip_protect(igtk, mgmt(0xd0, bytes(range(26))), 4, 10),
    'badmic': bytearray(ref.bip_protect(igtk, deauth, 4, 20)),
    'badbody': bytearray(ref.bip_protect(igtk, deauth, 4, 21)),
    'spoofed': bytearray(ref.bip_protect(igtk, deauth, 4, 22)),
    'wrongkey': ref.bip_protect(other, deauth, 4, 23),
    'otherid': ref.bip_protect(igtk, deauth, 5, 24),
    'badid': ref.bip_protect(igtk, deauth, 3, 25),
    'nommie': deauth,
    'short': deauth[:24] + ref.mmie(4, 26)[:17],
    'next': ref.bip_protect(igtk, deauth, 4, 27),
}
frames['flags'][1] |= 0x38
frames['badmic'][-1] ^= 1
frames['badbody'][24] ^= 4          # another reason code
frames['spoofed'][10 + 5] ^= 1      # another transmitter
assert (20 + len(frames['aligned']) - 24) % 16 == 0 and (20 + len(frames['action']) - 24) % 16 != 0

stubs = r'''
#define MIN(a,b) ((a)<(b)?(a):(b))
#define malloc(n,a,b) std::malloc(n)
#define free(p) std::free(p)
static mbuf_t none(ieee80211com*,mbuf_t,ieee80211_key*){assert(!"only BIP keys in this test");return nullptr;}
#define ieee80211_wep_decrypt none
#define ieee80211_tkip_decrypt none
#define ieee80211_ccmp_decrypt none
static u_int ieee80211_get_hdrlen(const ieee80211_frame*){return sizeof(ieee80211_frame);}
'''
main = '\n'.join('static const std::vector<uint8_t> f_%s=%s;' % (n, ref.carray(bytes(f))) for n, f in frames.items())
main += '\nstatic const u_int8_t igtk[16]=' + ref.carray(igtk) + ';\n'
main += r'''
static ieee80211com ic;static ieee80211_node ni;
static mbuf_t rx(const std::vector<uint8_t>&f){return ieee80211_decrypt(&ic,packet(f),&ni);}
static void accept(const std::vector<uint8_t>&f,u_int64_t ipn){
 mbuf_t m=rx(f);assert(m);auto v=flat(m);
 // the frame as sent before protection: the MMIE is gone, nothing else changed
 assert(v.size()==f.size()-IEEE80211_MMIE_LEN&&!memcmp(v.data(),f.data(),v.size())&&ic.ic_nw_keys[4].k_mgmt_rsc==ipn);mbuf_freem(m);}
static void reject(const std::vector<uint8_t>&f,unsigned replays,unsigned mics){
 const u_int64_t ipn=ic.ic_nw_keys[4].k_mgmt_rsc;assert(!rx(f)&&live==0);
 assert(ic.ic_stats.is_cmac_replays==replays&&ic.ic_stats.is_cmac_icv_errs==mics&&ic.ic_nw_keys[4].k_mgmt_rsc==ipn);}
int main(){
 // Without a key nothing verifies.
 assert(!rx(f_deauth)&&live==0);
 // The IGTK as the handshake delivers it: id 4, last packet number used 4.
 ieee80211_key*k=&ic.ic_nw_keys[4];k->k_id=4;k->k_cipher=IEEE80211_CIPHER_BIP;k->k_flags=IEEE80211_KEY_IGTK;k->k_len=16;k->k_mgmt_rsc=4;memcpy(k->k_key,igtk,16);
 // Not usable before the software context exists.
 assert(!rx(f_deauth)&&live==0);
 assert(ieee80211_bip_set_key(&ic,k)==0&&k->k_priv);k->k_flags|=IEEE80211_KEY_SWCRYPTO;

 accept(f_deauth,5);
 // The same frame captured and sent again, and anything not newer, is a replay.
 reject(f_deauth,1,0);
 accept(f_higher,6);reject(f_deauth,2,0);reject(f_higher,3,0);
 accept(f_disassoc,7);accept(f_flags,8);accept(f_action,9);accept(f_aligned,10);
 // A frame built without the key: flipped MIC, changed reason code, another
 // transmitter, another key. The packet number does not advance on any of them.
 reject(f_badmic,3,1);reject(f_badbody,3,2);reject(f_spoofed,3,3);reject(f_wrongkey,3,4);
 // A key id that is not installed, or not an IGTK id at all, no MMIE, a truncated one.
 reject(f_otherid,3,4);reject(f_badid,3,4);reject(f_nommie,3,4);reject(f_short,3,4);
 // After all that the next genuine frame still passes.
 accept(f_next,27);
 // The second IGTK slot, as after a rekey, has its own counter.
 ieee80211_key*k5=&ic.ic_nw_keys[5];k5->k_id=5;k5->k_cipher=IEEE80211_CIPHER_BIP;k5->k_flags=IEEE80211_KEY_IGTK;k5->k_len=16;k5->k_mgmt_rsc=23;memcpy(k5->k_key,igtk,16);
 assert(ieee80211_bip_set_key(&ic,k5)==0);k5->k_flags|=IEEE80211_KEY_SWCRYPTO;
 {mbuf_t m=rx(f_otherid);assert(m&&k5->k_mgmt_rsc==24&&k->k_mgmt_rsc==27);mbuf_freem(m);}
 ieee80211_bip_delete_key(&ic,k);ieee80211_bip_delete_key(&ic,k5);assert(!k->k_priv&&!k5->k_priv&&live==0);
 puts("PASS production BIP receive (key by MMIE id, AES-128-CMAC, AES): genuine Deauthentication/Disassociation/Action accepted once and stripped of the MMIE, flag bits outside the AAD, block boundaries, replay and older packet numbers, forged MIC/body/transmitter/key, unknown key id, missing and truncated MMIE, second key slot");
}
'''
ref.run('bip', ref.MBUF + ref.net80211() + stubs + code + main)
