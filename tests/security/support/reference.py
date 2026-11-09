"""Independent references and a small harness for the security host tests.

The cryptographic references are written from the specifications, not from
the driver sources: AES-128 and AES-CMAC (FIPS 197, RFC 4493) in plain Python,
the 802.11 PRF and KDF on top of hashlib. They are checked against the
published vectors every time the module is imported.
"""
import hashlib, hmac, re, subprocess, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NET = ROOT / 'mtk80211/openbsd/net80211'
CRYPTO = ROOT / 'mtk80211/openbsd/crypto'


# ---------------------------------------------------------------- AES-128
def _sbox():
    box, p, q = [0] * 256, 1, 1
    while True:
        p = (p ^ (p << 1) ^ (0x1b if p & 0x80 else 0)) & 0xff
        q ^= q << 1
        q ^= q << 2
        q ^= q << 4
        q &= 0xff
        if q & 0x80:
            q ^= 0x09
        rot = lambda v, n: ((v << n) | (v >> (8 - n))) & 0xff
        box[p] = q ^ rot(q, 1) ^ rot(q, 2) ^ rot(q, 3) ^ rot(q, 4) ^ 0x63
        if p == 1:
            break
    box[0] = 0x63
    return box


_SBOX = _sbox()


def _xtime(a):
    return ((a << 1) ^ (0x1b if a & 0x80 else 0)) & 0xff


def aes128_encrypt(key, block):
    assert len(key) == 16 and len(block) == 16
    words = [list(key[i:i + 4]) for i in range(0, 16, 4)]
    rcon = 1
    for i in range(4, 44):
        t = list(words[i - 1])
        if i % 4 == 0:
            t = [_SBOX[b] for b in t[1:] + t[:1]]
            t[0] ^= rcon
            rcon = _xtime(rcon)
        words.append([a ^ b for a, b in zip(words[i - 4], t)])
    rk = [sum(words[4 * r:4 * r + 4], []) for r in range(11)]
    s = [b ^ k for b, k in zip(block, rk[0])]
    for r in range(1, 11):
        s = [_SBOX[b] for b in s]
        s = [s[(i + 4 * (i % 4)) % 16] for i in range(16)]      # ShiftRows
        if r != 10:
            m = []
            for c in range(0, 16, 4):
                a = s[c:c + 4]
                x = a[0] ^ a[1] ^ a[2] ^ a[3]
                m += [a[i] ^ x ^ _xtime(a[i] ^ a[(i + 1) % 4]) for i in range(4)]
            s = m
        s = [b ^ k for b, k in zip(s, rk[r])]
    return bytes(s)


def aes_cmac(key, msg):
    """AES-CMAC, RFC 4493."""
    xor = lambda a, b: bytes(x ^ y for x, y in zip(a, b))

    def double(b):
        n = (int.from_bytes(b, 'big') << 1) & ((1 << 128) - 1)
        return (n ^ (0x87 if b[0] & 0x80 else 0)).to_bytes(16, 'big')

    k1 = double(aes128_encrypt(key, bytes(16)))
    k2 = double(k1)
    blocks = max(1, -(-len(msg) // 16))
    last = msg[16 * (blocks - 1):]
    if len(last) == 16:
        last = xor(last, k1)
    else:
        last = xor(last + b'\x80' + bytes(15 - len(last)), k2)
    x = bytes(16)
    for i in range(blocks - 1):
        x = aes128_encrypt(key, xor(x, msg[16 * i:16 * i + 16]))
    return aes128_encrypt(key, xor(x, last))


def _selftest():
    h = bytes.fromhex
    # FIPS 197 appendix C.1
    assert aes128_encrypt(h('000102030405060708090a0b0c0d0e0f'),
                          h('00112233445566778899aabbccddeeff')) == \
        h('69c4e0d86a7b0430d8cdb78070b4c55a')
    # RFC 4493 section 4
    key = h('2b7e151628aed2a6abf7158809cf4f3c')
    msg = h('6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51'
            '30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710')
    for length, mac in ((0, 'bb1d6929e95937287fa37d129b756746'),
                        (16, '070a16b46b4d4144f79bdd9dd04a287c'),
                        (40, 'dfa66747de9ae63030ca32611497c827'),
                        (64, '51f0bebf7e3b9d92fc49741779363cfe')):
        assert aes_cmac(key, msg[:length]) == h(mac), length


_selftest()


# ------------------------------------------------------ 802.11 key hierarchy
def prf_sha1(key, label, context, nbytes):
    """PRF-n of 802.11-2012 11.6.1.2: HMAC-SHA1(K, A || 0 || B || i)."""
    out, i = b'', 0
    while len(out) < nbytes:
        out += hmac.new(key, label + b'\0' + context + bytes([i]), hashlib.sha1).digest()
        i += 1
    return out[:nbytes]


def kdf_sha256(key, label, context, nbytes):
    """KDF-Hash-Length of 802.11-2012 11.6.1.7.2, with 16-bit little-endian
    counter and length: HMAC-SHA256(K, i || Label || Context || Length)."""
    out, i = b'', 1
    while len(out) < nbytes:
        out += hmac.new(key, i.to_bytes(2, 'little') + label + context +
                        (8 * nbytes).to_bytes(2, 'little'), hashlib.sha256).digest()
        i += 1
    return out[:nbytes]


def ptk_context(aa, spa, anonce, snonce):
    return min(aa, spa) + max(aa, spa) + min(anonce, snonce) + max(anonce, snonce)


# ----------------------------------------------------------------- 802.11w
def bip_mic(igtk, frame):
    """MIC of a group-addressed management frame ending in an MMIE whose MIC
    field is still zero (802.11-2012 11.4.4.5): AES-128-CMAC over the masked
    frame control, the three addresses and the frame body, first 64 bits."""
    assert frame[-18] == 76 and frame[-17] == 16 and frame[-8:] == bytes(8)
    aad = bytes([frame[0], frame[1] & ~0x38]) + frame[4:22]
    return aes_cmac(igtk, aad + frame[24:])[:8]


def mmie(keyid, ipn, mic=bytes(8)):
    return bytes([76, 16]) + keyid.to_bytes(2, 'little') + ipn.to_bytes(6, 'little') + mic


def bip_protect(igtk, frame, keyid, ipn):
    body = frame + mmie(keyid, ipn)
    return body[:-8] + bip_mic(igtk, body)


# ----------------------------------------------------------------- harness
def extract(source, name):
    """A function definition, sliced from driver source by its name."""
    m = re.search(r'(?m)^(?:static[ \t]+)?[A-Za-z_][A-Za-z0-9_ \t\*]*\n' + re.escape(name) + r'\(', source)
    assert m, name
    pos = source.index('{', m.start()) + 1
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[m.start():pos]


def between(source, start, end):
    a = source.index(start)
    return source[a:source.index(end, a)]


def unincluded(path):
    """A C source of the driver without its #include lines."""
    return '\n'.join(l for l in Path(path).read_text().splitlines()
                     if not l.lstrip().startswith('#include'))


def carray(data):
    return '{' + ','.join('0x%02x' % b for b in data) + '}'


# A packet buffer chain with the mbuf calls net80211 uses.
MBUF = r'''
#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
struct mbuf {std::vector<uint8_t> storage;size_t off=0,len=0,pklen=0;mbuf*next=nullptr;};
using mbuf_t=mbuf*;
static unsigned live=0;
static mbuf_t packet(const std::vector<uint8_t>&d,size_t headroom=0){auto*m=new mbuf;m->storage.assign(headroom,0);m->storage.insert(m->storage.end(),d.begin(),d.end());m->off=headroom;m->len=m->pklen=d.size();++live;return m;}
static void mbuf_freem(mbuf_t m){while(m){auto*n=m->next;assert(live);--live;delete m;m=n;}}
static size_t mbuf_len(mbuf_t m){return m->len;}
static size_t mbuf_pkthdr_len(mbuf_t m){return m->pklen;}
static void*mbuf_data(mbuf_t m){return m->storage.data()+m->off;}
static mbuf_t mbuf_next(mbuf_t m){return m->next;}
// like the kernel's: a positive count trims the front, a negative one the tail
static void mbuf_adj(mbuf_t m,int n){if(n>=0){assert((size_t)n<=m->len);m->off+=n;m->len-=n;m->pklen-=n;}else{assert((size_t)-n<=m->len&&!m->next);m->len+=n;m->pklen+=n;}}
static int mbuf_copydata(mbuf_t m,size_t off,size_t len,void*dest){uint8_t*d=(uint8_t*)dest;while(m&&off>=m->len){off-=m->len;m=m->next;}while(m&&len){size_t n=std::min(len,m->len-off);memcpy(d,(uint8_t*)mbuf_data(m)+off,n);d+=n;len-=n;off=0;m=m->next;}return len?EINVAL:0;}
static std::vector<uint8_t> flat(mbuf_t m){std::vector<uint8_t>v(m->pklen);assert(mbuf_copydata(m,0,m->pklen,v.data())==0);return v;}
#define mtod(m,t) ((t)mbuf_data(m))
using u_int8_t=uint8_t;using u_int16_t=uint16_t;using u_int32_t=uint32_t;using u_int64_t=uint64_t;using u_int=unsigned;using caddr_t=char*;
'''


def net80211():
    """The 802.11 definitions the sliced functions need. The Action categories
    and the robust-frame classification are the production ones."""
    h = (NET / 'ieee80211.h').read_text()
    categories = between(h, '/*\n * Action field category values',
                         '/*\n * Block Ack Action field values')
    return NET80211 + categories + NET80211_STRUCTS


NET80211 = r'''
#define __inline inline
#define __packed __attribute__((packed))
#define _KASSERT(e) ((void)0)
#define DPRINTF(x) ((void)0)
#define XYLog(...) ((void)0)
#define splnet() 0
#define splx(s) ((void)(s))
#define explicit_bzero(p,n) memset((p),0,(n))
#define nitems(a) (sizeof(a)/sizeof((a)[0]))
#define IEEE80211_ADDR_LEN 6
#define IEEE80211_ADDR_EQ(a,b) (memcmp((a),(b),6)==0)
#define IEEE80211_ADDR_COPY(d,s) memcpy((d),(s),6)
#define IEEE80211_IS_MULTICAST(a) (*(const u_int8_t*)(a)&1)
#define IEEE80211_FC0_VERSION_0 0x00
#define IEEE80211_FC0_TYPE_MASK 0x0c
#define IEEE80211_FC0_TYPE_MGT 0x00
#define IEEE80211_FC0_TYPE_DATA 0x08
#define IEEE80211_FC0_SUBTYPE_MASK 0xf0
#define IEEE80211_FC0_SUBTYPE_ASSOC_RESP 0x10
#define IEEE80211_FC0_SUBTYPE_PROBE_RESP 0x50
#define IEEE80211_FC0_SUBTYPE_BEACON 0x80
#define IEEE80211_FC0_SUBTYPE_DISASSOC 0xa0
#define IEEE80211_FC0_SUBTYPE_AUTH 0xb0
#define IEEE80211_FC0_SUBTYPE_DEAUTH 0xc0
#define IEEE80211_FC0_SUBTYPE_ACTION 0xd0
#define IEEE80211_FC0_SUBTYPE_ACTION_NOACK 0xe0
#define IEEE80211_FC0_SUBTYPE_SHIFT 4
#define IEEE80211_FC1_DIR_NODS 0x00
#define IEEE80211_FC1_RETRY 0x08
#define IEEE80211_FC1_PWR_MGT 0x10
#define IEEE80211_FC1_MORE_DATA 0x20
#define IEEE80211_FC1_PROTECTED 0x40
#define IEEE80211_SEQ_SEQ_SHIFT 4
#define IEEE80211_QOS_TID 0x0f
#define IEEE80211_WEP_EXTIV 0x20
#define IEEE80211_CCMP_HDRLEN 8
#define IEEE80211_TKIP_HDRLEN 8
#define IEEE80211_MMIE_LEN 18
#define IEEE80211_ELEMID_RSN 48
#define IEEE80211_ELEMID_MMIE 76
#define IEEE80211_PMKID_LEN 16
#define IEEE80211_RSNCAP_PTKSA_RCNT_MASK 0x000c
#define IEEE80211_RSNCAP_GTKSA_RCNT_MASK 0x0030
#define IEEE80211_RSNCAP_MFPR 0x0040
#define IEEE80211_RSNCAP_MFPC 0x0080
#define IEEE80211_RSNCAP_PBAC 0x1000
#define IEEE80211_REASON_AUTH_EXPIRE 2
#define IEEE80211_REASON_AUTH_LEAVE 3
#define IEEE80211_REASON_NOT_AUTHED 6
#define IEEE80211_REASON_NOT_ASSOCED 7
#define IEEE80211_ACTION_ADDBA_REQ 0
#define IEEE80211_ACTION_ADDBA_RESP 1
#define IEEE80211_ACTION_DELBA 2
#define IEEE80211_ACTION_SA_QUERY_REQ 0
#define IEEE80211_ACTION_SA_QUERY_RESP 1
#define IEEE80211_RXI_HWDEC 0x00000001
#define IEEE80211_RXI_HWDEC_SAME_PN 0x00000004
#define IEEE80211_F_RSNON 0x00200000
#define IEEE80211_F_PSK 0x00400000
#define IEEE80211_F_MFPR 0x01000000
#define IEEE80211_F_PBAR 0x04000000
#define IEEE80211_F_BGSCAN 0x08000000
#define IEEE80211_F_DISABLE_BG_AUTO_CONNECT 0x40000000
#define IEEE80211_F_STAYAUTH 0x00000004
#define IEEE80211_C_MFP 0x00002000
#define IEEE80211_PROTO_NONE 0
#define IEEE80211_PROTO_RSN 1
#define IEEE80211_PROTO_WPA 2
#define IEEE80211_NODE_RXPROT 0x0008
#define IEEE80211_NODE_TXPROT 0x0010
#define IEEE80211_NODE_TXRXPROT 0x0018
#define IEEE80211_NODE_RXMGMTPROT 0x0020
#define IEEE80211_NODE_TXMGMTPROT 0x0040
#define IEEE80211_NODE_MFP 0x0080
#define IEEE80211_NODE_PMKID 0x0200
#define IEEE80211_NODE_SA_QUERY 0x0800
#define IEEE80211_NODE_SA_QUERY_FAILED 0x1000
#define IEEE80211_KEY_GROUP 0x00000001
#define IEEE80211_KEY_TX 0x00000002
#define IEEE80211_KEY_IGTK 0x00000004
#define IEEE80211_KEY_SWCRYPTO 0x00000080
#define IEEE80211_GROUP_NKID 6
#define IEEE80211_SA_QUERY_IDLE 0
#define IEEE80211_SA_QUERY_ACTIVE 1
#define IEEE80211_SA_QUERY_HOLDOFF 2
#define LE_READ_2(p) ((u_int16_t)((((const u_int8_t*)(p))[0])|(((const u_int8_t*)(p))[1]<<8)))
#define LE_READ_6(p) ((u_int64_t)LE_READ_2(p)|((u_int64_t)LE_READ_2((const u_int8_t*)(p)+2)<<16)|((u_int64_t)LE_READ_2((const u_int8_t*)(p)+4)<<32))
#define LE_WRITE_2(p,v) do{((u_int8_t*)(p))[0]=(v)&0xff;((u_int8_t*)(p))[1]=((v)>>8)&0xff;}while(0)
enum ieee80211_opmode {IEEE80211_M_STA=1,IEEE80211_M_HOSTAP=6,IEEE80211_M_MONITOR=8};
enum ieee80211_state {IEEE80211_S_INIT=0,IEEE80211_S_SCAN=1,IEEE80211_S_AUTH=2,IEEE80211_S_ASSOC=3,IEEE80211_S_RUN=4};
enum ieee80211_cipher {IEEE80211_CIPHER_NONE=0,IEEE80211_CIPHER_USEGROUP=1,IEEE80211_CIPHER_WEP40=2,IEEE80211_CIPHER_TKIP=4,IEEE80211_CIPHER_CCMP=8,IEEE80211_CIPHER_WEP104=0x10,IEEE80211_CIPHER_BIP=0x20};
enum ieee80211_akm {IEEE80211_AKM_NONE=0,IEEE80211_AKM_8021X=1,IEEE80211_AKM_PSK=2,IEEE80211_AKM_SHA256_8021X=4,IEEE80211_AKM_SHA256_PSK=8};
'''

NET80211_STRUCTS = r'''
struct ieee80211_frame {u_int8_t i_fc[2],i_dur[2],i_addr1[6],i_addr2[6],i_addr3[6],i_seq[2];} __packed;
struct ieee80211_key {u_int8_t k_id;enum ieee80211_cipher k_cipher;u_int k_flags,k_len;u_int64_t k_rsc[16],k_mgmt_rsc,k_tsc;u_int8_t k_key[32];void*k_priv;};
struct ieee80211_rxinfo {u_int32_t rxi_flags=0;};
struct ieee80211_stats {u_int32_t is_ccmp_replays=0,is_tkip_replays=0,is_cmac_replays=0,is_cmac_icv_errs=0,is_rx_deauth=0,is_tx_nombuf=0,
 is_rx_mgmt_prot=0,is_rx_mgmt_unprot=0,is_rx_mgmt_badprot=0,is_sa_query_started=0,is_sa_query_answered=0,is_sa_query_timeouts=0,is_sa_query_replies=0;};
struct ieee80211com;
struct ieee80211_node {u_int8_t ni_macaddr[6]={},ni_bssid[6]={},ni_pmkid[16]={};u_int32_t ni_flags=0;u_int16_t ni_rsncaps=0;
 u_int ni_rsnprotos=0,ni_rsnakms=0,ni_rsnciphers=0;enum ieee80211_cipher ni_rsncipher=IEEE80211_CIPHER_NONE,ni_rsngroupcipher=IEEE80211_CIPHER_NONE,
 ni_rsngroupmgmtcipher=IEEE80211_CIPHER_NONE;ieee80211_key ni_pairwise_key={};int ni_inact=0;u_int16_t ni_txseq=0;ieee80211com*ni_ic=nullptr;void*ni_chan=nullptr;};
struct CTimeout;
struct _ifnet {const char*if_xname="mtk0";int if_flags=0,if_timer=0;void(*if_start)(_ifnet*)=nullptr;};
struct mbuf_queue {std::vector<mbuf_t> q;};
struct ieee80211com {_ifnet ic_if;enum ieee80211_opmode ic_opmode=IEEE80211_M_STA;enum ieee80211_state ic_state=IEEE80211_S_RUN;
 u_int32_t ic_flags=IEEE80211_F_RSNON,ic_caps=IEEE80211_C_MFP,ic_userflags=0;ieee80211_node*ic_bss=nullptr;u_int8_t ic_myaddr[6]={2,0,0,0,0,9};
 u_int ic_rsnprotos=IEEE80211_PROTO_RSN,ic_rsnakms=IEEE80211_AKM_PSK,ic_rsnciphers=IEEE80211_CIPHER_CCMP|IEEE80211_CIPHER_TKIP;int ic_curmode=0;mbuf_queue ic_mgtq;
 ieee80211_key ic_nw_keys[IEEE80211_GROUP_NKID]={};int ic_def_txkey=0,ic_igtk_kid=0;ieee80211_stats ic_stats;u_int16_t ic_deauth_reason=0;
 enum ieee80211_cipher ic_rsngroupmgmtcipher=IEEE80211_CIPHER_BIP;
 CTimeout*ic_sa_query_to=nullptr;u_int16_t ic_sa_query_trid=0;u_int8_t ic_sa_query_state=0,ic_sa_query_count=0;void*ic_softc=nullptr;
 void(*ic_event_handler)(ieee80211com*,int,void*)=nullptr;int(*ic_set_key)(ieee80211com*,ieee80211_node*,ieee80211_key*)=nullptr;
 void(*ic_delete_key)(ieee80211com*,ieee80211_node*,ieee80211_key*)=nullptr;int(*ic_send_mgmt)(ieee80211com*,ieee80211_node*,int,int,int)=nullptr;};
'''


def run(name, source, *flags):
    with tempfile.TemporaryDirectory(prefix='mtk-' + name + '-') as d:
        p = Path(d)
        (p / 'test.cpp').write_text(source)
        subprocess.run(['clang++', '-std=c++14', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-g', *flags,
                        str(p / 'test.cpp'), '-o', str(p / 'test')], check=True)
        subprocess.run([str(p / 'test')], check=True, timeout=60)
