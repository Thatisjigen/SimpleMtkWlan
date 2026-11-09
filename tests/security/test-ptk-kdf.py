#!/usr/bin/env python3
"""Pairwise key derivation of the 4-way handshake, production code.

ieee80211_derive_ptk() with its PRF and KDF is compiled as is and compared
with references written from the standard. The SHA-1 PRF of WPA2-PSK must
give what it always gave; the SHA-256 KDF of the PSK-SHA256 and
802.1X-SHA256 key managements hashes the label without a terminator and the
length of the key requested, 384 bits with CCMP: derived any other way the
access point rejects message 2 and the handshake never completes.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

crypto = (ref.NET / 'ieee80211_crypto.c').read_text()
code = '\n'.join(ref.extract(crypto, n) for n in
                 ('ieee80211_cipher_keylen', 'ieee80211_prf', 'ieee80211_kdf', 'ieee80211_derive_ptk'))

h = bytes.fromhex
pmk = h('0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af')
aa, spa = h('a0a1a1a3a4a5'), h('b0b1b2b3b4b5')
anonce = bytes(range(0xe0, 0x100))
snonce = bytes(range(0xc0, 0xe0))
label = b'Pairwise key expansion'
cases = []
# the order of the two addresses and of the two nonces must not matter
for a, s, an, sn in ((aa, spa, anonce, snonce), (spa, aa, snonce, anonce)):
    ctx = ref.ptk_context(a, s, an, sn)
    cases.append((a, s, an, sn,
                  ref.prf_sha1(pmk, label, ctx, 64),            # PSK, any cipher
                  ref.kdf_sha256(pmk, label, ctx, 48),          # SHA-256, CCMP
                  ref.kdf_sha256(pmk, label, ctx, 64)))         # SHA-256, TKIP
# what the code derived before: terminator hashed with the label, 512 bits asked
wrong = ref.kdf_sha256(pmk, label + b'\0', ref.ptk_context(aa, spa, anonce, snonce), 64)
assert wrong[:48] != cases[0][5]

stubs = r'''
#include <CommonCrypto/CommonHMAC.h>
#ifndef NBBY
#define NBBY 8
#endif
#define htole16(v) ((uint16_t)(v))
#define IEEE80211_ADDR_LEN 6
#define EAPOL_KEY_NONCE_LEN 32
#define IEEE80211_PMK_LEN 32
#define SHA1_DIGEST_LENGTH 20
#define SHA256_DIGEST_LENGTH 32
enum ieee80211_cipher {IEEE80211_CIPHER_NONE=0,IEEE80211_CIPHER_USEGROUP=1,IEEE80211_CIPHER_WEP40=2,IEEE80211_CIPHER_TKIP=4,IEEE80211_CIPHER_CCMP=8,IEEE80211_CIPHER_WEP104=0x10,IEEE80211_CIPHER_BIP=0x20};
enum ieee80211_akm {IEEE80211_AKM_NONE=0,IEEE80211_AKM_8021X=1,IEEE80211_AKM_PSK=2,IEEE80211_AKM_SHA256_8021X=4,IEEE80211_AKM_SHA256_PSK=8};
static int ieee80211_is_sha256_akm(enum ieee80211_akm a){return a==IEEE80211_AKM_SHA256_8021X||a==IEEE80211_AKM_SHA256_PSK;}
struct ieee80211_ptk {u_int8_t kck[16],kek[16],tk[32];} __attribute__((packed));
// HMAC from the system library: the construction around it is what is under test
typedef CCHmacContext HMAC_SHA1_CTX;typedef CCHmacContext HMAC_SHA256_CTX;
static void HMAC_SHA1_Init(HMAC_SHA1_CTX*c,const u_int8_t*k,u_int n){CCHmacInit(c,kCCHmacAlgSHA1,k,n);}
static void HMAC_SHA1_Update(HMAC_SHA1_CTX*c,const u_int8_t*d,u_int n){CCHmacUpdate(c,d,n);}
static void HMAC_SHA1_Final(u_int8_t*o,HMAC_SHA1_CTX*c){CCHmacFinal(c,o);}
static void HMAC_SHA256_Init(HMAC_SHA256_CTX*c,const u_int8_t*k,u_int n){CCHmacInit(c,kCCHmacAlgSHA256,k,n);}
static void HMAC_SHA256_Update(HMAC_SHA256_CTX*c,const u_int8_t*d,u_int n){CCHmacUpdate(c,d,n);}
static void HMAC_SHA256_Final(u_int8_t*o,HMAC_SHA256_CTX*c){CCHmacFinal(c,o);}
'''
main = 'struct Case{u_int8_t aa[6],spa[6],anonce[32],snonce[32],sha1[64],kdf_ccmp[48],kdf_tkip[64];};\nstatic const Case cases[]={\n'
main += ',\n'.join('{' + ','.join(ref.carray(x) for x in c) + '}' for c in cases) + '};\n'
main += 'static const u_int8_t pmk[32]=' + ref.carray(pmk) + ';\nstatic const u_int8_t wrong[64]=' + ref.carray(wrong) + ';\n'
main += r'''
int main(){
 static const u_int8_t zero[32]={};
 for(const Case&c:cases){
  ieee80211_ptk p;
  // WPA2-PSK and 802.1X: the 512 bits of the PRF, whatever the pairwise cipher.
  for(auto akm:{IEEE80211_AKM_PSK,IEEE80211_AKM_8021X})for(auto cipher:{IEEE80211_CIPHER_CCMP,IEEE80211_CIPHER_TKIP}){
   memset(&p,0xa5,sizeof(p));ieee80211_derive_ptk(akm,cipher,pmk,c.aa,c.spa,c.anonce,c.snonce,&p);assert(!memcmp(&p,c.sha1,64));}
  // SHA-256 key managements with CCMP: KCK, KEK and a 128-bit TK from KDF-384,
  // the rest of the structure cleared.
  for(auto akm:{IEEE80211_AKM_SHA256_PSK,IEEE80211_AKM_SHA256_8021X}){
   memset(&p,0xa5,sizeof(p));ieee80211_derive_ptk(akm,IEEE80211_CIPHER_CCMP,pmk,c.aa,c.spa,c.anonce,c.snonce,&p);
   assert(!memcmp(&p,c.kdf_ccmp,48)&&!memcmp(p.tk+16,zero,16));
   // ...and with TKIP its 256-bit TK from KDF-512, a different key from the first octet.
   memset(&p,0xa5,sizeof(p));ieee80211_derive_ptk(akm,IEEE80211_CIPHER_TKIP,pmk,c.aa,c.spa,c.anonce,c.snonce,&p);
   assert(!memcmp(&p,c.kdf_tkip,64)&&memcmp(&p,c.kdf_ccmp,16));}
 }
 // Not the key the terminated label and the 512-bit request used to give.
 {ieee80211_ptk p;const Case&c=cases[0];ieee80211_derive_ptk(IEEE80211_AKM_SHA256_PSK,IEEE80211_CIPHER_CCMP,pmk,c.aa,c.spa,c.anonce,c.snonce,&p);assert(memcmp(&p,wrong,16)&&memcmp(p.kek,wrong+16,16));}
 // An unknown cipher has no temporal key: KCK and KEK only, never past the structure.
 {ieee80211_ptk p;const Case&c=cases[0];memset(&p,0xa5,sizeof(p));ieee80211_derive_ptk(IEEE80211_AKM_SHA256_PSK,IEEE80211_CIPHER_NONE,pmk,c.aa,c.spa,c.anonce,c.snonce,&p);assert(!memcmp(p.tk,zero,32));}
 puts("PASS production ieee80211_derive_ptk: SHA-1 PRF unchanged for PSK and 802.1X with CCMP and TKIP, SHA-256 KDF at 384 bits with CCMP and 512 with TKIP against references written from the standard, address and nonce order, rest of the key cleared");
}
'''
ref.run('ptk-kdf', ref.MBUF + stubs + code + main)
