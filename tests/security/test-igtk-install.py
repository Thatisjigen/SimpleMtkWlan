#!/usr/bin/env python3
"""Installing the integrity group key the access point sends in the handshake.

Production ieee80211_install_igtk(), called for the IGTK KDE of message 3 of
the 4-way handshake and of message 1 of the group key handshake. The key goes
into slot 4 or 5 with the packet number the access point last used; a KDE
that repeats the key already installed must not reset that number (it would
make every captured group management frame valid again); a malformed one
installs nothing.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

pae = (ref.NET / 'ieee80211_pae_input.c').read_text()
code = ref.extract(pae, 'ieee80211_must_update_group_key') + '\n' + ref.extract(pae, 'ieee80211_install_igtk') + '\n'
# both handshakes go through it, and neither still parses the KDE itself
for name in ('ieee80211_recv_4way_msg3', 'ieee80211_recv_rsn_group_msg1'):
    body = ref.extract(pae, name)
    assert 'ieee80211_install_igtk(ic, ni, igtk)' in body and 'k_mgmt_rsc' not in body, name
# the group message does not hold the link back for a key installed later
assert 'case EBUSY:\n                deferlink = 1;' in ref.extract(pae, 'ieee80211_recv_4way_msg3')

main = r'''
using Bytes=std::vector<uint8_t>;
static ieee80211com ic;static ieee80211_node ni;
static std::vector<int> calls;	/* +id: set, -id: delete */
static int set_result=0;static int ctx[2];
static int set_key(ieee80211com*,ieee80211_node*n,ieee80211_key*k){assert(n==&ni);calls.push_back(k->k_id);
 if(set_result!=0&&set_result!=EBUSY)return set_result;k->k_priv=&ctx[k->k_id-4];k->k_flags|=IEEE80211_KEY_SWCRYPTO;return set_result;}
static void delete_key(ieee80211com*,ieee80211_node*n,ieee80211_key*k){assert(n==&ni&&k->k_priv==&ctx[k->k_id-4]);calls.push_back(-(int)k->k_id);memset(k,0,sizeof(*k));}
static Bytes kde(unsigned id,u_int64_t ipn,uint8_t fill,unsigned len=28){
 Bytes b={0xdd,uint8_t(len),0x00,0x0f,0xac,9,uint8_t(id),uint8_t(id>>8)};for(int i=0;i<6;++i)b.push_back(ipn>>(8*i));b.insert(b.end(),16,fill);return b;}
static bool holds(unsigned id,uint8_t fill,u_int64_t ipn){const ieee80211_key&k=ic.ic_nw_keys[id];u_int8_t want[16];memset(want,fill,16);
 return k.k_id==id&&k.k_cipher==IEEE80211_CIPHER_BIP&&k.k_len==16&&(k.k_flags&IEEE80211_KEY_IGTK)&&!(k.k_flags&IEEE80211_KEY_GROUP)&&k.k_mgmt_rsc==ipn&&!memcmp(k.k_key,want,16);}
static bool empty(unsigned id){static const ieee80211_key zero={};return !memcmp(&ic.ic_nw_keys[id],&zero,sizeof(zero));}
int main(){
 ic.ic_set_key=set_key;ic.ic_delete_key=delete_key;ni.ni_rsngroupmgmtcipher=IEEE80211_CIPHER_BIP;
 // The key of message 3: slot 4, last packet number 0x060504030201.
 assert(ieee80211_install_igtk(&ic,&ni,kde(4,0x060504030201ULL,0x11).data())==0);
 assert(holds(4,0x11,0x060504030201ULL)&&ic.ic_igtk_kid==4&&calls==(std::vector<int>{4})&&empty(5));
 // Group management frames arrive and the counter moves on. The same KDE again
 // (a retransmitted, or replayed, message) must not bring it back.
 ic.ic_nw_keys[4].k_mgmt_rsc=0x060504030299ULL;
 assert(ieee80211_install_igtk(&ic,&ni,kde(4,0x060504030201ULL,0x11).data())==0);
 assert(holds(4,0x11,0x060504030299ULL)&&calls.size()==1&&ic.ic_igtk_kid==4);
 // A rekey to the other slot leaves the first key in place for frames still on the air.
 assert(ieee80211_install_igtk(&ic,&ni,kde(5,7,0x22).data())==0);
 assert(holds(5,0x22,7)&&holds(4,0x11,0x060504030299ULL)&&ic.ic_igtk_kid==5&&calls==(std::vector<int>{4,5}));
 // A new key for a slot in use: the old one gives its context back first.
 assert(ieee80211_install_igtk(&ic,&ni,kde(4,9,0x33).data())==0);
 assert(holds(4,0x33,9)&&ic.ic_igtk_kid==4&&calls==(std::vector<int>{4,5,-4,4}));
 // A KDE of the wrong length, or naming a slot that is not an IGTK's, installs nothing.
 const size_t n=calls.size();
 assert(ieee80211_install_igtk(&ic,&ni,kde(5,1,0x44,27).data())==EINVAL&&ieee80211_install_igtk(&ic,&ni,kde(5,1,0x44,44).data())==EINVAL);
 for(unsigned id:{0u,3u,6u,0x104u})assert(ieee80211_install_igtk(&ic,&ni,kde(id,1,0x44).data())==EINVAL);
 assert(calls.size()==n&&holds(5,0x22,7)&&holds(4,0x33,9)&&ic.ic_igtk_kid==4);
 // A driver that cannot take the key: reported, and no half-installed key is left.
 set_result=ENOMEM;assert(ieee80211_install_igtk(&ic,&ni,kde(5,1,0x55).data())==ENOMEM&&empty(5)&&ic.ic_igtk_kid==4&&holds(4,0x33,9));
 // One that installs it later says so, and the key is the current one.
 set_result=EBUSY;assert(ieee80211_install_igtk(&ic,&ni,kde(5,2,0x66).data())==EBUSY&&holds(5,0x66,2)&&ic.ic_igtk_kid==5);
 puts("PASS production ieee80211_install_igtk: key, slot and packet number from the KDE, no reinstall of the current key, rekey to the other slot and within a slot, malformed KDE and driver failure install nothing, deferred install");
}
'''
stubs = r'''
#define IEEE80211_STA_ONLY 1	/* as the kexts are built */
'''
ref.run('igtk-install', ref.MBUF + ref.net80211() + stubs + code + main)
