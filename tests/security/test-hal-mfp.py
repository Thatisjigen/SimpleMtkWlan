#!/usr/bin/env python3
"""802.11w in the hardware layer: keys, leaving a BSS, boot-args.

Production mwx_set_key(), mwx_delete_key(), mwx_setkey_task() and
mwx_mgmt_prot_leave(), and the attach fragment that reads the boot-args.

The integrity group key must never reach the firmware: a key record with BIP
alone replaces the CCMP key of the access point's station record, and the
delete command for it removes that key. Pairwise and group keys go through the
key task as before.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

hal = (ref.ROOT / 'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()
front = (ref.ROOT / 'AirportMtkWlan/AirportItlwmV2.cpp').read_text()
attach = ref.between(hal, '\t/*\n\t * Management frame protection (802.11w), negotiated with the access',
                     '\t/*\n\t * 802.11n. 20/40 MHz must be advertised')
# macOS is told nothing new. Capabilities 75 and 76 (byte 9) are not about
# 802.11w: macOS tests them before SAE and GCMP, which the driver does not have
wcl = ref.between(front, '#ifdef AIRPORT_WCL\n    // WPA2 can use TKIP for group traffic', '#else\n')
assert 'capabilities[17]' in wcl and 'capabilities[9]' not in wcl
code = (ref.extract(hal, 'mwx_set_key') + '\n' + ref.extract(hal, 'mwx_delete_key') + '\n' +
        ref.extract(hal, 'mwx_setkey_task') + '\n' + ref.extract(hal, 'mwx_mgmt_prot_leave') + '\n' +
        'static void pmf_attach(struct mwx_softc *sc, struct ieee80211com *ic)\n{\n' + attach + '}\n')
# leaving a BSS runs the hook before the peer record, and its key, are removed
task = ref.extract(hal, 'mwx_newstate_task')
assert task.index('mwx_mgmt_prot_leave(sc, arg);') < task.index('mt7921_mcu_set_associated_bss(sc, 0)')
# the periodic line that shows the counters exists only with 802.11w on
assert 'if (verbose && (ic->ic_caps & IEEE80211_C_MFP)) {' in hal and '"pmf assoc=%d ap=%d negotiated=%d' in hal

stubs = r'''
#undef DPRINTF
#define DPRINTF(...) ((void)0)
#define MWX_DEV_LOG(...) ((void)0)
#define DEVNAME(sc) "test"
struct task {int unused;};
struct mwx_setkey_task_arg {struct ieee80211_node *ni;struct ieee80211_key *k;};
struct mwx_softc {ieee80211com sc_ic;mwx_setkey_task_arg sc_setkey_arg[8]={};int sc_setkey_cur=0,sc_setkey_tail=0;void*sc_nswq=nullptr;task sc_setkey_task;};
static bool transition_ok=true,cleanup_only=false;
struct MwxRxAmsduTransition {explicit MwxRxAmsduTransition(mwx_softc*){} bool ok() const {return transition_ok;}};
static bool mwx_rx_amsdu_key_cleanup_only(mwx_softc*){return cleanup_only;}
struct Call{char what;int id;};static std::vector<Call> calls;	/* S/D software set/delete, U/X firmware update/delete */
static int sw_ctx[8];static int sw_result=0,fw_result=0;static unsigned tasks=0;
static int ieee80211_set_key(ieee80211com*,ieee80211_node*,ieee80211_key*k){calls.push_back({'S',k->k_id});if(sw_result)return sw_result;k->k_priv=&sw_ctx[k->k_id];k->k_flags|=IEEE80211_KEY_SWCRYPTO;return 0;}
static void ieee80211_delete_key(ieee80211com*,ieee80211_node*,ieee80211_key*k){calls.push_back({'D',k->k_id});memset(k,0,sizeof(*k));}
static int mt7921_mcu_sta_key_update(mwx_softc*,ieee80211_node*,ieee80211_key*k){calls.push_back({'U',k->k_id});return fw_result;}
static void mt7921_mcu_sta_key_delete(mwx_softc*,ieee80211_node*,ieee80211_key*k){calls.push_back({'X',k->k_id});}
static void task_add(void*,task*){++tasks;}
static std::vector<int> mgmt;static unsigned starts=0,delays=0,saq_stops=0;
#define IEEE80211_SEND_MGMT(ic,ni,type,arg) (mgmt.push_back((type)<<8|(arg)),0)
static void mwx_start(_ifnet*){++starts;}
static void delay(int us){assert(us==30000);++delays;}
static void ieee80211_sta_sa_query_stop(ieee80211com*){++saq_stops;}
static const char*bootargs="";
static bool PE_parse_boot_argn(const char*name,void*,int){const char*p=strstr(bootargs,name);return p&&(p[strlen(name)]==' '||p[strlen(name)]==0);}
'''
main = r'''
static std::string trace(){std::string s;for(auto&c:calls){s+=c.what;s+=char('0'+c.id);}calls.clear();return s;}
static ieee80211_key key(unsigned id,enum ieee80211_cipher cipher,unsigned flags,unsigned len=16){ieee80211_key k={};k.k_id=id;k.k_cipher=cipher;k.k_flags=flags;k.k_len=len;k.k_mgmt_rsc=77;return k;}
int main(){
 mwx_softc sc;ieee80211com*ic=&sc.sc_ic;ieee80211_node ni;ic->ic_bss=&ni;ic->ic_softc=&sc;
 // --- The IGTK: software context only, usable at once (no EBUSY), nothing queued for the firmware.
 ieee80211_key igtk=key(4,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK);
 assert(mwx_set_key(ic,&ni,&igtk)==0&&trace()=="S4"&&tasks==0&&sc.sc_setkey_cur==0&&(igtk.k_flags&IEEE80211_KEY_SWCRYPTO)&&igtk.k_mgmt_rsc==77);
 mwx_setkey_task(&sc);assert(trace()=="");
 // Not a second context for a key that has one.
 assert(mwx_set_key(ic,&ni,&igtk)==0&&trace()==""&&tasks==0);
 // Only what 802.11w defines for this driver: BIP-CMAC-128 in slot 4 or 5.
 {auto k=key(3,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK);assert(mwx_set_key(ic,&ni,&k)==EINVAL);
  k=key(6,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK);assert(mwx_set_key(ic,&ni,&k)==EINVAL);
  k=key(4,IEEE80211_CIPHER_CCMP,IEEE80211_KEY_IGTK);assert(mwx_set_key(ic,&ni,&k)==EINVAL);
  k=key(4,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK,32);assert(mwx_set_key(ic,&ni,&k)==EINVAL);assert(trace()==""&&tasks==0);}
 // A software failure is reported, not hidden behind EBUSY.
 {auto k=key(5,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK);sw_result=ENOMEM;assert(mwx_set_key(ic,&ni,&k)==ENOMEM&&trace()=="S5"&&tasks==0);sw_result=0;}
 // Deleting it frees the context and sends the firmware nothing.
 mwx_delete_key(ic,&ni,&igtk);assert(trace()=="D4"&&igtk.k_priv==nullptr);
 // ...also from net80211's group key cleanup, which has no node at hand.
 igtk=key(5,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK);assert(mwx_set_key(ic,nullptr,&igtk)==0);trace();mwx_delete_key(ic,nullptr,&igtk);assert(trace()=="D5");
 // Like every software key it waits for the receive path: not while a transition is refused.
 igtk=key(4,IEEE80211_CIPHER_BIP,IEEE80211_KEY_IGTK);assert(mwx_set_key(ic,&ni,&igtk)==0);trace();
 transition_ok=false;mwx_delete_key(ic,&ni,&igtk);assert(trace()==""&&igtk.k_priv);cleanup_only=true;mwx_delete_key(ic,&ni,&igtk);assert(trace()=="D4");transition_ok=true;cleanup_only=false;

 // --- Pairwise and group keys as before: software context, then the firmware from the task.
 ieee80211_key ptk=key(0,IEEE80211_CIPHER_CCMP,0),gtk=key(1,IEEE80211_CIPHER_CCMP,IEEE80211_KEY_GROUP);
 ptk.k_rsc[3]=5;gtk.k_rsc[0]=9;
 assert(mwx_set_key(ic,&ni,&ptk)==EBUSY&&mwx_set_key(ic,&ni,&gtk)==EBUSY&&trace()=="S0S1"&&tasks==2&&sc.sc_setkey_cur==2);
 mwx_setkey_task(&sc);assert(trace()=="U0U1"&&sc.sc_setkey_tail==2);
 // The firmware's packet numbers start over with a new pairwise key, for data
 // and for protected management frames; the group key keeps what the handshake gave.
 assert(ptk.k_rsc[3]==0&&ptk.k_mgmt_rsc==0&&gtk.k_rsc[0]==9&&gtk.k_mgmt_rsc==77);
 // Not when the firmware refused the key.
 ptk.k_rsc[3]=5;ptk.k_mgmt_rsc=6;fw_result=EIO;assert(mwx_set_key(ic,&ni,&ptk)==EBUSY);mwx_setkey_task(&sc);assert(trace()=="U0"&&ptk.k_rsc[3]==5&&ptk.k_mgmt_rsc==6);fw_result=0;
 mwx_delete_key(ic,&ni,&ptk);assert(trace()=="X0D0");mwx_delete_key(ic,&ni,&gtk);assert(trace()=="X1D1");

 // --- Leaving a BSS that protects management frames: a Deauthentication goes
 // out while the firmware can still encrypt it, then protection is off.
 const unsigned prot=IEEE80211_NODE_TXMGMTPROT|IEEE80211_NODE_RXMGMTPROT;
 ni.ni_flags=prot|IEEE80211_NODE_MFP;mwx_mgmt_prot_leave(&sc,-1);
 assert(mgmt==(std::vector<int>{IEEE80211_FC0_SUBTYPE_DEAUTH<<8|IEEE80211_REASON_AUTH_LEAVE})&&starts==1&&delays==1&&!(ni.ni_flags&prot)&&(ni.ni_flags&IEEE80211_NODE_MFP)&&saq_stops==1);
 // Not when the access point ended the association itself...
 mgmt.clear();for(int arg:{IEEE80211_FC0_SUBTYPE_DEAUTH,IEEE80211_FC0_SUBTYPE_DISASSOC}){ni.ni_flags=prot;mwx_mgmt_prot_leave(&sc,arg);assert(mgmt.empty()&&!(ni.ni_flags&prot));}
 // ...nor before our own frames were protected, nor without 802.11w: as before, nothing.
 ni.ni_flags=IEEE80211_NODE_RXMGMTPROT;mwx_mgmt_prot_leave(&sc,-1);assert(mgmt.empty()&&ni.ni_flags==0);
 ni.ni_flags=0;mwx_mgmt_prot_leave(&sc,-1);assert(mgmt.empty()&&starts==1&&delays==1&&ni.ni_flags==0);
 ic->ic_bss=nullptr;mwx_mgmt_prot_leave(&sc,-1);ic->ic_bss=&ni;

 // --- Boot-args: off by default, -mtkpmf offered, -mtkpmfreq required.
 auto boot=[&](const char*args){bootargs=args;ic->ic_caps=0;ic->ic_flags=IEEE80211_F_RSNON;pmf_attach(&sc,ic);return (ic->ic_caps&IEEE80211_C_MFP?1:0)|(ic->ic_flags&IEEE80211_F_MFPR?2:0);};
 assert(boot("")==0&&boot("-mtkhe debug=0x100")==0&&ic->ic_flags==IEEE80211_F_RSNON);
 assert(boot("-mtkpmf")==1&&boot("-v -mtkpmf -mtkhe")==1&&boot("-mtkpmfreq")==3&&boot("-v -mtkpmfreq -mtkhe")==3&&boot("-mtkpmf -mtkpmfreq")==3);
 puts("PASS production HAL 802.11w: IGTK in software only (set, reject, delete, transition guard), pairwise/group keys through the firmware task with data and management packet numbers restarted, protected Deauthentication before leaving a protected BSS and only then, boot-args, SAE and GCMP capabilities towards macOS left clear");
}
'''
ref.run('hal-mfp', ref.MBUF + '#include <string>\n' + ref.net80211() + stubs + code + main)
