#!/usr/bin/env python3
"""Management frame protection on receive, and the SA Query a station starts.

The production ieee80211_input_mgmt_protect() decides which management frames
reach their handlers. Without 802.11w negotiated everything is as before.
With it a forged Deauthentication, Disassociation or Block Ack request is
dropped whatever its shape, while the genuine ones pass: encrypted by the
access point and decrypted by the hardware (packet number checked by the
production ieee80211_input_hwdecrypt) or in software, or group addressed with
a MIC verified by the production BIP, CMAC and AES code.

An unprotected frame claiming that the access point no longer knows us starts
the production SA Query: answered, the association stays; unanswered, the
station authenticates again as if the frame had been accepted.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

inp = (ref.NET / 'ieee80211_input.c').read_text()
crypto = (ref.NET / 'ieee80211_crypto.c').read_text()
bip = (ref.NET / 'ieee80211_crypto_bip.c').read_text()
proto = (ref.NET / 'ieee80211_proto.c').read_text()
code = (ref.unincluded(ref.CRYPTO / 'aes.h') + ref.unincluded(ref.CRYPTO / 'aes.c') +
        ref.unincluded(ref.CRYPTO / 'cmac.h') + ref.unincluded(ref.CRYPTO / 'cmac.c') +
        ref.between(bip, '/* BIP software crypto context */', 'mbuf_t\nieee80211_bip_encap(') +
        ref.extract(bip, 'ieee80211_bip_decap') + '\n' +
        ref.extract(crypto, 'ieee80211_get_rxkey') + '\n' + ref.extract(crypto, 'ieee80211_decrypt') + '\n' +
        ref.extract((ref.NET / 'ieee80211_crypto_ccmp.c').read_text(), 'ieee80211_ccmp_get_pn') + '\n' +
        ref.extract(inp, 'ieee80211_input_hwdecrypt') + '\n' +
        ref.extract(inp, 'ieee80211_sta_deauthenticated') + '\n' +
        ref.between(proto, '#define IEEE80211_SA_QUERY_RETRY_MS', '\n/*\n * Association comeback') +
        ref.extract(inp, 'ieee80211_mgmt_unprotected') + '\n' +
        ref.extract(inp, 'ieee80211_input_mgmt_protect') + '\n' +
        ref.extract(inp, 'ieee80211_recv_sa_query_req') + '\n' +
        ref.extract(inp, 'ieee80211_recv_sa_query_resp') + '\n' +
        ref.extract(inp, 'ieee80211_recv_action'))

h = bytes.fromhex
igtk = h('4ea9543e09cf2b1eca66ffc58bdecbcf')
ap, me, bcast = h('020000000001'), h('020000000009'), h('ffffffffffff')


def mgmt(subtype, body, da=bcast):
    return bytes([subtype, 0, 0, 0]) + da + ap + ap + bytes([0x90, 0x12]) + body


group = {
    'deauth': ref.bip_protect(igtk, mgmt(0xc0, h('0700')), 4, 11),
    'forged': ref.bip_protect(bytes(16), mgmt(0xc0, h('0700')), 4, 12),
    'csa': ref.bip_protect(igtk, mgmt(0xd0, h('0004') + bytes(5)), 4, 13),
}

stubs = r'''
#define IEEE80211_STA_ONLY 1	/* as the kexts are built */
#define MIN(a,b) ((a)<(b)?(a):(b))
#define malloc(n,a,b) std::malloc(n)
#define free(p) std::free(p)
#define IEEE80211_EVT_STA_DEAUTH 7
static mbuf_t none(ieee80211com*,mbuf_t,ieee80211_key*){assert(!"cipher not in this test");return nullptr;}
#define ieee80211_wep_decrypt none
#define ieee80211_tkip_decrypt none
static int ieee80211_tkip_get_tsc(uint64_t*,uint64_t**,mbuf_t,ieee80211_key*){assert(false);return EINVAL;}
// 24 octets, 28 with an HT Control field (Order bit)
static u_int ieee80211_get_hdrlen(const ieee80211_frame*w){return 24+((w->i_fc[1]&0x80)?4:0);}
static bool ieee80211_has_qos(const ieee80211_frame*){return false;}
static unsigned ieee80211_get_qos(const ieee80211_frame*){return 0;}
// Software CCMP stands in as a switch: it strips header and MIC when the test
// says the frame verifies, and consumes it otherwise.
static unsigned sw_calls=0;static bool sw_ok=true;
static mbuf_t ieee80211_ccmp_decrypt(ieee80211com*,mbuf_t m,ieee80211_key*){++sw_calls;if(!sw_ok){mbuf_freem(m);return nullptr;}
 auto*d=mtod(m,u_int8_t*);d[1]&=~IEEE80211_FC1_PROTECTED;memmove(d+8,d,24);mbuf_adj(m,8);mbuf_adj(m,-8);return m;}
struct Sent{int categ,action,arg;};static std::vector<Sent> sent;
#define IEEE80211_SEND_ACTION(ic,ni,categ,action,arg) (assert((ni)==(ic)->ic_bss),sent.push_back({(categ),(action),(int)(arg)}),0)
struct Trans{int state,mgt;};static std::vector<Trans> transitions;static std::vector<unsigned> events;
static int ieee80211_new_state(ieee80211com*,int s,int mgt){transitions.push_back({s,mgt});return 0;}
static int armed_ms=-1;
static int timeout_add_msec(CTimeout**,int ms){armed_ms=ms;return 1;}
static int timeout_add_sec(CTimeout**,int s){armed_ms=s*1000;return 1;}
static int timeout_del(CTimeout**){armed_ms=-1;return 1;}
static unsigned addba_req=0,addba_resp=0,delba=0;
static void ieee80211_recv_addba_req(ieee80211com*,mbuf_t,ieee80211_node*){++addba_req;}
static void ieee80211_recv_addba_resp(ieee80211com*,mbuf_t,ieee80211_node*){++addba_resp;}
static void ieee80211_recv_delba(ieee80211com*,mbuf_t,ieee80211_node*){++delba;}
void ieee80211_sta_deauthenticated(ieee80211com*,u_int16_t);
void ieee80211_sta_sa_query_start(ieee80211com*);
void ieee80211_sta_sa_query_response(ieee80211com*,u_int16_t);
void ieee80211_sta_sa_query_stop(ieee80211com*);
'''
main = '\n'.join('static const std::vector<uint8_t> g_%s=%s;' % (n, ref.carray(f)) for n, f in group.items())
main += '\nstatic const u_int8_t igtk[16]=' + ref.carray(igtk) + ';\n'
main += r'''
static const u_int8_t AP[6]={2,0,0,0,0,1},ME[6]={2,0,0,0,0,9},ALL[6]={255,255,255,255,255,255},EVIL[6]={2,0,0,0,0,0x66};
static ieee80211com ic;static ieee80211_node ni;static ieee80211_rxinfo hw,sw;
using Bytes=std::vector<uint8_t>;
static Bytes mk(unsigned subtype,Bytes body,const u_int8_t*da=ME,const u_int8_t*sa=AP,unsigned fc1=0){
 Bytes f(24);f[0]=subtype;f[1]=fc1;memcpy(&f[4],da,6);memcpy(&f[10],sa,6);memcpy(&f[16],AP,6);f[22]=0x90;f[23]=0x12;f.insert(f.end(),body.begin(),body.end());return f;}
// What the access point sent encrypted, as the driver hands it over after the
// hardware decrypted it: Protected set, CCMP header with the packet number, body in clear.
static Bytes hwdec(Bytes f,u_int64_t pn){f[1]|=IEEE80211_FC1_PROTECTED;const u_int8_t iv[8]={u_int8_t(pn),u_int8_t(pn>>8),0,0x20,u_int8_t(pn>>16),u_int8_t(pn>>24),u_int8_t(pn>>32),u_int8_t(pn>>40)};f.insert(f.begin()+24,iv,iv+8);return f;}
// ...and as it arrives when the hardware did not: the MIC still at the end.
static Bytes swenc(Bytes f,u_int64_t pn){f=hwdec(f,pn);f.insert(f.end(),8,0xee);return f;}
static mbuf_t rx(const Bytes&f,ieee80211_rxinfo&rxi=sw){return ieee80211_input_mgmt_protect(&ic,&ni,packet(f),&rxi);}
static void pass(const Bytes&f,const Bytes&want,ieee80211_rxinfo&rxi=sw){mbuf_t m=rx(f,rxi);assert(m&&flat(m)==want);mbuf_freem(m);assert(live==0);}
static void drop(const Bytes&f,ieee80211_rxinfo&rxi=sw){assert(!rx(f,rxi)&&live==0);}
static void fire(){assert(armed_ms>=0);armed_ms=-1;ieee80211_sta_sa_query_timeout(&ic);}
static void action(const Bytes&f,ieee80211_rxinfo&rxi){mbuf_t m=rx(f,rxi);assert(m);ieee80211_recv_action(&ic,m,&ni);mbuf_freem(m);assert(live==0);}
int main(){
 hw.rxi_flags=IEEE80211_RXI_HWDEC;ic.ic_bss=&ni;memcpy(ni.ni_macaddr,AP,6);memcpy(ni.ni_bssid,AP,6);
 ic.ic_event_handler=[](ieee80211com*c,int code,void*){events.push_back(code<<16|c->ic_deauth_reason);};
 ni.ni_rsncipher=IEEE80211_CIPHER_CCMP;ni.ni_pairwise_key.k_cipher=IEEE80211_CIPHER_CCMP;ni.ni_pairwise_key.k_flags=IEEE80211_KEY_SWCRYPTO;
 ieee80211_key*k=&ic.ic_nw_keys[4];k->k_id=4;k->k_cipher=IEEE80211_CIPHER_BIP;k->k_flags=IEEE80211_KEY_IGTK;k->k_len=16;k->k_mgmt_rsc=10;memcpy(k->k_key,igtk,16);
 assert(ieee80211_bip_set_key(&ic,k)==0);k->k_flags|=IEEE80211_KEY_SWCRYPTO;
 const Bytes deauth3=mk(0xc0,{3,0}),deauth7=mk(0xc0,{7,0}),disassoc6=mk(0xa0,{6,0}),addba=mk(0xd0,{3,0,1,2,0x10,0,0,0x10,0}),beacon=mk(0x80,{1,2,3},ALL);
 auto&st=ic.ic_stats;

 // --- Without management frame protection negotiated nothing changes: frames
 // pass as they are, and an encrypted one has no key to be read with.
 pass(deauth3,deauth3);pass(mk(0xc0,{7,0},ALL),mk(0xc0,{7,0},ALL));pass(addba,addba);pass(beacon,beacon);pass(g_deauth,g_deauth);
 drop(swenc(addba,1));assert(sw_calls==0&&sent.empty()&&st.is_rx_mgmt_unprot==0&&st.is_rx_mgmt_badprot==0&&st.is_rx_mgmt_prot==0);

 // --- Negotiated, keys installed.
 ni.ni_flags=IEEE80211_NODE_MFP|IEEE80211_NODE_RXMGMTPROT|IEEE80211_NODE_TXMGMTPROT;
 // A forged Deauthentication, Disassociation or Block Ack request is dropped,
 // addressed to us or to everybody.
 drop(deauth3);drop(mk(0xa0,{8,0}));drop(addba);drop(mk(0xd0,{3,2,0,0,1,0}));drop(mk(0xc0,{3,0},ALL));drop(mk(0xa0,{3,0},ALL));drop(mk(0xd0,{0,4,0},ALL));
 // A group addressed one long enough to hold a Management MIC element, without
 // one: other elements at its end, or an element 76 of the wrong length.
 {Bytes tail={3,0,0xdd,16};tail.insert(tail.end(),16,0x5a);drop(mk(0xc0,tail,ALL));Bytes csa={0,4};csa.insert(csa.end(),24,1);drop(mk(0xd0,csa,ALL));
  Bytes odd={3,0,0,0,76,15};odd.insert(odd.end(),15,0);drop(mk(0xc0,odd,ALL));}
 assert(st.is_rx_mgmt_unprot==10&&st.is_cmac_icv_errs==0&&sent.empty()&&transitions.empty()&&events.empty());
 // Frames 802.11w does not cover pass: beacons and the like, and the Action
 // categories exchanged without keys (Public, HT, VHT, vendor specific).
 pass(beacon,beacon);for(unsigned st2:{0x10u,0x50u,0xb0u}){auto f=mk(st2,{0,0,0,0});pass(f,f);}
 for(unsigned cat:{4u,7u,11u,15u,21u,30u,127u}){auto f=mk(0xd0,{u_int8_t(cat),0,0});pass(f,f);auto g=mk(0xd0,{u_int8_t(cat),0,0},ALL);pass(g,g);auto n=mk(0xe0,{u_int8_t(cat),0,0});pass(n,n);}
 for(unsigned cat:{0u,1u,3u,5u,8u,10u,126u}){drop(mk(0xd0,{u_int8_t(cat),0,0}));drop(mk(0xe0,{u_int8_t(cat),0,0}));}
 // ...judged on the octet the handler will read: an HT Control field does not
 // disguise a Block Ack frame as a Public one.
 drop(mk(0xd0,{3,2,0,0,4,0,0},ME,AP,0x80));
 // None of them, though, when marked Protected.
 drop(hwdec(beacon,1),hw);drop(swenc(mk(0xb0,{0,0,0,0}),1));
 assert(st.is_rx_mgmt_unprot==25&&st.is_rx_mgmt_prot==0);

 // The genuine ones. Decrypted by the hardware: the packet number must grow.
 pass(hwdec(deauth3,5),deauth3,hw);assert(ni.ni_pairwise_key.k_mgmt_rsc==5&&st.is_rx_mgmt_prot==1);
 drop(hwdec(deauth3,5),hw);drop(hwdec(addba,4),hw);assert(st.is_ccmp_replays==2&&st.is_rx_mgmt_badprot==2);
 pass(hwdec(addba,6),addba,hw);pass(hwdec(mk(0xd0,{4,0}),7),mk(0xd0,{4,0}),hw);assert(ni.ni_pairwise_key.k_mgmt_rsc==7&&ni.ni_pairwise_key.k_rsc[0]==0);
 // Not decrypted by the hardware: software CCMP has to verify it.
 pass(swenc(addba,8),addba);assert(sw_calls==1&&st.is_rx_mgmt_prot==4);
 sw_ok=false;drop(swenc(deauth3,9));sw_ok=true;assert(sw_calls==2&&st.is_rx_mgmt_badprot==3);
 // Group addressed: the MIC of the Management MIC element, with the real key only.
 {Bytes plain(g_deauth.begin(),g_deauth.end()-18);pass(g_deauth,plain);assert(k->k_mgmt_rsc==11);}
 drop(g_deauth);drop(g_forged);assert(st.is_cmac_replays==1&&st.is_cmac_icv_errs==1&&st.is_rx_mgmt_badprot==5);
 {Bytes plain(g_csa.begin(),g_csa.end()-18);pass(g_csa,plain);}
 // Never encrypted; and a frame in two buffers cannot be verified.
 {Bytes p=g_csa;p[1]|=IEEE80211_FC1_PROTECTED;drop(p);
  mbuf_t m=packet(Bytes(deauth3.begin(),deauth3.begin()+24));m->next=packet(Bytes(deauth3.begin()+24,deauth3.end()));m->pklen=deauth3.size();
  assert(!ieee80211_input_mgmt_protect(&ic,&ni,m,&sw)&&live==0&&st.is_rx_mgmt_badprot==7);}
 assert(sent.empty()&&transitions.empty()&&events.empty()&&ic.ic_sa_query_state==IEEE80211_SA_QUERY_IDLE);

 // --- SA Query. "You are not associated" (reason 7), unprotected: dropped, but
 // the access point is asked. The association stays up while it is.
 const unsigned unprot=st.is_rx_mgmt_unprot;
 drop(deauth7);assert(st.is_rx_mgmt_unprot==unprot+1&&st.is_sa_query_started==1&&ic.ic_sa_query_state==IEEE80211_SA_QUERY_ACTIVE);
 assert(sent.size()==1&&sent[0].categ==IEEE80211_CATEG_SA_QUERY&&sent[0].action==IEEE80211_ACTION_SA_QUERY_REQ&&sent[0].arg==1&&armed_ms==206);
 // A flood of forged frames does not start another one.
 for(int i=0;i<20;++i){drop(deauth7);drop(disassoc6);}assert(sent.size()==1&&st.is_sa_query_started==1);
 // No answer: a request every 201 TU, each with its own transaction, for 1000 TU...
 for(int i=2;i<=5;++i){fire();assert(sent.size()==size_t(i)&&sent.back().arg==i&&armed_ms==206&&transitions.empty());}
 // ...then the association is over, exactly as if a Deauthentication had been accepted.
 fire();assert(sent.size()==5&&armed_ms==-1&&st.is_sa_query_timeouts==1&&ic.ic_sa_query_state==IEEE80211_SA_QUERY_IDLE);
 assert(events.size()==1&&events[0]==(IEEE80211_EVT_STA_DEAUTH<<16|IEEE80211_REASON_AUTH_EXPIRE));
 assert(transitions.size()==1&&transitions[0].state==IEEE80211_S_AUTH&&transitions[0].mgt==IEEE80211_FC0_SUBTYPE_DEAUTH);
 sent.clear();transitions.clear();events.clear();

 // The access point answers, protected: the frame was forged or stale.
 drop(mk(0xa0,{6,0},ALL));assert(sent.size()==1&&sent[0].arg==6&&st.is_sa_query_started==2);
 fire();assert(sent.size()==2&&sent[1].arg==7);
 // An answer nobody asked for, or one that did not come protected, proves nothing.
 action(hwdec(mk(0xd0,{8,1,99,0}),20),hw);assert(ic.ic_sa_query_state==IEEE80211_SA_QUERY_ACTIVE);
 drop(mk(0xd0,{8,1,7,0}));assert(ic.ic_sa_query_state==IEEE80211_SA_QUERY_ACTIVE);
 // The answer to the first of the two requests counts as well.
 action(hwdec(mk(0xd0,{8,1,6,0}),21),hw);
 assert(ic.ic_sa_query_state==IEEE80211_SA_QUERY_HOLDOFF&&st.is_sa_query_answered==1&&armed_ms==10000&&transitions.empty()&&events.empty());
 // For a while no new query: forged frames cannot keep the station transmitting.
 for(int i=0;i<20;++i)drop(deauth7);assert(sent.size()==2&&st.is_sa_query_started==2);
 fire();assert(ic.ic_sa_query_state==IEEE80211_SA_QUERY_IDLE&&armed_ms==-1&&transitions.empty());
 drop(deauth7);assert(sent.size()==3&&st.is_sa_query_started==3&&sent[2].arg==8);
 ieee80211_sta_sa_query_stop(&ic);assert(ic.ic_sa_query_state==IEEE80211_SA_QUERY_IDLE&&armed_ms==-1);sent.clear();

 // No query for a frame from somebody else, for another reason code, before
 // our own frames can leave protected, or when not associated.
 drop(mk(0xc0,{7,0},ME,EVIL));drop(deauth3);drop(mk(0xc0,{7}));
 ni.ni_flags&=~IEEE80211_NODE_TXMGMTPROT;drop(deauth7);ni.ni_flags|=IEEE80211_NODE_TXMGMTPROT;
 ic.ic_state=IEEE80211_S_ASSOC;drop(deauth7);ic.ic_state=IEEE80211_S_RUN;
 assert(sent.empty()&&st.is_sa_query_started==3);

 // --- The access point asks us (somebody is associating in our name): answer
 // with its transaction identifier.
 action(hwdec(mk(0xd0,{8,0,0x34,0x12}),30),hw);
 assert(sent.size()==1&&sent[0].categ==IEEE80211_CATEG_SA_QUERY&&sent[0].action==IEEE80211_ACTION_SA_QUERY_RESP&&sent[0].arg==0x1234&&st.is_sa_query_replies==1);
 // An unprotected request never gets that far, and without keys there is no answer.
 drop(mk(0xd0,{8,0,1,0}));ni.ni_flags&=~IEEE80211_NODE_RXMGMTPROT;action(mk(0xd0,{8,0,1,0}),sw);assert(sent.size()==1);
 ni.ni_flags|=IEEE80211_NODE_RXMGMTPROT;
 // Block Ack frames reach their handlers only protected.
 action(hwdec(addba,31),hw);action(hwdec(mk(0xd0,{3,1,1,0,0,0,0x10,0,0}),32),hw);action(hwdec(mk(0xd0,{3,2,0,0,1,0}),33),hw);
 assert(addba_req==1&&addba_resp==1&&delba==1);

 ieee80211_bip_delete_key(&ic,k);assert(live==0);
 puts("PASS production management frame protection on receive: unchanged without 802.11w; forged unicast and group Deauthentication/Disassociation/robust Action dropped, non-robust frames passed, category judged where the handler reads it; hardware-decrypted frames with packet number and replay, software CCMP, real BIP for group frames; SA Query started on reason 6/7 only, once, retried five times, timeout re-authenticates, protected answer keeps the association and holds off new queries; SA Query request answered with its transaction");
}
'''
ref.run('mgmt-protect', ref.MBUF + ref.net80211() + stubs + code + main)
