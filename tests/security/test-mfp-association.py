#!/usr/bin/env python3
"""Negotiating management frame protection, and protecting what we send.

Production code: the RSN checks of ieee80211_match_bss(), the choice in
ieee80211_choose_rsnparams(), the RSN element of ieee80211_add_rsn() that goes
into the association request and into message 2 of the handshake, and the
Protected bit ieee80211_mgmt_output() sets for the driver.

802.11w is offered to the access points that advertise it and to nobody else:
towards the others the element is, octet for octet, the one always sent.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

node = (ref.NET / 'ieee80211_node.c').read_text()
out = (ref.NET / 'ieee80211_output.c').read_text()
# the RSN part of ieee80211_match_bss(), a function of its own for the test
rsn = ref.between(node, '    if (ic->ic_flags & IEEE80211_F_RSNON) {\n        /*\n         * If at least one RSN IE field',
                  '    if (ic->ic_if.if_flags & IFF_DEBUG && ieee80211_debug) {')
assert 'we require MFP but AP does not support it' in rsn
code = ('static int match_rsn(struct ieee80211com *ic, struct ieee80211_node *ni)\n{\n    int fail = 0;\n' + rsn +
        '    return fail;\n}\n' +
        ref.extract(node, 'ieee80211_choose_rsnparams') + '\n' +
        ref.extract(out, 'ieee80211_add_rsn_body') + '\n' + ref.extract(out, 'ieee80211_add_rsn') + '\n' +
        ref.extract(out, 'ieee80211_action_name') + '\n' +
        ref.extract(out, 'ieee80211_is_robust_mgmt') + '\n' + ref.extract(out, 'ieee80211_mgmt_output') + '\n' +
        ref.extract(out, 'ieee80211_get_sa_query') + '\n')

stubs = r'''
#define IEEE80211_STA_ONLY 1	/* as the kexts are built */
#define IEEE80211_NODE_ASSOCFAIL_WPA_PROTO 0x40
#define IFF_DEBUG 0x4
#define MBUF_DONTWAIT 0
#define MT_DATA 1
#define htole16(v) ((u_int16_t)(v))
#define panic(...) assert(!"panic")
static const u_int8_t IEEE80211_OUI[3]={0x00,0x0f,0xac},MICROSOFT_OUI[3]={0x00,0x50,0xf2};
struct ieee80211_pmk {u_int8_t pmk_pmkid[16];};
static ieee80211_pmk*ieee80211_pmksa_find(ieee80211com*,ieee80211_node*,const u_int8_t*){return nullptr;}
using ifnet_t=void*;
static int mbuf_prepend(mbuf_t*m,size_t n,int){assert((*m)->off>=n);(*m)->off-=n;(*m)->len+=n;(*m)->pklen+=n;return 0;}
static void mbuf_pkthdr_setrcvif(mbuf_t,ifnet_t){}
static void mbuf_setlen(mbuf_t m,size_t n){m->len=n;}
static void mbuf_pkthdr_setlen(mbuf_t m,size_t n){m->pklen=n;}
static void mq_enqueue(mbuf_queue*q,mbuf_t m){q->q.push_back(m);}
static const char*const ieee80211_mgt_subtype_name[16]={};static const char*const ieee80211_phymode_name[8]={};
static int ieee80211_chan2ieee(ieee80211com*,void*){return 0;}
static mbuf_t ieee80211_getmgmt(int,int,u_int n){return packet(std::vector<uint8_t>(n),24);}
'''
main = r'''
using Bytes=std::vector<uint8_t>;
static ieee80211com ic;static ieee80211_node ap;
static const u_int8_t AP[6]={2,0,0,0,0,1},ALL[6]={255,255,255,255,255,255};
// An access point as the scan left it: WPA2-PSK, CCMP, with the RSN capabilities given.
static void scanned(u_int16_t caps,u_int ciphers=IEEE80211_CIPHER_CCMP,u_int protos=IEEE80211_PROTO_RSN,enum ieee80211_cipher groupmgmt=IEEE80211_CIPHER_BIP){
 ap=ieee80211_node();memcpy(ap.ni_macaddr,AP,6);memcpy(ap.ni_bssid,AP,6);ap.ni_rsnprotos=protos;ap.ni_rsnakms=IEEE80211_AKM_PSK;ap.ni_rsnciphers=ciphers;
 ap.ni_rsngroupcipher=IEEE80211_CIPHER_CCMP;ap.ni_rsngroupmgmtcipher=groupmgmt;ap.ni_rsncaps=caps;}
static Bytes rsnie(){u_int8_t b[64];u_int8_t*e=ieee80211_add_rsn(b,&ic,&ap);assert(b[0]==IEEE80211_ELEMID_RSN&&b[1]==e-b-2);return Bytes(b,e);}
// version, group CCMP, pairwise CCMP, AKM PSK, then the capabilities and what follows
static Bytes rsn(std::initializer_list<uint8_t> tail){Bytes b={48,0,1,0,0,0x0f,0xac,4,1,0,0,0x0f,0xac,4,1,0,0,0x0f,0xac,2};b.insert(b.end(),tail);b[1]=b.size()-2;return b;}
static bool joins(){return match_rsn(&ic,&ap)==0;}
static bool mfp(){ieee80211_choose_rsnparams(&ic);return (ap.ni_flags&IEEE80211_NODE_MFP)!=0;}
// The Protected bit of a frame as ieee80211_mgmt_output() queues it for the driver.
static bool protects(int type,Bytes body,const u_int8_t*to=AP){memcpy(ap.ni_macaddr,to,6);mbuf_t m=packet(body,24);assert(ieee80211_mgmt_output(&ic.ic_if,&ap,m,type)==0);
 assert(ic.ic_mgtq.q.size()==1&&ic.ic_mgtq.q[0]==m);ic.ic_mgtq.q.clear();auto v=flat(m);mbuf_freem(m);memcpy(ap.ni_macaddr,AP,6);
 assert(v.size()==24+body.size()&&v[0]==type&&!memcmp(&v[4],to,6)&&!memcmp(&v[24],body.data(),body.size()));return (v[1]&IEEE80211_FC1_PROTECTED)!=0;}
int main(){
 ic.ic_bss=&ap;ic.ic_flags|=IEEE80211_F_PSK;ic.ic_if.if_start=[](_ifnet*){};
 const Bytes plain=rsn({0,0}),with_mfp=rsn({0x80,0, 0,0, 0,0x0f,0xac,6}),required=rsn({0xc0,0, 0,0, 0,0x0f,0xac,6});

 // --- An access point without 802.11w: joined, nothing negotiated, and the
 // element is the one a driver without 802.11w sends.
 scanned(0);assert(joins()&&!mfp()&&rsnie()==plain);
 ic.ic_caps=0;scanned(0);assert(joins()&&!mfp()&&rsnie()==plain);ic.ic_caps=IEEE80211_C_MFP;
 // --- One that is capable: negotiated, MFPC and the BIP group management cipher
 // after an empty PMKID list. Its replay counter bits are echoed as before.
 scanned(IEEE80211_RSNCAP_MFPC);assert(joins()&&mfp()&&rsnie()==with_mfp);
 scanned(IEEE80211_RSNCAP_MFPC|0x000c);assert(mfp()&&rsnie()==rsn({0x8c,0, 0,0, 0,0x0f,0xac,6}));
 // --- One that requires it: the same, since requiring is its policy, not ours.
 scanned(IEEE80211_RSNCAP_MFPC|IEEE80211_RSNCAP_MFPR);assert(joins()&&mfp()&&rsnie()==with_mfp);
 // With a cached PMK its identifier takes the place of the empty list.
 scanned(IEEE80211_RSNCAP_MFPC);assert(mfp());ap.ni_flags|=IEEE80211_NODE_PMKID;memset(ap.ni_pmkid,0x5a,16);
 {Bytes want=rsn({0x80,0, 1,0});want.insert(want.end(),16,0x5a);want.insert(want.end(),{0,0x0f,0xac,6});want[1]=want.size()-2;assert(rsnie()==want);}

 // --- Turned off (-mtknopmf): capable access points are joined without it,
 // with the old element; one that requires it is not joined.
 ic.ic_caps=0;scanned(IEEE80211_RSNCAP_MFPC);assert(joins()&&!mfp()&&rsnie()==plain);
 scanned(IEEE80211_RSNCAP_MFPC|IEEE80211_RSNCAP_MFPR);assert(!joins());ic.ic_caps=IEEE80211_C_MFP;

 // --- Required by us (-mtkpmfreq): MFPR in our element, and an access point
 // that cannot protect management frames is not joined.
 ic.ic_flags|=IEEE80211_F_MFPR;
 scanned(IEEE80211_RSNCAP_MFPC);assert(joins()&&mfp()&&rsnie()==required);
 scanned(0);assert(!joins());
 scanned(IEEE80211_RSNCAP_MFPC,IEEE80211_CIPHER_TKIP);assert(!joins());
 ic.ic_flags&=~IEEE80211_F_MFPR;

 // --- 802.11w needs CCMP between the two: with TKIP as the only pairwise
 // cipher it is not negotiated, and an access point requiring it is not joined.
 scanned(IEEE80211_RSNCAP_MFPC,IEEE80211_CIPHER_TKIP);assert(joins()&&!mfp());
 scanned(IEEE80211_RSNCAP_MFPC,IEEE80211_CIPHER_TKIP|IEEE80211_CIPHER_CCMP);assert(joins()&&mfp()&&ap.ni_rsncipher==IEEE80211_CIPHER_CCMP);
 scanned(IEEE80211_RSNCAP_MFPC|IEEE80211_RSNCAP_MFPR,IEEE80211_CIPHER_TKIP);assert(!joins());
 // Nor with a group management cipher other than BIP-CMAC-128.
 scanned(IEEE80211_RSNCAP_MFPC,IEEE80211_CIPHER_CCMP,IEEE80211_PROTO_RSN,IEEE80211_CIPHER_NONE);assert(!joins());
 // A choice made for one access point does not stick to the next.
 scanned(IEEE80211_RSNCAP_MFPC);assert(mfp());ap.ni_rsncaps=0;assert(!mfp());

 // --- What we send. Before the keys of the association are in place nothing
 // can be protected; afterwards the robust frames to the access point are.
 const Bytes addba={3,0,1,2,0x10,0,0,0x10,0},saq={8,1,0x34,0x12};
 scanned(IEEE80211_RSNCAP_MFPC);assert(mfp());
 assert(!protects(IEEE80211_FC0_SUBTYPE_DEAUTH,{3,0})&&!protects(IEEE80211_FC0_SUBTYPE_ACTION,addba));
 ap.ni_flags|=IEEE80211_NODE_TXMGMTPROT;
 assert(protects(IEEE80211_FC0_SUBTYPE_DEAUTH,{3,0})&&protects(IEEE80211_FC0_SUBTYPE_DISASSOC,{8,0}));
 assert(protects(IEEE80211_FC0_SUBTYPE_ACTION,addba)&&protects(IEEE80211_FC0_SUBTYPE_ACTION,saq)&&protects(IEEE80211_FC0_SUBTYPE_ACTION,{3,2,0,0,1,0}));
 // Not the frames 802.11w leaves alone, nor anything group addressed: the
 // hardware would be asked to encrypt it with a pairwise key.
 assert(!protects(IEEE80211_FC0_SUBTYPE_AUTH,{0,0,1,0,0,0})&&!protects(0x00,{0,0})&&!protects(0x40,{0,0}));
 for(unsigned cat:{4u,7u,21u,127u})assert(!protects(IEEE80211_FC0_SUBTYPE_ACTION,{u_int8_t(cat),0,0}));
 assert(!protects(IEEE80211_FC0_SUBTYPE_ACTION,{}));
 assert(!protects(IEEE80211_FC0_SUBTYPE_DEAUTH,{3,0},ALL)&&!protects(IEEE80211_FC0_SUBTYPE_ACTION,addba,ALL));
 // And never with 802.11w turned off, whatever the node says.
 ic.ic_caps=0;assert(!protects(IEEE80211_FC0_SUBTYPE_DEAUTH,{3,0})&&!protects(IEEE80211_FC0_SUBTYPE_ACTION,addba));ic.ic_caps=IEEE80211_C_MFP;

 // SA Query frames carry the transaction they are told, two octets.
 {mbuf_t m=ieee80211_get_sa_query(&ic,&ap,IEEE80211_ACTION_SA_QUERY_RESP,0x1234);assert(m&&flat(m)==(Bytes{8,1,0x34,0x12}));mbuf_freem(m);}
 assert(live==0);
 puts("PASS production 802.11w negotiation and transmit: element unchanged towards access points without it, MFPC and BIP cipher towards capable ones (with and without PMKID), off and required policies in the join decision, CCMP and BIP-CMAC-128 needed, no stale choice; Protected bit only for robust frames to the access point once its keys are installed");
}
'''
ref.run('mfp-association', ref.MBUF + ref.net80211() + stubs + code + main)
