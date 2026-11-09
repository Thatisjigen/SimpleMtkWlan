#!/usr/bin/env python3
"""Production association comeback and authentication again while associating.

ieee80211_sta_assoc_comeback() and its timeout, sliced from
ieee80211_proto.c, and ieee80211_sta_deauthenticated() from
ieee80211_input.c: an association refused for now (status 30) is asked
again once the comeback time of the Timeout Interval element has passed,
three times at most in a join; a Deauthentication with reason 6 or 7 while
associating sends a new authentication, twice at most, without telling the
front end; everything else ends the join as before.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
import reference as ref

inp = (ref.NET / 'ieee80211_input.c').read_text()
proto = (ref.NET / 'ieee80211_proto.c').read_text()
code = (ref.between(proto, '#define IEEE80211_TIE_ASSOC_COMEBACK', '\nvoid\nieee80211_ht_negotiate_chw(') +
        ref.extract(inp, 'ieee80211_sta_deauthenticated') + '\n')
recv = ref.extract(inp, 'ieee80211_recv_assoc_resp')
assert recv.index('status == IEEE80211_STATUS_TRY_AGAIN_LATER') < recv.index('associd = LE_READ_2(frm)')
assert 'ieee80211_sta_assoc_comeback(ic, frm + 2, efrm);' in recv

stubs = r'''
#define IEEE80211_STA_ONLY 1
#define IEEE80211_ELEMID_TIE 56
#define IEEE80211_TRANS_WAIT 5
#define IEEE80211_FC0_SUBTYPE_ASSOC_REQ 0x00
#define IEEE80211_EVT_STA_DEAUTH 7
#define LE_READ_4(p) ((u_int32_t)LE_READ_2(p)|((u_int32_t)LE_READ_2((const u_int8_t*)(p)+2)<<16))
struct Trans{int state,mgt;};static std::vector<Trans> transitions;static std::vector<unsigned> events;
static int ieee80211_new_state(ieee80211com*,int s,int mgt){transitions.push_back({s,mgt});return 0;}
static std::vector<int> sent;
#define IEEE80211_SEND_MGMT(ic,ni,type,arg) (assert((ni)==(ic)->ic_bss),sent.push_back(type),0)
static int armed_ms=-1;
static int timeout_add_msec(CTimeout**,int ms){armed_ms=ms;return 1;}
'''
main = r'''
static ieee80211com ic;static ieee80211_node ni;
using Bytes=std::vector<uint8_t>;
// The elements after the AID field; with none, both ends point past it.
static const u_int8_t past[1]={};
static void refuse(const Bytes&ies){const u_int8_t*b=ies.empty()?past:ies.data();ieee80211_sta_assoc_comeback(&ic,b,b+ies.size());}
static Bytes tie(unsigned type,u_int32_t tu){return {56,5,u_int8_t(type),u_int8_t(tu),u_int8_t(tu>>8),u_int8_t(tu>>16),u_int8_t(tu>>24)};}
static void fire(){assert(armed_ms>=0);armed_ms=-1;ieee80211_sta_assoc_comeback_timeout(&ic);}
int main(){
 ic.ic_bss=&ni;ic.ic_state=IEEE80211_S_ASSOC;
 ic.ic_event_handler=[](ieee80211com*c,int code,void*){events.push_back(code<<16|c->ic_deauth_reason);};
 auto&st=ic.ic_stats;

 // --- Refused for now: wait the comeback time, here 2000 TU after other elements,
 // with the join's timer covering it; then the request goes out again.
 Bytes ies={1,2,0x82,0x84};Bytes t=tie(3,2000);ies.insert(ies.end(),t.begin(),t.end());
 refuse(ies);assert(armed_ms==2049&&ic.ic_assoc_comeback&&ic.ic_assoc_comebacks==1&&st.is_assoc_comeback==1);
 assert(ic.ic_mgt_timer==2+1+IEEE80211_TRANS_WAIT&&ic.ic_if.if_timer==1&&sent.empty());
 // Another refusal while waiting changes nothing.
 refuse(ies);assert(armed_ms==2049&&ic.ic_assoc_comebacks==1);
 fire();assert(sent==std::vector<int>{IEEE80211_FC0_SUBTYPE_ASSOC_REQ}&&!ic.ic_assoc_comeback);sent.clear();
 // Without the element, or with another interval type, 1000 TU; a value
 // above 10000 TU is capped; a truncated element list is read up to its end.
 refuse({});assert(armed_ms==1025);fire();sent.clear();
 Bytes other=tie(2,7);refuse(other);assert(armed_ms==1025&&ic.ic_assoc_comebacks==3);fire();sent.clear();
 ic.ic_assoc_comebacks=0;
 refuse(tie(3,0xffffffffu));assert(armed_ms==10241);fire();sent.clear();
 Bytes cut=tie(3,500);cut.pop_back();refuse(cut);assert(armed_ms==1025);fire();sent.clear();
 Bytes nul={56,0};refuse(nul);assert(armed_ms==1025);fire();sent.clear();
 assert(ic.ic_assoc_comebacks==3&&st.is_assoc_comeback==6);
 // Three times in a join at most: then the join times out as before.
 refuse(tie(3,100));assert(armed_ms==-1&&!ic.ic_assoc_comeback&&st.is_assoc_comeback==6);
 // Nothing outside an association attempt, and nothing sent if the state moved on.
 ic.ic_assoc_comebacks=0;ic.ic_state=IEEE80211_S_RUN;refuse({});assert(armed_ms==-1);
 ic.ic_state=IEEE80211_S_ASSOC;refuse({});ic.ic_state=IEEE80211_S_AUTH;fire();assert(sent.empty()&&!ic.ic_assoc_comeback);
 ic.ic_state=IEEE80211_S_ASSOC;ic.ic_opmode=IEEE80211_M_HOSTAP;refuse({});assert(armed_ms==-1);ic.ic_opmode=IEEE80211_M_STA;
 assert(transitions.empty()&&events.empty());

 // --- Dropped while associating (reason 6 or 7): authenticate again at once,
 // the front end is not told.
 ieee80211_sta_deauthenticated(&ic,IEEE80211_REASON_NOT_AUTHED);
 assert(transitions.size()==1&&transitions[0].state==IEEE80211_S_AUTH&&transitions[0].mgt==IEEE80211_FC0_SUBTYPE_AUTH);
 assert(events.empty()&&ic.ic_assoc_reauths==1&&st.is_assoc_reauth==1);
 ieee80211_sta_deauthenticated(&ic,IEEE80211_REASON_NOT_ASSOCED);
 assert(transitions.size()==2&&transitions[1].mgt==IEEE80211_FC0_SUBTYPE_AUTH&&events.empty()&&ic.ic_assoc_reauths==2);
 // Twice at most: then, as before, the front end hears of it and the join ends.
 ieee80211_sta_deauthenticated(&ic,IEEE80211_REASON_NOT_AUTHED);
 assert(transitions.size()==3&&transitions[2].mgt==IEEE80211_FC0_SUBTYPE_DEAUTH);
 assert(events==std::vector<unsigned>{IEEE80211_EVT_STA_DEAUTH<<16|IEEE80211_REASON_NOT_AUTHED}&&st.is_assoc_reauth==2);
 transitions.clear();events.clear();ic.ic_assoc_reauths=0;
 // Another reason, or the same reasons in RUN or AUTH: as before.
 ieee80211_sta_deauthenticated(&ic,IEEE80211_REASON_AUTH_LEAVE);
 ic.ic_state=IEEE80211_S_RUN;ieee80211_sta_deauthenticated(&ic,IEEE80211_REASON_NOT_AUTHED);
 ic.ic_state=IEEE80211_S_AUTH;ieee80211_sta_deauthenticated(&ic,IEEE80211_REASON_NOT_ASSOCED);
 assert(transitions.size()==3&&events.size()==3&&ic.ic_assoc_reauths==0);
 for(auto&x:transitions)assert(x.state==IEEE80211_S_AUTH&&x.mgt==IEEE80211_FC0_SUBTYPE_DEAUTH);
 puts("PASS production association comeback: refused for now waits the Timeout Interval's comeback time (default 1000 TU, capped at 10000 TU, malformed elements tolerated), the join timer covers it, three times per join at most, nothing outside ASSOC; reason 6/7 while associating authenticates again twice at most without telling the front end, everything else ends the join as before");
}
'''
ref.run('assoc-comeback', ref.MBUF + ref.net80211() + stubs + code + main)
