#!/usr/bin/env python3
"""Actual A-MPDU reordering functions and the real dispatch condition of
ieee80211_inputm(): frames sent with the HTP Ack policy must go through the
reordering buffer like the others, or a burst with a gap is delivered out of
order and the packet number check that follows drops good frames."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[2]; net=root/'mtk80211/openbsd/net80211'
inp=(net/'ieee80211_input.c').read_text(); h=(net/'ieee80211.h').read_text()
nodeh=(net/'ieee80211_node.h').read_text(); priv=(net/'ieee80211_priv.h').read_text()
def fn(name):
 m=re.search(r'(?m)^(?:void|int)\n'+name+r'\(',inp);assert m,name
 a=inp.index('{',m.start());pos=a+1;depth=1
 while depth:depth+=(inp[pos]=='{')-(inp[pos]=='}');pos+=1
 return inp[m.start():pos]
def define(src,name):
 m=re.search(r'(?m)^#define\s+'+name+r'\b((?:[^\n]*\\\n)*[^\n]*)',src);assert m,name
 return m.group()
# The statement that decides whether a QoS data frame is reordered, as it stands in the source.
body=fn('ieee80211_inputm');a=body.index('if (ba_state == IEEE80211_BA_AGREED &&')
b=body.index('}',body.index("return;    /* don't free m! */",a))+1
dispatch=body[a:b]
assert 'IEEE80211_QOS_ACK_POLICY_NOACK' in dispatch and 'IEEE80211_QOS_ACK_POLICY_NORMAL' not in dispatch, dispatch
# The condition before the fix, to show that this burst does catch the bug.
previous,n=re.subn(r'\(qos & IEEE80211_QOS_ACK_POLICY_MASK\) !=\s*IEEE80211_QOS_ACK_POLICY_NOACK',
 '((qos & IEEE80211_QOS_ACK_POLICY_MASK) == IEEE80211_QOS_ACK_POLICY_BA || (qos & IEEE80211_QOS_ACK_POLICY_MASK) == IEEE80211_QOS_ACK_POLICY_NORMAL)',dispatch)
assert n==1
defines='\n'.join([define(h,x) for x in ['IEEE80211_QOS_ACK_POLICY_NORMAL','IEEE80211_QOS_ACK_POLICY_NOACK','IEEE80211_QOS_ACK_POLICY_NOEXPLACK','IEEE80211_QOS_ACK_POLICY_BA','IEEE80211_QOS_ACK_POLICY_MASK','IEEE80211_SEQ_SEQ_SHIFT']]
 +[define(nodeh,x) for x in ['IEEE80211_BA_INIT','IEEE80211_BA_REQUESTED','IEEE80211_BA_AGREED','IEEE80211_BA_MAX_WINSZ','IEEE80211_BA_GAP_TIMEOUT','IEEE80211_BA_MAX_WINMISS','IEEE80211_RXI_AMPDU_DONE']]
 +[define(priv,'SEQ_LT')])
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
using u_int8_t=uint8_t;using u_int16_t=uint16_t;using u_int32_t=uint32_t;
// A received MPDU: just what the reordering code reads (the sequence number in
// the 802.11 header) plus what the test needs to judge the delivery.
struct mbuf_s{uint8_t hdr[26]={};uint16_t sn=0,qos=0;uint64_t pn=0;};
using mbuf_t=mbuf_s*;
#define mtod(m,t) ((t)(m)->hdr)
#define _KASSERT(x) assert(x)
#define letoh16(x) (x)
#define XYLog(...) ((void)0)
struct ieee80211_frame{uint8_t i_fc[2],dur[2],a1[6],a2[6],a3[6],i_seq[2];};
static bool ieee80211_has_seq(const ieee80211_frame*){return true;}
struct ieee80211_rxinfo{uint32_t rxi_flags=0;};
struct ieee80211_ba_buf{mbuf_t m=nullptr;ieee80211_rxinfo rxi;};
struct CTimeout{};
struct ieee80211_node;
struct ieee80211_rx_ba{ieee80211_node*ba_ni=nullptr;ieee80211_ba_buf*ba_buf=nullptr;CTimeout*ba_to=nullptr;int ba_timeout_val=0,ba_state=0;
 u_int16_t ba_params=0,ba_winstart=0,ba_winend=0,ba_winsize=0,ba_head=0;CTimeout*ba_gap_to=nullptr;int ba_gapwait=0,ba_winmiss=0;uint16_t ba_missedsn=0;};
struct netstat{unsigned inputErrors=0;};
struct _ifnet{netstat stat;netstat*netStat=&stat;};
struct mbuf_list{};
#define MBUF_LIST_INITIALIZER() {}
struct ieee80211com{_ifnet ic_if;struct{unsigned is_ht_rx_frame_below_ba_winstart=0,is_ht_rx_ba_no_buf=0,is_ht_rx_frame_above_ba_winend=0,
 is_ht_rx_ba_window_jump=0,is_ht_rx_ba_window_slide=0,is_ht_rx_ba_frame_lost=0,is_ht_rx_ba_window_gap_timeout=0,is_ht_rx_ba_htp_ack=0;}ic_stats;};
struct ieee80211_node{ieee80211com*ni_ic=nullptr;ieee80211_rx_ba ni_rx_ba[16];};
static bool gap_armed=false;
static void timeout_add_usec(CTimeout**,int){}
static void timeout_add_msec(CTimeout**,int){gap_armed=true;}
static bool timeout_pending(CTimeout**){return gap_armed;}
static void timeout_del(CTimeout**){gap_armed=false;}
static int splnet(){return 0;} static void splx(int){}
static void if_input(_ifnet*,mbuf_list*){}
// Where frames end up: the packet number check of the replay protection, per
// TID and strictly increasing, as ieee80211_input_hwdecrypt() applies it.
static std::vector<uint16_t> delivered;static unsigned replays=0,freed=0;static uint64_t last_pn[16];
static void deliver(mbuf_t m){unsigned tid=m->qos&15;if(m->pn<=last_pn[tid])++replays;else{last_pn[tid]=m->pn;delivered.push_back(m->sn);}delete m;}
static void mbuf_freem(mbuf_t m){++freed;delete m;}
static void ieee80211_inputm(_ifnet*,mbuf_t m,ieee80211_node*,ieee80211_rxinfo*,mbuf_list*){deliver(m);}
void ieee80211_input_ba_seq(ieee80211com*,ieee80211_node*,uint8_t,uint16_t,mbuf_list*);
void ieee80211_input_ba_flush(ieee80211com*,ieee80211_node*,ieee80211_rx_ba*,mbuf_list*);
void ieee80211_ba_move_window(ieee80211com*,ieee80211_node*,u_int8_t,u_int16_t,mbuf_list*);
void ieee80211_input_ba(ieee80211com*,mbuf_t,ieee80211_node*,int,ieee80211_rxinfo*,mbuf_list*);
'''
post=r'''
// The dispatch of ieee80211_inputm() for a QoS data frame in RUN state.
static void rx(ieee80211com*ic,ieee80211_node*ni,mbuf_t m){
 mbuf_list list;mbuf_list*ml=&list;ieee80211_rxinfo info;ieee80211_rxinfo*rxi=&info;
 const u_int16_t qos=m->qos;const int tid=qos&15;const int ba_state=ni->ni_rx_ba[tid].ba_state;
 DISPATCH
 deliver(m);
}
static mbuf_t mk(uint16_t sn,uint16_t policy,unsigned tid=0){auto*f=new mbuf_s;f->sn=sn;f->qos=tid|policy;f->pn=5000+sn;f->hdr[0]=0x88;
 uint16_t seq=sn<<IEEE80211_SEQ_SEQ_SHIFT;memcpy(f->hdr+22,&seq,2);return f;}
struct World{ieee80211com ic;ieee80211_node ni;ieee80211_ba_buf buf[IEEE80211_BA_MAX_WINSZ];
 World(int state){ni.ni_ic=&ic;auto&ba=ni.ni_rx_ba[0];ba.ba_ni=&ni;ba.ba_buf=buf;ba.ba_state=state;ba.ba_winstart=100;ba.ba_winsize=64;ba.ba_winend=163;
  delivered.clear();replays=freed=0;memset(last_pn,0,sizeof last_pn);gap_armed=false;}
 void drain(){auto&ba=ni.ni_rx_ba[0];for(int i=0;i<200&&ba.ba_gapwait>0;++i)ieee80211_input_ba_gap_timeout(&ba);assert(ba.ba_gapwait==0);}};
int main(){
 // The burst seen on air: frame 100 never arrives, 101..140 do, and every
 // eighth of them carries the HTP Ack policy. Then the gap timeout fires.
 {World w(IEEE80211_BA_AGREED);
  for(uint16_t sn=101;sn<=140;++sn)rx(&w.ic,&w.ni,mk(sn,sn%8==0?IEEE80211_QOS_ACK_POLICY_NOEXPLACK:IEEE80211_QOS_ACK_POLICY_NORMAL));
#ifndef PREVIOUS
  assert(delivered.empty()&&w.ni.ni_rx_ba[0].ba_gapwait==40); /* all 40 wait behind the gap, HTP Ack ones included */
  w.drain();
  assert(replays==0&&freed==0&&delivered.size()==40);for(unsigned i=0;i<40;++i)assert(delivered[i]==101+i);
  assert(w.ic.ic_stats.is_ht_rx_ba_htp_ack==5&&w.ic.ic_stats.is_ht_rx_ba_frame_lost==1&&w.ic.ic_stats.is_ht_rx_ba_window_gap_timeout==1);
#else
  assert(delivered.size()==5&&delivered[4]==136);            /* the five HTP Ack frames went ahead */
  w.drain();
  assert(replays==31&&delivered.size()==5+4);                 /* 101..135 less the four HTP ones are lost; only 137..140 pass */
#endif
 }
#ifndef PREVIOUS
 // Block Ack policy is reordered too; No Ack is not part of the agreement and is handed on at once.
 {World w(IEEE80211_BA_AGREED);rx(&w.ic,&w.ni,mk(101,IEEE80211_QOS_ACK_POLICY_BA));assert(delivered.empty()&&w.ni.ni_rx_ba[0].ba_gapwait==1);
  rx(&w.ic,&w.ni,mk(100,IEEE80211_QOS_ACK_POLICY_NORMAL));assert(delivered.size()==2&&delivered[0]==100&&delivered[1]==101&&w.ni.ni_rx_ba[0].ba_winstart==102);
  rx(&w.ic,&w.ni,mk(300,IEEE80211_QOS_ACK_POLICY_NOACK));assert(delivered.size()==3&&delivered[2]==300&&w.ni.ni_rx_ba[0].ba_gapwait==0&&w.ni.ni_rx_ba[0].ba_winstart==102);
  assert(replays==0&&freed==0&&w.ic.ic_stats.is_ht_rx_ba_htp_ack==0);}
 // Without an agreement nothing is buffered, whatever the policy.
 {World w(IEEE80211_BA_INIT);rx(&w.ic,&w.ni,mk(101,IEEE80211_QOS_ACK_POLICY_NOEXPLACK));rx(&w.ic,&w.ni,mk(102,IEEE80211_QOS_ACK_POLICY_NORMAL));
  assert(delivered.size()==2&&w.ni.ni_rx_ba[0].ba_gapwait==0&&w.ic.ic_stats.is_ht_rx_ba_htp_ack==0);}
 // In order and complete, an HTP Ack frame costs nothing: each frame is released as it arrives.
 {World w(IEEE80211_BA_AGREED);for(uint16_t sn=100;sn<=110;++sn){rx(&w.ic,&w.ni,mk(sn,sn==105?IEEE80211_QOS_ACK_POLICY_NOEXPLACK:IEEE80211_QOS_ACK_POLICY_NORMAL));assert(delivered.size()==unsigned(sn-99));}
  assert(replays==0&&w.ni.ni_rx_ba[0].ba_winstart==111&&w.ic.ic_stats.is_ht_rx_ba_htp_ack==1);}
 puts("PASS actual reordering with the real dispatch condition: burst with a gap and HTP Ack frames delivered in order without replays; Block Ack reordered, No Ack and no-agreement frames passed on");
#else
 puts("PASS previous condition reproduces the failure: 31 good frames dropped as replays");
#endif
}
'''
functions='\n'.join(fn(x) for x in ['ieee80211_input_ba','ieee80211_input_ba_seq','ieee80211_input_ba_flush','ieee80211_input_ba_gap_skip','ieee80211_input_ba_gap_timeout','ieee80211_ba_move_window'])
with tempfile.TemporaryDirectory(prefix='mtk-reorder-ackpol-') as td:
 p=Path(td)
 for name,block,flags in [('fixed',dispatch,[]),('previous',previous,['-DPREVIOUS'])]:
  (p/(name+'.cpp')).write_text(pre+defines+'\n'+functions+post.replace('DISPATCH',block))
  subprocess.run(['clang++','-std=c++14','-fsanitize=undefined','-fno-sanitize-recover=all','-g']+flags+[str(p/(name+'.cpp')),'-o',str(p/name)],check=True)
  subprocess.run([str(p/name)],check=True)
