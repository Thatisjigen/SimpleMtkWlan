from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[2]
out=Path(__file__).resolve().parent
s=(root/'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()
def fn(name):
    a=s.index('\n'+name+'(');a=s.rindex('\n',0,a)+1
    return s[a:s.index('\n}\n',a)+3]
for plist in ('AirportMtkWlan/WCL/Info.plist','SimpleMtkWlan/Info.plist'):
    assert '0x792014c3' in (root/plist).read_text(),plist
assert '{ PCI_VENDOR_MEDIATEK, PCI_PRODUCT_MEDIATEK_MT7920 },' in s
fw=(root/'include/FwBinary.cpp').read_text()
assert 'MTK_FW_DESC("mwx-mt7961_patch_mcu_1a_2_hdr"' in fw and 'MTK_FW_DESC("mwx-mt7961_ram_code_1a"' in fw
pre=r'''
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <cstring>
enum mwx_hw_type{MWX_HW_MT7921,MWX_HW_MT7922,MWX_HW_MT7920};
#define MT_HW_BOUND_MT7920 (1U<<7)
'''
for line in s.splitlines():
    if line.startswith(('#define	MT7920_','#define	MT7921_','#define	MT7922_')) and ('ROM_PATCH' in line or 'FIRMWARE_WM' in line):
        pre+=line+'\n'
post=r'''
static void firmware(mwx_hw_type t,const char *patch,const char *ram){
 const char *p=nullptr,*r=nullptr;mwx_hw_firmware(t,&p,&r);
 assert(!strcmp(p,patch));assert(!strcmp(r,ram));
}
int main(){
 // Linux mt7921_pci_probe(): chip 0x7961 with HW_BOUND bit 7 is MT7920, whatever the PCI ID says.
 assert(mwx_hw_type_from_chip(MWX_HW_MT7920,0x7961,0x80)==MWX_HW_MT7920);
 assert(mwx_hw_type_from_chip(MWX_HW_MT7921,0x7961,0xffffffff)==MWX_HW_MT7920);
 assert(mwx_hw_type_from_chip(MWX_HW_MT7921,0x7961,0)==MWX_HW_MT7921);
 assert(mwx_hw_type_from_chip(MWX_HW_MT7920,0x7961,0x7f)==MWX_HW_MT7921);
 // MT7922 and a dead read keep the type from the PCI ID.
 assert(mwx_hw_type_from_chip(MWX_HW_MT7922,0x7922,0x80)==MWX_HW_MT7922);
 assert(mwx_hw_type_from_chip(MWX_HW_MT7920,0xffff,0xffffffff)==MWX_HW_MT7920);
 // Each type its own pair: no fallthrough from MT7920 to the ordinary MT7961 files.
 firmware(MWX_HW_MT7920,"mwx-mt7961_patch_mcu_1a_2_hdr","mwx-mt7961_ram_code_1a");
 firmware(MWX_HW_MT7921,"mwx-mt7961_patch_mcu_1_2_hdr","mwx-mt7961_ram_code_1");
 firmware(MWX_HW_MT7922,"mwx-mt7922_patch_mcu_1_1_hdr","mwx-mt7922_ram_code_1");
 assert(!strcmp(mwx_hw_name(MWX_HW_MT7920),"MT7920"));
 assert(!strcmp(mwx_hw_name(MWX_HW_MT7921),"MT7921"));
 assert(!strcmp(mwx_hw_name(MWX_HW_MT7922),"MT7922"));
 puts("PASS MT7920 detection as in Linux, firmware pair per type, PCI ID and table entries.");}
'''
p=out/'test-mt7920.cpp';p.write_text(pre+fn('mwx_hw_name')+'\n'+fn('mwx_hw_type_from_chip')+'\n'+fn('mwx_hw_firmware')+'\n'+post)
subprocess.run(['clang++','-std=c++14','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
subprocess.run([str(p.with_suffix(''))],check=True)
