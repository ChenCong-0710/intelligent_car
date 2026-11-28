"""Host check of the actual sonar polling function with register stubs.
Checks software timing/result ownership, not electrical echo correctness.
"""
from pathlib import Path
import subprocess,runpy
r=Path(__file__).resolve().parents[1]
s=(r/'Core/Src/main.c').read_text(encoding='utf-8')
f=s[s.index('static void ultrasound(uint32_t now)'):s.index('\n#define LCD_CS_LOW')]
stub=r"""
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#define TIM_SR_CC2IF 4U
#define TIM_SR_CC2OF 1024U
#define TIM_CR1_CEN 1U
#define SET_BIT(r,b) ((r)|=(b))
#define ALIGN_SCAN 1
#define BC_SIDE_SCAN 14
struct {int phase;struct {unsigned triggers;uint32_t since;} side_scan;} g_box_center;
struct Timer {uint32_t SR,CCR2,CNT,CR1;} echo_timer,trigger_timer;
#define TIM8 (&echo_timer)
#define TIM17 (&trigger_timer)
struct {int phase;uint32_t since;unsigned scan_trigger_count;} g_align;
static uint32_t us_trigger_ms,us_echo_ms,us_seq,us_result_trigger_ms,us_period_ms,us_result_period_ms;
static uint16_t us_pulse_us,us_mm;
static bool us_valid,us_pending;
static const char *us_reason;
"""
tests=r"""
int main(void){
 TIM8->SR=0;ultrasound(60);assert(us_pending&&us_trigger_ms==60);
 TIM8->SR=TIM_SR_CC2IF;TIM8->CCR2=1000;ultrasound(61);
 assert(us_seq==1&&us_valid&&us_result_trigger_ms==60&&us_pulse_us==1000&&us_mm==172);
 TIM8->SR=0;ultrasound(120);
 assert(us_pending&&us_seq==1&&us_valid&&us_result_trigger_ms==60&&us_trigger_ms==120);
 TIM8->SR=TIM_SR_CC2IF;TIM8->CCR2=1200;ultrasound(121);
 assert(us_seq==2&&us_result_trigger_ms==120&&us_result_period_ms==60);
 TIM8->SR=0;ultrasound(180);TIM8->SR=0;ultrasound(206);
 assert(us_seq==3&&!us_valid&&us_pulse_us==0&&strcmp(us_reason,"TIMEOUT")==0);
 TIM8->SR=0;ultrasound(240);TIM8->SR=TIM_SR_CC2IF|TIM_SR_CC2OF;TIM8->CCR2=1000;ultrasound(241);
 assert(us_seq==4&&!us_valid&&strcmp(us_reason,"OVERCAP")==0);
 us_pending=false;us_trigger_ms=300;g_align.phase=ALIGN_SCAN;g_align.since=300;g_align.scan_trigger_count=0;
 for(unsigned t=360;t<=540;t+=60){TIM8->SR=0;ultrasound(t);assert(us_trigger_ms==t);}
 assert(g_align.scan_trigger_count==4);TIM8->SR=0;ultrasound(600);assert(us_trigger_ms==540);
 g_align.phase=0;TIM8->SR=0;ultrasound(600);assert(us_trigger_ms==600);
 us_pending=false;us_trigger_ms=600;g_box_center.phase=BC_SIDE_SCAN;g_box_center.side_scan.since=600;
 for(unsigned t=660;t<=840;t+=60){TIM8->SR=0;ultrasound(t);assert(us_trigger_ms==t);}
 assert(g_box_center.side_scan.triggers==4);TIM8->SR=0;ultrasound(900);assert(us_trigger_ms==840);
 g_box_center.phase=0;
 us_pending=false;us_trigger_ms=UINT32_MAX-29U;TIM8->SR=0;ultrasound(30);
 assert(us_trigger_ms==30&&us_period_ms==60);
 puts("PASS: 60ms scheduling, immutable completed echo across next trigger, timeout/overcapture results, four-slot scan and tick wrap");
}
"""
p=r/'work/tests/test_sonar_transactions.c';p.write_text(stub+f+tests)
exe=p.with_suffix('.exe')
subprocess.run([r'E:/mingw/mingw64/bin/gcc.exe','-std=c11','-Wall','-Wextra',str(p),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
parse=runpy.run_path(str(r/'tools/capture_stage1.py'))['parse']
assert parse('U,fw=BOX_DIAG_V7,seq=1,dt=60,why=OK,brL=1')['brL']==1
assert parse('M,fw=BOX_REV25_V6,seq=1')['fw']=='BOX_REV25_V6'
print('PASS: raw sonar trace and prior firmware log parsing')
