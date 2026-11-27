
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
struct Timer {uint32_t SR,CCR2,CNT,CR1;} echo_timer,trigger_timer;
#define TIM8 (&echo_timer)
#define TIM17 (&trigger_timer)
struct {int phase;uint32_t since;unsigned scan_trigger_count;} g_align;
static uint32_t us_trigger_ms,us_echo_ms,us_seq,us_result_trigger_ms,us_period_ms,us_result_period_ms;
static uint16_t us_pulse_us,us_mm;
static bool us_valid,us_pending;
static const char *us_reason;
static void ultrasound(uint32_t now) {
    if(TIM8->SR&TIM_SR_CC2IF) {
        uint32_t sr=TIM8->SR;uint16_t pulse=(uint16_t)TIM8->CCR2;
        TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
        if(us_pending && now-us_trigger_ms<=25U) {
            ++us_seq;us_echo_ms=now;us_pending=false;
            us_result_trigger_ms=us_trigger_ms;us_result_period_ms=us_period_ms;
            us_pulse_us=pulse;
            us_valid=!(sr&TIM_SR_CC2OF)&&pulse>=234U&&pulse<=23324U;
            us_mm=(uint16_t)(((uint32_t)pulse*343U+1000U)/2000U);
            us_reason=(sr&TIM_SR_CC2OF)?"OVERCAP":pulse<234U?"SHORT":pulse>23324U?"LONG":"OK";
        }
    }
    if(us_pending && now-us_trigger_ms>25U) {
        us_pending=false;us_valid=false;us_mm=0;us_pulse_us=0;++us_seq;us_echo_ms=now;
        us_result_trigger_ms=us_trigger_ms;us_result_period_ms=us_period_ms;us_reason="TIMEOUT";
    }
    bool scan=g_align.phase==ALIGN_SCAN;
    bool due=scan?(g_align.scan_trigger_count<4U &&
        now-g_align.since>=60U*(1U+g_align.scan_trigger_count)):true;
    if(due && now-us_trigger_ms>=60U) {
        if(scan)++g_align.scan_trigger_count;
        us_period_ms=now-us_trigger_ms;us_trigger_ms=now;us_pending=true;
        TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
        TIM17->CNT=0;SET_BIT(TIM17->CR1,TIM_CR1_CEN);
    }
}

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
 us_pending=false;us_trigger_ms=UINT32_MAX-29U;TIM8->SR=0;ultrasound(30);
 assert(us_trigger_ms==30&&us_period_ms==60);
 puts("PASS: 60ms scheduling, immutable completed echo across next trigger, timeout/overcapture results, four-slot scan and tick wrap");
}
