#include "initial_align.h"
#include <stdio.h>
#include <string.h>
int main(void){InitialAlign a;
{uint16_t v[37]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,166,0,0,0,0,0,0,0,0,0,0,0,0,0};memset(&a,0,sizeof a);memcpy(a.range,v,sizeof v);int ok=align_fit(&a);printf("proxy_run=0 ok=%d normal=%.1f rms=%.1f edges=%u..%u\n",ok,a.normal_deg,a.rms_mm,a.fit_first,a.fit_last);}
{uint16_t v[37]={0,0,0,0,0,142,173,137,111,137,107,107,107,143,104,139,118,114,114,128,144,183,174,0,0,0,0,0,0,0,0,0,0,0,0,0,0};memset(&a,0,sizeof a);memcpy(a.range,v,sizeof v);int ok=align_fit(&a);printf("proxy_run=1 ok=%d normal=%.1f rms=%.1f edges=%u..%u\n",ok,a.normal_deg,a.rms_mm,a.fit_first,a.fit_last);}
{uint16_t v[37]={0,0,0,0,0,142,125,136,128,137,107,107,107,121,104,139,118,114,123,128,136,183,174,0,0,0,0,0,0,0,0,0,0,0,0,0,0};memset(&a,0,sizeof a);memcpy(a.range,v,sizeof v);int ok=align_fit(&a);printf("proxy_run=2 ok=%d normal=%.1f rms=%.1f edges=%u..%u\n",ok,a.normal_deg,a.rms_mm,a.fit_first,a.fit_last);}
{uint16_t v[37]={0,0,0,0,0,158,146,120,111,111,111,118,107,107,132,104,128,135,114,128,150,160,170,0,0,0,0,0,0,0,0,0,0,0,0,0,0};memset(&a,0,sizeof a);memcpy(a.range,v,sizeof v);int ok=align_fit(&a);printf("proxy_run=3 ok=%d normal=%.1f rms=%.1f edges=%u..%u\n",ok,a.normal_deg,a.rms_mm,a.fit_first,a.fit_last);}
return 0;}