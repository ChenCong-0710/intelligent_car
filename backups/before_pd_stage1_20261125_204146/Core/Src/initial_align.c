#include "initial_align.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define RAD 0.0174532925199433f
static float scan_cos[ALIGN_POINTS], scan_sin[ALIGN_POINTS];
static float angle(unsigned i) { return (90.0f-5.0f*(float)i)*RAD; }
static uint16_t pulse(unsigned i) {
    /* Calibrated piecewise mapping: +90=2400us, 0=1460us, -90=500us.
       One straight line between endpoints would give an incorrect 1450us centre. */
    if (i <= 18U)
        return (uint16_t)((float)ALIGN_LEFT_US +
            ((float)ALIGN_CENTRE_US-ALIGN_LEFT_US)*(float)i/18.0f+0.5f);
    return (uint16_t)((float)ALIGN_CENTRE_US +
        ((float)ALIGN_RIGHT_US-ALIGN_CENTRE_US)*(float)(i-18U)/18.0f+0.5f);
}
static void fail(InitialAlign *a, AlignError e) {
    a->error=e; a->phase=ALIGN_FAILED; a->speed[0]=a->speed[1]=0;
}
void align_start(InitialAlign *a, uint32_t now) {
    memset(a,0,sizeof(*a)); a->phase=ALIGN_SCAN; a->since=a->started=now;
    a->servo_us=ALIGN_LEFT_US;
}
/* Fit inverse range over a candidate continuous patch. Reject ill-conditioned,
   grazing and mixed-plane patches using individual range residuals. */
static bool patch(const InitialAlign *a, unsigned lo, unsigned hi,
                  float *phi, float *rho, float *rms) {
    float cc=0,ss=0,cs=0,cy=0,sy=0;
    for (unsigned i=lo;i<=hi;++i) {
        if (!a->range[i]) return false;
        float c=scan_cos[i],s=scan_sin[i],y=1.0f/a->range[i];
        cc+=c*c;ss+=s*s;cs+=c*s;cy+=c*y;sy+=s*y;
    }
    float det=cc*ss-cs*cs;
    if (det<0.0001f) return false;
    float x=(cy*ss-sy*cs)/det,y=(sy*cc-cy*cs)/det;
    *rho=1.0f/hypotf(x,y); *phi=atan2f(y,x);
    if (!isfinite(*rho) || *rho<40 || *rho>400 || fabsf(*phi)>80*RAD) return false;
    float sum=0,band=fmaxf(3.0f,0.025f*(*rho));
    for (unsigned i=lo;i<=hi;++i) {
        float c=(*rho)*(x*scan_cos[i]+y*scan_sin[i]);
        if (c<0.35f) return false;
        float e=(float)a->range[i]-*rho/c;
        if (fabsf(e)>band) return false;
        sum+=e*e;
    }
    *rms=sqrtf(sum/(float)(hi-lo+1));
    /* Normal should lie in or near the observed angular patch. */
    return *phi>=angle(hi)-10*RAD && *phi<=angle(lo)+10*RAD;
}
bool align_fit(InitialAlign *a) {
    for(unsigned i=0;i<ALIGN_POINTS;++i) {
        scan_cos[i]=cosf(angle(i));scan_sin[i]=sinf(angle(i));
    }
    unsigned best_n=0,blo=0,bhi=0;float best_rho=1e9f,bphi=0,brms=0;
    /* Windows permit a continuous corner to split into two planes; no single
       minimum-range reading can authorize rotation. Prefer supported patches. */
    for (unsigned lo=0;lo+3<ALIGN_POINTS;++lo) {
        for (unsigned hi=lo;hi<ALIGN_POINTS;++hi) {
            if (!a->range[hi]) break;
            if (hi>lo && fabsf((float)a->range[hi]-a->range[hi-1])>
                fmaxf(30,0.25f*fminf(a->range[hi],a->range[hi-1]))) break;
            if (hi-lo<3) continue;
            float phi,rho,rms;
            if (!patch(a,lo,hi,&phi,&rho,&rms)) continue;
            unsigned n=hi-lo+1;
            /* Compare nearest planes with a 5 mm noise band, then support. */
            if (!best_n || rho<best_rho-5 || (fabsf(rho-best_rho)<=5 && n>best_n)) {
                best_n=n;best_rho=rho;blo=lo;bhi=hi;bphi=phi;brms=rms;
            }
        }
    }
    if (!best_n) return false;
    a->normal_deg=bphi/RAD;a->rho_mm=best_rho;a->rms_mm=brms;
    a->fit_first=(uint8_t)blo;a->fit_last=(uint8_t)bhi;
    /* A scan limit or fitted subwindow boundary is NOT a physical box edge. */
    a->edge_left_seen=blo>0 && (!a->range[blo-1] ||
        fabsf((float)a->range[blo]-a->range[blo-1])>fmaxf(30,0.25f*fminf(a->range[blo],a->range[blo-1])));
    a->edge_right_seen=bhi+1<ALIGN_POINTS && (!a->range[bhi+1] ||
        fabsf((float)a->range[bhi]-a->range[bhi+1])>fmaxf(30,0.25f*fminf(a->range[bhi],a->range[bhi+1])));
    a->side_left=a->normal_deg>=0;
    if (a->edge_left_seen && a->edge_right_seen &&
        abs((int)a->range[blo]-(int)a->range[bhi])>5)
        a->side_left=a->range[blo]<a->range[bhi];
    a->target_deg=a->normal_deg+(a->side_left?-90.0f:90.0f);
    return true;
}
void align_step(InitialAlign *a,uint32_t now,uint32_t seq,uint32_t trigger_ms,
                uint32_t echo_ms,bool valid,uint16_t mm,float heading,const float measured[2]) {
    a->speed[0]=a->speed[1]=0;
    bool fresh=seq!=a->last_seq;
    if (fresh) a->last_seq=seq;
    uint32_t elapsed=now-a->since;
    if (a->phase==ALIGN_SCAN) {
        /* First traverse may span 180 degrees: give it 1.2 seconds to settle. */
        uint32_t settle=a->index==0?1200U:120U;
        if (fresh && valid && mm>=40 && mm<=400 && elapsed>=settle &&
            (uint32_t)(trigger_ms-a->since)>=settle &&
            (uint32_t)(trigger_ms-a->since)<=elapsed && now-echo_ms<=100U) {
            a->sum+=mm;if(a->hits<250) ++a->hits;
        }
        if (elapsed>=settle+180U) {
            a->range[a->index]=a->hits>=2?(uint16_t)(a->sum/a->hits):0;
            a->sum=0;a->hits=0;
            if (++a->index==ALIGN_POINTS) a->phase=ALIGN_FIT;
            else {a->servo_us=pulse(a->index);a->since=now;}
        }
    } else if (a->phase==ALIGN_FIT) {
        if (!align_fit(a)) {fail(a,ALIGN_NO_FACE);return;}
        a->start_rad=heading;a->turn_since=a->since=now;a->phase=ALIGN_TURN;
        for(unsigned i=0;i<2;++i) a->bad_since[i]=a->stall_since[i]=now;
    } else if (a->phase==ALIGN_TURN) {
        a->actual_deg=(heading-a->start_rad)/RAD;
        float sign=a->target_deg>=0?1.0f:-1.0f;
        float remaining=fabsf(a->target_deg)-sign*a->actual_deg;
        if (!isfinite(heading)) {fail(a,ALIGN_ENCODER);return;}
        if (remaining<=2.0f) {a->phase=ALIGN_BRAKE;a->since=now;return;}
        if (now-a->turn_since>=15000U) {fail(a,ALIGN_TURN_TIMEOUT);return;}
        float speed=fminf(70.0f,fmaxf(25.0f,remaining*2.0f));
        a->speed[0]=-sign*speed;a->speed[1]=sign*speed;
        for (unsigned i=0;i<2;++i) {
            bool wrong=a->speed[i]*measured[i]<-100;
            bool stalled=now-a->turn_since>500 && fabsf(measured[i])<5;
            if (!wrong) a->bad_since[i]=now;
            if (!stalled) a->stall_since[i]=now;
            if (wrong && now-a->bad_since[i]>=150) {fail(a,ALIGN_ENCODER);return;}
            if (stalled && now-a->stall_since[i]>=800) {fail(a,ALIGN_STALL);return;}
        }
    } else if (a->phase==ALIGN_BRAKE) {
        a->actual_deg=(heading-a->start_rad)/RAD;
        if (elapsed>=300U) {
            a->phase=ALIGN_LOCK;a->since=now;
            a->servo_us=a->side_left?ALIGN_LEFT_US:ALIGN_RIGHT_US;
        }
    } else if (a->phase==ALIGN_LOCK) {
        if (fresh && valid && mm>=40 && mm<=400 && elapsed>=1200U &&
            (uint32_t)(trigger_ms-a->since)>=1200U &&
            (uint32_t)(trigger_ms-a->since)<=elapsed && now-echo_ms<=100U) {
            a->side_mm=mm;a->phase=ALIGN_DONE;
        } else if (elapsed>=2500U) fail(a,ALIGN_SIDE_NO_ECHO);
    }
}
const char *align_phase_name(AlignPhase p) {
    static const char *const n[]={"IDLE","SCAN","FIT","TURN","BRAKE","LOCK","DONE","FAILED","CANCELLED"};
    return (unsigned)p<sizeof(n)/sizeof(n[0])?n[p]:"UNKNOWN";
}
const char *align_error_name(AlignError e) {
    static const char *const n[]={"OK","NO_FACE","TURN_TIMEOUT","ENCODER","STALL","SIDE_NO_ECHO","HARDWARE"};
    return (unsigned)e<sizeof(n)/sizeof(n[0])?n[e]:"UNKNOWN";
}
