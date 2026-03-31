#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"

// ── Pin ─────────────────────────────────────────────────
#define I2C_SDA  8
#define I2C_SCL  9

// ── OLED ────────────────────────────────────────────────
#define OLED_ADDR  0x3C
#define OLED_W     128
#define OLED_H      64
#define OLED_RST    -1
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RST);

// ── Sensor config ────────────────────────────────────────
#define CFG_LED_BRIGHTNESS  0x7F
#define CFG_SAMPLE_AVG         4
#define CFG_SAMPLE_RATE      100   // Hz
#define CFG_PULSE_WIDTH      411
#define CFG_ADC_RANGE      16384
#define CFG_LED_MODE_RED_IR    2

#define FINGER_THR         30000UL  // Giảm ngưỡng phát hiện tay

// ── DC block ────────────────────────────────────────────
#define DC_ALPHA_FAST  0.80f       // Tăng tốc hội tụ
#define DC_ALPHA_SLOW  0.98f      
#define DC_WARMUP_SAMPLES 100      

// ── Peak detection ───────────────────────────────────────
#define PEAK_THRESHOLD_FACTOR   0.350f  // Giảm ngưỡng
#define PEAK_MIN_DISTANCE_MS    100   // Giảm để phát hiện BPM cao hơn
#define PEAK_MAX_DISTANCE_MS    800   // Thêm ngưỡng max

// ── HR ───────────────────────────────────────────────────
#define BPM_HIST_SIZE   5       // Giảm để phản hồi nhanh hơn
#define BPM_MIN        40
#define BPM_MAX        200

// ── SpO2 ─────────────────────────────────────────────────
#define SPO2_WINDOW_SIZE  100
#define SPO2_MIN           85
#define SPO2_MAX          100

// ── OLED refresh ─────────────────────────────────────────
#define OLED_REFRESH_MS  200

// ════════════════════════════════════════════════════════
// Kalman 1D
// ════════════════════════════════════════════════════════
struct Kalman1D {
    float Q, R, P, x;
    Kalman1D(float q=0.02f, float r=0.3f): Q(q),R(r),P(1.0f),x(0.0f){}
    float process(float z){
        P += Q;
        float K = P/(P+R);
        x += K*(z-x);
        P *= (1.0f-K);
        return x;
    }
    void reset(){ P=1.0f; x=0.0f; }
};

// ════════════════════════════════════════════════════════
// Butterworth Bandpass
// ════════════════════════════════════════════════════════
struct ButterworthBP {
    const float S1b0= 0.06745527f, S1b1=0.0f, S1b2=-0.06745527f;
    const float S1a1=-1.82267479f, S1a2= 0.86506072f;
    float w1_0=0, w1_1=0;

    const float S2b0= 1.0f,        S2b1=0.0f, S2b2=-1.0f;
    const float S2a1=-1.96521036f, S2a2= 0.96601021f;
    float w2_0=0, w2_1=0;

    float process(float x){
        float y1 = S1b0*x + w1_0;
        w1_0 = S1b1*x - S1a1*y1 + w1_1;
        w1_1 = S1b2*x - S1a2*y1;

        float y2 = S2b0*y1 + w2_0;
        w2_0 = S2b1*y1 - S2a1*y2 + w2_1;
        w2_1 = S2b2*y1 - S2a2*y2;

        return y2;
    }

    void reset(){ w1_0=w1_1=w2_0=w2_1=0; }
};

// ════════════════════════════════════════════════════════
// Objects
// ════════════════════════════════════════════════════════
MAX30105 sensor;

Kalman1D    kalIR (0.02f, 0.3f),  kalRed(0.02f, 0.3f);
ButterworthBP bpIR,               bpRed;

// ── State variables ─────────────────────────────────────
float dcIR=0, dcRed=0;
bool  dcInit=false;

float    peakBuf[3]   = {};
uint32_t peakTimes[3] = {};
uint32_t lastPeakMs   = 0;
float    ampMax       = 0;
const float ampDecay  = 0.90f;     // Decay nhanh

int warmupCount=0;

uint32_t rriHistory[BPM_HIST_SIZE] = {};
int rriHead=0, rriCount=0;

// BPM filter
int32_t bpmHistory[3] = {0};
int bpmHistoryIdx = 0;
int bpmHistoryCount = 0;

float sumAC2_IR=0, sumAC2_Red=0, sumDC_IR=0, sumDC_Red=0;
int   spo2Cnt=0;

int32_t  dispBPM=0, dispSpO2=0;
bool     dispFinger=false;
float    dispIR_AC=0;
uint32_t dispIR_Raw=0;
bool     peakNow=false;
uint32_t lastOledMs=0;

#define WAVE_W 64
float waveBuf[WAVE_W]={};
int   waveIdx=0;

// ════════════════════════════════════════════════════════
// Hàm xử lý BPM với trung vị
// ════════════════════════════════════════════════════════
static int32_t calcBPM(){
    if(rriCount<2) return 0;
    uint32_t sum=0;
    int n=min(rriCount, BPM_HIST_SIZE);
    for(int i=0;i<n;i++){
        int idx=(rriHead-1-i+BPM_HIST_SIZE)%BPM_HIST_SIZE;
        sum+=rriHistory[idx];
    }
    uint32_t avg=sum/n;
    if(avg==0) return 0;
    int32_t bpm=60000/avg;
    if(bpm<BPM_MIN||bpm>BPM_MAX) return 0;
    return bpm;
}

static int32_t getMedianBPM(int32_t newBPM) {
    if(newBPM == 0) return dispBPM;
    
    bpmHistory[bpmHistoryIdx] = newBPM;
    bpmHistoryIdx = (bpmHistoryIdx + 1) % 3;
    if(bpmHistoryCount < 3) bpmHistoryCount++;
    
    // Sắp xếp 3 số
    int32_t a = bpmHistory[0];
    int32_t b = bpmHistory[1];
    int32_t c = bpmHistory[2];
    
    if(bpmHistoryCount == 3) {
        // Tìm trung vị
        if((a >= b && a <= c) || (a <= b && a >= c)) return a;
        if((b >= a && b <= c) || (b <= a && b >= c)) return b;
        return c;
    } else if(bpmHistoryCount == 2) {
        return (a + b) / 2;
    }
    return newBPM;
}

static int32_t calcSpO2(){
    if(spo2Cnt<20) return 0;

    float rmsIR  = sqrtf(sumAC2_IR /spo2Cnt);
    float rmsRed = sqrtf(sumAC2_Red/spo2Cnt);
    float avgIR  = sumDC_IR /spo2Cnt;
    float avgRed = sumDC_Red/spo2Cnt;

    if(avgIR<1000.0f||avgRed<1000.0f) return 0;
    if(rmsIR<1.0f) return 0;

    float R    = (rmsRed/avgRed)/(rmsIR/avgIR);
    float spo2 = 110.0f - 25.0f*R;

    int32_t s=(int32_t)roundf(spo2);
    if(s<SPO2_MIN||s>SPO2_MAX) return 0;
    return s;
}

// ════════════════════════════════════════════════════════
// Process Sample - Cải thiện peak detection
// ════════════════════════════════════════════════════════
static void processSample(uint32_t rawIR, uint32_t rawRed, uint32_t nowMs){

    if(!dcInit){
        dcIR=(float)rawIR; dcRed=(float)rawRed;
        kalIR.reset(); kalRed.reset();
        bpIR.reset();  bpRed.reset();
        ampMax=0; rriCount=0; rriHead=0;
        lastPeakMs=0; warmupCount=0;
        memset(peakBuf,0,sizeof(peakBuf));
        dcInit=true;
        return;
    }

    float alpha = (warmupCount < DC_WARMUP_SAMPLES) ? DC_ALPHA_FAST : DC_ALPHA_SLOW;
    dcIR  = alpha*dcIR  + (1.0f-alpha)*(float)rawIR;
    dcRed = alpha*dcRed + (1.0f-alpha)*(float)rawRed;

    float acIR  = (float)rawIR  - dcIR;
    float acRed = (float)rawRed - dcRed;

    float kIR  = kalIR.process(acIR);
    float kRed = kalRed.process(acRed);

    float fIR  = bpIR.process(kIR);
    float fRed = bpRed.process(kRed);

    dispIR_AC = fIR;
    
    bool isWarmup = (warmupCount < DC_WARMUP_SAMPLES);

    if(isWarmup){
        warmupCount++;
        ampMax = 0;
    } else {
        // Cập nhật biên độ
        float absAC = fabsf(fIR);
        if(absAC > ampMax) {
            ampMax = absAC;
        } else {
            ampMax *= ampDecay;
        }
        if(ampMax > 15000) ampMax = 15000;
        if(ampMax < 10) ampMax = 10;

        // Waveform buffer
        waveBuf[waveIdx] = fIR;
        waveIdx = (waveIdx + 1) % WAVE_W;

        // Peak detection - đơn giản hóa
        peakBuf[0] = peakBuf[1]; 
        peakBuf[1] = peakBuf[2]; 
        peakBuf[2] = fIR;
        peakTimes[0] = peakTimes[1]; 
        peakTimes[1] = peakTimes[2]; 
        peakTimes[2] = nowMs;

        peakNow = false;

        // Phát hiện đỉnh với ngưỡng thích ứng
        float dynamicThr = max(PEAK_THRESHOLD_FACTOR * ampMax, 50.0f);
        
        bool isPeak = (peakBuf[1] > peakBuf[0]) &&
                      (peakBuf[1] > peakBuf[2]) &&
                      (peakBuf[1] > dynamicThr);
        
        if(isPeak && (peakTimes[1] - lastPeakMs) >= PEAK_MIN_DISTANCE_MS) {
            uint32_t rri = peakTimes[1] - lastPeakMs;
            
            // Chỉ chấp nhận RRI trong khoảng hợp lý
            if(lastPeakMs != 0 && rri >= 350 && rri <= 800) {
                rriHistory[rriHead] = rri;
                rriHead = (rriHead + 1) % BPM_HIST_SIZE;
                if(rriCount < BPM_HIST_SIZE) rriCount++;
                
                int32_t rawBPM = 60000 / rri;
                int32_t bpm = calcBPM();
                
                if(dispBPM == 0) {
                    dispBPM = rawBPM;
                } else {
                    dispBPM = (dispBPM * 2 + rawBPM) / 3;  // Làm mượt nhẹ
                }
                
                // Debug chi tiết
                static uint32_t lastDebug = 0;
                if(millis() - lastDebug > 1000) {
                    lastDebug = millis();
                    Serial.printf("RRI=%dms → BPM=%d (raw=%d, amp=%.0f, thr=%.0f)\n", 
                                 rri, dispBPM, rawBPM, ampMax, dynamicThr);
                }
                
                peakNow = true;
            }
            lastPeakMs = peakTimes[1];
        }
    }

    // SpO2 accumulation
    sumAC2_IR  += fIR * fIR;
    sumAC2_Red += fRed * fRed;
    sumDC_IR   += dcIR;
    sumDC_Red  += dcRed;
    spo2Cnt++;

    if(spo2Cnt >= SPO2_WINDOW_SIZE){
        int32_t s = calcSpO2();
        if(s > 0) dispSpO2 = s;
        sumAC2_IR = sumAC2_Red = sumDC_IR = sumDC_Red = 0;
        spo2Cnt = 0;
    }
}

// ════════════════════════════════════════════════════════
// renderOLED
// ════════════════════════════════════════════════════════
static void renderOLED(){
    oled.clearDisplay();

    if(!dispFinger){
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(8,20); oled.print("Place wrist on");
        oled.setCursor(8,32); oled.print("sensor...");
        oled.display();
        return;
    }

    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(0,0);
    if(dispBPM>0) oled.printf("%3ld",dispBPM);
    else          oled.print(" --");
    oled.setTextSize(1);
    oled.setCursor(52,4); oled.print("bpm");

    oled.setTextSize(2);
    oled.setCursor(0,18);
    if(dispSpO2>0) oled.printf("%3ld",dispSpO2);
    else           oled.print(" --");
    oled.setTextSize(1);
    oled.setCursor(52,22); oled.print("SpO2%");

    if(peakNow) oled.fillCircle(122,4,3,SSD1306_WHITE);

    // Waveform
    const int GY=40, GH=22;
    float wMin=waveBuf[0], wMax=waveBuf[0];
    for(int i=1;i<WAVE_W;i++){
        if(waveBuf[i]<wMin) wMin=waveBuf[i];
        if(waveBuf[i]>wMax) wMax=waveBuf[i];
    }
    float wRange=wMax-wMin;
    if(wRange<0.001f) wRange=1.0f;

    for(int i=0;i<WAVE_W-1;i++){
        int xi=(waveIdx+i)%WAVE_W;
        int xj=(waveIdx+i+1)%WAVE_W;
        int x0=i*2, x1=(i+1)*2;
        int y0=GY+GH-(int)((waveBuf[xi]-wMin)/wRange*(GH-1));
        int y1=GY+GH-(int)((waveBuf[xj]-wMin)/wRange*(GH-1));
        y0=constrain(y0,GY,GY+GH-1);
        y1=constrain(y1,GY,GY+GH-1);
        oled.drawLine(x0,y0,x1,y1,SSD1306_WHITE);
    }
    oled.drawFastHLine(0,GY-2,OLED_W,SSD1306_WHITE);
    oled.display();
}

// ════════════════════════════════════════════════════════
// Setup & Loop
// ════════════════════════════════════════════════════════
void setup(){
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if(!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
        Serial.println("OLED FAIL");
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(10,24); oled.print("Initializing...");
    oled.display();

    if(!sensor.begin(Wire, I2C_SPEED_FAST)){
        Serial.println("MAX30102 FAIL");
        while(1) delay(100);
    }
    sensor.setup(CFG_LED_BRIGHTNESS, CFG_SAMPLE_AVG,
                 CFG_LED_MODE_RED_IR, CFG_SAMPLE_RATE,
                 CFG_PULSE_WIDTH, CFG_ADC_RANGE);
}

void loop(){
    sensor.check();

    while(sensor.available()){
        uint32_t ir  = sensor.getFIFOIR();
        uint32_t red = sensor.getFIFORed();
        sensor.nextSample();

        uint32_t now   = millis();
        bool     finger= (ir > FINGER_THR);

        dispIR_Raw = ir;
        dispFinger = finger;

        if(finger){
            processSample(ir, red, now);
        } else {
            dcInit=false; ampMax=0;
            rriCount=0; rriHead=0; warmupCount=0;
            spo2Cnt=0;
            bpmHistoryCount = 0;
            sumAC2_IR=sumAC2_Red=sumDC_IR=sumDC_Red=0;
            memset(waveBuf,0,sizeof(waveBuf));
            peakNow=false;
            dispBPM = 0;
        }
    }

    static uint32_t lastLog=0;
    if(millis()-lastLog>1000){
        lastLog=millis();
        Serial.printf("IR:%lu  AC:%.1f  ampMax:%.1f  warm:%d  BPM:%ld  SpO2:%ld%%\n",
                       dispIR_Raw, dispIR_AC, ampMax, warmupCount, dispBPM, dispSpO2);
    }

    if(millis()-lastOledMs>OLED_REFRESH_MS){
        lastOledMs=millis();
        renderOLED();
        peakNow=false;
    }
}