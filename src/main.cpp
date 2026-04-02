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
#define CFG_LED_BRIGHTNESS  0x7F   // Chỉ dùng khi khởi tạo, AGC sẽ override
#define CFG_SAMPLE_AVG         4
#define CFG_SAMPLE_RATE      100   // Hz
#define CFG_PULSE_WIDTH      411
#define CFG_ADC_RANGE      16384
#define CFG_LED_MODE_RED_IR    2

#define FINGER_THR         30000UL

// ── DC block ────────────────────────────────────────────
#define DC_ALPHA_FAST  0.80f
#define DC_ALPHA_SLOW  0.98f
#define DC_WARMUP_SAMPLES 100

// ── Peak detection ───────────────────────────────────────
#define PEAK_THRESHOLD_FACTOR   0.350f
#define PEAK_MIN_DISTANCE_MS    100
#define PEAK_MAX_DISTANCE_MS    1500

// ── HR ───────────────────────────────────────────────────
#define BPM_HIST_SIZE   5
#define BPM_MIN        40
#define BPM_MAX        200

// ── SpO2 ─────────────────────────────────────────────────
#define SPO2_WINDOW_SIZE  100
#define SPO2_MIN           85
#define SPO2_MAX          100

// ── OLED refresh ─────────────────────────────────────────
#define OLED_REFRESH_MS  200

// ═══════════════════════════════════════════════════════════
//  AGC — Automatic Gain Control
//
//  Mục tiêu: giữ DC của IR signal trong vùng tối ưu
//  [AGC_DC_TARGET_LOW, AGC_DC_TARGET_HIGH] bằng cách
//  tự động tăng/giảm LED brightness (0x00–0xFF).
//
//  MAX30102 ADC range = 16384 (14-bit), tín hiệu tốt
//  khi DC ≈ 50000–180000 (sensor trả về 18-bit raw).
//
//  Thuật toán:
//    1. Đo DC level trung bình mỗi AGC_INTERVAL_MS
//    2. Nếu DC < TARGET_LOW  → tăng LED (tín hiệu yếu)
//    3. Nếu DC > TARGET_HIGH → giảm LED (tín hiệu bão hòa)
//    4. Clamp trong [AGC_LED_MIN, AGC_LED_MAX]
//    5. Chỉ điều chỉnh sau warmup để tránh nhiễu lúc đặt tay
// ═══════════════════════════════════════════════════════════
#define AGC_DC_TARGET_LOW    15000.0f   // DC quá thấp → LED quá yếu
#define AGC_DC_TARGET_HIGH  50000.0f   // DC quá cao  → LED quá mạnh / bão hòa
#define AGC_DC_OPTIMAL      100000.0f   // Điểm tối ưu muốn hướng đến
#define AGC_LED_MIN         0x3F        // Brightness tối thiểu (tránh tắt hẳn)
#define AGC_LED_MAX         0xFF        // Brightness tối đa
#define AGC_STEP_UP         0x08        // Bước tăng mỗi lần điều chỉnh
#define AGC_STEP_DOWN       0x04        // Bước giảm nhỏ hơn để tránh dao động
#define AGC_INTERVAL_MS     500         // Tần suất điều chỉnh (ms)
#define AGC_SETTLE_SAMPLES  20          // Bỏ qua N mẫu sau khi thay đổi LED

struct AGC {
    uint8_t  ledLevel      = CFG_LED_BRIGHTNESS;  // Brightness hiện tại
    float    dcAccum       = 0.0f;                 // Tích lũy DC để tính trung bình
    int      dcCount       = 0;                    // Số mẫu đã tích lũy
    uint32_t lastAdjustMs  = 0;                    // Thời điểm điều chỉnh gần nhất
    int      settleCount   = 0;                    // Đếm mẫu bỏ qua sau adjust
    bool     active        = false;                // AGC chỉ active sau warmup
    int      adjustCount   = 0;                    // Tổng số lần đã điều chỉnh (debug)

    // Gọi mỗi sample — cung cấp DC level hiện tại
    // Trả về true nếu vừa điều chỉnh LED (caller nên bỏ qua mẫu này)
    bool update(float dc, uint32_t nowMs, MAX30105& sensor, bool fingerOn) {
        if (!fingerOn) {
            // Reset khi nhấc tay
            reset();
            return false;
        }

        // Chưa active → chờ warmup từ processSample
        if (!active) return false;

        // Đang trong settle period → bỏ qua mẫu
        if (settleCount > 0) {
            settleCount--;
            return true;  // báo caller bỏ qua mẫu này
        }

        // Tích lũy DC
        dcAccum += dc;
        dcCount++;

        // Chưa đến kỳ điều chỉnh
        if (nowMs - lastAdjustMs < AGC_INTERVAL_MS) return false;
        if (dcCount == 0) return false;

        float avgDC = dcAccum / dcCount;
        dcAccum  = 0;
        dcCount  = 0;
        lastAdjustMs = nowMs;

        uint8_t newLevel = ledLevel;

        if (avgDC < AGC_DC_TARGET_LOW) {
            // Tín hiệu yếu — tăng LED
            int step = AGC_STEP_UP;
            // Tăng mạnh hơn nếu DC rất thấp
            if (avgDC < AGC_DC_TARGET_LOW * 0.5f) step = AGC_STEP_UP * 2;
            newLevel = (uint8_t)min((int)ledLevel + step, (int)AGC_LED_MAX);

        } else if (avgDC > AGC_DC_TARGET_HIGH) {
            // Tín hiệu bão hòa — giảm LED
            int step = AGC_STEP_DOWN;
            if (avgDC > AGC_DC_TARGET_HIGH * 1.2f) step = AGC_STEP_DOWN * 2;
            newLevel = (uint8_t)max((int)ledLevel - step, (int)AGC_LED_MIN);
        }

        if (newLevel != ledLevel) {
            ledLevel     = newLevel;
            settleCount  = AGC_SETTLE_SAMPLES;
            adjustCount++;

            // Áp dụng ngay cho cả IR và Red LED
            sensor.setPulseAmplitudeIR(ledLevel);
            sensor.setPulseAmplitudeRed(ledLevel);

            Serial.printf("[AGC] #%d DC=%.0f → LED=0x%02X (%s)\n",
                adjustCount, avgDC, ledLevel,
                avgDC < AGC_DC_TARGET_LOW ? "INCREASE" : "DECREASE");
            return true;
        }

        // Debug log mỗi 5 giây dù không adjust
        static uint32_t lastLog = 0;
        if (nowMs - lastLog > 5000) {
            lastLog = nowMs;
            Serial.printf("[AGC] DC=%.0f  LED=0x%02X  status=STABLE\n", avgDC, ledLevel);
        }

        return false;
    }

    void reset() {
        ledLevel     = CFG_LED_BRIGHTNESS;
        dcAccum      = 0;
        dcCount      = 0;
        lastAdjustMs = 0;
        settleCount  = 0;
        active       = false;
        adjustCount  = 0;
    }

    void activate(MAX30105& sensor) {
        active      = true;
        settleCount = AGC_SETTLE_SAMPLES;  // bỏ qua mẫu đầu sau warmup
        Serial.printf("[AGC] Activated — initial LED=0x%02X\n", ledLevel);
    }
} agc;


// Kalman 1D
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

// Butterworth Bandpass
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

// Objects
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
const float ampDecay  = 0.97f;

int warmupCount=0;

uint32_t rriHistory[BPM_HIST_SIZE] = {};
int rriHead=0, rriCount=0;

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

// ── BPM helpers ──────────────────────────────────────────

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
    int32_t a = bpmHistory[0];
    int32_t b = bpmHistory[1];
    int32_t c = bpmHistory[2];
    if(bpmHistoryCount == 3) {
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


// ═══════════════════════════════════════════════════════════
//  processSample — tích hợp AGC
// ═══════════════════════════════════════════════════════════

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

    // ── Kích hoạt AGC sau warmup ──────────────────────────
    if (warmupCount == DC_WARMUP_SAMPLES && !agc.active) {
        agc.activate(sensor);
    }

    // ── AGC update — nếu đang settle thì bỏ qua mẫu này ─
    bool agcSettle = agc.update(dcIR, nowMs, sensor, true);
    if (agcSettle) {
        // LED vừa thay đổi hoặc đang settle → bỏ qua mẫu
        // để tránh nhiễu do DC chưa ổn định
        if (warmupCount < DC_WARMUP_SAMPLES) warmupCount++;
        return;
    }
    // ─────────────────────────────────────────────────────

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
        float absAC = fabsf(fIR);
        if(absAC > ampMax) {
            ampMax = absAC;
        } else {
            ampMax *= ampDecay;
        }
        if(ampMax > 15000) ampMax = 15000;
        if(ampMax < 10)    ampMax = 10;

        waveBuf[waveIdx] = fIR;
        waveIdx = (waveIdx + 1) % WAVE_W;

        peakBuf[0] = peakBuf[1];
        peakBuf[1] = peakBuf[2];
        peakBuf[2] = fIR;
        peakTimes[0] = peakTimes[1];
        peakTimes[1] = peakTimes[2];
        peakTimes[2] = nowMs;

        peakNow = false;

        // float dynamicThr = max(PEAK_THRESHOLD_FACTOR * ampMax, ampMax * 0.25f);
        float dynamicThr;
        if (ampMax > 500)
            dynamicThr = PEAK_THRESHOLD_FACTOR * ampMax;
        else
            dynamicThr = ampMax * 0.15f;

        bool isPeak = (peakBuf[1] > peakBuf[0]) &&
                      (peakBuf[1] > peakBuf[2]) &&
                      (peakBuf[1] > dynamicThr);

        if(isPeak && (peakTimes[1] - lastPeakMs) >= PEAK_MIN_DISTANCE_MS) {
            uint32_t rri = peakTimes[1] - lastPeakMs;
            if (rri > 1200 && ampMax < 80) {
                Serial.println("Rejected low-signal slow beat");
                return;
            }
            int ampThr = (ampMax > 500) ? 200 : 50;

            if(lastPeakMs != 0 && rri >= 300 && rri <= 1500 && ampMax > ampThr)
            {
                rriHistory[rriHead] = rri;
                rriHead = (rriHead + 1) % BPM_HIST_SIZE;
                if(rriCount < BPM_HIST_SIZE) rriCount++;

                int32_t rawBPM = 60000 / rri;
                if(dispBPM == 0) {
                    dispBPM = rawBPM;
                } else {
                    dispBPM = (dispBPM * 2 + rawBPM) / 3;
                }

                static uint32_t lastDebug = 0;
                if(millis() - lastDebug > 1000) {
                    lastDebug = millis();
                    Serial.printf("RRI=%dms → BPM=%d (raw=%d, amp=%.0f, thr=%.0f, LED=0x%02X)\n",
                                 rri, dispBPM, rawBPM, ampMax, dynamicThr, agc.ledLevel);
                }

                peakNow = true;
            }
            lastPeakMs = peakTimes[1];
        }
    }

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


// ── renderOLED ───────────────────────────────────────────

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

    // Hiển thị LED level AGC ở góc (debug — có thể xóa)
    oled.setTextSize(1);
    oled.setCursor(90, 22);
    oled.printf("G:%02X", agc.ledLevel);

    if(peakNow) oled.fillCircle(122,4,3,SSD1306_WHITE);

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


// ── setup / loop ─────────────────────────────────────────

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

    Serial.printf("[AGC] Init — LED=0x%02X  Target DC=[%.0f, %.0f]\n",
                  CFG_LED_BRIGHTNESS, AGC_DC_TARGET_LOW, AGC_DC_TARGET_HIGH);
}

void loop(){
    sensor.check();

    while(sensor.available()){
        uint32_t ir  = sensor.getFIFOIR();
        uint32_t red = sensor.getFIFORed();
        sensor.nextSample();

        uint32_t now    = millis();
        bool     finger = (ir > FINGER_THR);

        dispIR_Raw = ir;
        dispFinger = finger;

        if(finger){
            processSample(ir, red, now);
        } else {
            // Reset tất cả kể cả AGC khi nhấc tay
            agc.reset();
            // Reset LED về mức mặc định
            sensor.setPulseAmplitudeIR(CFG_LED_BRIGHTNESS);
            sensor.setPulseAmplitudeRed(CFG_LED_BRIGHTNESS);

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
        Serial.printf("IR:%lu  AC:%.1f  ampMax:%.1f  warm:%d  BPM:%ld  SpO2:%ld%%  LED:0x%02X\n",
                       dispIR_Raw, dispIR_AC, ampMax, warmupCount,
                       dispBPM, dispSpO2, agc.ledLevel);
    }

    if(millis()-lastOledMs>OLED_REFRESH_MS){
        lastOledMs=millis();
        renderOLED();
        peakNow=false;
    }
}