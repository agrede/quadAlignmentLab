/* Quad-alignment Arduino
*/
#include <IIRFilter.h>
#include <ADC.h>
#define HWSERIAL Serial1

// Read order
const int syncReads[6][2][3] = {A0, 0, 0, A2, 0, 2, A2, 0, 1, A3, 0, 3, A8, 2, 0, A12, 3, 0, A9, 2, 1, A13, 3, 1, A14, 3, 2, A10, 2, 2, A11, 2, 3, A16, 3, 3};
const int singleReads[4][3] = {A4, 1, 0, A5, 1, 1, A6, 1, 2, A7, 1, 3};
const float coeff = 1.323714190441312; // partial derivative
const float orderCoeff[3] = {0.25069945567551805, 0.0007026174540612872, 7.931892352488745e-05};
float coeffScale[4] = {1., 1., 1., 1.}; // correction factor
float errOffs[4][2] = {0, 0, 0, 0, 0, 0, 0, 0}; // X and Y offsets
const float p0s[4][2] = {-14.786, -26.88, -20.864, 26.88, 16.786, 26.88, 22.846, -26.88}; // x, y centers of quadrant PDs
// ADC filter
const float b0[] = {0.0004166, 0.0016664, 0.0024996, 0.0016664, 0.0004166};
const float a0[] = {1.0, -3.18063855,  3.86119435, -2.11215536,  0.43826514};
IIRFilter *iir0[4][4];
// Position filter
const float b1[] = {0.00094469, 0.00188938, 0.00094469};
const float a1[] = {1.0, -1.91119707,  0.91497583};
IIRFilter *iir1[4][2];
// Current values of filters
float ciir0[4][4];
float ciir1[4][2];
// Offsets x[mm] y[mm] theta[rad]
float offsets[3] = {0.0, 0.0, 0.0};
// ADC reader
ADC *adc = new ADC();
ADC::Sync_result syncRes;
unsigned long period[30];
int period_index = 29;
int period_count = 0;
float zero[4][4];
String serialComm;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    HWSERIAL.begin(9600);
    pinMode(14, INPUT);
    pinMode(25, INPUT);
    pinMode(28, INPUT);
    pinMode(29, INPUT);
    pinMode(30, INPUT);
    pinMode(32, INPUT);
    // ADC0
    adc->setAveraging(1);
    adc->setResolution(16);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
    // ADC1
    adc->setAveraging(1, ADC_1);
    adc->setResolution(16, ADC_1);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS, ADC_1);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_1);
    // Sync Reads
    for(int i=0;i<6;i++) {
        for(int j=0;j<2;j++){
            pinMode(syncReads[i][j][0], INPUT);
        }
    }
    for(int i=0;i<4;i++){
        pinMode(singleReads[i][0], INPUT);
    }
    // Setup filters
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            iir0[i][j] = new IIRFilter(b0, a0);
            zero[i][j] = 0.0;
        }
        for(int j=0;j<2;j++){
            iir1[i][j] = new IIRFilter(b1, a1);
        }
    }
}

void loop() {
    if (period_count % 1000 == 0) {
        period_index = (period_index+1) % 30;
        period[period_index] = micros();
        period_count = 0;
    }
    period_count++;
    updatePos();
    if (Serial.available() > 0) {
        serialComm = Serial.readStringUntil('\n');
        if (serialComm == "values") {
            for(int i=0;i<4;i++){
                for(int j=0;j<4;j++){
                    Serial.print(ciir0[i][j]);
                    Serial.print(", ");
                }
            }
            Serial.println("");
        } else if (serialComm == "deltas") {
            for(int i=0;i<4;i++){
                Serial.print(ciir1[i][0]);
                Serial.print(", ");
                Serial.print(ciir1[i][1]);
                Serial.print("; ");
            }
            Serial.println("");
        } else if (serialComm == "offsets") {
            updateOffsets();
            for(int i=0;i<3;i++){
                Serial.print(offsets[i]);
                Serial.print(", ");
            }
            Serial.println("");
        } else if (serialComm == "freq") {
            float frq = avgFreq();
            Serial.println(frq);
        } else if (serialComm == "period") {
            long rtn[30];
            for (int i=0;i<30;i++) {
                int j = (i + period_index) % 30;
                int k = (i + period_index - 1) % 30;
                Serial.print(period[j]);
                Serial.print(", ");
                Serial.print(j);
                Serial.print(", ");
                Serial.print(k);
                Serial.print("; ");
                if (period[j] > 0 && period[k] > 0) {
                    long prd = period[j]-period[k];
                    if (prd > 0) {
                        rtn[i] = prd;
                    } else {
                        rtn[i] = -1;
                    }
                } else {
                    rtn[i] = 0;
                }
            }
            Serial.println("");
            for (int i=0;i<30;i++) {
                Serial.print(rtn[i]);
                Serial.print(", ");
            }
            Serial.println("");
        } else if (serialComm == "volts") {
            int rtn[4][4];
            for(int i=0;i<6;i++) {
                for (int j=0;j<2;j++) {
                    rtn[syncReads[i][j][1]][syncReads[i][j][2]] = adc->analogRead(syncReads[i][j][0]);
                }
            }
            // Single Reads
            for(int i=0;i<4;i++) {
                rtn[singleReads[i][1]][singleReads[i][2]] = adc->analogRead(singleReads[i][0]);
            }
            for(int i=0;i<4;i++) {
                for(int j=0;j<4;j++) {
                    Serial.print(rtn[i][j]);
                    Serial.print(", ");
                }
                Serial.print("; ");
            }
            Serial.println("");
        } else if (serialComm == "zero") {
            for (int i=0;i<4;i++) {
                for (int j=0;j<4;j++) {
                    zero[i][j] += ciir0[i][j];
                }
            }
        }
    }
    if (HWSERIAL.available() > 0) {
        HWSERIAL.readStringUntil('\n'); // discard to eol
        updateOffsets();
        HWSERIAL.print(offsets[0]); // x in mm
        HWSERIAL.print(" ");
        HWSERIAL.print(offsets[1]); // y in mm
        HWSERIAL.print(" ");
        HWSERIAL.println(offsets[2]); // theta in rad
    }
}


void updatePos() {
    // Sync reads
    for(int i=0;i<6;i++) {
        syncRes = adc->analogSynchronizedRead(syncReads[i][0][0],
                                              syncReads[i][1][0]);
        syncRes.result_adc0 = (uint16_t)syncRes.result_adc0;
        syncRes.result_adc1 = (uint16_t)syncRes.result_adc1;
        updateValue(syncReads[i][0][1], syncReads[i][0][2],
                    (float)syncRes.result_adc0);
        updateValue(syncReads[i][1][1],
                    syncReads[i][1][2],
                    (float)syncRes.result_adc1);
    }
    // Single Reads
    for(int i=0;i<4;i++){
        float value = (float)adc->analogRead(singleReads[i][0]);
        updateValue(singleReads[i][1], singleReads[i][2], value);
    }
    updateDeltas();
}

void updateValue(int i, int j, float value) {
    ciir0[i][j] = iir0[i][j]->filter(value)-zero[i][j];
}

void updateDeltas() {
    for(int i=0;i<4;i++) {
        float sum = 0;
        for(int j=0;j<4;j++) {
            sum += ciir0[i][j];
        }
        if (sum < 1000) {
            sum = 65.536e3;
        }
        ciir1[i][0] = iir1[i][0]->filter((ciir0[i][2]+ciir0[i][3]
                                          -ciir0[i][0]-ciir0[i][1])
                                         /sum);
        ciir1[i][1] = iir1[i][1]->filter((ciir0[i][2]+ciir0[i][3]
                                          -ciir0[i][0]-ciir0[i][1])
                                         /sum);
    }
}

void updateOffsets() {
    float a[2] = {0.0, 0.0};
    float b[2] = {0.0, 0.0};
    float c[2] = {0.0, 0.0};
    for(int i=0;i<4;i++){
        float dx = coeff*coeffScale[i]*(ciir1[i][0]-errOffs[i][0]);
        float dy = coeff*coeffScale[i]*(ciir1[i][1]-errOffs[i][1]);
        a[0] += dx;
        a[1] += dy;
        b[0] += dx*p0s[i][0];
        b[1] += dy*p0s[i][0];
        c[0] += dx*p0s[i][1];
        c[1] += dy*p0s[i][1];
    }
    offsets[0] = a[0]*orderCoeff[0]-b[0]*orderCoeff[1]-c[0]*orderCoeff[0];
    offsets[1] = a[1]*orderCoeff[0]-b[1]*orderCoeff[1]-c[1]*orderCoeff[0];
    offsets[2] = a[1]*orderCoeff[1]+c[0]*orderCoeff[1]+c[1]*orderCoeff[2];
}

float avgFreq() {
    long rtn = 0;
    for (int i=0;i<30;i++) {
        int j = (i + period_index) % 30;
        int k = (i + period_index - 1) % 30;
        if (period[j] > 0 && period[k] > 0) {
            long prd = period[j]-period[k];
            if (prd > 0) {
                rtn += prd;
            }
        }
    }
    return ((float)rtn/30.0);
}
