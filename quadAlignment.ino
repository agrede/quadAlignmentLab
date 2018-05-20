/* Quad-alignment Arduino
*/
#include <IIRFilter.h>
#include <ADC.h>
#include <CmdMessenger.h>

CmdMessenger usrMsg = CmdMessenger(Serial);
CmdMessenger errMsg = CmdMessenger(Serial1);
enum{
    kGetValues,
    kGetDeltas,
    kGetOffsets,
};
enum{
    kRequestError,
    kRecieveError,
};

// Read order
const int syncReads[6][2][3] = {
    A0, 0, 2, A2, 0, 0,
    A1, 0, 1, A3, 0, 3,
    A8, 2, 2, A13, 3, 2,
    A9, 2, 1, A12, 3, 1,
    A14, 3, 0, A10, 2, 0,
    A11, 2, 3, A19, 3, 3};
const int singleReads[4][3] = {
    A4, 1, 2,
    A5, 1, 1,
    A6, 1, 0,
    A7, 1, 3};
// Coefficients
const float lstSqCoeff[3][3] = {
    0.35967258840323113, 0.012546421946241499, 0.002939649003336809,
    0.0125464219462415, 0.339309687001476, 0.0014002703065001674,
    0.002939649003336809, 0.0014002703065001674, 0.00032808582626526887}; // y order: tx, ty, theta x order sum dx, sum dy, inner product
float errScale[4][2] = {
    1.57855048, 3.83336533,
    0.62300071, 0.87135483,
    1.0, 1.0,
    1.02438082, 0.71731386}; // Error cell slopes [1/mm]
float errOffs[4][2] = {
    0.43339709, -0.09767916,
    -0.31518955, 0.04775822,
    0.0, 0.0,
    -0.39531853, -0.16846749}; // Error celll X and Y offsets [mm]
const float p0s[4][2] = {
    -14.786, 26.88, -20.864, -26.88,
    16.786, -26.88, 22.846, 26.88}; // x, y centers of quadrant PDs [mm]
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


void getValues() {
    usrMsg.sendCmdStart(kGetValues);
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++){
            usrMsg.sendCmdBinArg<float>(ciir0[i][j]);
        }
    }
}
void getDeltas() {
    usrMsg.sendCmdStart(kGetDeltas);
    for(int i=0; i<4; i++) {
        for(int j=0; j<2; j++){
            usrMsg.sendCmdBinArg<float>(ciir1[i][j]);
        }
    }
}
void getOffsets() {
    usrMsg.sendCmdStart(kGetValues);
    updateOffsets();
    for(int i=0; i<3; i++) {
        usrMsg.sendCmdBinArg<float>(offsets[i]);
    }
    usrMsg.sendCmdEnd();
}
void sendOffsets() {
    errMsg.sendCmdStart(kRecieveError);
    updateOffsets();
    for(int i=0; i<3; i++) {
        errMsg.sendCmdBinArg<float>(offsets[i]);
    }
    errMsg.sendCmdEnd();
}


void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial1.begin(115200);
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
        }
        for(int j=0;j<2;j++){
            iir1[i][j] = new IIRFilter(b1, a1);
        }
    }
    usrMsg.attach(kGetValues, getValues);
    usrMsg.attach(kGetDeltas, getDeltas);
    usrMsg.attach(kGetOffsets, getOffsets);
    errMsg.attach(kRequestError, sendOffsets);
}

void loop() {
    updatePos();
    usrMsg.feedinSerialData();
    errMsg.feedinSerialData();
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
    ciir0[i][j] = iir0[i][j]->filter(value);
}

void updateDeltas() {
    for(int i=0;i<4;i++) {
        float sum = 0;
        for(int j=0;j<4;j++) {
            sum += ciir0[i][j];
        }
        if (sum < 200) {
            sum = 65.536e3;
        }
        ciir1[i][0] = iir1[i][0]->filter((ciir0[i][2]+ciir0[i][3]
                                          -ciir0[i][0]-ciir0[i][1])
                                         /sum);
        ciir1[i][1] = iir1[i][1]->filter((ciir0[i][3]+ciir0[i][0]
                                          -ciir0[i][2]-ciir0[i][1])
                                         /sum);
    }
}

void updateOffsets() {
    float X[3] = {0.0, 0.0, 0.0};
    for(int i=0;i<3;i++){
        if (i==2) {
            i++;
        }
        float dx = errScale[i][0]*ciir1[i][0]+errOffs[i][0];
        float dy = errScale[i][1]*ciir1[i][1]+errOffs[i][1];
        X[0] += dx;
        X[1] += dy;
        X[2] += -dx*p0s[i][1]+dy*p0s[i][0];
    }
    for (int i=0;i<3;i++){
        offsets[i] = 0.0;
        for (int j=0;j<3;j++){
            offsets[i] += lstSqCoeff[i][j]*X[j];
        }
    }
}
