

int pds[4][4] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16};
String serialComm;
float pos[4][2];
bool constOut = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(14, INPUT);
  pinMode(25, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(32, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    serialComm = Serial.readStringUntil('\n');
    if (serialComm == "volts") {
      for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
          Serial.print(analogRead(pds[i][j]));
          Serial.print(",");
        }
      }
      Serial.println("");
    } else if (serialComm == "pos"){
      getPos();
      for(int i=0;i<4;i++){
        for(int j=0;j<2;j++){
          Serial.println(pos[i][j]);
        }
      }
    } else if (serialComm == "const") {
      constOut = !(constOut);      
    }
  }
  if (constOut) {
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        Serial.print(analogRead(pds[i][j]));
        Serial.print(" ");
      }
    }
    Serial.println("");
  }
  delay(100);
}


void getPos() {
  for(int i=0;i<4;i++){
    long dta[4];
    long sum = 0;
    for(int j=0;j<4;j++) {
      dta[j] = analogRead(pds[i][j])-512;
      sum += dta[j];
    }
    pos[i][0] = (-(float)dta[0]-(float)dta[1]+(float)dta[2]+(float)dta[3])/((float)sum);
    pos[i][1] = ((float)dta[0]-(float)dta[1]-(float)dta[2]+(float)dta[3])/((float)sum);
  }
}

