#include <Arduino.h>
#include <Wire.h>

#include "SD_MMC.h"

#include <PID_v1.h>

#include "INA226.h"
#include <AD5272.h>

#define TAG "[ESP32-S3]"
#define PIN_SDA GPIO_NUM_17
#define PIN_SCL GPIO_NUM_18

esp_err_t ret;

INA226 ina(0x40);
AD5272 digiPot(0x2F);

double supplyCurrent;
double constCurrent = 350;
double allowedCurrentErr = 5;
double ignoredCurrent = 50;
double voltageLimit = 4.1;
double drainVoltage = 3.05;
double powerVoltage;
double difference = 0;

unsigned long startMillis;
unsigned long loopMillis;
const unsigned int intervalMillis = 125;

int mode = 0; // 0 charge 1 drain

int resistanceStep;

char* fileName;
File logFile;
int logFileCnt = 0;

// const double Kp=0.016, Ki=0.002, Kd=0.004;
// PID currentPID(&supplyCurrent, &powerVoltage, &constCurrent, Kp, Ki, Kd, DIRECT);

int voltageToResistanceStep(double volts) {
    double resistance = double(6800)*(volts/double(0.8) - double(1)) - double(10000);
    return int(resistance * double(1023) / double(20000));
}

double resistanceStepToVoltage(int step) {
    return (((double(step) / double(1023)) * double(20000) + double(10000)) / double(6800) + double(1)) * double(0.8);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
	// Mount microSD card
    SD_MMC.setPins(11, 12, 10, 9, 14, 13);
    if(!SD_MMC.begin()) {
        while(true) {
            Serial.println("SD Error!");
            delay(1000);
        }
    }
    do {
        logFileCnt++;
        fileName = (char *)malloc(sizeof(char) * 24);
        sprintf(fileName, "/sdcard/logFile_%03d.csv\0", logFileCnt);
    } while(SD_MMC.exists(fileName));
    free(fileName);
    logFileCnt--;

    Wire.begin(PIN_SDA, PIN_SCL);
    ina.begin();
    digiPot.init();

    Serial.println(ina.setMaxCurrentShunt(0.45, 0.05));
    delay(1000);
    powerVoltage = ina.getBusVoltage();
    digiPot.write_data(AD5272_RDAC_WRITE, voltageToResistanceStep(max(powerVoltage, voltageLimit)));
    resistanceStep = voltageToResistanceStep(powerVoltage);
    digiPot.write_data(AD5272_50TP_WRITE, 0);
    delay(10000);
    // currentPID.SetOutputLimits(powerVoltage - 0.05, powerVoltage + 0.05);
    // currentPID.SetMode(AUTOMATIC);
    mode = 1;
    pinMode(GPIO_NUM_3, OUTPUT);
    pinMode(GPIO_NUM_8, OUTPUT);
    digitalWrite(GPIO_NUM_3, LOW);
    digitalWrite(GPIO_NUM_8, HIGH);
    startMillis = millis();
}

void loop() {
    loopMillis = millis();

    double current = -ina.getCurrent_mA();
    double voltage = ina.getBusVoltage();
    // put your main code here, to run repeatedly:
    Serial.print("microSD ");
    Serial.print((ret == ESP_OK) ? "Connected/" : "Disconnected/");
    Serial.print(digiPot.read_rdac() * 20000 / 1024);
    Serial.print(" Ohms/");
    Serial.print(resistanceStepToVoltage(resistanceStep), 4);
    Serial.print(" V expected/");
    Serial.print(voltage, 4);
    Serial.print(" V real/");
    if(current >= 0) {
        Serial.print(current);
        Serial.println(" mA supplied");
    }
    else {
        Serial.print(-current);
        Serial.println(" mA draining");
    }

    // currentPID.SetOutputLimits(powerVoltage - 0.05, powerVoltage + 0.05);
    // currentPID.Compute();
    switch(mode) {
        case 0:
            if(ina.getBusVoltage() < voltageLimit) {
                logFile.printf("%d,%.4f,%.4f\n",millis() - startMillis, voltage, current);
                if(current <= constCurrent - allowedCurrentErr) resistanceStep++;
                if(current >= constCurrent + allowedCurrentErr) resistanceStep--;
            }
            if(ina.getBusVoltage() >= voltageLimit) {
                logFile.printf("%d,%.4f,%.4f\n",millis() - startMillis, voltage, current);
                if(current <= ignoredCurrent) {
                    logFile.close();
                    free(fileName);
                    digitalWrite(GPIO_NUM_3, LOW);
                    Serial.println("Switching to draining mode in 5 seconds!");
                    delay(5000);
                    mode = 1;
                    digitalWrite(GPIO_NUM_8, HIGH);
                    startMillis = millis();
                }
            }
            break;
        case 1:
            if(ina.getBusVoltage() <= drainVoltage) {
                digitalWrite(GPIO_NUM_8, LOW);
                Serial.println("Switching to charging mode in 5 seconds!");
                powerVoltage = ina.getBusVoltage();
                digiPot.write_data(AD5272_RDAC_WRITE, voltageToResistanceStep(powerVoltage));
                resistanceStep = voltageToResistanceStep(powerVoltage);
                digiPot.write_data(AD5272_50TP_WRITE, 0);
                logFileCnt++;
                fileName = (char *)malloc(sizeof(char) * 24);
                sprintf(fileName, "/sdcard/logFile_%03d.csv\0", logFileCnt);
                Serial.print(fileName);
                Serial.println(" opened");
                logFile = SD_MMC.open(fileName, FILE_WRITE, true);
                logFile.println("time_ms,bus_volt,milliamps_raw");
                delay(5000);
                mode = 0;
                digitalWrite(GPIO_NUM_3, HIGH);
                startMillis = millis();
            }
    }

    digiPot.write_data(AD5272_RDAC_WRITE, resistanceStep);
    digiPot.write_data(AD5272_50TP_WRITE, 0);
    while((millis() - loopMillis) < intervalMillis) {
        continue;
    }
}