
#include "Wire.h"

float
    gyro_pitch_input , gyro_yaw_input , gyro_roll_input ,
    calX , calY , calZ ,
    rotX , rotY , rotZ ;

long gyroX , gyroY , gyroZ ;

int
    counter = 0 ,
    cal_int ;


void setup(){

    /*
        SENSOR SIGNAL

        X = PITCH
        Y = ROLL
        Z = YAW

        If you want to change the directions of the
        movement direction, change the rotations.

        Go to the bottom of the code...
    */

    Wire.begin();
    Serial.begin(9600);
    setupMPU();
}


// Put your main code here, to run repeatedly:

void loop(){
    recordGyroRegisters();
    printMPU();
}


void printMPU(){

    counter++;

    if(counter % 100==0){

        // Ön negatif(-), Arka pozitif(+)

        Serial.print("PITCH =   ");
        Serial.print(gyro_pitch_input);

        // Sol negatif(-), Sağ pozitif(+)

        Serial.print("    Roll =   ");
        Serial.print(gyro_roll_input);

        // Saat yönü negatif(-), Saatin tersi (+)

        Serial.print("    yaw =   ");
        Serial.print(gyro_yaw_input);

        Serial.println();
        counter = 0;
    }
}


/*
    MPU650 READING DATA AND CALCULATING ERRORS
    https://www.youtube.com/watch?v=UxABxSADZ6U
*/

void setupMPU(){

    /*
        This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
        Accessing the register 6B - Power Management (Sec. 4.28)
        Setting SLEEP register to 0. (Required; see Note on p. 9)
    */

    Wire.beginTransmission(0b1101000);
    Wire.write(0x6B);
    Wire.write(0b00000000);
    Wire.endTransmission();

    /*
        I2C address of the MPU
        Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
        Setting the gyro to full scale +/- 250deg./s
    */

    Wire.beginTransmission(0b1101000);
    Wire.write(0x1B);
    Wire.write(0x00000000);
    Wire.endTransmission();

    /*
        I2C address of the MPU
        Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
        Setting the accel to +/- 2g
    */

    Wire.beginTransmission(0b1101000);
    Wire.write(0x1C);
    Wire.write(0b00000000);
    Wire.endTransmission();

    Serial.print("Reading MPU 6050 .");

    for(cal_int = 0;cal_int < 2000;cal_int++){

        recordGyroRegisters();

        if(cal_int % 200 == 0)
            Serial.print(".");

        calX += rotX;
        calY += rotY;
        calZ += rotZ;

        delayMicroseconds(1000);
        delay(3);
    }

    Serial.println();

    calX /= 2000;
    calY /= 2000;
    calZ /= 2000;
}


void processGyroData(){

    rotX = gyroX / 131.0;
    rotY = gyroY / 131.0;
    rotZ = gyroZ / 131.0;

    if(cal_int == 2000){
        rotX -= calX;
        rotY -= calY;
        rotZ -= calZ;
    }
}


void recordGyroRegisters(){

    /*
        I2C address of the MPU
        Starting register for Gyro Readings
        Request Gyro Registers (43 - 48)
    */

    Wire.beginTransmission(0b1101000);
    Wire.write(0x43);
    Wire.endTransmission();

    Wire.requestFrom(0b1101000,6);

    while(Wire.available() < 6);

    /*
        Store first two bytes into accelX
        Store middle two bytes into accelY
        Store middle two bytes into accelY
    */

    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();

    processGyroData();

    /*
        If you want to change the directions of the
        movement direction, change the rotations.!!!
    */

    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((rotX / 65.5) * 0.3);
    gyro_roll_input = (gyro_roll_input * 0.7) + ((rotY / 65.5) * 0.3);
    gyro_yaw_input = (gyro_yaw_input * 0.7) + ((rotZ / 65.5) * 0.3);
}
