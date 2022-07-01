
#include <cmath>


byte last_channels[6];
int receiver_input[6];

unsigned long
    current_time ,
    timers[6] ;


int limit(int minimum,int value,int maximum){
    return std::min(std::max(minimum,value),maximum);
}


void setup(){

    Serial.begin(9600);

    //  Set PCIE0 to enable PCMSK2 scan.

    PCICR |= (1 << PCIE2);

    //  Set PCIE0 to enable PCMSK0 scan.

    PCICR |= (1 << PCIE0);

    /*
        Set PCINT to trigger an interrupt
        on state change of a digital input.

        Pins: 2 , 4 , 5 , 6 , 7 , 8
    */

    PCMSK2 |=
        (1 << PCINT18) |
        (1 << PCINT20) |
        (1 << PCINT21) |
        (1 << PCINT22) |
        (1 << PCINT23);

    PCMSK0 |= (1 << PCINT0);
}


void loop(){
    read_pwm();
    printReceiver();
}


void printReceiver(){

    //  Channel 1 - 6

    Serial.print(" PITCH = ");
    Serial.print(receiver_input[0]);

    Serial.print("    ROLL = ");
    Serial.print(receiver_input[1]);

    Serial.print("    Throttle = ");
    Serial.print(receiver_input[2]);

    Serial.print("    YAW = ");
    Serial.print(receiver_input[3]);

    Serial.print("    AUX1 = ");
    Serial.print(receiver_input[4]);

    Serial.print("    AUX2 = ");
    Serial.print(receiver_input[5]);

    Serial.println();
}


void read_pwm(){

    for(int channel = 0;channel < 6;channel++)
        receiver_input[channel] = limit(1000,receiver_input[channel],200);
}


void processChannel(int channel,int input_mask){

    const auto lastChannel = last_channels[channel];


    //  Is input X high?

    if(PIND & input_mask){

        //  Input X changed from 0 to 1.

        if(lastChannel == 0){

            //  Remember current input state.

            last_channels[channel] = 1;

            //  Set timer_1 to current_time.

            timers[channel] = current_time;
        }

        return
    }


    //  Input X is not high and changed from 1 to 0.

    if(lastChannel == 1){

        //  Remember current input state.

        last_channels[channel] = 0;

        //  Channel X is current_time - timer_1.

        receiver_input[channel] = current_time - times[channel];
    }
}


ISR(PCINT2_vect){

    current_time = micros();

    processChannel(0,B00100000);
    processChannel(1,B00010000);
    processChannel(2,B00000100);
    processChannel(3,B01000000);
    processChannel(4,B10000000);
}


ISR(PCINT0_vect){
    processChannel(5,B00000001);
}
