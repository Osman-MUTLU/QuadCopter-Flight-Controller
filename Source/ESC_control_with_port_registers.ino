

/*
 *  RECEIVER values
 */

int
    receiver_throttle , receiver_yaw , receiver_pitch ,
    receiver_roll , receiver_aux1 , receiver_aux2 ;

byte
    last_channel_1 , last_channel_2 , last_channel_3 ,
    last_channel_4 , last_channel_5 , last_channel_6 ;

unsigned long
    timer_1 , timer_2 , timer_3 , timer_4 ,
    timer_5 , timer_6 , current_time ;

int receiver_input[7];


/*
 *  Motors
 */

int
    min_throttle = 1150 ,
    motor_1_pow = 0 ,
    motor_2_pow = 0 ,
    motor_3_pow = 0 ,
    motor_4_pow = 0 ,
    throttle;


/*
 *  Timers
 */

unsigned long
    timer_motor_1 , timer_motor_2 , timer_motor_3 ,
    timer_motor_4 , esc_timer , esc_loop_timer ,
    loop_timer ;

int timer;



void setup(){

    Serial.begin(9600);

    PCICR |= (1 << PCIE2);

    //  Set PCINT2 (digital input 2) to trigger an interrupt on state change.

    PCMSK2 |= (1 << PCINT18);

    //  Configure digital poort 3 as output.

    DDRD |= B00001000;

    //  Configure digital poort 9 , 10 and 11 as output.

    DDRB |= B00001110;
}


void loop(){

    throttle = receiver_input[3];

    throttle = (throttle > min_throttle + 20)
        ? throttle - (min_throttle - 1000);
        : 1000 ;

    if(throttle > 1600)
        throttle = 2000;

    Serial.print("    Throttle = ");
    Serial.print(throttle); // CH3 Throttle
    Serial.println();

    motor_1_pow
        = motor_2_pow
        = motor_3_pow
        = motor_4_pow
        = throttle ;

    while(micros() - loop_timer < 4000);

    loop_timer = micros();

    //  Set digital poort 3 high.

    PORTD |= B00001000;

    //  Set digital poort 9, 10 and 11 high.

    PORTB |= B00001110;


    //  Calculate the time of the falling edge of the esc-[1,2,3,4] pulse.

    timer_motor_1 = motor_1_pow + loop_timer;
    timer_motor_2 = motor_2_pow + loop_timer;
    timer_motor_3 = motor_3_pow + loop_timer;
    timer_motor_4 = motor_4_pow + loop_timer;

    delayMicroseconds(1000);

    boolean
        esc_1 = false ,
        esc_2 = false ,
        esc_3 = false ,
        esc_4 = false ;

    while(true){

        //  Stay in this loop until output 3,9,10 and 11 are low.

        esc_loop_timer = micros();


        //  Set digital output 3 to low if the time is expired.

        if(timer_motor_1 <= esc_loop_timer){
            PORTD &= B11110111;
            esc_1 = true;
        }

        //  Set digital output 9 to low if the time is expired.

        if(timer_motor_2 <= esc_loop_timer){
            PORTB &= B11110111;
            esc_2 = true;
        }

        //  Set digital output 10 to low if the time is expired.

        if(timer_motor_3 <= esc_loop_timer){
            PORTB &= B11111011;
            esc_3 = true;
        }

        //  Set digital output 11 to low if the time is expired.

        if(timer_motor_4 <= esc_loop_timer){
            PORTB &= B11111101;
            esc_4 = true;
        }

        if(esc_1 && esc_2 && esc_3 && esc_4)
            break;
    }
}


ISR(PCINT2_vect){

    current_time = micros();

    //  Channel 3


    //  Is input 2 high?  Throttle channel 3

    if(PIND & B00000100){

        //  Input 2 changed from 0 to 1.

        if(last_channel_3 == 0){

            //  Remember current input state.

            last_channel_3 = 1;

            //  Set timer_3 to current_time.

            timer_3 = current_time;
        }
    } else

    //  Input 2 is not high and changed from 1 to 0.

    if(last_channel_3 == 1){

        //  Remember current input state.

        last_channel_3 = 0;

        //  Channel 3 is current_time - timer_3.

        receiver_input[3] = current_time - timer_3;

        if(receiver_input[3] < min_throttle)
            min_throttle = receiver_input[3];
    }
}
