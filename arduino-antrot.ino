#include <stdint.h>

/* Speed- and direction controlling a geared DC motor.
 * The intention was to use a L298 as motor controller, but on the first test I
 *   killed the L298 when driving the motor with 40V, then reversing, since the
 *   motor was still spinning and thus set up a reverse voltage. Second test
 *   confirmed this would happen also at 30V. This way for testing was getting
 *   expensive, so I turned to the alternate solution of using relays -- and
 *   adding a slight bit of logic and delays when reversing direction and
 *   stopping.
*/

// Need to be PWM pins for speed control
#define MOTOR_OUT_A     5
#define MOTOR_OUT_B     6
#define ANALOG_INPUT    A0
#define DIGITAL_INPUT_A 2   // Interrupt pin
#define DIGITAL_INPUT_B 3   // Interrupt pin

#define CLOCKWISE           1 // 01
#define COUNTERCLOCKWISE    2 // 10

#define STOP                0
#define RUN                 1

uint8_t direction = CLOCKWISE;
uint8_t run = STOP;
uint32_t lastinterrupt = 0;

void setup()
{
    Serial.begin(115200);
    
    // During debugging do naught until Serial is up
    while(!Serial)
    {
        
    }

    pinMode(MOTOR_OUT_A, OUTPUT);
    pinMode(MOTOR_OUT_B, OUTPUT);
    pinMode(DIGITAL_INPUT_A, INPUT_PULLUP);
    pinMode(DIGITAL_INPUT_B, INPUT_PULLUP);
    pinMode(ANALOG_INPUT, INPUT);

    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_A), directionChange, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_B), toggleRun, FALLING);

    delay(100);

    digitalWrite(MOTOR_OUT_A, direction & CLOCKWISE);
    digitalWrite(MOTOR_OUT_B, direction & COUNTERCLOCKWISE);

    delay(100);

}

void directionChange()
{
    uint32_t thistime = millis();
    if(thistime < lastinterrupt)
    {
        // handle rollover
    }
    
    // normal case
    if(thistime - lastinterrupt > 50)
    {
        direction = (direction ^ 0x3) & 0x3;
        lastinterrupt = thistime;
    }
}

void toggleRun()
{
    uint32_t thistime = millis();
    if(thistime < lastinterrupt)
    {
        // handle rollover
    }
    
    // normal case
    if(thistime - lastinterrupt > 50)
    {
        run = !run;
        lastinterrupt = thistime;
    }
}

void loop()
{
    uint16_t input = analogRead(ANALOG_INPUT);
    
    if(run)
    {
        // on/off
        digitalWrite(MOTOR_OUT_A, direction & CLOCKWISE);
        digitalWrite(MOTOR_OUT_B, direction & COUNTERCLOCKWISE);
        
        // speed control
        //analogWrite(MOTOR_OUT_A, ((direction & CLOCKWISE) ? input : 0x0000));
        //analogWrite(MOTOR_OUT_B, ((direction & COUNTERCLOCKWISE) ? input : 0x0000));
        // delay(50);
    }
    else
    {
        digitalWrite(MOTOR_OUT_A, 0);
        digitalWrite(MOTOR_OUT_B, 0);
    }
    

}