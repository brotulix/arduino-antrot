#include <stdint.h>
#include <Stepper.h>

#if 0 // example

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor


// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int stepCount = 0;  // number of steps the motor has taken

void setup() {
  // nothing to do inside the setup
}

void loop() {
  // read the sensor value:
  int sensorReading = analogRead(A0);
  // map it to a range from 0 to 100:
  int motorSpeed = map(sensorReading, 0, 1023, 0, 100);
  // set the motor speed:
  if (motorSpeed > 0) {
    myStepper.setSpeed(motorSpeed);
    // step 1/100 of a revolution:
    myStepper.step(stepsPerRevolution / 100);
  }
}

#else // my test
#define STEPPER_DESIDEGREES_PER_STEP    18
#define STEPPER_STEPS_PER_REVOLUTION    200
#define STEPPER_PIN_1                   8
#define STEPPER_PIN_2                   9
#define STEPPER_PIN_3                   10
#define STEPPER_PIN_4                   11

// Find by experimentation
#define STEPPER_MAX_RPM                 10
//#define STEPPER_MAX_STEPS_PER_MINUTE    STEPPER_MAX_RPM * STEPPER_STEPS_PER_REVOLUTION

#define INTERVAL_MS                     1000
#define INTERVALS_PER_MINUTE            (float)(60000.0 / INTERVAL_MS)

Stepper stepper(
    STEPPER_STEPS_PER_REVOLUTION, 
    STEPPER_PIN_1,
    STEPPER_PIN_2,
    STEPPER_PIN_3,
    STEPPER_PIN_4
);

uint32_t mli = 0; // when was last iteration?
uint32_t krsfli = 0; // 1000 * remaining steps from last iteration

void setup()
{
    Serial.begin(115200);
    while(!Serial)
    {
        // do naught
    }
    Serial.println("Allo!\n");

    // Set an initial speed of 1 RPM
    stepper.setSpeed(1);

    pinMode(A0, INPUT);

}

#if 0 // old test
void loop()
{
    // Keep track of time (find time delta)
    uint32_t mti = millis();
    if(mti < mli)
    {
        mti = ((uint32_t)-1) - mli + mti;
    }
    else
    {
        mti = mli - mti;
    }

    uint16_t val = 0;
    uint16_t rpm_this_interval = 0;
    uint32_t spm_this_interval = 0;
    uint32_t steps_this_interval = 0;
    uint32_t k_steps_this_interval = 0;
    uint32_t k_max_steps_this_interval = 0;

    if(mti > INTERVAL_MS)
    {
        val = analogRead(A0);

        rpm_this_interval = map(val, 0, 1023, 1, STEPPER_MAX_RPM);
        

        Serial.print("\r");
        Serial.print(spi);
        Serial.print(" steps at ");
        Serial.print(rpm);
        Serial.print(" RPM");
        stepper.setSpeed(abs(rpm));
        
        mli = millis();
        stepper.step(spi);
    }
}
#else // loop
float remainder = 0.0;

void loop() {
    int8_t direction = 1;
    int16_t steps = 0;
    uint16_t val = 0;
    uint16_t setrpm = 0;
    float scaledval = 0.0;
    float scaledrpm = 0.0;
    float scaledsteps = 0.0;

    //val = analogRead(A0); // = 123
    val = 123;
    Serial.print("analogRead: ");
    Serial.print(val);

    scaledval = ((float)val - 512.0) / 512.0; // = 123-512 / 511 = -389/512 = -0.76125
    Serial.print(", scaledval: ");
    Serial.print(scaledval);

    if(scaledval < 0.0) // -0.76125 < 0.0 = true
    {
        direction = -1; // direction = -1
    }
    
    scaledrpm = 1 + (((float)STEPPER_MAX_RPM - 1.0) * scaledval); // 1 + ((10 - 1) * -0.76125) = -5.8513
    Serial.print(", scaledrpm: ");
    Serial.print(scaledrpm);

    setrpm = (uint16_t)(abs(scaledrpm)); // = floor(5.8513) = 5
    Serial.print(", setrpm: ");
    Serial.print(setrpm);
    
    Serial.print(", direction: ");
    Serial.print(direction);
    
    scaledsteps = remainder + (direction * setrpm * STEPPER_STEPS_PER_REVOLUTION / INTERVALS_PER_MINUTE);
    //          =       0.0 + (       -1 *      5 *                          200 * (1.0/       10) * 1/60)
    //          = -1.6667
    Serial.print(", scaledsteps: ");
    Serial.print(scaledsteps);
    
    steps = (int16_t)scaledsteps; // -1
    Serial.print(", steps: ");
    Serial.print(steps);
    
    remainder = scaledsteps - steps; // -1.6667 - (-1) = -0.6667
    Serial.print(", remainder: ");
    Serial.println(remainder);
    
    stepper.setSpeed(setrpm);
    stepper.step(steps);
    
    //delay(INTERVAL_MS);
    delay(10000);
}
#endif // loop
#endif