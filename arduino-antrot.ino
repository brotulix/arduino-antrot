#include <stdint.h>
#include <EtherCard.h> // for ENC28J60 module
#include <IPAddress.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

static byte mymac[] = { 0x00, 0x90, 0x62, 0xF1, 0xFA, 0xF0 };
byte Ethernet::buffer[384];
const uint16_t srcPort = 31550;
const uint16_t dstPort = 31560;

#define COMMAND_LINE_MAX_COMMAND_LENGTH         8 // Currently something like "#G,360;"
#define COMMAND_LINE_MIN_COMMAND_LENGTH         3 // Currently something like "#P;"
#define COMMAND_LINE_BUFFER_SIZE                64

char cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };
uint8_t cmdlength = 0;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#define MAX_SEQUENCE_LENGTH     20
#define ANALOG_INPUT            A0
#define DIGITAL_INPUT_A         2 // Interrupt pin
#define DIGITAL_INPUT_B         3 // Interrupt pin

// Need to be PWM pins for speed control
#define MOTOR_PWM           MOTOR_OUT_A

#define CLOCKWISE           0
#define COUNTERCLOCKWISE    1

#define STOP                0
#define RUN                 1

#define SPEED_CREEP         0
#define SPEED_FAST          1

uint32_t lastinterrupt = 0;

enum outputPins
{
    OUTPUT_PIN_UNUSED          = -1,
    OUTPUT_PIN_MOTOR_OUT_A     = 5,
    OUTPUT_PIN_MOTOR_OUT_B     = 6,
    OUTPUT_PIN_RELAY_OUT_A     = 7,
    OUTPUT_PIN_RELAY_OUT_B     = 8,
    OUTPUT_PIN_RELAY_OUT_C     = 9,
    OUTPUT_PIN_ETHER_CS        = 10
};

/*
    RELAY_VOLTAGE         = RELAY_OUT_A,
    RELAY_DIRECTION     = RELAY_OUT_B,
    RELAY_DRIVE         = RELAY_OUT_C
*/


typedef enum OutputType
{
    OUTPUT_TYPE_RELAY = 0,
    OUTPUT_TYPE_PWM,
    OUTPUT_TYPE_VARIABLE,
    OUTPUT_TYPE_STATE
} tOutputType;

typedef enum Outputs
{
    OUTPUT_RELAY_VOLTAGE,       // For setting the voltage relay
    OUTPUT_RELAY_DRIVE,         // For setting the drive relay
    OUTPUT_RELAY_DIRECTION,     // For setting the direction relay
    OUTPUT_PWM_MOTOR_A,         // For setting the PWM output
    OUTPUT_VARIABLE_HEADING,    // For storing a target heading to reach
    OUTPUT_VARIABLE_SPEED,      // For storing the target motor speed
    OUTPUT_VARIABLE_STATE,      // For storing the state
    OUTPUT_STATE,               // For triggering state change
    OUTPUT_NUM_VALUES
} tOutputs;

typedef struct Output
{
    tOutputs output;
    tOutputType type;
    int pin;
    int value;
} tOutput;

tOutput output[OUTPUT_NUM_VALUES];

// @todo: Consider encoding both output type and pin number in same enum value
//          (eg. MSB as tOutputType) and LSB as tOutputChannel).
//          Or a struct with known pin and variable definitions
typedef struct sequence_step
{
    uint16_t countdown;
    int16_t value;
    tOutputs channel;
} tSequenceStep;

typedef struct sequence
{
    uint8_t items;
    tSequenceStep sequence[MAX_SEQUENCE_LENGTH];
} tSequence;

tSequence sequence;


typedef enum States
{
    STATE_IDLE          = 0,
    STATE_STARTING      = 1,
    STATE_TRAVERSING    = 2,
    STATE_REVERSING     = 3,
    STATE_STOPPING      = 4,
    STATE_CORRECTING    = 5,
    STATE_ERROR         = 6,
    STATE_MAX_STATE     = STATE_ERROR
} tStates;


#define HEADING_AVERAGE_NUM_VALS    10
typedef struct sHdgAvg {
    uint16_t numvals;
    uint16_t nextval;
    float vals[HEADING_AVERAGE_NUM_VALS];
} tHdgAvg;

tHdgAvg hdgavg = { 0 };

void hdgavgClear()
{
    uint8_t i = 0;
    
    hdgavg.nextval = 0;
    hdgavg.numvals = 0;
    
    for(i = 0; i < HEADING_AVERAGE_NUM_VALS; i++)
    {
        hdgavg.vals[i] = 0.0;
    }
}

float hdgavgGet()
{
    uint8_t i = 0;
    float sum = 0.0;
    for(i = 0; i < hdgavg.numvals; i++)
    {
        sum += hdgavg.vals[i];
    }

    return sum / hdgavg.numvals;
}

void hdgavgPut(float val)
{
    uint8_t i = 0;
    
    hdgavg.vals[hdgavg.nextval++] = val;
    
    if(hdgavg.nextval >= HEADING_AVERAGE_NUM_VALS)
    {
        hdgavg.nextval = 0;
    }
    
    if(hdgavg.numvals < HEADING_AVERAGE_NUM_VALS)
    {
        hdgavg.numvals++;
    }
}




void initializeOutputs()
{
    int i = 0;

    output[OUTPUT_RELAY_VOLTAGE] = 
    {
        OUTPUT_RELAY_VOLTAGE,
        OUTPUT_TYPE_RELAY,
        OUTPUT_PIN_RELAY_OUT_A,
        0
    };
    
    output[OUTPUT_RELAY_DRIVE] = 
    {
        OUTPUT_RELAY_DRIVE,
        OUTPUT_TYPE_RELAY,
        OUTPUT_PIN_RELAY_OUT_B,
        0
    };
    
    output[OUTPUT_RELAY_DIRECTION] = 
    {
        OUTPUT_RELAY_DIRECTION,
        OUTPUT_TYPE_RELAY,
        OUTPUT_PIN_RELAY_OUT_C,
        0
    };
    
    output[OUTPUT_PWM_MOTOR_A] = 
    {
        OUTPUT_PWM_MOTOR_A,
        OUTPUT_TYPE_PWM,
        OUTPUT_PIN_MOTOR_OUT_A,
        0
    };
    
    output[OUTPUT_VARIABLE_HEADING] = 
    {
        OUTPUT_VARIABLE_HEADING,
        OUTPUT_TYPE_VARIABLE,
        OUTPUT_PIN_UNUSED,
        0
    };

    output[OUTPUT_VARIABLE_SPEED] = 
    {
        OUTPUT_VARIABLE_SPEED,
        OUTPUT_TYPE_VARIABLE,
        OUTPUT_PIN_UNUSED,
        0
    };

    output[OUTPUT_VARIABLE_STATE] =
    {
        OUTPUT_VARIABLE_STATE,
        OUTPUT_TYPE_VARIABLE,
        OUTPUT_PIN_UNUSED,
        0
    };

    output[OUTPUT_STATE] = 
    {
        OUTPUT_STATE,
        OUTPUT_TYPE_STATE,
        OUTPUT_PIN_UNUSED,
        0
    };

    for(i = 0; i < OUTPUT_NUM_VALUES; i++)
    {
        if(output[i].pin != OUTPUT_PIN_UNUSED)
        {
            pinMode(output[i].pin, OUTPUT);
        }

        setOutput(output[i].output, 0);
    }
}



void setOutput(tOutputs o, int16_t v)
{
    switch (output[o].type)
    {
        case OUTPUT_TYPE_RELAY:
        {
            if(v < 0)
            {
                output[o].value = !output[o].value;
            }
            else
            {
                output[o].value = v;
            }

            if(output[o].pin != OUTPUT_PIN_UNUSED)
            {
                digitalWrite(output[o].pin, output[o].value);
            }
            
            break;
        }

        case OUTPUT_TYPE_PWM:
        {
            if(v<0)
            {
                output[o].value = 0;
            }
            else
            {
                output[o].value = v;
            }

            analogWrite(output[o].pin, output[o].value);
            
            break;
        }

        case OUTPUT_TYPE_VARIABLE:
        {
            output[o].value = v;
            break;
        }

        case OUTPUT_TYPE_STATE:
        {
            setState(v);
            
            break;
        }
        
        default:
        {
            Serial.print("#E,setOutput: Unknown pin ");
            Serial.print(o);
            Serial.println(";");
            break;
        }
    }
}

void clearSequence()
{
    uint16_t i = 0;
    char* ptr = (char*)&sequence;
    for(i = 0; i < sizeof(tSequence); i++)
    {
        ptr[i] = 0;
    }
}

void printSequence()
{
    uint8_t i = 0;
    Serial.print("Items in sequence: ");
    Serial.println(sequence.items);
    
    //for(i = 0; i < MAX_SEQUENCE_LENGTH; i++)
    for(i = 0; i < sequence.items; i++)
    {
        Serial.print("- Item ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sequence.sequence[i].countdown);
        Serial.print(", ");
        Serial.print(sequence.sequence[i].value);
        Serial.print(", ");
        Serial.println(sequence.sequence[i].channel);
    }
}

uint8_t getStepCount()
{
    return sequence.items;
}

int8_t pushSequence(tSequenceStep* srcstep)
{
    if(sequence.items >= MAX_SEQUENCE_LENGTH)
    {
        Serial.println("Failed to push sequence step!");
        return -1;
    }

    sequence.sequence[sequence.items].countdown = srcstep->countdown;
    sequence.sequence[sequence.items].value     = srcstep->value;
    sequence.sequence[sequence.items].channel   = srcstep->channel;

    sequence.items++;

    //printSequence();

    return (int8_t)sequence.items;


}

// Pop first item from queue.
// Returns number of items left in queue, or -1 if queue already was empty.
// If argument is not NULL, first item is copied to pointer address before sequence shift.
// Rest of sequence is shifted one step, empty steps are additionally cleared.
int8_t popSequence(tSequenceStep* dststep)
{
    int i = 0;

    if(sequence.items == 0)
    {
        Serial.println("Sequence already empty!");
        clearSequence();
        return -1;
    }

    if(dststep != NULL)
    {
        dststep->countdown  = sequence.sequence[0].countdown;
        dststep->value      = sequence.sequence[0].value;
        dststep->channel    = sequence.sequence[0].channel;
    }

    if(sequence.items == 1)
    {
        clearSequence();
        return (int8_t)sequence.items;
    }

    sequence.items--;

    // Copy items one position forward in queue:
    for(i = 0; i < sequence.items; i++)
    {
        sequence.sequence[i].countdown  = sequence.sequence[i+1].countdown;
        sequence.sequence[i].value      = sequence.sequence[i+1].value;
        sequence.sequence[i].channel    = sequence.sequence[i+1].channel;
    }

    // Clear remaining positions:
    for(;i < MAX_SEQUENCE_LENGTH; i++)
    {
        sequence.sequence[i].countdown  = 0;
        sequence.sequence[i].value      = 0;
        sequence.sequence[i].channel    = 0;
    }

    //printSequence();

    return (int8_t)sequence.items;

}

int8_t replaceSequence(tSequenceStep* new_step)
{
    if(sequence.items == 0)
    {
        return -1;
    }

    if(new_step == NULL)
    {
        return -1;
    }

    sequence.sequence[sequence.items].countdown = new_step->countdown;
    sequence.sequence[sequence.items].value     = new_step->value;
    sequence.sequence[sequence.items].channel   = new_step->channel;

    return 0;
}

int8_t peekStep(tSequenceStep* dststep, uint8_t src_step)
{
    if(src_step > sequence.items)
    {
        return -1;
    }

    if(dststep != NULL)
    {
        dststep->countdown = sequence.sequence[src_step].countdown;
        dststep->value     = sequence.sequence[src_step].value;
        dststep->channel   = sequence.sequence[src_step].channel;
    }

    return (int8_t)src_step;
}

/*
void sendMsg((char*)src, uint16_t len)
{
    ether.sendUdp(src, len, srcPort, <ip>, dstPort );
}
*/

int16_t stepDance(uint16_t milliseconds)
{
    if(sequence.items == 0)
    {
        // Non-fatal error
        return -1;
    }

    if(sequence.sequence[0].countdown > milliseconds)
    {
        sequence.sequence[0].countdown -= milliseconds;
        return sequence.sequence[0].countdown;
    }

    //Serial.println("Executing step!");

    setOutput(sequence.sequence[0].channel, sequence.sequence[0].value);

    popSequence(NULL);
}

void softStart()
{
    tSequenceStep step = {0};
    uint16_t curval = output[OUTPUT_VARIABLE_SPEED].value;
    //clearSequence();
    step = {0, 0, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    //step = {1000, 0, OUTPUT_RELAY_VOLTAGE};
    //pushSequence(&step);
    step = {100, 1, OUTPUT_RELAY_DRIVE};
    pushSequence(&step);
    step = {100, curval/8, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, curval/4, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, curval/2, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, curval, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
}

void softStop()
{
    tSequenceStep step = {0};
    int16_t curval = output[OUTPUT_VARIABLE_SPEED].value;
    //clearSequence();
    step = {0, curval/2, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, curval/4, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, curval/8, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, 0, OUTPUT_PWM_MOTOR_A};
    pushSequence(&step);
    step = {100, 0, OUTPUT_RELAY_DRIVE};
    pushSequence(&step);
    //step = {1000, 0, OUTPUT_RELAY_VOLTAGE};
    //pushSequence(&step);
}

void hardStop()
{
    setOutput(OUTPUT_PWM_MOTOR_A, 0);
    setOutput(OUTPUT_RELAY_DRIVE, 0);
    setOutput(OUTPUT_RELAY_VOLTAGE, 0);
}

void setState(uint8_t newState)
{
    tStates set = STATE_ERROR;
    tSequenceStep step = {0};
    int16_t curval = 0;
    
    if(newState <= STATE_MAX_STATE)
    {
        set = newState;
    }

    Serial.print(millis());
    Serial.print(": S");
    Serial.print(output[OUTPUT_VARIABLE_STATE].value);
    Serial.print(" -> S");
    Serial.println(set);

    switch(set)
    {
        default:
        case STATE_IDLE:
        {
            break;
        }
        case STATE_STARTING:
        {
            clearSequence();
            softStart();
            step = {0, STATE_TRAVERSING, OUTPUT_STATE};
            pushSequence(&step);
            break;
        }
        case STATE_TRAVERSING:
        {
            break;
        }
        case STATE_REVERSING:
        {
            clearSequence();
            softStop();
            step = {0, -1, OUTPUT_RELAY_DIRECTION};
            pushSequence(&step);
            softStart();
            step = {0, STATE_TRAVERSING, OUTPUT_STATE};
            pushSequence(&step);
            break;
        }
        case STATE_STOPPING:
        {
            clearSequence();
            softStop();
            step = {0, STATE_IDLE, OUTPUT_STATE};
            pushSequence(&step);
            break;
        }
        case STATE_CORRECTING:
        {
            break;
        }
        case STATE_ERROR:
        {
            clearSequence();
            
            // Set STOP sequence
            hardStop();

            break;
        }
    }

    //step = {0, set, OUTPUT_VARIABLE_STATE};
    //pushSequence(&step);

    setOutput(OUTPUT_VARIABLE_STATE, set);

}

/** Check for bytes in UART receive buffer, and append them to our serial line
 *    buffer, or at least as many as there is room for at the moment.
 */
void grabSerial()
{
    int cmdlen = 0;
    int i = 0;
    int avail = 0;

    // Check for bytes in UART receive buffer
    cmdlen = Serial.available();

    if(cmdlen == 0)
    {
        return;
    }

    /*
    Serial.print("There's ");
    Serial.print(cmdlen);
    Serial.println(" bytes to be read.");
    */

    avail = COMMAND_LINE_BUFFER_SIZE - cmdlength;

    /*
    Serial.print("Can write ");
    Serial.print(avail);
    Serial.println(" bytes to buffer.");
    */

    if(avail <= 0)
    {
        return;
    }
    
    if(cmdlen > avail)
    {
        cmdlen = avail;
    }

    /*
    Serial.print("Grabbing ");
    Serial.print(cmdlen);
    Serial.println(" bytes from serial port.");
    */


    //cmdlength += Serial.readBytes((char*)(&cmdline[cmdlength]), avail);

    cmdlength += Serial.readBytes((char*)(&cmdline[cmdlength]), cmdlen);

    //appendCommand();

}

void appendCommand(char* cmd, uint16_t len)
{
    uint16_t i = 0;
    int16_t avail = 0;

    avail = COMMAND_LINE_BUFFER_SIZE - cmdlength;

    if(avail >= len)
    {
        Serial.print("Copying ");
        Serial.print(len);
        Serial.println(" bytes from UDP packet.");
        for(i = 0; i < len; i++)
        {
            cmdline[cmdlength + i] = cmd[i];
        }

        cmdlength += len;
    }
}

/** Remove all characters in array up to the specified character.
 * @param   i   The last character to remove (eg. linefeed).
 */
void stripCommand(uint8_t i)
{
    uint8_t j = 0;
    
    if(i == 0)
    {
        return;
    }

    if(i > COMMAND_LINE_BUFFER_SIZE)
    {
        // This is an error, but we "solve" it by clearing the whole buffer.
        i = COMMAND_LINE_BUFFER_SIZE;
    }
    else
    {
        // Shift the rest of the command buffer i characters to the left
        for(j = 0; j < COMMAND_LINE_BUFFER_SIZE; j++)
        {
            if(j < COMMAND_LINE_BUFFER_SIZE - (i + 1))
            {
                cmdline[j] = cmdline[j + i];
            }
            else
            {
                cmdline[j] = '\0';
            }
            
        }
    }
    
    // Decrement length counter
    if(i > cmdlength)
    {
        cmdlength = 0;
    }
    else
    {
        cmdlength -= i;
    }

    /*
    Serial.print("Command string: [");
    Serial.flush();
    for(j = 0; j < COMMAND_LINE_BUFFER_SIZE; j++)
    {
        Serial.print(cmdline[j] > 20 ? cmdline[j] : '.');
        Serial.flush();
    }
    Serial.println("]");
    Serial.flush();
    */
}

uint8_t parseCommand(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint16_t p = 0;
    int32_t v = 0;
    int8_t k = 0;
    uint8_t c = 0;
    uint8_t begin = cmdlength;
    uint8_t end = 0;
    uint8_t cmdlen = 0;

    if(cmdlength == 0)
    {
        // Do nothing if the buffer is empty
        return 0;
    }
    
    // Locate a command
    for(i = 0; i < cmdlength; i++)
    {
        if(begin == cmdlength)
        {
            if(cmdline[i] == '#')
            {
                begin = i;
            }
        }
        
        if(end == 0)
        {
            if(cmdline[i] == ';')
            {
                end = i;
            }
        }
    }

    if(begin == cmdlength)
    {
        // There's no command start character in buffer; we can toss it all
        stripCommand(cmdlength);
        return cmdlength;
    }

    if(begin > 0)
    {
        // Command start character is not the first in buffer; clean out crap
        stripCommand(begin);
        return begin;
    }
    
    if(begin > end)
    {
        // There's garbage.
        stripCommand(begin);
        return begin;
    }

    cmdlen = (end - begin) + 1;
    
    if((cmdlen > COMMAND_LINE_MAX_COMMAND_LENGTH) || (cmdlen < COMMAND_LINE_MIN_COMMAND_LENGTH))
    {
        // Command is too long or too short; it's rubbish
        stripCommand(end);
        return end;
    }

    Serial.print("Command accepted (");
    Serial.print(cmdlen);
    Serial.print("): [");
    for(i = 0; i < cmdlen; i++)
    {
        // Don't print those pesky control characters
        if(cmdline[i] < 32)
        {
            Serial.print(".");
        }
        else
        {
            Serial.print(cmdline[i]);
        }
    }
    Serial.println("]");

    i = 1;
    j = i;

    // Ok, there's a command at the start of buffer, and it's not too long or short, so we look at what it is...
    switch (cmdline[1])
    {
        case 'P':
        {
            Serial.println("#P;");
            // netSend("#P;", 3);
            break;
        }

        case 'S':
        {
            if(cmdlen == COMMAND_LINE_MIN_COMMAND_LENGTH)
            {
                // Report current state
                Serial.print("S");
                Serial.println(output[OUTPUT_VARIABLE_STATE].value);
                break;
            }

            if(cmdlen > (COMMAND_LINE_MIN_COMMAND_LENGTH + 1))
            {
                Serial.println("#E;");
                break;
            }

            j = (uint8_t)cmdline[++j] - 0x30;
            if(j > STATE_MAX_STATE)
            {
                // Error!
                Serial.println("Error parsing target state!");
            }
            else
            {
                setState(j);
            }
            break;
        }

        case 'F': // Fast mode
        {
            if(cmdlen == COMMAND_LINE_MIN_COMMAND_LENGTH)
            {
                // Report current value
                Serial.print("#F,");
                Serial.print(output[OUTPUT_RELAY_VOLTAGE].value);
                Serial.println(";");
                break;
            }

            if(cmdlen > (COMMAND_LINE_MIN_COMMAND_LENGTH + 1))
            {
                Serial.println("#E;");
                break;
            }

            if(output[OUTPUT_VARIABLE_STATE].value != STATE_IDLE)
            {
                Serial.println("#E,Illegal state for voltage change;");
                break;
            }

            j = (uint8_t)cmdline[++j] - 0x30;
            if(j > 1)
            {
                // Illegal value, just set it to the safe choice:
                j = 0;
            }
            else
            {
                setOutput(OUTPUT_RELAY_VOLTAGE, j);
            }
            break;
        }

        case 'D':
        {
            if(cmdlen == COMMAND_LINE_MIN_COMMAND_LENGTH)
            {
                // Report current state
                Serial.print("#D");
                Serial.print(output[OUTPUT_RELAY_DIRECTION].value);
                Serial.println(";");
                break;
            }

            if(cmdlen > (COMMAND_LINE_MIN_COMMAND_LENGTH + 2))
            {
                Serial.println("#E;");
                break;
            }

            if(cmdline[j+1] == '-')
            {
                j++;
                k = -1;
            }
            else
            {
                k = 1;
            }

            j = (uint8_t)cmdline[++j] - 0x30;
            if(j > 1)
            {
                // Illegal value, just set it to the safe choice:
                j = 0;
            }

            if(output[OUTPUT_VARIABLE_STATE].value == STATE_IDLE)
            {
                setOutput(OUTPUT_RELAY_DIRECTION, k * j);
                break;
            }

            if(output[OUTPUT_VARIABLE_STATE].value == STATE_TRAVERSING)
            {
                setState(STATE_REVERSING);
                break;
            }

            Serial.println("#E;");
            break;
        }
        
        case 'A': // Analog output value; set PWM rate
        {
            if(cmdlen == COMMAND_LINE_MIN_COMMAND_LENGTH)
            {
                // Report current state
                Serial.print("#A");
                Serial.print(output[OUTPUT_VARIABLE_SPEED].value);
                Serial.println(";");

                break;
            }

            if(cmdlen > (COMMAND_LINE_MIN_COMMAND_LENGTH + 4))
            {
                Serial.println("#E;");
                break;
            }

            v = 0;
            p = 1;
            for(j = 2; j < (cmdlen - i); j++)
            {

                k = (uint8_t)cmdline[cmdlen - j] - 0x30;
                Serial.print("cmdline[");
                Serial.print(cmdlen - j);
                Serial.print("] is ");
                Serial.println(k);
                    
                if(k <= 9) // it's cast to uint, so can't be less than 0
                {
                    v += p * k;
                    Serial.print("New value: ");
                    Serial.println(v);
                    p = p * 10;
                }
                else
                {
                    break;
                }
            }

            if(j == 0)
            {
                Serial.println("#E,Error parsing speed value;");
                break;
            }

            setOutput(OUTPUT_VARIABLE_SPEED, v);

            break;
        }
        
        case 'H': // Set target heading
        {
            if(cmdlen == COMMAND_LINE_MIN_COMMAND_LENGTH)
            {
                // Report current state
                Serial.print("#H,C,");
                Serial.print(hdgavgGet());
                Serial.print(",T,");
                Serial.print(output[OUTPUT_VARIABLE_HEADING].value);
                Serial.println(";");

                break;
            }

            if(cmdlen > (COMMAND_LINE_MIN_COMMAND_LENGTH + 4))
            {
                Serial.println("#E;");
                break;
            }

            if(cmdline[j+1] == '-')
            {
                k = -1;
            }
            else
            {
                k = 1;
            }

            v = 0;
            p = 1;
            for(j = 2; (k > 0 ? (j < (cmdlen - i)) : (j < (cmdlen - (i + 1)))); j++)
            {
                c = (uint8_t)cmdline[cmdlen - j] - 0x30;
                
                /*
                Serial.print("cmdline[");
                Serial.print(cmdlen - j);
                Serial.print("] is ");
                Serial.println(c);
                */
                    
                if(c >= 0 && c <= 9)
                {
                    v += p * c;
                    p = p * 10;
                }
                else
                {
                    break;
                }
            }

            v *= k;
            /*
            Serial.print("New value: ");
            Serial.println(v);
            */

            if(k > 0 ? (j < (cmdlen - i)) : (j < (cmdlen - (i + 2))))
            {
                Serial.println("#E,Error parsing heading value;");
                break;
            }

            if(v < -1 || v > 359)
            {
                Serial.println("#E,Heading outside bounds;");
                break;
            }

            Serial.print("#H,T,");
            Serial.print(v);
            Serial.println(";");
            
            setOutput(OUTPUT_VARIABLE_HEADING, v);
            break;
        }
        
        default:
        {
            Serial.println("#E;");
            // netSend("#E;", 3);
            break;
        }
    }

    stripCommand(cmdlen);
    
    return cmdlen;
} // parseCommand()

void netRecv(
    uint16_t dest_port,
    uint8_t src_ip[IP_LEN],
    uint16_t src_port,
    const char *data,
    uint16_t len
)
{
    uint16_t i = 0;
    uint16_t begin = len;
    uint16_t end = 0;
    //IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);

    if(dest_port != srcPort)
    {
        return;
    }

    Serial.print("Received packet (");
    Serial.print(len);
    Serial.print(" bytes) from ");
    ether.printIp(src_ip);
    Serial.print(":");
    Serial.print(src_port);
    Serial.print(": [");
    for(i = 0; i < len; i++)
    {
        // Don't print those pesky control characters
        if(data[i] < 32)
        {
            Serial.print(".");
        }
        else
        {
            Serial.print(data[i]);
        }
    }
    Serial.println("]");

    appendCommand(data, len);


}

void clearCommandBuffer()
{
    uint8_t cmdlength = 0;
    //Serial.println("Clearing command buffer...");
    for(cmdlength = COMMAND_LINE_BUFFER_SIZE; cmdlength > 0; cmdlength--)
    {
        cmdline[cmdlength-1] = '\0';
    }
}

void setup()
{
    Serial.begin(115200);
    //Serial.begin(460800);
    
    // During debugging do naught until Serial is up
    while(!Serial)
    {
        
    }

    Serial.print("\r");
    Serial.println("arduino-antrot 2020-04-06");

    Serial.println("Initializing outputs...");
    
    initializeOutputs();
    
    delay(100);

    Serial.println("Initializing inputs...");

    pinMode(DIGITAL_INPUT_A,    INPUT_PULLUP);
    pinMode(DIGITAL_INPUT_B,    INPUT_PULLUP);
    pinMode(ANALOG_INPUT,       INPUT);

    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_A), directionChange, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_B), toggleRun, FALLING);

    delay(100);

    Serial.println("Initializing Ethernet...");

    // Change 'SS' to your Slave Select pin, if you arn't using the default pin
    if (!ether.begin(sizeof Ethernet::buffer, mymac, OUTPUT_PIN_ETHER_CS))
    {
        //Serial.println( "Failed to access Ethernet controller!");
        Serial.println("#E,Ethernet initialization failed;");
        while(!ether.begin(sizeof Ethernet::buffer, mymac, OUTPUT_PIN_ETHER_CS))
        {
            delay(10);
        }
    }

    if (!ether.dhcpSetup())
    {
        Serial.println("#E,DHCP failed;");
    }

    Serial.print("#I,");
    ether.printIp(ether.myip);
    Serial.print(",");
    ether.printIp(ether.gwip);
    Serial.print(",");
    ether.printIp(ether.dnsip);
    Serial.println(";");

    ether.udpServerListenOnPort(&netRecv, srcPort);

    Serial.println("Initializing magnetometer...");
    if(!mag.begin())
    {
        Serial.println("#E,Magnetometer error;");
        while(1)
        {
            // Do naught
            delay(10);
        }
    }

    Serial.println("Initializing hdgavg...");
    hdgavgClear();

    Serial.println("Done.");
}

void directionChange()
{
    static uint32_t last = millis();
    uint32_t thistime = millis();
    uint32_t diff = 0;
    
    if(last > thistime)
    {
        // handle rollover
        diff--; // Roll over to max uint32_t
        diff -= last; // Subtract last to get remainder until rollover
        diff += thistime; // Add current time
    }
    else
    {
        diff = thistime - last;
    }
    
    // normal case
    if(diff > 50)
    {
        setOutput(OUTPUT_RELAY_DIRECTION, -1);
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
    }
}

float getCompassHeading()
{
    sensors_event_t event; 
    float heading = 0.0;
    float temp = 0.0;
    
    // @TODO: Add configuration option for declination
    const float declinationRadians = 0.064577; // Approx +3.7° for JO59 in 2020.
    
    mag.getEvent(&event);

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    /*
    Serial.print("#M,");
    Serial.print(event.magnetic.x);
    Serial.print(",");
    Serial.print(event.magnetic.y);
    Serial.print(",");
    Serial.print(event.magnetic.z);
    Serial.println(",uT;");
    */

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    
    // @TODO: Check installed orientation of sensor
    heading = atan2(event.magnetic.y, event.magnetic.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    heading += declinationRadians;

    // Correct for when signs are reversed.
    if(heading < 0)
    {
        heading += 2 * M_PI;
    }

    // Check for wrap due to addition of declination.
    if(heading > 2 * M_PI)
    {
        heading -= 2 * M_PI;
    }

    // Convert to degrees
    temp = (heading * 0.5 * M_1_PI * 360.0) + 0.5;

    /*
    Serial.print("#H,");
    Serial.print(millis());
    Serial.print(",ms,");
    Serial.print(heading);
    Serial.print(",frad,");
    Serial.print(temp);
    Serial.println(",fdeg;");
    */

    return temp;
}

void stateMachine()
{
    static uint32_t lastreading = 0;
    static uint32_t lastaverage = 0;
    static float lasthdg = -1.0;

    tSequenceStep step = {0};
    uint32_t now = millis();
    uint32_t readdiff = 0;
    uint32_t avgdiff = 0;
    float dps = 0.0; // degrees per iteration
    float hdg = 0.0; // current heading
    float dlh = 0.0; // delta last heading
    float dth = 0.0; // delta target heading
    uint32_t iacs = 0; // iterations at current speed

    // First call regardless of state:
    if(lasthdg < 0.0 && lastreading == 0)
    {
        Serial.println("Getting first reading...");
        //Serial.print("Getting first reading @ ");
        lastreading = now; //millis();
        lastaverage = now; //millis();
        hdg = getCompassHeading();
        
        lasthdg = hdg;
        
        hdgavgPut(hdg);

        //Serial.print("lasthdg is now: ");
        //Serial.println(lasthdg);
    }
    else
    {
        if(now > lastreading)
        {
            readdiff = now - lastreading;
        }
        else
        {
            readdiff = ((uint32_t)-1) - lastreading;
            readdiff += now; // add wraparound positive value
        }
        
        if(now > lastaverage)
        {
            avgdiff = now - lastaverage;
        }
        else
        {
            avgdiff = ((uint32_t)-1) - lastaverage;
            avgdiff += now; // add wraparound positive value
        }
    }
    
    switch (output[OUTPUT_VARIABLE_STATE].value)
    {
        default:
        case STATE_IDLE:
        {
            break;
        }

        case STATE_STARTING:
        {
            break;
        }

        case STATE_TRAVERSING:
        {
            
            // If no step, add one in a bit:
            if(!getStepCount())
            {
                //Serial.println("Sequence empty, adding refresh step...");
                step = {
                    100,                               // countdown
                    output[OUTPUT_VARIABLE_SPEED].value,   // value
                    OUTPUT_PWM_MOTOR_A                  // channel
                };
                pushSequence(&step);
            }

            if(lastreading != now && readdiff >= 10)
            {
                hdg = getCompassHeading();

                hdgavgPut(hdg);
                lastreading = now;
            }

            if(lastaverage != now && avgdiff >= 500)
            {
                hdg = hdgavgGet();

                Serial.print("#H,C,");
                Serial.print(hdg);
                Serial.print(",T,");
                Serial.print(output[OUTPUT_VARIABLE_HEADING].value);
                Serial.println(";");

                if(output[OUTPUT_VARIABLE_HEADING].value != -1)
                {
                    dlh = hdg - lasthdg;

                    dth = hdg - output[OUTPUT_VARIABLE_HEADING].value;

                    if(dth > 180.0)
                    {
                        Serial.println("-360");
                        dth -= 360.0;
                    }

                    if(dth < -180.0)
                    {
                        Serial.println("+360");
                        dth += 360.0;
                    }

                    dps = abs(1000.0 * dlh) / avgdiff;
                    iacs = (abs(dth) / dps);

                    /*
                    Serial.print("\t\tDelta heading: ");
                    Serial.print(dlh);
                    Serial.print(" (=");
                    Serial.print(dps);
                    Serial.print(" dps) vs target hdg ");
                    Serial.print(output[OUTPUT_VARIABLE_HEADING].value);
                    Serial.print(" => ");
                    Serial.print(iacs);
                    Serial.print(" it ");
                    if(dth >= 0.0)
                    {
                        Serial.println("clockwise");
                    }
                    else
                    {
                        Serial.println("counter-clockwise");
                    }
                    */

                    if(abs(dth) < 2)
                    {
                        setOutput(OUTPUT_VARIABLE_HEADING, -1);
                        //Serial.println("Sequence empty, adding refresh step...");
                        step = {
                            0,                  // countdown
                            STATE_STOPPING,     // value
                            OUTPUT_STATE        // channel
                        };
                        pushSequence(&step);
                    }
                    else
                    {
                        if(
                            (dth >= 0.0 && output[OUTPUT_RELAY_DIRECTION].value != 0)
                            ||
                            (dth < 0.0 && output[OUTPUT_RELAY_DIRECTION].value != 1)
                        )
                        {
                            step = {
                                0,                  // countdown
                                STATE_REVERSING,    // value
                                OUTPUT_STATE        // channel
                            };
                            pushSequence(&step);
                        }
                    }
                }

                lasthdg = hdg;
                lastaverage = now;
            }

            break;
        }

        case STATE_STOPPING:
        {
            break;
        }

        case STATE_CORRECTING:
        {
            break;
        }

        case STATE_REVERSING:
        {
            break;
        }

        case STATE_ERROR:
        {
            break;
        }
    
    }
}

void doAlways()
{
    // Always read serial port (for any new commands)
    grabSerial();

    // Always parse any arrived messages
    parseCommand();

    stateMachine();
}

void loop()
{
    uint16_t input = analogRead(ANALOG_INPUT);
    uint16_t i = 0;
    static uint32_t last = 0;
 
    uint32_t now = millis();
    int diff = now - last;
    
    ether.packetLoop(ether.packetReceive());
    
    // State machine!
    
    doAlways();

    if(last == 0)
    {
        last = millis();
    }
    
    if(diff >= 1)
    {
        stepDance((uint16_t)diff);
        last = millis();
    }

    delayMicroseconds(50);
}