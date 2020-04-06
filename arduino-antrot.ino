#include <stdint.h>
#include <EtherCard.h> // for ENC28J60 module
#include <IPAddress.h>

static byte mymac[] = { 0x00, 0x90, 0x62, 0xF1, 0xFA, 0xF0 };
byte Ethernet::buffer[384];
const uint16_t srcPort PROGMEM = 31550;
const uint16_t dstPort PROGMEM = 31560;

#define COMMAND_LINE_MAX_COMMAND_LENGTH         8 // Currently something like "#G,360;"
#define COMMAND_LINE_MIN_COMMAND_LENGTH         3 // Currently something like "#P;"
#define COMMAND_LINE_SERIAL_BUFFER_SIZE         32
#define COMMAND_LINE_BUFFER_SIZE                128
#define COMMAND_LINE_APPEND_THRESHOLD           96 // ((COMMAND_LINE_BUFFER_SIZE / 4) * 3)

char serbuf[COMMAND_LINE_SERIAL_BUFFER_SIZE] = { 0 };
uint8_t serlength = 0;

char cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };
uint8_t cmdlength = 0;

#define MAX_SEQUENCE_LENGTH     10

enum outputPins {
    MOTOR_OUT_A         = 5,
    MOTOR_OUT_B         = 6,
    ANALOG_INPUT        = A0,
    DIGITAL_INPUT_A     = 2,   // Interrupt pin
    DIGITAL_INPUT_B     = 3,   // Interrupt pin
    RELAY_OUT_A         = 7,
    RELAY_OUT_B         = 8,
    RELAY_OUT_C         = 9,
    ETHER_CS            = 10,
    RELAY_SPEED         = RELAY_OUT_A,
    RELAY_DIRECTION     = RELAY_OUT_B,
    RELAY_DRIVE         = RELAY_OUT_C
};

typedef enum OutputType {
    OUTPUT_TYPE_RELAY = 0,
    OUTPUT_TYPE_PWM,
    OUTPUT_TYPE_VARIABLE,
    OUTPUT_TYPE_STATE
} tOutputType;

typedef enum OutputChannel {
    OUTPUT_CHANNEL_TARGET_HEADING = -1,
    OUTPUT_CHANNEL_UNUSED = 0,
    OUTPUT_CHANNEL_MOTOR_A = MOTOR_OUT_A,
    OUTPUT_CHANNEL_MOTOR_B = MOTOR_OUT_B,
    OUTPUT_CHANNEL_PWM = MOTOR_OUT_A,
    OUTPUT_CHANNEL_DIRECTION = RELAY_OUT_C,
    OUTPUT_CHANNEL_VOLTAGE = RELAY_OUT_A,
    OUTPUT_CHANNEL_ENABLE = RELAY_OUT_B
} tOutputChannel;

typedef struct sequence_step {
    uint16_t countdown;
    int16_t value;
    tOutputChannel channel;
    tOutputType output;
} tSequenceStep;

typedef struct sequence {
    uint8_t items;
    tSequenceStep sequence[MAX_SEQUENCE_LENGTH];
} tSequence;

tSequence sequence;

// Need to be PWM pins for speed control
#define MOTOR_PWM           MOTOR_OUT_A

#define CLOCKWISE           0
#define COUNTERCLOCKWISE    1

#define STOP                0
#define RUN                 1

#define SPEED_CREEP         0
#define SPEED_FAST          1

uint8_t direction = CLOCKWISE;
uint8_t run = STOP;
uint32_t lastinterrupt = 0;

uint8_t stateMachineState = 0;

#define STATE_IDLE                      0
#define STATE_STARTING                  1
#define STATE_TRAVERSING                2
#define STATE_STOPPING                  3
#define STATE_CORRECTING                4
#define STATE_ERROR                     5
#define STATE_MAX_STATE                 STATE_ERROR



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
    Serial.print("Items in sequnce: ");
    Serial.println(sequence.items);
    
    for(i = 0; i < MAX_SEQUENCE_LENGTH; i++)
    {
        Serial.print("- Item ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sequence.sequence[i].countdown);
        Serial.print(", ");
        Serial.print(sequence.sequence[i].value);
        Serial.print(", ");
        Serial.print(sequence.sequence[i].channel);
        Serial.print(", ");
        Serial.println(sequence.sequence[i].output);
    }
}

int8_t pushSequence(tSequenceStep* src_step)
{
    if(sequence.items >= MAX_SEQUENCE_LENGTH)
    {
        Serial.println("Failed to push sequence step!");
        return -1;
    }

    sequence.sequence[sequence.items].countdown = src_step->countdown;
    sequence.sequence[sequence.items].value     = src_step->value;
    sequence.sequence[sequence.items].channel   = src_step->channel;
    sequence.sequence[sequence.items].output    = src_step->output;

    sequence.items++;

    return (int8_t)sequence.items;
}

// Pop first item from queue.
// Returns number of items left in queue, or -1 if queue already was empty.
// If argument is not NULL, first item is copied to pointer address before sequence shift.
// Rest of sequence is shifted one step, empty steps are additionally cleared.
int8_t popSequence(tSequenceStep* dst_step)
{
    int i = 0;

    if(sequence.items == 0)
    {
        Serial.println("Sequence already empty!");
        clearSequence();
        return -1;
    }

    if(dst_step != NULL)
    {
        dst_step.countdown  = sequence.sequence[0].countdown;
        dst_step.value      = sequence.sequence[0].value;
        dst_step.channel    = sequence.sequence[0].channel;
        dst_step.output     = sequence.sequence[0].output;
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
        sequence.sequence[i].output     = sequence.sequence[i+1].output;
    }

    // Clear remaining positions:
    for(;i < MAX_SEQUENCE_LENGTH; i++)
    {
        sequence.sequence[i].countdown  = 0;
        sequence.sequence[i].value      = 0;
        sequence.sequence[i].channel    = 0;
        sequence.sequence[i].output     = 0;
    }

    return (int8_t)sequence.items;

}

int8_t replaceSequence(tSequenceStep* new_step)
{
    return 0;
}

int8_t peekStep(tSequenceStep* dst_step, uint8_t src_step)
{
    if(src_step > sequence.items)
    {
        return -1;
    }

    if(dst_step != NULL)
    {
        dst_step.countdown = sequence.sequence[src_step]->countdown;
        dst_step.value     = sequence.sequence[src_step]->value;
        dst_step.channel   = sequence.sequence[src_step]->channel;
        dst_step.output    = sequence.sequence[src_step]->output;
    }

    return (int8_t)src_step;
}

/*
void sendMsg((char*)src, uint16_t len)
{
    ether.sendUdp(src, len, srcPort, <ip>, dstPort );
}
*/

void setState(uint8_t newState)
{
    uint8_t set = STATE_IDLE;
    
    if(newState <= STATE_MAX_STATE)
    {
        set = newState;
    }

    Serial.print(millis());
    Serial.print(": S");
    Serial.print(stateMachineState);
    Serial.print(" -> S");
    Serial.println(set);

    stateMachineState = set;
    
    switch(stateMachineState)
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
        case STATE_ERROR:
        {
            break;
        }
    }
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

    Serial.print("There's ");
    Serial.print(cmdlen);
    Serial.println(" bytes to be read.");

    avail = COMMAND_LINE_BUFFER_SIZE - cmdlength;

    Serial.print("Can write ");
    Serial.print(avail);
    Serial.println(" bytes to buffer.");

    if(avail <= 0)
    {
        return;
    }
    
    if(cmdlen > avail)
    {
        cmdlen = avail;
    }

    Serial.print("Grabbing ");
    Serial.print(cmdlen);
    Serial.println(" bytes from serial port.");


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
    uint8_t begin = cmdlength;
    uint8_t end = 0;
    uint8_t cmdlen = 0;

    if(cmdlength == 0)
    {
        // Do nothing if the buffer is empty
        return 0;
    }
    
    Serial.print("Command string: [");
    Serial.flush();
    for(j = 0; j < COMMAND_LINE_BUFFER_SIZE; j++)
    {
        Serial.print(cmdline[j] > 20 ? cmdline[j] : '.');
        Serial.flush();
    }
    Serial.println("]");
    Serial.flush();

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

    //Serial.println("A");
    
    if(begin > 0)
    {
        // Command start character is not the first in buffer; clean out crap
        stripCommand(begin);
        return begin;
    }

    //Serial.println("B");
    
    if(begin > end)
    {
        // There's garbage.
        stripCommand(begin);
        return begin;
    }

    cmdlen = (end - begin) + 1;

    //Serial.println("C");
    
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
                Serial.println(stateMachineState);
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

    pinMode(MOTOR_OUT_A,        OUTPUT);
    pinMode(MOTOR_OUT_B,        OUTPUT);
    pinMode(RELAY_SPEED,        OUTPUT);
    pinMode(RELAY_DIRECTION,    OUTPUT);
    pinMode(RELAY_DRIVE,       OUTPUT);
    pinMode(DIGITAL_INPUT_A,    INPUT_PULLUP);
    pinMode(DIGITAL_INPUT_B,    INPUT_PULLUP);
    pinMode(ANALOG_INPUT,       INPUT);

    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_A), directionChange, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_B), toggleRun, FALLING);
    
    delay(100);

    // Alright, nobody move!
    digitalWrite(RELAY_SPEED, LOW);
    digitalWrite(RELAY_DIRECTION, LOW);
    digitalWrite(RELAY_DRIVE, LOW);

    delay(100);

    // Change 'SS' to your Slave Select pin, if you arn't using the default pin
    if (!ether.begin(sizeof Ethernet::buffer, mymac, ETHER_CS))
    {
        Serial.println( "Failed to access Ethernet controller!");
        while(!ether.begin(sizeof Ethernet::buffer, mymac, ETHER_CS))
        {
            delay(100);
        }
    }

    if (!ether.dhcpSetup())
    {
        Serial.println("DHCP failed");
    }

    ether.printIp("IP:  ", ether.myip);
    ether.printIp("GW:  ", ether.gwip);
    ether.printIp("DNS: ", ether.dnsip);

    Serial.println("Starting listener...");
    ether.udpServerListenOnPort(&netRecv, srcPort);

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

void doAlways()
{
    // Always read serial port (for any new commands)
    grabSerial();

    // Always parse any arrived messages
    parseCommand();
}

void loop()
{
    uint16_t input = analogRead(ANALOG_INPUT);
    uint16_t i = 0;
    static int last = 0;
    int now = millis();
    ether.packetLoop(ether.packetReceive());
    // State machine!
    
    doAlways();

    if(last == 0)
    {
        last = millis();
    }
    
    if(false /* now-last > 5000 */)
    {
        Serial.print("Command string: [");
        Serial.flush();
        for(i = 0; i < COMMAND_LINE_BUFFER_SIZE; i++)
        {
            Serial.print(cmdline[i] > 20 ? cmdline[i] : '.');
            Serial.flush();
        }
        Serial.println("]");
        Serial.flush();
        
        last = millis();
    }

    delayMicroseconds(50);
}