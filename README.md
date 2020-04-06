# arduino-antrot
Arduino Antenna Rotator

Control will be via Ethernet, eventually, but for a start I'll probably try using the Orbitron DDE to Serial tool and spit its control messages over to the Arduino. Or, write a DDE application that creates UDP messages and hand those over to the Arduino. We'll see.

I haven't decided yet if I want to keep separate coaxes for each antenna, yet, or if I'll use an inherited rotational coupling with dual coax feed-through and let the rotator spin freely. If I use separate coaxes, I'll also have to think about how to solve end-stops and counting revolutions and such.

I won't bother with encoder to determine the pointing direction, I'll try using a magnetometer to get the compass direction directly.

# Hardware
Current hardware consists of the largest ``pedal`` gear and the second smallest ``gear`` sprocket from a bicycle and some chain. I made adapters to fit things together by drilling and tapping some aluminum at the drill press. And filing. A lot of filing.

Prototype does rotate, with the chain almost never derailing.

I started out with an AC motor meant for a marquise, but it was too fast.

Then, I did one attempt with a stepper motor, but the NEMA 19 stepper motor I had at hand was too weak.

Then, the local hams dug out a couple of geared DC motors from the scrap pile; one with 1:162 reduction and one with 1:400. Both would be usable, but the 1:162 was getting into the ridiculous speed range at max voltage (24-61V rating), so I stuck with the 1:400 motor (ITT dunkermotoren GR 52.0). Motor itself does 1500-8000 rpm and draws up to 2.2A. Unloaded tests show it to spin down to at least 6V (at something like 150mA), which I'll use as the ``creep`` speed, since I have an Efore modular multi-voltage PSU with 6, 12 and 48V outputs. I'll leave the 12V output for control (stepping down to 5V for Arduino), and use the 48V for ``fast`` speed.

# Schematic and PCB
A few quick notes about PCB layout:

It's not made to be produced; it's only for my own notes to see how I could fit everything on a single 14x20 perf board. I did, however, not count well enough before I started soldering the damn thing, so as it is laid out in Fritzing it won't quite fit in the alotted space. Close, but no cigar. Some components have therefore been stacked and are sharing holes, etc. But it's done. If I need to remake it, I'll consider doing a better job of the layout and have it ordered instead.

I added the FET on the low side of the final relay stage in case I want to try to lower the speed further by PWM driving it.

# Arduino
## State machine
Suggested stated:89.10.136.125

* Idle: From startup. Does nothing except monitor input from computer.
    - Allowed transition: Enabling
        + Trigger: Move command
        + Trigger: Button press
    - Allowed transition: Error
        + Trigger: TBD
    - Allowed transition: Idle
        + Trigger: TBD

* Enabling: Step-state between idle and traversing to accomodate continuing to process data without interrupts. Have received a move command from the computer and will begin moving there.
    - Allowed transition: Stopping
        + Trigger: Stop command
        + Trigger: Button release
    - Allowed transition: Error
        + Trigger: TBD
    - Allowed transition: Traversing
        + Trigger: Reached end of process

* Traversing: Oscar Mike.
    - Allowed transition: Stopping
        + Trigger: Reached destination (or approaching at some rate we expect to take us there by the time we stop).
        + Trigger: Stop command
        + Trigger: Button release
    - Allowed transition: Error
        + Trigger: Not reading position change (or moving the wrong way)
        + Trigger: TBD

* Stopping: Step-state between traversing and idle. Have arrived at destination, or close enough.
    - Allowed transition: Idle
        + Trigger: Reached end of process
    - Allowed transition: Enabling
        + Trigger: Move command
        + Trigger: Button press

* Correcting: Idle-like state between Traversing and Traversing... the other way.
    - Allowed transition: Traversing
        + Trigger: Reached end of process
    - Allowed transition: Stopping
        + Trigger: 
* Error: Something's gone wrong. Eg. no movement detected from magnetometer after some time trying to move.


### All states
* Check command buffer
* Check for errors
* Check buttons (interrupt)

### Idle
* Transition to Enabling

### Enabling
* Do the switching of relays in proper sequence and with proper timing.
    - Transition to Traversing

### Traversing
Do in this state:
* Set PWM speed control at refresh interval.
* Check for nearness to target direction.
    - Adjust speed accordingly.
    - Transition to Stopping
    - Transition to Correcting

### Stopping
* Do the switching of relays in proper sequence and with proper timing.
    - Transition to Idle.

### Correcting
* Do the switching of relays in proper sequence and with proper timing for reversal of direction.
* Check for nearness to target direction.
    - Transition to Stopping
    - Transition to Correcting

### Error




# Commands
Suggested command format:
`$[cmd],[arg]<[CR]LF>`

The following commands will be useful in initial debugging:
* Stop (`D`isable)
* Speed (`S`peed)
* Direction (`R`etning)
* Start (`E`nable)

# Sequencing
I'm thinking that for the fancy I want a dynamic sequence mechanic so I can reuse the same timer and functions for setting all changes. So I'll define a sequence step type, and have an array capable of holding the expected maximum number of steps.

The expected maximum number of steps is, I guess, a soft reversal:

1) Lower speed (new PWM value)
2) Lower speed yet (new PWM value)
3) Dead stop ("Drive" relay disabled)
4) Reverse direction ("Direction" relay toggled)
5) Run ("Drive" relay enabled)
6) Higher speed (new PWM value)
7) Higher speed yet (new PWM value)

In addition, adding "creep" speed to the list, plus possibly setting of new target compass value (though not sure what need that'd cover). So in total, I think 10 elements will suffice for now.

Adding output types to the list means we can decide what kind of value we set. Eg. using -1 for a boolean (relay) to toggle its state, while also setting a PWM value of 10 bits resolution to the output transistor pin using the same data type.

So, in summary:
```
typedef enum tOutputType {
    OUTPUT_TYPE_RELAY = 0,
    OUTPUT_TYPE_PWM,
    OUTPUT_TYPE_VARIABLE,
    OUTPUT_TYPE_STATE
};

typedef enum tOutputChannel {
    OUTPUT_CHANNEL_TARGET_HEADING = -1,
    OUTPUT_CHANNEL_UNUSED = 0,
    OUTPUT_CHANNEL_MOTOR_A = MOTOR_OUT_A,
    OUTPUT_CHANNEL_MOTOR_B = MOTOR_OUT_B,
    OUTPUT_CHANNEL_PWM = MOTOR_OUT_A,
    OUTPUT_CHANNEL_DIRECTION = RELAY_OUT_C,
    OUTPUT_CHANNEL_VOLTAGE = RELAY_OUT_A,
    OUTPUT_CHANNEL_ENABLE = RELAY_OUT_B
};

typedef struct sequence_step {
    uint16_t countdown;
    int16_t value;
    tOutputChannel channel;
    tOutputType output;
} tSequenceStep;

typedef struct sequence {
    uint8_t items;
    tSequenceStep sequence[10];
} tSequence;

tSequence sequence = { 0 };
```

And then functions to peek, push and pop items on that sequence. The various states will peek at the first element of the sequence (if any) and read its countdown. If the countdown has a value, decrease it. If the countdown value is 0, execute the action in that sequence element, and pop the item from the sequence.

When changing states, a set of steps to be executed in that state are pushed to the sequence. The last item in the sequence should be to go to the defined state.

So, the whole sequence of going from idle, move to a position, then stopping would be something like:

* Push step
    - countdown = 0
    - value = 240
    - output = OUTPUT_TYPE_VARIABLE
    - channel = OUTPUT_CHANNEL_TARGET_HEADING
* Push step
    - countdown = 0
    - value = STATE_STARTING
    - output = OUTPUT_TYPE_STATE
    - channel = OUTPUT_CHANNEL_UNUSED

Setting STATE_STARTING:
* Push step
    - countdown = 0
    - value = 128
    - output = OUTPUT_TYPE_PWM
    - channel = OUTPUT_CHANNEL_PWM
* Push step
    - countdown = 0
    - value = VOLTAGE_48V
    - output = OUTPUT_TYPE_RELAY
    - channel = OUTPUT_CHANNEL_VOLTAGE
* Push step
    - countdown = 0
    - value = DIRECTION_CW
    - output = OUTPUT_TYPE_RELAY
    - channel = OUTPUT_CHANNEL_HEADING
* Push step
    - countdown = 50
    - value = DRIVE_ENABLE
    - output = OUTPUT_TYPE_RELAY
    - channel = OUTPUT_CHANNEL_DRIVE
* Push step
    - countdown = 100
    - value = 1024
    - output = OUTPUT_TYPE_PWM
    - channel = OUTPUT_CHANNEL_PWM
* Push step
    - countdown = 0
    - value = STATE_TRAVERSING
    - output = OUTPUT_TYPE_STATE
    - channel = OUTPUT_CHANNEL_UNUSED

Then, in STATE_TRAVERSING, the action performed is to keep observing the rate of change of magnetometer heading, and estimating how many iterations remain before changing speed. When it's low enough:
* Push step
    - countdown = 0
    - value = 512
    - output = OUTPUT_TYPE_PWM
    - channel = OUTPUT_CHANNEL_PWM

Repeat until close enough to target, ideally. Then:
* Push step
    - countdown = 0
    - value = STATE_STOPPING
    - output = OUTPUT_TYPE_STATE
    - channel = OUTPUT_CHANNEL_UNUSED

Setting STATE_STOPPING:
* Push step
    - countdown = 0
    - value = 0
    - output = OUTPUT_TYPE_PWM
    - channel = OUTPUT_CHANNEL_PWM
* Push step
    - countdown = 0
    - value = DRIVE_DISABLE
    - output = OUTPUT_TYPE_RELAY
    - channel = OUTPUT_CHANNEL_DRIVE
* Push step
    - countdown = 0
    - value = STATE_IDLE
    - output = OUTPUT_TYPE_STATE
    - channel = OUTPUT_CHANNEL_UNUSED

The steps added when setting STATE_STOPPING have no delays, so should occur in a tight loop.

*Et voila!*
