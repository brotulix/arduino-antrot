# arduino-antrot
Arduino Antenna Rotator


# Thoughts on using stepper motor (in Norwegian)
Hastigheten skaleres med utslag fra midtpunkt på potmeter (0-511, 513-1023).
Finn maksimalt antall RPM obtainable med stepperen med og uten last og med og uten tannhjul.
Dette krever sikkert flere iterasjoner med økende MAX_RPM som potmeter-utslaget normaliseres til.

På lave hastigheter blir antallet steps per iterasjon alltid 0, slik at man får en unaturlig vid dødsone rundt midten. Dette kan løses med å akkumulere opp fraksjoner av steps og utføre step når dette blir over 1.

For alle tilfeller gjelder 1.8 grader per step, altså 200 steps per rotasjon.
For alle tilfeller gjelder et loop-intervall på 10ms, eller 1/100 sekund.

For eksempel med MAX_RPM = 10:
Dette gir maksimalt 10 rotasjoner/minutt * 200 step/rotasjon = 2000 steps/minutt.
Altså maksimalt 2000 step/s * 1/100 s/iterasjon  = 20 step/iterasjon.
Første iterasjon leses A0 av, verdi 837. 837-512=325 => 325/511 = 0.636.
Dette gir skalert RPM inneværende iterasjo på 1 + ((MAX_RPM-1) * 0.636) = 6.724 RPM.
6.724 rotasjoner/minutt * 200 step/rotasjon * 1/100 s/iterasjon * 1/60 minutt/s
= 0.22413 step/iterasjon, altså ingen step, bare en rest på 0.22413 step til neste iterasjon.

Dvs:
```
static float remainder = 0.0;
loop() {
    uint16_t val = analogRead(A0); // = 837
    float scaledval = ((float)val - 512.0) / 511.0; // = 837-512 / 511 = 325/511 = 0.636
    float scaledrpm = 1 + (((float)MAX_RPM - 1.0) * scaledval); // 1 + ((10 - 1) * 0.6) = 1 + 5.724 = 6.724
    uint16_t setrpm = (uint16_t)scaledrpm; // = floor(6.724) = 6
    scaledsteps = remainder + setrpm * 200 * 1.0/100 * 1/60; // = 0.2
    uint16_t steps = (uint16_t)scaledsteps; // = 0
    remainder = scaledsteps - steps; // 0.2
    stepper.setSpeed(setrpm);
    stepper.step(steps);
}
```