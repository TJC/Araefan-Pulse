#include <Wire.h>
#include <SI114.h>

/*
For Arduino users, use the SDA and SCL pins on your controller. (A4 and A5 on Nano)
For Teensy 3.x/LC users, likewise.
Typically pin 18 is SCL, and 19 is SDA.

The original docs here said to use 10k resisters in series, but that seems confused.
I note that you definitely need 5k pull-up resistors going to a 3V3 source. These
are on the SDA and SCL lines.

*/


const int SAMPLES_TO_AVERAGE = 5;             // samples for smoothing 1 to 10 seem useful 5 is default
                                              // increase for smoother waveform (with less resolution - slower!) 


// #define SERIAL_OUTPUT


int binOut;     // 1 or 0 depending on state of heartbeat
int BPM;
unsigned long red;        // read value from visible red LED
unsigned long IR1;        // read value from infrared LED1
unsigned long IR2;       // read value from infrared LED2
unsigned long IR_total;     // IR LED reads added together
unsigned long led_tick = 0;

PulsePlug pulse;

void logmsg(char *msg) {
  #ifdef SERIAL_OUTPUT
  Serial.println(msg);
  #endif
}

void blink() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

inline void led_beat_on() {
  digitalWrite(13, HIGH);
  led_tick = millis() + 250; // this is setting the time when the LED should turn off again
}

inline void led_beat_timeout() {
  if (led_tick > 0 && led_tick <= millis()) {
    digitalWrite(13, LOW);
    led_tick = 0;
  }
}

void setup() {
  pinMode(13, OUTPUT); // for blinking LED on-board

  #ifdef SERIAL_OUTPUT
  Serial.begin(57600);
  #endif

  // kill time to wait for Serial monitor to open.
  for (int i = 0; i < 3; i++) {
    logmsg("Pulse monitor");
    blink();
  }

  if (pulse.isPresent()) {
    logmsg("SI114x Pulse Sensor found");
    #ifdef SERIAL_OUTPUT
    pulse.id();
    #endif
    delay(1000);
  }
  else {
    while (1) {
      logmsg("No SI114x found");
      delay(1000);
    }
  }

  pulse.initSensor();
}


void loop() {
  led_beat_timeout();
  readPulseSensor();
}



// simple smoothing function for  heartbeat detection and processing
float smooth(float data, float filterVal, float smoothedVal) {

  if (filterVal > 1) {      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0.0) {
    filterVal = 0.01;
  }

  smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);
  return smoothedVal;
}


void readPulseSensor() {

  static int foundNewFinger, red_signalSize, red_smoothValley;
  static long red_valley, red_Peak, red_smoothRedPeak, red_smoothRedValley,
    red_HFoutput, red_smoothPeak; // for PSO2 calc
  static  int IR_valley = 0, IR_peak = 0, IR_smoothPeak, IR_smoothValley, binOut, lastBinOut, BPM;
  static unsigned long lastTotal, lastMillis, IRtotal, valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime = millis(), lastBeat, beat;
  static float IR_baseline, red_baseline, IR_HFoutput, IR_HFoutput2, shiftedOutput, LFoutput, hysterisis;

  unsigned long total = 0, start;
  int i = 0;
  int IR_signalSize;
  red = 0;
  IR1 = 0;
  IR2 = 0;
  total = 0;
  start = millis();


  while (i < SAMPLES_TO_AVERAGE) {
    uint16_t* ledValues = pulse.fetchLedData();

    red += ledValues[0];
    IR1 += ledValues[1];
    IR2 += ledValues[2];
    i++;
  }

  red = red / i;  // get averages
  IR1 = IR1 / i;
  IR2 = IR2 / i;
  total = IR1 + IR2 + red;  // red excluded
  IRtotal = IR1 + IR2;


  if (lastTotal < 20000L && total > 20000L) foundNewFinger = 1;  // found new finger!

  lastTotal = total;

  // if found a new finger prime filters first 20 times through the loop
  if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover 

  if (foundNewFinger < 20) {
    IR_baseline = total - 200;   // take a guess at the baseline to prime smooth filter
    logmsg("found new finger");
  }

  else if (total > 20000L) {    // main running function


                                // baseline is the moving average of the signal - the middle of the waveform
                                // the idea here is to keep track of a high frequency signal, HFoutput and a 
                                // low frequency signal, LFoutput
                                // The LF signal is shifted downward slightly downward (heartbeats are negative peaks)
                                // The high freq signal has some hysterisis added. 
                                // When the HF signal crosses the shifted LF signal (on a downward slope), 
                                // we have found a heartbeat.
    IR_baseline = smooth(IRtotal, 0.99, IR_baseline);   // 
    IR_HFoutput = smooth((IRtotal - IR_baseline), 0.2, IR_HFoutput);    // recycling output - filter to slow down response

    red_baseline = smooth(red, 0.99, red_baseline);
    red_HFoutput = smooth((red - red_HFoutput), 0.2, red_HFoutput);

    // beat detection is performed only on the IR channel so 
    // fewer red variables are needed

    IR_HFoutput2 = IR_HFoutput + hysterisis;
    LFoutput = smooth((IRtotal - IR_baseline), 0.95, LFoutput);
    // heartbeat signal is inverted - we are looking for negative peaks
    shiftedOutput = LFoutput - (IR_signalSize * .05);

    if (IR_HFoutput  > IR_peak) IR_peak = IR_HFoutput;
    if (red_HFoutput  > red_Peak) red_Peak = red_HFoutput;

    // default reset - only if reset fails to occur for 1800 ms
    if (millis() - lastPeakTime > 1800) {  // reset peak detector slower than lowest human HB
      IR_smoothPeak = smooth((float)IR_peak, 0.6, (float)IR_smoothPeak);  // smooth peaks
      IR_peak = 0;

      red_smoothPeak = smooth((float)red_Peak, 0.6, (float)red_smoothPeak);  // smooth peaks
      red_Peak = 0;

      lastPeakTime = millis();
    }

    if (IR_HFoutput  < IR_valley)   IR_valley = IR_HFoutput;
    if (red_HFoutput  < red_valley)   red_valley = red_HFoutput;



    if (millis() - lastValleyTime > 1800) {  // insure reset slower than lowest human HB
      IR_smoothValley = smooth((float)IR_valley, 0.6, (float)IR_smoothValley);  // smooth valleys
      IR_valley = 0;
      lastValleyTime = millis();
    }

    //     IR_signalSize = IR_smoothPeak - IR_smoothValley;  // this the size of the smoothed HF heartbeat signal
    hysterisis = constrain((IR_signalSize / 15), 35, 120);  // you might want to divide by smaller number
                                                            // if you start getting "double bumps"

                                                            // Serial.print(" T  ");
                                                            // Serial.print(IR_signalSize); 

    if (IR_HFoutput2 < shiftedOutput) {
      // found a beat - pulses are valleys
      lastBinOut = binOut;
      binOut = 1;
      hysterisis = -hysterisis;
      IR_smoothValley = smooth((float)IR_valley, 0.99, (float)IR_smoothValley);  // smooth valleys
      IR_signalSize = IR_smoothPeak - IR_smoothValley;
      IR_valley = 0x7FFF;

      red_smoothValley = smooth((float)red_valley, 0.99, (float)red_smoothValley);  // smooth valleys
      red_signalSize = red_smoothPeak - red_smoothValley;
      red_valley = 0x7FFF;

      lastValleyTime = millis();

    }
    else {
      lastBinOut = binOut;
      binOut = 0;
      IR_smoothPeak = smooth((float)IR_peak, 0.99, (float)IR_smoothPeak);  // smooth peaks
      IR_peak = 0;

      red_smoothPeak = smooth((float)red_Peak, 0.99, (float)red_smoothPeak);  // smooth peaks
      red_Peak = 0;
      lastPeakTime = millis();
    }

    if (lastBinOut == 1 && binOut == 0) {
      logmsg("off");
      usbMIDI.sendNoteOff(60, 99, 1);
    }

    if (lastBinOut == 0 && binOut == 1) {
      lastBeat = beat;
      beat = millis();
      BPM = 60000 / (beat - lastBeat);
      logmsg("on");
      /*
      Serial.print("\t BPM ");
      Serial.print(BPM);
      Serial.print("\t IR ");
      Serial.print(IR_signalSize);
      Serial.print("\t PSO2 ");
      Serial.println(((float)red_baseline / (float)(IR_baseline / 2)), 3);
      */
      usbMIDI.sendNoteOn(60, 99, 1);
      led_beat_on();
    }

  }

  // MIDI Controllers should discard incoming MIDI messages.
  while (usbMIDI.read()) {
  }
}



