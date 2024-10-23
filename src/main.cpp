#include <Arduino.h>
#include <CapacitiveSensor.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 */

/**
 * @brief
 * 1. Check pot values for thresholds - 200ms loop
 * 2. Update display values - 200ms loop
 * 3. Check cap sensors - 50ms loop
 *    a. IF both high - check impedence every 50ms for 500ms - sending BOTH_HANDS while doing so
 *       i. IF impedence passes, send JOINED
 *          - Monitor impedence for max 3s for drop off, exit state if necessary
 *          - Fall back to cap check and repeat
 *    b. Send state at end of each 50ms loop
 * 4. Indicator LEDs update at each 50ms loop
 *
 * States:
 * OutputState  - current state for outputting
 * SensingState - current state for which sensing path we are checking
 *
 *
 * Functions:
 * updateThresholds()     - updates thresholds every 200ms
 * updateDisplay()        - updates display every 200ms
 * capacitiveCheck()      - checks cap sensors individually, updates state if necessary
 * impedenceCheck()       - checks impedence sensing circuit, updates state if necessary
 * updateSensingState()   - switches sensing state, updating the relay
 * updateOutputState()    - updates output state if necessary
 * sendOutputState()      - prints output state via serial
 * updateLEDs()           - updates LEDs
 */

#define CAP_SEND_PIN 7
#define CAP_RECEIVE_1 5
#define CAP_RECEIVE_2 9
#define IMP_CHECK A7
#define CAP_L_POT A0
#define CAP_R_POT A1
#define IMP_POT A2
#define RELAY_PIN_1 12
#define RELAY_PIN_2 11
#define CAP_L_LED 2
#define CAP_R_LED 3
#define IMP_LED 4

CapacitiveSensor CS_1 = CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_1); // 10M resistor between pins 4 & 8, pin 8 is sensor pin, add a wire and or foil
CapacitiveSensor CS_2 = CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_2); // 10M resistor between pins 4 & 6, pin 6 is sensor pin, add a wire and or foil

// Variables
long cs1Value = 0;
long cs2Value = 0;
bool capSw1Active = false;
bool capSw2Active = false;
bool bothActive = false;
int impedence = 0;
unsigned long curMillis = 0;
unsigned long prevThresholdUpdateMillis = 0;
unsigned long prevDisplayUpdateMillis = 0;
unsigned long prevStateUpdateMillis = 0;
unsigned long prevThresholdUpdateMillis = 0;
unsigned long curImpCheckMillis = 0;
unsigned long prevImpCheckMillis = 0;

// State handling
enum OutputState
{
  IDLE,
  LEFT,
  RIGHT,
  BOTH,
  JOINED
};

enum SensingState
{
  CAPACITIVE,
  IMPEDENCE
};

OutputState curOutputState = IDLE;
SensingState curSensingState = CAPACITIVE;

// Function Declarations
void updateThresholds();   // - updates thresholds every 200ms
void updateDisplay();      // - updates display every 200ms
void capacitiveCheck();    // - checks cap sensors individually, updates state if necessary
void impedenceCheck();     // - checks impedence sensing circuit, updates state if necessary
void updateSensingState(); // - switches sensing state, updating the relay
void updateOutputState();  // - updates output state if necessary
void sendOutputState();    // - prints output state via serial
void updateLEDs();         // - Updates indicator LEDs

void setup()
{
  Serial.begin(9600);
  // CS_1.set_CS_AutocaL_Millis(0xFFFFFFFF); // turn off autocalibrate on channel 1 - just as an example
  // CS_1.set_CS_AutocaL_Millis(0xFFFFFFFF);
  // CS_2.set_CS_AutocaL_Millis(0xFFFFFFFF);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(CAP_L_LED, OUTPUT);
  pinMode(CAP_R_LED, OUTPUT);
  pinMode(IMP_LED, OUTPUT);
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(CAP_L_LED, LOW);
  digitalWrite(CAP_R_LED, LOW);
  digitalWrite(IMP_LED, LOW);
  Serial.println('Magic Hands setup complete');
}

void loop()
{
  curMillis = millis();
  prevMillis = curMillis;
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  while (curMillis - prevMillis < 8000)
  {
    curMillis = millis();
    checkCapSwitches();
  }

  prevMillis = curMillis;
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  while (curMillis - prevMillis < 8000)
  {
    curMillis = millis();
    checkImpedence();
  }
}

void checkCapSwitches()
{
  //   cs1Value = CS_1.capacitiveSensor(30);
  //   cs2Value = CS_2.capacitiveSensor(30);
  cs1Value = CS_1.capacitiveSensorRaw(50);
  cs2Value = CS_2.capacitiveSensorRaw(50);
  Serial.print("CS 1 Val: "); // check on performance in milliseconds
  Serial.print(cs1Value);     // print sensor output 2
  Serial.print("\t|\t");
  Serial.print("CS 2 Val: ");
  Serial.println(cs2Value);
  delay(100);
}

void checkImpedence()
{
  impedence = analogRead(IMP_CHECK);
  Serial.print("Impedence: ");
  Serial.println(impedence);
  delay(100);
}
