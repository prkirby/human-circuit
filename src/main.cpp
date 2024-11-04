#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

/**
 * General Overview:
 *
 * 1. Check pot values for thresholds - intervaled loop
 * 2. Update display values - when outputs change
 * 3. Check cap sensors - 50ms loop
 *    a. IF both high - check impedence every sensor cycle for defined interval - sending BOTH_HANDS while doing so
 *       i. IF impedence passes, send JOINED
 *          - Fall back to cap check if no longer joined, and repeat
 *    b. Send state at end of each 50ms loop
 * 4. Indicator LEDs update when ouput state changes
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
#define CAP_RECEIVE_L 5
#define CAP_RECEIVE_R 9
#define IMP_CHECK A7
#define CAP_L_POT A0
#define CAP_R_POT A1
#define IMP_POT A2
#define RELAY_PIN_1 12
#define RELAY_PIN_2 11
#define CAP_L_LED 4
#define CAP_R_LED 3
#define IMP_LED 2

CapacitiveSensor CapSensorL = CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_L);
CapacitiveSensor CapSensorR = CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_R);

// Sensor Variables
const int minCapThreshold = 0;
const int maxCapThreshold = 15000;
int curCapLeftThreshold = maxCapThreshold;
int curCapRightThreshold = maxCapThreshold;
int curImpThreshold = 0;
int capLeftValue = 0;
int capRightValue = 0;
int impedenceValue = 0;
bool capLeftActive = false;
bool capRightActive = false;

// Threshold Buffers
const int ThresholdBufferSize = 20;
int capLeftThresholdBuffer[ThresholdBufferSize];
int capRightThresholdBuffer[ThresholdBufferSize];
int impThresholdBuffer[ThresholdBufferSize]; // the readings from the analog input
int thresholdBufferIndex = 0;                // the index of the current reading

// Timing Variables (in Milliseconds)
const int SensorCheckInterval = 50;
const int ThresholdUpdateInterval = 25;
const int ImpCheckBufferInterval = 500;
const int CapCheckBufferInterval = 500;
const int DisplayUpdateInterval = 500;
unsigned long curMillis = 0;
unsigned long prevSensorCheckMillis = 0;
unsigned long prevThresholdUpdateMillis = 0;
unsigned long prevCapCheckBufferMillis = 0;
unsigned long prevImpCheckBufferMillis = 0;
unsigned long prevThresholdDisplayMillis = 0;
unsigned long prevValueDisplayMillis = 0;
unsigned long prevActiveDisplayMillis = 0;

// Display string variables
char labelDisplayString[20];
char thresholdDisplayString[20];
char valueDisplayString[20];
char activeDisplayString[20];

// State Variables
enum OutputState
{
  OUTPUT_INIT,
  IDLE,
  LEFT,
  RIGHT,
  BOTH,
  JOINED
};

enum SensingState
{
  SENSING_INIT,
  CAPACITIVE,
  IMPEDENCE
};

OutputState curOutputState = OUTPUT_INIT;
SensingState curSensingState = SENSING_INIT;

// Function Declarations
void updateThresholds();                      // - updates thresholds every 200ms
void updateThresholdDisplay();                // - updates threshold display when changing
int bufferedThresholdRead(int[], pin_size_t); // - buffers threshold readings to make them more consistent with pots
void checkSensors();                          // - checks the appropraite sensors and handles timing buffers
void capacitiveCheck();                       // - checks cap sensors individually, updates state if necessary
void impedenceCheck();                        // - checks impedence sensing circuit, updates state if necessary
void updateValueDisplay();                    // - updates value display when changing
void updateSensingState(SensingState);        // - switches sensing state, updating the relay
void updateOutputState(OutputState);          // - updates output state if necessary
void sendOutputState();                       // - prints output state via serial
void updateLEDs();                            // - Updates indicator LEDs
void updateActiveDisplay();                   // - updates active sensors when changing

void setup()
{
  Serial.begin(9600);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(CAP_L_LED, OUTPUT);
  pinMode(CAP_R_LED, OUTPUT);
  pinMode(IMP_LED, OUTPUT);
  updateSensingState(CAPACITIVE);
  updateOutputState(IDLE);

  for (int i = 0; i < ThresholdBufferSize; i++)
  {
    capLeftThresholdBuffer[i] = 0;
    capRightThresholdBuffer[i] = 0;
    impThresholdBuffer[i] = 0;
  }

  lcd.init();
  lcd.backlight();
  sprintf(labelDisplayString, "LEFT | RGHT | JOIN");
  lcd.setCursor(0, 0);
  lcd.print(labelDisplayString);

  curMillis = millis();
  prevThresholdUpdateMillis = curMillis;
}

void loop()
{

  // Update current millis to be used across all function calls for main loop
  curMillis = millis();

  if (curMillis - prevThresholdUpdateMillis > ThresholdUpdateInterval)
  {
    prevThresholdUpdateMillis = curMillis;
    updateThresholds();
  }

  if (curMillis - prevSensorCheckMillis > SensorCheckInterval)
  {
    prevSensorCheckMillis = curMillis;
    checkSensors();
  }
}

/**
 * @brief Updates thresholds every ThresholdUpdateInterval milliseconds
 *
 */
void updateThresholds()
{
  curCapLeftThreshold = map(bufferedThresholdRead(capLeftThresholdBuffer, CAP_L_POT), 0, 1023, minCapThreshold, maxCapThreshold);
  curCapRightThreshold = map(bufferedThresholdRead(capRightThresholdBuffer, CAP_R_POT), 0, 1023, minCapThreshold, maxCapThreshold);
  curImpThreshold = bufferedThresholdRead(impThresholdBuffer, IMP_POT);

  // Increase buffer
  thresholdBufferIndex++;
  if (thresholdBufferIndex >= ThresholdBufferSize)
  {
    thresholdBufferIndex = 0;
  }

  updateThresholdDisplay();
  return;
}

/**
 * @brief
 *
 */
int bufferedThresholdRead(int buffer[], pin_size_t PIN)
{
  buffer[thresholdBufferIndex] = analogRead(PIN);
  int total = 0;
  for (int i = 0; i < ThresholdBufferSize; i++)
  {
    total += buffer[i];
  }

  return total / ThresholdBufferSize;
}

/**
 * @brief
 *
 */
void updateThresholdDisplay()
{
  if (curMillis - prevThresholdDisplayMillis < DisplayUpdateInterval)
    return;

  prevThresholdDisplayMillis = curMillis;
  sprintf(thresholdDisplayString, "%05u| %05u| %04u", curCapLeftThreshold, curCapRightThreshold, curImpThreshold);
  lcd.setCursor(0, 1);
  lcd.print(thresholdDisplayString);
  return;
}

/**
 * @brief
 *
 * @param newSensingState
 */
void updateSensingState(SensingState newSensingState)
{
  // Only run update if new state
  if (curSensingState != newSensingState)
  {
    curSensingState = newSensingState;

    switch (curSensingState)
    {
    case CAPACITIVE:
      digitalWrite(RELAY_PIN_1, HIGH);
      digitalWrite(RELAY_PIN_2, HIGH);

      // Update Cap buffer millis to prevent constant, quick switching
      prevCapCheckBufferMillis = curMillis;
      break;

    case IMPEDENCE:
      digitalWrite(RELAY_PIN_1, LOW);
      digitalWrite(RELAY_PIN_2, LOW);

      // Update Impedence buffer millis to allow time for impedence check to stabalize
      prevImpCheckBufferMillis = curMillis;
      break;

    // Default to capacitive checking
    default:
      digitalWrite(RELAY_PIN_1, HIGH);
      digitalWrite(RELAY_PIN_2, HIGH);
      break;
    }
  }
}

/**
 * @brief
 *
 */
void checkSensors()
{
  switch (curSensingState)
  {
  case CAPACITIVE:
    capacitiveCheck();
    break;

  case IMPEDENCE:
    impedenceCheck();
    break;
  }

  sendOutputState(); // Send output state after every sensor check
  updateValueDisplay();
}

/**
 * @brief
 *
 */
const int CapSensorSamples = 100;
void capacitiveCheck()
{
  capLeftValue = CapSensorL.capacitiveSensorRaw(CapSensorSamples);
  capRightValue = CapSensorR.capacitiveSensorRaw(CapSensorSamples);
  capLeftActive = capLeftValue > curCapLeftThreshold;
  capRightActive = capRightValue > curCapRightThreshold;

  if (capLeftActive && !capRightActive)
  {
    updateOutputState(LEFT);
    return;
  }

  if (!capLeftActive && capRightActive)
  {
    updateOutputState(RIGHT);
    return;
  }

  if (capLeftActive && capRightActive)
  {
    updateOutputState(BOTH);

    // Only switch to Impedence sensing if buffer interval has worn off, to prevent constant switching when two separate people touch the hands
    if (curMillis - prevCapCheckBufferMillis > CapCheckBufferInterval)
    {
      updateSensingState(IMPEDENCE);
    }

    return;
  }

  updateOutputState(IDLE);
  return;
}

/**
 * @brief
 *
 */
void impedenceCheck()
{
  impedenceValue = analogRead(IMP_CHECK);

  // Update and keep checking impedence while value is still above threshold
  if (impedenceValue < curImpThreshold)
  {
    updateOutputState(JOINED);
    prevImpCheckBufferMillis = curMillis;
    return;
  }

  // Otherwise, continue checking impedence if we haven't triggered while buffer interval is still active, to allow for stabalizing of the signal
  if (curOutputState != JOINED && curMillis - prevImpCheckBufferMillis < ImpCheckBufferInterval)
  {
    return;
  }

  // Finally, return to cap sensing if buffer has expired without triggering
  updateSensingState(CAPACITIVE);
  return;
}

/**
 * @brief
 *
 */
void updateValueDisplay()
{
  if (curMillis - prevValueDisplayMillis < DisplayUpdateInterval)
    return;

  prevValueDisplayMillis = curMillis;

  switch (curSensingState)
  {
  case CAPACITIVE:
    sprintf(valueDisplayString, "%05u| %05u| %4s", capLeftValue, capRightValue, " NA ");
    break;
  case IMPEDENCE:
    sprintf(valueDisplayString, "%4s | %4s | %04u", " NA ", " NA ", impedenceValue);
    break;
  }

  lcd.setCursor(0, 2);
  lcd.print(valueDisplayString);
  return;
}

/**
 * @brief
 *
 * @param newOutputState
 */
void updateOutputState(OutputState newOutputState)
{
  if (curOutputState != newOutputState)
  {
    curOutputState = newOutputState;
    updateLEDs(); // Only update LEDs when a new state is detected
    updateActiveDisplay();
  }
}

/**
 * @brief
 *
 */
void sendOutputState()
{
  switch (curOutputState)
  {
  case LEFT:
    Serial.println("[100]");
    break;

  case RIGHT:
    Serial.println("[010]");
    break;

  case BOTH:
    Serial.println("[110]");
    break;

  case JOINED:
    Serial.println("[001]");
    break;

  default:
    Serial.println("[000]");
    break;
  }
}

/**
 * @brief
 *
 */
void updateLEDs()
{
  switch (curOutputState)
  {
  case LEFT:
    digitalWrite(CAP_L_LED, HIGH);
    digitalWrite(CAP_R_LED, LOW);
    digitalWrite(IMP_LED, LOW);
    break;

  case RIGHT:
    digitalWrite(CAP_L_LED, LOW);
    digitalWrite(CAP_R_LED, HIGH);
    digitalWrite(IMP_LED, LOW);
    break;

  case BOTH:
    digitalWrite(CAP_L_LED, HIGH);
    digitalWrite(CAP_R_LED, HIGH);
    digitalWrite(IMP_LED, LOW);
    break;

  case JOINED:
    digitalWrite(CAP_L_LED, LOW);
    digitalWrite(CAP_R_LED, LOW);
    digitalWrite(IMP_LED, HIGH);
    break;

  default:
    digitalWrite(CAP_L_LED, LOW);
    digitalWrite(CAP_R_LED, LOW);
    digitalWrite(IMP_LED, LOW);
    break;
  }
}

/**
 * @brief
 *
 */
void updateActiveDisplay()
{
  switch (curOutputState)
  {
  case LEFT:
    sprintf(activeDisplayString, "%4s | %4s | %4s", " ON ", "", "");
    break;

  case RIGHT:
    sprintf(activeDisplayString, "%4s | %4s | %4s", "", " ON ", "");
    break;

  case BOTH:
    sprintf(activeDisplayString, "%4s | %4s | %4s", " ON ", " ON ", "");
    break;

  case JOINED:
    sprintf(activeDisplayString, "%4s | %4s | %4s", "", "", " ON ");
    break;

  case IDLE:
    sprintf(activeDisplayString, "%4s | %4s | %4s", "", "", "");
    break;
  }

  lcd.setCursor(0, 3);
  lcd.print(activeDisplayString);
  return;
}