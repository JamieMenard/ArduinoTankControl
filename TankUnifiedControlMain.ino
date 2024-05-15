#include <SoftwareSerial.h> // Include SoftwareSerial library for Bluetooth communication
#include "Arduino_LED_Matrix.h" // Include for LED matrix library (if used)
#include "ArduinoR4DigitDisplay.h" // Include for digit display library (if used)
#include <Arduino_CAN.h>
#include <algorithm>

#define ACK_TIMEOUT 100
const int DAC_RESOLUTION = 256;
const int analogPin = A5;
const int dacPin = A0;
const int ledPin = 13; // LED pin for connection status indicator (optional)
const int btRx = 2; // Bluetooth module receive pin
const int btTx = 3; // Bluetooth module transmit pin
float referenceVoltage = 5.0; // Assuming 5V reference voltage
float offsetVoltage = 0.83; // Offset voltage to consider

const int emerMsgLen = strlen("emergencyStop");
const int begMsgLen = strlen("beginnersMode");
const int interMsgLen = strlen("intermediateMode");
const int advMsgLen = strlen("advancedmode");
const int numOfModes = 4; //emerg, beg, int, advanced, change with more modes

int voltageIndex;

float beginnersLookup[DAC_RESOLUTION]; // Lookup table for 0.83V to 2.5V output
float intermediateLookup[DAC_RESOLUTION]; // Lookup table for 0.83V to 3.5V output
float baseThrottle[DAC_RESOLUTION];  //lookup table to get indexes from .83v to 5v
float dacOutput;
float voltage;
float voltage_min = 0.83; // Minimum voltage to consider
float voltage_max = 5.0;  // Maximum voltage from sensor (assuming 5V reference)
float target_voltage_min_beginners = 0.83;  // Minimum voltage for beginnersLookup (0.83V)
float target_voltage_max_beginners = 2.5;  // Maximum voltage for beginnersLookup (2.5V)
float target_voltage_min_intermediate = 0.83;  // Minimum voltage for intermediateLookup (0.83V)
float target_voltage_max_intermediate = 3.5;  // Maximum voltage for intermediateLookup (3.5V)

SoftwareSerial bluetooth(btRx, btTx); // Create software serial object

ArduinoLEDMatrix matrix; // Assuming LED matrix library is included

int beginnersDacOutput = 1; // Flag for 0.83-2.5V DAC output (default)
int intermediateDacOutput = 0; // Flag for 0.83-3.5V DAC output
int emergencyStopOutput = 0;   // Flag to enable/disable setting DAC to 0V
int advancedDacOutput = 0;
bool connected = false; // Flag to indicate connection status
bool modeChanged = false;
bool ack_received = false;

static uint32_t msg_cnt = 0;

int modeState[] = {emergencyStopOutput, beginnersDacOutput, intermediateDacOutput, advancedDacOutput};
int prevModeState[] = {emergencyStopOutput, beginnersDacOutput, intermediateDacOutput, advancedDacOutput};

static uint32_t const CAN_ID = 0x20;

  uint8_t frame[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
  };

typedef struct {
  bool enabled;  // Flag for emergency mode (0 - disabled, 1 - enabled)
} EmergencyModeMsg;

typedef struct {
  bool enabled;  // Flag for beginners mode (0 - disabled, 1 - enabled)
} BeginnersModeMsg;

typedef struct {
  bool enabled;  // Flag for intermediate mode (0 - disabled, 1 - enabled)
} IntermediateModeMsg;

typedef struct {
  bool enabled;  // Flag for advanced mode (0 - disabled, 1 - enabled)
} AdvancedModeMsg;

// Function to send CAN message
void sendCANMessage(uint32_t canId, const void* messageData, size_t messageSize) {
  CanMsg canMsg;

  // Set CAN message parameters
  canMsg.id = canId;  // Use uint32_t for message ID
  canMsg.data_length = messageSize;  // Length of the provided message data

  // Copy message data to CAN message data array
  memcpy(&canMsg.data[0], messageData, messageSize);

  // Send CAN message
   if (int const rc = CAN.write(canMsg); rc < 0) {
    /*Serial.print("Failed to send CAN message (ID: 0x");
    Serial.print(canId, HEX);
    Serial.println(")!");*/
  } else {
    Serial.print("Sent CAN message (ID: 0x");
    Serial.print(canId, HEX);
    Serial.println(")!");
  }
}


void setup() {
  float voltage_step = (voltage_max - voltage_min) / (DAC_RESOLUTION - 1);
  float newVoltage;

  matrix.begin();

  // Fill beginnersLookup table
  for (int i = 0; i < DAC_RESOLUTION; i++) {
    voltage = voltage_min + i * voltage_step;
    float scaling_factor = (target_voltage_max_beginners - target_voltage_min_beginners) / (5 - .83);
    float newVoltage = target_voltage_min_beginners + scaling_factor * (voltage - .83);
    beginnersLookup[i] = newVoltage;
  }
  // Fill intermediateLookup table
  for (int i = 0; i < DAC_RESOLUTION; i++) {
    voltage = voltage_min + i * voltage_step;
    float scaling_factor = (target_voltage_max_intermediate - target_voltage_min_intermediate) / (5 - .83);
    float newVoltage = target_voltage_min_intermediate + scaling_factor * (voltage - .83);
    intermediateLookup[i] = newVoltage;
  }
  //make array for baseThrottleInput
  for (int i = 0; i < DAC_RESOLUTION; i++) {
    voltage = voltage_min + i * voltage_step;
    baseThrottle[i] = voltage;
  }

  // Configure analog pin A5 as input (sensor)
  pinMode(A5, INPUT);

  // Configure analog pin A0 as output (DAC)
  pinMode(A0, OUTPUT);
  Serial.begin(9600);
  while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
  delay(500);
}


 
void loop() {
  /*modeState[0] = emergencyStopOutput;
  modeState[1] = beginnersDacOutput;
  modeState[2] = intermediateDacOutput;
  modeState[3] = advancedDacOutput;*/

  beginnersDacOutput = 0;
  advancedDacOutput = 1;

    if (CAN.available()) {
    CanMsg const msg = CAN.read();
    uint32_t canId = msg.id;
    Serial.println(msg);
    }
  float throttleIn = analogRead(A5) * (5.0 / 1023.0); // Assuming 5V reference
   // Handle button presses (unchanged)
  /*if (buttonState == xButton) {
    emergencyStopOutput = !emergencyStopOutput; // Toggle emergency stop
    beginnersDacOutput = intermediateDacOutput = advancedDacOutput = 0; // Reset other outputs
    //drawNumber(0,frame);
    //matrix.renderBitmap(frame, 8, 12);
    Serial.println("Emergency Stop Activated");
  } else if (buttonState == circleButton) {
    beginnersDacOutput = !beginnersDacOutput; // Toggle beginner DAC output
    intermediateDacOutput = emergencyStopOutput = advancedDacOutput = 0; // Reset other outputs
    drawNumber(1,frame);
    matrix.renderBitmap(frame, 8, 12);
    Serial.println("Beginners Mode Activated");
  } else if (buttonState == squareButton) {
    intermediateDacOutput = !intermediateDacOutput; // Toggle intermediate DAC output
    beginnersDacOutput = emergencyStopOutput = advancedDacOutput = 0; // Reset other outputs
    drawNumber(5,frame);
    matrix.renderBitmap(frame, 8, 12);
    Serial.println("Intermediate Mode Activated");
  } else if (buttonState == triangleButton) {
    beginnersDacOutput = intermediateDacOutput = emergencyStopOutput = 0;
    drawNumber(10,frame);
    matrix.renderBitmap(frame, 8, 12);
    Serial.println("Advanced Mode Activated");
  }*/
  
  delay(1000);

  for (int i = 0; i < (DAC_RESOLUTION - 1); i++) {
      if (abs(baseThrottle[i] - throttleIn) <= 0.1f) {
        voltageIndex = i;
        break;  // Exit the loop if found
      }
      else if(i == (DAC_RESOLUTION-1)) {
        //Serial.println("Failed");
      }
     }
  // Output DAC value based on selected flag and lookup table
  float dacOutput;
  if (emergencyStopOutput == 1) {
    dacOutput = .83; // Set DAC to 0V for emergency stop
    drawNumber((0),frame);
    matrix.renderBitmap(frame, 8, 12);
  } else if (beginnersDacOutput == 1) {
    dacOutput = beginnersLookup[voltageIndex];
    drawNumber((1),frame);
    matrix.renderBitmap(frame, 8, 12);
  } else if (intermediateDacOutput == 1) {
    dacOutput = intermediateLookup[voltageIndex];
    drawNumber((2),frame);
    matrix.renderBitmap(frame, 8, 12);
  } else {
    // Pass analog reading from A5 directly to DAC (unchanged voltage)
    dacOutput = throttleIn;
    drawNumber((3),frame);
    matrix.renderBitmap(frame, 8, 12);
  }


  float outputScalingFactor =  (DAC_RESOLUTION) / referenceVoltage;
  int scaledDacOutput = outputScalingFactor * (dacOutput);
  analogWrite(A0, (scaledDacOutput+5)); //dacOutput
  /*drawNumber((dacOutput*10),frame);
  matrix.renderBitmap(frame, 8, 12);*/
  delay(100);

  // Send CAN message based on flags
  /* Assemble a CAN message with the format of
   * 0xCA 0xFE 0x00 0x00 [4 byte message counter]
  uint8_t const msg_data[] = {0xCA,0xFE,0,0,0,0,0,0};
  memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);*/

  for (int i = 0; i < numOfModes; i++) {
    if (modeState[i] != prevModeState[i]) {
      modeChanged = true;
      break;
    }
  }
 

   if (modeChanged){
    if (emergencyStopOutput == 1) {
        EmergencyModeMsg  emergencyModeMsg;
        emergencyModeMsg.enabled = true;
        sendCANMessage(0x100U, &emergencyModeMsg, emerMsgLen);
        Serial.println("CAN SENT");
      } else if (beginnersDacOutput == 1) {
        BeginnersModeMsg beginnersModeMsg;
        beginnersModeMsg.enabled = true;
        sendCANMessage(0x200U, &beginnersModeMsg, begMsgLen);
        Serial.println("CAN SENT");
      } else if (intermediateDacOutput == 1) {
        IntermediateModeMsg intermediateModeMsg;
        intermediateModeMsg.enabled = true;
        sendCANMessage(0x300U, &intermediateModeMsg, interMsgLen);
        Serial.println("CAN SENT");
      } else if (advancedDacOutput == 1) {
        AdvancedModeMsg advancedModeMsg;
        advancedModeMsg.enabled = true;
        sendCANMessage(0x400U, &advancedModeMsg, advMsgLen);
        //Serial.println("CAN SENT");
      }
  }
  // Reset acknowledgement flag
  ack_received = false;
  // Start timer for acknowledgement timeout
  unsigned long start_time = millis();
  while (!ack_received && (millis() - start_time) < ACK_TIMEOUT) {
    // Check for received messages (optional, for debugging)
    if (CAN.available()) {
      CanMsg received_message;
      CAN.read(received_message);
      if (received_message.id == CAN_ACK_ID) {
        // Received message ID matches the acknowledgement ID
        ack_received = true;
        break;  // Exit the waiting loop
      }
    }
  }

  // Handle acknowledgement status
  if (ack_received) {
    Serial.println("Acknowledgement received!");
  } else {
    Serial.println("Acknowledgement timeout!");
    // Optional: Implement actions for timeout, like resending
  }

  /* Increase the message counter. */
  msg_cnt++;
 prevModeState[0] = emergencyStopOutput;
 prevModeState[1] = beginnersDacOutput;
 prevModeState[2] = intermediateDacOutput;
 prevModeState[3] = advancedDacOutput;
 modeChanged = false;
}
