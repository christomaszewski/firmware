#include "Platypus.h"
#include "Components.h"
// #include <adk.h>

// Arduino headers used in Platypus.h
// (informs the IDE to link these libraries)
#include <Servo.h>
#include <Scheduler.h>

// JSON parsing library
#include <ArduinoJson.h>

// TODO: remove me
#include "Board.h"

//Typedefs
typedef platypus::SerialState SerialState;


// ADK USB Host configuration
/* Make server accept version 4.x.x */
char applicationName[] = "Platypus Server"; // the app on Android
char accessoryName[] = "Platypus Control Board"; // your Arduino board
char companyName[] = "Platypus LLC";
char versionNumber[] = "3.0";
char serialNumber[] = "3";
char url[] = "http://senseplatypus.com";

// Board parameters
char firmwareVersion[] = "4.2.0";
char boardVersion[] = "4.2.0";

// Android send/receive buffers
const size_t INPUT_BUFFER_SIZE = 512;
char input_buffer[INPUT_BUFFER_SIZE+1];
char debug_buffer[INPUT_BUFFER_SIZE+1];

const size_t OUTPUT_BUFFER_SIZE = 576;
char output_buffer[OUTPUT_BUFFER_SIZE+3];

static unsigned long last_command_time = 0;
static unsigned long time_at_connected = 0;

// Time betweeen commands before we consider the Android/Raspberry PI
// server to be unresponsive - keep this fairly loose until heartbeat implemented
const size_t RESPONSE_TIMEOUT_MS = 3000;
const size_t CONNECT_STANDBY_TIMEOUT = 5000;

// Define the systems on this board
// TODO: move this board.h?
platypus::Led rgb_led;

/**
 * Wrapper for ADK send command that copies data to debug port.
 * Requires a null-terminated char* pointer.
 */
void send(char *str)
{
  // Add newline terminatio
  // TODO: Make sure we don't buffer overflow
  size_t len = strlen(str);
  str[len++] = '\r';
  str[len++] = '\n';
  str[len] = '\0';

  // Write string to ADK Port (Android Phone)
  //if (adk.isReady()) adk.write(len, (uint8_t*)str);
  // Write string to Serial Port (Raspberry PI/ODroid)
  Serial.print(str);
}

/**
 * Returns a JSON error message to the connected USB device and the
 * serial debugging console.
 */
void reportError(const char *error_message, const char *buffer)
{
  // Construct a JSON error message.
  snprintf(output_buffer, OUTPUT_BUFFER_SIZE,
           "{"
           "\"type\":\"error\","
           "\"data\": \"%s\","
           "\"args\": \"%s\""
           "}",
           error_message, buffer);
  send(output_buffer);
}

/**
 * Handler to respond to incoming JSON commands and dispatch them to
 * configurable hardware components.
 */
void handleCommand(char *buffer)
{

  /* Don't need this? SHould be handled below
     if (strcmp(buffer,"arm") == 0)
     {
     Serial.println("Arming eboard");
     platypus::eboard->set("cmd","arm");
     return;
     }
  */

  //Serial.print("Handling command: ");
  //Serial.println(buffer);

  // Allocate buffer for JSON parsing
  StaticJsonDocument<200> jsonBuffer;

  // Attempt to parse JSON in buffer
  DeserializationError error  = deserializeJson(jsonBuffer, buffer);

  // Check for parsing error
  if (error)
    {
      // Parsing Failure
      reportError(error.c_str(), buffer);
      return;
    }

  JsonObject command = jsonBuffer.as<JsonObject>();

  for (JsonPair p :  command)
  {
      const char * key = p.key().c_str();

      platypus::Configurable * target_object;
      size_t object_index;

      // Determine target object
      switch (key[0]){
      case 'm': // Motor command
        object_index = key[1] - '0';

        if (object_index >= board::NUM_MOTORS){
          reportError("Invalid motor index.", buffer);
          return;
        }

        target_object = platypus::motors[object_index];
        break;

      case 's': // Sensor command
        object_index = key[1] - '0';

        if (object_index >= board::NUM_SENSORS){
          reportError("Invalid sensor index.", buffer);
          return;
        }
        target_object = platypus::sensors[object_index];
        break;

      case 'e': //eboard command
        target_object = platypus::eboard;
        break;

      default: // Unrecognized target
        reportError("Unknown command target.", buffer);
        return;
      }

      // Extract JsonObject with param:value pairs
      JsonObject paramsObj = p.value().as<JsonObject>();
      // Todo: Move this parsing to specific components and pass ref to params instead
      // Iterate over and set parameter:value pairs on target object
      for (JsonPair param : paramsObj)
        {

          const char * param_name = param.key().c_str();
          bool setSuccess = false;

          //Serial.print("Sending command to ");
          //Serial.print(key);
          //Serial.print(": ");
          //Serial.print(param_name);
          //Serial.print(" : ");
          if (param.value().is<char*>())
          {
            setSuccess = target_object->set(param_name, param.value().as<char*>());
            //Serial.println(param.value().as<char*>());
          } else if (param.value().is<float>())
          {
            setSuccess = target_object->set(param_name, param.value().as<float>());
            //Serial.println(param.value().as<float>());
          }

          if (!setSuccess) {
            reportError("Could not set specified parameter.", buffer);
            continue; // Todo: Should we return or continue?
          }
        }
    }
}

void setup()
{
  delay(100);

  // Latch power shutdown line high to keep board from turning off.
  pinMode(board::PWR_KILL, OUTPUT);
  digitalWrite(board::PWR_KILL, HIGH);

  // Initialize debugging serial console.
  Serial.begin(115200);
  // Start the system in the disconnected state

  // Set ADC Precision:
  analogReadResolution(12);

  // Initialize EBoard object
  platypus::eboard = new platypus::EBoard();
  platypus::eboard->setState(SerialState::STANDBY);

  // Initialize and power all peripherals (WiFi & Pump)
  platypus::peripherals[0] = new platypus::Peripheral(0, true);
  platypus::peripherals[1] = new platypus::Peripheral(1, true);

  // Initialize External sensors
  platypus::sensors[0] = new platypus::EmptySensor(0, 0);
  platypus::sensors[1] = new platypus::EmptySensor(1, 1);
  platypus::sensors[2] = new platypus::AdafruitGPS(2, 2);
  platypus::sensors[3] = new platypus::EmptySensor(3, 3); // No serial on sensor 3!!!

  // Initialize Internal sensors
  platypus::sensors[4] = new platypus::BatterySensor(4);
  platypus::sensors[5] = new platypus::IMU(5);

  // Initialize motors
  platypus::motors[0] = new platypus::AfroESC(0);
  platypus::motors[1] = new platypus::AfroESC(1);

  // Make the ADK buffers into null terminated string.
  debug_buffer[INPUT_BUFFER_SIZE] = '\0';
  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';
  // Create secondary tasks for system.
  Scheduler.startLoop(motorUpdateLoop);
  Scheduler.startLoop(serialLoop);
  Scheduler.startLoop(debugLoop);

  // Initialize Platypus library.
  platypus::init();

  // Turn LED to startup state.
  //rgb_led.set(255, 0, 255);
  delay(100);
}

void loop()
{
  /*
    If the board is in an active state and hasnt recieved a command
    in a while drop it to connected
  */

  switch (platypus::eboard->getState()){
    case SerialState::ACTIVE:
      if (millis() - last_command_time >= RESPONSE_TIMEOUT_MS){
        platypus::eboard->setState(SerialState::CONNECTED);
        Serial.println("{\"type\":\"state\",\"data\":\"connected\"}");
      }
      break;
    case SerialState::CONNECTED:
      if (millis() - last_command_time >= CONNECT_STANDBY_TIMEOUT){
        platypus::eboard->setState(SerialState::STANDBY);
        Serial.println("{\"type\":\"state\",\"data\":\"standby\"}");
      }
      break;
    case SerialState::STANDBY:
      if (millis() - last_command_time < CONNECT_STANDBY_TIMEOUT){
        platypus::eboard->setState(SerialState::CONNECTED);
        Serial.println("{\"type\":\"state\",\"data\":\"connected\"}");
      }
  }

  yield();
}

/**
 * Periodically sends motor velocity updates.
 */
void motorUpdateLoop()
{
  // Handle the motors appropriately for each system state.
  switch (platypus::eboard->getState())
    {
    case SerialState::STANDBY:
      // Turn off motors.
      for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx)
        {
          platypus::Motor* motor = platypus::motors[motor_idx];
          if (motor->is_armed())
            {
              //Serial.print("Disabling motor "); Serial.println(motor_idx);
              motor->disarm();
            }
        }
      break;
    case SerialState::CONNECTED:
      // Decay all motors exponentially towards zero speed.
      for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx)
        {
          platypus::Motor* motor = platypus::motors[motor_idx];
          motor->set("v", 0.0);
        }
      break;
    case SerialState::ACTIVE:
      // Rearm motors if necessary.
      for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx)
        {
          platypus::Motor* motor = platypus::motors[motor_idx];
          if (!motor->is_armed())
            {
              motor->arm();
            }
        }
      break;
    }

  yield();
}

/**
 * Reads from serial console and attempts to execute commands.
 */

void serialLoop()
{
  static size_t debug_buffer_idx = 0;
  // Wait until characters are received.
  if (Serial.available())
    {
      last_command_time = millis();
      // Put the new character into the buffer, ignore \n and \r
      char c = Serial.read();
      if (c != '\n' && c != '\r'){
        debug_buffer[debug_buffer_idx++] = c;
      }
      // If it is the end of a line, or we are out of space, parse the buffer.
      if (debug_buffer_idx >= INPUT_BUFFER_SIZE || c == '\n' || c == '\r')
        {
          // Properly null-terminate the buffer.
          debug_buffer[debug_buffer_idx] = '\0';
          debug_buffer_idx = 0;

          handleCommand(debug_buffer);
        }
    }
  yield();
}

void debugLoop(){
  switch (platypus::eboard->getState()){
    case SerialState::ACTIVE:
      Serial.println("{\"type\":\"state\",\"data\":\"active\"}");

      // Send motor status update over USB
      snprintf(output_buffer, OUTPUT_BUFFER_SIZE,
               "{"
               "\"m0\":{"
               "\"v\":%f"
               "},"
               "\"m1\":{"
               "\"v\":%f"
               "}"
               "}",
               platypus::motors[0]->velocity(),
               platypus::motors[1]->velocity()
               );
      send(output_buffer);
      break;

    case SerialState::CONNECTED:
      Serial.println("{\"type\":\"state\",\"data\":\"connected\"}");
      break;

    case SerialState::STANDBY:
      Serial.println("{\"type\":\"state\",\"data\":\"standby\"}");
  }

  delay(1000);
}