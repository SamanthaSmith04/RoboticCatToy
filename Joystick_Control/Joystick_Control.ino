#include <Bluepad32.h>
#include <Adafruit_NeoPixel.h> // Include the Adafruit NeoPixel library

int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;

int motor2Pin1 = 25;
int motor2Pin2 = 19;
int enable2Pin = 32;

int ledPin = 33;

const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 0;
const int resolution = 8;

const int CONTROLLER_MAX = 512;

const int NUM_LEDS = 8;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, ledPin, NEO_GRB + NEO_KHZ800); 


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }

  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void printXY(ControllerPtr ctl){
  Serial.printf("buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d", 
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY()        // (-511 - 512) right Y axis
  );
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void setMotor(int pin1, int pin2, int enablePin, int dutyCycle, int dir) {
  if (dir == 0) { // Stop
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, 0); // Stop the motor
  } else if (dir == 1) { // Move forward
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablePin, dutyCycle); // Set speed
  } else { // Move backward
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, dutyCycle); // Set speed
  }
}

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 
  //== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
  }
  if (ctl->buttons() != 0x0001) {
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    setMotor(motor1Pin1, motor1Pin2, enable1Pin, 200, 1);
    printf("motor1 forward");
  }
  if (ctl->buttons() != 0x0004) {
    setMotor(motor1Pin1, motor1Pin2, enable2Pin, 200, 0);
    printf("motor1 off");
  }

  //== PS4 Triangle button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
  }
  if (ctl->buttons() != 0x0008) {
  }

  //== PS4 Circle button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    setMotor(motor2Pin1, motor2Pin2, 0, 200, -1);
  }
  if (ctl->buttons() != 0x0002) {
    setMotor(motor2Pin1, motor2Pin2, 0, 200, 0);
  }

  //== PS4 Dpad UP button = 0x01 ==//
  if (ctl->buttons() == 0x01) {
    // code for when dpad up button is pushed
  }
  if (ctl->buttons() != 0x01) {
    // code for when dpad up button is released
  }

  //==PS4 Dpad DOWN button = 0x02==//
  if (ctl->buttons() == 0x02) {
    // code for when dpad down button is pushed
  }
  if (ctl->buttons() != 0x02) {
    // code for when dpad down button is released
  }

  //== PS4 Dpad LEFT button = 0x08 ==//
  if (ctl->buttons() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->buttons() != 0x08) {
    // code for when dpad left button is released
  }

  //== PS4 Dpad RIGHT button = 0x04 ==//
  if (ctl->buttons() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->buttons() != 0x04) {
    // code for when dpad right button is released
  }

  //== PS4 R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== PS4 R2 trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when R2 button is pushed
  }
  if (ctl->buttons() != 0x0080) {
    // code for when R2 button is released
  }

  //== PS4 L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== PS4 L2 trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when L2 button is pushed
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }

  // LEFT MOTOR / JOYSTICK DRIVE
  if (ctl->axisY() > 25 || ctl->axisY() < -25) { // if not in dead zone
    int dutyCycle = 0;
    int dir = 0;
    if (ctl->axisY() > 0) { // forward
      dutyCycle = map(ctl->axisY(), 0, 512, 0, 200);
      dir = 1;
    }
    else {
      dutyCycle = map(-1* ctl->axisY(), 0, 512, 0, 200);
      dir = -1;
    }
    setMotor(motor1Pin1, motor1Pin2, enable1Pin, dutyCycle, dir);
  }
  else { // is in dead zone
    setMotor(motor1Pin1, motor1Pin2, enable1Pin, 0, 0);
  }

  // Right MOTOR / JOYSTICK DRIVE
  if (ctl->axisRY() > 25 || ctl->axisRY() < -25) { // if not in dead zone
    int dutyCycle = 0;
    int dir = 0;
    if (ctl->axisRY() > 0) { // forward
      dutyCycle = map(ctl->axisRY(), 0, 512, 0, 200);
      dir = 1;
    }
    else {
      dutyCycle = map(-1* ctl->axisRY(), 0, 512, 0, 200);
      dir = -1;
    }
    setMotor(motor2Pin1, motor2Pin2, enable2Pin, dutyCycle, dir);
  }
  else { // is in dead zone
    setMotor(motor2Pin1, motor2Pin2, enable2Pin, 0, 0);
  }

  //dumpGamepad(ctl);
  printXY(ctl);
}

void processControllers() {
  Serial.println("process controller is being called");
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setLEDStripFromSpeed(int dutyCycle) {
  float color = map(dutyCycle, 0, 200, 25, 255);

  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, 255, 0, color);
  }
  strip.show();
}


void setup() {
  Serial.begin(115200);

  strip.begin();

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);


  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated){
    processControllers();



    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
    Serial.printf("test");
  }

  delay(150);
}
