#include <Bluepad32.h>


#include "AudioFileSourceSD_MMC.h"
#include "AudioOutputI2S.h"
#include "AudioGeneratorMP3.h"
#include "SD_MMC.h"
#include "FS.h"

// Declare pointers for the MP3 generator, file source, and output.
AudioGeneratorMP3 *mp3;
AudioFileSourceSD_MMC *file;
AudioOutputI2S *out;

bool next = false;


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

void setup() {
  // Start the serial communication.
  Serial.begin(115200);
  delay(1000);

  // Initialize the SD card. If it fails, print an error message.
  if (!SD_MMC.begin()) {
    Serial.println("SD card mount failed!");
  }

  const uint8_t *addr = BP32.localBdAddress();

    BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);


  // Open the MP3 file from the SD card. Replace "/To Alice.mp3" with your own MP3 file name.
  file = new AudioFileSourceSD_MMC("/audio1.mp3");
  
  // Set up the I2S output on ESP32's internal DAC.
  out = new AudioOutputI2S(0, 1);
  
  // Set the output to mono.
  out->SetOutputModeMono(true);

  // Initialize the MP3 generator with the file and output.
  mp3 = new AudioGeneratorMP3();
  mp3->begin(file, out);
}

void loop() {
  bool dataUpdated = BP32.update();

  // If the MP3 is running, loop it. Otherwise, stop it.
  if (mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop(); }
  } 
  // If the MP3 is not running, print a message and wait for 1 second.
  else {
    //Serial.println("MP3 done");

    next = true;
  }

  if (next) {
    file = new AudioFileSourceSD_MMC("/audio2.mp3");

    mp3->begin(file, out);
    next = false;
  }
}
