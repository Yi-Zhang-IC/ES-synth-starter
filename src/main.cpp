#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>


U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

volatile uint32_t currentStepSize = 0;

// Phase accumulator increment sizes for each note
const uint32_t stepSizes[] = {51076057,  54113197,  57330935,  60740010,  64351799,  68178356,  72232452,  76527617,
                              81078186,  85899346,  91007187,  96418756,  102152113, 108226394, 114661870, 121480020,
                              128703598, 136356712, 144464904, 153055234, 162156372, 171798692, 182014374, 192837512,
                              204304227, 216452788, 229323741, 242960040, 257407196, 272713424, 288929808, 306110469,
                              324312744, 343597384, 364028747, 385675023};

const std::array<std::string, 12> noteNames = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

struct {
    SemaphoreHandle_t mutex;
    std::bitset<32> keys; // input state
    std::array<int32_t, 4> knobPos; // rotation state
    std::optional<uint8_t> noteIdx; // note state
} sysState;

// Function to update the phase accumulator and set the analogue output voltage
// at each sample interval
void sampleISR()
{
    static uint32_t phaseAcc = 0;

    phaseAcc += currentStepSize;
    int32_t Vout = (phaseAcc - 0x80000000) / 8;
    analogWrite(OUTR_PIN, Vout + 0x80000000);
}

// Function to scan the keyboard
void scanKeysTask(void *pvParameters)
{

  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Variables to hold the previous and current state of the knob signals
  uint8_t previousKnobState = 0; // Initial state for knob signals {B,A} set to 00
  int rotationVariable = 0; // This variable will keep track of the knob position
  int lastChange = 0; // Variable to keep track of the last direction change

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;

    // Key scanning loop
    for (int rowIdx = 0; rowIdx < 4; rowIdx++)
    {
      setRow(rowIdx);                                   // Set row select address
      delayMicroseconds(3);                             // Wait for row select to settle
      std::bitset<4> rowInputs = readCols();            // Read columns
      inputs |= (rowInputs.to_ulong() << (rowIdx * 4)); // Copy results into inputs bitset
    }

    // Decoding knob 3 using the inputs from row 3, columns 0 and 1
    uint8_t currentKnobState = (inputs[3 * 4 + 1] << 1) | inputs[3 * 4]; // {B,A} for knob 3
    int change = 0; // Variable to track the change in rotation

    // State transition decoding for knob 3
    int combinedState = (previousKnobState << 2) | currentKnobState; // Combine previous and current state

    switch (combinedState)
    {
      // Cases for valid transitions
      case 0b0001: // 00 -> 01
      case 0b1110: // 11 -> 10
        if (lastChange != combinedState) { // Check if the last change was different to avoid double counting
          change = +1;
          lastChange = combinedState;
        }
        break;
      case 0b0100: // 01 -> 00
      case 0b1011: // 10 -> 11
        if (lastChange != combinedState) {
          change = -1;
          lastChange = combinedState;
        }
        break;
      // Handle 'impossible' transitions by assuming the same direction as the last legal transition
      case 0b0011: // 00 -> 11
      case 0b1100: // 11 -> 00
        // Guess the direction based on the last change
        change = (lastChange > 0) ? +1 : -1;
        break;
      // Add cases for other impossible transitions if they are to be considered
      // e.g., 01 -> 10 and 10 -> 01
      case 0b0110: // 01 -> 10 (impossible transition)
      case 0b1001: // 10 -> 01 (impossible transition)
        change = (lastChange > 0) ? +1 : -1;
        break;
      // No change or intermediate states do not modify the rotation variable
      case 0b0000: // No change
      case 0b0101: // No change
      case 0b1010: // No change
      case 0b1111: // No change
      case 0b0010: // Intermediate state
      case 0b1000: // Intermediate state
      case 0b0111: // Intermediate state
      case 0b1101: // Intermediate state
        change = 0;
        break;
    }

    // Update the rotation variable and the lastChange
    if (change != 0) {
      rotationVariable += change; // Update the rotation variable
      lastChange = change; // Keep track of the last direction change
    }

    rotationVariable += change; // Update the rotation variable

    // Store the current state as the previous state for the next iteration
    previousKnobState = currentKnobState;

    // Critical section for updating the shared state
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.rotationVariable = rotationVariable; // Store the rotation variable in sysState
    xSemaphoreGive(sysState.mutex);


    // Perform read and conditional operations first
    uint32_t tempStepSize = 0;
    uint32_t note = 0;
    bool updateNeeded = false;

    if ((inputs.to_ulong() & 0xFFF) == 0xFFF) {
      tempStepSize = 0;
      note = 0;
      updateNeeded = true; // Indicates that store operations are needed
    } else {
      for (int i = 0; i < 12; i++) {
        if (inputs[i] == 0) {
          tempStepSize = stepSizes[i];
          note = i+1;
          updateNeeded = true; // Indicates that store operations are needed
          break; // Assuming you only need the first occurrence
        }
      }
    }

    // Use mutex to protect all the store instructions
    if (updateNeeded) {
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      __atomic_store_n(&currentStepSize, tempStepSize, __ATOMIC_RELAXED);
      __atomic_store_n(&sysState.note, note, __ATOMIC_RELAXED);
      xSemaphoreGive(sysState.mutex);
    }

  }
}

// Function to update the display
void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Update display
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
    // u8g2.drawStr(2, 10, "Hello World!"); // write something to the internal memory

    // Print the notes state
    u8g2.setCursor(2, 10);
    u8g2.printf("Notes: %s", note_name[sysState.note]);
    u8g2.sendBuffer(); // transfer internal memory to the display

    // Print the rotation state
    u8g2.setCursor(2, 20);
    u8g2.printf("Rotation: %d", sysState.rotationVariable);
    u8g2.sendBuffer(); // transfer internal memory to the display

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void setup()
{
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(WEN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);

    // Initialise display
    setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
    setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply
    u8g2.begin();
    u8g2.setContrast(0);

    // Initialise UART
    Serial.begin(115200);
    Serial.println("Hello World");

    analogWriteResolution(32 /* bits */); // To eliminate conversions to uint8_t
    audioSampleClock.setup(TIM6);
    audioSampleClock.setOverflow(22000, HERTZ_FORMAT);
    audioSampleClock.attachInterrupt(sampleISR);
    audioSampleClock.resume();

    // Mutex handle
    sysState.mutex = xSemaphoreCreateMutex();

    // Set up task handles
    TaskHandle_t scanKeysHandle = nullptr;
    xTaskCreate(scanKeysTask, /* Function that implements the task */
                "scanKeys", /* Text name for the task */
                64, /* Stack size in words, not bytes */
                NULL, /* Parameter passed into the task */
                2, /* Task priority */
                &scanKeysHandle);

    TaskHandle_t displayUpdateHandle = nullptr;
    xTaskCreate(displayUpdateTask, /* Function that implements the task */
                "displayUpdate", /* Text name for the task */
                256, /* Stack size in words, not bytes */
                NULL, /* Parameter passed into the task */
                1, /* Task priority */
                &displayUpdateHandle);

    // Set up FreeRTOS task
    vTaskStartScheduler();
}

void loop()
{
    // This should never be reached
    while (1) {
        Serial.println("Error: loop() should never be reached");
        delay(1000);
    }
}
