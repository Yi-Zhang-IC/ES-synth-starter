#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

// Constants
const uint32_t interval = 100; // Display update interval

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int WEN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

volatile uint32_t currentStepSize = 0;

// Step sizes for each note
const uint32_t stepSizes[] = {
    51076057, 54113197, 57330935, 60740010, 64351799, 68178356,
    72232452, 76527617, 81078186, 85899346, 91007187, 96418756,
    102152113, 108226394, 114661870, 121480020, 128703598, 136356712,
    144464904, 153055234, 162156372, 171798692, 182014374, 192837512,
    204304227, 216452788, 229323741, 242960040, 257407196, 272713424,
    288929808, 306110469, 324312744, 343597384, 364028747, 385675023
    };

// Notes for each step
const char *note_name[] = {
    "None","C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"};

//
struct
{
  // input state
  std::bitset<32> inputs;
  // note state
  std::uint32_t note;
} sysState;

uint8_t highestBitSet(std::bitset<12> bits)
{
  for (int i = 11; i >= 0; i--)
  {
    if (bits.test(i))
    {
      return i;
    }
  }
  return 0;
}

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(WEN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(WEN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(WEN_PIN, LOW);
}

// Function to read inputs using key matrix
std::bitset<4> readCols()
{
  std::bitset<4> result;

  // Read the inputs
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

// Function to select a given row of the switch matrix
void setRow(uint8_t rowIdx)
{
  // Disable row select enable
  digitalWrite(WEN_PIN, LOW);

  // Set row select address pins
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);

  // Enable row select enable
  digitalWrite(WEN_PIN, HIGH);
}

// Function to update the phase accumulator and set the analogue output voltage at each sample interval
void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, (Vout + 128) / 8);
}

// Function to scan the keyboard
void scanKeysTask(void *pvParameters)
{

  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;

    // Key scanning loop
    for (int rowIdx = 0; rowIdx < 3; rowIdx++)
    {
      setRow(rowIdx);                                   // Set row select address
      delayMicroseconds(3);                             // Wait for row select to settle
      std::bitset<4> rowInputs = readCols();            // Read columns
      inputs |= (rowInputs.to_ulong() << (rowIdx * 4)); // Copy results into inputs bitset
    }

    // Store the result in sysState.inputs
    sysState.inputs = inputs;

    // Check state of each key in inputs
    if ((inputs.to_ulong() & 0xFFF) == 0xFFF)
    {
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
      sysState.note = 0;
    }
    else
    {
      uint32_t tempStepSize;
      uint32_t note = 0;
      for (int i = 0; i < 12; i++)
      {
        if (inputs[i] == 0)
        {
          tempStepSize = stepSizes[i];
          note = i+1;
        }
      }
      __atomic_store_n(&currentStepSize, tempStepSize, __ATOMIC_RELAXED);
      __atomic_store_n(&sysState.note, note, __ATOMIC_RELAXED);
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
    u8g2.drawStr(2, 10, "Hello World!"); // write something to the internal memory

    // Print the notes state
    u8g2.setCursor(2, 20);
    u8g2.printf("Notes: %s", note_name[sysState.note]);
    u8g2.sendBuffer(); // transfer internal memory to the display

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Set pin directions
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
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  for (int i = 0; i < 12; i++)
  {
    Serial.printf("Step size for note %d: %d\n", i, stepSizes[i]);
  }

  // Create timer instance
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  // Configure timer
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask, /* Function that implements the task */
      "scanKeys",   /* Text name for the task */
      64,           /* Stack size in words, not bytes */
      NULL,         /* Parameter passed into the task */
      2,            /* Task priority */
      &scanKeysHandle);

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask, /* Function that implements the task */
      "displayUpdate",   /* Text name for the task */
      256,               /* Stack size in words, not bytes */
      NULL,              /* Parameter passed into the task */
      1,                 /* Task priority */
      &displayUpdateHandle);

  // Set up FreeRTOS task
  vTaskStartScheduler();
}

void loop()
{
  // This should never be reached
  while (1)
  {
    Serial.println("Error: loop() should never be reached");
    delay(1000);
  }
}
