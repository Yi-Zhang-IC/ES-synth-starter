#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

#include <bitset>
#include <optional>
#include <array>

#include "BoardIO.hpp"
#include "QuadratureEncoder.hpp"
#include "BitUtils.hpp"

U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);
HardwareTimer audioSampleClock;

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
    const TickType_t xInterval = 10 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    QuadratureEncoder knobs[4];

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        // Scan input matrix
        std::bitset<32> inputs;
        for (int rowIdx = 0; rowIdx < 8; rowIdx++) {
            selectRow(rowIdx);
            auto rowInputs = readRow();
            inputs |= (rowInputs.to_ulong() << (rowIdx * 4));
        }
        inputs.flip(); // Inputs are active-low; flip back to positive logic

        // Decode knobs
        for (int knobIdx = 0; knobIdx < 4; knobIdx++) {
            uint8_t knob_signal_bitpos = 18 - (2 * knobIdx);
            uint8_t knob_signals = BitUtils::extractBitField(inputs.to_ulong(), knob_signal_bitpos, 2);
            knobs[knobIdx].update(knob_signals);
        }

        // Critical section for updating the shared state
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        for (int i = 0; i < 4; i++) {
            sysState.knobPos[i] = knobs[i].getPosition() / 2;
        }
        xSemaphoreGive(sysState.mutex);

        auto pianoKeys = inputs.to_ulong() & 0x00000FFF;
        std::optional<uint8_t> noteIdx = 0;
        if (!pianoKeys) {
            noteIdx = {};
        } else {
            noteIdx = BitUtils::highestBitSet(pianoKeys);
        }

        auto newStepSize = noteIdx.has_value() ? stepSizes[noteIdx.value()] : 0;
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        currentStepSize = newStepSize;
        sysState.noteIdx = noteIdx;
        xSemaphoreGive(sysState.mutex);
    }
}

// Function to update the display
void displayUpdateTask(void *pvParameters)
{
    const TickType_t xInterval = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        u8g2.clearBuffer(); // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

        auto displayedNoteName = sysState.noteIdx.has_value() ? noteNames[sysState.noteIdx.value()] : "None";
        u8g2.setCursor(0, 8);
        u8g2.printf("Note: %s", displayedNoteName.c_str());

        u8g2.setCursor(0, 19);
        u8g2.printf("Knobs: %d, %d, %d, %d", sysState.knobPos[0], sysState.knobPos[1], sysState.knobPos[2],
                    sysState.knobPos[3]);

        u8g2.setCursor(0, 30);
        u8g2.printf("Volume: 12%%");

        u8g2.sendBuffer();

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
