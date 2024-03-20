#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <stream_buffer.h>
#include <ES_CAN.h>

#include <optional>
#include <array>
#include <list>

#include "BoardIO.hpp"
#include "QuadratureEncoder.hpp"
#include "BitUtils.hpp"
#include "Audio.hpp"
#include "Phasor.hpp"
#include "UniqueID.hpp"
#include "MessageFormatter.hpp"

const uint32_t AUDIO_BUFFER_SIZE = 256;
const uint32_t AUDIO_SAMPLE_RATE_HZ = 22000;

const uint32_t CAN_ID_BASE = 0x120;
uint32_t ownCANID;
HardwareTimer audioSampleClock;
StreamBufferHandle_t generatedSamples;

struct keyboardFormation {
    uint8_t ownIndex = 0;
    bool othersPresent = false;
    bool isWestMost = false;
    bool isEastMost = false;
} keyboardFormation;

struct audioState {
    SemaphoreHandle_t mutex;
    std::list<Phasor> playingNotes;
    uint8_t volumePercent;
    WaveformName waveformType;
} audioState;

void isrOutputAudioSample()
{
    uint16_t sample;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    auto sampleReceived =
        xStreamBufferReceiveFromISR(generatedSamples, &sample, sizeof(sample), &xHigherPriorityTaskWoken);
    if (sampleReceived != 0) {
        analogWrite(OUTR_PIN, sample);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void scanKeysTask(void *pvParameters)
{
    const auto xInterval = 10 / portTICK_PERIOD_MS;
    auto xLastWakeTime = xTaskGetTickCount();

    uint16_t pianoKeys;
    std::array<QuadratureEncoder, 4> knobs;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        // Read all inputs including piano keys and knobs
        uint32_t inputs = 0;
        for (int rowIdx = 0; rowIdx < 8; rowIdx++) {
            inputs |= (selectAndReadRow(rowIdx) << (rowIdx * 4));
        }
        inputs = ~inputs;

        // Get change in piano key states
        uint16_t newPianoKeys = inputs & 0xFFF;
        uint16_t changedKeys = newPianoKeys ^ pianoKeys;

        // Process changed key states: start/stop notes, send CAN messages
        for (int i = 0; i < 12; i++) {
            if (BitUtils::bitIsSet(changedKeys, i)) {
                bool keyPressed = BitUtils::bitIsSet(newPianoKeys, i);
                uint32_t keyFreqHz = noteFreqs.at(i);

                xSemaphoreTake(audioState.mutex, portMAX_DELAY);
                if (keyPressed) {
                    audioState.playingNotes.push_back(Phasor(keyFreqHz, AUDIO_SAMPLE_RATE_HZ));
                } else {
                    audioState.playingNotes.remove_if(
                        [keyFreqHz](Phasor &phasor) { return phasor.getFreqHz() == keyFreqHz; });
                }
                xSemaphoreGive(audioState.mutex);

                if (keyboardFormation.othersPresent) {
                    auto msg = MessageFormatter::keyEvent(keyPressed, i);
                    CAN_TX(CAN_ID_BASE | keyboardFormation.ownIndex, msg.data(), msg.size());
                }
            }
        }
        pianoKeys = newPianoKeys;

        // Decode knobs
        for (int knobIdx = 0; knobIdx < 4; knobIdx++) {
            uint8_t knob_signal_bitpos = 18 - (2 * knobIdx);
            uint8_t knob_signals = BitUtils::extractBitField(inputs, knob_signal_bitpos, 2);
            knobs.at(knobIdx).update(knob_signals);
        }

        if (keyboardFormation.othersPresent) {
            uint8_t canRxMsg[8];
            uint32_t canRxId;
            uint8_t canRxLength;
            while (CAN_CheckRXLevel()) {
                CAN_RX(canRxId, canRxMsg, canRxLength);
            }
        }
    }
}

void displayUpdateTask(void *pvParameters)
{
    U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

    setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
    delay(1);
    setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset

    delay(keyboardFormation.ownIndex * 5); // Stagger power enable to avoid voltage dip -> reset
    setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

    u8g2.begin();

    const auto xInterval = 100 / portTICK_PERIOD_MS;
    auto xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        u8g2.clearBuffer(); // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

        u8g2.setCursor(0, 19);
        u8g2.printf("Index: %u", keyboardFormation.ownIndex);

        u8g2.setCursor(0, 30);
        u8g2.printf("Volume: 100%%");

        u8g2.sendBuffer();

        digitalToggle(LED_BUILTIN);
    }
}

void generateSamplesTask(void *pvParameters)
{
    const size_t batchSize = AUDIO_BUFFER_SIZE / 2;
    const auto xInterval = (batchSize / (AUDIO_SAMPLE_RATE_HZ / 1000)) / portTICK_PERIOD_MS;
    auto xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        bool generatedAny = false;
        int32_t sample_s20[batchSize] = {};

        xSemaphoreTake(audioState.mutex, portMAX_DELAY);
        if (!audioState.playingNotes.empty()) {
            for (int i = 0; i < batchSize; i++) {
                for (auto &phasor: audioState.playingNotes) {
                    sample_s20[i] += WaveformGenerators.at(audioState.waveformType)(phasor.next());
                }
            }
            generatedAny = true;
        }
        xSemaphoreGive(audioState.mutex);

        if (generatedAny) {
            uint16_t sample_u16[batchSize];
            for (int i = 0; i < batchSize; i++) {
                auto sample = (sample_s20[i] >> 3) - INT16_MIN;
                if (sample < 0) {
                    sample = 0;
                }
                if (sample > UINT16_MAX) {
                    sample = UINT16_MAX;
                }
                sample_u16[i] = sample;
            }
            xStreamBufferSend(generatedSamples, &sample_u16, batchSize * sizeof(sample_u16[0]), portMAX_DELAY);
        } else {
            uint16_t neutral = UINT16_MAX / 2;
            xStreamBufferSend(generatedSamples, &neutral, sizeof(neutral), portMAX_DELAY);
        }
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

    pinMode(C0_PIN, INPUT_PULLUP);
    pinMode(C1_PIN, INPUT_PULLUP);
    pinMode(C2_PIN, INPUT_PULLUP);
    pinMode(C3_PIN, INPUT_PULLUP);
    pinMode(JOYX_PIN, INPUT_ANALOG);
    pinMode(JOYY_PIN, INPUT_ANALOG);

    // DEBUG
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1);

    // Initialise debug UART
    Serial.begin(115200);

    // Initialise CAN
    CAN_Init(false);
    setCANFilter(CAN_ID_BASE, 0x7E0);
    CAN_Start();

    // Detect attached keyboards
    setOutMuxBit(HKOW_BIT, HIGH);
    setOutMuxBit(HKOE_BIT, HIGH);

    // Allow enough time for all keyboards to boot before checking HK lines
    delay(100);

    bool westConnected = !BitUtils::bitIsSet(selectAndReadRow(5), 3);
    bool eastConnected = !BitUtils::bitIsSet(selectAndReadRow(6), 3);

    if (!westConnected && !eastConnected) {
        Serial.println("No other keyboards connected");
        keyboardFormation.othersPresent = false;
        keyboardFormation.ownIndex = 0;
    } else {
        keyboardFormation.othersPresent = true;

        bool thisAnnounced = false;
        bool allDone = false;

        if (!westConnected) {
            // This is the westmost keyboard; initiate enumeration
            keyboardFormation.ownIndex = 0;
            keyboardFormation.isWestMost = true;
            keyboardFormation.isEastMost = false;
            ownCANID = CAN_ID_BASE;

            delay(100);
            setOutMuxBit(HKOE_BIT, LOW);
            auto msg = MessageFormatter::enumerationPositionAndID(keyboardFormation.ownIndex, UniqueID::calculate());
            CAN_TX(ownCANID, msg.data(), msg.size());

            thisAnnounced = true;
        } else {
            keyboardFormation.isWestMost = false;
        }

        while (!thisAnnounced) {
            uint32_t rxID;
            uint8_t rxBuffer[8];
            uint8_t rxLength;
            CAN_RX(rxID, rxBuffer, rxLength);

            bool westAsserted = !BitUtils::bitIsSet(selectAndReadRow(5), 3);
            if (westAsserted) {
                // Not our turn yet; wait for the next message
                continue;
            }

            if ((rxBuffer[0] & 0xE0) == 0xC0) {
                // ID & position announcement msg
                uint8_t prevIndex = rxBuffer[0] & 0x1F;
                keyboardFormation.ownIndex = prevIndex + 1;
                ownCANID = CAN_ID_BASE + keyboardFormation.ownIndex;

                setOutMuxBit(HKOE_BIT, LOW);
                auto msg =
                    MessageFormatter::enumerationPositionAndID(keyboardFormation.ownIndex, UniqueID::calculate());
                CAN_TX(ownCANID, msg.data(), msg.size());

                thisAnnounced = true;

                if (!eastConnected) {
                    // This is the eastmost keyboard; also send "done" message
                    keyboardFormation.isEastMost = true;

                    auto msg = MessageFormatter::enumerationDone();
                    CAN_TX(ownCANID, msg.data(), msg.size());

                    allDone = true;
                } else {
                    keyboardFormation.isEastMost = false;
                }
            }
        }

        while (!allDone) {
            uint32_t rxID;
            uint8_t rxBuffer[8];
            uint8_t rxLength;
            CAN_RX(rxID, rxBuffer, rxLength);

            if ((rxBuffer[0] & 0xE0) == 0xE0) {
                allDone = true;
            }
        }

        setOutMuxBit(HKOW_BIT, LOW);
        setOutMuxBit(HKOE_BIT, LOW);
    }

    // Set up audio generation and output
    audioState.volumePercent = 100;
    audioState.waveformType = WaveformName::TRIANGLE;
    audioState.mutex = xSemaphoreCreateMutex();

    analogWriteResolution(16 /* bits */);
    audioSampleClock.setup(TIM6);
    audioSampleClock.setOverflow(AUDIO_SAMPLE_RATE_HZ, HERTZ_FORMAT);
    audioSampleClock.attachInterrupt(isrOutputAudioSample);
    audioSampleClock.resume();

    // Set up task handles
    TaskHandle_t scanKeysHandle = nullptr;
    xTaskCreate(scanKeysTask, "scanKeys", 1024, nullptr, 2, &scanKeysHandle);

    TaskHandle_t displayUpdateHandle = nullptr;
    xTaskCreate(displayUpdateTask, "displayUpdate", 1024, nullptr, 1, &displayUpdateHandle);

    TaskHandle_t generateSamplesTaskHandle = nullptr;
    xTaskCreate(generateSamplesTask, "generateSamples", 1024, nullptr, 1, &generateSamplesTaskHandle);
    generatedSamples = xStreamBufferCreate(AUDIO_BUFFER_SIZE * sizeof(uint16_t), 1);

    vTaskStartScheduler();
}

void loop() {}
