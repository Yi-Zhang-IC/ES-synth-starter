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
#include "Note.hpp"
#include "UniqueID.hpp"
#include "MessageFormatter.hpp"
#include "EventTypes.hpp"
#include "Capped.hpp"
#include "Bitmaps.hpp"

const uint32_t AUDIO_BUFFER_SIZE = 256;
const uint32_t AUDIO_SAMPLE_RATE_HZ = 22000;

const uint32_t CAN_ID_BASE = 0x120;
uint32_t ownCANID;
HardwareTimer audioSampleClock;
StreamBufferHandle_t generatedSamples;

struct KeyboardFormation {
    uint8_t ownIndex = 0;
    bool othersPresent = false;
    bool isWestMost = false;
    bool isEastMost = false;
} KeyboardFormation;

struct AudioState {
    SemaphoreHandle_t mutex;
    std::list<Note> playingNotes;
    CappedInt<0, 7> volumeLevel;
    WaveformName waveformType;
} AudioState;

QueueHandle_t eventQueue;


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
                uint8_t noteIdxInOctave = i;
                uint8_t noteIdx = 12 * KeyboardFormation.ownIndex + noteIdxInOctave;
                GenericEvent keyEvent = { EventType::NOTE_CHANGE, keyPressed, noteIdx };
                xQueueSendToBack(eventQueue, &keyEvent, portMAX_DELAY);

                if (KeyboardFormation.othersPresent) {
                    auto msg = MessageFormatter::keyEvent(keyPressed, i);
                    CAN_TX(CAN_ID_BASE | KeyboardFormation.ownIndex, msg.data(), msg.size());
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

        auto volumeKnobChange = knobs.at(0).getChange();
        if (volumeKnobChange) {
            Serial.printf("volumeKnobChange = %i\n", volumeKnobChange);
            auto newVolume = AudioState.volumeLevel + volumeKnobChange;
            GenericEvent e = { EventType::SETTINGS_UPDATE, (uint8_t)SettingName::VOLUME, newVolume };
            xQueueSendToBack(eventQueue, &e, portMAX_DELAY);
        }

        auto waveformKnobChange = knobs.at(3).getChange();
        if (waveformKnobChange) {
            Serial.printf("waveformKnobChange = %i\n", waveformKnobChange);
            WaveformName newWaveform;
            if (waveformKnobChange > 0) {
                switch (AudioState.waveformType) {
                case WaveformName::SAWTOOTH: newWaveform = WaveformName::TRIANGLE; break;
                case WaveformName::TRIANGLE: newWaveform = WaveformName::SINE; break;
                case WaveformName::SINE: newWaveform = WaveformName::SQUARE; break;
                case WaveformName::SQUARE: newWaveform = WaveformName::SAWTOOTH; break;
                }
            } else {
                switch (AudioState.waveformType) {
                case WaveformName::SAWTOOTH: newWaveform = WaveformName::SQUARE; break;
                case WaveformName::TRIANGLE: newWaveform = WaveformName::SAWTOOTH; break;
                case WaveformName::SINE: newWaveform = WaveformName::TRIANGLE; break;
                case WaveformName::SQUARE: newWaveform = WaveformName::SINE; break;
                }
            }
            GenericEvent e = { EventType::SETTINGS_UPDATE, (uint8_t)SettingName::WAVEFORM, (uint8_t)newWaveform };
            xQueueSendToBack(eventQueue, &e, portMAX_DELAY);
        }

        if (KeyboardFormation.othersPresent) {
            uint8_t canRxMsg[8];
            uint32_t canRxId;
            uint8_t canRxLength;
            while (CAN_CheckRXLevel()) {
                CAN_RX(canRxId, canRxMsg, canRxLength);
            }
        }
    }
}

void processEventsTask(void *pvParameters)
{
    while (true) {
        GenericEvent event;
        xQueueReceive(eventQueue, &event, portMAX_DELAY);

        if (event.type == EventType::NOTE_CHANGE) {
            bool start = (event.subtype != 0);
            uint8_t noteIdx = event.value;
            uint32_t keyFreqHz = noteFreqs.at(noteIdx);

            xSemaphoreTake(AudioState.mutex, portMAX_DELAY);
            if (start) {
                AudioState.playingNotes.push_back(
                    { noteIdx, noteNames.at(noteIdx % 12), Phasor(keyFreqHz, AUDIO_SAMPLE_RATE_HZ) });
            } else {
                AudioState.playingNotes.remove_if([noteIdx](Note &note) { return note.idx == noteIdx; });
            }
            xSemaphoreGive(AudioState.mutex);
        } else if (event.type == EventType::SETTINGS_UPDATE) {
            switch ((SettingName)event.subtype) {
            case SettingName::VOLUME: AudioState.volumeLevel.set(event.value); break;
            case SettingName::WAVEFORM: AudioState.waveformType = (WaveformName)event.value; break;
            default: break;
            }
        }
    }
}

void displayUpdateTask(void *pvParameters)
{
    const static std::unordered_map<WaveformName, Bitmap> waveformIcons = { { WaveformName::SQUARE, squareIcon },
        { WaveformName::SAWTOOTH, sawtoothIcon }, { WaveformName::TRIANGLE, triangleIcon },
        { WaveformName::SINE, sineIcon } };

    U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

    setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
    delay(1);
    setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset

    delay(KeyboardFormation.ownIndex * 10); // Stagger power-on to avoid big voltage dip -> reset
    setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

    u8g2.begin();
    u8g2.setContrast(0);

    const auto xInterval = 100 / portTICK_PERIOD_MS;
    auto xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        // Copy over current playing notes to avoid hogging mutex while making string
        xSemaphoreTake(AudioState.mutex, portMAX_DELAY);
        const auto playingNotes = AudioState.playingNotes;
        const auto waveformType = AudioState.waveformType;
        const auto volume = AudioState.volumeLevel.get();
        xSemaphoreGive(AudioState.mutex);

        u8g2.clearBuffer();
        u8g2.enableUTF8Print(); // For the arrows
        u8g2.setFontMode(1); // Transparent text on; for the occtave indicator

        uint8_t savedColor; // For saving old draw color when temporarily changing color

        // Draw arrows pointing at knobs
        u8g2.setFont(u8g2_font_siji_t_6x10);
        u8g2.setCursor(-2, 29);
        u8g2.printf("\ue109");
        u8g2.setCursor(118, 29);
        u8g2.printf("\ue109");

        // Draw octave indicator
        u8g2.drawBox(0, 0, 9, 10);
        savedColor = u8g2.getDrawColor();
        u8g2.setFont(u8g2_font_questgiver_tr);
        u8g2.setCursor(2, 9);
        u8g2.setDrawColor(2);
        u8g2.printf("%u", KeyboardFormation.ownIndex + 1); // 1-based for humans
        u8g2.setDrawColor(savedColor);

        // Draw volume bar
        u8g2.setCursor(11, 29);
        u8g2.printf("Vol");
        u8g2.drawFrame(30, 21, 20, 8);
        u8g2.drawBox(32, 23, 2 * (volume + 1), 4);

        // Draw waveform type icon
        u8g2.setCursor(90, 29);
        u8g2.printf("Wvfm");
        auto &icon = waveformIcons.at(waveformType);
        savedColor = u8g2.getDrawColor();
        u8g2.setDrawColor(0);
        u8g2.drawXBM(62, 21, icon.width, icon.height, icon.bits);
        u8g2.setDrawColor(savedColor);

        // Construct & draw string with names of currently playing notes
        std::string playingNoteNames = "";
        for (auto &note: playingNotes) {
            if (!playingNoteNames.empty()) {
                playingNoteNames.append(",");
            }
            playingNoteNames.append(note.name);
        }
        u8g2.setCursor(13, 9);
        u8g2.print(playingNoteNames.c_str());

        // Flush framebuffer to SSD1305
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

        xSemaphoreTake(AudioState.mutex, portMAX_DELAY);
        if (!AudioState.playingNotes.empty()) {
            for (int i = 0; i < batchSize; i++) {
                for (auto &note: AudioState.playingNotes) {
                    sample_s20[i] += WaveformGenerators.at(AudioState.waveformType)(note.phase.next());
                }
            }
            generatedAny = true;
        }
        xSemaphoreGive(AudioState.mutex);

        if (generatedAny) {
            uint16_t sample_u16[batchSize];
            for (int i = 0; i < batchSize; i++) {
                auto sample = (sample_s20[i] >> (7 - AudioState.volumeLevel.get())) - INT16_MIN;
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

    // Initialise debug UART
    Serial.begin(115200);

    // Initialise CAN
    CAN_Init(false);
    setCANFilter(CAN_ID_BASE, 0x7E0);
    CAN_Start();

    // Detect attached keyboards
    setOutMuxBit(HKOW_BIT, HIGH);
    setOutMuxBit(HKOE_BIT, HIGH);

    // Allow enough time for all keyboards to assert their HK lines
    delay(100);

    bool westConnected = !BitUtils::bitIsSet(selectAndReadRow(5), 3);
    bool eastConnected = !BitUtils::bitIsSet(selectAndReadRow(6), 3);

    if (!westConnected && !eastConnected) {
        Serial.println("No other keyboards connected");
        KeyboardFormation.othersPresent = false;
        KeyboardFormation.ownIndex = 0;
    } else {
        KeyboardFormation.othersPresent = true;

        bool thisAnnounced = false;
        bool allDone = false;

        if (!westConnected) {
            // This is the westmost keyboard; initiate enumeration
            KeyboardFormation.ownIndex = 0;
            KeyboardFormation.isWestMost = true;
            KeyboardFormation.isEastMost = false;
            ownCANID = CAN_ID_BASE;

            delay(100); // Allow enough time for all keyboards to sample their initial HK inputs
            setOutMuxBit(HKOE_BIT, LOW);
            auto msg = MessageFormatter::enumerationPositionAndID(KeyboardFormation.ownIndex, UniqueID::calculate());
            CAN_TX(ownCANID, msg.data(), msg.size());

            thisAnnounced = true;
        } else {
            KeyboardFormation.isWestMost = false;
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
                KeyboardFormation.ownIndex = prevIndex + 1;
                ownCANID = CAN_ID_BASE + KeyboardFormation.ownIndex;

                setOutMuxBit(HKOE_BIT, LOW);
                auto msg =
                    MessageFormatter::enumerationPositionAndID(KeyboardFormation.ownIndex, UniqueID::calculate());
                CAN_TX(ownCANID, msg.data(), msg.size());

                thisAnnounced = true;

                if (!eastConnected) {
                    // This is the eastmost keyboard; also send "done" message
                    KeyboardFormation.isEastMost = true;

                    auto msg = MessageFormatter::enumerationDone();
                    CAN_TX(ownCANID, msg.data(), msg.size());

                    allDone = true;
                } else {
                    KeyboardFormation.isEastMost = false;
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
    AudioState.mutex = xSemaphoreCreateMutex();
    AudioState.volumeLevel.set(7);
    AudioState.waveformType = WaveformName::SAWTOOTH;

    analogWriteResolution(16 /* bits */);
    audioSampleClock.setup(TIM6);
    audioSampleClock.setOverflow(AUDIO_SAMPLE_RATE_HZ, HERTZ_FORMAT);
    audioSampleClock.attachInterrupt(isrOutputAudioSample);
    audioSampleClock.resume();

    // Set up task handles
    TaskHandle_t scanKeysHandle = nullptr;
    xTaskCreate(scanKeysTask, "scanKeys", 1024, nullptr, 2, &scanKeysHandle);

    TaskHandle_t processEventsHandle = nullptr;
    xTaskCreate(processEventsTask, "processEvents", 1024, nullptr, 2, &processEventsHandle);
    eventQueue = xQueueCreate(10, sizeof(GenericEvent));

    TaskHandle_t displayUpdateHandle = nullptr;
    xTaskCreate(displayUpdateTask, "displayUpdate", 1024, nullptr, 1, &displayUpdateHandle);

    TaskHandle_t generateSamplesTaskHandle = nullptr;
    xTaskCreate(generateSamplesTask, "generateSamples", 1024, nullptr, 1, &generateSamplesTaskHandle);
    generatedSamples = xStreamBufferCreate(AUDIO_BUFFER_SIZE * sizeof(uint16_t), 1);

    vTaskStartScheduler();
}

void loop() {}
