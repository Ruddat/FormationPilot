/**
 * FormationPilot - Lora Radio Driver (Implementation)
 * =====================================================
 * RadioLib-based driver for SX1278/RFM95W Lora modules.
 */

#include "lora_radio.h"
#include "config.h"
#include <RadioLib.h>

// Static interrupt flag
volatile bool LoraRadio::interrupt_flag_ = false;

LoraRadio::LoraRadio()
    : radio_(nullptr)
    , initialized_(false)
    , packet_available_(false)
    , last_rssi_(0)
    , last_snr_(0.0f)
    , last_packet_time_(0)
    , tx_seq_(0)
    , rx_count_(0)
    , rx_errors_(0)
    , tx_count_(0)
    , rx_buffer_len_(0)
{
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

void LoraRadio::onInterrupt() {
    interrupt_flag_ = true;
}

bool LoraRadio::begin() {
    Serial.println("[Lora] Initializing SX1278...");

    // Create RadioLib SX1278 instance
    // Module(cs, irq, rst, gpio)
    radio_ = new SX1278(new Module(PIN_LORA_NSS, PIN_LORA_DIO0,
                                    PIN_LORA_RST, RADIOLIB_NC));

    // Initialize the radio with our configuration
    int16_t state = radio_->begin(
        LORA_FREQUENCY,              // Carrier frequency (MHz)
        LORA_BANDWIDTH,              // Bandwidth (kHz)
        LORA_SPREADING_FACTOR,       // Spreading factor
        LORA_CODING_RATE,            // Coding rate
        RADIOLIB_SX127X_SYNC_WORD,   // Sync word (default: 0x12)
        LORA_TX_POWER,               // TX power (dBm)
        LORA_PREAMBLE_LEN,           // Preamble length
        2                            // Gain (0 = auto)
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Lora] ERROR: Initialization failed, code %d\n", state);
        delete radio_;
        radio_ = nullptr;
        initialized_ = false;
        return false;
    }

    // Set the interrupt handler for received packets
    radio_->setDio0Action(onInterrupt, RISING);

    // Start listening
    startReceive();

    initialized_ = true;
    Serial.printf("[Lora] Ready: %.3f MHz, SF%d, BW%.0fkHz, %ddBm\n",
                  LORA_FREQUENCY, LORA_SPREADING_FACTOR,
                  LORA_BANDWIDTH, LORA_TX_POWER);

    return true;
}

void LoraRadio::startReceive() {
    if (!radio_ || !initialized_) return;

    int16_t state = radio_->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Lora] startReceive failed: %d\n", state);
    }
}

bool LoraRadio::checkReceive() {
    if (!radio_ || !initialized_) return false;

    packet_available_ = false;

    // Check if interrupt fired (packet received)
    if (!interrupt_flag_) {
        return false;
    }
    interrupt_flag_ = false;

    // Read the received data
    int16_t len = radio_->readData(rx_buffer_, MAX_PACKET_SIZE);
    if (len < 0) {
        rx_errors_++;
        // Re-start receive
        startReceive();
        return false;
    }

    rx_buffer_len_ = (uint8_t)len;

    // Get signal quality
    last_rssi_ = radio_->getRSSI();
    last_snr_  = radio_->getSNR();
    last_packet_time_ = millis();

    // Parse the raw bytes using our protocol parser
    ReceivedPacket parsed;
    if (PacketParser::parse(rx_buffer_, rx_buffer_len_, parsed)) {
        last_packet_ = parsed;
        last_packet_.rssi = last_rssi_;
        packet_available_ = true;
        rx_count_++;

        // Debug output
        Serial.printf("[Lora] RX: type=0x%02X seq=%d len=%d RSSI=%ddBm SNR=%.1fdB\n",
                      parsed.type, parsed.sequence, parsed.payload_len,
                      last_rssi_, last_snr_);
    } else {
        rx_errors_++;
        Serial.printf("[Lora] Parse error (%d bytes, RSSI=%d)\n",
                      rx_buffer_len_, last_rssi_);
    }

    // Re-start receive for next packet
    startReceive();

    return packet_available_;
}

const ReceivedPacket& LoraRadio::getLastPacket() const {
    return last_packet_;
}

bool LoraRadio::sendAck(uint8_t follower_id, uint8_t acked_seq, uint8_t status) {
    if (!radio_ || !initialized_) return false;

    // Build ACK packet
    uint8_t packet_buf[16];
    size_t pkt_len = PacketParser::buildAckPacket(
        packet_buf, follower_id, acked_seq, status, tx_seq_
    );

    // Transmit
    int16_t state = radio_->transmit(packet_buf, pkt_len);

    if (state == RADIOLIB_ERR_NONE) {
        tx_count_++;
        Serial.printf("[Lora] TX ACK: fid=%d seq=%d status=%d\n",
                      follower_id, acked_seq, status);
        // Go back to receive mode
        startReceive();
        return true;
    } else {
        Serial.printf("[Lora] TX failed: %d\n", state);
        startReceive();
        return false;
    }
}
