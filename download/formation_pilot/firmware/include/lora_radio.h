/**
 * FormationPilot - Lora Radio Driver (Header)
 * =============================================
 * Wrapper around RadioLib for SX1278/RFM95W Lora modules.
 * Handles receiving formation data packets from the leader.
 */

#ifndef LORA_RADIO_H
#define LORA_RADIO_H

#include <Arduino.h>
#include "config.h"
#include "packet_protocol.h"

// Forward declaration for RadioLib
class SX1278;

class LoraRadio {
public:
    LoraRadio();

    /**
     * Initialize the Lora radio module.
     * Configures frequency, bandwidth, SF, coding rate, and TX power.
     * @return true if initialization succeeded
     */
    bool begin();

    /**
     * Check if a packet has been received.
     * Call this in the main loop. When a packet is received,
     * it will be parsed and stored internally.
     * @return true if a new complete packet is available
     */
    bool checkReceive();

    /**
     * Get the last received packet.
     * Only valid after checkReceive() returns true.
     */
    const ReceivedPacket& getLastPacket() const;

    /**
     * Send an ACK packet back to the leader.
     * @param follower_id This follower's ID
     * @param acked_seq Sequence number being acknowledged
     * @param status Current status byte
     * @return true if transmission succeeded
     */
    bool sendAck(uint8_t follower_id, uint8_t acked_seq, uint8_t status);

    /**
     * Get RSSI of last received packet.
     */
    int16_t getLastRSSI() const { return last_rssi_; }

    /**
     * Get SNR of last received packet.
     */
    float getLastSNR() const { return last_snr_; }

    /**
     * Get number of successfully received packets.
     */
    uint32_t getRxCount() const { return rx_count_; }

    /**
     * Get number of CRC errors.
     */
    uint32_t getRxErrors() const { return rx_errors_; }

    /**
     * Get number of transmitted packets.
     */
    uint32_t getTxCount() const { return tx_count_; }

    /**
     * Check if the radio module is initialized and ready.
     */
    bool isReady() const { return initialized_; }

    /**
     * Put the radio into continuous receive mode.
     * Called after initialization and after TX.
     */
    void startReceive();

    /**
     * Get time of last received packet in millis().
     */
    uint32_t getLastPacketTime() const { return last_packet_time_; }

private:
    SX1278* radio_;            // RadioLib SX1278 instance
    bool initialized_;
    volatile bool packet_available_;

    // Last received packet data
    ReceivedPacket last_packet_;
    int16_t last_rssi_;
    float last_snr_;
    uint32_t last_packet_time_;

    // TX sequence counter
    uint8_t tx_seq_;

    // Statistics
    uint32_t rx_count_;
    uint32_t rx_errors_;
    uint32_t tx_count_;

    // Raw receive buffer
    uint8_t rx_buffer_[MAX_PACKET_SIZE + 4];  // Extra space for potential overflow
    uint8_t rx_buffer_len_;

    // Interrupt flag
    static volatile bool interrupt_flag_;
    static void onInterrupt();
};

#endif // LORA_RADIO_H
