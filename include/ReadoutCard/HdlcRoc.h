#pragma once

#include <chrono>
#include "ReadoutCard/ChannelFactory.h"
#include "ReadoutCard/Hdlc.h"

/// \file  HdlcRoc.h
/// \brief Interface to shift register-base HDLC communication core
///
/// \author Stefan Kirsch, kirsch@fias.uni-frankfurt.de
namespace AliceO2 {
namespace roc {

namespace Registers {
  constexpr uint32_t GBT_SC_BASE = 0x00f00000 / 4;     // SCA module base address

  constexpr uint16_t GBT_SC_TX_FRAME_0{32};            // HDLC Tx frame registers 39..32
  constexpr uint16_t GBT_SC_TX_CTRL{40};
  constexpr uint16_t GBT_SC_TX_STATUS{41};
  constexpr uint16_t GBT_SC_RX_FRAME_0{48};            // HDLC Rx frame registers 47..40
  constexpr uint16_t GBT_SC_RX_CTRL{56};
  constexpr uint16_t GBT_SC_RX_STATUS{57};
  constexpr uint16_t GBT_SC_MISC_CTRL{58};             // HDLC core misc control
  constexpr uint16_t GBT_SC_MUTEX{59};

} // namespace Registers

class HdlcRoc
{
  public:
    HdlcRoc(const std::string& cardId, int channel, bool requestTridAuto = true) :
        mBar(AliceO2::roc::ChannelFactory().getBar(
               Parameters::makeParameters(Parameters::cardIdFromString(cardId), channel))),
        mRequestTridAuto(requestTridAuto),
        mRequestTrid(1)
      {}

    HdlcRoc(const Parameters& params, bool requestTridAuto = true) :
        mBar(AliceO2::roc::ChannelFactory().getBar(params)),
        mRequestTridAuto(requestTridAuto),
        mRequestTrid(1)
      {}

    /// Simply send command to SCA, do not wait for reply frame
    void transmitCommand(int link, HdlcEcFramePayload& request);

    /// Execute SCA command cycle with request/response
    /// \note Implements timeout when waiting for response
    /// \note Implements maximum number of receive retries
    /// \return Number of retries (0 in case of no error)
    uint32_t executeCommand(int link, HdlcEcFramePayload& request, HdlcEcFramePayload& reply);

    /// Send supervisory-level HDLC connect frame
    void sendSvlConnect(int link) const;

    /// Send supervisory-level HDLC reset frame
    void sendSvlReset(int link) const;

    /// Send supervisory-level HDLC test frame
    void sendSvlTest(int link) const;

    /// Reset HDLC core
    void rst() const;

    /// Clear HDLC core (statistics, etc.)
    void clr() const;

    /// Initialize HDLC core
    void init() const { rst(); clr(); }

    /// Dump full core status
    void status() const { std::cout << *this << std::endl; }

    /// Dump core register content
    void dumpRegisters(std::ostream& os = std::cout) const;

  private:
    std::shared_ptr<BarInterface> mBar;

    /// Print core status
    friend std::ostream& operator<<(std::ostream& os, const HdlcRoc& h);

    // Retransmit / timeout
    static constexpr uint8_t sTransceiveMaxRetries{2};
    static constexpr std::chrono::milliseconds sTransceiveTimeout{100};

    // TX control
    static constexpr uint8_t B_TX_SWSEQ_CNT_INC {16}; // B -> Bit, F -> Field
    static constexpr uint8_t B_TX_USR_RST       { 8};
    static constexpr uint8_t B_TX_CLR           { 1};
    static constexpr uint8_t B_TX_START         { 0};
    // TX status
    static constexpr uint8_t F_TX_SWSEQ_CNT_LSB {12};
    static constexpr uint8_t F_TX_SWSEQ_CNT_WDT { 4};
    static constexpr uint8_t F_TX_SEQ_CNT_LSB   { 8};
    static constexpr uint8_t F_TX_SEQ_CNT_WDT   { 4};
    static constexpr uint8_t B_TX_DONE          { 0};
    // RX control
    static constexpr uint8_t B_RX_SWSEQ_CNT_INC {16}; // B -> Bit, F -> Field
    static constexpr uint8_t B_RX_USR_RST       { 8};
    static constexpr uint8_t B_RX_CLR           { 1};
    static constexpr uint8_t B_RX_ACK           { 0};
    // RX status
    static constexpr uint8_t F_RX_SOF_CNT_LSB   {16};
    static constexpr uint8_t F_RX_SOF_CNT_WDT   { 8};
    static constexpr uint8_t F_RX_SWSEQ_CNT_LSB {12};
    static constexpr uint8_t F_RX_SWSEQ_CNT_WDT { 4};
    static constexpr uint8_t F_RX_SEQ_CNT_LSB   { 8};
    static constexpr uint8_t F_RX_SEQ_CNT_WDT   { 4};
    static constexpr uint8_t B_ONGOING_SEQ      { 1};
    static constexpr uint8_t B_RX_DONE          { 0};
    // Misc control
    static constexpr uint8_t F_LINK_MUX_LSB     { 0};
    static constexpr uint8_t F_LINK_MUX_WDT     { 5};
    static constexpr uint8_t B_MUTEX_RST        {31};

    //----
    // Helpers for register access
    inline uint32_t read(uint32_t addr) const { return mBar->readRegister(Registers::GBT_SC_BASE + addr); }

    inline uint32_t readField(uint32_t addr, uint8_t lsb, uint8_t wdt) const
      { return (read(addr) >> lsb) & ((wdt == 32) ? 0xffffffff : ((1 << wdt) - 1)); }

    inline uint8_t readBit(uint32_t addr, uint8_t idx) const { return (read(addr) >> idx) & 0x1; }

    inline void write(uint32_t addr, uint32_t val) const
      { mBar->writeRegister(Registers::GBT_SC_BASE + addr, val); }

    inline void writeField(uint32_t addr, uint8_t lsb, uint8_t wdt, uint32_t val) const
      { mBar->modifyRegister(Registers::GBT_SC_BASE + addr, lsb, wdt, val); }

    inline void writeBit(uint32_t addr, uint8_t idx, uint8_t val) const
      { writeField(addr, idx, 1, val); }

    inline uint8_t writeBitRb(uint32_t addr, uint8_t idx, uint8_t val) const
      {
        writeBit(addr, idx, val);
        return readBit(addr, idx);
      }

    inline uint32_t writeFieldRb(uint32_t addr, uint8_t lsb, uint8_t wdt, uint32_t val) const
      {
        writeField(addr, lsb, wdt, val);
        return readField(addr, lsb, wdt);
      }

    inline uint32_t writeRb(uint32_t addr, uint32_t val) const
      {
        write(addr, val);
        return read(addr);
      }

    //----
    // String representation of HDLC payload errors
    static const std::string sHdlcPayloadErrors[];

    bool    mRequestTridAuto; /// Let class handle the transaction ID of the request automatically
    uint8_t mRequestTrid;     /// Request transaction ID

    /// Adjust HDLC frame transaction ID (if option enabled)
    void adjustRequestTrid(HdlcEcFramePayload& r)
      {
        r.trid = mRequestTridAuto ? mRequestTrid : r.trid;
        mRequestTrid = (mRequestTrid % 0xfe) + 1;
      }

    /// Assemble Tx frame buffer content, load and send frame
    void sendFrame(HdlcEcFramePayload& request);

    /// Trigger transmission of supervisory-level frame
    void sendSFrame(int link, HdlcEcFrame::s_frame_t s) const;

    /// Fetch Rx frame buffer content and decode frame
    /// \return Number of bits in received frame on success, 0 otherwise
    uint32_t getFrame(HdlcEcFrame& r) const;

    /// Get/set link multiplexer
    uint32_t linkMux(int link = -1) const
      {
        return (link == -1 ? readField(Registers::GBT_SC_MISC_CTRL, F_LINK_MUX_LSB, F_LINK_MUX_WDT) :
                             writeFieldRb(Registers::GBT_SC_MISC_CTRL, F_LINK_MUX_LSB, F_LINK_MUX_WDT, link));
      }

    //----
    /// Unlock mutex
    inline void unlock() const { write(Registers::GBT_SC_MUTEX, 0x0); } // Any write clears mutex lock

    /// Lock mutex
    //  \return True if access to mutex is granted (process may then proceed with using the core), false otherwise
    inline bool lock() const { return readBit(Registers::GBT_SC_MUTEX, 0) == 0; }

    /// Tx ready flag
    inline bool isTxReady() const { return readBit(Registers::GBT_SC_TX_STATUS, B_TX_DONE) == 1; }

    /// Rx ready flag
    inline bool isRxReady() const { return readBit(Registers::GBT_SC_RX_STATUS, B_RX_DONE) == 1; }

    /// Check for ongoing request/reply cycle
    inline bool isOngoingSequence() const { return readBit(Registers::GBT_SC_RX_STATUS, B_ONGOING_SEQ) == 1; }

    /// Initiate Tx of txShreg content
    inline void txStart() const
      {
        writeBit(Registers::GBT_SC_TX_CTRL, B_TX_START, 0x1);
        writeBit(Registers::GBT_SC_TX_CTRL, B_TX_START, 0x0);
      }

    /// Acknowledge received frame
    inline void rxAck() const
      {
        writeBit(Registers::GBT_SC_RX_CTRL, B_RX_ACK, 0x1);
        writeBit(Registers::GBT_SC_RX_CTRL, B_RX_ACK, 0x0);
      }

    ///
    inline uint8_t txSeqCnt() const
      { return static_cast<uint8_t>(readField(Registers::GBT_SC_TX_STATUS, F_TX_SEQ_CNT_LSB, F_TX_SEQ_CNT_WDT)); }

    /// Get software-managed Tx sequence counter
    inline uint8_t txSwSeqCnt() const
      { return static_cast<uint8_t>(readField(Registers::GBT_SC_TX_STATUS, F_TX_SWSEQ_CNT_LSB, F_TX_SWSEQ_CNT_WDT)); }

    /// Increment Tx sequence counter
    inline uint8_t txSwSeqCntInc() const
      {
        writeBit(Registers::GBT_SC_TX_CTRL, B_TX_SWSEQ_CNT_INC, 0x1);
        writeBit(Registers::GBT_SC_TX_CTRL, B_TX_SWSEQ_CNT_INC, 0x0);
        return txSwSeqCnt();
      }

    ///
    inline uint8_t rxSeqCnt() const
      { return static_cast<uint8_t>(readField(Registers::GBT_SC_RX_STATUS, F_RX_SEQ_CNT_LSB, F_RX_SEQ_CNT_WDT)); }

    /// Get software-managed Rx sequence counter
    inline uint8_t rxSwSeqCnt() const
      { return static_cast<uint8_t>(readField(Registers::GBT_SC_RX_STATUS, F_RX_SWSEQ_CNT_LSB, F_RX_SWSEQ_CNT_WDT)); }

    /// Increment software-managed Rx sequence counter
    inline uint8_t rxSwSeqCntInc() const
      {
        writeBit(Registers::GBT_SC_RX_CTRL, B_RX_SWSEQ_CNT_INC, 0x1);
        writeBit(Registers::GBT_SC_RX_CTRL, B_RX_SWSEQ_CNT_INC, 0x0);
        return rxSwSeqCnt();
      }
};

}  // namespace roc
}  // namespace AliceO2
