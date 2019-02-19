#include "CommandLineUtilities/AliceLowlevelFrontend/HdlcSw.h"
#include "ReadoutCard/HdlcRoc.h"

// #define HDLC_DEBUG
#ifdef HDLC_DEBUG
  #define hdlcdebug(s)  std::cout << __func__ << " : " << s << std::endl;
#else
  #define hdlcdebug(...)
#endif

namespace AliceO2 {
namespace roc {

using HdlcFrameBuffer = hdlc_sw_ec_256_t;

const std::string
HdlcRoc::sHdlcPayloadErrors[] =
{
  "Unknown/Undocumented",
  "Invalid channel request",
  "Invalid command request",
  "Invalid transaction number request",
  "Invalid request length",
  "Channel disabled",
  "Channel busy",
  "Command in treatment"
};

void HdlcRoc::rst() const
{
  writeBit(Registers::GBT_SC_TX_CTRL, B_TX_USR_RST, 0x1);
  writeBit(Registers::GBT_SC_TX_CTRL, B_TX_USR_RST, 0x0);
  writeBit(Registers::GBT_SC_RX_CTRL, B_RX_USR_RST, 0x1);
  writeBit(Registers::GBT_SC_RX_CTRL, B_RX_USR_RST, 0x0);
  unlock(); // Clear core mutex
}

void HdlcRoc::clr() const
{
  writeBit(Registers::GBT_SC_TX_CTRL, B_TX_CLR, 0x1);
  writeBit(Registers::GBT_SC_TX_CTRL, B_TX_CLR, 0x0);
  writeBit(Registers::GBT_SC_RX_CTRL, B_RX_CLR, 0x1);
  writeBit(Registers::GBT_SC_RX_CTRL, B_RX_CLR, 0x0);
}

void HdlcRoc::sendSFrame(int link, HdlcEcFrame::s_frame_t s) const
{
  HdlcFrameBuffer hf;
  HdlcSwEc256::sFrame(hf, s);

  // Acquire lock and set link multiplexer
  while (!lock()) {}
  linkMux(link);

  // Load Tx frame registers
  for (auto i = 0; i < 8; i++)
    write(Registers::GBT_SC_TX_FRAME_0 + i, static_cast<uint32_t>((hf >> i*32) & 0xffffffff));

  txStart();

  // Release lock
  unlock();
}

void HdlcRoc::sendFrame(HdlcEcFramePayload& r)
{
  adjustRequestTrid(r);

  HdlcFrameBuffer hf;
  HdlcSwEc256::iFrame(hf, r, txSwSeqCnt(), rxSwSeqCnt());

  // Load Tx frame registers
  for (auto i = 0; i < 8; i++)
    write(Registers::GBT_SC_TX_FRAME_0 + i, static_cast<uint32_t>((hf >> i*32) & 0xffffffff));

  rxAck();
  txStart();
  txSwSeqCntInc();
}

uint32_t HdlcRoc::getFrame(HdlcEcFrame& r) const
{
  // Fetch receive registers
  HdlcFrameBuffer hf(0);
  for (auto i = 0; i < 8; i++)
    hf |= (HdlcFrameBuffer(read(Registers::GBT_SC_RX_FRAME_0 + i)) << (i * 32));

  std::vector<size_t> flagPos(HdlcSwEc256::getStartFlagPositions(hf));

  // Only process first frame encountered in buffer, ignore others (there should be only one anyway)
  uint32_t bitsInFrame(0);
  if (flagPos.size()) {
    bitsInFrame = HdlcSwEc256::deFrame(r, hf);
    rxSwSeqCntInc();
  }

  return ((bitsInFrame > 16) && r.crcMatch()) ? bitsInFrame : 0;
}

// Sent SCA command
void HdlcRoc::transmitCommand(int link, HdlcEcFramePayload& request)
{
  // Acquire mutex
  while(!lock()) {}
  hdlcdebug("mutex acquired")

  // Set link multiplexer
  linkMux(link);
  hdlcdebug("link multiplexer set to " << link)

  while (!isTxReady()) {} // Don't do anything if a request/response cycle is ongoing
  sendFrame(request);
  rxAck();
  unlock();
  hdlcdebug("sent" << request << ", mutex released")
}

// Execute SCA command cycle with request/response
uint32_t HdlcRoc::executeCommand(int link, HdlcEcFramePayload& request, HdlcEcFramePayload& reply)
{
  uint8_t retries(0);
  bool retry(true);
  HdlcEcFrame f;

  // Acquire mutex
  while(!lock()) {}
  hdlcdebug("mutex acquired")

  // Set link multiplexer
  linkMux(link);
  hdlcdebug("link multiplexer set to " << link)

  while (retry) {
   try {
      // Transmit request frame
      // while (!isTxReady() || isOngoingSequence()) {} // Don't do anything if a request/response cycle is ongoing
      while (!isTxReady()) {} // Don't do anything if a request/response cycle is ongoing
      sendFrame(request);
      hdlcdebug("sent" << request << ", waiting for reply")

      // Wait for reply frame, detect Rx timeout
      std::chrono::high_resolution_clock::time_point ts = std::chrono::high_resolution_clock::now();
      while (!isRxReady()) {
        if ((std::chrono::high_resolution_clock::now() - ts) > sTransceiveTimeout)
          throw HdlcException("Rx timeout");
      }

      hdlcdebug("decoding")

      // Decode reply frame, only accept HDLC I-frames
      if (getFrame(f) && f.isIFrame()) {
        reply = f.payload;
        retry = false;
      }
    }
    catch (HdlcException& e) {
      retries++;
      hdlcdebug(e.what())

      // Number of retries exceeded, rethrow to escalate
      if (retries > sTransceiveMaxRetries) {
        unlock();
        throw;
      }
    }
  }

  unlock();
  hdlcdebug("done, mutex released")

  return static_cast<uint32_t>(retries);
}

void HdlcRoc::sendSvlConnect(int link) const { sendSFrame(link, HdlcEcFrame::SFT_CONNECT); }
void HdlcRoc::sendSvlReset(int link) const { sendSFrame(link, HdlcEcFrame::SFT_RESET); }
void HdlcRoc::sendSvlTest(int link) const { sendSFrame(link, HdlcEcFrame::SFT_TEST); }

void HdlcRoc::dumpRegisters(std::ostream& os) const
{
  os << std::hex << std::setfill('0') << std::right;
  os << "Registers\n"
     << "  txCtrl     0x" << std::setw(8) << read(Registers::GBT_SC_TX_CTRL)
     << "  | rxCtrl     0x" << std::setw(8) << read(Registers::GBT_SC_RX_CTRL) << "\n"
     << "  txStatus   0x" << std::setw(8) << read(Registers::GBT_SC_TX_STATUS)
     << "  | rxStatus   0x" << std::setw(8) << read(Registers::GBT_SC_RX_STATUS) << "\n";
  for (uint32_t i = 0; i < 8; i++)
    os << "  txData" << i << "    0x" << std::setw(8) << read(Registers::GBT_SC_TX_FRAME_0 + i)
       << "  | rxData" << i << "    0x" << std::setw(8) << read(Registers::GBT_SC_RX_FRAME_0 + i) << "\n";
  os << std::dec << std::setfill(' ');

}

std::ostream& operator<<(std::ostream& os, const HdlcRoc& h)
{
  os << "HDLC Sw core status [0x"
     << std::hex << std::setfill('0') << std::right
     << std::setw(8) << (Registers::GBT_SC_BASE * 4) << "]\n"
     << "Flags / Counters\n"
     << "  txRdy(" << std::dec << h.isTxReady() << ") "
     << "  txSeqCnt(" << static_cast<uint32_t>(h.txSeqCnt()) << ")"
     << "  txSwSeqCnt(" << static_cast<uint32_t>(h.txSwSeqCnt()) << ")\n"
     << "  rxDone(" << std::dec << h.isRxReady() << ")"
     << "  rxSeqCnt(" << static_cast<uint32_t>(h.rxSeqCnt()) << ")"
     << "  rxSwSeqCnt(" << static_cast<uint32_t>(h.rxSwSeqCnt()) << ")\n"
     << "  ongoingSequence(" << h.isOngoingSequence() << ")\n"
     << "  linkSelect(" << h.linkMux() << ")\n"
     ;
  h.dumpRegisters(os);
  return os;
}

}  // namespace roc
}  // namespace AliceO2

#undef HDLC_DEBUG
