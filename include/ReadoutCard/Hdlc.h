#pragma once

#include <cstdint>
#include <string>
#include <iostream>
#include <exception>
#include <iomanip>

namespace AliceO2 {
namespace roc {

//------------------------------------------------------------------------------
/** HDLC exception class
 */
class HdlcException : public std::runtime_error
{
  public:
    HdlcException(const std::string& s) : std::runtime_error("ERROR : HDLC : " + s) {}
};


//------------------------------------------------------------------------------
/** Common definitions related to HDLC EC communication
 */
class HdlcEcFramePayload
{
  public:
    /** EC frame payload struct
     */
    uint8_t trid;
    uint8_t channel;
    uint8_t length;
    union {
      uint8_t command;
      uint8_t error;
    };
    uint32_t data;

    /** HDLC error field definitions, from GBT SCA manual v8.2, p18
     */
    enum error_field_t : uint8_t {
      ERR_GENERIC          = 0x01,
      ERR_INV_CHANNEL      = 0x02,
      ERR_INV_COMMAND      = 0x04,
      ERR_INV_TRID         = 0x08,
      ERR_INV_LENGTH       = 0x10,
      ERR_CHANNEL_DISABLED = 0x20,
      ERR_CHANNEL_BUSY     = 0x40,
      ERR_COMMAND_BUSYD    = 0x80
    };

    /** Print HdlcEcFramePayload objects with details
     */
    friend std::ostream& operator<< (std::ostream& out, const HdlcEcFramePayload& r) {
        out << std::setfill(' ') << std::right
            << "trid = "         << std::setw(3) << static_cast<uint32_t>(r.trid)    << ", "
            << "chan = "         << std::setw(3) << static_cast<uint32_t>(r.channel) << ", "
            << "len = "          << std::setw(3) << static_cast<uint32_t>(r.length)  << ", "
            << std::setfill('0')
            << "cmd/error = 0x"  << std::setw(2) << std::hex << static_cast<uint32_t>(r.command) << ", "
            << "data = 0x"       << std::setw(8) << std::hex << r.data
            << std::dec << std::left;
        return out;
      }

};

/** Structure with HDLC EC frame contents
 */
class HdlcEcFrame
{
  public:
    uint8_t addr;
    uint8_t ctrl;
    HdlcEcFramePayload payload;
    uint16_t fcs;
    uint16_t fcs_recalc;

    /** Supported S-frame types (controls frame ctrl field)
     */
    enum s_frame_t : uint8_t {
      SFT_CONNECT = 0x2f,
      SFT_RESET   = 0x8f,
      SFT_TEST    = 0xe3
    };

    /** Test for HDLC I-frame\n
     *  @note Only valid after 'ctrl' field was filled
     *  @return True, if I-frame
     */
    inline bool isIFrame() const { return (ctrl & 0x1) == 0x0; }

    /** Test for HDLC S-frame\n
     *  @note Only valid after 'ctrl' field was filled
     *  @return True, if S-frame
     */
    inline bool isSFrame() const { return (ctrl & 0x3) == 0x1; }

    /** Test for HDLC U-frame\n
     *  @note Only valid after 'ctrl' field was filled
     *  @return True, if U-frame
     */
    inline bool isUFrame() const { return (ctrl & 0x3) == 0x3; }

    /** Test if received and recalculated CRC fields match
     *  @note Only valid after 'fcs' and 'fcs_recalc' fields were populated
     *  @return True, if match
     */
    inline bool crcMatch() const { return fcs == fcs_recalc; }

    /** Print HdlcEcFrame objects with details
     */
    friend std::ostream& operator<< (std::ostream& out, const HdlcEcFrame& r) {
        switch(r.ctrl & 0x3) {
          case 0x0:
          case 0x2: out << "I-frame: PKT "; break;
          case 0x1: out << "S-frame: ";
            switch(r.ctrl & 0xf) {
              case 0x5: out << "RNR "; break;
              case 0x9: out << "RNR "; break;
              case 0xd: out << "SREJ"; break;
              default:  out << "??? "; break;
            }
            break;
          case 0x3: out << "U-frame: ??? "; break;
          default : out << "Unknown: ";
        }
        out << ", ";
        if (r.isIFrame())
          out << r.payload << ", ";

        out << std::setfill('0') << std::hex << std::right
            << "h_addr = 0x" << std::setw(2) << static_cast<uint32_t>(r.addr) << ", "
            << "h_ctrl = 0x" << std::setw(2) << static_cast<uint32_t>(r.ctrl) << ", "
            << "h_crc = 0x"  << std::setw(4) << static_cast<uint32_t>(r.fcs) << ", "
            << "h_crc_rc = 0x"  << std::setw(4) << static_cast<uint32_t>(r.fcs_recalc)
            << std::dec << std::setfill('0') << std::left;

        return out;
      }
};

} // namespace roc
} // namespace AliceO2
