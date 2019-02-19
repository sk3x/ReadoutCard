#pragma once

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/crc.hpp>
#include <iostream>
#include <stdexcept>
#include <bitset>

#include "ReadoutCard/Hdlc.h"

#ifdef HDLC_DEBUG
  #define hdebug(s)  std::cout << s << std::endl;
  #define hfdebug(s) std::cout << s << std::endl;
#else
  #define hdebug(...)
  #define hfdebug(...)
#endif

namespace AliceO2 {
namespace roc {

//------------------------------------------------------------------------------
/** HDLC frame compostition in software\n
 *  Class is designed to use the following underlying types
 *    as frame buffer:\n
 *    unit32_t\n
 *    uint64_t\n
 *    boost::multiprecision::uint128_t\n
 *    boost::multiprecision::uint256_t\n
 *    boost::multiprecision::uint512_t\n
 *    boost::multiprecision::uint1024_t\n
 */
template <class T>
class Hdlc
{
  protected:
    static const size_t  digits_{std::numeric_limits<T>::digits};

    static const uint8_t flag_   {0x7e}; /**< HDLC flag (marks frame start and end) */
    static const uint8_t bs_     {0x1f}; /**< HDLC bitstuffing pattern */
    static const uint8_t iff_min_{0x7f}; /**< HDLC interframe fill (minimum 7 consecutive bits) */
    static const uint8_t abort_  {0x3f}; /**< HDLC abort pattern within frame (6 consecutive bits outside flag) */

    static const uint16_t crc_poly_{0x1021};  /**< CRC16 polynomial (CCITT) */
    static const uint16_t crc_init_{0xffff};  /**< CRC16 init value */

  public:
    /** Frame construction common procedures\n
     *  Expects current data in frame buffer at [fbi-1 .. 0] and
     *   unused part [digits_-1 .. fbi] filled with zeros\n
     *  Performs bitstuffing, adds SOF/EOF flags, performs some sanity
     *    checks and adds inter-frame padding\n
     *  @param f Frame buffer
     *  @param fbi Current number of bits used in frame buffer [fbi-1 .. 0]
     *  @return Total number of bits in completed frame
     *  @throws HdlcException
     */
    static uint32_t completeFrame(T& f, uint32_t fbi);

    /** Scan buffer a for start flags\n
     *  Useful to detect if multiple frames, e.g. SREJ and regular PKT, are
     *    in the buffer
     *  @param a Buffer
     *  @return Vector of start flag positions
     */
    static std::vector<size_t> getStartFlagPositions(const T& a);

    /** Shift and pad frame\n
     *  May shift to next frame start position in case of multiple frames
     *    in buffer
     *  @param f Frame buffer
     *  @param shift_by Shifts to the right
     */
    static void padShiftFrame(T& f, size_t shift_by);

    /** Apply bitstuffing to the complete content of buffer 'a'\n
     *  A '0' is inserted after five consecutive occurrences of '1'
     *    found in the buffer
     *  @warning Requires sufficent space (worst case 20%) for '0'-insertion, as
     *    it does not protect against buffer overflow
     *  @param f Frame buffer
     *  @return Number of inserted bits
     */
    static uint32_t bitstuff(T& f);

    /** Apply inter-frame pattern according to ISO/IEC 13239:2002(4), Section 4.5.1, to unused
     *    parts of the buffer\n
     *    Generally applies 0x7f pattern for upper non-used buffer bits\n
     *    Last inter-frame fill before bit at index 'bitot-1' might have 7 to 14 consecutive '1',
     *    depending on actual number of used bits after bitstuffing
     *  @warning Intended to be used with hardware that shifts a '01111111' (LSB first) afterwards
     *  @param f Frame buffer
     *  @param bitot Number bits the frame in the buffer currently occupies [bitot-1 .. 0]
     */
    static void interFramePadding(T& f, uint32_t bitot);

    /** De-bitstuff content in buffer 'a'\n
     *  @param f Frame buffer
     *  @param remb Store for removed bits count (if != nullptr)
     *  @return Number of removed bits
     */
    static uint32_t deBitstuff(T& f, uint32_t* const remb = nullptr);

    /** Calculate CRC16 (CCITT)\n
     *  @note Would logically belong to some HdlcEc class, but keep here for practical reasons and
     *          easier testing
     *  @param v Value to calculate CRC on
     *  @param nb Number of bytes in v
     *  @return Calculated CRC16
     */
    static uint16_t
    crc16(const T& v, const uint32_t nb)
      {
        boost::crc_basic<16> crc(crc_poly_, crc_init_, 0, true, true);
        for (uint32_t i = 0; i < nb; i++)
          crc.process_byte(uint8_t((v >> (i * 8)) & 0xff));
        return crc.checksum();
      };

    static std::ostream& printBin(std::ostream& os, const T&, bool marker_8bit = true);

    Hdlc() {
      static_assert(
        std::is_same<T, uint32_t>::value ||
        std::is_same<T, uint64_t>::value ||
        std::is_same<T, boost::multiprecision::uint128_t>::value ||
        std::is_same<T, boost::multiprecision::uint256_t>::value ||
        std::is_same<T, boost::multiprecision::uint512_t>::value ||
        std::is_same<T, boost::multiprecision::uint1024_t>::value,
        "Value must be one of: uint32_t, uint64_t "\
        "boost::multiprecision::uint128_t, "\
        "boost::multiprecision::uint256_t, "\
        "boost::multiprecision::uint512_t, "\
        "boost::multiprecision::uint1024_t"
      );
    }
};


//------------------------------------------------------------------------------
template <class T>
uint32_t
Hdlc<T>::completeFrame(T& f, uint32_t fbi)
{
  uint32_t bitot(fbi + 8 + 8); // SOF + EOF
  bitot += Hdlc<T>::bitstuff(f);
  f = (T(Hdlc<T>::flag_) << (bitot - 8) | (f << 8) | Hdlc<T>::flag_);

  hfdebug("Frame (acp, fcs, bs, xof, bs)  = 0x" << std::hex << f << std::dec
          << " (bitot = " << bitot << ", bifill = " << (Hdlc<T>::digits_ - bitot) << ")")

  // All frame content, included added zeros and bitstuffing bits, must fit in frame buffer.
  //   We also want at least 8 bits at the end of the buffer as safe inter-frame separation marker
  if (bitot >= (digits_ - 8))
    throw HdlcException("Frame size exceeds available frame buffers size, unable to construct. fs(" +
                        std::to_string(bitot) + ") / fb_avail (" +
                        std::to_string(digits_ - 8) + ")");
  Hdlc<T>::interFramePadding(f, bitot);
  return bitot;
}


template <class T>
std::ostream&
Hdlc<T>::printBin(std::ostream& os, const T& t, bool marker_8bit)
{
  for (int i = digits_ - 1; i >= 0; --i) {
    os << ((t >> i) & 0x1);
    if (marker_8bit && i && (i%8 == 0))
      os << "_";
  }
  return os;
}


template <class T>
void
Hdlc<T>::padShiftFrame(T& f, size_t shift_by)
{
  T mask = T(-1) ^ (shift_by ? ((T(1) << (digits_ - shift_by)) - 1) : T(0));
  f = mask | (f >> shift_by);
}


template <class T>
std::vector<size_t>
Hdlc<T>::getStartFlagPositions(const T& a)
{
  enum marker_def_t {M_FLAG = 1, M_IFFILL};
  using marker_t = std::pair<marker_def_t, size_t>;
  std::vector<marker_t> markers;

  // Search for flags and frame abort patterns within the buffer
  //   and store position and type
  for (size_t ci = 0; ci <= digits_ - 8; ++ci) {
    if (((a >> ci) & 0xff) == iff_min_) {
      if (markers.back().first != M_IFFILL)
        markers.push_back(marker_t(M_IFFILL, ci));
    }

    if (((a >> ci) & 0xff) == flag_) {
      markers.push_back(marker_t(M_FLAG, ci));
      ci += 7;
    }
  }

  // Determine start-of-frame flag positions
  std::vector<size_t> sof_pos;
  for (size_t s = 1; s < markers.size(); s++) {
    if ((markers[s - 1].first == M_FLAG) && (markers[s].first == M_FLAG))
      sof_pos.push_back(markers[s-1].second);
  }

  return sof_pos;
}


template <class T>
uint32_t
Hdlc<T>::bitstuff(T& value)
{
  uint32_t inserted_bits(0);

  for (size_t ci = 0; ci < digits_ - 5; ++ci) {
    if (((value >> ci) & bs_) == bs_) {
      inserted_bits++;
      T mask_high = ((ci + 5 + 1) == digits_) ? T(0) : T(-1) ^ ((T(1) << (ci + 5 + 1)) - 1);
      T mask_low  = (T(1) << (ci + 5)) - 1;
      value = ((value << 1) & mask_high) | (value & mask_low);
    }
  }

  return inserted_bits;
}


template <class T>
void
Hdlc<T>::interFramePadding(T& f, uint32_t bitot)
{
  uint32_t ci;
  f |= (bitot == Hdlc<T>::digits_) ? T(0) : T(-1) ^ ((T(1) << bitot) - 1);   // Pad '1'
  for (ci = Hdlc<T>::digits_ - 1; ci >= bitot + 8 - 1; ci -= 8)              // Flip certain bits to '0'
    f &= ~(T(1) << ci);
}


template <class T>
uint32_t
Hdlc<T>::deBitstuff(T& value, uint32_t * const removed_bits)
{
  if (static_cast<uint8_t>(value & 0xff) != flag_) {
    std::stringstream sseen; sseen << std::hex << static_cast<uint32_t>(value & 0xff);
    std::stringstream ssof; ssof << std::hex << static_cast<uint32_t>(flag_);
    throw HdlcException("Illegal frame start marker detected during debitstuffing: 0x" +
                        sseen.str() + "/0x" + ssof.str());
  }

  uint32_t frame_bits(8);
  if (removed_bits != nullptr)
    *removed_bits = 0;

  for (size_t ci = 8; ci <= digits_ - 8; ci++) {
    // End marker detection
    if (((value >> ci) & 0xff) == flag_) {
      frame_bits += 8;
      break;
    }
    else if (ci == (digits_ - 8))
      throw HdlcException("Unable to detect frame end marker in buffer");

    if (((value >> ci) & abort_) == abort_) {
      throw HdlcException("Frame abort detected (bit " + std::to_string(ci + 5) + ")");
    }

    // Debitstuffing detection, if bs leftfill with '1'
    if (((value >> ci) & bs_) == bs_) {
      T mask_high = T(-1) ^ ((T(1) << (ci + 5 + 1)) - 1);
      T mask_low = (T(1) << (ci + 5)) - 1;
      value = (value & mask_low) | ((value & mask_high) >> 1) | (T(1) << (digits_ - 1));
      // std::cout << "VALUE = " << std::bitset<32>(value) << "  (" << ci << ")" << std::endl;
      ci += 4;
      if (removed_bits != nullptr)
        (*removed_bits)++;
      frame_bits += 4;
    }

    frame_bits++;
  }

  // Nullify leftfilled bits
  value &= ((T(1) << frame_bits) - 1);

  return frame_bits;
}



//------------------------------------------------------------------------------
/** HDLC EC communication with (targeted at hdlc_sw core)
 *
 *  @see Hdlc
 *  @see GBT-SCA manual v8.0 p9ff
 */
template <class T>
class HdlcSwEc : public Hdlc<T>
{
  public:
    HdlcSwEc() {
        // Smaller buffers don't make sense for HDLC SCA frame in software
        static_assert(
          std::is_same<T, boost::multiprecision::uint256_t>::value ||
          std::is_same<T, boost::multiprecision::uint512_t>::value ||
          std::is_same<T, boost::multiprecision::uint1024_t>::value,
          "Value must be one of: "\
          "boost::multiprecision::uint256_t, "\
          "boost::multiprecision::uint512_t, "\
          "boost::multiprecision::uint1024_t"
        );
      }

    /** Construct HDLC I-frame for regular SCA communication\n
     *    Contructs the basic frame consisting of addr, ctrl and payload field. Then
     *    adds CRC16, then applies bitstuffing before finally adding the SOF/EOF markers.
     *  @param dest Destination buffer
     *  @param src Frame payload
     *  @param tx_seq_cnt Tx sequence counter (enters ctrl field)
     *  @param rx_seq_cnt Rx sequence counter (enters ctrl field)
     *  @param hdlc_addr Address of the HDLC frame (optional)
     *  @return Number of bits in frame
     *  @throws HdlcException
     */
    static uint32_t iFrame(T& dest, const HdlcEcFramePayload& src,
                           const uint8_t tx_seq_cnt, const uint8_t rx_seq_cnt,
                           const uint8_t hdlc_addr = 0x0, bool add_zero_padding = false);

    /** Construct HDLC S-frame with supervisory-level commands
     *  @param dest Destination buffer
     *  @param sft  Type of the S-Frame
     *  @return Number of bits (frame + stuffed bits) in the frame
     *  @throws HdlcException
     */
    static uint32_t sFrame(T& dest, HdlcEcFrame::s_frame_t sft, const uint8_t hdlc_addr = 0x0);

    /** Deconstruct received HDLC frame and fill corresponding frame struct
     *  @param dest Destination frame struct
     *  @param src Frame buffer content
     *  @return Number of total bits in raw frame (includes bitstuffing bits)
     *  @throws HdlcException
     */
    static uint32_t deFrame(HdlcEcFrame& dest, T src);
};



//------------------------------------------------------------------------------
template <class T>
uint32_t
HdlcSwEc<T>::iFrame(T& f, const HdlcEcFramePayload& p,
                   const uint8_t tx_seq_cnt, const uint8_t rx_seq_cnt,
                   const uint8_t hdlc_addr, bool add_zero_padding)
{
  uint32_t biacp((1 + 1 + 8) * 8); // Bits of the frame address, ctrl and payload field
  uint32_t hdlc_ctrl = (((uint32_t(rx_seq_cnt) & 0x7) << 5) |
                        ((uint32_t(tx_seq_cnt) & 0x7) << 1));

  /* The following is sample output from the simulation of the gbt_sca_verilog core, captured
   *   on it's output. The frame payload was selected so that not bitstuffing has to occur, but is
   *   identical in the presented cases except for the payload length, which was varied.
   *
   *   payload = {trid=0xab, chan=0x2, len=VAR, cmd=0x21, data=0xdeadabab}
   *   len=0 -> received:      0x7eecb200000000addeabab210002ab00007e
   *   len=1 -> received:      0x7ecbcf00000000addeabab210102ab02007e
   *   len=2 -> received:      0x7ea24800000000addeabab210202ab04007e
   *   len=3 -> received:      0x7e853500000000addeabab210302ab06007e
   *   len=4 -> received:  0x7e70b8000000000000addeabab210402ab08007e
   *
   * Apparently, the cores adds a varying number of 0s for padding between payload and FCS field
   *
   * Reproduce gbt_sca_verilog behavior and reproduce frame zero padding
   *   ...looks like this is normally not needed, but keep in as option in any case
   */
  uint32_t bizp(0);
  if (add_zero_padding) {
    switch(p.length) {
      case 0:
      case 1:
      case 2:
      case 3: bizp = 32; break;
      case 4: bizp = 48; break;
      default:
        throw HdlcException("Illegal payload length value: " + std::to_string(p.length));
    }
  }

  // Rearrange data field to broken mapping in gbt_verilog_sca core
  //   Order the core sends in is [trid, channel, length, command, d1, d0, d3, d2]
  uint32_t data = ((p.data << 8) & 0xff00ff00) | ((p.data >> 8) & 0x00ff00ff);

  // Construct frame
  f = T(0) |
      (T(data)             << 48) |
      (uint64_t(p.command) << 40) |
      (uint64_t(p.length)  << 32) |
      (uint32_t(p.channel) << 24) |
      (uint32_t(p.trid)    << 16) |
      (hdlc_ctrl           <<  8) |
      hdlc_addr;
  f |= (T(Hdlc<T>::crc16(f, (biacp + bizp) / 8)) << (biacp + bizp));   // Add CRC

  hfdebug("Frame (acp, fcs)               = 0x" << std::hex << f << std::dec << " (biacp = " << biacp << ", tx_seq_cnt "
          << static_cast<uint32_t>(tx_seq_cnt) << ", rx_seq_cnt " << static_cast<uint32_t>(rx_seq_cnt) << ")")

  uint32_t bitot = Hdlc<T>::completeFrame(f, biacp + bizp + 16); // Total bits in frame before completing: biacp + bizp + 16(fcs)

  hfdebug("Frame (acp, fcs, bs, xof, pad) = 0x" << std::hex << f << std::dec << " (bitot = " << bitot << ")")

  return bitot;
}


template <class T>
uint32_t
HdlcSwEc<T>::sFrame(T& f, HdlcEcFrame::s_frame_t sft, const uint8_t hdlc_addr)
{
  uint32_t biac((1 + 1) * 8); // Bits of the frame address and ctrl field
  f = T(0) | sft << 8 | hdlc_addr;
  f |= (T(Hdlc<T>::crc16(f, biac / 8)) << biac);
  uint32_t bitot = Hdlc<T>::completeFrame(f, biac + 16);

  hfdebug("Frame (acp, fcs, bs, xof, pad) = 0x" << std::hex << f << std::dec << " (bitot = " << bitot << ")")

  return bitot;
}


template <class T>
uint32_t
HdlcSwEc<T>::deFrame(HdlcEcFrame& dest, T src)
{
  hfdebug("Frame (in)      = 0x" << std::hex << src << std::dec)

  uint32_t brm(0);
  uint32_t bif(Hdlc<T>::deBitstuff(src, &brm));

  hfdebug("Frame (in, dbs) = 0x" << std::hex << src << std::dec
              << " (" << brm << " / " << bif << ")")

  if ((bif % 8) != 0) {
    Hdlc<T>::printBin(std::cout, src);
    std::cout << std::endl;
    throw HdlcException("Frame size after debitstuffing is not multiple of 8 (" +
                          std::to_string(bif) + "bit)");
  }

  if ((static_cast<uint8_t>(src & 0xff) != Hdlc<T>::flag_) ||
      (static_cast<uint8_t>((src >> (bif - 8)) & 0xff) != Hdlc<T>::flag_))
    throw HdlcException("Unable to find frame start and end flags");

  dest.addr = static_cast<uint8_t>((src >> 8) & 0xff);
  dest.ctrl = static_cast<uint8_t>((src >> 16) & 0xff);

  // Work around the zero-padding stuff, see note in frame()
  //  -> read fcs field relative to frame end
  dest.fcs  = static_cast<uint16_t>((src >> (bif - 3 * 8)) & 0xffff);
  dest.fcs_recalc = Hdlc<T>::crc16((src >> 8), (bif - 4 * 8) / 8);

  if (dest.isIFrame()) {
    // I-frame
    //   Length and error fields are apparently switched wrt request
    //   Core gbt_sca_verilog receives in the order [trid, channel, error, length, d1, d0, d3, d2]
    dest.payload.trid    = static_cast<uint8_t>((src >> 24) & 0xff);
    dest.payload.channel = static_cast<uint8_t>((src >> 32) & 0xff);
    dest.payload.length  = static_cast<uint8_t>((src >> 48) & 0xff);
    dest.payload.error   = static_cast<uint8_t>((src >> 40) & 0xff);
    dest.payload.data    = static_cast<uint32_t>((src >> 56) & 0xffffffff);
    dest.payload.data    = ((dest.payload.data >> 8) & 0x00ff00ff) | ((dest.payload.data << 8) & 0xff00ff00);
  }
  else if (dest.isSFrame()) {}
  else {
    // Should only fire when seeing U-frames
    //   (which is ok, technically, but not expected)
    std::stringstream ss; ss << std::hex << dest.ctrl;
    throw HdlcException("Unexpected frame ctrl field detected (" + ss.str() + ")");
  }

  return bif;
}



//------------------------------------------------------------------------------
using hdlc_sw_ec_256_t = boost::multiprecision::uint256_t;
using HdlcSwEc256 = HdlcSwEc<hdlc_sw_ec_256_t>;

#undef hdebug
#undef hfdebug

} // namespace roc
} // namespace AliceO2
