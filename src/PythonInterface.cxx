/// \file PythonInterface.cxx
/// \brief Python wrapper interface for simple channel actions
///
/// \author Pascal Boeschoten (pascal.boeschoten@cern.ch)

#include <iostream>
#include <string>
#include <boost/lexical_cast/try_lexical_convert.hpp>
#include <boost/python.hpp>
#include "Common/GuardFunction.h"
#include "ExceptionInternal.h"
#include "ReadoutCard/ChannelFactory.h"
#include "ReadoutCard/HdlcRoc.h"

namespace {
using namespace AliceO2::roc;
/// This is a Python wrapper class for a BAR channel. It only provides register read and write access.

/// Documentation for the init function (constructor)
auto sInitDocString =
R"(Initializes a BarChannel object

Args:
    card id: String containing PCI address (e.g. 42:0.0) or serial number (e.g. 12345)
    channel number: Number of the BAR channel to open)";

/// Documentation for the register read function
auto sRegisterReadDocString =
R"(Read the 32-bit value at given 32-bit aligned address

Args:
    index: 32-bit aligned address of the register
Returns:
    The 32-bit value of the register)";

/// Documentation for the register write function
auto sRegisterWriteDocString =
R"(Write a 32-bit value at given 32-bit aligned address

Args:
    index: 32-bit aligned address of the register
    value: 32-bit value to write to the register)";

/// Documentation for the register modify function
auto sRegisterModifyDocString =
R"(Modify the width# of bits value at given position of the 32-bit aligned address

Args:
    index: 32-bit aligned address of the register
    position: position to modify (0-31)
    width: number of bits to modify
    value: width bits value to write at position (masked to width if more))";

auto sHdlcEcFramePayloadInitDocString =
R"(HDLC frame payload data structure for communication with GBT SCA. See SCA documentation for details.

Members:
    trid: Transaction ID, potentially auto-filled by core (uint8_t)
    channel: SCA channel number (uint8_t)
    length: Command length (uint8_t)
    command/error: Command (request) or error code (reply) (uint8_t)
    data: Data field sent/received (uint32_t))";

auto sHdlcRocInitDocString =
R"(HDLC communcation core interface object

Args:
    card id: String containing PCI address in BDF format or card serial number
    channel number: Bar channel to open)";

auto sHdlcRocExecuteCommandDocString =
R"(Send command and execute HDLC request/reply cycle

Args:
    link: Link index to send the command to (int)
    request: Command frame payload to send (HdlcEcFramePayload)
    reply: Response frame payload that was received (HdlcEcFramePayload))";

class BarChannel
{
  public:
    BarChannel(std::string cardIdString, int channelNumber)
    {
      auto cardId = Parameters::cardIdFromString(cardIdString);
      mBarChannel = ChannelFactory().getBar(Parameters::makeParameters(cardId, channelNumber));
    }

    uint32_t read(uint32_t address)
    {
      return mBarChannel->readRegister(address / 4);
    }

    void write(uint32_t address, uint32_t value)
    {
      return mBarChannel->writeRegister(address / 4, value);
    }

    void modify(uint32_t address, int position, int width, uint32_t value)
    {
      return mBarChannel->modifyRegister(address / 4, position, width, value);
    }

  private:
    std::shared_ptr<AliceO2::roc::BarInterface> mBarChannel;
};

} // Anonymous namespace

//
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ovrLinkMux, linkMux, 0, 1)

// Note that the name given here to BOOST_PYTHON_MODULE must be the actual name of the shared object file this file is
// compiled into
BOOST_PYTHON_MODULE(libReadoutCard)
{
  using namespace boost::python;

  class_<BarChannel>("BarChannel", init<std::string, int>(sInitDocString))
      .def("register_read", &BarChannel::read, sRegisterReadDocString)
      .def("register_write", &BarChannel::write, sRegisterWriteDocString)
      .def("register_modify", &BarChannel::modify, sRegisterModifyDocString);

  class_<HdlcEcFramePayload>("HdlcEcFramePayload", init<>(sHdlcEcFramePayloadInitDocString))
    .def_readwrite("trid", &HdlcEcFramePayload::trid)
    .def_readwrite("channel", &HdlcEcFramePayload::channel)
    .def_readwrite("length", &HdlcEcFramePayload::length)
    .def_readwrite("command", &HdlcEcFramePayload::command)
    .def_readwrite("error", &HdlcEcFramePayload::error)
    .def_readwrite("data", &HdlcEcFramePayload::data);

  class_<HdlcRoc>("HdlcRoc", init<std::string, int>(sHdlcRocInitDocString))
    .def("rst", &HdlcRoc::rst, "Reset HDLC core")
    .def("clr", &HdlcRoc::clr, "Clear HDLC core (statistivs, etc)")
    .def("init", &HdlcRoc::init, "Initialize HDLC core")
    .def("status", &HdlcRoc::status, "Print HDLC core status")
    .def("sendSvlConnect", &HdlcRoc::sendSvlConnect, "Send supervisory-level connect frame. Args: link (int)")
    .def("sendSvlReset", &HdlcRoc::sendSvlReset, "Send supervisory-level reset frame. Args: link (int)")
    .def("executeCommand", &HdlcRoc::executeCommand, sHdlcRocExecuteCommandDocString);
}
