/// \file Cru/DatapathWrapper.cxx
/// \brief Implementation of Datapath Wrapper
///
/// \author Kostas Alexopoulos (kostas.alexopoulos@cern.ch)

#include "Constants.h"
#include "DatapathWrapper.h"

namespace AliceO2 {
namespace roc {

DatapathWrapper::DatapathWrapper(std::shared_ptr<Pda::PdaBar> pdaBar) :
  mPdaBar(pdaBar)
{
}

/// Set links with a bitmask
void DatapathWrapper::setLinksEnabled(uint32_t dwrapper, uint32_t mask)
{
  uint32_t address = getDatapathWrapperBaseAddress(dwrapper) +
    Cru::Registers::DWRAPPER_GREGS.index + 
    Cru::Registers::DWRAPPER_ENREG.index;
  mPdaBar->writeRegister(address/4, mask);
}

/// Set particular link's enabled bit
void DatapathWrapper::setLinkEnabled(Link link){
  uint32_t address = getDatapathWrapperBaseAddress(link.dwrapper) +
    Cru::Registers::DWRAPPER_GREGS.address + 
    Cru::Registers::DWRAPPER_ENREG.address;
  mPdaBar->modifyRegister(address/4, link.dwrapperId, 1, 0x1);
}

/// Get particular link's enabled bit
bool DatapathWrapper::getLinkEnabled(Link link){
  uint32_t address = getDatapathWrapperBaseAddress(link.dwrapper) +
    Cru::Registers::DWRAPPER_GREGS.address + 
    Cru::Registers::DWRAPPER_ENREG.address;
  uint32_t enabled = mPdaBar->readRegister(address/4);
  return ((enabled >> link.dwrapperId) & 0x1);
}

/// Set Datapath Mode
void DatapathWrapper::setDatapathMode(Link link, uint32_t mode)
{
  uint32_t address = getDatapathWrapperBaseAddress(link.dwrapper) +
    Cru::Registers::DATAPATHLINK_OFFSET.address +
    Cru::Registers::DATALINK_OFFSET.address * link.dwrapperId +
    Cru::Registers::DATALINK_CONTROL.address;

  uint32_t val = 0;
  val |= 0x1EA;   //=RAWMAXLEN
  val |= (0<<24); //=RAWBYID
  val |= (mode << 31);
      
  mPdaBar->writeRegister(address/4, val);
}

/// Get Datapath Mode
DatapathMode::type DatapathWrapper::getDatapathMode(Link link)
{

  uint32_t address = getDatapathWrapperBaseAddress(link.dwrapper) +
    Cru::Registers::DATAPATHLINK_OFFSET.address +
    Cru::Registers::DATALINK_OFFSET.address * link.dwrapperId +
    Cru::Registers::DATALINK_CONTROL.address;

  uint32_t value = mPdaBar->readRegister(address/4); //1 = packet | 0 = continuous
  DatapathMode::type mode;
  if ((value >> 31) == 0x1) {
    mode = DatapathMode::type::Packet;
  } else {
    mode = DatapathMode::type::Continuous;
  }
  return mode;
}

/// Set Packet Arbitration
void DatapathWrapper::setPacketArbitration(int wrapperCount, int arbitrationMode)
{
  uint32_t value = 0x0;
  value |= (arbitrationMode << 15);

  for(int i=0; i<wrapperCount; i++) {
    uint32_t address = getDatapathWrapperBaseAddress(i) +
      Cru::Registers::DWRAPPER_GREGS.address +
      Cru::Registers::DWRAPPER_MUX_CONTROL.address;

    mPdaBar->writeRegister(address/4, value);
  }
} 

/// Set Flow Control 
void DatapathWrapper::setFlowControl(int wrapper, int allowReject, int forceReject)
{
  uint32_t value = 0;
  value |= (allowReject << 0);
  value |= (forceReject << 4);

  uint32_t address = getDatapathWrapperBaseAddress(wrapper) +
    Cru::Registers::FLOW_CONTROL_OFFSET.address +
    Cru::Registers::FLOW_CONTROL_REGISTER.address;

  mPdaBar->writeRegister(address, value);
}

uint32_t DatapathWrapper::getDatapathWrapperBaseAddress(int wrapper)
{
  if (wrapper == 0) {
    return Cru::Registers::DWRAPPER_BASE0.address;
  } else if (wrapper == 1) {
    return Cru::Registers::DWRAPPER_BASE1.address;
  }

  return 0x0;
}

} // namespace roc
} // namespace AliceO2
