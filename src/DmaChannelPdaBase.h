/// \file DmaChannelPdaBase.h
/// \brief Definition of the DmaChannelPdaBase class.
///
/// \author Pascal Boeschoten (pascal.boeschoten@cern.ch)

#ifndef ALICEO2_SRC_READOUTCARD_DMACHANNELPDABASE_H_
#define ALICEO2_SRC_READOUTCARD_DMACHANNELPDABASE_H_

#include <boost/scoped_ptr.hpp>
#include "DmaBufferProvider/DmaBufferProviderInterface.h"
#include "DmaChannelBase.h"
#include "Pda/PdaBar.h"
#include "Pda/PdaDmaBuffer.h"
#include "ReadoutCard/DmaChannelInterface.h"
#include "ReadoutCard/Exception.h"
#include "ReadoutCard/MemoryMappedFile.h"
#include "ReadoutCard/Parameters.h"
#include "RocPciDevice.h"

namespace AliceO2 {
namespace roc {

/// Partially implements the DmaChannelInterface. It takes care of PDA-based functionality that is common to the
/// C-RORC and CRU implementations.
class DmaChannelPdaBase: public DmaChannelBase
{
  public:
    using AllowedChannels = std::set<int>;

    /// Constructor for the DmaChannel object
    /// \param parameters Parameters of the channel
    /// \param allowedChannels Channels allowed by this card type
    DmaChannelPdaBase(const Parameters& parameters, const AllowedChannels& allowedChannels);

    ~DmaChannelPdaBase();

    virtual void startDma() final override;
    virtual void stopDma() final override;
    void resetChannel(ResetLevel::type resetLevel) final override;
    virtual PciAddress getPciAddress() final override;
    virtual int getNumaNode() final override;

  protected:

    /// Maximum amount of PDA DMA buffers for channel FIFOs (1 per channel, so this also represents the max amount of
    /// channels)
    static constexpr int PDA_DMA_BUFFER_INDEX_FIFO_MAX = 100;
    /// Start of integer range for PDA DMA buffers for pages
    static constexpr int DMA_BUFFER_INDEX_PAGES_OFFSET = 1000000000;
    /// Maximum amount of PDA DMA buffers for pages per channel
    static constexpr int DMA_BUFFER_INDEX_PAGES_CHANNEL_MAX = 1000;

    static int getPdaDmaBufferIndexFifo(int channel)
    {
      assert(channel < PDA_DMA_BUFFER_INDEX_FIFO_MAX);
      return channel;
    }

    static int getPdaDmaBufferIndexPages(int channel, int bufferNumber)
    {
      assert(bufferNumber < DMA_BUFFER_INDEX_PAGES_CHANNEL_MAX);
      return DMA_BUFFER_INDEX_PAGES_OFFSET + (channel * DMA_BUFFER_INDEX_PAGES_CHANNEL_MAX) + bufferNumber;
    }

    static int pdaBufferIndexToChannelBufferIndex(int channel, int pdaIndex)
    {
      return (pdaIndex - DMA_BUFFER_INDEX_PAGES_OFFSET) - (channel * DMA_BUFFER_INDEX_PAGES_CHANNEL_MAX);
    }

    /// Namespace for describing the state of the DMA
    struct DmaState
    {
        /// The state of the DMA
        enum type
        {
          UNKNOWN = 0, STOPPED = 1, STARTED = 2
        };
    };

    /// Perform some basic checks on a superpage
    void checkSuperpage(const Superpage& superpage);

    /// Template method called by startDma() to do device-specific (CRORC, RCU...) actions
    virtual void deviceStartDma() = 0;

    /// Template method called by stopDma() to do device-specific (CRORC, RCU...) actions
    virtual void deviceStopDma() = 0;

    /// Template method called by resetChannel() to do device-specific (CRORC, RCU...) actions
    virtual void deviceResetChannel(ResetLevel::type resetLevel) = 0;

    /// Function for getting the bus address that corresponds to the user address + given offset
    uintptr_t getBusOffsetAddress(size_t offset);

    const DmaBufferProviderInterface& getBufferProvider() const
    {
      return *(mBufferProvider.get());
    }

    const RocPciDevice& getRocPciDevice() const
    {
      return *(mRocPciDevice.get());
    }

  private:
    /// Contains addresses & size of the buffer
    std::unique_ptr<DmaBufferProviderInterface> mBufferProvider;

    /// Current state of the DMA
    DmaState::type mDmaState;

    /// PDA device objects
    boost::scoped_ptr<RocPciDevice> mRocPciDevice;
};

} // namespace roc
} // namespace AliceO2

#endif // ALICEO2_SRC_READOUTCARD_DMACHANNELPDABASE_H_