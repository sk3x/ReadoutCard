/// \file ReadoutMode.cxx
/// \brief Implementation of the ReadoutMode enum and supporting functions.
///
/// \author Pascal Boeschoten (pascal.boeschoten@cern.ch)

#include "ReadoutCard/ParameterTypes/ReadoutMode.h"
#include "Utilities/Enum.h"

namespace AliceO2 {
namespace roc {
namespace {

static const auto converter = Utilities::makeEnumConverter<ReadoutMode::type>("ReadoutMode", {
  { ReadoutMode::Continuous, "CONTINUOUS" },
});

} // Anonymous namespace

std::string ReadoutMode::toString(const ReadoutMode::type& mode)
{
  return converter.toString(mode);
}

ReadoutMode::type ReadoutMode::fromString(const std::string& string)
{
  return converter.fromString(string);
}

} // namespace roc
} // namespace AliceO2

