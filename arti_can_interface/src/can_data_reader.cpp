#include <arti_can_interface/can_data_reader.h>
#include <algorithm>
#include <stdexcept>

namespace arti_can_interface
{
CanDataReader::CanDataReader(Buffer::const_iterator begin, Buffer::const_iterator end)
  : begin_(begin), end_(end)
{
}

bool CanDataReader::canRead() const
{
  return begin_ < end_;
}

void CanDataReader::skipBytes(ptrdiff_t n)
{
  begin_ += std::min(n, end_ - begin_);
}

uint8_t CanDataReader::readUnsignedInt8()
{
  if (begin_ < end_)
  {
    return *begin_++;
  }
  throw std::out_of_range("tried to read beyond end of buffer");
}

uint16_t CanDataReader::readUnsignedInt16()
{
  const uint16_t result
    = (static_cast<uint16_t>(readUnsignedInt8()) << 0) +
      (static_cast<uint16_t>(readUnsignedInt8()) << 8);
  return result;
}

uint32_t CanDataReader::readUnsignedInt32()
{
  const uint32_t result
    = (static_cast<uint32_t>(readUnsignedInt8()) << 0) +
      (static_cast<uint32_t>(readUnsignedInt8()) << 8) +
      (static_cast<uint32_t>(readUnsignedInt8()) << 16) +
      (static_cast<uint32_t>(readUnsignedInt8()) << 24);
  return result;
}

int8_t CanDataReader::readInt8()
{
  return static_cast<int8_t>(readUnsignedInt8());
}

int16_t CanDataReader::readInt16()
{
  return static_cast<int16_t>(readUnsignedInt16());
}

int32_t CanDataReader::readInt32()
{
  return static_cast<int32_t>(readUnsignedInt32());
}

double CanDataReader::readInt16AsFloat()
{
  return static_cast<double>(readInt16());
}

double CanDataReader::readInt32AsFloat()
{
  return static_cast<double>(readInt32());
}
}
