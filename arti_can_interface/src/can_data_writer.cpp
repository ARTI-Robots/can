#include <arti_can_interface/can_data_writer.h>
#include <algorithm>
#include <stdexcept>

namespace arti_can_interface
{
CanDataWriter::CanDataWriter(Buffer& buffer)
  : CanDataWriter(buffer, buffer.end())
{
}

CanDataWriter::CanDataWriter(Buffer& buffer, Buffer::iterator position)
  : buffer_(buffer), position_(position)
{
}

void CanDataWriter::writeUnsignedInt8(uint8_t value)
{
  position_ = buffer_.insert(position_, value) + 1;
}

void CanDataWriter::writeUnsignedInt16(uint16_t value)
{
  writeUnsignedInt8(static_cast<uint8_t>(value >> 0));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 8));
}

void CanDataWriter::writeUnsignedInt32(uint32_t value)
{
  writeUnsignedInt8(static_cast<uint8_t>(value >> 0));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 8));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 16));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 24));
}

void CanDataWriter::writeInt8(int8_t value)
{
  writeUnsignedInt8(static_cast<uint8_t>(value));
}

void CanDataWriter::writeInt16(int16_t value)
{
  writeUnsignedInt16(static_cast<uint16_t>(value));
}

void CanDataWriter::writeInt32(int32_t value)
{
  writeUnsignedInt32(static_cast<uint32_t>(value));
}

void CanDataWriter::writeFloatAsInt16(double value)
{
  if (std::numeric_limits<int16_t>::min() <= value && value <= std::numeric_limits<int16_t>::max())
  {
    writeInt16(static_cast<int16_t>(value));
  }
  else
  {
    throw std::out_of_range("value is outside range for float16: " + std::to_string(value));
  }
}

void CanDataWriter::writeFloatAsInt32(double value)
{
  if (std::numeric_limits<int32_t>::min() <= value && value <= std::numeric_limits<int32_t>::max())
  {
    writeInt32(static_cast<int32_t>(value));
  }
  else
  {
    throw std::out_of_range("value is outside range for float32: " + std::to_string(value));
  }
}

void CanDataWriter::writeBlock(Buffer::const_iterator begin, Buffer::const_iterator end)
{
  const ptrdiff_t n = std::distance(begin, end);
  position_ = buffer_.insert(position_, begin, end) + n;
}
}
