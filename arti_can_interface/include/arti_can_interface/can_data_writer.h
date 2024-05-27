#ifndef ARTI_CAN_INTERFACE_CAN_DATA_WRITER_H
#define ARTI_CAN_INTERFACE_CAN_DATA_WRITER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace arti_can_interface
{
class CanDataWriter
{
public:
  typedef std::vector<uint8_t> Buffer;

  explicit CanDataWriter(Buffer& buffer);
  CanDataWriter(Buffer& buffer, Buffer::iterator position);

  void writeUnsignedInt8(uint8_t value);
  void writeUnsignedInt16(uint16_t value);
  void writeUnsignedInt32(uint32_t value);

  void writeInt8(int8_t value);
  void writeInt16(int16_t value);
  void writeInt32(int32_t value);

  void writeFloatAsInt16(double value);
  void writeFloatAsInt32(double value);

  void writeBlock(Buffer::const_iterator begin, Buffer::const_iterator end);

protected:
  Buffer& buffer_;
  Buffer::iterator position_;
};
}

#endif  // ARTI_CAN_INTERFACE_CAN_DATA_WRITER_H
