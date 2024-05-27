#ifndef ARTI_CAN_INTERFACE_CAN_DATA_READER_H
#define ARTI_CAN_INTERFACE_CAN_DATA_READER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace arti_can_interface
{
class CanDataReader
{
public:
  typedef std::vector<uint8_t> Buffer;

  CanDataReader(Buffer::const_iterator begin, Buffer::const_iterator end);

  bool canRead() const;
  void skipBytes(ptrdiff_t n);

  uint8_t readUnsignedInt8();
  uint16_t readUnsignedInt16();
  uint32_t readUnsignedInt32();

  int8_t readInt8();
  int16_t readInt16();
  int32_t readInt32();

  double readInt16AsFloat();
  double readInt32AsFloat();

protected:
  Buffer::const_iterator begin_;
  Buffer::const_iterator end_;
};
}

#endif  // ARTI_CAN_INTERFACE_CAN_DATA_READER_H
