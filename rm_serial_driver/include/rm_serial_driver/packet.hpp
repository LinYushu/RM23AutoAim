#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  float bullet_speed;
  float yaw;
  float pitch;
  float roll;
  char enemy_color;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint32_t packat_id = 0;
  uint8_t suggest_fire = 0;
  float offset_yaw = 0 ;
  float offset_pitch = 0;
  uint16_t checksum = 0;
} __attribute__((packed));


inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
