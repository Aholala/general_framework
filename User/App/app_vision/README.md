# Vision USB VCP protocol

Both directions use the same fixed 12-byte little-endian frame. Angles use radians.

| Byte | Field |
|---:|---|
| 0 | Header `0xA5` |
| 1 | Header `0x5A` |
| 2 | Mode: `0` manual aim, `1` automatic aim |
| 3..6 | Pitch (`float32`, rad) |
| 7..10 | Yaw (`float32`, rad) |
| 11 | CRC-8 checksum |

CRC-8 uses polynomial `0x31`, initial value `0xFF`, and covers bytes 0 through 10.

Robot-to-vision pitch/yaw are the BMI088-derived attitude angles. Vision-to-robot pitch/yaw are target
angles. An automatic-mode target is accepted while the operator selects automatic aim; loss of
valid frames for 200 ms returns the gimbal to the manual target.
