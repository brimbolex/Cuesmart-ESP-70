#ifndef DB_R70_V2_CAN_OBJECT_CODEC_H
#define DB_R70_V2_CAN_OBJECT_CODEC_H

#include "DbCanMapping.h"
#include "R70Frame.h"

namespace dbcan {
namespace r70v2 {

namespace codec {

bool decodeCanId(uint16_t canId, CanIdParts& parts);
bool decodeAdmin(uint8_t adminByte, AdminParts& parts);
void decodeObjectAddress(const Frame& frame, const AdminParts& admin, ObjectAddress& objectAddress);
bool normalizeCanFrame(const Frame& frame, RequestOrigin origin, NormalizedRequest& request);
bool normalizeTcpWrappedCanEnvelope(const TcpEnvelope& envelope, NormalizedRequest& request);
bool normalizeTcpEnvelope(const TcpEnvelope& envelope, NormalizedRequest& request);
bool encodeTcpReply(const NormalizedRequest& request,
                    uint8_t replyNode,
                    uint8_t* out,
                    size_t outCapacity,
                    uint8_t& outLength);
bool sameLogicalShape(const NormalizedRequest& left, const NormalizedRequest& right);

}  // namespace codec

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_CAN_OBJECT_CODEC_H
