#ifndef DB_R70_V2_PDF_LANE_H
#define DB_R70_V2_PDF_LANE_H

#include "DbCanMapping.h"

namespace dbcan {
namespace r70v2 {

namespace pdflane {

const MappingEntry* entries();
size_t entryCount();
const MappingEntry* findMapping(const ObjectAddress& objectAddress);

}  // namespace pdflane

}  // namespace r70v2
}  // namespace dbcan

#endif  // DB_R70_V2_PDF_LANE_H
