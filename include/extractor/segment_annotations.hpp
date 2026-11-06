#ifndef OSRM_EXTRACTOR_SEGMENT_ANNOTATIONS_HPP
#define OSRM_EXTRACTOR_SEGMENT_ANNOTATIONS_HPP

#include "extractor/bikestreets_type.hpp"

#include <cstdint>

namespace osrm::extractor
{

// Per-segment overrides of the Bike Streets `bikestreets` and `name_is_proper` route
// annotations (Bike Streets fork). Those two are per-edge annotations fixed at extract time;
// osrm-customize can set a segment's value from `--segment-annotation-file` so a graph whose
// on-network state is applied after the build (speed + turn files on an unsplit graph) also
// reports it. Stored in `.osrm.segment_annotations`, one byte per segment and direction,
// indexed exactly like the segment datasources of `.osrm.geometry`.
//
// Byte layout: bit 7 set = overridden, bits 0-1 = BikeStreetsType, bit 2 = name_is_proper.
// 0 (and a graph without the file) means "as built", so default responses are unchanged.
using SegmentAnnotation = std::uint8_t;

static constexpr SegmentAnnotation SEGMENT_ANNOTATION_AS_BUILT = 0;
static constexpr SegmentAnnotation SEGMENT_ANNOTATION_SET = 0x80;
static constexpr SegmentAnnotation SEGMENT_ANNOTATION_BIKESTREETS_MASK = 0x03;
static constexpr SegmentAnnotation SEGMENT_ANNOTATION_NAME_IS_PROPER = 0x04;

inline SegmentAnnotation encodeSegmentAnnotation(const BikeStreetsType bikestreets,
                                                 const bool name_is_proper)
{
    return SEGMENT_ANNOTATION_SET |
           (static_cast<SegmentAnnotation>(bikestreets) & SEGMENT_ANNOTATION_BIKESTREETS_MASK) |
           (name_is_proper ? SEGMENT_ANNOTATION_NAME_IS_PROPER : 0);
}

inline BikeStreetsType segmentBikeStreets(const SegmentAnnotation value,
                                          const BikeStreetsType as_built)
{
    return (value & SEGMENT_ANNOTATION_SET)
               ? static_cast<BikeStreetsType>(value & SEGMENT_ANNOTATION_BIKESTREETS_MASK)
               : as_built;
}

inline bool segmentNameIsProper(const SegmentAnnotation value, const bool as_built)
{
    return (value & SEGMENT_ANNOTATION_SET) ? (value & SEGMENT_ANNOTATION_NAME_IS_PROPER) != 0
                                            : as_built;
}

} // namespace osrm::extractor

#endif // OSRM_EXTRACTOR_SEGMENT_ANNOTATIONS_HPP
