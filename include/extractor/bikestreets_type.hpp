#ifndef OSRM_EXTRACTOR_BIKESTREETS_TYPE_HPP
#define OSRM_EXTRACTOR_BIKESTREETS_TYPE_HPP

#include <cstdint>
#include <string>

namespace osrm::extractor
{

// Per-edge Bike Streets classification, surfaced as the `bikestreets` route annotation.
// Mutually exclusive: an edge is either not a bike street (None), or exactly one category.
// Backed by std::uint8_t so it can be packed into a 3-bit bitfield alongside other
// NodeBasedEdgeAnnotation flags.
enum class BikeStreetsType : std::uint8_t
{
    None = 0,
    Street = 1,
    Path = 2,
    Sidewalk = 3
};

// Parse the string a Lua profile assigns to `result.bikestreets`. Unknown/empty/nil
// values fall back to None.
inline BikeStreetsType bikeStreetsTypeFromString(const char *value)
{
    if (value == nullptr)
    {
        return BikeStreetsType::None;
    }
    const std::string v(value);
    if (v == "street")
    {
        return BikeStreetsType::Street;
    }
    if (v == "path")
    {
        return BikeStreetsType::Path;
    }
    if (v == "sidewalk")
    {
        return BikeStreetsType::Sidewalk;
    }
    return BikeStreetsType::None;
}

inline const char *bikeStreetsTypeToString(const BikeStreetsType type)
{
    switch (type)
    {
    case BikeStreetsType::Street:
        return "street";
    case BikeStreetsType::Path:
        return "path";
    case BikeStreetsType::Sidewalk:
        return "sidewalk";
    case BikeStreetsType::None:
    default:
        return "none";
    }
}

} // namespace osrm::extractor

#endif // OSRM_EXTRACTOR_BIKESTREETS_TYPE_HPP
