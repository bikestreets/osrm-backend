#ifndef OSRM_GUIDANCE_BUFFER_LOCAL_IDS_HPP_
#define OSRM_GUIDANCE_BUFFER_LOCAL_IDS_HPP_

#include "extractor/turn_lane_types.hpp"
#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/guidance/turn_lanes.hpp"
#include "util/typedefs.hpp"

#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace osrm::guidance
{

// Guidance numbers its classes and lane data first-seen. Its pipeline processes buffers of
// intersections in parallel, so taking those ids from shared maps in the parallel stage
// numbered them in whatever order the threads got there, and .icd, .tld, .tls and the ids in
// every turn of .edges changed from run to run. Instead each buffer hands out its own ids,
// in its own first-seen order, starting at `first_id`. The serial, in-order output stage
// then gives each of the buffer's keys a shared id, in that order, which is the id a
// single-threaded run gives it.
template <typename Key, typename ID> class BufferLocalIDs
{
  public:
    explicit BufferLocalIDs(const ID first_id = 0) : first_id(first_id) {}

    ID FindOrAdd(const Key &key)
    {
        const auto [it, inserted] = ids.try_emplace(key, first_id);
        if (inserted)
        {
            // the maximum is every one of these id types' INVALID value
            if (static_cast<std::size_t>(first_id) + keys.size() >= std::numeric_limits<ID>::max())
            {
                throw util::exception("Too many distinct guidance ids in one buffer: " +
                                      std::to_string(keys.size()) + SOURCE_REF);
            }
            it->second = static_cast<ID>(first_id + keys.size());
            keys.push_back(key);
        }
        return it->second;
    }

    bool IsLocal(const ID id) const
    {
        return id != std::numeric_limits<ID>::max() && id >= first_id;
    }
    std::size_t Index(const ID id) const { return id - first_id; }

    // By local id, less first_id.
    const std::vector<Key> &Keys() const { return keys; }

  private:
    ID first_id;
    std::unordered_map<Key, ID> ids;
    std::vector<Key> keys;
};

// Find-or-add on a util::ConcurrentIDMap's data without its locks: only the guidance output
// stage writes these maps, and nothing reads them during the pipeline.
template <typename Map, typename Key> auto findOrAddSerially(Map &map, const Key &key)
{
    using ID = typename decltype(map.data)::mapped_type;
    return map.data.try_emplace(key, static_cast<ID>(map.data.size())).first->second;
}

namespace lanes
{
// The lane description and lane data ids the turn lane handler hands out for one buffer.
// Descriptions known before guidance (those of the ways) keep their ids; descriptions it
// combines for sliproads get buffer-local ids from known.size() up, so the two never clash
// in the lane data keys that contain them.
struct BufferLaneIDs
{
    using KnownDescriptions = std::unordered_map<extractor::TurnLaneDescription, LaneDescriptionID>;

    explicit BufferLaneIDs(const KnownDescriptions &known)
        : known_descriptions(known),
          descriptions(static_cast<LaneDescriptionID>(known_descriptions.size()))
    {
    }

    LaneDescriptionID FindOrAddDescription(const extractor::TurnLaneDescription &description)
    {
        const auto known = known_descriptions.find(description);
        if (known != known_descriptions.end())
            return known->second;
        return descriptions.FindOrAdd(description);
    }

    LaneDataID FindOrAddLaneData(const util::guidance::LaneTupleIdPair &key)
    {
        return lane_data.FindOrAdd(key);
    }

    const KnownDescriptions &known_descriptions;
    BufferLocalIDs<extractor::TurnLaneDescription, LaneDescriptionID> descriptions;
    BufferLocalIDs<util::guidance::LaneTupleIdPair, LaneDataID> lane_data;
};
} // namespace lanes

} // namespace osrm::guidance

#endif // OSRM_GUIDANCE_BUFFER_LOCAL_IDS_HPP_
