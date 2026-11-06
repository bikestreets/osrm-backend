#ifndef OSRM_EXTRACTOR_TURN_CLASS_VARIANTS_HPP
#define OSRM_EXTRACTOR_TURN_CLASS_VARIANTS_HPP

#include "extractor/extraction_turn.hpp"
#include "extractor/scripting_environment.hpp"

#include "util/typedefs.hpp"

#include <boost/numeric/conversion/cast.hpp>

#include <array>
#include <cstdint>

namespace osrm::extractor
{

// Turn class variants (Bike Streets): with the profile property
// `highway_turn_classification_variant = <class>`, osrm-extract evaluates `process_turn` for
// every turn a second time per combination of
//
//     {as built, <class>} for the source x {as built, <class>} for the target
//
// highway turn classification, and writes the results to `.osrm.turn_variants`, aligned 1:1
// with `.osrm.turn_penalties_index` / `.osrm.turn_weight_penalties`. Nothing else about the turn
// changes. A caller that later re-classifies some edges (e.g. applying a network selection to
// an already built graph with an osrm-customize turn file) can then copy the turn penalty a
// build with those classes would have stored, without re-implementing the profile. The file
// is build-only: osrm-customize, osrm-datastore and osrm-routed never open it.
//
// Combination index: `source_is_variant + 2 * target_is_variant`, so 0 is the as-built turn.
//
// Limits, by design:
// - Only the source and target edges' classes change (also in `source_road` / `target_road`,
//   the same two edges). The other roads at the intersection (`roads_on_the_left/right`) keep
//   their as-built classes: a variant can't know which of them a caller would re-classify.
// - Class 0 means "off", so it can't be a variant class.
// - Obstacle penalties that graph_compressor folds into compressed edges come from "fake" turns
//   (ExtractionTurn's obstacle constructor), whose classes are always 0 whatever the edges are.
//   They are not re-evaluated, and they don't depend on the edges' classes in any build.
// - With `properties.turn_variants_label = "<text>"` the text is stored with the variants, so a
//   reader can check the file came from the run it expects (e.g. not a stale file beside a
//   rebuilt graph).
static constexpr std::size_t NUM_TURN_CLASS_VARIANTS = 4;

struct TurnClassVariantPenalties
{
    std::array<TurnPenalty, NUM_TURN_CLASS_VARIANTS> weight;
    std::array<TurnPenalty, NUM_TURN_CLASS_VARIANTS> duration;
};

// The conversion every turn penalty goes through before it is stored. Shared by the as-built
// turn and its variants so a variant column holds exactly what a build with those classes
// stores, truncation included (`numeric_cast` truncates towards zero).
// Turn penalties are limited to [-2^15, 2^15) which roughly translates to 54 minutes and fits
// signed 16bit deci-seconds.
inline TurnPenalty toTurnWeightPenalty(const ExtractionTurn &turn, const double weight_multiplier)
{
    return TurnPenalty{
        boost::numeric_cast<TurnPenalty::value_type>(turn.weight * weight_multiplier)};
}

inline TurnPenalty toTurnDurationPenalty(const ExtractionTurn &turn)
{
    return TurnPenalty{boost::numeric_cast<TurnPenalty::value_type>(turn.duration * 10.)};
}

// `processed` is the as-built turn, already passed through ProcessTurn; its values are the
// as-built column, so that column is the stored penalty by construction. The other
// combinations re-run ProcessTurn on a copy with only the classifications replaced (they are
// read-only to the profile), except where a side already has the variant class: then the
// combination is the same classes as an earlier one and copies it.
inline TurnClassVariantPenalties
evaluateTurnClassVariants(ScriptingEnvironment &scripting_environment,
                          const ExtractionTurn &processed,
                          const int variant_classification,
                          const double weight_multiplier)
{
    TurnClassVariantPenalties result;
    result.weight[0] = toTurnWeightPenalty(processed, weight_multiplier);
    result.duration[0] = toTurnDurationPenalty(processed);
    const bool source_is_variant =
        processed.source_highway_turn_classification == variant_classification;
    const bool target_is_variant =
        processed.target_highway_turn_classification == variant_classification;
    for (std::size_t combination = 1; combination < NUM_TURN_CLASS_VARIANTS; ++combination)
    {
        std::size_t same = combination;
        if ((same & 1) && source_is_variant)
            same &= ~std::size_t{1};
        if ((same & 2) && target_is_variant)
            same &= ~std::size_t{2};
        if (same != combination)
        {
            result.weight[combination] = result.weight[same];
            result.duration[combination] = result.duration[same];
            continue;
        }
        const int source_class = (combination & 1) ? variant_classification
                                                   : processed.source_highway_turn_classification;
        const int target_class = (combination & 2) ? variant_classification
                                                   : processed.target_highway_turn_classification;
        ExtractionTurn variant{processed, source_class, target_class};
        scripting_environment.ProcessTurn(variant);
        result.weight[combination] = toTurnWeightPenalty(variant, weight_multiplier);
        result.duration[combination] = toTurnDurationPenalty(variant);
    }
    return result;
}

} // namespace osrm::extractor

#endif // OSRM_EXTRACTOR_TURN_CLASS_VARIANTS_HPP
