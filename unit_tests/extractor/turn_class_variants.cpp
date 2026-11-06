#include "extractor/turn_class_variants.hpp"
#include "extractor/extraction_turn.hpp"
#include "extractor/files.hpp"
#include "extractor/scripting_environment.hpp"

#include "util/typedefs.hpp"

#include <boost/test/unit_test.hpp>

#include <filesystem>
#include <vector>

BOOST_AUTO_TEST_SUITE(turn_class_variants)

using namespace osrm;
using namespace osrm::extractor;

namespace
{

// A profile whose turn weight encodes the classes it was called with, so a test can tell which
// combination produced which column, and whose fractional part checks the stored truncation.
class ClassEchoScriptingEnvironment final : public ScriptingEnvironment
{
  public:
    const ProfileProperties &GetProfileProperties() override
    {
        static ProfileProperties properties;
        return properties;
    }
    std::vector<std::string> GetNameSuffixList() override { return {}; }
    std::vector<std::vector<std::string>> GetExcludableClasses() override { return {}; }
    std::vector<std::string> GetClassNames() override { return {}; }
    std::vector<std::string> GetRelations() override { return {}; }
    std::vector<std::string> GetRestrictions() override { return {}; }
    void ProcessSegment(ExtractionSegment &) override {}
    void ProcessElements(const osmium::memory::Buffer &,
                         const RestrictionParser &,
                         const ManeuverOverrideRelationParser &,
                         const ExtractionRelationContainer &,
                         std::vector<std::pair<const osmium::Node &, ExtractionNode>> &,
                         std::vector<std::pair<const osmium::Way &, ExtractionWay>> &,
                         std::vector<InputTurnRestriction> &,
                         std::vector<InputManeuverOverride> &) override
    {
    }
    bool HasLocationDependentData() const override { return false; }

    void ProcessTurn(ExtractionTurn &turn) override
    {
        ++calls;
        // Everything but the classes must reach the profile unchanged.
        BOOST_CHECK_EQUAL(turn.angle, 180. - 30.);
        BOOST_CHECK_EQUAL(turn.number_of_roads, 3);
        BOOST_CHECK_EQUAL(turn.source_priority_class, RoadPriorityClass::SIDE_RESIDENTIAL);
        BOOST_CHECK_EQUAL(turn.target_priority_class, RoadPriorityClass::TERTIARY);
        BOOST_CHECK_EQUAL(turn.from, 1u);
        BOOST_CHECK_EQUAL(turn.via, 2u);
        BOOST_CHECK_EQUAL(turn.to, 3u);
        BOOST_CHECK_EQUAL(turn.roads_on_the_left.size(), 1u);
        BOOST_CHECK_EQUAL(turn.roads_on_the_left[0].highway_turn_classification, 5);
        BOOST_CHECK_EQUAL(turn.source_road.highway_turn_classification,
                          turn.source_highway_turn_classification);
        BOOST_CHECK_EQUAL(turn.target_road.highway_turn_classification,
                          turn.target_highway_turn_classification);

        turn.weight = 10 * turn.source_highway_turn_classification +
                      turn.target_highway_turn_classification + 0.19;
        // Negative, to check truncation goes towards zero as for real turns.
        turn.duration = -(turn.source_highway_turn_classification +
                          0.1 * turn.target_highway_turn_classification) -
                        0.05;
    }

    int calls = 0;
};

ExtractionTurnLeg makeLeg(int highway_turn_classification, RoadPriorityClass::Enum priority)
{
    return {
        false, false, false, 1, highway_turn_classification, 0, 15, 100., priority, false, false};
}

ExtractionTurn makeTurn(int source_class, int target_class)
{
    return {30.,
            3,
            false,
            false,
            false,
            // source
            false,
            TRAVEL_MODE_DRIVING,
            false,
            false,
            1,
            source_class,
            0,
            15,
            RoadPriorityClass::SIDE_RESIDENTIAL,
            // target
            false,
            TRAVEL_MODE_DRIVING,
            false,
            false,
            1,
            target_class,
            0,
            15,
            RoadPriorityClass::TERTIARY,
            // other
            makeLeg(source_class, RoadPriorityClass::SIDE_RESIDENTIAL),
            makeLeg(target_class, RoadPriorityClass::TERTIARY),
            {},
            {makeLeg(5, RoadPriorityClass::MAIN_RESIDENTIAL)},
            1,
            2,
            3};
}

} // namespace

BOOST_AUTO_TEST_CASE(columns_follow_source_plus_two_target)
{
    ClassEchoScriptingEnvironment environment;
    auto turn = makeTurn(0, 3);
    environment.ProcessTurn(turn);
    environment.calls = 0;

    const auto variants = evaluateTurnClassVariants(environment, turn, 2, 10.);

    // The as-built column reuses the processed turn; the other three call the profile.
    BOOST_CHECK_EQUAL(environment.calls, 3);
    // weight = 10 * src + tgt + 0.19, stored as trunc(weight * 10)
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[0]), 31);  // (0, 3): 3.19
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[1]), 231); // (2, 3): 23.19
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[2]), 21);  // (0, 2): 2.19
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[3]), 221); // (2, 2): 22.19
    // duration = -(src + tgt / 10) - 0.05, stored as trunc(duration * 10)
    BOOST_CHECK_EQUAL(from_alias<int>(variants.duration[0]), -3);  // -0.35
    BOOST_CHECK_EQUAL(from_alias<int>(variants.duration[1]), -23); // -2.35
    BOOST_CHECK_EQUAL(from_alias<int>(variants.duration[2]), -2);  // -0.25
    BOOST_CHECK_EQUAL(from_alias<int>(variants.duration[3]), -22); // -2.25

    // And the as-built column is exactly what the extractor stores for the turn itself.
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[0]),
                      from_alias<int>(toTurnWeightPenalty(turn, 10.)));
    BOOST_CHECK_EQUAL(from_alias<int>(variants.duration[0]),
                      from_alias<int>(toTurnDurationPenalty(turn)));
}

BOOST_AUTO_TEST_CASE(already_variant_class_is_not_reevaluated)
{
    ClassEchoScriptingEnvironment environment;
    auto turn = makeTurn(2, 0);
    environment.ProcessTurn(turn);
    environment.calls = 0;

    const auto variants = evaluateTurnClassVariants(environment, turn, 2, 10.);

    // Source already has the variant class: columns 1 and 3 equal 0 and 2.
    BOOST_CHECK_EQUAL(environment.calls, 1);
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[0]), 201);
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[1]), 201);
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[2]), 221);
    BOOST_CHECK_EQUAL(from_alias<int>(variants.weight[3]), 221);
}

BOOST_AUTO_TEST_CASE(variant_copy_keeps_everything_but_classes)
{
    const auto turn = makeTurn(0, 3);
    const ExtractionTurn variant{turn, 2, 6};
    BOOST_CHECK_EQUAL(variant.angle, turn.angle);
    BOOST_CHECK_EQUAL(variant.source_highway_turn_classification, 2);
    BOOST_CHECK_EQUAL(variant.target_highway_turn_classification, 6);
    BOOST_CHECK_EQUAL(variant.source_road.highway_turn_classification, 2);
    BOOST_CHECK_EQUAL(variant.target_road.highway_turn_classification, 6);
    BOOST_CHECK_EQUAL(variant.source_road.priority_class, turn.source_road.priority_class);
    BOOST_CHECK_EQUAL(variant.roads_on_the_left[0].highway_turn_classification, 5);
    BOOST_CHECK_EQUAL(variant.weight, 0.);
    BOOST_CHECK_EQUAL(variant.duration, 0.);
}

BOOST_AUTO_TEST_CASE(file_round_trip)
{
    const auto path = std::filesystem::temp_directory_path() / "turn_class_variants_test.osrm";
    const std::vector<TurnPenalty> weight{
        TurnPenalty{1}, TurnPenalty{2}, TurnPenalty{-3}, TurnPenalty{32767}};
    const std::vector<TurnPenalty> duration{
        TurnPenalty{5}, TurnPenalty{6}, TurnPenalty{7}, TurnPenalty{-32768}};
    files::writeTurnClassVariants(path, 2, 10., "run-1/7-100", weight, duration);

    std::int32_t highway_turn_classification = 0;
    double weight_multiplier = 0;
    std::string label;
    std::vector<TurnPenalty> read_weight, read_duration;
    files::readTurnClassVariants(
        path, highway_turn_classification, weight_multiplier, label, read_weight, read_duration);
    std::filesystem::remove(path);

    BOOST_CHECK_EQUAL(highway_turn_classification, 2);
    BOOST_CHECK_EQUAL(weight_multiplier, 10.);
    BOOST_CHECK_EQUAL(label, "run-1/7-100");
    BOOST_CHECK(read_weight == weight);
    BOOST_CHECK(read_duration == duration);
}

BOOST_AUTO_TEST_SUITE_END()
