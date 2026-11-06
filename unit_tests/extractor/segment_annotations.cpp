#include "extractor/segment_annotations.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(segment_annotations)

using namespace osrm::extractor;

BOOST_AUTO_TEST_CASE(as_built_keeps_the_edge_annotation)
{
    BOOST_CHECK(segmentBikeStreets(SEGMENT_ANNOTATION_AS_BUILT, BikeStreetsType::Path) ==
                BikeStreetsType::Path);
    BOOST_CHECK(segmentNameIsProper(SEGMENT_ANNOTATION_AS_BUILT, true));
    BOOST_CHECK(!segmentNameIsProper(SEGMENT_ANNOTATION_AS_BUILT, false));
}

BOOST_AUTO_TEST_CASE(an_override_replaces_both_including_with_none_and_false)
{
    for (const auto type : {BikeStreetsType::None,
                            BikeStreetsType::Street,
                            BikeStreetsType::Path,
                            BikeStreetsType::Sidewalk})
    {
        for (const bool name_is_proper : {false, true})
        {
            const auto value = encodeSegmentAnnotation(type, name_is_proper);
            // Never "as built", even for (None, false): an override to off must stick.
            BOOST_CHECK(value != SEGMENT_ANNOTATION_AS_BUILT);
            BOOST_CHECK(segmentBikeStreets(value, BikeStreetsType::Sidewalk) == type);
            BOOST_CHECK_EQUAL(segmentNameIsProper(value, !name_is_proper), name_is_proper);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
