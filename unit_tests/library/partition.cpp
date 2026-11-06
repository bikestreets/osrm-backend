#include <boost/test/unit_test.hpp>

#include "osrm/extractor.hpp"
#include "osrm/extractor_config.hpp"
#include "osrm/partitioner.hpp"
#include "osrm/partitioner_config.hpp"

#include "partitioner/multi_level_partition.hpp"
#include "storage/tar.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

BOOST_AUTO_TEST_SUITE(library_partition)

BOOST_AUTO_TEST_CASE(test_partition_with_invalid_config)
{
    using namespace osrm;

    osrm::PartitionerConfig config;
    config.requested_num_threads = std::thread::hardware_concurrency();
    BOOST_CHECK_THROW(osrm::partition(config),
                      std::exception); // including osrm::util::exception, etc.
}

namespace
{
// Extracts and partitions monaco (car.lua, which has turn lanes) into `directory` with
// `threads` threads.
std::filesystem::path extractAndPartition(const std::filesystem::path &directory,
                                          const unsigned threads)
{
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    const auto base = directory / "monaco.osrm";

    osrm::ExtractorConfig extractor_config;
    extractor_config.input_path = OSRM_TEST_DATA_DIR "/monaco.osm.pbf";
    extractor_config.UseDefaultOutputNames(base);
    extractor_config.profile_path = OSRM_TEST_DATA_DIR "/../../profiles/car.lua";
    extractor_config.small_component_size = 1000;
    extractor_config.requested_num_threads = threads;
    osrm::extract(extractor_config);

    osrm::PartitionerConfig partitioner_config;
    partitioner_config.UseDefaultOutputNames(base);
    partitioner_config.requested_num_threads = threads;
    osrm::partition(partitioner_config);

    return base;
}

std::vector<char> readFile(const std::filesystem::path &path)
{
    std::ifstream stream(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
} // namespace

// Every file osrm-extract and osrm-partition write must be the same at any thread count:
// unstable sorts, ids taken in thread order and an inertial-flow cut picked by whichever
// thread finished first made them differ run to run.
BOOST_AUTO_TEST_CASE(test_extract_and_partition_do_not_depend_on_thread_count)
{
    const auto root = std::filesystem::temp_directory_path() / "osrm-determinism-test";
    const auto single = extractAndPartition(root / "1", 1).parent_path();

    for (const unsigned threads : {2u, 8u, 8u})
    {
        const auto multi =
            extractAndPartition(root / std::to_string(threads), threads).parent_path();

        std::vector<std::string> single_files, multi_files;
        for (const auto &entry : std::filesystem::directory_iterator(single))
            single_files.push_back(entry.path().filename().string());
        for (const auto &entry : std::filesystem::directory_iterator(multi))
            multi_files.push_back(entry.path().filename().string());
        std::sort(single_files.begin(), single_files.end());
        std::sort(multi_files.begin(), multi_files.end());
        BOOST_REQUIRE_EQUAL_COLLECTIONS(
            single_files.begin(), single_files.end(), multi_files.begin(), multi_files.end());
        BOOST_REQUIRE(!single_files.empty());

        for (const auto &file : single_files)
        {
            BOOST_TEST_CONTEXT(file << " at " << threads << " threads")
            {
                BOOST_CHECK(readFile(single / file) == readFile(multi / file));
            }
        }
    }
    std::filesystem::remove_all(root);
}

// .partition stores LevelData byte for byte: its padding and the slots past num_level must be
// zero, or two identical partitions write different files.
BOOST_AUTO_TEST_CASE(test_partition_level_data_has_no_garbage)
{
    using LevelData = osrm::partitioner::MultiLevelPartition::LevelData;

    const auto root = std::filesystem::temp_directory_path() / "osrm-level-data-test";
    const auto base = extractAndPartition(root, 1);

    std::array<unsigned char, sizeof(LevelData)> bytes;
    {
        osrm::storage::tar::FileReader reader(base.string() + ".partition",
                                              osrm::storage::tar::FileReader::VerifyFingerprint);
        reader.ReadInto("/mld/multilevelpartition/level_data", bytes.data(), bytes.size());
    }
    std::filesystem::remove_all(root);

    LevelData level_data;
    std::memcpy(&level_data, bytes.data(), sizeof(LevelData));
    BOOST_REQUIRE_GT(level_data.num_level, 1);

    const auto all_zero = [&](std::size_t begin, std::size_t end)
    {
        return std::all_of(
            bytes.begin() + begin, bytes.begin() + end, [](const auto byte) { return byte == 0; });
    };

    // padding after each member
    BOOST_CHECK(all_zero(offsetof(LevelData, num_level) + sizeof(level_data.num_level),
                         offsetof(LevelData, lidx_to_offset)));
    BOOST_CHECK(all_zero(offsetof(LevelData, lidx_to_offset) + sizeof(level_data.lidx_to_offset),
                         offsetof(LevelData, lidx_to_mask)));
    BOOST_CHECK(all_zero(offsetof(LevelData, lidx_to_mask) + sizeof(level_data.lidx_to_mask),
                         offsetof(LevelData, bit_to_level)));
    BOOST_CHECK(all_zero(offsetof(LevelData, bit_to_level) + sizeof(level_data.bit_to_level),
                         offsetof(LevelData, lidx_to_children_offsets)));
    BOOST_CHECK(all_zero(offsetof(LevelData, lidx_to_children_offsets) +
                             sizeof(level_data.lidx_to_children_offsets),
                         sizeof(LevelData)));

    // Offsets have one slot per level plus the sentinel, masks one per level; the rest is unused.
    for (std::size_t lidx = level_data.num_level; lidx < level_data.lidx_to_offset.size(); ++lidx)
        BOOST_CHECK_EQUAL(level_data.lidx_to_offset[lidx], 0);
    for (std::size_t lidx = level_data.num_level - 1; lidx < level_data.lidx_to_mask.size(); ++lidx)
        BOOST_CHECK_EQUAL(level_data.lidx_to_mask[lidx], 0);
}

BOOST_AUTO_TEST_SUITE_END()
