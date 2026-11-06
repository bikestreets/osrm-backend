#include "updater/csv_source.hpp"

#include <boost/test/unit_test.hpp>

#include <tbb/info.h>
#include <tbb/task_arena.h>

#include <filesystem>
#include <fstream>
#include <string>

BOOST_AUTO_TEST_SUITE(csv_last_line)

using namespace osrm;
using namespace osrm::updater;

BOOST_AUTO_TEST_CASE(a_key_repeated_in_one_file_takes_its_last_line_at_any_thread_count)
{
    // The parser documents "largest file index, then largest line number" wins. Within one
    // file that must not depend on how the parallel sort orders equal keys.
    const auto path = std::filesystem::temp_directory_path() / "csv_last_line_test.csv";
    {
        std::ofstream csv(path);
        // 1,000 keys, each on 20 lines; the speed is the line's index, so the last line of key k
        // is line 19,000 + k.
        for (int line = 0; line < 20000; ++line)
        {
            const auto key = line % 1000;
            csv << key << "," << key + 1 << "," << line << "\n";
        }
    }
    const int threads = std::max(2, tbb::info::default_concurrency());
    for (int run = 0; run < 5; ++run)
    {
        tbb::task_arena arena(run == 0 ? 1 : threads);
        arena.execute(
            [&]
            {
                const auto lookup = csv::readSegmentValues({path.string()});
                for (int key = 0; key < 1000; ++key)
                {
                    const auto value = lookup(
                        {static_cast<std::uint64_t>(key), static_cast<std::uint64_t>(key + 1)});
                    BOOST_REQUIRE(value);
                    BOOST_CHECK_EQUAL(value->speed, 19000 + key);
                }
            });
    }
    std::filesystem::remove(path);
}

BOOST_AUTO_TEST_SUITE_END()
