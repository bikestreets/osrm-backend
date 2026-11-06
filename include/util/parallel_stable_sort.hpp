#ifndef OSRM_UTIL_PARALLEL_STABLE_SORT_HPP
#define OSRM_UTIL_PARALLEL_STABLE_SORT_HPP

#include <tbb/parallel_sort.h>

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace osrm::util
{

// tbb::parallel_sort, but elements that compare equal keep their input order, so the result
// doesn't depend on the thread count or timing. tbb::parallel_sort isn't stable and splits
// its work by thread timing, so with ties in `less` its output varied from run to run.
//
// Ties break on the input position, carried in a copy of the elements while they are sorted.
// Already sorted input, the common case for some callers, is returned as is without the copy.
template <typename T, typename Less> void parallelStableSort(std::vector<T> &values, Less less)
{
    if (std::is_sorted(values.begin(), values.end(), less))
        return;

    struct Positioned
    {
        T value;
        std::size_t position;
    };
    std::vector<Positioned> positioned;
    positioned.reserve(values.size());
    for (std::size_t position = 0; position < values.size(); ++position)
        positioned.push_back({std::move(values[position]), position});

    tbb::parallel_sort(positioned.begin(),
                       positioned.end(),
                       [&less](const Positioned &lhs, const Positioned &rhs)
                       {
                           if (less(lhs.value, rhs.value))
                               return true;
                           if (less(rhs.value, lhs.value))
                               return false;
                           return lhs.position < rhs.position;
                       });

    for (std::size_t position = 0; position < values.size(); ++position)
        values[position] = std::move(positioned[position].value);
}

template <typename T> void parallelStableSort(std::vector<T> &values)
{
    parallelStableSort(values, [](const T &lhs, const T &rhs) { return lhs < rhs; });
}

} // namespace osrm::util

#endif // OSRM_UTIL_PARALLEL_STABLE_SORT_HPP
