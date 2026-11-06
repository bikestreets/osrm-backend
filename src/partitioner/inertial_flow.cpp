#include "partitioner/inertial_flow.hpp"
#include "partitioner/bisection_graph.hpp"
#include "partitioner/bisection_graph_view.hpp"
#include "partitioner/reorder_first_last.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <mutex>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace osrm::partitioner
{
namespace
{
// Spatially ordered sources and sink ids.
// The node ids refer to nodes in the GraphView.
struct SpatialOrder
{
    std::unordered_set<NodeID> sources;
    std::unordered_set<NodeID> sinks;
};

// Creates a spatial order of n * sources "first" and n * sink "last" node ids.
// The slope determines the spatial order for sorting node coordinates.
SpatialOrder
makeSpatialOrder(const BisectionGraphView &view, const double ratio, const double slope)
{
    struct NodeWithCoordinate
    {
        NodeWithCoordinate(NodeID nid_, const util::Coordinate &coordinate_)
            : nid{nid_}, coordinate{coordinate_}
        {
        }

        NodeID nid;
        util::Coordinate coordinate;
    };

    using Embedding = std::vector<NodeWithCoordinate>;

    Embedding embedding;
    embedding.reserve(view.NumberOfNodes());

    // adress of the very first node
    const auto node_zero = &(*view.Begin());

    std::transform(view.Begin(),
                   view.End(),
                   std::back_inserter(embedding),
                   [&](const auto &node)
                   {
                       const auto node_id = static_cast<NodeID>(&node - node_zero);
                       return NodeWithCoordinate{node_id, node.coordinate};
                   });

    const auto project = [slope](const auto &each)
    {
        auto lon = static_cast<std::int32_t>(each.coordinate.lon);
        auto lat = static_cast<std::int32_t>(each.coordinate.lat);

        return slope * lon + (1. - std::fabs(slope)) * lat;
    };

    const auto spatially = [&](const auto &lhs, const auto &rhs)
    { return project(lhs) < project(rhs); };

    const std::size_t n = ratio * embedding.size();

    reorderFirstLast(embedding, n, spatially);

    SpatialOrder order;

    order.sources.reserve(n);
    order.sinks.reserve(n);

    for (auto it = begin(embedding), last = begin(embedding) + n; it != last; ++it)
        order.sources.insert(it->nid);

    for (auto it = end(embedding) - n, last = end(embedding); it != last; ++it)
        order.sinks.insert(it->nid);

    return order;
}

// Makes n cuts with different spatial orders and returns the best.
DinicMaxFlow::MinCut bestMinCut(const BisectionGraphView &view,
                                const std::size_t n,
                                const double ratio,
                                const double balance)
{
    DinicMaxFlow::MinCut best;
    best.num_edges = -1;

    const auto get_balance = [&view, balance](const auto num_nodes_source)
    {
        const auto perfect_balance = view.NumberOfNodes() / 2;
        const auto allowed_balance = balance * perfect_balance;
        const auto bigger_side =
            std::max(num_nodes_source, view.NumberOfNodes() - num_nodes_source);

        if (bigger_side > allowed_balance)
            return bigger_side / static_cast<double>(allowed_balance);
        else
            return 1.0;
    };

    auto best_balance = 1;

    const auto balance_delta = [&view](const auto num_nodes_source)
    {
        const std::int64_t difference =
            static_cast<std::int64_t>(view.NumberOfNodes()) / 2 - num_nodes_source;
        return std::abs(difference);
    };

    // The cuts are computed in parallel but folded into the best one in slope order, as a
    // single thread does. Folding each as it finished let thread timing decide between cuts of
    // equal quality (and the rule isn't a strict order, so the fold's order matters beyond
    // ties): the same graph was bisected differently run to run. A cut that finishes before
    // an earlier slope's waits in `finished` until it can be folded.
    const auto fold = [&](DinicMaxFlow::MinCut &cut)
    {
        auto cut_balance = get_balance(cut.num_nodes_source);
        if (cut.num_edges * cut_balance < best.num_edges * best_balance ||
            (cut.num_edges == best.num_edges &&
             balance_delta(cut.num_nodes_source) < balance_delta(best.num_nodes_source)))
        {
            best_balance = cut_balance;
            std::swap(best, cut);
        }
    };

    std::mutex lock;
    std::vector<std::optional<DinicMaxFlow::MinCut>> finished(n);
    std::size_t next_to_fold = 0;

    tbb::parallel_for(tbb::blocked_range<std::size_t>{0, n, 1},
                      [&](const auto &chunk)
                      {
                          for (auto round = chunk.begin(), end = chunk.end(); round != end; ++round)
                          {
                              const auto slope = -1. + round * (2. / n);

                              auto order = makeSpatialOrder(view, ratio, slope);
                              auto cut = DinicMaxFlow()(view, order.sources, order.sinks);

                              std::lock_guard<std::mutex> guard{lock};
                              finished[round] = std::move(cut);
                              while (next_to_fold < n && finished[next_to_fold])
                              {
                                  fold(*finished[next_to_fold]);
                                  finished[next_to_fold].reset();
                                  ++next_to_fold;
                              }
                          }
                      });
    BOOST_ASSERT(next_to_fold == n);

    return best;
}
} // namespace

DinicMaxFlow::MinCut computeInertialFlowCut(const BisectionGraphView &view,
                                            const std::size_t num_slopes,
                                            const double balance,
                                            const double source_sink_rate)
{
    return bestMinCut(view, num_slopes, source_sink_rate, balance);
}

} // namespace osrm::partitioner
