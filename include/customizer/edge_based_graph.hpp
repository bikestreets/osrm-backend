#ifndef OSRM_CUSTOMIZE_EDGE_BASED_GRAPH_HPP
#define OSRM_CUSTOMIZE_EDGE_BASED_GRAPH_HPP

#include "extractor/edge_based_edge.hpp"
#include "partitioner/edge_based_graph.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "util/static_graph.hpp"
#include "util/typedefs.hpp"

#include "storage/shared_memory_ownership.hpp"

#include <boost/filesystem/path.hpp>

namespace osrm
{
namespace customizer
{

// TODO: Change to turn_id only
using EdgeBasedGraphEdgeData = partitioner::EdgeBasedGraphEdgeData;

template <typename EdgeDataT, storage::Ownership Ownership> class MultiLevelGraph;

namespace serialization
{
template <typename EdgeDataT, storage::Ownership Ownership>
void read(storage::tar::FileReader &reader,
          const std::string &name,
          MultiLevelGraph<EdgeDataT, Ownership> &graph);

template <typename EdgeDataT, storage::Ownership Ownership>
void write(storage::tar::FileWriter &writer,
           const std::string &name,
           const MultiLevelGraph<EdgeDataT, Ownership> &graph);
}

template <typename EdgeDataT, storage::Ownership Ownership>
class MultiLevelGraph : public partitioner::MultiLevelGraph<EdgeDataT, Ownership>
{
  private:
    using SuperT = partitioner::MultiLevelGraph<EdgeDataT, Ownership>;
    using SuperC = partitioner::MultiLevelGraph<partitioner::EdgeBasedGraphEdgeData,
                                                storage::Ownership::Container>;
    template <typename T> using Vector = util::ViewOrVector<T, Ownership>;

  public:
    MultiLevelGraph() = default;
    MultiLevelGraph(MultiLevelGraph &&) = default;
    MultiLevelGraph(const MultiLevelGraph &) = default;
    MultiLevelGraph &operator=(MultiLevelGraph &&) = default;
    MultiLevelGraph &operator=(const MultiLevelGraph &) = default;

    // TODO: add constructor for EdgeBasedGraphEdgeData
    MultiLevelGraph(SuperC &&graph,
                    Vector<EdgeWeight> node_weights_,
                    Vector<EdgeDuration> node_durations_)
        : node_weights(std::move(node_weights_)), node_durations(std::move(node_durations_))
    {
        std::tie(SuperT::node_array,
                 SuperT::edge_array,
                 SuperT::node_to_edge_offset,
                 SuperT::connectivity_checksum) = std::move(graph).data();
        // TODO: add EdgeArrayEntry shaving
    }

    MultiLevelGraph(Vector<typename SuperT::NodeArrayEntry> node_array_,
                    Vector<typename SuperT::EdgeArrayEntry> edge_array_,
                    Vector<typename SuperT::EdgeOffset> node_to_edge_offset_,
                    Vector<EdgeWeight> node_weights_,
                    Vector<EdgeDuration> node_durations_)
        : SuperT(std::move(node_array_), std::move(edge_array_), std::move(node_to_edge_offset_)),
          node_weights(std::move(node_weights_)), node_durations(std::move(node_durations_))
    {
        // TODO: add EdgeArrayEntry shaving
    }

    EdgeWeight GetNodeWeight(NodeID node) const { return node_weights[node]; }

    EdgeWeight GetNodeDuration(NodeID node) const { return node_durations[node]; }

    friend void
    serialization::read<EdgeDataT, Ownership>(storage::tar::FileReader &reader,
                                              const std::string &name,
                                              MultiLevelGraph<EdgeDataT, Ownership> &graph);
    friend void
    serialization::write<EdgeDataT, Ownership>(storage::tar::FileWriter &writer,
                                               const std::string &name,
                                               const MultiLevelGraph<EdgeDataT, Ownership> &graph);

  protected:
    Vector<EdgeWeight> node_weights;
    Vector<EdgeDuration> node_durations;
};

using MultiLevelEdgeBasedGraph =
    MultiLevelGraph<EdgeBasedGraphEdgeData, storage::Ownership::Container>;
using MultiLevelEdgeBasedGraphView =
    MultiLevelGraph<EdgeBasedGraphEdgeData, storage::Ownership::View>;
}
}

#endif
