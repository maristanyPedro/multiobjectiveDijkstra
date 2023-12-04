#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "../../datastructures/includes/typedefs.h"
#include "../../graph/includes/graph.h"

struct Solution {

    Solution() = delete;
    Solution(const std::string& graphName, Node sourceId, Node targetId, const Graph& G) :
        source{sourceId}, target{targetId}, graphName{graphName}, nodes{G.nodesCount}, arcs{G.arcsCount} {}

    Node source{INVALID_NODE};
    Node target{INVALID_NODE};

    const std::string graphName;
    size_t nodes{0};
    size_t arcs{0};

    size_t iterations{0};
    size_t extractions{0};
    size_t permanents{0};
    size_t solutionsCt{0};
    size_t maxHeapSize{0};
    size_t memoryConsumption{0};
    double duration{0};

    void print(const std::string& algoName, const std::string& hostName) const {
        printf("%s;%s;%s;%lu;%lu;%u;%u;%lu;%lu;%lu;%.4f;%.2f;%lu\n",
               algoName.c_str(), hostName.c_str() ,this->graphName.c_str(), nodes, arcs,
               source, target,
               iterations, extractions, solutionsCt, duration, memoryConsumption/10e5, maxHeapSize);
    }
};

//template <typename >
//void printSolutions(const NodeInfoContainer& backwardExpander, bool mainOptIndex, const std::list<>) {
//    auto getIncomingArcs = this->backwardExpander.forward ?
//                           [](const Graph& graph, const Node& n) -> const Neighborhood & {return graph.outgoingArcs(n);} :
//                           [](const Graph& graph, const Node& n) -> const Neighborhood & {return graph.incomingArcs(n);};
//    for (const Label_BOA* solution : this->solutions) {
//        std::vector<std::pair<Node, Info<CostType>>> path;
//        //From this node until the target, the path follows the preprocessing path.
//        const NodeInfo& lastSearchNode = this->backwardExpander.getInfo(solution->n);
//        const NodeInfo* iterator = &lastSearchNode;
//        uint16_t permanentIndexOfSubpath = solution->permanentIndexOfSubpath;
//        ArcId lastArcId = solution->predArcId;
//        CostType c1 = solution->c1 - lastSearchNode.potential[mainOptIndex];
//        CostType c2 = solution->c2;
//        //minLabel->c2 += oppositeMinNodeInfo.preprocessingTieBreaker[mainOptIndex];
//        //printf("Node: %u with costs %u %u\n", solution->n, c1, c2);
//
//        while (iterator) {
//            path.push_back(std::make_pair(iterator->n,Info<CostType>{c1, c2}));
//            if (iterator->n == this->backwardExpander.targetNode) {
//                break;
//            }
//            if (lastArcId != INVALID_ARC) {
//                const Neighborhood& incomingArcs{getIncomingArcs(this->G, iterator->n)};
//                const Arc& a{incomingArcs[lastArcId]};
//                c1 = c1-a.c[mainOptIndex];
//                c2 = c2-a.c[!mainOptIndex];
//                //printf("Node: %u with costs %u %u\n", a.n, c1, c2);
//                iterator = &this->backwardExpander.getInfo(a.n);
//                lastArcId = iterator->incomingEfficientArcs[permanentIndexOfSubpath];
//                permanentIndexOfSubpath = iterator->pathIds[permanentIndexOfSubpath];
//            }
//        }
//        std::reverse(path.begin(), path.end());
//        //Now, we backtrack the path in the dijkstra tree that we got from the preprocessing phase.
//        c1 = solution->c1 - lastSearchNode.potential[mainOptIndex];
//        c2 = solution->c2;
//        const Arc* predArc{lastSearchNode.preprocessingParent[mainOptIndex]};
//        while (predArc != nullptr) {
//            c1 = c1+predArc->c[mainOptIndex];
//            c2 = c2+predArc->c[!mainOptIndex];
//            path.push_back(std::make_pair(predArc->n, Info<CostType>{c1,c2}));
//            //printf("Node: %u with costs %u %u\n", predArc->n, c1, c2);
//            const NodeInfo& parentNodeInfo = backwardExpander.getInfo(predArc->n);
//            predArc = parentNodeInfo.preprocessingParent[mainOptIndex];
//        }
//        for (const auto& n : path) {
//            printf("%u (%u,%u)\n", n.first, n.second[0], n.second[1]);
//        }
//        printf("\n\n");
//    }
//}

#endif