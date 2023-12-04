//
// Created by pedro on 23.09.21.
//

#ifndef MDA_STAR_H
#define MDA_STAR_H

#include <unordered_map>

#include "../../datastructures/includes/BinaryHeapMosp.h"
#include "../../datastructures/includes/Label.h"
#include "../../datastructures/includes/MemoryPool.h"

#include "Permanents.h"
#include "SolutionsList.h"
#include "Solution.h"

struct LastProcessedPathInfo {
    size_t index{0};
    bool nclChecked{false};
};

/**
 * See https://arxiv.org/abs/2110.10978 for details about this algorithm.
 */
class MDA  {
    typedef std::vector<Label*> MinCompleteSet;

public:
    MDA(Graph& G, const std::vector<CostArray>& potential) :
            G{G},
            truncatedFront(G.nodesCount),
            potential(potential),
            lastProcessedPath(G.arcsCount, {0,false}),
            minCompleteSets(G.nodesCount) {}

    void run(Solution& solutionData) {
        auto mergeFunction =
                [](TruncatedFront& front, const CostArray& c) -> void {
                    DIM==3 ? merge_3d(front, c) : truncatedInsertionBackward(front, c);
                };

        size_t extractions{0};
        size_t iterations{0};
        size_t permanents{0};
        auto start_bda = std::chrono::high_resolution_clock::now();
        Node target = G.target;

        Pool<Label>* labelsPool = new Pool<Label>();

        std::unordered_map<Node, Label*> heapLabels;
        size_t maxHeapLabelsSize = 0;
        Label* startLabel = labelsPool->newItem();
        CostArray initialCostVector{generate(0)};
        startLabel->update(initialCostVector, G.source, INVALID_ARC, MAX_PATH);

        BinaryHeapMosp heap = BinaryHeapMosp(1);
        heap.push(startLabel);
        heapLabels[G.source] = startLabel;
        CostArray source_n_costs{generate()};
        TruncatedFront& targetFront{this->truncatedFront[this->G.target]};
        while (heap.size()) {
            //printf("%lu\n", heap.size());
            Label *minLabel = heap.pop();
            const Node n{minLabel->n};
            ++extractions;
            source_n_costs = minLabel->c;
            //printf("%lu;%u;%u;%u;%u\n", extractions-1, n, minLabel->c[0], minLabel->c[1], minLabel->c[2]);
            //printf("%lu;%u;%u;%u;%u\n", extractions-1, n, source_n_costs[0], source_n_costs[1], source_n_costs[2]);
//            minNodeInfo.efficientCosts.push_back(minLabel.c);
            TruncatedFront& currentFront{this->truncatedFront[n]};
            mergeFunction(currentFront, minLabel->c);
            ////////////////////NEW NCL
            const Neighborhood &incomingArcs{this->G.incomingArcs(n)};
            Label* nextQueuePath = labelsPool->newItem();
            assert(!nextQueuePath->valid);
            for (size_t i = 0; i < incomingArcs.size(); ++i) {
                const Arc& predArc{incomingArcs[i]};
                Node predNode = predArc.n;
                const MinCompleteSet& permanentPredPaths{this->minCompleteSets[predNode]};
                size_t iteratorIndex{this->lastProcessedPath[predArc.id].index};
                while (iteratorIndex < permanentPredPaths.size()) {
                    const Label* predLabel = permanentPredPaths[iteratorIndex];
                    CostArray newCost = add(predLabel->c, predArc.c);
                    bool dominatedAtFront = this->lastProcessedPath[predArc.id].nclChecked ?
                                            dominates(minLabel->c, newCost) : truncatedDominance(currentFront, newCost);
                    if (dominatedAtFront || truncatedDominance(targetFront, newCost)) {
                        ++iteratorIndex;
                        this->lastProcessedPath[predArc.id].nclChecked = false;
                        continue;
                    }
                    else {
                        if (!nextQueuePath->valid || lexSmaller(newCost, nextQueuePath->c)) {
                            nextQueuePath->update(newCost, n, i, iteratorIndex);
                        }
                        this->lastProcessedPath[predArc.id].nclChecked = true;
                        break;
                    }
                }
                this->lastProcessedPath[predArc.id].index = iteratorIndex;
            }
            if (nextQueuePath->valid) {
                heap.push(nextQueuePath);
                assert(nextQueuePath->n == n);
                heapLabels[n] = nextQueuePath;
            } else {
                heapLabels.erase(n);
                labelsPool->free(nextQueuePath);
            }
            ////////////////////END NEW NCL
            ++iterations;
            if (n == target) {
                //printf("Solution %u %u %u\n", minLabel->c[0], minLabel->c[1], minLabel->c[2]);
                solutions.solutions.push_back(minLabel);
                ++sols;
                continue;
            }
            const Neighborhood &outgoingArcs{this->G.outgoingArcs(n)};
            //If made permanent, the currently extracted path is the last one in the list of permanent s-n-paths.
            const size_t predPathIndex = this->minCompleteSets[n].size();

            bool expanded = false;
            CostArray costVector = source_n_costs;
            for (const Arc &a : outgoingArcs) {
                costVector = source_n_costs;
                const Node successorNode = a.n;
                addInPlace(costVector, a.c);
                if (truncatedDominance(targetFront, costVector) || truncatedDominance(truncatedFront[successorNode], costVector)) {
                    continue;
                }
                expanded = true;
                auto it = heapLabels.find(successorNode);
                //Check if there is a label for the current successor node already in the heap.
                if (it != heapLabels.end()) {
                    Label *successorLabel{it->second};
                    assert(successorLabel->n == successorNode);

                    if (lexSmaller(costVector, successorLabel->c)) {
                        expanded = true;
                        Label* l = labelsPool->newItem();
                        l->update(costVector, successorNode, a.revArcIndex, predPathIndex);
                        it->second = l;
                        heap.decreaseKey(successorLabel, l);
                    }
                } else {
                    Label* successorLabel = labelsPool->newItem();
                    successorLabel->update(costVector, successorNode, a.revArcIndex, predPathIndex);
                    heapLabels.insert(std::make_pair(successorNode, successorLabel));
                    heap.push(successorLabel);
                }
            }
            if (expanded) {
                this->minCompleteSets[n].push_back(minLabel);
                ++permanents;
            }
            else {
                labelsPool->free(minLabel);
            }
            if (heapLabels.size() > maxHeapLabelsSize) {
                maxHeapLabelsSize = heapLabels.size();
            }
        }
        auto end_bda = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration_bda = end_bda - start_bda;
        solutionData.duration = duration_bda.count();
        solutionData.permanents = permanents;
        solutionData.iterations = iterations;
        solutionData.extractions = extractions;
        solutionData.solutionsCt = solutions.solutions.size();
        solutionData.memoryConsumption = this->memory(maxHeapLabelsSize, labelsPool->size());
        solutionData.maxHeapSize = heap.maxSize();
    }

    size_t memory(size_t maxHeapSize, size_t labelsPoolSize) const {
        size_t heapTrackingSize = maxHeapSize*sizeof(Label*);
        size_t epxloredPathsSize = labelsPoolSize*sizeof(Label);
        size_t listsSize = sizeof(List)*G.arcsCount;
        //size_t frontForNclSize = permanents * DIM * sizeof(CostType);
        return heapTrackingSize + epxloredPathsSize + listsSize;
    }

private:
    Graph& G;
    std::vector<TruncatedFront> truncatedFront;
    const std::vector<CostArray>& potential;
    std::vector<LastProcessedPathInfo> lastProcessedPath;
    std::vector<MinCompleteSet> minCompleteSets;
    size_t sols{0};
    SolutionsList solutions;
};

#endif
