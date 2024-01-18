//
// Created by pedro on 23.09.21.
//

#ifndef NAMOA_H
#define NAMOA_H

#include "../../datastructures/includes/BinaryHeapMosp.h"
#include "../../datastructures/includes/Label.h"
#include "../../datastructures/includes/MemoryPool.h"

#include "Permanents.h"
#include "SolutionsList.h"
#include "Solution.h"

/**
 * See https://arxiv.org/abs/2110.10978 for details about this algorithm.
 */
class Martins  {
    typedef std::list<Label*> OpenCosts;
public:
    Martins(const Graph& G, const std::vector<CostArray>& potential) :
            G{G},
            exploredPaths(G.nodesCount),
            truncatedFront(G.nodesCount),
            targetFront{this->truncatedFront[this->G.target]},
            potential(potential),
            permanentLabels() {}

    void nextQueuePath(Node n, Pool<Label>* labelsPool, BinaryHeapMosp& heap) {
        OpenCosts& currentOpenCosts = this->exploredPaths[n];
        currentOpenCosts.pop_front();
        bool success = false;
        if (!currentOpenCosts.empty()) {
            Label* newHeapLabel{currentOpenCosts.front()};
            while (!currentOpenCosts.empty()) {
                if (truncatedDominance(targetFront, newHeapLabel->c)) {
                    currentOpenCosts.pop_front();
                    labelsPool->free(newHeapLabel);
                    if (!currentOpenCosts.empty()) {
                        newHeapLabel = currentOpenCosts.front();
                    }
                }
                else {
                    success = true;
                    heapLabels[n] = newHeapLabel;
                    heap.push(newHeapLabel);
                    break;
                }
            }
        }
        if (!success) {
            heapLabels.erase(n);
        }
    }

    void propagate(Node n, Label* minLabel, Pool<Label>* labelsPool, BinaryHeapMosp& heap) {
        const Neighborhood &outgoingArcs{this->G.outgoingArcs(n)};
        const size_t predPathIndex = permanentLabels.getCurrentIndex();
        bool expanded = false;
        CostArray costVector = minLabel->c;
        for (const Arc &a : outgoingArcs) {
            costVector = minLabel->c;
            const Node successorNode = a.n;
            addInPlace(costVector, a.c);
            if (truncatedDominance(targetFront, costVector) ||
                truncatedDominance(truncatedFront[successorNode], costVector)) {
                continue;
                }

            expanded = true;
            OpenCosts& successorOpenCosts = this->exploredPaths[successorNode];
            //Label* successorLabel{&heapLabels[successorNode]};
            Label* newLabel = labelsPool->newItem();
            newLabel->update(costVector, successorNode, a.revArcIndex, predPathIndex);
            if (successorOpenCosts.empty()) {
                successorOpenCosts.push_back(newLabel);
                heap.push(newLabel);
                heapLabels[successorNode] = newLabel;
                assert(newLabel != labelsPool->firstFreeSpace);
            }
            else {
                bool newLexMin = merge(labelsPool, successorOpenCosts, newLabel);
                if (newLexMin) {
                    Label* oldHeapLabel = heapLabels[successorNode];
                    heapLabels[successorNode] = newLabel;
                    heap.decreaseKey(oldHeapLabel, newLabel);
                }
            }
        }
        if (expanded) {
            permanentLabels.addElement(minLabel->permanentIndexOfSubpath, minLabel->predArcId);
        }
    }

    void run(Solution& solutionData) {
        auto mergeFunction =
                [](TruncatedFront& front, const CostArray& c) -> void {
                    DIM==3 ? merge_3d(front, c) : truncatedInsertionBackward(front, c);
                };
        size_t extractions{0};
        size_t iterations{0};
        Node target = G.target;
        auto start_time = std::chrono::high_resolution_clock::now();

        Pool<Label>* labelsPool = new Pool<Label>();

        Label* startLabel = labelsPool->newItem();
        startLabel->update(potential[G.source], G.source, INVALID_ARC, MAX_PATH);

        BinaryHeapMosp heap = BinaryHeapMosp(1);
        heap.push(startLabel);
        heapLabels[G.source] = startLabel;
        this->exploredPaths[G.source].push_back(startLabel);
        while (heap.size()) {
            //printf("%lu\n", heap.size());
            Label* minLabel = heap.pop();
            const Node n{minLabel->n};
            ++extractions;
            //printf("Extracting %u %u %u %u\n", n, minLabel->c[0], minLabel->c[1], minLabel->c[2]);
            TruncatedFront& currentFront{this->truncatedFront[n]};
            mergeFunction(currentFront, minLabel->c);

            this->nextQueuePath(n, labelsPool, heap);
            ++iterations;
            if (n == target) {
                //solutions.addSolution(minLabel);
                //printf("Solution with costs %u %u %u. Node %u.\n", minLabel.c[0], minLabel.c[1], minLabel.c[2], n);
//                printf("Solution %u %u %u\n", minLabel.c[0], minLabel.c[1], minLabel.c[2]);
                solutions.solutions.push_back(minLabel);
                ++sols;
                continue;
            }
            this->propagate(n, minLabel, labelsPool, heap);
            labelsPool->free(minLabel);
        }
        //printSolutions();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration_time = end_time - start_time;
        solutionData.duration = duration_time.count();
        solutionData.iterations = iterations;
        solutionData.extractions = extractions;
        solutionData.solutionsCt = solutions.solutions.size();
        solutionData.memoryConsumption = labelsPool->size()*sizeof(Label);
        solutionData.maxHeapSize = heap.maxSize();
        delete labelsPool;
    }

private:
    const Graph& G;
    std::unordered_map<Node, Label*> heapLabels;
    std::vector<OpenCosts> exploredPaths;
    std::vector<TruncatedFront> truncatedFront;
    TruncatedFront& targetFront;
    const std::vector<CostArray>& potential;
    Permanents permanentLabels;
    size_t sols{0};
    SolutionsList solutions;

    inline static bool merge(Pool<Label>* pool, OpenCosts& open, Label* newLabel) {
        auto it = open.begin();
        assert (!open.empty());
        while (it != open.end() && lexSmaller((*it)->c, newLabel->c)) {
            assert((*it)->n == newLabel->n);
            if (dominates((*it)->c, newLabel->c)) {
                pool->free(newLabel);
                return false;
            }
            ++it;
        }
        it  = open.insert(it, newLabel);
        bool insertedAtBeginning = it == open.begin();
        ++it;
        while (it != open.end()) {
            assert((*it)->n == newLabel->n);
            if (dominates(newLabel->c, (*it)->c)) {
                pool->free(*it);
                it = open.erase(it);
            }
            else {
                ++it;
            }
        }
        return insertedAtBeginning;
    }
};

#endif
