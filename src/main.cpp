    #define _NAMOA_
    #define _MDA_

    #include <chrono>  // for high_resolution_clock

    #include <iostream>
    #include <memory>
    #include <sstream>
    #include <cinttypes>
    #include <unistd.h>
    #include <limits.h>

    #include "../datastructures/includes/NodeInfoContainer.h"
    #include "../datastructures/includes/NodeInfo.h"

    #include "../preprocessing/includes/Preprocessor.h"
    #include "../search/includes/MDA.h"
    #include "../search/includes/Martins.h"
    using namespace std;

    int main(int argc, char *argv[]) {

        char hostname[HOST_NAME_MAX];
        char username[LOGIN_NAME_MAX];
        gethostname(hostname, HOST_NAME_MAX);
        getlogin_r(username, LOGIN_NAME_MAX);
        std::string host(hostname);
        if (argc != 4) {
            printf("The program is meant to receive three arguments: file-directory, id of the source node, and id of the target node.\n");
            exit(1);
        }

        //printNumLimits();

        Node sourceId = INVALID_NODE;
        Node targetId = INVALID_NODE;
        stringstream(argv[2]) >> sourceId;
        stringstream(argv[3]) >> targetId;

        unique_ptr<Graph> G_ptr = setupGraph(argv[1], sourceId, targetId);
        Graph& G = *G_ptr;
        const string graphName{split(argv[1], '/').back()};
        G.setName(graphName);
        if (G.costComponentAdded) {
            G.exportGraph();
            exit(1);
        }


        if (G.nodesCount < sourceId || G.nodesCount < targetId) {
            throw;
        }

        Preprocessor preprocessor(G);
        NodeInfoContainer<NodeInfo> prepInfo(G); //For the search!
        preprocessor.run(prepInfo);
        std::vector<CostArray> potential(G.nodesCount);
        for (Node n = 0; n < G.nodesCount; ++n) {
            potential[n] = prepInfo.getInfo(n).potential;
        }

#ifdef _MDA_
        MDA mda{G, potential};
        Solution sol_bda_forward(graphName, sourceId, targetId, G);
        mda.run(sol_bda_forward);
        sol_bda_forward.print("MDA", host);
#endif

#ifdef _NAMOA_
        {
            Solution sol(graphName, sourceId, targetId, G);
            Martins martins{G, potential};
            martins.run(sol);
            sol.print("Martins", host);
        }
#endif

        //assert(namoa_solutions == mda_solutions);
    }
