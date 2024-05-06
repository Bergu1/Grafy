#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <set>

using namespace std;
using namespace chrono;

struct Edge;

struct Vertex {
    int index;
    string name;
    vector<shared_ptr<Edge>> edges;
};

struct Edge {
    int index;
    int value;
    Vertex* v1;
    Vertex* v2;
};

class GraphList {
private:
    vector<Vertex> vertices;
    int numVertices;
    int numEdges;

public:
    GraphList(int V) : numVertices(V), numEdges(0) {
        vertices.reserve(V);
        for (int i = 0; i < V; ++i) {
            vertices.push_back(Vertex{i, to_string(i)});
        }
    }

    void insertRandomEdges(double density) {
        srand(time(nullptr));
        int maxEdges = numVertices * (numVertices - 1) / 2;
        int E = static_cast<int>(maxEdges * density / 100); 
        set<pair<int, int>> addedEdges;

        while (numEdges < E) {
            int u = rand() % numVertices;
            int v = rand() % numVertices;
            if (u != v && addedEdges.find({u, v}) == addedEdges.end() && addedEdges.find({v, u}) == addedEdges.end()) {
                int weight = rand() % 10 + 1;
                insertEdge(u, v, weight);
                addedEdges.insert({u, v});
                addedEdges.insert({v, u});
            }
        }
    }

    void insertEdge(int vIndex1, int vIndex2, int value) {
        if (vIndex1 < 0 || vIndex1 >= numVertices || vIndex2 < 0 || vIndex2 >= numVertices) {
            return; 
        }

        auto edge = make_shared<Edge>(Edge{numEdges++, value, &vertices[vIndex1], &vertices[vIndex2]});
        vertices[vIndex1].edges.push_back(edge);
        vertices[vIndex2].edges.push_back(edge);
    }

    void dijkstra(int srcIndex) {
        if (srcIndex < 0 || srcIndex >= numVertices) {
            cout << "Invalid source index." << endl;
            return;
        }

        vector<int> dist(numVertices, INT_MAX);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

        pq.push({0, srcIndex});
        dist[srcIndex] = 0;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto& edge : vertices[u].edges) {
                int vIndex = (edge->v1->index == u) ? edge->v2->index : edge->v1->index;
                int weight = edge->value;
                if (dist[vIndex] > dist[u] + weight) {
                    dist[vIndex] = dist[u] + weight;
                    pq.push({dist[vIndex], vIndex});
                }
            }
        }
    }
};

int main() {
    int numInstances = 10;
    int V = 1000; 
    double density = 50; 

    double totalDuration = 0;

    for (int i = 0; i < numInstances; ++i) {
        GraphList graph(V);
        graph.insertRandomEdges(density);

        auto start = high_resolution_clock::now();
        graph.dijkstra(0); 
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);

        totalDuration += duration.count();
    }

    double averageDuration = totalDuration / numInstances;

    cout << "Average time taken by Dijkstra's algorithm over " << numInstances << " instances: " << averageDuration << " microseconds." << endl;
    return 0;
}
