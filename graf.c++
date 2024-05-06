#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <climits>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <set>

using namespace std;
using namespace chrono;

struct Vertex {
    int index;
    string name;
};

struct Edge {
    int index;
    int value;
    Vertex* v1;
    Vertex* v2;
};

class GraphMatrix {
private:
    vector<Vertex> vertices;
    vector<vector<shared_ptr<Edge>>> adjMatrix;
    int numVertices;
    int numEdges;

public:
    GraphMatrix(int V) : numVertices(V), numEdges(0) {
        vertices.reserve(V);
        adjMatrix.resize(V, vector<shared_ptr<Edge>>(V, nullptr));
        for (int i = 0; i < V; ++i) {
            vertices.push_back(Vertex{i, to_string(i)});
        }
    }

    void insertRandomEdges(double density) {
        srand(time(nullptr));
        int maxEdges = numVertices * (numVertices - 1) / 2;
        int E = static_cast<int>(maxEdges * density);

        set<pair<int, int>> addedEdges;

        while (numEdges < E) {
            int u = rand() % numVertices;
            int v = rand() % numVertices;
            if (u != v && addedEdges.find({u, v}) == addedEdges.end() && addedEdges.find({v, u}) == addedEdges.end()) {
                int weight = rand() % 10 + 1;
                insertEdge(to_string(u), to_string(v), weight);
                addedEdges.insert({u, v});
                addedEdges.insert({v, u});  
            }
        }
    }

    void insertEdge(const string& vName1, const string& vName2, int value) {
        int index1 = findVertexIndex(vName1);
        int index2 = findVertexIndex(vName2);
        if (index1 == -1 || index2 == -1) {
            return; 
        }

        if (adjMatrix[index1][index2] == nullptr) {
            auto edge = make_shared<Edge>(Edge{numEdges++, value, &vertices[index1], &vertices[index2]});
            adjMatrix[index1][index2] = edge;
            adjMatrix[index2][index1] = edge; 
        }
    }

    void dijkstra(int srcIndex) {
        if (srcIndex < 0 || srcIndex >= numVertices) {
            cout << "Invalid source index." << endl;
            return;
        }

        vector<int> dist(numVertices, INT_MAX);
        vector<bool> visited(numVertices, false);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

        pq.push({0, srcIndex});
        dist[srcIndex] = 0;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            if (visited[u]) continue;
            visited[u] = true;

            for (int v = 0; v < numVertices; ++v) {
                auto edge = adjMatrix[u][v];
                if (edge && !visited[v]) {
                    int weight = edge->value;
                    if (dist[v] > dist[u] + weight) {
                        dist[v] = dist[u] + weight;
                        pq.push({dist[v], v});
                    }
                }
            }
        }

        // Output distances
        //cout << "Distances from vertex " << srcIndex << ":" << endl;
        //for (int i = 0; i < numVertices; ++i) {
            //cout << "to vertex " << i << " is " << (dist[i] == INT_MAX ? "unreachable" : to_string(dist[i])) << endl;
       //}
    }


private:
    int findVertexIndex(const string& name) {
        for (int i = 0; i < numVertices; ++i) {
            if (vertices[i].name == name) {
                return i;
            }
        }
        return -1;
    }
};

int main() {

    int numInstances = 10;
    int V = 10; 
    double density = 1; 

    double totalDuration = 0;

    for (int i = 0; i < numInstances; ++i) {
        GraphMatrix graph(V);
        graph.insertRandomEdges(density);

        auto start = high_resolution_clock::now();
        graph.dijkstra(0); 
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);

        totalDuration += duration.count();
    }

    double averageDuration = totalDuration / numInstances;

    cout << "Average time taken by Dijkstra's algorithm over " << numInstances << " instances: " << averageDuration << " nanoseconds." << endl;

    return 0;
}
