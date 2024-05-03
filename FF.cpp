#include <bits/stdc++.h>
using namespace std;

struct Edge {
    int destination;
    int capacity;
    int length;

    Edge(int destination, int capacity, int length) : destination(destination), capacity(capacity), length(length) {}
};

class Graph {
public:
    int vertex;
    vector<Edge> *adj;

    Graph(int vertex);
    void addEdge(int u, int v, int capacity, int length, bool isBidirectional);
    void print();
    vector<int> bfs(int source, int target, vector<int>& parent);
    int fordFulkerson(int source, int target);
    void recoverPath(int source, int target, vector<vector<int>>& paths);
};

Graph::Graph(int vertex) {
    this->vertex = vertex;
    this->adj = new vector<Edge>[vertex];
}

void Graph::addEdge(int u, int v, int capacity, int length, bool isBidirectional=false) {
    adj[u].push_back(Edge(v, capacity, length));
    if (isBidirectional) {
        adj[v].push_back(Edge(u, capacity, length));
    }
}

void Graph::print() {
    for (int i = 0; i < vertex; i++) {
        cout << i << "-> ";
        for (Edge edge : adj[i]) {
            cout << "(" << edge.destination << "," << edge.capacity << "," << edge.length << ") ";
        }
        cout << endl;
    }
}

vector<int> Graph::bfs(int source, int target, vector<int>& parent) {
    vector<int> visited(vertex, false);
    queue<pair<int,int>> q;
    q.push({source, INT_MAX});
    visited[source] = true;

    while (!q.empty()) {
        int u = q.front().first;
        int capacity = q.front().second;
        q.pop();

        for (Edge edge : adj[u]) {
            int v = edge.destination;
            int residual_capacity = edge.capacity;

            if (!visited[v] && residual_capacity > 0) {
                parent[v] = u;
                int min_capacity = min(capacity, residual_capacity);
                if (v == target)
                    return {min_capacity, v};

                q.push({v, min_capacity});
                visited[v] = true;
            }
        }
    }

    return {0, -1};
}

int Graph::fordFulkerson(int source, int target) {
    vector<int> parent(vertex, -1);
    int maxFlow = 0;
    vector<vector<int>> paths;

    while (true) {
        vector<int> path = bfs(source, target, parent);
        int min_capacity = path[0];
        int node = path[1];

        if (node == -1)
            break;

        vector<int> currentPath;
        int v = node;

        while (v != source) {
            currentPath.push_back(v);
            v = parent[v];
        }
        currentPath.push_back(source);
        reverse(currentPath.begin(), currentPath.end());
        paths.push_back(currentPath);

        maxFlow += min_capacity;
        v = node;

        while (v != source) {
            int u = parent[v];
            for (Edge& edge : adj[u]) {
                if (edge.destination == v) {
                    edge.capacity -= min_capacity;
                    break;
                }
            }
            bool found = false;
            for (Edge& edge : adj[v]) {
                if (edge.destination == u) {
                    edge.capacity += min_capacity;
                    found = true;
                    break;
                }
            }
            if (!found)
                adj[v].push_back(Edge(u, min_capacity, 0));
            v = u;
        }
    }

    cout << "Max Flow: " << maxFlow << endl;
    recoverPath(source, target, paths);
    return maxFlow;
}

void Graph::recoverPath(int source, int target, vector<vector<int>>& paths) {
    cout << "Paths from source " << source << " to target " << target << ":" << endl;
    int pathCount = 0;
    for (auto path : paths) {
        cout << "Path " << ++pathCount << ": ";
        for (int i = 0; i < path.size() - 1; ++i) {
            cout << path[i] << " -> ";
        }
        cout << path.back() << endl;
    }
}

int main() {
    Graph g(6);
    g.addEdge(0, 1, 11, 16);
    g.addEdge(0, 2, 13, 13);
    g.addEdge(1, 2, 0, 10);
    g.addEdge(1, 3, 12, 12);
    g.addEdge(2, 1, 1, 4);
    g.addEdge(2, 4, 11, 14);
    g.addEdge(3, 2, 0, 9);
    g.addEdge(3, 5, 19, 20);
    g.addEdge(4, 3, 7, 7);
    g.addEdge(4, 5, 4, 4);

    int source = 0;
    int sink = 5;

    g.fordFulkerson(source, sink);

    return 0;
}
