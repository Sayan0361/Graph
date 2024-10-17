import java.util.*;

public class Graph {
    private final int V; // Number of vertices
    private final LinkedList<Edge>[] adjList; // Adjacency list

    // Edge class to represent a weighted edge in the graph
    static class Edge {
        int dest, weight;

        Edge(int dest, int weight) {
            this.dest = dest;
            this.weight = weight;
        }
    }

    public Graph(int v) {
        V = v;
        adjList = new LinkedList[V];
        for (int i = 0; i < V; i++) {
            adjList[i] = new LinkedList<>();
        }
    }

    // Method to add an edge to the graph
    public void addEdge(int src, int dest, int weight) {
        adjList[src].add(new Edge(dest, weight));
        // If it's an undirected graph, uncomment the line below
        // adjList[dest].add(new Edge(src, weight));
    }

    // DFS Traversal
    public void DFS(int start) {
        boolean[] visited = new boolean[V];
        DFSUtil(start, visited);
        System.out.println();
    }

    private void DFSUtil(int vertex, boolean[] visited) {
        visited[vertex] = true;
        System.out.print(vertex + " ");

        for (Edge edge : adjList[vertex]) {
            if (!visited[edge.dest]) {
                DFSUtil(edge.dest, visited);
            }
        }
    }

    // BFS Traversal
    public void BFS(int start) {
        boolean[] visited = new boolean[V];
        Queue<Integer> queue = new LinkedList<>();
        visited[start] = true;
        queue.add(start);

        while (!queue.isEmpty()) {
            int vertex = queue.poll();
            System.out.print(vertex + " ");

            for (Edge edge : adjList[vertex]) {
                if (!visited[edge.dest]) {
                    visited[edge.dest] = true;
                    queue.add(edge.dest);
                }
            }
        }
        System.out.println();
    }

    // Topological Sort (Kahn's Algorithm)
    public void topologicalSort() {
        int[] inDegree = new int[V];
        for (int i = 0; i < V; i++) {
            for (Edge edge : adjList[i]) {
                inDegree[edge.dest]++;
            }
        }

        Queue<Integer> queue = new LinkedList<>();
        for (int i = 0; i < V; i++) {
            if (inDegree[i] == 0) {
                queue.add(i);
            }
        }

        List<Integer> topoOrder = new ArrayList<>();
        while (!queue.isEmpty()) {
            int vertex = queue.poll();
            topoOrder.add(vertex);

            for (Edge edge : adjList[vertex]) {
                if (--inDegree[edge.dest] == 0) {
                    queue.add(edge.dest);
                }
            }
        }

        System.out.println("Topological Sort (Kahn's Algorithm): " + topoOrder);
    }

    // Dijkstra's Algorithm
    public void dijkstra(int start) {
        int[] dist = new int[V];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[start] = 0;

        PriorityQueue<Edge> pq = new PriorityQueue<>(Comparator.comparingInt(e -> e.weight));
        pq.add(new Edge(start, 0));

        while (!pq.isEmpty()) {
            Edge edge = pq.poll();
            int vertex = edge.dest;

            for (Edge neighbor : adjList[vertex]) {
                int newDist = dist[vertex] + neighbor.weight;
                if (newDist < dist[neighbor.dest]) {
                    dist[neighbor.dest] = newDist;
                    pq.add(new Edge(neighbor.dest, newDist));
                }
            }
        }

        System.out.println("Dijkstra's Algorithm: ");
        for (int i = 0; i < V; i++) {
            System.out.println("Distance from " + start + " to " + i + " is " + dist[i]);
        }
    }

    // Bellman-Ford Algorithm
    public void bellmanFord(int start) {
        int[] dist = new int[V];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[start] = 0;

        for (int i = 1; i < V; i++) {
            for (int u = 0; u < V; u++) {
                for (Edge edge : adjList[u]) {
                    if (dist[u] != Integer.MAX_VALUE && dist[u] + edge.weight < dist[edge.dest]) {
                        dist[edge.dest] = dist[u] + edge.weight;
                    }
                }
            }
        }

        System.out.println("Bellman-Ford Algorithm: ");
        for (int i = 0; i < V; i++) {
            System.out.println("Distance from " + start + " to " + i + " is " + dist[i]);
        }
    }

    // Floyd-Warshall Algorithm
    public void floydWarshall() {
        int[][] dist = new int[V][V];

        for (int i = 0; i < V; i++) {
            Arrays.fill(dist[i], Integer.MAX_VALUE);
            dist[i][i] = 0;
        }

        for (int u = 0; u < V; u++) {
            for (Edge edge : adjList[u]) {
                dist[u][edge.dest] = edge.weight;
            }
        }

        for (int k = 0; k < V; k++) {
            for (int i = 0; i < V; i++) {
                for (int j = 0; j < V; j++) {
                    if (dist[i][k] != Integer.MAX_VALUE && dist[k][j] != Integer.MAX_VALUE) {
                        dist[i][j] = Math.min(dist[i][j], dist[i][k] + dist[k][j]);
                    }
                }
            }
        }

        System.out.println("Floyd-Warshall Algorithm: ");
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (dist[i][j] == Integer.MAX_VALUE) {
                    System.out.print("INF ");
                } else {
                    System.out.print(dist[i][j] + " ");
                }
            }
            System.out.println();
        }
    }

    // Prim's Algorithm
    public void prims() {
        boolean[] inMST = new boolean[V];
        Edge[] minEdge = new Edge[V];
        int[] key = new int[V];
        Arrays.fill(key, Integer.MAX_VALUE);
        key[0] = 0;
        minEdge[0] = new Edge(0, 0);
        PriorityQueue<Edge> pq = new PriorityQueue<>(Comparator.comparingInt(e -> e.weight));
        pq.add(new Edge(0, 0));

        while (!pq.isEmpty()) {
            Edge edge = pq.poll();
            int u = edge.dest;

            if (inMST[u]) continue;

            inMST[u] = true;

            for (Edge neighbor : adjList[u]) {
                int v = neighbor.dest;
                if (!inMST[v] && neighbor.weight < key[v]) {
                    key[v] = neighbor.weight;
                    pq.add(new Edge(v, key[v]));
                    minEdge[v] = edge; // For showing the minimum edges
                }
            }
        }

        System.out.println("Prim's Algorithm: ");
        for (int i = 1; i < V; i++) {
            System.out.println("Edge: " + minEdge[i].dest + " - " + i + " Weight: " + key[i]);
        }
    }

    // Kruskal's Algorithm
    public void kruskal() {
        List<Edge> edges = new ArrayList<>();
        for (int i = 0; i < V; i++) {
            for (Edge edge : adjList[i]) {
                edges.add(edge);
            }
        }

        edges.sort(Comparator.comparingInt(e -> e.weight));
        UnionFind uf = new UnionFind(V);
        List<Edge> mstEdges = new ArrayList<>();

        for (Edge edge : edges) {
            if (uf.find(edge.dest) != uf.find(edge.weight)) {
                uf.union(edge.dest, edge.weight);
                mstEdges.add(edge);
            }
        }

        System.out.println("Kruskal's Algorithm: ");
        for (Edge edge : mstEdges) {
            System.out.println("Edge: " + edge.dest + " Weight: " + edge.weight);
        }
    }

    // Union-Find class for Kruskal's Algorithm
    static class UnionFind {
        private final int[] parent, rank;

        public UnionFind(int size) {
            parent = new int[size];
            rank = new int[size];
            for (int i = 0; i < size; i++) {
                parent[i] = i;
                rank[i] = 0;
            }
        }

        public int find(int u) {
            if (parent[u] != u) {
                parent[u] = find(parent[u]); // Path compression
            }
            return parent[u];
        }

        public void union(int u, int v) {
            int rootU = find(u);
            int rootV = find(v);
            if (rootU != rootV) {
                if (rank[rootU] > rank[rootV]) {
                    parent[rootV] = rootU;
                } else if (rank[rootU] < rank[rootV]) {
                    parent[rootU] = rootV;
                } else {
                    parent[rootV] = rootU;
                    rank[rootU]++;
                }
            }
        }
    }

    public static void main(String[] args) {
        Graph graph = new Graph(6);
        graph.addEdge(0, 1, 4);
        graph.addEdge(0, 2, 3);
        graph.addEdge(1, 2, 1);
        graph.addEdge(1, 3, 2);
        graph.addEdge(2, 3, 4);
        graph.addEdge(3, 4, 2);
        graph.addEdge(4, 5, 6);
        graph.addEdge(3, 5, 3);

        System.out.println("DFS Traversal:");
        graph.DFS(0);

        System.out.println("BFS Traversal:");
        graph.BFS(0);

        graph.topologicalSort();
        graph.dijkstra(0);
        graph.bellmanFord(0);
        graph.floydWarshall();
        graph.prims();
        graph.kruskal();
    }
}
