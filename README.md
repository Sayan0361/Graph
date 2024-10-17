# Graph Operations

This Java program provides a comprehensive implementation of graph operations, including various graph algorithms such as Depth First Search (DFS), Breadth First Search (BFS), Dijkstra's Algorithm, Bellman-Ford Algorithm, Floyd-Warshall Algorithm, Prim's Algorithm, and Kruskal's Algorithm.

## Features

- **Graph Creation:**  
  Users can create a graph by adding edges with weights, supporting both directed and undirected graphs.

- **Graph Traversal:**  
  - Depth First Search (DFS) traversal.
  - Breadth First Search (BFS) traversal.

- **Graph Algorithms:**
  - **Topological Sort:** Ordering of vertices in a directed acyclic graph (DAG).
  - **Dijkstra's Algorithm:** Finding the shortest paths from a source vertex to all other vertices in a weighted graph.
  - **Bellman-Ford Algorithm:** Finding the shortest paths from a source vertex to all other vertices, handling negative weights.
  - **Floyd-Warshall Algorithm:** Finding shortest paths between all pairs of vertices.
  - **Prim's Algorithm:** Finding the minimum spanning tree (MST) of a weighted undirected graph.
  - **Kruskal's Algorithm:** Another method for finding the minimum spanning tree (MST) of a weighted undirected graph.

## Structure

- **`Graph.java`:**  
  Main Java file containing the graph implementation and various operations.

- **`Edge.java`:**  
  Class representing a weighted edge in the graph.

- **`UnionFind.java`:**  
  Class for implementing the union-find data structure, used in Kruskal's Algorithm.

## Dependencies

- The program uses the `Scanner` class for user input.
- The following classes are utilized for various operations:
  - `LinkedList`
  - `Queue`
  - `PriorityQueue`
  - `ArrayList`
  - `Map`
  - `Arrays`

