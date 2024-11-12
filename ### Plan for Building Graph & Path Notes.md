### Plan for Building Graph & Path Notes
1. Create a comprehensive class structure
2. Implement basic graph representations (adjacency matrix and list)
3. Add common graph algorithms
4. Include path finding algorithms
5. Add example usage

### Implementation

```java
// Graph and Path Notes

// 1. Graph Representations
public class GraphNotes {
    
    // Adjacency Matrix representation
    static class AdjacencyMatrix {
        private int V; // Number of vertices
        private int[][] matrix;
        
        public AdjacencyMatrix(int vertices) {
            this.V = vertices;
            matrix = new int[vertices][vertices];
        }
        
        public void addEdge(int source, int destination) {
            matrix[source][destination] = 1;
            matrix[destination][source] = 1; // For undirected graph
        }
    }
    
    // Adjacency List representation
    static class AdjacencyList {
        private int V;
        private ArrayList<ArrayList<Integer>> adj;
        
        public AdjacencyList(int vertices) {
            this.V = vertices;
            adj = new ArrayList<>(V);
            for (int i = 0; i < V; i++) {
                adj.add(new ArrayList<>());
            }
        }
        
        public void addEdge(int source, int destination) {
            adj.get(source).add(destination);
            adj.get(destination).add(source); // For undirected graph
        }
    }
    
    // 2. Graph Traversal Algorithms
    static class GraphTraversal {
        // BFS implementation
        public void BFS(int[][] graph, int start) {
            boolean[] visited = new boolean[graph.length];
            Queue<Integer> queue = new LinkedList<>();
            
            visited[start] = true;
            queue.offer(start);
            
            while (!queue.isEmpty()) {
                int vertex = queue.poll();
                System.out.print(vertex + " ");
                
                for (int i = 0; i < graph.length; i++) {
                    if (graph[vertex][i] == 1 && !visited[i]) {
                        visited[i] = true;
                        queue.offer(i);
                    }
                }
            }
        }
        
        // DFS implementation
        public void DFS(int[][] graph, int start, boolean[] visited) {
            visited[start] = true;
            System.out.print(start + " ");
            
            for (int i = 0; i < graph.length; i++) {
                if (graph[start][i] == 1 && !visited[i]) {
                    DFS(graph, i, visited);
                }
            }
        }
    }
    
    // 3. Path Finding Algorithms
    static class PathFinding {
        // Dijkstra's Algorithm
        public int[] dijkstra(int[][] graph, int start) {
            int V = graph.length;
            int[] distances = new int[V];
            boolean[] visited = new boolean[V];
            
            Arrays.fill(distances, Integer.MAX_VALUE);
            distances[start] = 0;
            
            for (int i = 0; i < V-1; i++) {
                int minVertex = findMinDistance(distances, visited);
                visited[minVertex] = true;
                
                for (int j = 0; j < V; j++) {
                    if (!visited[j] && graph[minVertex][j] != 0 && 
                        distances[minVertex] != Integer.MAX_VALUE &&
                        distances[minVertex] + graph[minVertex][j] < distances[j]) {
                        distances[j] = distances[minVertex] + graph[minVertex][j];
                    }
                }
            }
            return distances;
        }
        
        private int findMinDistance(int[] distances, boolean[] visited) {
            int min = Integer.MAX_VALUE;
            int minIndex = -1;
            
            for (int i = 0; i < distances.length; i++) {
                if (!visited[i] && distances[i] < min) {
                    min = distances[i];
                    minIndex = i;
                }
            }
            return minIndex;
        }
    }
    
    // Example Usage
    public static void main(String[] args) {
        // Create a sample graph
        int V = 5;
        AdjacencyMatrix graph = new AdjacencyMatrix(V);
        graph.addEdge(0, 1);
        graph.addEdge(0, 2);
        graph.addEdge(1, 2);
        graph.addEdge(2, 3);
        graph.addEdge(3, 4);
        
        // Demonstrate traversal
        System.out.println("BFS starting from vertex 0:");
        GraphTraversal traversal = new GraphTraversal();
        traversal.BFS(graph.matrix, 0);
        
        System.out.println("\nDFS starting from vertex 0:");
        traversal.DFS(graph.matrix, 0, new boolean[V]);
        
        // Demonstrate path finding
        int[][] weightedGraph = {
            {0, 4, 0, 0, 0},
            {4, 0, 8, 0, 0},
            {0, 8, 0, 7, 0},
            {0, 0, 7, 0, 9},
            {0, 0, 0, 9, 0}
        };
        
        PathFinding pathFinder = new PathFinding();
        int[] distances = pathFinder.dijkstra(weightedGraph, 0);
        
        System.out.println("\n\nShortest distances from vertex 0:");
        for (int i = 0; i < V; i++) {
            System.out.println("To vertex " + i + ": " + distances[i]);
        }
    }
}
```

This code provides a comprehensive overview of graphs and paths in Java, including:
- Different graph representations
- Basic traversal algorithms (BFS and DFS)
- Path finding with Dijkstra's algorithm
- Example usage with sample graphs

### Planning Steps
1. Create a documentation class for Graph Theory concepts
2. Define graph types (directed, undirected, weighted, etc.)
3. Document key terminology
4. Add visual representations through ASCII art
5. Include practical examples
6. Link concepts to implementations

```java



public class GraphTheoryNotes {
    /**
     * GRAPH THEORY FUNDAMENTALS
     * 
     * 1. BASIC DEFINITIONS
     * - Graph: A collection of vertices/nodes connected by edges
     * - Vertex: A fundamental unit/node in a graph
     * - Edge: Connection between two vertices
     * - Path: Sequence of vertices connected by edges
     * - Cycle: Path that starts and ends at same vertex
     * 
     * 2. TYPES OF GRAPHS
     * a) By Edge Direction:
     *    - Undirected Graph: Edges have no direction
     *      A --- B
     *    - Directed Graph (Digraph): Edges have direction
     *      A --→ B
     * 
     * b) By Edge Weight:
     *    - Unweighted Graph: All edges have equal importance
     *    - Weighted Graph: Edges have associated costs/weights
     *      A ---(5)--- B
     * 
     * c) Special Types:
     *    - Tree: Connected graph with no cycles
     *    - DAG: Directed Acyclic Graph
     *    - Complete Graph: Every vertex connected to every other vertex
     *    - Bipartite Graph: Vertices can be divided into two sets
     * 
     * 3. GRAPH PROPERTIES
     * - Connected: Path exists between any two vertices
     * - Strongly Connected: Directed path exists between any two vertices
     * - Dense: Many edges (close to complete graph)
     * - Sparse: Few edges
     */
    
    // Example implementations for different graph types
    static class DirectedGraph {
        private int V;
        private ArrayList<ArrayList<Integer>> adj;
        
        public DirectedGraph(int vertices) {
            this.V = vertices;
            adj = new ArrayList<>(V);
            for (int i = 0; i < V; i++) {
                adj.add(new ArrayList<>());
            }
        }
        
        public void addEdge(int source, int destination) {
            adj.get(source).add(destination); // Only add in one direction
        }
    }
    
    static class WeightedGraph {
        private int V;
        private ArrayList<ArrayList<Edge>> adj;
        
        static class Edge {
            int destination, weight;
            Edge(int destination, int weight) {
                this.destination = destination;
                this.weight = weight;
            }
        }
        
        public WeightedGraph(int vertices) {
            this.V = vertices;
            adj = new ArrayList<>(V);
            for (int i = 0; i < V; i++) {
                adj.add(new ArrayList<>());
            }
        }
        
        public void addEdge(int source, int destination, int weight) {
            adj.get(source).add(new Edge(destination, weight));
            adj.get(destination).add(new Edge(source, weight)); // For undirected weighted graph
        }
    }
    
    /**
     * 4. COMMON GRAPH OPERATIONS
     * - Traversal: BFS, DFS
     * - Path Finding: Dijkstra's, Bellman-Ford, Floyd-Warshall
     * - Minimum Spanning Tree: Prim's, Kruskal's
     * - Cycle Detection
     * - Topological Sort (for DAGs)
     * 
     * 5. TIME COMPLEXITIES
     * - Adjacency Matrix:
     *   * Space: O(V²)
     *   * Add Edge: O(1)
     *   * Remove Edge: O(1)
     *   * Check Edge: O(1)
     * 
     * - Adjacency List:
     *   * Space: O(V + E)
     *   * Add Edge: O(1)
     *   * Remove Edge: O(degree(v))
     *   * Check Edge: O(degree(v))
     */
    
    public static void main(String[] args) {
        // Example usage of directed graph
        DirectedGraph dGraph = new DirectedGraph(4);
        dGraph.addEdge(0, 1);
        dGraph.addEdge(1, 2);
        dGraph.addEdge(2, 3);
        
        // Example usage of weighted graph
        WeightedGraph wGraph = new WeightedGraph(4);
        wGraph.addEdge(0, 1, 5);
        wGraph.addEdge(1, 2, 3);
        wGraph.addEdge(2, 3, 1);
    }
}
```

### Step-by-Step Plan
1. Create a GraphTerminology class
2. Add sections for:
   - Basic graph types
   - Search algorithms
   - Graph properties
   - Common operations
3. Include ASCII art examples
4. Add code examples

```java



public class GraphTerminology {
    /**
     * GRAPH TERMINOLOGY AND CONCEPTS
     * 
     * 1. GRAPH TYPES
     * ===============
     * Directed Graph (Digraph):
     * A → B (one-way connection)
     * - Edges have direction
     * - Used in: one-way streets, web pages links
     * 
     * Undirected Graph:
     * A ― B (two-way connection)
     * - Edges have no direction
     * - Used in: social networks, computer networks
     * 
     * Weighted Graph:
     * A ---(5)--- B
     * - Edges have values/weights
     * - Used in: road networks (distances), network costs
     * 
     * 2. SEARCH ALGORITHMS
     * ===================
     * Breadth-First Search (BFS):
     * - Visits neighbors before going deeper
     * - Uses queue data structure
     * - Best for: shortest path in unweighted graphs
     * Time complexity: O(V + E)
     * 
     * Depth-First Search (DFS):
     * - Explores as far as possible before backtracking
     * - Uses stack/recursion
     * - Best for: maze solving, topological sorting
     * Time complexity: O(V + E)
     * 
     * 3. PATH ALGORITHMS
     * =================
     * Dijkstra's Algorithm:
     * - Finds shortest path in weighted graphs
     * - Cannot handle negative weights
     * Time complexity: O(V^2) or O((V+E)logV) with priority queue
     * 
     * Bellman-Ford:
     * - Can handle negative weights
     * - Detects negative cycles
     * Time complexity: O(VE)
     * 
     * 4. GRAPH PROPERTIES
     * ==================
     * Connected: Every vertex can reach every other vertex
     * Cyclic: Contains at least one cycle
     * Acyclic: No cycles (like a tree)
     * Complete: Every vertex connected to every other vertex
     * 
     * 5. COMMON OPERATIONS
     * ===================
     * - Add/Remove vertex: O(1)
     * - Add/Remove edge: O(1)
     * - Find neighbors: O(1) for adjacency list
     * - Check if edge exists: O(1) for adjacency matrix
     */

    // Example implementation of BFS
    public static void bfs(int[][] graph, int start) {
        boolean[] visited = new boolean[graph.length];
        Queue<Integer> queue = new LinkedList<>();
        
        visited[start] = true;
        queue.offer(start);
        
        while (!queue.isEmpty()) {
            int vertex = queue.poll();
            System.out.print(vertex + " ");
            
            for (int i = 0; i < graph.length; i++) {
                if (graph[vertex][i] == 1 && !visited[i]) {
                    visited[i] = true;
                    queue.offer(i);
                }
            }
        }
    }

    // Example implementation of DFS
    public static void dfs(int[][] graph, int start, boolean[] visited) {
        visited[start] = true;
        System.out.print(start + " ");
        
        for (int i = 0; i < graph.length; i++) {
            if (graph[start][i] == 1 && !visited[i]) {
                dfs(graph, i, visited);
            }
        }
    }

    public static void main(String[] args) {
        // Example graph
        int[][] graph = {
            {0, 1, 1, 0},
            {1, 0, 0, 1},
            {1, 0, 0, 1},
            {0, 1, 1, 0}
        };

        System.out.println("BFS traversal starting from vertex 0:");
        bfs(graph, 0);
        
        System.out.println("\nDFS traversal starting from vertex 0:");
        dfs(graph, 0, new boolean[graph.length]);
    }
}
```
/**
 * Definition of a Tree data structure:
 * - Connected: Every node is reachable from any other node through a path
 * - Acyclic: No cycles exist between nodes (no closed loops)
 * - Hierarchical: One node (root) serves as the parent, with child nodes below
 * 
 * Properties:
 * - Has exactly n-1 edges where n is the number of nodes
 * - Has exactly one path between any two nodes
 * - Removing any edge disconnects the tree
 * - Adding any edge creates a cycle
 */
A tree is a connected, acyclic and hierarchical structure.
public class TreeGraphNotes {
    /**
     * TREE AS A SPECIAL TYPE OF GRAPH
     * ==============================
     * Properties:
     * - Connected, undirected graph
     * - No cycles (acyclic)
     * - N vertices have exactly N-1 edges
     * - Exactly one path between any two vertices
     * 
     * Types of Trees:
     * 1. Rooted Tree:
     *    - Has a designated root node
     *    - All edges point away from root
     *    Example:
     *        A
     *       / \
     *      B   C
     *     / \
     *    D   E
     * 
     * 2. Binary Tree:
     *    - Each node has at most 2 children
     * 
     * 3. N-ary Tree:
     *    - Each node can have up to N children 
     */
    
     ### Plan for Tree Graph Implementation:
     1. Complete TreeNode class with children
     2. Add TreeGraph class with basic operations
     3. Include common tree traversal methods
     4. Add example usage
     5. Include tree validation methods
     
     ```java
     
     
     ...existing code...
             int value;
             List<TreeNode> children;
             
             public TreeNode(int value) {
                 this.value = value;
                 this.children = new ArrayList<>();
             }
         }
         
         static class TreeGraph {
             TreeNode root;
             
             // Tree validation method
             public boolean isValidTree(TreeNode node, Set<TreeNode> visited) {
                 if (visited.contains(node)) {
                     return false; // Contains cycle
                 }
                 
                 visited.add(node);
                 for (TreeNode child : node.children) {
                     if (!isValidTree(child, visited)) {
                         return false;
                     }
                 }
                 return true;
             }
             
             // Tree traversals
             public void preorderTraversal(TreeNode node) {
                 if (node == null) return;
                 
                 System.out.print(node.value + " ");
                 for (TreeNode child : node.children) {
                     preorderTraversal(child);
                 }
             }
             
             public void levelOrderTraversal(TreeNode root) {
                 if (root == null) return;
                 
                 Queue<TreeNode> queue = new LinkedList<>();
                 queue.offer(root);
                 
                 while (!queue.isEmpty()) {
                     TreeNode current = queue.poll();
                     System.out.print(current.value + " ");
                     queue.addAll(current.children);
                 }
             }
         }
         
         public static void main(String[] args) {
             // Example usage
             TreeGraph tree = new TreeGraph();
             tree.root = new TreeNode(1);
             TreeNode node2 = new TreeNode(2);
             TreeNode node3 = new TreeNode(3);
             TreeNode node4 = new TreeNode(4);
             TreeNode node5 = new TreeNode(5);
             
             tree.root.children.add(node2);
             tree.root.children.add(node3);
             node2.children.add(node4);
             node2.children.add(node5);
             
             System.out.println("Preorder traversal:");
             tree.preorderTraversal(tree.root);
             
             System.out.println("\nLevel order traversal:");
             tree.levelOrderTraversal(tree.root);
             
             System.out.println("\nIs valid tree: " + 
                 tree.isValidTree(tree.root, new HashSet<>()));
         }
     }
     ```
     
/**
 * Adjacency Matrix:
 * - Representation of graph using a 2D array
 * - Size: V x V where V is number of vertices
 * - matrix[i][j] = 1 if edge exists between i and j
 * - matrix[i][j] = 0 if no edge exists
 * - Space complexity: O(V^2)
 * - Time complexity for edge operations: O(1)
 */
### Review and Additional Information
### Review and Additional Information

#### Tree Graph Implementation

In the provided code excerpt, we have an implementation of a **Tree Graph** using custom classes and methods.

```java
// Creating tree nodes
TreeNode node2 = new TreeNode(2);
TreeNode node3 = new TreeNode(3);
TreeNode node4 = new TreeNode(4);
TreeNode node5 = new TreeNode(5);

// Building the tree structure
tree.root.children.add(node2);
tree.root.children.add(node3);
node2.children.add(node4);
node2.children.add(node5);

// Traversal methods
System.out.println("Preorder traversal:");
tree.preorderTraversal(tree.root);

System.out.println("\nLevel order traversal:");
tree.levelOrderTraversal(tree.root);

// Tree validation
System.out.println("\nIs valid tree: " + 
    tree.isValidTree(tree.root, new HashSet<>()));
```

**Explanation:**

- **TreeNode Class**: Represents each node in the tree with a value and a list of child nodes.
  
- **Tree Structure Creation**:
  - **Nodes**:
    - `node2`, `node3`, `node4`, `node5` are instances of `TreeNode` with values 2 to 5.
  - **Building the Tree**:
    - `tree.root.children.add(node2);` adds `node2` as a child of the root node.
    - `tree.root.children.add(node3);` adds `node3` as another child of the root.
    - `node2.children.add(node4);` and `node2.children.add(node5);` add `node4` and `node5` as children of `node2`.

**Visual Representation of the Tree:**

```
        1
       / \
      2   3
     / \
    4   5
```

#### Traversal Methods

**1. Preorder Traversal:**

```java
tree.preorderTraversal(tree.root);
```

- **Definition**: Visits the root node first, then recursively traverses each subtree.
- **Order of Visit**: `1 → 2 → 4 → 5 → 3`
- **Use Cases**: Copying trees, prefix expression evaluation.

**2. Level Order Traversal:**

```java
tree.levelOrderTraversal(tree.root);
```

- **Definition**: Visits nodes level by level starting from the root.
- **Order of Visit**: `1 → 2 → 3 → 4 → 5`
- **Use Cases**: Finding the shortest path, serializing trees.

#### Tree Validation

```java
tree.isValidTree(tree.root, new HashSet<>());
```

- **Purpose**: Checks if the tree is valid (i.e., acyclic and connected).
- **Method**:
  - Uses Depth-First Search (DFS) to traverse the tree.
  - Stores visited nodes in a `HashSet` to detect cycles.
  - Returns `true` if there are no cycles and all nodes are connected.

#### Adjacency Matrix

**Definition and Key Points:**

```java
/**
 * Adjacency Matrix:
 * - Graph representation using a 2D array.
 * - Dimensions: V x V, where V is the number of vertices.
 * - `matrix[i][j] = 1` if an edge exists between vertex `i` and `j`.
 * - `matrix[i][j] = 0` if no edge exists.
 * - **Space Complexity**: O(V²).
 * - **Edge Operations Time Complexity**: O(1) for adding, removing, or checking edges.
 */
```

- **Advantages**:
  - Efficient edge look-up.
  - Simple and intuitive representation.
- **Disadvantages**:
  - High space consumption for sparse graphs.
- **Use Cases**:
  - Dense graphs where edge look-up speed is critical.

#### Key Terms and Concepts

**Graph Types:**

- **Directed Graph (Digraph)**:
  - Edges have a direction (`A → B`).
  - Represents one-way relationships.
- **Undirected Graph**:
  - Edges are bidirectional (`A ― B`).
  - Represents mutual relationships.
- **Weighted Graph**:
  - Edges carry a weight/cost (`A --(5)-- B`).
  - Useful for finding the shortest path considering weights.
- **Tree**:
  - A connected acyclic undirected graph.
  - Special case of a graph with hierarchical structure.

**Traversal Algorithms:**

- **Breadth-First Search (BFS)**:
  - Explores neighbor nodes first before moving to the next level.
  - Utilizes a queue data structure.
  - **Time Complexity**: O(V + E), where V is vertices and E is edges.
  - **Applications**: Shortest path in unweighted graphs, level order traversal.

- **Depth-First Search (DFS)**:
  - Explores as far as possible along each branch before backtracking.
  - Can be implemented recursively or using a stack.
  - **Time Complexity**: O(V + E).
  - **Applications**: Topological sorting, cycle detection, path finding.

**Graph Properties:**

- **Connected Graph**:
  - There is a path between every pair of vertices.
- **Acyclic Graph**:
  - A graph without cycles.
- **Cyclic Graph**:
  - Contains at least one cycle.
- **Complete Graph**:
  - Every pair of distinct vertices is connected by a unique edge.

**Common Operations:**

- **Add/Remove Vertex**:
  - Adjacency List: O(1).
  - Adjacency Matrix: O(V²), may require resizing.
- **Add/Remove Edge**:
  - Adjacency List: O(1).
  - Adjacency Matrix: O(1).
- **Edge Existence Check**:
  - Adjacency Matrix: O(1).
  - Adjacency List: O(k), where k is the degree of the vertex.

**Additional Concepts:**

- **Path**:
  - A sequence of edges connecting a sequence of vertices.
- **Cycle**:
  - A path that starts and ends at the same vertex without repeating edges or vertices.
- **Degree of a Vertex**:
  - Number of edges incident to the vertex.
  - **In-degree**: For directed graphs, number of incoming edges.
  - **Out-degree**: For directed graphs, number of outgoing edges.
  
- **Sparse vs. Dense Graphs**:
  - **Sparse**: Contains few edges relative to the number of vertices.
  - **Dense**: Contains a large number of edges close to the maximum number of edges.

#### Practical Applications

- **Trees**:
  - Represent hierarchical data (e.g., file systems, organizational structures).
  - Used in search algorithms, parsers, and databases.
- **Graphs**:
  - Modeling networks (social, communications, transportation).
  - Solving puzzles and games (e.g., mazes, shortest path problems).

#### Summary

The provided code demonstrates how to implement and utilize tree structures in Java. It includes:

- Creation of nodes and building a tree.
- Tree traversal methods to visit nodes in different orders.
- A method to validate the structure of the tree.

Understanding these concepts is fundamental in learning how data structures like trees and graphs are used to model and solve real-world problems.

### Adjacency Matrix Representation

```
    0  1  2  3
0  0  1  1  0
1  1  0  1  1
2  1  1  0  0
3  0  1  0  0
```

This matrix represents a graph where:
- Vertices are numbered 0-3
- 1 indicates an edge exists
- 0 indicates no edge
- Matrix is symmetric (undirected graph)

Visual representation:
```
  0 ---- 1
  | \    |
  |   \  |
  2     3
```

Key characteristics:
- Space complexity: O(V²)
- Good for dense graphs
- Quick edge lookups: O(1)
- Wastes space for sparse graphs

```
    y
    ^
 3 |   0 1 0 0
 2 |   1 1 0 0
 1 |   1 0 1 1
 0 |   0 1 0 0
    +-------------> x
      0 1 2 3
```

```
 y
 7 |  0 0 0 0 1
 6 |  0 0 0 0
 5 |  0 0 0 0
 4 |  0 1 0 0
 3 |  0 1 1 0 1
 2 |  0 1 0 1
 1 |  0 1 1 1 1
 0 |  0 0 0 0 
    +----------------
      0 1 2 3 4 5 6 7 x
```


