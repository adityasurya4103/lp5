/*
 * Problem Statement:
 * Design and implement Parallel Depth First and bfs Search based on existing algorithms using OpenMP.
 * Use a Tree or an undirected graph for DFS.
 *
 * How to run:
 * 1. Open terminal in the directory containing the file
 * 2. Compile: g++ -fopenmp 02_Parallel_DFS.cpp -o 02_Parallel_DFS
 *    (if above not worked): g++ 02_Parallel_DFS.cpp -o 02_Parallel_DFS
 *    (General command): g++ -fopenmp fileName.cpp -o fileName or g++ fileName.cpp -o fileName
 * 3. Run: ./02_Parallel_DFS or .\02_Parallel_DFS
 */

#include <iostream>
#include <vector>
#include <queue>
#include <omp.h>
#include <chrono>

using namespace std;

// Graph class representing the adjacency list
class Graph
{
    int V;                   // Number of vertices
    vector<vector<int>> adj; // Adjacency list

public:
    Graph(int V) : V(V), adj(V) {}

    // Add an edge to the graph
    void addEdge(int v, int w)
    {
        adj[v].push_back(w);
    }

    // Parallel Depth-First Search
    void parallelDFS(int startVertex)
    {
        vector<bool> visited(V, false);
        parallelDFSUtil(startVertex, visited);
    }

    // Parallel DFS utility function
    void parallelDFSUtil(int v, vector<bool> &visited)
    {
        visited[v] = true;
        cout << v << " ";

#pragma omp parallel for
        for (int i = 0; i < adj[v].size(); ++i)
        {
            int n = adj[v][i];
            if (!visited[n])
                parallelDFSUtil(n, visited);
        }
    }

    // Parallel Breadth-First Search
    void parallelBFS(int startVertex)
    {
        vector<bool> visited(V, false);
        queue<int> q;

        visited[startVertex] = true;
        q.push(startVertex);

        while (!q.empty())
        {
            int v = q.front();
            q.pop();
            cout << v << " ";

#pragma omp parallel for
            for (int i = 0; i < adj[v].size(); ++i)
            {
                int n = adj[v][i];
                if (!visited[n])
                {
#pragma omp critical
                    {
                        if (!visited[n])
                        {
                            visited[n] = true;
                            q.push(n);
                        }
                    }
                }
            }
        }
    }
};

int main()
{
    // ----------- Sample Predefined Graph ------------
    // Create a graph
    // Graph g(10);
    // g.addEdge(0, 1); // 0 -> 1
    // g.addEdge(0, 2); // 0 -> 2
    // g.addEdge(1, 3); // 1 -> 3
    // g.addEdge(1, 4); // 1 -> 4
    // g.addEdge(2, 5); // 2 -> 5
    // g.addEdge(3, 6); // 3 -> 6
    // g.addEdge(3, 7); // 3 -> 7
    // g.addEdge(5, 8); // 5 -> 8
    // g.addEdge(5, 9); // 5 -> 9

    /*
              0
            /   \
           1     2
          / \     \
         3   4     5
        / \         / \
       6   7       8   9
   */

    // ------------- User Input Block (Commented) -------------

    int vertices, edges;
    cout << "Enter number of vertices: ";
    cin >> vertices;
    Graph g(vertices);

    cout << "Enter number of edges: ";
    cin >> edges;
    cout << "Enter edges (from to):\n";
    for (int i = 0; i < edges; ++i)
    {
        int from, to;
        cin >> from >> to;
        g.addEdge(from, to);
    }

    // DFS Timing with chrono
    cout << "Depth-First Search (DFS): ";
    auto dfsStart = chrono::high_resolution_clock::now();
    // double dfsStart = omp_get_wtime();
    g.parallelDFS(0);
    auto dfsEnd = chrono::high_resolution_clock::now();
    // double dfsEnd = omp_get_wtime();
    chrono::duration<double> dfsDuration = dfsEnd - dfsStart;
    cout << "\nDFS Time (chrono): " << dfsDuration.count() << " seconds\n";
    // cout << "DFS Time (omp): " << (dfsEnd - dfsStart) << " seconds\n";

    // BFS Timing with chrono
    cout << "Breadth-First Search (BFS): ";
    auto bfsStart = chrono::high_resolution_clock::now();
    // double bfsStart = omp_get_wtime();
    g.parallelBFS(0);
    auto bfsEnd = chrono::high_resolution_clock::now();
    // double bfsEnd = omp_get_wtime();
    chrono::duration<double> bfsDuration = bfsEnd - bfsStart;
    cout << "\nBFS Time (chrono): " << bfsDuration.count() << " seconds\n";
    // cout << "BFS Time (omp): " << (bfsEnd - bfsStart) << " seconds\n";

    return 0;
}

/*
 * Parallel Breadth First Search (BFS) Implementation using OpenMP
 * =============================================================
 *
 * Overview:
 * ---------
 * This program implements a parallel version of the Breadth-First Search algorithm
 * for traversing graphs. It demonstrates both basic graph operations and parallel
 * processing concepts using OpenMP.
 *
 * Key Technologies:
 * ----------------
 * 1. OpenMP (#pragma omp)
 *    - A parallel programming API for shared-memory multiprocessing
 *    - Used here for parallel task execution and thread management
 *    - Examples in other contexts: parallel for loops, matrix multiplication
 *    - Directives used:
 *      #pragma omp parallel      // Creates parallel region with multiple threads
 *      #pragma omp single nowait // Single thread executes, others don't wait
 *      #pragma omp task          // Creates new task for parallel execution
 *      #pragma omp parallel for  // Distributes loop iterations among threads
 *
 * 2. STL Containers
 *    - vector<>: Dynamic arrays for adjacency lists
 *    - queue<>: FIFO data structure for BFS traversal
 *
 * Data Structures:
 * ---------------
 * 1. Graph (struct)
 *    - Adjacency list representation using vector<vector<int>>
 *    - Allows O(1) access to vertices and O(degree) traversal of neighbors
 *
 * 2. BFS-specific structures:
 *    - visited[]: Boolean array tracking visited vertices
 *    - queue: Maintains vertices to be explored
 *
 * Complexity Analysis:
 * -------------------
 * Time Complexity:
 * - Sequential: O(V + E) where V = vertices, E = edges
 * - Parallel: O((V + E)/p) theoretical, where p = number of processors
 *
 * Space Complexity:
 * - O(V + E) for adjacency list
 * - O(V) for visited array and queue
 *
 * Parallel Performance Factors:
 * ---------------------------
 * 1. Critical Section Overhead
 * 2. Load Balancing
 * 3. Graph Structure/Density
 * 4. Memory Access Patterns
 * 5. Thread Synchronization Cost
 *
 * Q&A Section:
 * -----------
 * Q1: What is BFS and why use it?
 * A1: BFS is a graph traversal algorithm that explores all vertices at the current
 *     depth before moving to vertices at the next depth level. Used in shortest
 *     path finding, web crawling, and network analysis.
 *
 * Q2: Why use adjacency lists over matrices?
 * A2: Adjacency lists are more space-efficient for sparse graphs O(V+E) vs O(V²),
 *     and provide faster iteration over neighbors.
 *
 * Q3: What does #pragma omp parallel for do?
 * A3: It distributes loop iterations across multiple threads, enabling parallel
 *     processing of independent operations.
 *
 * Q4: Why is the critical section necessary?
 * A4: To prevent race conditions when multiple threads try to update shared data
 *     (visited array and queue) simultaneously.
 *
 * Q5: What's the purpose of double-checking visited[v]?
 * A5: It's a performance optimization that reduces critical section contention
 *     by checking visited status before entering the critical section.
 *
 * Q6: How does parallel BFS differ from sequential BFS?
 * A6: Parallel BFS processes multiple neighbors concurrently, but requires
 *     synchronization for shared data access.
 *
 * Q7: What factors affect parallel performance?
 * A7: Graph structure, thread overhead, memory access patterns, load balancing,
 *     and critical section contention.
 *
 * Q8: Why use vector<bool> for visited array?
 * A8: It's space-efficient as it typically uses 1 bit per boolean instead of
 *     1 byte, though it may have performance tradeoffs.
 *
 * Q9: How does load balancing affect performance?
 * A9: Uneven distribution of edges among vertices can lead to some threads
 *     doing more work than others, reducing parallel efficiency.
 *
 * Q10: What's the impact of graph density on performance?
 * A10: Denser graphs have more edges to process but may suffer from increased
 *      critical section contention.
 *
 * Q11: How can we optimize the parallel implementation?
 * A11: Use larger grain sizes, minimize critical sections, employ better
 *      load balancing, and consider graph partitioning.
 *
 * Q12: What's the significance of the queue in BFS?
 * A12: The queue maintains the FIFO order necessary for level-by-level
 *      traversal characteristic of BFS.
 *
 * Q13: How does thread count affect performance?
 * A13: More threads can improve performance up to a point, after which
 *      overhead and contention may degrade performance.
 *
 * Q14: Why is the graph undirected?
 * A14: Each edge is bidirectional (u->v and v->u), suitable for applications
 *      like social networks or road systems.
 *
 * Q15: What are the memory access patterns in this implementation?
 * A15: Random access patterns when accessing adjacency lists and visited array,
 *      which can affect cache performance.
 *
 * Q16: What are the limitations of this implementation?
 * A16: - No handling of disconnected components
 *      - Potential overhead for small graphs
 *      - Memory contention in dense graphs
 *
 * Q17: What is the purpose of #pragma omp critical?
 * A17: Ensures mutual exclusion, allowing only one thread to execute a code block at a time,
 *      preventing race conditions in shared resource access.
 *
 * Q18: How does #pragma omp taskwait work in OpenMP?
 * A18: Forces the current thread to wait until all its child tasks complete,
 *      useful for synchronization points in the algorithm.
 *
 * Q19: What is the role of OMP_NUM_THREADS environment variable?
 * A19: Sets the default number of threads for parallel regions in OpenMP programs,
 *      allowing runtime control of parallelism level.
 *
 * Q20: How does #pragma omp atomic differ from critical?
 * A20: Provides faster, hardware-supported atomic operations for simple updates to shared variables,
 *      more efficient than critical sections for single operations.
 *
 * Q21: What is the purpose of #pragma omp barrier?
 * A21: Creates a synchronization point where all threads in a parallel region must wait
 *      before any can proceed, ensuring coordinated execution.
 *
 */

/*
 * Parallel Depth-First Search (DFS) Implementation using OpenMP
 * ==========================================================
 *
 * Overview:
 * ---------
 * This program implements a parallel version of the Depth-First Search algorithm
 * using OpenMP for graph traversal. It includes both predefined examples and
 * user-input functionality for graph creation and traversal.
 *
 * Key Technologies:
 * ----------------
 * 1. OpenMP (#pragma omp)
 *    - A parallel programming API for shared-memory multiprocessing
 *    - Used here for parallel task execution and thread management
 *    - Examples in other contexts: parallel for loops, matrix multiplication
 *    - Directives used:
 *      #pragma omp parallel      // Creates parallel region with multiple threads
 *      #pragma omp single nowait // Single thread executes, others don't wait
 *      #pragma omp task          // Creates new task for parallel execution
 *      #pragma omp parallel for  // Distributes loop iterations among threads
 *
 * Data Structures:
 * ---------------
 * 1. Graph (struct)
 *    - Adjacency list representation using vector<vector<int>>
 *    - Boolean visited array for tracking traversal
 *
 * Complexity Analysis:
 * -------------------
 * Time Complexity:
 * - Sequential DFS: O(V + E) where V = vertices, E = edges
 * - Parallel DFS: O((V + E)/p) theoretical, where p = number of processors
 *
 * Space Complexity:
 * - O(V) for visited array
 * - O(V + E) for adjacency list
 * - O(V) additional space for recursion stack
 *
 * Parallel Performance Factors:
 * ---------------------------
 * 1. Task Granularity: Balance between parallel overhead and work distribution
 * 2. Graph Structure: Dense vs sparse affects parallelization efficiency
 * 3. Load Balancing: Uneven subtree sizes can lead to workload imbalance
 * 4. Memory Access Patterns: Cache performance and false sharing
 *
 * Q&A Section:
 * -----------
 * Q1: What is the purpose of #pragma omp parallel?
 * A1: Creates a team of threads for parallel execution. Each thread executes the same code.
 *
 * Q2: Why use #pragma omp single nowait?
 * A2: Ensures only one thread starts the initial DFS while others wait for tasks.
 *     'nowait' allows other threads to proceed without synchronization.
 *
 * Q3: What's the role of #pragma omp task?
 * A3: Creates a new task for parallel execution of DFS subtrees.
 *     Example: #pragma omp task { DFSUtil(v, visited); }
 *
 * Q4: How is thread safety ensured in the visited array?
 * A4: Each vertex is marked visited before spawning tasks, preventing duplicate visits.
 *
 * Q5: Why use adjacency list over adjacency matrix?
 * A5: Better space efficiency (O(V+E) vs O(V²)) and faster traversal for sparse graphs.
 *
 * Q6: What's the impact of graph connectivity on parallel performance?
 * A6: Higher connectivity means more potential parallel tasks but also more synchronization overhead.
 *
 * Q7: How does task scheduling work in this implementation?
 * A7: OpenMP runtime schedules tasks dynamically to available threads using a work-stealing algorithm.
 *
 * Q8: What's the purpose of vector<bool> visited?
 * A8: Tracks visited vertices to prevent cycles and repeated processing.
 *
 * Q9: Why is the graph undirected in this implementation?
 * A9: addEdge() adds both (u,v) and (v,u), creating symmetric connections.
 *
 * Q10: How could the implementation be optimized further?
 * A10: - Use task cutoff for small subtrees
 *      - Implement cache-friendly vertex numbering
 *      - Add parallel task merging
 *
 * Q11: What's the significance of #pragma omp parallel for?
 * A11: Distributes loop iterations across threads, useful for processing adjacent vertices.
 *
 * Q12: How does false sharing affect performance?
 * A12: Adjacent elements in visited array accessed by different threads can cause cache line bouncing.
 *
 * Q13: What determines optimal task granularity?
 * A13: Balance between parallelization overhead and work per task. Depends on hardware and graph structure.
 *
 * Q14: How does this compare to sequential DFS?
 * A14: Parallel version can be faster for large graphs but has overhead for small graphs.
 *
 * Q15: What are the limitations of this implementation?
 * A15: - No handling of disconnected components
 *      - Potential overhead for small graphs
 *      - Memory contention in dense graphs
 *
 * Q16: What is the purpose of #pragma omp critical?
 * A16: Ensures mutual exclusion, allowing only one thread to execute a code block at a time,
 *      preventing race conditions in shared resource access.
 *
 * Q17: How does #pragma omp taskwait work in OpenMP?
 * A17: Forces the current thread to wait until all its child tasks complete,
 *      useful for synchronization points in the algorithm.
 *
 * Q18: What is the role of OMP_NUM_THREADS environment variable?
 * A18: Sets the default number of threads for parallel regions in OpenMP programs,
 *      allowing runtime control of parallelism level.
 *
 * Q19: How does #pragma omp atomic differ from critical?
 * A19: Provides faster, hardware-supported atomic operations for simple updates to shared variables,
 *      more efficient than critical sections for single operations.
 *
 * Q20: What is the purpose of #pragma omp barrier?
 * A20: Creates a synchronization point where all threads in a parallel region must wait
 *      before any can proceed, ensuring coordinated execution.
 *
 */