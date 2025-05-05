/*
 * Problem Statement:
 * Write a program to implement Parallel Merge sort using OpenMP.
 * Use existing algorithms and measure the performance of sequential and parallel algorithms.
 *
 * How to run:
 * 1. Open terminal in the directory containing the file
 * 2. Compile: g++ -fopenmp 04_Merge_Sort.cpp -o 04_Merge_Sort
 *    (if above not worked): g++ 04_Merge_Sort.cpp -o 04_Merge_Sort
 *    (General command): g++ -fopenmp fileName.cpp -o fileName or g++ fileName.cpp -o fileName
 * 3. Run: ./04_Merge_Sort or .\04_Merge_Sort
 */

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <omp.h>
using namespace std;

// Merge function
void merge(vector<int> &arr, int l, int m, int r)
{
    vector<int> temp;
    int left = l, right = m + 1;

    while (left <= m && right <= r)
    {
        if (arr[left] <= arr[right])
            temp.push_back(arr[left++]);
        else
            temp.push_back(arr[right++]);
    }

    while (left <= m)
        temp.push_back(arr[left++]);

    while (right <= r)
        temp.push_back(arr[right++]);

    for (int i = l; i <= r; ++i)
        arr[i] = temp[i - l];
}

// Sequential Merge Sort
void sequentialMergeSort(vector<int> &arr, int l, int r)
{
    if (l < r)
    {
        int m = l + (r - l) / 2;
        sequentialMergeSort(arr, l, m);
        sequentialMergeSort(arr, m + 1, r);
        merge(arr, l, m, r);
    }
}

// Parallel Merge Sort
void parallelMergeSort(vector<int> &arr, int l, int r)
{
    if (l < r)
    {
        int m = l + (r - l) / 2;

#pragma omp parallel sections
        {
#pragma omp section
            parallelMergeSort(arr, l, m);

#pragma omp section
            parallelMergeSort(arr, m + 1, r);
        }

        merge(arr, l, m, r);
    }
}

int main()
{
    int n = 100000; // Adjust size to see clear performance difference
    cout << "Generating " << n << " random numbers..." << endl;

    vector<int> arr(n), arr_copy(n);
    srand(time(0));

    for (int i = 0; i < n; ++i)
        arr[i] = rand() % 100000;

    arr_copy = arr; // Copy for parallel version

    auto seqStart = chrono::high_resolution_clock::now();
    sequentialMergeSort(arr, 0, n - 1);
    auto seqEnd = chrono::high_resolution_clock::now();

    auto parStart = chrono::high_resolution_clock::now();
    parallelMergeSort(arr_copy, 0, n - 1);
    auto parEnd = chrono::high_resolution_clock::now();

    cout << "\nFirst 10 elements of sorted array (sequential): ";
    for (int i = 0; i < 10; ++i)
        cout << arr[i] << " ";

    cout << "\nFirst 10 elements of sorted array (parallel): ";
    for (int i = 0; i < 10; ++i)
        cout << arr_copy[i] << " ";

    chrono::duration<double> seqDuration = seqEnd - seqStart;
    chrono::duration<double> parDuration = parEnd - parStart;

    cout << "\n\nSequential Merge Sort time: " << seqDuration.count() << " seconds";
    cout << "\nParallel Merge Sort time:   " << parDuration.count() << " seconds";
    cout << "\nSpeedup: " << seqDuration.count() / parDuration.count() << "x\n";

    return 0;
}

/*
 * MERGE SORT IMPLEMENTATION (SEQUENTIAL VS PARALLEL)
 * ===============================================
 *
 * Overview:
 * ---------
 * This program implements both sequential and parallel versions of the Merge Sort algorithm.
 * It demonstrates the performance benefits of parallel processing by sorting a large array
 * of random integers using both approaches and comparing their execution times.
 *
 * Key Technologies:
 * ----------------
 * 1. OpenMP (#pragma omp)
 *    - A parallel programming API for shared-memory multiprocessing
 *    - Used for parallelizing computationally intensive tasks
 *    - Examples: parallel for loops, sections, tasks
 *    - Other applications: matrix multiplication, image processing
 *
 * 2. Chrono Library
 *    - High-precision timing functionality
 *    - Used for performance measurement
 *    - Provides nanosecond-level accuracy
 *
 * Data Structures:
 * ---------------
 * 1. Vector<int>
 *    - Dynamic array implementation
 *    - Used for main array and temporary storage
 *    - Allows dynamic resizing during merge operations
 *
 * Complexity Analysis:
 * ------------------
 * Time Complexity:
 * - Sequential: O(n log n) - standard merge sort
 * - Parallel: O(n log n / p) where p is number of processors
 *
 * Space Complexity:
 * - O(n) for temporary arrays during merging
 *
 * Parallel Performance Factors:
 * ---------------------------
 * 1. Array Size: Larger arrays benefit more from parallelization
 * 2. Hardware Threads: Performance scales with available cores
 * 3. Overhead: Thread creation and management costs
 * 4. Memory Access Patterns: Cache efficiency impacts performance
 *
 * Q&A Section:
 * -----------
 * Q1: What is OpenMP and how is it used here?
 * A1: OpenMP is a parallel programming API. Here it's used via #pragma omp parallel sections
 *     to split merge sort recursion into parallel tasks.
 *
 * Q2: Why use vector instead of array?
 * A2: Vectors provide dynamic sizing needed for merge operations and are exception-safe.
 *
 * Q3: What's the minimum array size for parallel efficiency?
 * A3: Generally, arrays should be >10000 elements for parallel overhead to be worthwhile.
 *
 * Q4: How does the merge function work?
 * A4: It combines two sorted subarrays by comparing elements and creating a temporary sorted array.
 *
 * Q5: Why use chrono instead of time.h?
 * A5: Chrono provides higher precision and type-safe duration calculations.
 *
 * Q6: What's the purpose of arr_copy?
 * A6: It maintains an identical array for fair comparison between sequential and parallel versions.
 *
 * Q7: How does parallel speedup scale?
 * A7: Theoretically linear with cores, but practically sub-linear due to overhead and memory bottlenecks.
 *
 * Q8: What's the significance of sections vs parallel for?
 * A8: Sections create independent task blocks, while parallel for distributes loop iterations.
 *
 * Q9: How is thread safety ensured?
 * A9: Each thread works on separate array sections, preventing data races.
 *
 * Q10: What limits parallel performance?
 * A10: Memory bandwidth, thread overhead, and load balancing are key limiters.
 *
 * Q11: How to optimize the parallel version?
 * A11: Use task scheduling, adjust grain size, and optimize memory access patterns.
 *
 * Q12: Why use (r-l)/2 instead of /2 directly?
 * A12: Prevents integer overflow for large arrays and maintains proper indexing.
 *
 * Q13: What's the base case for recursion?
 * A13: When l >= r, meaning the subarray has 1 or 0 elements.
 *
 * Q14: How does load balancing work?
 * A14: OpenMP runtime distributes tasks across threads, but subarray sizes may vary.
 *
 * Q15: What's the impact of cache locality?
 * A15: Better cache usage improves performance; sequential access patterns are preferred.
 *
 * Q16: Can this be further parallelized?
 * A16: Yes, the merge operation itself could be parallelized for additional speedup.
 *
 * Q17: Why use srand(time(0))?
 * A17: Ensures different random numbers on each run for realistic testing.
 *
 * Q18: What's the memory overhead?
 * A18: O(n) extra space for temporary arrays during merging, plus thread stack space.
 *
 *
 */