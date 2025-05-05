/*
 * Problem Statement:
 * Write a program to implement Parallel Bubble Sort using OpenMP.
 * Use existing algorithms and measure the performance of sequential and parallel algorithms.
 *
 * How to run:
 * 1. Open terminal in the directory containing the file
 * 2. Compile: g++ -fopenmp 03_Bubble_Sort.cpp -o 03_Bubble_Sort
 *    (if above not worked): g++ 03_Bubble_Sort.cpp -o 03_Bubble_Sort
 *    (General command): g++ -fopenmp fileName.cpp -o fileName or g++ fileName.cpp -o fileName
 * 3. Run: ./03_Bubble_Sort or .\03_Bubble_Sort
 */

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <omp.h>
using namespace std;

void sequentialBubbleSort(vector<int> &arr)
{
    int n = arr.size();
    bool swapped = true;
    while (swapped)
    {
        swapped = false;
        for (int i = 0; i < n - 1; i++)
        {
            if (arr[i] > arr[i + 1])
            {
                swap(arr[i], arr[i + 1]);
                swapped = true;
            }
        }
    }
}

void parallelOddEvenSort(vector<int> &arr)
{
    int n = arr.size();
    bool isSorted = false;

    while (!isSorted)
    {
        isSorted = true;

        // Even phase
#pragma omp parallel for shared(arr)
        for (int i = 0; i < n - 1; i += 2)
        {
            if (arr[i] > arr[i + 1])
            {
                swap(arr[i], arr[i + 1]);
                isSorted = false;
            }
        }

        // Odd phase
#pragma omp parallel for shared(arr)
        for (int i = 1; i < n - 1; i += 2)
        {
            if (arr[i] > arr[i + 1])
            {
                swap(arr[i], arr[i + 1]);
                isSorted = false;
            }
        }
    }
}

int main()
{
    int n = 1000;
    cout << "Generating " << n << " random numbers..." << endl;

    vector<int> arr(n), arr_copy(n);
    srand(time(0));
    for (int i = 0; i < n; i++)
    {
        arr[i] = rand() % 10000;
    }
    arr_copy = arr;

    auto seqStart = chrono::high_resolution_clock::now();
    sequentialBubbleSort(arr);
    auto seqEnd = chrono::high_resolution_clock::now();

    auto parStart = chrono::high_resolution_clock::now();
    parallelOddEvenSort(arr_copy);
    auto parEnd = chrono::high_resolution_clock::now();

    cout << "\nFirst 10 elements of sorted array (sequential): ";
    for (int i = 0; i < 10; i++)
        cout << arr[i] << " ";

    cout << "\nFirst 10 elements of sorted array (parallel): ";
    for (int i = 0; i < 10; i++)
        cout << arr_copy[i] << " ";
    cout << endl;

    chrono::duration<double> seqDuration = seqEnd - seqStart;
    chrono::duration<double> parDuration = parEnd - parStart;

    cout << "\nSequential Bubble Sort time: " << seqDuration.count() << " seconds";
    cout << "\nParallel Odd-Even Sort time: " << parDuration.count() << " seconds" << endl;
    cout << "Speedup: " << seqDuration.count() / parDuration.count() << "x" << endl;

    return 0;
}

/*
 * Parallel vs Sequential Bubble Sort Implementation
 * ==============================================
 *
 * Overview:
 * This program implements and compares sequential bubble sort with a parallel odd-even sort
 * algorithm. It generates random numbers, sorts them using both methods, and measures performance.
 *
 * Key Technologies:
 * 1. OpenMP (#pragma omp)
 *    - A parallel programming API for shared-memory multiprocessing
 *    - Used for parallelizing loops in the odd-even sort
 *    - Example: Also used in parallel matrix multiplication, parallel for loops
 *
 * 2. Chrono Library
 *    - High-resolution timing library in C++
 *    - Used for precise performance measurements
 *    - Common in benchmarking applications
 *
 * Data Structures:
 * - vector<int>: Dynamic array implementation
 *   - Used for storing and sorting numbers
 *   - Provides contiguous memory access
 *   - O(1) random access
 *
 * Complexity Analysis:
 * Sequential Bubble Sort:
 * - Time: O(n²) worst and average case, O(n) best case
 * - Space: O(1) auxiliary space
 *
 * Parallel Odd-Even Sort:
 * - Time: O(n²) sequential steps, but divided by number of threads
 * - Space: O(1) auxiliary space
 *
 * Parallel Performance Factors:
 * 1. Thread Overhead: Creation and management cost
 * 2. Data Sharing: Cache coherency and false sharing
 * 3. Load Balancing: Even distribution of work
 * 4. Memory Access Patterns: Contiguous vs scattered
 * 5. Problem Size: Should be large enough to justify parallelization
 *
 * Q&A Section:
 * ============
 * Q1: What is OpenMP and how is it used here?
 * A1: OpenMP is a parallel programming API. Here it's used with #pragma omp parallel for
 *     to parallelize the sorting loops.
 *
 * Q2: Why use odd-even sort for parallelization instead of regular bubble sort?
 * A2: Odd-even sort allows for better parallelization as elements can be compared
 *     and swapped independently in each phase.
 *
 * Q3: What's the difference between shared and private variables in OpenMP?
 * A3: Shared variables (like 'arr' here) are accessible by all threads, while
 *     private variables are thread-specific.
 *
 * Q4: How does the parallel implementation ensure thread safety?
 * A4: By using separate even and odd phases and ensuring no overlapping comparisons.
 *
 * Q5: What's the significance of the 'isSorted' flag?
 * A5: It helps terminate the sorting when no more swaps are needed, improving efficiency.
 *
 * Q6: Why use vector instead of array?
 * A6: Vectors provide dynamic sizing and better memory management, plus STL compatibility.
 *
 * Q7: How does false sharing affect performance?
 * A7: When threads modify data in the same cache line, it causes cache invalidation
 *     and reduced performance.
 *
 * Q8: What's the impact of array size on parallel performance?
 * A8: Larger arrays benefit more from parallelization due to better thread utilization.
 *
 * Q9: How is speedup calculated?
 * A9: Speedup = Sequential time / Parallel time
 *
 * Q10: Why use high_resolution_clock?
 * A10: It provides the most precise timing measurements available in the chrono library.
 *
 * Q11: What's the purpose of arr_copy?
 * A11: It ensures both algorithms sort identical data for fair comparison.
 *
 * Q12: How does the parallel sort handle race conditions?
 * A12: By separating odd and even phases and ensuring independent element access.
 *
 * Q13: What's the memory access pattern in bubble sort?
 * A13: Sequential, adjacent element access, which is cache-friendly.
 *
 * Q14: How could the parallel implementation be improved?
 * A14: By adjusting chunk sizes, using SIMD instructions, or implementing hybrid approaches.
 *
 * Q15: Why is the parallel version not always faster?
 * A15: Thread overhead and synchronization costs might exceed benefits for small arrays.
 *
 * Q16: What is the main principle behind odd-even sort?
 * A16: Odd-even sort alternates between comparing and swapping odd-indexed and even-indexed
 *      pairs of adjacent elements.
 *
 * Q17: How does odd-even sort differ from bubble sort?
 * A17: Odd-even sort processes elements in two distinct phases, making it more suitable
 *      for parallel implementation than bubble sort's sequential passes.
 *
 * Q18: What is the stability of odd-even sort?
 * A18: Like bubble sort, odd-even sort is stable, maintaining the relative order
 *      of equal elements.
 *
 * Q19: Can odd-even sort be vectorized?
 * A19: Yes, the independent comparisons in each phase make it suitable for SIMD
 *      (Single Instruction Multiple Data) vectorization.
 */
