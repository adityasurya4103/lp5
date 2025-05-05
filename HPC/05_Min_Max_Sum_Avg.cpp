/*
 * Problem Statement:
 * Write a program to implement Parallel Merge sort using OpenMP.
 * Use existing algorithms and measure the performance of sequential and parallel algorithms.
 *
 * How to run:
 * 1. Open terminal in the directory containing the file
 * 2. Compile: g++ -fopenmp 05_Min_Max_Sum_Avg.cpp -o 05_Min_Max_Sum_Avg
 *    (if above not worked): g++ 05_Min_Max_Sum_Avg.cpp -o 05_Min_Max_Sum_Avg
 *    (General command): g++ -fopenmp fileName.cpp -o fileName or g++ fileName.cpp -o fileName
 * 3. Run: ./05_Min_Max_Sum_Avg or .\05_Min_Max_Sum_Avg
 */

#include <iostream>
#include <vector>
#include <omp.h>
#include <iomanip>
#include <chrono>
using namespace std;
using namespace std::chrono;

// Pure parallel functions with no internal timing
int parallelMin(const vector<int> &vec)
{
    int min_val = vec[0];
#pragma omp parallel for reduction(min : min_val)
    for (int i = 1; i < vec.size(); i++)
    {
        if (vec[i] < min_val)
        {
            min_val = vec[i];
        }
    }
    return min_val;
}

int parallelMax(const vector<int> &vec)
{
    int max_val = vec[0];
#pragma omp parallel for reduction(max : max_val)
    for (int i = 1; i < vec.size(); i++)
    {
        if (vec[i] > max_val)
        {
            max_val = vec[i];
        }
    }
    return max_val;
}

int parallelSum(const vector<int> &vec)
{
    int sum = 0;
#pragma omp parallel for reduction(+ : sum)
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }
    return sum;
}

float parallelAverage(const vector<int> &vec)
{
    int sum = parallelSum(vec);
    return float(sum) / vec.size();
}

int main()
{
    int n = 1000000;
    cout << "Array size: " << n << endl;

    vector<int> vec(n);
    srand(time(0));
    for (int i = 0; i < n; ++i)
    {
        vec[i] = rand() % 10000;
    }

    auto start = high_resolution_clock::now();
    int min_val = parallelMin(vec);
    auto end = high_resolution_clock::now();
    cout << "Minimum value: " << min_val << endl;
    cout << "Time taken for Min: " << duration_cast<milliseconds>(end - start).count() << " ms\n\n";

    start = high_resolution_clock::now();
    int max_val = parallelMax(vec);
    end = high_resolution_clock::now();
    cout << "Maximum value: " << max_val << endl;
    cout << "Time taken for Max: " << duration_cast<milliseconds>(end - start).count() << " ms\n\n";

    start = high_resolution_clock::now();
    int sum = parallelSum(vec);
    end = high_resolution_clock::now();
    cout << "Sum of values: " << sum << endl;
    cout << "Time taken for Sum: " << duration_cast<milliseconds>(end - start).count() << " ms\n\n";

    start = high_resolution_clock::now();
    float avg = parallelAverage(vec);
    end = high_resolution_clock::now();
    cout << fixed << setprecision(2);
    cout << "Average of values: " << avg << endl;
    cout << "Time taken for Average: " << duration_cast<milliseconds>(end - start).count() << " ms\n";

    return 0;
}

/*
 * Program Overview:
 * This program demonstrates parallel computation of basic array operations (min, max, sum, average)
 * using OpenMP for parallel processing. It generates a random array and performs these operations
 * while measuring execution time.
 *
 * Key Technologies:
 * 1. OpenMP (Open Multi-Processing):
 *    - A parallel programming API for shared memory multiprocessing
 *    - Uses pragma directives for parallelization
 *    - Example: #pragma omp parallel for reduction(op : var)
 *    - Other uses: parallel regions, critical sections, atomic operations
 *
 * 2. Chrono Library:
 *    - High-resolution timing functionality
 *    - Used for performance measurement
 *    - Provides various clock types and duration measurements
 *
 * Data Structures:
 * - std::vector<int>: Dynamic array storing integers
 *   - Contiguous memory allocation
 *   - Random access O(1)
 *   - Dynamic resizing capability
 *
 * Complexity Analysis:
 * Time Complexity:
 * - Sequential: O(n) for all operations
 * - Parallel: O(n/p) where p is number of threads
 * Space Complexity: O(n) for input vector
 *
 * Parallel Performance Factors:
 * 1. Thread Overhead: Creation and management cost
 * 2. Data Locality: Cache utilization
 * 3. Load Balancing: Even distribution of work
 * 4. Reduction Operations: Combining partial results
 * 5. Vector Size: Affects parallelization benefit
 *
 * Q&A Section:
 * Q1: What is OpenMP reduction clause?
 * A1: Reduction manages shared variables in parallel regions by applying an operation
 *     Example: #pragma omp parallel for reduction(+ : sum)
 *
 * Q2: Why use vector instead of array?
 * A2: Vector provides dynamic sizing, bounds checking, and RAII benefits
 *
 * Q3: How does parallel minimum finding work?
 * A3: Each thread finds local minimum, reduction combines using min operation
 *
 * Q4: What's the benefit of high_resolution_clock?
 * A4: Provides finest granularity for time measurement, crucial for performance analysis
 *
 * Q5: Why initialize min_val/max_val with first element?
 * A5: Avoids potential race conditions and ensures valid comparison start
 *
 * Q6: How does parallel reduction affect performance?
 * A6: Reduces thread synchronization overhead by combining local results efficiently
 *
 * Q7: What's the ideal array size for parallelization?
 * A7: Generally >10000 elements to overcome thread overhead costs
 *
 * Q8: Why use float for average calculation?
 * A8: Provides decimal precision for non-integer results
 *
 * Q9: How to control number of threads?
 * A9: Use omp_set_num_threads() or OMP_NUM_THREADS environment variable
 *
 * Q10: What affects parallel speedup?
 * A10: Hardware cores, data size, memory bandwidth, thread overhead
 *
 * Q11: Why measure duration for each operation?
 * A11: Helps analyze performance benefits of parallelization
 *
 * Q12: How to handle false sharing?
 * A12: Use padding or ensure thread-local data alignment
 *
 * Q13: Why use rand() % 10000?
 * A13: Limits random numbers to reasonable range for testing
 *
 * Q14: What's the purpose of fixed and setprecision?
 * A14: Controls floating-point output format for readability
 *
 * Q15: How to optimize parallel performance?
 * A15: Adjust chunk size, minimize critical sections, ensure good load balance
 *
 * Q16: Why separate functions for each operation?
 * A16: Modular design, easier testing and maintenance
 *
 * Q17: What's the impact of cache coherence?
 * A17: Affects performance when multiple threads access shared data
 *
 * Q18: How to handle thread safety?
 * A18: OpenMP handles automatically for reduction operations
 */
