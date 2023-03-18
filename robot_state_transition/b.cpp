/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-18 14:22:44
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-03-18 14:31:44
 * @FilePath: /robot_state_transition/b.cpp
 * @Description: 多线程案例
 */
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <omp.h>
std::mutex mtx; // 互斥量

void sum_array(int *arr, int start, int end, int &result)
{
    int sum = 0;
    for (int i = start; i < end; i++)
    {
        sum += arr[i];
    }
    mtx.lock(); // 加锁
    result += sum;
    mtx.unlock(); // 解锁
}

int main()
{
    const int n = 1000000;
    const int num_threads = 4;

    int arr[n];
    for (int i = 0; i < n; i++)
    {
        arr[i] = i; // 初始化被加数组
    }

    std::vector<std::thread> threads;
    int results[num_threads] = {0};   // 多个子结果
    int chunk_size = n / num_threads; // 子数量
    for (int i = 0; i < num_threads; i++)
    {
        int start = i * chunk_size;
        int end = (i == num_threads - 1) ? n : (i + 1) * chunk_size;
        threads.push_back(std::thread(sum_array, arr, start, end, std::ref(results[i])));
    }
    for (int i = 0; i < num_threads; i++)
    {
        threads[i].join();
    }
    int sum = 0;
    for (int i = 0; i < num_threads; i++)
    {
        sum += results[i];
    }
    std::cout << "sum: " << sum << std::endl;
    return 0;
}
