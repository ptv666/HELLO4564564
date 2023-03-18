
//g++ c.cpp -fopenmp -o c.exe

#include <iostream>
#include <omp.h>

using namespace std;

int main() {
    const int N = 1000000;
    int a[N], b[N], c[N];

    // 初始化数组
    for(int i=0; i<N; i++) {
        a[i] = i;
        b[i] = i;
    }

    // 计算两个数组的和（串行）
    double start = omp_get_wtime();
    for(int i=0; i<N; i++) {
        c[i] = a[i] + b[i];
    }
    double end = omp_get_wtime();

    cout << "串行计算时间：" << end - start << " 秒" << endl;

    // 计算两个数组的和（并行）
    start = omp_get_wtime();
    #pragma omp parallel for
    for(int i=0; i<N; i++) {
        c[i] = a[i] + b[i];
    }
    end = omp_get_wtime();

    cout << "并行计算时间：" << end - start << " 秒" << endl;

    return 0;
}
