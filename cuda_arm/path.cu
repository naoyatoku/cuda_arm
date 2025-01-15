#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "commonmodule.h"
#include "path.h"
//=============================================================================================
//      軌跡データ
//=============================================================================================
/*


double _path_data[_PATH_DATA_MAX];
int _path_idx;


void init_path()
{
    memset((void*)&_path_data[0] , 0 , sizeof(_path_data) );
    _path_idx = 0;
}
//GPUから軌跡データへ追記します。
void append_path(void* gpu_src, void* gpu_n,int offset)
{
    cudaError_t s;
    int n;
    //データ数の取得です。
    s = cudaMemcpy((void*)&n, gpu_n, sizeof(int), cudaMemcpyDeviceToHost);
    _Assert(s == cudaSuccess, "cudaMalloc failed!" );
    //データ本体の取り込みです。
    _Assert(_path_idx + n < _PATH_DATA_MAX , "path data overflow\n" );
    s = cudaMemcpy((void*)&_path_data[_path_idx+offset] , gpu_src ,  n * sizeof(double), cudaMemcpyDeviceToHost );
    _Assert(s == cudaSuccess, "cudaMalloc failed!");
    //インデックス更新です。
    _path_idx += n;
}
*/