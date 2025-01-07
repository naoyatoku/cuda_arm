#pragma once
#include "CommonModule.h"
#include "arm.h"

//終了条件
enum {
    FIN_SPD,
    FIN_POS_PASS,   //行きすぎた最初で終了（ぴったりでない可能性がある）
};
enum{
    START,
    END
};

//ここは共通でいいと思う
#define N_THREAD    512     //
#define N_BLOCK     72		//
#define N_ALL_THREAD (N_THREAD*N_BLOCK)


void circle_path();         //成功配列から、一番効率のいいものを選択して実際にアームを動作させる
void circle_circle_path();
void circle_path_2();       //

//直線運動      kernel_linear.cu
void line_path();
void _dump_path(cpu_gpu_mem<arm>& path, cpu_gpu_mem<int>& path_idx, bool disp_link = false, int wait = 0);


//円運動        kernel_circle.cu
void circle_path();


//ベジェ曲線	kernel_bezier.cu
//void bezier_path();


//カーネル共通(kernel.cu
/*template<class T>
__host__
bool _kernel(arm * _arm , void  *_p_path_class , float tgt_spd, int dir , _cood *tgt_pos,arm *_path , int *_path_idx , int fin_condition)
{;
}
*/

__device__
void _do_reduction(int* reduction_arr, int arr_size);

