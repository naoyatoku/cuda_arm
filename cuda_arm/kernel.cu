#include "kernels.h"
#include "draw.h"
#include "stdio.h"
//リダクション.最大のものだけを選択する
__device__
void _do_reduction(int* reduction_arr, int arr_size)
{
    int _idx = threadIdx.x;     //ここで使われるのはスレッドのインデックスです。
    int abre_val = -1;                //あぶれた値。（-1の場合には無効となる。）
    for (int i = arr_size / 2; i > 0; i /= 2) {
        if (_idx < i && reduction_arr[_idx + i] > reduction_arr[_idx]) {
            reduction_arr[_idx] = reduction_arr[_idx + i];    //半分より大きな部分と比較して大きなほうを
        }
        //
        if (_idx == 0) {
            if (abre_val != -1) {   //もしあぶれた数がある場合にはそれを再度評価します。
                if (abre_val > reduction_arr[_idx]) {
                    reduction_arr[_idx] = abre_val;
                }
            }
            //次回のあぶれ数を
            if (i & 1) {   //もし奇数の場合には一つあぶれます。
                abre_val = reduction_arr[i - 1];
            }
            else {
                abre_val = -1;
            }
        }
        __syncthreads();
    }
}

//
__host__
void _dump_path(cpu_gpu_mem<arm>& path, cpu_gpu_mem<int>& path_idx, bool disp_link/*=false*/)
{
    path.Transfer_to_CPU();
    path_idx.Transfer_to_CPU();
    for (int i = 0; i < path_idx(CPU); ++i) {
        const arm& cur = path(CPU, i);
        printf("[%d](%f,%f,%f)spd(%f)\r\n", i, cur.x, cur.y, cur.rad, cur.d.spd);
        if (disp_link) {
            for (int l = 0; l < 3; ++l) {
                printf("        L%d(pos[%f(deg:%f)]spd[%f]\r\n", l, cur.lnk(l).r, RAD2DEG(cur.lnk(l).r), cur.lnk(l).d.spd);
            }
        }
    }
}
static int __a; //デバッグ用
__host__
void _draw_path(cpu_gpu_mem<arm>& path, cpu_gpu_mem<int>& path_idx, int wait/*=1*/)
{
    path.Transfer_to_CPU();
    path_idx.Transfer_to_CPU();
    for (int i = 0; i < path_idx(CPU); ++i) {
        const arm& cur = path(CPU, i);
        draw(cur);
//        ::Sleep(wait);
        //ウェイトを
        for (int i = 0; i < (100000)*wait; ++i) {
            __a++;
        }
//        printf(".");
    }
}



template<>
__device__
int type<linear>() { return TYPE_LINEAR; }
template<>
__device__
int type<bezier>() { return TYPE_BEZIER; }
