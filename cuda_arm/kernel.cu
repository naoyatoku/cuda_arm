#include "kernels.h"

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


template<>
__device__
int type<linear>() { return TYPE_LINEAR; }
template<>
__device__
int type<bezier>() { return TYPE_BEZIER; }
