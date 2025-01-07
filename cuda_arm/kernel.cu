#include "kernels.h"
#include "linear.h"
#include "bezier.h"

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
/*

enum {
    TYPE_LINEAR = 0,
    TYPE_BEZIER,
};

//デバイスコードで型毎にことなる処理を行うためのしくみ
template<class T>
__device__ int type();
template<>
__device__
int type<linear>(){   return TYPE_LINEAR;}
template<>
__device__
int type<bezier>(){   return TYPE_BEZIER;}



__host__
static bool _check_kernl_error()
{
    cudaError_t s;
    s = cudaGetLastError();
    if (s != cudaSuccess) {
        printf("kernel failed: %s\n", cudaGetErrorString(s));
        return false;
    } //
    return true;
}

//共通で使えるCUDAコードをおいておきます。
template<class T>
__device__
bool check_condition(const arm* _arm , float tgt_spd , int dir , _cood *tgt_pos , int fin_condition)
{
    switch(fin_condition){
    case FIN_SPD:
        if (_equal<float>(_arm->d.spd, tgt_spd,0.1) == true) {
            return true;
        }
        break;
    case  FIN_POS_PASS:
        switch(type<T>()){
            case TYPE_LINEAR:
                if(dir == linear::DIR_P){   //プラス方向に進行している場合、
                    if( _arm->x >= tgt_pos->x ){
                        *fin=true;
                    }
                }else{
                    if( _arm->x <= tgt_pos->x ){
                        *fin=true;
                    }
                }
                break;
            case TYPE_BEZIER:
                //ベジェの場合はt==1.0になったらおしまいだけど、ぴったりにならないきもする
                if(_arm->read_add_info<bezier_pos>().t >= tgt_pos->read_add_info<bezier_pos>().t){  //目標位置より進んでいたら
                    *fin = true;
                }
                break;
        }
        break;
        default:
            _Assert(0,"check_condition():unknown fin_condition\r\n");break;
    }
    return false;
}




//各軌跡関数に応じたターゲット位置の計算です。
template<class T>
__device__ inline
static _cood _tgt_cood(const arm* _arm, const void* p_path_class , float tgt_spd, int dir, int idx, int n_all_thread)
{
    float d_spd = tgt_spd - _arm->d.spd;                             //目標速度にむけての加速度です。(mm/sec)
    //-----------------------------------------------------------------------------------------------------------------
    //  ※toku 
    //  ここは、現在速度～目標速度の間だけだと、減速が効かないので、0もしくは、あるていどの減速までを範囲に含める必要がある。
    //-----------------------------------------------------------------------------------------------------------------
    //　(現在速度＋目標速度への加速度)/1000 (1msあたりに換算)
    float dx = ((d_spd / n_all_thread) * idx) / 1000; //
    //デバッグしやすいように計算を分けます。
    dx += _arm->d.spd / 1000;   //現在速度で1mあたりに進む距離
    //dxこれが現在速度からの加速分です。現在速度の
    _cood c;
    //仮想関数が使えないのでここは枝分かれです。
    switch (type<T>()) {
    case TYPE_LINEAR:
        c = _cood( ((const linear*)p_path_class)->progress(*_arm,dx,dir) , _arm->rad);   //デバッグ用に変数にします。    
        break;

    case TYPE_BEZIER:
        bezier_pos pos;
        c = _cood ( ((const bezier*)p_path_func)->progress(GPU, _arm->read_add_info<bezier_pos>(), dx, pos), _arm->rad);   //デバッグ用に変数にします。
        //cにベジェの位置情報を付加します。
        c.write_add_info<bezier_pos>(pos);
        break;
    }
    return c;
}


template<class T>
__global__
void _calc_kernel(arm* _arm, const void* _p_path_class , int* block_result, float tgt_spd, int dir)
{
    __shared__ extern int _reduction[N_THREAD];        //int*512 -> 2kbyte

    //ブロックとスレッドの組み合わせで成功失敗の
    int idx = (blockIdx.x * blockDim.x) + threadIdx.x;                                          //
    link_stat s = _arm->move_able(_tgt_cood<T>(_arm, _p_path_class , tgt_spd,dir, idx, N_ALL_THREAD));     //

    _reduction[threadIdx.x] = s.stat == MOV_OK ? idx : -1;     //成功したら自分のインデックスを入れる失敗したら
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    _do_reduction(_reduction, blockDim.x);
}

template<class T>
__global__
void _move_kernel(arm* _arm , const void* _p_path_class ,int* block_result , float tgt_spd ,_cood *tgt_pos ,  int dir, bool* fin,arm* _path, int* _path_idx , int fin_condition)
{
    //まずブロック毎のデータの中でさらに一番いいものを選択する。
    _do_reduction(block_result , blockDim.x );

    if(threadIdx.x==0){ //
        _gpuAssert(block_result[0] >= 0, "move_kernel():no available result\r\n");       //有効なインデックスがなかった場合はエラーです。
        _cood tgt =_tgt_cood<T>( _arm , _p_path_class , tgt_spd,dir,block_result[0],N_ALL_THREAD);
        _arm->move(tgt);                                   //動かす。
        _path[(*_path_idx)++] = *_arm;    //アーム記録してもいいかも

        if (*_path_idx > MAX_PATH-1) {  //overflow
            printf("\r\npath overflow\r\n");
            *fin = true;
            goto _fin;
        }
        *fin = check_condition<T>(_arm, tgt_spd, dir, tgt_pos, fin_condition);
    }
_fin:
    __syncthreads();    //これいらないか？    
}

template<class T>
__host__
bool _kernel(arm * _arm , void  *_p_path_class , float tgt_spd, int dir , _cood *tgt_pos,arm *_path , int *_path_idx , int fin_condition)
{
    const dim3 threads( N_THREAD);
    const dim3 blocks(N_BLOCK);
    cpu_gpu_mem<int>	block_result(N_BLOCK);		//これが全部の結果を保持するようにします。一度の
    cpu_gpu_mem<bool>fin;

    esc_clr();  //画面はクリアする。


    //※アーム状態はカーネル呼び出し前に設定してください。
    for (fin(CPU) = false;  fin(CPU) != true; fin.Transfer_to_CPU() ) {      //この処理に時間がかかるのでこのループ自体も
        _calc_kernel<T> << <blocks, threads >> > ( _arm , _p_path_class , block_result.gpu , tgt_spd, dir);
        cudaDeviceSynchronize();
        if (_check_kernl_error() != true) {
            goto _error;
        }
        //この時点でベストが見つかっているとする。
        _move_kernel<T> << <1, blocks >> > (_arm , _p_path_class , block_result.gpu , tgt_spd, tgt_pos , dir , fin.gpu , _path , _path_idx ,fin_condition );
        cudaDeviceSynchronize();
        if (_check_kernl_error() != true) {
            goto _error;
        }
    }

    return true;
_error:
    return false;
}
*/