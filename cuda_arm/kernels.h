#pragma once
#include "CommonModule.h"
#include "arm.h"
#include "linear.h"
#include "bezier.h"

//終了条件
enum {
    FIN_SPD,
    FIN_POS_PASS,   //行きすぎた最初で終了（ぴったりでない可能性がある）
};
enum{
    START,
    END
};

//方向を表す共通定義です。
enum{
   	DIR_P=1, 
   	DIR_M=-1,
};

//カーネルの終了状態をbool以外に表現できるようにします。
enum {
    KERNEL_RUNNING = 0,
    KERNEL_FIN_OK = 1,
    KERNEL_FIN_NG_NO_AVAIL_MOVE,
    KERNEL_FIN_NG_PATH_OVERFLOW,
    KERNEL_FIN_NG_CUDA_ERROR,     //CUDAのエラー
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

//ベジェ曲線運動 kernel_bezier.cu
void bezier_path();

__device__
void _do_reduction(int* reduction_arr, int arr_size);




//=================================================================================
//  テンプレート実装です。
//=================================================================================

enum {
    TYPE_LINEAR = 0,
    TYPE_BEZIER,
};

//デバイスコードで型毎にことなる処理を行うためのしくみ
template<class T>
__device__ int type();



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

//共通で使えるCUDAコードをおいておきます。arit
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
                if(dir == DIR_P){   //プラス方向に進行している場合、
                    if( _arm->x >= tgt_pos->x ){
                        return true;
                    }
                }else{
                    if( _arm->x <= tgt_pos->x ){
                        return true;
                    }
                }
                break;
            case TYPE_BEZIER:
                //ベジェの場合はt==1.0になったらおしまいだけど、ぴったりにならないきもする
                if(_arm->read_add_info<bezier_pos>().t >= tgt_pos->read_add_info<bezier_pos>().t){  //目標位置より進んでいたら
                    return true;
                }                
                break;
        }
        break;
        default:
            _gpuAssert(0,"check_condition():unknown fin_condition\r\n");break;
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
        bezier_pos src_pos = _arm->read_add_info<bezier_pos>();
        c = _cood ( ((const bezier*)p_path_class)->progress(GPU, src_pos , dx,dir, pos), _arm->rad);   //デバッグ用に変数にします。
//        c = _cood ( ((const bezier*)p_path_class)->progress(GPU, _arm->read_add_info<bezier_pos>(), dx, pos), _arm->rad);   //デバッグ用に変数にします。
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
    __shared__ int _reduction[N_THREAD];        //int*512 -> 2kbyte

    //ブロックとスレッドの組み合わせで成功失敗の
    int idx = (blockIdx.x * blockDim.x) + threadIdx.x;                                          //
    link_stat s = _arm->move_able(_tgt_cood<T>(_arm, _p_path_class , tgt_spd,dir, idx, N_ALL_THREAD));     //

    _reduction[threadIdx.x] = s.stat == MOV_OK ? idx : -1;     //成功したら自分のインデックスを入れる失敗したら
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    _do_reduction(_reduction, blockDim.x);
    //
    //ここでブロック内のチャンピオンデータが_reduction[0]に保管されている。自分のブロックデータを更新します。
    if(threadIdx.x==0){
        block_result[blockIdx.x] = _reduction[0];
#if 0   //
            printf("bl[%d]a[%d]r[%d]\t", blockIdx.x,acc, _reduction[0]);
#endif
    }    
}

template<class T>
__global__
void _move_kernel(arm* _arm , const void* _p_path_class ,int* block_result , float tgt_spd ,_cood *tgt_pos ,  int dir, int* fin,arm* _path, int* _path_idx ,const int path_n_max, int fin_condition)
{
    //まずブロック毎のデータの中でさらに一番いいものを選択する。
    _do_reduction(block_result , blockDim.x );

    if(threadIdx.x==0){ //
        //もしベジェの場合、一つも動けない場合は、t=1.0で切られて急ブレーキがかかる場合があり、その場合、これ以上すすめなくなります。この場合は
//        _gpuAssert(block_result[0] >= 0, "move_kernel():no available result\r\n");       //有効なインデックスがなかった場合はエラーです。
        if(block_result[0] < 0){
            *fin = KERNEL_FIN_NG_NO_AVAIL_MOVE;
            goto _fin;
        }
        _cood tgt =_tgt_cood<T>( _arm , _p_path_class , tgt_spd,dir,block_result[0],N_ALL_THREAD);
        _arm->move(tgt);                                   //動かす。
        _path[(*_path_idx)++] = *_arm;    //アーム記録してもいいかも

        if (*_path_idx > path_n_max -1) {  //overflow
            printf("\r\npath overflow\r\n");
            *fin = KERNEL_FIN_NG_PATH_OVERFLOW;
            goto _fin;
        }
        *fin = check_condition<T>(_arm, tgt_spd, dir, tgt_pos, fin_condition) ==true ? KERNEL_FIN_OK : KERNEL_RUNNING;
    }
_fin:
    __syncthreads();    //これいらないか？    
}

template<class T>
__host__
int _kernel(arm * _arm , void  *_p_path_class , float tgt_spd, int dir , _cood *tgt_pos,arm *_path , int *_path_idx , const int path_n_max ,int fin_condition)
{
    const dim3 threads( N_THREAD);
    const dim3 blocks(N_BLOCK);
    cpu_gpu_mem<int>	block_result(N_BLOCK);		//これが全部の結果を保持するようにします。一度の
    cpu_gpu_mem<int>fin;

    esc_clr();  //画面はクリアする。

    int i = 0;
    //※アーム状態はカーネル呼び出し前に設定してください。
    for (fin(CPU) = KERNEL_RUNNING;  fin(CPU) == KERNEL_RUNNING ; fin.Transfer_to_CPU() ) {      //この処理に時間がかかるのでこのループ自体も
        if (i == 5065) {
            printf("break%d\r\n", i++);
        }
        printf("[kernl]%d\r\n",i++);
        _calc_kernel<T> << <blocks, threads >> > ( _arm , _p_path_class , block_result.gpu , tgt_spd, dir);
        cudaDeviceSynchronize();
        if (_check_kernl_error() != true) {
            fin(CPU)=KERNEL_FIN_NG_CUDA_ERROR;
            break;
        }
        //この時点でベストが見つかっているとする。
        _move_kernel<T> << <1, blocks >> > (_arm , _p_path_class , block_result.gpu , tgt_spd, tgt_pos , dir , fin.gpu , _path , _path_idx ,path_n_max ,fin_condition );
        cudaDeviceSynchronize();
        if (_check_kernl_error() != true) {
            fin(CPU)=KERNEL_FIN_NG_CUDA_ERROR;
            break;
        }
    }
    return fin(CPU);
}
