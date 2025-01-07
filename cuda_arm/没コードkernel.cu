
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include "CommonModule.h"
#include "draw.h"
#include "circle.h"
#include "arm.h"
#include "path.h"

#define N_THREAD        512         //一ブロックで実行するスレッドの数はdefineでcudaカーネルでも使えるようにします。
#define N_BLOCK         512

#define GPU_PATH_MAX    2048        //GPU側に送るパスのバッファサイズです。
#define PATH_MAX        4096        //CPU側でまとめて保存するバッファサイズです。_path_data[]本体の

//円形に動作する
// 一定速動作、
//減速停止

//cudaMallocは、グローバルメモリに保存される
//まず大きなグローバルメモリをとり、一回分の計算を格納する。
//違うカーネルで全ブロック分の最大の分布をみて、経路を完成させる。

//現在の位置から _arm 
__global__ void circle_calc_kernel(arm* _arm , circle* _c ,int* succeed , int *succeed_blc ,int dir,double init_spd ,  float spd )
{
    //初速と目標速度との関係で減速するのか、加速するのかを決めます。
    __shared__ bool             acc;                    //加速するかどうか
    __shared__ double           spd_span;               //速度の幅
    __shared__ int              default_idx;            //
    __shared__ int     reduction[N_THREAD];     //

    //ブロックとスレッドの組み合わせで成功失敗の
    int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
    //__shared__メモリの初期化です.各ブロックに
    if (threadIdx.x == 0) {
        acc = (spd > init_spd) ? true : false;    //加減速スイッチです。
        spd_span = fabs(spd - init_spd);          //加減速による速度の幅です。（初速度～目標速度間の幅）
        default_idx = acc ? -1 : (blockDim.x*gridDim.x);        //（成功しない場合に
    }
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    //今回チャレンジする移動距離をdtとします。
    double dt = dir * (spd_span / (N_BLOCK*N_THREAD)) * (idx + (acc ? 1 : 0)) / 1000; //1ms分なので、 spd rad/sec -> rad/msec に修正        idx=511で bloclDim.xとなるように
    if (blockIdx.x == 511 && threadIdx.x == 511) {          //debug stopeer
        reduction[threadIdx.x] = succeed[idx];               //高速にリダクションするためにsharedに入れます。
    }
    link_stat s = _arm->move_able(_cood(_c->progress_rad(*_arm, dt, dir), _arm->rad + dt));


    succeed[idx] = s.stat == MOV_OK ? idx : default_idx;     //成功したら自分のインデックスを入れる
    reduction[threadIdx.x] = succeed[idx];               //高速にリダクションするためにsharedに入れます。

    //※失敗の場合のデフォルト値は、減速の場合には小さいほうが採用されるように大きな値にしておく。
    // （idx : 0 - blockDim.x-1 なので、 ）
//     __device__ __host__ vec2 progress_rad(const vec2 & start, double rad, int dir = 0) const;
//        printf("[%d]:%d %d\n", idx, result[idx], succeed[idx]);
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    {   //リダクション。減速と加速で条件が違います。
        //条件分岐が少ないように重複して書いてみる。本当にパフォーマンス変わるのか
        int _idx = threadIdx.x;     //ここで使われるのはスレッドのインデックスです。
        if (acc) {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && reduction[_idx + i] > reduction[_idx]) {
                    reduction[_idx] = reduction[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        else {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && reduction[_idx + i] < reduction[_idx]) {       //減速の場合は小さいほうを選択する。
                    reduction[_idx] = reduction[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        //ここでreduction[0]が、このブロック内で一番いい値です。
        if (threadIdx.x == 0) { 
            succeed_blc[blockIdx.x] = reduction[0]; 
        }
    }

}
//成功配列から、一番効率のいいものを選択して実際にアームを動作させる
__global__ void circle_move_kernel(arm* _arm, circle* _c, int* succeed, int* succeed_blc,  double* path, int* n_path,int *fin , int dir , double e_rad , double init_spd , double spd,bool pos , int kernelno=0)
{
    __shared__ bool             acc;                    //加速するかどうか
    __shared__ double           spd_span;               //速度の幅
    __shared__ int              default_idx;            //
    //ブロックとスレッドの組み合わせで成功失敗の
#if 1       //debug
    if (kernelno == 20) {       //20番カーネルが、arm状態があっているかどうか
        __syncthreads();
    }
#endif

    int idx = threadIdx.x;
    //__shared__メモリの初期化です.各ブロックに
    if (threadIdx.x == 0) {
        acc = (spd > init_spd) ? true : false;                  //加減速スイッチです。
        spd_span = fabs(spd - init_spd);                        //加減速による速度の幅です。（初速度～目標速度間の幅）
//        default_idx = acc ? -1 : (blockDim.x * gridDim.x);          //（成功しない場合に
        default_idx = acc ? -1 : 0x7fffffff;    //int最大
    }
    __syncthreads();
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    int adopted_idx;        //これが採用する
    {   //リダクション。減速と加速で条件が違います。
        //条件分岐が少ないように重複して書いてみる。本当にパフォーマンス変わるのか
        int _idx = threadIdx.x;     //ここで使われるのはスレッドのインデックスです。
        if (acc) {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && succeed_blc[_idx + i] > succeed_blc[_idx]) {
                    succeed_blc[_idx] = succeed_blc[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        else {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && succeed_blc[_idx + i] < succeed_blc[_idx]) {       //減速の場合は小さいほうを選択する。
                    succeed_blc[_idx] = succeed_blc[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        //ここでscceed_[0]が、このブロック内で一番いい値です。
        if (threadIdx.x == 0) {
            adopted_idx = succeed_blc[0];   //succeed_blcには、ブロック、スレッドを通したインデックスが入っています。
        }
    }

    //選択されたインデックスでアームを動かし、その結果条件にあっているかを判定します。
    if (threadIdx.x == 0) {
        if (adopted_idx < 0) {  //一つも動かせない場合
            printf("cant move\n");     *fin = -1;       goto error;
        }
//        if (pos && (*n_path > 105)) {            printf("-");        }
        double dt = dir * (spd_span / (N_BLOCK * N_THREAD)) * (adopted_idx + (acc ? 1 : 0)) / 1000;     //1ms分なので、 spd rad/sec -> rad/msec に修正        idx=511で bloclDim.xとなるように
        _arm->move(_cood(_c->progress_rad(*_arm, dt, dir), _arm->rad + dt));                            //動かす。
//        printf("[%d]t:%lf,spd:%lf,acc%lf,dlt:%lf\n", *n_path, _arm->rad, _arm->wd.spd, _arm->wd.acc, fabs(_arm->wd.spd) - spd);
        // 記録します。(dt)
        path[(*n_path)++] = dt;

        //===============================================
        //  終了条件の判定をします。
        //===============================================
        if (pos) {
            //位置の場合には、ピッタリとまらないことがあるので、行きすぎたら終わる。
//              //もしくは、次の加速で
            if ((dir == CIRCLE_DIR_CW && _arm->rad > e_rad)
                || (dir == CIRCLE_DIR_CCW && _arm->rad < e_rad)) {
                printf("pos toutatu!!!\n");
                *fin = (int)true;
            }
            else {
                *fin = (int)false;
            }
        }
        else {
            //動かしたところで、目標到達速度に達すると終了。目標速度
            if (_equal( (double)fabs(_arm->wd.spd), (double)spd,0.0005)) {   //精度はちょっと甘くしてみます。
                printf("spd toutatu!!! (%d)-(%d)(%d)\n" , kernelno,blockIdx.x,threadIdx.x);
                *fin = (int)true;
            }
            else {
                *fin = (int)false;
            }
        }
    }
    return;
error:
    //finに結果が表示されます。
    return;
}



#define N_SEARCH    32     //着地探索計算の回数。

//
void circle_path_2()
{
//    const int threads   = 512;    //
//    const int blocks    =   128;    //

    circle* p_c;
    arm* p_arm;

    //計算分を格納するバッファを作ります。
    static  int    succeed[N_THREAD * N_BLOCK];       //staticとしてみる。
    static  int    succeed_blc[N_BLOCK];       //staticとしてみる。
    int *p_succeed;                                //デバイス側です。
    int* p_succeed_blc;                          //ブロック単位でのリダクション結果を格納するための

    //経路データです。
    double* p_path;     //GPUに渡す軌跡データ
    int* p_n_path;     //GPUで計算した軌跡の数
    cudaError_t s;

    //アームが目的の状態になったかどうか（カーネルが判断した結果を格納します。)
    //int fin;
    int* p_fin;

    //==========================================================================
    //  軌跡となる円を設定します。
    //==========================================================================
    circle c(100, 50, 50);


    //==========================================================================
    init_path();                    //軌跡の記録データのセットアップです。
    //==========================================================================


    //試しに、0から90度まで、初速0 - 最高速 90/sec- 終了速度 0 で動ける軌跡を探索する。
    //======= 条件です ==============================
    double s_rad = 0;               //姿勢も角度と同じとしてみます。
    double e_rad = PI / 2;            //b
    double spd = PI / 2;              //PI/2の速度まで
    //======= 条件です ==============================
    //armサークルオブジェクトをgpuへ転送
    {
        //
        s = cudaMalloc((void**)&p_c, sizeof(circle));                               _Assert(s == cudaSuccess, "cudaMalloc failed!");
        s = cudaMemcpy(p_c, (void*)&c, sizeof(circle), cudaMemcpyHostToDevice);     _Assert(s == cudaSuccess, "cudaMalloc failed!");        //
        s = cudaMalloc((void**)&p_arm, sizeof(arm));                                _Assert(s == cudaSuccess, "cudaMalloc failed!");        //
        //失敗成功(全データを見られるように)    ※
        s = cudaMalloc((void**)&p_succeed       , sizeof(succeed));                  _Assert(s == cudaSuccess, "cudaMalloc failed!");
        s = cudaMalloc((void**)&p_succeed_blc   , sizeof(succeed_blc));              _Assert(s == cudaSuccess, "cudaMalloc failed!");
        //ブロック単位での成功失敗を格納するためのメモリです。
//        s = cudaMalloc()
        //経路の記録。
        s = cudaMalloc((void**)&p_path, sizeof(double) * PATH_MAX);                  _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データ
        s = cudaMalloc((void**)&p_n_path, sizeof(int));                              _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データの数
        //カーネルの判定用。
        s = cudaMalloc((void**)&p_fin, sizeof(int));                                 _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データの数
    }

    //==========================================================================================================
    //   到達位置から逆に加速させ、目標速度に到達するようにする。
    //==========================================================================================================
    double dec_start_pos;       //減速開始位置です。
    {
        _arm.set(_cood(c.rf(e_rad), e_rad));           //アームを終了位置へセットします。
        s = cudaMemcpy(p_arm, (void*)&_arm, sizeof(arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");

        for (int fin = 0 ; !fin ;) {
            //一回分のカーネルです。
            {
                circle_calc_kernel << <N_BLOCK, N_THREAD >> > (p_arm, p_c, p_succeed, p_succeed_blc, CIRCLE_DIR_CCW, 0 , spd);
                cudaDeviceSynchronize();       s = cudaGetLastError();        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
            }
#if 1   //このカーネルではp_succeedに

#endif
            //ここで各ブロックの集計結果をまとめて、一番いいものを選択し、実際にアームを動作させ、軌跡を記録します。
            {
                circle_move_kernel << <1, N_BLOCK >> > (p_arm, p_c, p_succeed, p_succeed_blc, p_path, p_n_path, p_fin, CIRCLE_DIR_CCW , e_rad , 0 ,spd ,  false);        //速度が到達したらおしまい。
                cudaDeviceSynchronize();        //終了待ち
            }
            //結果をCPUに転送して評価していきます。ここが一番時間がかかる。
            { s = cudaMemcpy((void*)&fin, (void*)p_fin, sizeof(int), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
        }
        printf("save path data..");
        //    append_path(p_path, p_n_path);  //軌跡データです。
        //現時点のアームの位置が、おおよその減速開始位置です。
        s = cudaMemcpy((void*)&_arm, p_arm, sizeof(arm), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMalloc failed!");

        //
        dec_start_pos = _arm.rad;
        printf("fin\n");
    }
    //==========================================================================================================
    //  初期位置から減速位置まで動作させていく。
    //  探索のために、ちょっと行きすぎるようにしてみます。
    //==========================================================================================================
    init_path();        //軌跡はクリアです。

    //
    {   //GPU側の軌跡カウンタクリアと、アームの初期位置のセットです。
        int n_path = 0;
        s = cudaMemcpy(p_n_path, (void*)&n_path, sizeof(int), cudaMemcpyHostToDevice);     _Assert(s == cudaSuccess, "cudaMalloc failed!");        //
        _arm.set(_cood(c.rf(s_rad), s_rad));           //アームを開始位置へセットします。
        s = cudaMemcpy(p_arm, (void*)&_arm, sizeof(arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
    }
    printf("start search path...");
    {   //探索
        int i = 0;
//        double ov = fabs( e_pos - dec_start_pos)     ;        //少し行きすぎる場所まで記録する。
        for (int fin = 0; !fin; ++i) {
            //一回分のカーネルです。
            {
                circle_calc_kernel << <N_BLOCK, N_THREAD >> > (p_arm, p_c, p_succeed, p_succeed_blc, CIRCLE_DIR_CW, 0 , spd);
                cudaDeviceSynchronize();    s = cudaGetLastError();        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
            }
/*            if (i>105) {
                { s = cudaMemcpy((void*)&succeed[0], (void*)p_succeed, sizeof(succeed), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
                { s = cudaMemcpy((void*)&succeed_blc[0], (void*)p_succeed_blc, sizeof(succeed_blc), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
            }
            */
            //ここで各ブロックの集計結果をまとめて、一番いいものを選択し、実際にアームを動作させ、軌跡を記録します。
            {
                //※ちょっと行きすぎる場所まで
                circle_move_kernel << <1, N_BLOCK >> > (p_arm, p_c, p_succeed, p_succeed_blc, p_path, p_n_path, p_fin, CIRCLE_DIR_CW, dec_start_pos + _rad(1), 0, spd, true);        //位置を目標に
                cudaDeviceSynchronize();        //終了待ち
            }
            //結果をCPUに転送して評価していきます。ここが一番時間がかかる。
            { s = cudaMemcpy((void*)&fin, (void*)p_fin, sizeof(int), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
        }
        printf("fin(loop : %d)\n", i);
    }
    //軌跡をアップロードします。
    append_path(p_path, p_n_path);  //これでコピーします
    //==========================================================================================================
    //  ここで、_path_data[_n_path] には０～radまでの軌跡が記録されている。
    //  後半のいくつかを使って、そこから全力で減速させてみる。
    //  もっとも目的位置に近い場所て停止した軌跡を採用するようにする。
    //==========================================================================================================
    {
        //
        static arm _s_arm[N_SEARCH];                 //探索開始位置の

        //一応探索結果も
        static  int    s_succeed[N_SEARCH*N_THREAD * N_BLOCK];       //staticとしてみる。
        static  int    s_succeed_blc[N_SEARCH*N_BLOCK];       //staticとしてみる。
        arm* p_s_arm;          //

        int* p_s_succeed;                                //デバイス側です。
        int* p_s_succeed_blc;                          //ブロック単位でのリダクション結果を格納するための

        double* p_s_path;       //GPUに渡す軌跡データ
        int* p_n_s_path;        //GPUで計算した軌跡の数

        //各軌跡計算の終了状況。
        int* p_s_fin;
        int s_fin[N_SEARCH];

        //=============================================================================
        //      アーム、経路データは探索回数分用意します。メモリ
        //      
        //=============================================================================
        {
            s = cudaMalloc((void**)&p_s_arm, sizeof(_s_arm));                                _Assert(s == cudaSuccess, "cudaMalloc failed!");        //

            //GPUで記録される軌跡データです。
            s = cudaMalloc((void**)&p_s_path, sizeof(double) * PATH_MAX * N_SEARCH);            _Assert(s == cudaSuccess, "cudaMalloc failed!");
            s = cudaMalloc((void**)&p_n_s_path, sizeof(int) * N_SEARCH);                          _Assert(s == cudaSuccess, "cudaMalloc failed!");        //
            //
            //失敗成功(全データを見られるように)    ※
            s = cudaMalloc((void**)&p_s_succeed, sizeof(s_succeed));                                  _Assert(s == cudaSuccess, "cudaMalloc failed!");
            s = cudaMalloc((void**)&p_s_succeed_blc, sizeof(s_succeed_blc));                          _Assert(s == cudaSuccess, "cudaMalloc failed!");

            //終了
            s = cudaMalloc((void**)&p_s_fin, sizeof(int) * N_SEARCH);                                 _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データの数
        }

        //軌跡一つは1ms分のデータです。最後の128msくらいまでを試します。s
        int st = max( _path_idx - N_SEARCH , 0);
        _arm.set(_cood(c.rf(s_rad), s_rad));                                                                                                                  //アームをスタート位置へ移動
        for (int i = 0; i < st; ++i) {  _arm.move(_cood(c.progress_rad(_arm, _path_data[i], CIRCLE_DIR_CW), _arm.rad + _path_data[i]));     }                 //
        //ここから終点まではアーム状態を記録していきます。

        for (int i =st ; i < _path_idx ; ++i) {       //
            _arm.move(_cood(c.progress_rad(_arm, _path_data[i], CIRCLE_DIR_CW), _arm.rad + _path_data[i]));     //アームを動かし、
            _s_arm[i-st] = _arm;      //探索開始位置として記録する。
        }

        //ここで _s_arm[] に 減速開始位置までの  「N_SEARCH ms」分のアームが保持されている。
        //N_SEARCH分のarm状態から減速停止していき、それらを


        //gpuに転送します。
        {
            s = cudaMemcpy(p_s_arm, (void*)&_s_arm, sizeof(_s_arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        }
        //============================================================================================
        //  ここから探索していく。探索分カーネルを起動できればうれしいが
        //============================================================================================
        for (int i = 0; i < N_SEARCH; ++i) s_fin[i] = false;     //fin初期化です。
        for (int i=0; ; ++i) {
            printf("[%d]",i);


            //速度０になるまで、おのおののアーム状態（s_arm[t] : gpu:p_s_arm[t]）を動作させるカーネルを
            {
                for (int t = 0; t < N_SEARCH; ++t) {        //複数個カーネルを起動します。
                        //すでに終了しているカーネルは起動しない
                    if (s_fin[t])continue;
                        circle_calc_kernel << <N_BLOCK, N_THREAD >> > (  &p_s_arm[t]
                            , p_c
                            , &p_s_succeed[t * N_THREAD * N_BLOCK]
                            , &p_s_succeed_blc[t * N_BLOCK]
                            , CIRCLE_DIR_CW
                            , _s_arm[t].wd.spd ,0);
                }
                cudaDeviceSynchronize();    s = cudaGetLastError();        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
            }
#if 1
            if (1) {
               { s = cudaMemcpy((void*)&s_succeed[0]        ,   (void*)p_s_succeed      ,   sizeof(s_succeed)       , cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
               { s = cudaMemcpy((void*)&s_succeed_blc[0]    ,   (void*)p_s_succeed_blc  ,   sizeof(s_succeed_blc)   , cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }                
            }
#endif

            //これがmoveカーネル。サーチ分動作させる。
            //ここで各ブロックの集計結果をまとめて、一番いいものを選択し、実際にアームを動作させ、軌跡を記録します。
            {
                //※ちょっと行きすぎる場所まで
                for (int t = 0; t < N_SEARCH; ++t) {
                    if (s_fin[t])continue;
                    circle_move_kernel << <1, N_BLOCK >> > (
                          &p_s_arm[t] 
                        , p_c
                        , &p_s_succeed[t * N_THREAD * N_BLOCK]
                        , &p_s_succeed_blc[t * N_BLOCK] 
                        , &p_s_path[ t * PATH_MAX]
                        , &p_n_s_path[ t ]
                        , &p_s_fin[t]
                        , CIRCLE_DIR_CW, 0, _s_arm[t].wd.spd, 0, false , t);        //速度が０になるのを
                }
                cudaDeviceSynchronize();    s = cudaGetLastError();        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
            }
            //結果をCPUに転送して評価していきます。ここが一番時間がかかる。
            { s = cudaMemcpy((void*)&s_fin[0], (void*)p_s_fin, sizeof(int)*N_SEARCH, cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
            //終わっていないものがなければ終了します。
            for (int i = 0; i < N_SEARCH; ++i) {       if(!s_fin[i] ) goto _loop;        }
            break;      //何も実行するものがなくなった。
        _loop:;
        }
        //--------------------------------------------------
        //      ここで終了です。一番目標位置に近い場所を探します。
        //--------------------------------------------------
        int min_idx;
        {
            //止まっているアーム状態を取り出します。
            { s = cudaMemcpy((void*)&_s_arm[0], (void*)p_s_arm, sizeof(_s_arm)  , cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
            //
            double min = 10;
            for (int i = 0; i < N_SEARCH; ++i) {
                if (fabs(_s_arm[i].rad - e_rad) < min) {
                    min = fabs(_s_arm[i].rad - e_rad);
                    min_idx = i;
                }
            }
              
        }
        //ここでmin_idxが最強のパスでしたので、それを足していきます。
//        このままじゃだめです。min_idxの場所のパスから足さないといけない。
//       差し込む場所は、、「データ末尾 - N_SEARCH + min_idxの場所」
        append_path(&p_s_path[min_idx* PATH_MAX], &p_n_s_path[min_idx] , min_idx - N_SEARCH+1);



        //メモリ解放を・・・
        {
            cudaFree((void*)p_s_arm);
            cudaFree((void*)p_s_succeed);                               //デバイス側です。
            cudaFree((void*)p_s_succeed_blc);                           //ブロック単位でのリダクション結果を格納するための
            cudaFree((void*)p_s_path);                                  //GPUに渡す軌跡データ
            cudaFree((void*)p_n_s_path);                                //GPUで計算した軌跡の数
            cudaFree((void*) p_s_fin);                                  //
        }

    }

    //
    {
        printf("free gpu memory..");
        cudaFree( (void*)p_c);
        cudaFree( (void*)p_arm);
        cudaFree( (void*) p_succeed);
        cudaFree( (void*)p_succeed_blc);
        cudaFree( (void*)p_path);
        cudaFree( (void*)p_n_path);
        cudaFree((void*)p_fin);
        printf("fin\n");
    }

    //ここで計算した軌跡にそって動作させてみます。
    
    {
        printf("arm move..");
        _arm.set(_cood(c.rf(s_rad), s_rad));           //アームをスタート位置へ移動。
        for (int i = 0; i < _path_idx; ++i) {
            if (i > 962) {
                ::Sleep(1);
            }
            _arm.move(_cood(c.progress_rad(_arm, _path_data[i], CIRCLE_DIR_CW), _arm.rad + _path_data[i]));
            //各関節の指示データを保存していきますか
            _arm.save();        //ファイルに保存してみます。
            ::Sleep(1);

        }
        printf("fin\n");
    }

    printf("all fin\n");
    return;
Error:;
    printf("failed");

}



//======================================================================================================================================== 
//  最初の試み   スレッド数だけで分割するので512分割するパターン。ブロックの中で実行できるスレッドが512で制限されてしまった。
//========================================================================================================================================
__global__ void circle_kernel(arm* _arm, circle* _c , double *path ,int *n_path, int dir,  double  e_rad, double spd , bool pos/*位置を優先して終了する*/)
{
    const int nmem = 1024;
    //    printf("in kernel");
//    __shared__ short    result[nmem];                  //結果符号拡張してくれるかしら
    __shared__ short    succeed[nmem];     //成功失敗
    __shared__ bool     fin;                //終了フラグ

    //初速と目標速度との関係で減速するのか、加速するのかを決めます。
    __shared__ bool     acc;                //加速するかどうか
    __shared__ double   spd_span;           //速度の幅
    __shared__ int      default_idx;        //

    //ブロックとスレッドの組み合わせで
    int idx = threadIdx.x;      //512
    int blc = blockIdx.x;       //512

    //初期化
    if (idx == 0) {
        fin = false;        //
        acc = (spd > _arm->wd.spd) ? true : false;      //加減速スイッチです。
        spd_span = fabs(spd - _arm->wd.spd);            //加減速による速度の幅です。（初速度～目標速度間の幅）
        default_idx =   acc ? -1 : (blockDim.x);        //
        *n_path = 0;                                    //データ個数の初期化です。
    }
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    for (int loop = 0 ; fin!=true ;++loop ) {
        if (loop > 500) {
            fin = false;
        }
        //現在のアーム地点から指示速度
        //減速するのはどうするんだろうか
        double dt =dir *  (spd_span / blockDim.x)*(idx+ (acc?1:0) ) / 1000; //1ms分なので、 spd rad/sec -> rad/msec に修正        idx=511で bloclDim.xとなるように
        //加速の場合、idx==511で最高速(spd)にならないといけないので+1.
        //減速の場合、idx==0 で再低速（spdにならないといけないので+0）

        link_stat s = _arm->move_able(_cood(_c->progress_rad(*_arm, dt, dir), _arm->rad + dt  ));
        //    mem[idx] =    _arm->move_able(_cood(_c->progress_rad(*_arm, dt , CIRCLE_DIR_CW) , _arm->rad +dt )).stat==MOV_OK ? 1:0;
//        result[idx] = s.stat;
        succeed[idx] = s.stat == MOV_OK ? idx : default_idx;     //成功したら自分のインデックスを入れる。（リダクションのため）
                                                                                    //※失敗の場合のデフォルト値は、減速の場合には小さいほうが採用されるように大きな値にしておく。
                                                                                    // （idx : 0 - blockDim.x-1 なので、 ）
        //     __device__ __host__ vec2 progress_rad(const vec2 & start, double rad, int dir = 0) const;
//        printf("[%d]:%d %d\n", idx, result[idx], succeed[idx]);
        __syncthreads();        //あるブロック内部のスレッドの同期です。

        //===================================================================================
        //      ここで、一番多く動かせたのを探します。
        //      一番大きく動作できたものを選択して、_armにセットする。
        //===================================================================================
        {   //リダクション。減速と加速で条件が違います。
            //条件分岐が少ないように重複して書いてみる。本当にパフォーマンス変わるのか
            if (acc) {
                for (int i = blockDim.x / 2; i > 0; i /= 2) {
                    if (idx < i && succeed[idx + i] > succeed[idx]) {
                        succeed[idx] = succeed[idx + i];    //半分より大きな部分と比較して大きなほうを
                    }
                    __syncthreads();
                }
            }
            else {
                for (int i = blockDim.x / 2; i > 0; i /= 2) {
                    if (idx < i && succeed[idx + i] < succeed[idx]) {       //減速の場合は小さいほうを選択する。
                        succeed[idx] = succeed[idx + i];    //半分より大きな部分と比較して大きなほうを
                    }
                    __syncthreads();
                }
            }
        }
        //ここで一つも進める候補がみつからない場合には終わります。
        if (succeed[0] == default_idx) {
            printf("no succeed idx(fin)\n");
            goto fin;   //
        }

        //最後、succeed[0]に一番大きな添え字が残っているので、そこへ移動させる。
        if (idx == succeed[0] ) {
//            double dt = (spd / blockDim.x) * idx / 1000; //1ms分なので、 spd rad/sec -> rad/msec に修正
            _arm->move(_cood(_c->progress_rad(*_arm, dt, dir), _arm->rad + dt ));
            printf("%d,t:%lf,spd:%lf,acc%lf,dlt:%lf\n" , loop ,_arm->rad ,  _arm->wd.spd , _arm->wd.acc , fabs(_arm->wd.spd)- spd);
           // 記録します。(dt)
            path[(*n_path)++] = dt;

            //===============================================
            //  終了条件の判定をします。
            //===============================================
            if ( pos ) {
                //位置の場合には、ピッタリとまらないことがあるので、行きすぎたら終わる。
//              //もしくは、次の加速で
                if ((dir == CIRCLE_DIR_CW && _arm->rad > e_rad)
                    || (dir == CIRCLE_DIR_CCW && _arm->rad < e_rad)) {
                    printf("pos toutatu!!!\n");
                    fin = true;
                }
            }
            else {
                //動かしたところで、目標到達速度に達すると終了。目標速度
                if (_equal<float>(fabs(_arm->wd.spd), spd)) {
                    printf("spd toutatu!!!\n");
                    fin = true;
                    //                break;
                }
            }
        }
        __syncthreads();

        //とりあえずここで目標速度に到達したら終了としてみる。
    }   //次にすすみます。
fin:;
    if(idx==0)    printf("[%d]kernel fin\n" ,blockIdx.x);   //一回だけ
}

//カーネル呼び出し
void circle_path()
{
    circle* p_c;
    arm* p_arm;
    //まず一回の計算分を格納するバッファを作ります。
//    char 


    //
    double* p_path;     //GPUに渡す軌跡データ
    int    *p_n_path;     //GPUで計算した軌跡の数
    cudaError_t s;

    //==========================================================================
    //  軌跡となる円を設定します。
    //==========================================================================
    circle c(100, 50, 50);


    //==========================================================================
    init_path();                    //軌跡の記録データのセットアップです。
    //==========================================================================


    //試しに、0から90度まで、初速0 - 最高速 90/sec- 終了速度 0 で動ける軌跡を探索する。
    //======= 条件です ==============================
    double s_rad = 0;               //姿勢も角度と同じとしてみます。
    double e_rad = PI/2;            //b
    double spd = PI/2;              //PI/2の速度まで
    //======= 条件です ==============================
    //armサークルオブジェクトをgpuへ転送
    {
        s = cudaMalloc((void**)&p_c         ,   sizeof(circle));                                _Assert(s == cudaSuccess, "cudaMalloc failed!");
        s = cudaMemcpy(p_c, (void*)&c, sizeof(circle), cudaMemcpyHostToDevice);                 _Assert(s == cudaSuccess, "cudaMalloc failed!");        //
        s = cudaMalloc((void**)&p_arm       ,   sizeof(arm));                                   _Assert(s == cudaSuccess, "cudaMalloc failed!");        //

        s = cudaMalloc((void**)&p_path      ,   sizeof(double) * GPU_PATH_MAX);                 _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データ
        s = cudaMalloc((void**)&p_n_path    ,   sizeof(int));                                   _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データの数
    }
    //経路の加速減速を区間を決めていくか？無理ですよね。

    //どうやるのか。とりあえず、始点から始めて、一区間分で動ける位置を探すのを１スレッドでやるか
//    circle_kernel << <1, 512 >> > (p_arm, p_c, CIRCLE_DIR_CW, e_rad, spd);
    //==========================================================================================================
    //   到達位置から逆に加速させ、目標速度に到達するようにする。
    //==========================================================================================================
    double dec_start_pos;       //減速開始位置です。
    {
        _arm.set(_cood(c.rf(e_rad), e_rad));           //アームを終了位置へセットします。
        s = cudaMemcpy(p_arm, (void*)&_arm, sizeof(arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        circle_kernel << <1, 512 >> > (p_arm , p_c , p_path , p_n_path ,  CIRCLE_DIR_CCW, e_rad, spd, false);   //減速開始位置の計算です。
        s = cudaGetLastError();        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
        //結果をCPUへ転送します。
        s = cudaMemcpy((void*)&_arm, p_arm, sizeof(arm), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        dec_start_pos = _arm.rad;
    }
//    addKernel << <1, size >> > (dev_c, dev_a, dev_b);
    //=====================================================================================
    //  ためしに、減速開始位置から速度を反転させて減速させてみる。
    //=====================================================================================
    //ここで、停止位置から逆方向に加速して、速度が到達したところで、終了とする。
    //ここでの_armの位置が減速開始位置になる。
    //ここでアーム状態を書きもどす。
    //減速させてみる。
//    _arm.stop();            //逆方向に
    if(0)
    {
        _arm.reverse();     //減速開始位置でマイナス方向に加速したので反転します。(目標速度で)

        s = cudaMemcpy(p_arm, (void*)&_arm, sizeof(arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        circle_kernel << <1, 512 >> > (p_arm, p_c ,  p_path, p_n_path , CIRCLE_DIR_CW, e_rad, 0,false);        //目標速度を0としてやってみる。
        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
        //データを取り出します。
    }
    //=====================================================================================
    // 初期位置から、減速開始位置まで
    //=====================================================================================
#if 1
    {
        _arm.set(_cood(c.rf(s_rad), s_rad) );           //アームをスタート位置へ移動。
        s = cudaMemcpy(p_arm, (void*)&_arm, sizeof(arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");

        circle_kernel << <1, 512 >> > (p_arm, p_c , p_path, p_n_path , CIRCLE_DIR_CW, dec_start_pos , spd , true);        //目標速度を0としてやってみる。
        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
        //結果をCPUへ移動します。
//  debug(armの状態が知りたければ)
//        s = cudaMemcpy((void*)&_arm, p_arm, sizeof(arm), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        //ここでは同じか超えている場合
        append_path(p_path, p_n_path);  //これでコピーします
    }
    //※本当は一つ前がいいが
    //=====================================================================================
    // ここから減速させます。
    //=====================================================================================
    //ひとまず速度０まで落としてみます。
    {
        circle_kernel << <1, 512 >> > (p_arm, p_c , p_path, p_n_path , CIRCLE_DIR_CW, e_rad , 0 ,false );        //目標速度を0としてやってみる。
        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
        //結果をCPUへ移動します。
        s = cudaMemcpy((void*)&_arm, p_arm, sizeof(arm), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        append_path(p_path, p_n_path);  //これでコピーします
    }
    //ここで、目的位置

    //ピッタリの場所にいくように低い速度でうごかす最低速度で（即停止できる）
    double min_spd = spd / 400;  //toku ※いつでも止まれる速度を計算する必要がある。(それぞれのリンクの加速度が)
    {
        circle_kernel << <1, 512 >> > (p_arm, p_c, p_path, p_n_path, ( e_rad > _arm.rad) ?  CIRCLE_DIR_CW : CIRCLE_DIR_CCW , e_rad ,min_spd , true );        //目標速度を0としてやってみる。
        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
        //結果をCPUへ移動します。
        s = cudaMemcpy((void*)&_arm, p_arm, sizeof(arm), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
        append_path(p_path, p_n_path);  //これでコピーします
    }
#endif

    cudaFree((void*)p_c);
    cudaFree((void*)p_arm);

    //
    //ここで計算した軌跡にそって動作させてみます。
    {

        _arm.set(_cood(c.rf(s_rad), s_rad));           //アームをスタート位置へ移動。
        for (int i = 0; i < _path_idx; ++i) {
            _arm.move(_cood(c.progress_rad(_arm, _path_data[i], CIRCLE_DIR_CW), _arm.rad + _path_data[i]));
            //各関節の指示データを保存していきますか
            _arm.save();        //ファイルに保存してみます。
            ::Sleep(1);
        }
    }
//    draw_thread();
#if 0
    arm
      __host__ __device__ void set(_cood pos) { move(pos, false); stop(); }//

    cood:
    _cood(vec2 v, double _rad, spd_acc _wd = spd_acc()) :vec2(v), rad(_rad), wd(_wd) { ; }

    circle::
    __device__ __host__ vec2 rf(double rad)	const {
        return vec2(rvec2(_r, rad)) + _org;
    }
#endif

    //

    printf("success fin");
    return;

Error:;
    return;

}

//サブルーチン
__device__ __host__ _cood _m(arm* _arm, circle* _c,double dt,int dir,double rate)
{
    _cood m;
    {
        double _t;
        //c[1]の軌道にすすんだあと、c[0]のangleを評価しても_armが円周上にない。
        //現在の制御では、アームがどの円周上でも一様に角度が増加するのでそれを利用する。PI/2以上の場合は円➁になるという判断です。
        if ((_t = ((_arm->rad + dt*rate) - PI / 2)) > 0) {  //円①の軌道上での90度を超える場合、
            //円周２の
            m = _cood(_c[1].rf(PI * 3 / 2 - _t), _arm->rad + (dt * rate));       //アーム角度はそのまま継続して増加させてみます。
            //最終的に
//^^^^^^^^^  角度を足すのをやめてみる。
        }
        else {
            m = _cood(_c[0].progress_rad(*_arm, dt, dir), _arm->rad + dt);
        }
    }
    return m;
}

__global__ void circle_circle_calc_kernel(arm* _arm, circle* _c, int* succeed, int* succeed_blc, int dir, double init_spd, double spd,double rate)
{
    //初速と目標速度との関係で減速するのか、加速するのかを決めます。
    __shared__ bool             acc;                    //加速するかどうか
    __shared__ double           spd_span;               //速度の幅
    __shared__ int              default_idx;            //
    __shared__ int     reduction[N_THREAD];     //

    //ブロックとスレッドの組み合わせで成功失敗の
    int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
    //__shared__メモリの初期化です.各ブロックに
    if (threadIdx.x == 0) {
        acc = (spd > init_spd) ? true : false;    //加減速スイッチです。
        spd_span = fabs(spd - init_spd);          //加減速による速度の幅です。（初速度～目標速度間の幅）
        default_idx = acc ? -1 : (blockDim.x * gridDim.x);        //（成功しない場合に

    }
    __syncthreads();        //あるブロック内部のスレッドの同期です。

#if 1   //debug code(break point用)
    if (blockIdx.x == 468 ) {          //debug stopeer
        reduction[threadIdx.x] = succeed[idx];               //高速にリダクションするためにsharedに入れます。
    }
#endif

    //今回チャレンジする移動距離をdtとします。
    double dt = dir * (spd_span / (N_BLOCK * N_THREAD)) * (idx + (acc ? 1 : 0)) / 1000; //1ms分なので、 spd rad/sec -> rad/msec に修正        idx=511で bloclDim.xとなるように

    //もしここで境界を超える場合には、
    //今回のdtで、円１の範囲を超える場合には、ポインタ
    //_dtは角度、_armのvec2が現在の円周上の座標。
    //進む位置と角度を決定します。
    _cood m;
#if 0
    {
        double _t;
        //c[1]の軌道にすすんだあと、c[0]のangleを評価しても_armが円周上にない。
        //現在の制御では、アームがどの円周上でも一様に角度が増加するのでそれを利用する。PI/2以上の場合は円➁になるという判断です。
        if (  ( _t  = ( (_arm->rad +  dt) - PI/2) ) > 0 )  {  //円①の軌道上での90度を超える場合、
            //円周２の
            m = _cood(_c[1].rf(PI * 3 / 2 - _t), _arm->rad + (dt*1.2) );       //アーム角度はそのまま継続して増加させてみます。
                                                                        //最終的に
                                                //^^^^^^^^^  角度を足すのをやめてみる。
        }
        else {
            m = _cood(_c[0].progress_rad(*_arm, dt, dir), _arm->rad + dt);
        }
    }
#else
    m = _m(_arm, _c, dt, dir,rate);  //cirlce_moveカーネルと処理を共通化するため
#endif

    link_stat s = _arm->move_able(m);


    //※ここは減速でだめになりますので注意
#if 1
 /*   enum {
        //
        MOV_SPD_OVER = 3,		//最大速度を超えて動かそうとした
        MOV_ACC_OVER = 2,		//加速が大きすぎてNG
        MOV_DEC_OVER = 1,		//減速が大きすぎてNG
        MOV_OK = 0,				//成功
    };*/
    //succeedの値には全リンクのステータスを書きます。
    if (s.stat == MOV_OK) {
//        succeed[idx] = s.stat == MOV_OK ? idx : (default_idx * ((s.stat * 10) + s.no));     //成功したら自分のインデックスを入れる
        succeed[idx] = s.stat == MOV_OK ? idx :default_idx ;     //成功したら自分のインデックスを入れる
    }
    else {      //
        //全リンクのNGの理由を調べます。
        link_stat s[3];
        _arm->move_able(m, 1.0,s);
        succeed[idx] = default_idx * ((s[2].stat * 100000 + (s[2].no * 10000)) + (s[1].stat * 1000 + s[1].no * 100) + ((s[0].stat * 10) + s[0].no));
    }
#else
    succeed[idx] = s.stat == MOV_OK ? idx : default_idx ;     //成功したら自分のインデックスを入れる
#endif
    reduction[threadIdx.x] = succeed[idx];               //高速にリダクションするためにsharedに入れます。

    //※失敗の場合のデフォルト値は、減速の場合には小さいほうが採用されるように大きな値にしておく。
    // （idx : 0 - blockDim.x-1 なので、 ）
//     __device__ __host__ vec2 progress_rad(const vec2 & start, double rad, int dir = 0) const;
//        printf("[%d]:%d %d\n", idx, result[idx], succeed[idx]);
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    {   //リダクション。減速と加速で条件が違います。
        //条件分岐が少ないように重複して書いてみる。本当にパフォーマンス変わるのか
        int _idx = threadIdx.x;     //ここで使われるのはスレッドのインデックスです。
        if (acc) {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && reduction[_idx + i] > reduction[_idx]) {
                    reduction[_idx] = reduction[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        else {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && reduction[_idx + i] < reduction[_idx]) {       //減速の場合は小さいほうを選択する。
                    reduction[_idx] = reduction[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        //ここでreduction[0]が、このブロック内で一番いい値です。
        if (threadIdx.x == 0) {
            succeed_blc[blockIdx.x] = reduction[0];
        }
    }

}
//成功配列から、一番効率のいいものを選択して実際にアームを動作させる
__global__ void circle_circle_move_kernel(arm* _arm, circle* _c, int* succeed, int* succeed_blc, double* path, int* n_path, int* fin, int dir, double e_rad, double init_spd, double spd, bool pos, double rate,int kernelno = 0)
{
    __shared__ bool             acc;                    //加速するかどうか
    __shared__ double           spd_span;               //速度の幅
    __shared__ int              default_idx;            //
    //ブロックとスレッドの組み合わせで成功失敗の
#if 1       //debug
    if (kernelno == 20) {       //20番カーネルが、arm状態があっているかどうか
        __syncthreads();
    }
#endif

    int idx = threadIdx.x;
    //__shared__メモリの初期化です.各ブロックに
    if (threadIdx.x == 0) {
        acc = (spd > init_spd) ? true : false;                  //加減速スイッチです。
        spd_span = fabs(spd - init_spd);                        //加減速による速度の幅です。（初速度～目標速度間の幅）
        //        default_idx = acc ? -1 : (blockDim.x * gridDim.x);          //（成功しない場合に
        default_idx = acc ? -1 : 0x7fffffff;    //int最大
    }
    __syncthreads();
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    int adopted_idx;        //これが採用する
    {   //リダクション。減速と加速で条件が違います。
        //条件分岐が少ないように重複して書いてみる。本当にパフォーマンス変わるのか
        int _idx = threadIdx.x;     //ここで使われるのはスレッドのインデックスです。
        if (acc) {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && succeed_blc[_idx + i] > succeed_blc[_idx]) {
                    succeed_blc[_idx] = succeed_blc[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        else {
            for (int i = blockDim.x / 2; i > 0; i /= 2) {
                if (_idx < i && succeed_blc[_idx + i] < succeed_blc[_idx]) {       //減速の場合は小さいほうを選択する。
                    succeed_blc[_idx] = succeed_blc[_idx + i];    //半分より大きな部分と比較して大きなほうを
                }
                __syncthreads();
            }
        }
        //ここでscceed_[0]が、このブロック内で一番いい値です。
        if (threadIdx.x == 0) {
            adopted_idx = succeed_blc[0];   //succeed_blcには、ブロック、スレッドを通したインデックスが入っています。
        }
    }

    //選択されたインデックスでアームを動かし、その結果条件にあっているかを判定します。
    if (threadIdx.x == 0) {
        if (adopted_idx < 0) {  //一つも動かせない場合
            printf("cant move\n");     *fin = -1;       goto error;
        }
        //        if (pos && (*n_path > 105)) {            printf("-");        }
        double dt = dir * (spd_span / (N_BLOCK * N_THREAD)) * (adopted_idx + (acc ? 1 : 0)) / 1000;     //1ms分なので、 spd rad/sec -> rad/msec に修正        idx=511で bloclDim.xとなるように

        _cood m = _m(_arm, _c, dt, dir,rate);  //cirlce_moveカーネルと処理を共通化するため
        _arm->move(m);                            //動かす。
        //      _arm->move(_cood(_c->progress_rad(*_arm, dt, dir), _arm->rad + dt));                            //動かす。
        //        printf("[%d]t:%lf,spd:%lf,acc%lf,dlt:%lf\n", *n_path, _arm->rad, _arm->wd.spd, _arm->wd.acc, fabs(_arm->wd.spd) - spd);
                // 記録します。(dt)
        path[(*n_path)++] = dt;

        //===============================================
        //  終了条件の判定をします。
        //===============================================
        if (pos) {
            //位置の場合には、ピッタリとまらないことがあるので、行きすぎたら終わる。
//              //もしくは、次の加速で
            if ((dir == CIRCLE_DIR_CW && _arm->rad > e_rad)
                || (dir == CIRCLE_DIR_CCW && _arm->rad < e_rad)) {
                printf("pos toutatu!!!\n");
                *fin = (int)true;
            }
            else {
                *fin = (int)false;
            }
        }
        else {
            //動かしたところで、目標到達速度に達すると終了。目標速度
            if (_equal<float>(fabs(_arm->wd.spd), spd, 0.0005)) {   //精度はちょっと甘くしてみます。
                printf("spd toutatu!!! (%d)-(%d)(%d)\n", kernelno, blockIdx.x, threadIdx.x);
                *fin = (int)true;
            }
            else {
                *fin = (int)false;
            }
        }
    }
    return;
error:
    //finに結果が表示されます。
    return;
}

static int __a; //
//異なる円に接続する挑戦をしてみる。
void circle_circle_path()
{
    //    const int threads   = 512;    //
    //    const int blocks    =   128;    //

    circle* p_c;

    arm* p_arm;

    //計算分を格納するバッファを作ります。
    static  int    succeed[N_THREAD * N_BLOCK];       //staticとしてみる。
    static  int    succeed_blc[N_BLOCK];       //staticとしてみる。
    int* p_succeed;                                //デバイス側です。
    int* p_succeed_blc;                          //ブロック単位でのリダクション結果を格納するための

    //経路データです。
    double* p_path;     //GPUに渡す軌跡データ
    int* p_n_path;     //GPUで計算した軌跡の数
    cudaError_t s;

    //アームが目的の状態になったかどうか（カーネルが判断した結果を格納します。)
    //int fin;
    int* p_fin;

    //==========================================================================
    //  軌跡となる円を設定します。
    //==========================================================================
    circle c[2] = { circle(100, 50, 50)  , circle(100,50,250)};

    //==========================================================================
    init_path();                    //軌跡の記録データのセットアップです。
    //==========================================================================
    //試しに、0から90度まで、初速0 - 最高速 90/sec- 終了速度 0 で動ける軌跡を探索する。
    //======= 条件です ==============================
    double s_rad = 0;               //姿勢も角度と同じとしてみます。
    double e_rad = PI;              //pi相当の位置まで動かします。
                                    //  PI/2以降は、円2に遷移して、PI/2 - PI を、 
                                    //      3PI/2 - PI までマイナスにすすませるか
                                    //      (3PI/2) - ( Θ  - PI/2)　=>  2PI - Θ かな?
    double spd = PI/4;                //PI/2の速度まで
    //======= 条件です ==============================
    //armサークルオブジェクトをgpuへ転送
    {
        //円をコピーです。
        s = cudaMalloc((void**)&p_c, sizeof(c));                               _Assert(s == cudaSuccess, "cudaMalloc failed!");
        s = cudaMemcpy(p_c, (void*)&c, sizeof(c), cudaMemcpyHostToDevice);     _Assert(s == cudaSuccess, "cudaMalloc failed!");
        //アームをコピーです。
        s = cudaMalloc((void**)&p_arm, sizeof(arm));                                _Assert(s == cudaSuccess, "cudaMalloc failed!");        //
        //失敗成功(全データを見られるように)    ※
        s = cudaMalloc((void**)&p_succeed, sizeof(succeed));                        _Assert(s == cudaSuccess, "cudaMalloc failed!");
        s = cudaMalloc((void**)&p_succeed_blc, sizeof(succeed_blc));                _Assert(s == cudaSuccess, "cudaMalloc failed!");
        //ブロック単位での成功失敗を格納するためのメモリです。
//        s = cudaMalloc()
        //経路の記録。
        s = cudaMalloc((void**)&p_path, sizeof(double) * PATH_MAX);                  _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データ
        s = cudaMalloc((void**)&p_n_path, sizeof(int));                              _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データの数
        //カーネルの判定用。
        s = cudaMalloc((void**)&p_fin, sizeof(int));                                 _Assert(s == cudaSuccess, "cudaMalloc failed!");        //軌跡データの数
    }

    //とりあえず最高速のまま、次の円に接続できるのかを確かめてみる。
    //==========================================================================================================
    //  初期位置から減速位置まで動作させていく。
    //  探索のために、ちょっと行きすぎるようにしてみます。
    //==========================================================================================================
    init_path();        //軌跡はクリアです。
    //
    {   //GPU側の軌跡カウンタクリアと、アームの初期位置のセットです。
        int n_path = 0;
        s = cudaMemcpy(p_n_path, (void*)&n_path, sizeof(int), cudaMemcpyHostToDevice);     _Assert(s == cudaSuccess, "cudaMalloc failed!");        //

        //円１からスタートです。
        _arm.set(_cood(c[0].rf(s_rad), s_rad));           //アームを開始位置へセットします。
        s = cudaMemcpy(p_arm, (void*)&_arm, sizeof(arm), cudaMemcpyHostToDevice);   _Assert(s == cudaSuccess, "cudaMalloc failed!");
    }
    double rate = 1.1;  //
    printf("start search path...");
    {   //探索
        int i = 0;
        //一つも動けなかった場合にいろいろと振ってみるためのパラメータ
        int retry = 0; 
        //        double ov = fabs( e_pos - dec_start_pos)     ;        //少し行きすぎる場所まで記録する。
        for (int fin = 0; !fin; ++i) {

            if (i >= 1111) {
                fin += 1;
                fin -= 1;
            }

            //一回分のカーネルです。
            {
                circle_circle_calc_kernel << <N_BLOCK, N_THREAD >> > (p_arm, p_c, p_succeed, p_succeed_blc, CIRCLE_DIR_CW, 0, spd,rate);
                cudaDeviceSynchronize();    s = cudaGetLastError();        if (s != cudaSuccess) { printf("kernel failed: %s\n", cudaGetErrorString(s));        goto Error; }
            }
            ///break
             if (i>=900) {
                          { s = cudaMemcpy((void*)&succeed[0], (void*)p_succeed, sizeof(succeed), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
                          { s = cudaMemcpy((void*)&succeed_blc[0], (void*)p_succeed_blc, sizeof(succeed_blc), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
                          s = (cudaError_t)1;
             }
                        //ここで各ブロックの集計結果をまとめて、一番いいものを選択し、実際にアームを動作させ、軌跡を記録します。
            {
                //※ちょっと行きすぎる場所まで
                circle_circle_move_kernel << <1, N_BLOCK >> > (p_arm, p_c, p_succeed, p_succeed_blc, p_path, p_n_path, p_fin, CIRCLE_DIR_CW  , e_rad  , 0, spd,true,rate);        //位置を目標に
                cudaDeviceSynchronize();        //終了待ち
            }
            //結果をCPUに転送して評価していきます。ここが一番時間がかかる。
            { s = cudaMemcpy((void*)&fin, (void*)p_fin, sizeof(int), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }

            if (retry && fin == 1) { //リトライでうまくいった
                retry = 0;
            }
            if (fin < 0) {  //finが－１の場合には進めなかった。
                if (! retry ) {
                    rate = 0.5;         //0.5からスターとです。
                }
                else {
                    rate += 0.01;           //ちょっとづつ角度を振ってみる。
                }
                ++retry;
                --i;        //iはインクリメントしないようにする。
                fin = 0;    //継続するようにします。
            }


            if (i >= 900) {
                { s = cudaMemcpy((void*)&_arm, (void*)p_arm, sizeof(arm ), cudaMemcpyDeviceToHost);   _Assert(s == cudaSuccess, "cudaMemcpy failed!");  }
                //このあたりから円➁にさしかかります。
         //                if (!retry) rate = 0.1;     //はじめだけ
            }

        }
        printf("fin(n_path : %d)\n", i-1);
    }
    //軌跡をアップロードします。
    append_path(p_path, p_n_path);  //これでコピーします
    //
    {
        printf("free gpu memory..");
        cudaFree((void*)p_c);
        cudaFree((void*)p_arm);
        cudaFree((void*)p_succeed);
        cudaFree((void*)p_succeed_blc);
        cudaFree((void*)p_path);
        cudaFree((void*)p_n_path);
        cudaFree((void*)p_fin);
        printf("fin\n");
    }

    //ここで計算した軌跡にそって動作させてみます。
    {
        printf("arm move..");
        _arm.set(_cood(c[0].rf(s_rad), s_rad));           //アームをスタート位置へ移動。
        for (int i = 0; i < _path_idx; ++i) {
            //            if (i > 962) {                ::Sleep(1);            }
            //これは動かし方が違う。c[1]に接続しないと
//            _arm.move(_cood(c[0].progress_rad(_arm, _path_data[i], CIRCLE_DIR_CW), _arm.rad + _path_data[i]));
            _cood m = _m(&_arm , c, _path_data[i] , CIRCLE_DIR_CW , rate);  //cirlce_moveカーネルと処理を共通化するため
            _arm.move(m);                            //動かす。
            //各関節の指示データを保存していきますか
            _arm.save();        //ファイルに保存してみます。
            //            ::Sleep(1);
        }
        printf("fin\n");
    }

    printf("all fin\n");
    return;
Error:;
    printf("failed");

}


//リダクション.最大のものだけを選択する
__device__
void _do_reduction(int* reduction_arr, int arr_size)
{
    int _idx = threadIdx.x;     //ここで使われるのはスレッドのインデックスです。
    int abre_val = -1;                //あぶれた値。（-1の場合には無効となる。）
#if 1
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
#else
    if (select_max) {
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
    else {
        for (int i = arr_size / 2; i > 0; i /= 2) {
            if (_idx < i && reduction_arr[_idx + i] < reduction_arr[_idx]) {  //減速の場合は小さいほうを選択する。
                reduction_arr[_idx] = reduction_arr[_idx + i];                //半分より大きな部分と比較して大きなほうを
            }
            if (_idx == 0) {
                if (abre_val != -1) {   //もしあぶれた数がある場合にはそれを再度評価します。
                    if (abre_val < reduction_arr[_idx]) {
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
#endif
    //対象となる配列の数が

}