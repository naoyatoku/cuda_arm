#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <math.h>
#include "inv_kinetic.h"
#include "linear.h"
#include "arm.h"
#include "CommonModule.h"
#include "draw.h"
#include "kernels.h"

//やはり1ブロックあたりのスレッド数に限りがある(1024まで)なので、ブロックに分けていかないといけない。
//RTX4060Laptopのデバイスでは

//SM 24個
//SM 1つあたり 1536(512*3)スレッド  →   ×24 => 36864スレッド平行にできる。
//1ブロック当たり512スレッドとすると、72ブロックが並列処理の最大数。

//(共有メモリ)
//SM 1つあたり 64kbyteが限界。
//ブロックあたり    48byteの制約がある。
//SMあたり実行するように想定したブロック数（今回の場合は3個）分、合計して64kbyteを超える場合、平行に処理されなくなるので、注意する。
//  (実行はできてしまうが、順番になってしまう可能性がある。)
//  ※共有メモリはSM毎に

//（レジスタ）
//1スレッドあたりの最大レジスタ数： 255
//1 SMあたりのレジスタ数        ：  65,536
//共有メモリとは関係ない。

//(グローバルメモリ)
//RTX4060Laptopの場合、  8Gbyteある。


//方向を示す定義


//カーネルの結果をチェックする。
__host__
bool check_kernl_error()
{
    cudaError_t s;
    s = cudaGetLastError();
    if (s != cudaSuccess) {
        printf("kernel failed: %s\n", cudaGetErrorString(s));
        return false;
    } //
    return true;
}
//一貫性を持たせるため（カーネル分割するので）関数状態にします。
//dirは、速度に符号をつけて一緒にしてもよい。
__device__ inline
_cood _tgt_cood(const arm* _arm , const linear*_line ,float tgt_spd , int dir,int idx,int n_all_thread)
{
    float d_spd  = tgt_spd - _arm->d.spd;                             //目標速度にむけての加速度です。(mm/sec)

//  float dx =  dir * (spd_span / n_all_thread) * (idx + ( (tgt_spd > _arm->d.spd) ? 1 : 0)) / 1000; //1ms分なので、 spd rad/sec -> rad/msec に修正        idx=511で bloclDim.xとなるように

    //-----------------------------------------------------------------------------------------------------------------
    //  ※toku 
    //  ここは、現在速度～目標速度の間だけだと、減速が効かないので、0もしくは、あるていどの減速までを範囲に含める必要がある。
    //-----------------------------------------------------------------------------------------------------------------

    //　(現在速度＋目標速度への加速度)/1000 (1msあたりに換算)
    float dx = ((d_spd / n_all_thread) * idx)  / 1000; //
    //デバッグしやすいように計算を分けます。
    dx += _arm->d.spd / 1000;   //現在速度で1mあたりに進む距離
    //dxこれが現在速度からの加速分です。現在速度の

    _cood c(_line->progress(*_arm,dx,dir),_arm->rad);   //デバッグ用に変数にします。
    return c;
}
//1msおきで進める位置を求める。
__global__
void line_calc_kernel(arm* _arm , linear* _line ,int* block_result , float tgt_spd , int dir)
{
    //今回のケースの場合、SMあたり3ブロック実行させたいので、21kByteまでに抑える必要がある。
    //リダクション（高速実行するために）
    __shared__ int _reduction[N_THREAD];        //int*512 -> 2kbyte
//    bool acc = (tgt_spd > _arm->d.spd ) ? true : false;       //加減速スイッチです。

    //ブロックとスレッドの組み合わせで成功失敗の
    int idx = (blockIdx.x * blockDim.x) + threadIdx.x;                          //
    link_stat s = _arm->move_able(_tgt_cood(_arm , _line, tgt_spd, dir,idx,N_ALL_THREAD ));      //

    //リダクション用のバッファに結果を入れて言います。
//    _reduction[threadIdx.x] = s.stat == MOV_OK ? idx : (acc?-1:N_ALL_THREAD) ;     //成功したら自分のインデックスを入れる失敗したら
     //_reductionにはいる数値（インデックス）は、大きいほうがもっとも理想に近いものになっています（加速、減速時ともに）
    _reduction[threadIdx.x] = s.stat == MOV_OK ? idx : -1 ;     //成功したら自分のインデックスを入れる失敗したら
    __syncthreads();        //あるブロック内部のスレッドの同期です。
    //==================================================================================
    // リダクションでブロック単位の評価を行います。
    //===================================================================================
    _do_reduction(_reduction,blockDim.x);

    //ここでブロック内のチャンピオンデータが_reduction[0]に保管されている。自分のブロックデータを更新します。
    if(threadIdx.x==0){
        block_result[blockIdx.x] = _reduction[0];
#if 0   //
//        if (blockIdx.x < 3) {
//            printf("\x1B[%d;%dH ac[%d]bl[%d]r[%d]\t",0, blockIdx.x*100,  acc, blockIdx.x, _reduction[0]);
            printf("bl[%d]a[%d]r[%d]\t", blockIdx.x,acc, _reduction[0]);
//        }
#endif
    }
}
//一番ベストな位置へアームを動作する
__global__ void line_move_kernel(arm* _arm , linear* _line ,int* block_result , float tgt_spd ,_cood *tgt_pos ,  int dir, bool* fin,arm* _path, int* _path_idx , int fin_condition)
{
    //まずブロック毎のデータの中でさらに一番いいものを選択する。
    _do_reduction(block_result , blockDim.x );

    //いちいちCPUに戻さなくてもいいように、アームをここで動作させ、終了判定をします。
    if(threadIdx.x==0){ //
        _gpuAssert(block_result[0] >= 0, "move_kernel():no available result\r\n");       //有効なインデックスがなかった場合はエラーです。
        _cood tgt =_tgt_cood( _arm , _line , tgt_spd,dir,block_result[0],N_ALL_THREAD);
        _arm->move(tgt);                                   //動かす。

#if 0
        printf("\r\n ====> [%d]ms(%f,%f),spd(%f) acc(%f) block_result[%d]\r\n", *_path_idx , _arm->x ,_arm->y , _arm->d.spd , _arm->d.acc, block_result[0]);
#endif

//        printf("[%d]t:%lf,spd:%lf,acc%lf,dlt:%lf\n", *n_path, _arm->rad, _arm->wd.spd, _arm->wd.acc, fabs(_arm->wd.spd) - spd);
        _path[(*_path_idx)++] = *_arm;    //アーム記録してもいいかも

        if (*_path_idx > MAX_PATH-1) {  //overflow
            printf("\r\npath overflow\r\n");
            *fin = true;
            goto _fin;
        }

        //終了判定します。
        {
            *fin = false;
            switch(fin_condition){
                case FIN_SPD:
                    if (_equal<float>(_arm->d.spd, tgt_spd,0.1) == true) {
                        *fin = true;
                    }
                    break;
                case  FIN_POS_PASS:
                    if(dir == DIR_P){   //プラス方向に進行している場合、
                        if( _arm->x >= tgt_pos->x ){
                            *fin=true;
                        }
                    }else{
                        if( _arm->x <= tgt_pos->x ){
                            *fin=true;
                        }
                    }
            }
        }
    }
_fin:
    __syncthreads();    //これいらないか？
#if 0   //break用
    if (threadIdx.x == 1) {
        if (*fin == true) {
            //ダンプしてみる。
            for (int i = 0; i < *_path_idx ; ++i) {
                const arm& cur = _path[i];
                printf("[%d](%f,%f,%f)spd(%f)\r\n", i, cur.x, cur.y, cur.rad, cur.d.spd);
            }
        }
    }
    __syncthreads();    //これいらないか？
#endif
    //
}
//もう一個、目的位置まで動作するカーネルを作る。

//__global__
//toku 最初、ここはカーネル呼び出しのはずだったが、うまくいかなかったのでホストコードにした。
//現状、ポインタはGPUアドレスを受けていますがこのままにしておきます。のちにカーネルにできるならしたいので
__host__
bool line_kernel(arm * _arm , linear * _line , float tgt_spd, int dir , _cood *tgt_pos,arm *_path , int *_path_idx , int fin_condition)
{
    const dim3 threads( N_THREAD);
    const dim3 blocks(N_BLOCK);
    cpu_gpu_mem<int>	block_result(N_BLOCK);		//これが全部の結果を保持するようにします。一度の
    cpu_gpu_mem<bool>fin;

    esc_clr();  //画面はクリアする。


    //※アーム状態はカーネル呼び出し前に設定してください。
    for (fin(CPU) = false;  fin(CPU) != true; fin.Transfer_to_CPU() ) {      //この処理に時間がかかるのでこのループ自体も
        line_calc_kernel << <blocks, threads >> > ( _arm , _line , block_result.gpu , tgt_spd, dir);
        cudaDeviceSynchronize();
        if (check_kernl_error() != true) {
            goto _error;
        }
        //この時点でベストが見つかっているとする。
        line_move_kernel << <1, blocks >> > (_arm , _line , block_result.gpu , tgt_spd, tgt_pos , dir , fin.gpu , _path , _path_idx ,fin_condition );
        cudaDeviceSynchronize();
        if (check_kernl_error() != true) {
            goto _error;
        }
    }

    return true;
_error:
    return false;
}

//void line_calc_kernel(arm* _arm , linear* _line ,int* block_result , float tgt_spd , int dir)
//__global__ void line_move_kernel(arm* _arm , linear* _line ,int* block_result , float tgt_spd ,int dir, bool* fin,_cood* _path, int* _path_idx )



//=======================================================================================================================================================================================
//		lineパス計算（カーネル呼び出し）
//=======================================================================================================================================================================================

void _dump_path(cpu_gpu_mem<arm>&path , cpu_gpu_mem<int>&path_idx,bool disp_link/*=false*/,int wait/*=0*/)
{
    path.Transfer_to_CPU();
    path_idx.Transfer_to_CPU();
    for (int i = 0; i < path_idx(CPU); ++i) {
        const arm& cur = path(CPU, i);
        //描画できるように
        _arm = cur;
        printf("[%d](%f,%f,%f)spd(%f)\r\n", i, cur.x, cur.y, cur.rad, cur.d.spd);
        if(disp_link){
        for (int l = 0; l< 3; ++l){
            printf("        L%d(pos[%f(deg:%f)]spd[%f]\r\n", l, cur.lnk(l).r ,RAD2DEG( cur.lnk(l).r) ,  cur.lnk(l).d.spd);
        }
        }
//        printf("\r\n");
       // イベントをシグナル状態に設定
        SetEvent(_draw_req_event);

        ::Sleep(wait);
    }
}



static vec2 _v; //debug
//スレッドのx,yで、同じブロックで行ってみる。
void line_path()
{

	//なんかあれなんでここで
	cpu_gpu_mem<linear> _line;                           //
	cpu_gpu_mem<arm>	_arm;                            //アーム
    cpu_gpu_mem<arm>    _path(MAX_PATH);                    //アームの状態を直接記録します。各リンクの動作や速度もトレースできるので
    cpu_gpu_mem<int>    _path_idx;                          //パスの現在インデックスです。+		gpu	0x0000000705804400 {???}	int *
    cpu_gpu_mem<_cood>  _tgt_pos(2);                   //始点～終点の位置

    printf("line path start!\r\n");
    //==========================================================================
    //  軌跡の設定
    //==========================================================================
    _tgt_pos(CPU,0) = _cood(vec2(150,250),PI/2);   //始点
    _tgt_pos(CPU,1) = _cood(vec2(230,330),PI/2);   //終点

#if 0       //始点と終点の位置を確認します。
    {
        _arm(CPU).set(_tgt_pos(CPU, START));
            _v = _arm(CPU).lnk(0).linked_vect();
            _v = _arm(CPU).lnk(1).linked_vect();
            _v = _arm(CPU).lnk(2).linked_vect();
        _arm(CPU).set(_tgt_pos(CPU, END));
             _v = _arm(CPU).lnk(0).linked_vect();
            _v = _arm(CPU).lnk(1).linked_vect();
            _v = _arm(CPU).lnk(2).linked_vect();
    }
#endif
    const float tgt_spd = 30.0;                             //10mm/sec という意味で設定しますがd2096
    _line(CPU) = linear( 1.0 , 100.0 );                     //直線の設定

    //==========================================================================
    //  もろもろ初期設定です。
    //
    //  軌跡となる直線を設定する。(開始時のアーム位置が(100,200)として、
    //  傾き1.0の直線→切片は100
    //=========================================================================
    {   //path
        memset((void*)&_path(CPU) , 0 , _path.size);        //
        _path_idx(CPU) = 0;
    }
    //gpuへ転送
    {
        _line.Transfer_to_GPU();
        _path.Transfer_to_GPU();
        _path_idx.Transfer_to_GPU();
        _tgt_pos.Transfer_to_GPU();
    }
    //==========================================================================================================
    //==========================================================================================================
    {
        //カーネルからカーネルが呼べないので
        {
            //まず停止予定位置から、マイナス方向に目標速度になるまで動作させる。
            //停止する開始時間を記録するようにします。
            int approx_dec_time;   //
            {
/*                cpu_gpu_mem<_cood> dec_start;           dec_start(CPU) = *_arm.Transfer_to_CPU();   //さきほど計算した_armの位置（減速開始位置）を記録する。
                dec_start.Transfer_to_GPU();                                                        //GPUへ転送
*/
                _arm(CPU).set(_tgt_pos(CPU, END));  _arm.Transfer_to_GPU();
                line_kernel(_arm.gpu ,  _line.gpu, tgt_spd, DIR_M, &_tgt_pos.gpu[END], _path.gpu, _path_idx.gpu, FIN_SPD);      //
                approx_dec_time = *_path_idx.Transfer_to_CPU();
                _path_idx(CPU) = 0; _path_idx.Transfer_to_GPU();        //0にして戻します。
                //debug dump
#if 0
                {
                    printf("idx=%d , pos=(%f,%f)\r\n" , approx_dec_time , );
                }
#endif
            }

            //ここで、現在の_arm位置が、減速を開始するおおよその位置です。これを記録します。

            //アームを開始位置にリセットし減速開始位置まで動作させる。
            {
                _arm(CPU).set(_tgt_pos(CPU, START));
                _arm.Transfer_to_GPU();                         //GPUへ転送
                //やはりここで後の
                line_kernel(_arm.gpu, _line.gpu, tgt_spd, DIR_P, &_tgt_pos.gpu[END] , _path.gpu, _path_idx.gpu, FIN_POS_PASS);      //減速開始位置に
            }


//            line_kernel()」//これを何種類かやって、ぴったりにとまるところを探す。
            {
                const int n_try=5;
                const int t_max=256;
                //一度軌跡を戻します。
                _path.Transfer_to_CPU();        //時間がかかるので必要な部分だけ取り出したいが
                _path_idx.Transfer_to_CPU(); 
                //100個くらいできるか?
                cpu_gpu_mem<arm>decpath[n_try] = { cpu_gpu_mem<arm>(t_max) ,cpu_gpu_mem<arm>(t_max)  , cpu_gpu_mem<arm>(t_max) , cpu_gpu_mem<arm>(t_max) ,cpu_gpu_mem<arm>(t_max) };       //
                cpu_gpu_mem<int>decpath_idx [n_try];        //
                cpu_gpu_mem<arm>_temparm;

                for (int t = 0; t  < n_try ; ++t) {
                    _temparm(CPU) = _path(CPU,_path_idx(CPU) - 1  - t) ;    //最終位置からtさかのぼって考える(_path_idx - 1が、最終インデックスです。
                    _temparm.Transfer_to_GPU();                         //gpuに送る
                    printf("dec try [-%d]\r\n", t);
                    line_kernel( _temparm.gpu , _line.gpu, 0, DIR_P, &_tgt_pos.gpu[END], decpath[t].gpu, decpath_idx[t].gpu, FIN_SPD);
                    _dump_path(decpath[t], decpath_idx[t]);
                    //本来、ここで一番いいものを選定します。
                    //
                    //      ※まだ作っていません
                    //
                }

                //toku とりあえず最後のを採用して、pathと結合する。
                {
                    const int t=n_try-1;
                    //
                    decpath[t].Transfer_to_CPU();
                    decpath_idx[t].Transfer_to_CPU();
                    //
                    const int path_renketsu_point = _path_idx(CPU) - 1  - t;//ここから連結します。
                    for(int i =0 ; i < decpath_idx[t](CPU) ; ++i ){
                        _path(CPU,path_renketsu_point + i ) = decpath[t](CPU,i);
                    }
                    //終了位置を合わせます。
                    _path_idx(CPU) = path_renketsu_point + decpath_idx[t](CPU);

                    //GPUも合わせておきます・・・ヘんか？あとでGPUを使われてもいいように
                    _path.Transfer_to_GPU();
                    _path_idx.Transfer_to_GPU();
                }
            }
        }
    }
    for (int i=0;i<1;++i) {
        _dump_path(_path, _path_idx, true);
//    ::Sleep(1000);
    }

    //各リンクの軌跡を記録します。これは関数にするか
    {
        csv bin[3] = { csv("l1.bin",true) , csv("12.bin",true) , csv("l3.bin",true) };  //こちらは軸指示用(bin)
        csv log("arm.log",true);
        for(int i=0 ; i<_path_idx(CPU) ; ++i) {     //
            arm & _arm = _path(CPU,i);  //この瞬間のアーム状態です
            log.writef("%d,%f,%f,%f,%f,%f," , i,_arm.x,_arm.y,_arm.rad,_arm.d.spd,_arm.d.acc);  //アームの状態。
            for(int l=0;l<3;++l){
                bin[l].bin_write<float>(_arm.lnk(l).r);
                log.writef("%f,%f,%f," , _arm.lnk(l).r, _arm.lnk(l).d.spd,_arm.lnk(l).d.acc);   //リンクの Θ,spd,accを記録
            }
            log.write("\r\n");
        }
    }
    //

_error:;
    printf("end\r\n");

}

