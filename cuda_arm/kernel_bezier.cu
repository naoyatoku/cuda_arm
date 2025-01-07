#include "commonmodule.h"
#include "arm.h"
#include "bezier.h"
#include "kernels.h"

//#define MAX_PATH    4096    //パス最大値
//- rdc = true

//テンプレートを使用可能にする
//template bool _kernel<bezier>(arm* _arm, void* _p_path_class, float tgt_spd, int dir, _cood* tgt_pos, arm* _path, int* _path_idx, int fin_condition);

/*
void bezier_path()
{
	cpu_gpu_mem<bezier> _bezier;                       //
    //※デフォルトコンストラクタでアサートしてしまっている。
	cpu_gpu_mem<arm>	_arm;                          //アーム
    cpu_gpu_mem<arm>    _path(MAX_PATH);               //アームの状態を直接記録します。各リンクの動作や速度もトレースできるので
    cpu_gpu_mem<int>    _path_idx;                     //パスの現在インデックスです。+		gpu	0x0000000705804400 {???}	int *
    cpu_gpu_mem<_cood>  _tgt_pos(2);                   //始点～終点の位置


    printf("line path start!\r\n");
    //==========================================================================
    //  軌跡の設定
    //==========================================================================
    _tgt_pos(CPU,START) = _cood(vec2(150,250),PI/2);   //始点
    _tgt_pos(CPU,END) = _cood(vec2(230,330),PI/2);      //終点
    const float tgt_spd = 30.0;                             //10mm/sec という意味で設定しますがd2096
    //ベジェ曲線の設定
    _bezier(CPU,0).set_params( _tgt_pos(CPU,START) , vec2(180,300), vec2(210,170), _tgt_pos(CPU,END) , 1000);
    //==========================================================================
    //  軌跡記録の初期化
    //=========================================================================
    {   //path
        memset((void*)&_path(CPU) , 0 , _path.size);        //
        _path_idx(CPU) = 0;
    }
    //gpuへ転送
    {
        _bezier.Transfer_to_GPU();
        _path.Transfer_to_GPU();
        _path_idx.Transfer_to_GPU();
        _tgt_pos.Transfer_to_GPU();
        _bezier.Transfer_to_GPU();
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
                _arm(CPU).set(_tgt_pos(CPU, END));  _arm.Transfer_to_GPU();     //アームを終点位置へ
                //終点→目標速度に到達するまでの軌跡計算です。
                   _kernel<bezier>(_arm.gpu ,  _bezier.gpu, tgt_spd, 0 , &_tgt_pos.gpu[END], _path.gpu, _path_idx.gpu, FIN_SPD);      //
//bool _kernel(arm * _arm , void  *_p_path_class , float tgt_spd, int dir , _cood *tgt_pos,arm *_path , int *_path_idx , int fin_condition);
                approx_dec_time = *_path_idx.Transfer_to_CPU();
                _path_idx(CPU) = 0; _path_idx.Transfer_to_GPU();        //0にして戻します。
                //debug dump
            }
        }
    }

}

*/