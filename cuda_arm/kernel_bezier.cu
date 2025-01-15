#include "commonmodule.h"
#include "arm.h"
#include "bezier.h"
#include "kernels.h"

#define MAX_N_PATH    8192    //パス最大値


void bezier_path()
{
	cpu_gpu_mem<bezier> _bezier;                       //
    //※デフォルトコンストラクタでアサートしてしまっている。
	cpu_gpu_mem<arm>	_arm;                          //アーム
    cpu_gpu_mem<arm>    _path(MAX_N_PATH);               //アームの状態を直接記録します。各リンクの動作や速度もトレースできるので
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
    _tgt_pos(CPU,START).write_add_info<bezier_pos>( _bezier(CPU).pos(0.0) );    //始点と終点にベジェの始点終点情報を加えます。※ちょっと面倒
    _tgt_pos(CPU,END).write_add_info<bezier_pos>( _bezier(CPU).pos(1.0) );

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
    }
    //==========================================================================================================
    //==========================================================================================================
    {
        //=====================================================================
        //      終点～逆に移動していき、目標速度にたっするまでの時間を記録する。
        //=====================================================================
        int approx_dec_time;   //
        {
            {   //アーム初期状態を記録
                _arm(CPU).set(_tgt_pos(CPU, END));
                _arm(CPU).write_add_info<bezier_pos>(
                    bezier_pos( 1.0 , _bezier(CPU).find_len_from_t(CPU,1.0) )
                );    //終点の位置情報を記録します。
            }
            _arm.Transfer_to_GPU();     //アームを終点位置へ
            //終点→目標速度に到達するまでの軌跡計算です。
            if(_kernel<bezier>(_arm.gpu ,  _bezier.gpu, tgt_spd, DIR_M , &_tgt_pos.gpu[END], _path.gpu, _path_idx.gpu, _path.size, FIN_SPD) != KERNEL_FIN_OK) {
                printf("[phase 1]error:kernel error\r\n");
                return;
            }
//bool _kernel(arm * _arm , void  *_p_path_class , float tgt_spd, int dir , _cood *tgt_pos,arm *_path , int *_path_idx , int fin_condition);
            approx_dec_time = *_path_idx.Transfer_to_CPU();
            _path_idx(CPU) = 0; _path_idx.Transfer_to_GPU();        //0にして戻します。
        }
        //=====================================================================
        //      開始位置から終点位置まで動作させてしまう。
        //=====================================================================
        {
            {   //アーム初期状態を設定
                _arm(CPU).set(_tgt_pos(CPU, START));
                _arm(CPU).write_add_info<bezier_pos>(
                    bezier_pos( 0.0 , _bezier(CPU).find_len_from_t(CPU,0.0) )       //付加情報
                );
                _arm.Transfer_to_GPU();                                             //GPUへ転送
            }
            //bezierの場合は、
            if(_kernel<bezier>(_arm.gpu, _bezier.gpu, tgt_spd, DIR_P, &_tgt_pos.gpu[END] , _path.gpu, _path_idx.gpu,_path.size , FIN_POS_PASS)!=KERNEL_FIN_OK) {
                //この場合は、tが1.0の頭打ちになった場合があるので、許容するかどうかをここで決めます。
                _arm.Transfer_to_CPU(); //現在のARMを取得して、目的位置に近ければOK
                if( (1.0 - _arm(CPU).read_add_info<bezier_pos>().t) < 0.01) {    //ある程度近い場合にはOKとする。これは99%です。
					printf("OK\r\n");
				}
				else {
                    printf("[phase 2]error:kernel error\r\n");
                    return;
                }
			}
        }
        //=====================================================================
        //    停止開始位置を探します。
        //=====================================================================
        {
#if 1       //armがどこまですすんだか確かめます。

            _arm.Transfer_to_CPU();
                //
#endif
            const int n_try=approx_dec_time + (approx_dec_time/5);        // ± 推定減速開始時刻の半分 + 10%程度の範囲を探す。
            //とりあえず、全部ではなくていちいち上書きするようにする。
            const int t_max=256;        //これは少なくとも、停止時間(400ms分ないといけないと思う。)
            //一度軌跡を戻します。
            _path.Transfer_to_CPU();        //時間がかかるので必要な部分だけ取り出したいが
            _path_idx.Transfer_to_CPU(); 

            //approprox_dec_timeを目安に、目標位置に一番近い場所で止まるタイミングを探します。(×n_try回数分)
            //※各パスを記録して検証する。
            cpu_gpu_mem<arm>decpath(t_max);
            cpu_gpu_mem<int>decpath_idx;
            cpu_gpu_mem<arm>_temparm;

            //一番近くに停止する位置を決めていきます。
            _cood best_arm;
            float min_dist= 999999.9;    //有効数字7桁

            //最小パスを保存しておくパスバッファをを作ります。
            //異常判定のために変な値を初期設定しておく。
            arm* best_path = new arm[t_max];
            int best_path_idx;
            int best_timing=-1;
            for (int t = 0; t  < n_try ; ++t) {
                int time = (_path_idx(CPU) - 1 - approx_dec_time) - (t - (n_try / 2));  //今回評価する時間です。
                {
                    decpath_idx(CPU) = 0;
                    decpath_idx.Transfer_to_GPU();                    
                    _temparm(CPU) = _path(CPU,time );    //最終位置からtさかのぼって考える(_path_idx - 1が、最終インデックスです。
                    _temparm.Transfer_to_GPU();                             //gpuに送る
                }
                printf("dec try t=[%d]\r\n", t);
                if (_kernel<bezier>(_temparm.gpu, _bezier.gpu, 0, DIR_P, &_tgt_pos.gpu[END], decpath.gpu, decpath_idx.gpu, decpath.size, FIN_SPD) != KERNEL_FIN_OK) {
                    printf("kernel error\r\n");
                }else{
                    //最小距離を判定していきます。
                    _temparm.Transfer_to_CPU();     //アームが最終的に到達した位置です。
                    //最小を更新していきます。初登録の場合は強制的にベストとします。
                    if ( _tgt_pos(CPU,END).distance(_temparm(CPU)) < min_dist) {
                        min_dist = _tgt_pos(CPU,END).distance(_temparm(CPU));    //最小距離を更新委
                        best_arm = _temparm(CPU);                       //アーム最終位置を更新
                        best_timing = time;                             //減速開始時刻です。
                        //ベストパスをコピーします。
                        {
                            decpath.Transfer_to_CPU();
                            decpath_idx.Transfer_to_CPU();
                            best_path_idx = decpath_idx(CPU);
                            memcpy((void*)&best_path[0], (const void*)decpath.cpu, best_path_idx * sizeof(arm)); // 必要な分だけコピー
                        }
                    }
                }
                _dump_path(decpath, decpath_idx);
            }
            //このループが終わった時点でbest_pathとbest_path_idxとbest_timingに減速開始の情報が保存されています。
            _Assert(best_timing >= 0, "best_timing is not found");
            //大本のpathに減速を連結していきます。
            {
                _Assert(best_timing + best_path_idx < _path.size , "path overflow (renketu)");
                memcpy((void*)(_path.cpu + best_timing), (const void*)best_path, best_path_idx * sizeof(arm));   //NGならforでoperator=を使っていく。
                _path_idx(CPU) = best_timing + best_path_idx;
                _path.Transfer_to_GPU();
                _path_idx.Transfer_to_GPU();

                for (;;) {
                    _draw_path(_path, _path_idx, 2);
                }
            }
        }
    }
    return;
}
