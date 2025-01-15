#include "windows.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "commonmodule.h"
#include "kernels.h"
#include "draw.h"


#include "inv_kinetic.h"
int main()
{
    device_query();

#if 0
{
      _3link_calc_T<float> calc(230, 210, 144);
      mlti<_Vector3d<float>,2>r = calc.calc_inv_kinetic(220, 320, 1.571);

      //検算する
      _cood p = calc.calc_fwd_kinetic(&r[0][0]);hak
      p = calc.calc_fwd_kinetic(&r[1][0]);

#endif

    log_init();

    //描画スレッドです。
    HANDLE hThread = CreateThread(NULL, 0, draw_thread, NULL, 0, NULL);

//    circle_path();
//    circle_path_2();
      //2つの円を接続
//    circle_circle_path();
//    line_path();
    //ベジェ曲線上の軌跡を書いてみます。
    bezier_path();

    if(hThread) {
        WaitForSingleObject(hThread, INFINITE); // スレッドが終了するのを待つ
        CloseHandle(hThread); // スレッドのハンドルを閉じる
    }
    return 0;
}
