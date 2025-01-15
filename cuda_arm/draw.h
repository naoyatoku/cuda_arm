
#pragma once;
#include <Windows.h>
//void draw();
DWORD WINAPI draw_thread(LPVOID param);

//描画リクエスト受付を作ってみます。
void	draw(const arm& _arm);
