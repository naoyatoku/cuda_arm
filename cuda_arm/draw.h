
#pragma once;
#include <Windows.h>
//void draw();
DWORD WINAPI draw_thread(LPVOID param);

extern HANDLE _draw_req_event; // イベントオブジェクトのハンドル
