
#pragma once;
#include <Windows.h>
//void draw();
DWORD WINAPI draw_thread(LPVOID param);

extern HANDLE _draw_req_event; // �C�x���g�I�u�W�F�N�g�̃n���h��
