
#pragma once;
#include <Windows.h>
//void draw();
DWORD WINAPI draw_thread(LPVOID param);

//�`�惊�N�G�X�g��t������Ă݂܂��B
void	draw(const arm& _arm);
