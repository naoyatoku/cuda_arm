#include "SFML/Graphics.hpp"
#include "units.h"
#include "arm.h"	//
#include "CommonModule.h"

HANDLE _draw_req_event; // イベントオブジェクトのハンドル

const int WINDOW_WIDTH = 1500;
const int WINDOW_HEIGHT = 1000;

//中心
const int ORG_X = WINDOW_WIDTH / 2;
const int ORG_Y = WINDOW_HEIGHT / 2;

//このモジュール独自にarmオブジェクトを持ちます。
static arm  __arm;

//アーム状態を

//外部からアームを受けて描画更新をしてみる。
void	draw(const arm& _arm) {
	__arm = _arm;
	::SetEvent(_draw_req_event);
}




//void draw()
DWORD WINAPI draw_thread(LPVOID param) 
{
	_draw_req_event = CreateEvent(NULL, FALSE, FALSE, NULL);
	_Assert(_draw_req_event != NULL, "dreaw_thread : event create failed");

        // ウィンドウの作成
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "arm");
//	window.clear(sf::Color::White);
//	window.display();
	//=====================================
	//		座標軸です。
	//=====================================
	sf::Vector2f y1(ORG_X, 0);	sf::Vector2f y2(ORG_X, ORG_Y * 2);
	sf::Vector2f x1(0, ORG_Y);	sf::Vector2f x2(ORG_X * 2, ORG_Y);
	sf::VertexArray	axis_y(sf::Lines, 2);
	axis_y[0].position = y1;			axis_y[1].position = y2;
	axis_y[0].color = sf::Color::Black;	axis_y[1].color = sf::Color::Black;
	sf::VertexArray	axis_x(sf::Lines, 2);
	axis_x[0].position = x1; 			axis_x[1].position = x2;
	axis_x[0].color = sf::Color::Black;	axis_x[1].color = sf::Color::Black;

	//line?これは何のためかわからない。
/*	sf::Vector2f p1(100, 100);
	sf::Vector2f p2(700, 500);
	sf::VertexArray line(sf::Lines, 2);
	line[0].position = p1;
	line[1].position = p2;
*/
	window.clear(sf::Color::White);
	//
	sf::RectangleShape l1(sf::Vector2f(230.f, 3.f));    l1.setFillColor(sf::Color::Blue);
	sf::RectangleShape l2(sf::Vector2f(210.f, 2.f));    l2.setFillColor(sf::Color::Green);
	sf::RectangleShape l3(sf::Vector2f(144.f, 2.f));    l3.setFillColor(sf::Color::Magenta);


	_cood before;
	while (window.isOpen()) {
		// イベント処理
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed)
				window.close();
		}

		//同期です
		{
			// イベントを待機
			WaitForSingleObject(_draw_req_event, 1000);
		}
//		if (__arm.operator==(before)) {
//			continue;
//		}
//		printf("D");

		window.clear(sf::Color::White);

		//各アームを動作させる
		l1.setPosition(ORG_X, ORG_Y);				//これはリンクの先端の座標。
		l1.setRotation(-1 * _deg(__arm.lnk(0).r));	//角度をそこにする。y軸は反対になる。
		//この場合、根本の座標なんで一つ前の軸の先端座標を指定しないといけない。
		l2.setPosition(ORG_X + __arm.lnk(0).linked_vect().x, ORG_Y - __arm.lnk(0).linked_vect().y);	//l1の先端座標にする。
		l2.setRotation(-1 * _deg(__arm.lnk(0).r + __arm.lnk(1).r));								//角度をそこにする。
		l3.setPosition(ORG_X + __arm.lnk(1).linked_vect().x, ORG_Y - __arm.lnk(1).linked_vect().y);	//21の先端座標にする。
		l3.setRotation(-1 * _deg(__arm.lnk(0).r + __arm.lnk(1).r + __arm.lnk(2).r));	//角度をそこにする。

		//
		// 
		
		//=============
		//	draw
		//=============
		window.draw(l1);				//
		window.draw(l2);				//
		window.draw(l3);				//
		//座標軸です。
		window.draw(axis_x);	window.draw(axis_y);

		window.display();


		before = __arm;
//		::Sleep(1);
	}
	return 0;
}