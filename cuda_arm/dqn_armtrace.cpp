#include <SFML/Graphics.hpp>

#include "dqn.h"
#include <random>

#include "arm.h"
#include "circle.h"


//debug用フラグです
static bool _force=false;

//===========================================================================================================
//		アーム動作の学習を作ってみる。
//===========================================================================================================
// ポールの長さ
#define		PI		3.14159

//ターゲットとなる円軌道を
static circle __c1(100.0, 100.0, 100.0);		//この円です。
static circle __c2(100.0, 100.0, -100.0);		//この円です。

//データ構造を作ります。
struct arm_DQN_param : public DQN_state
{
	arm __arm;			//アーム実体をパラメータとしてもってみる。
	circle *fnc;		//
	//ここのパラメータは変わってくる。
	//アーム自体を持っているのがいいのか？
    arm_DQN_param(): __arm(false) {		//ログを取らないようにします。
		__c1.set_dir(CIRCLE_DIR_CCW);		//C1は反時計回り
		__c2.set_dir(CIRCLE_DIR_CW);		//c2は時計回り
		reset();
	}
    virtual void reset() {
		//アームの初期状態を作ります。
//		__arm.set(_cood(__c1.rf(0), 0));		//アームをセットします。	開始位置はいつも0.とする
		double rad = PI * 2.98 / 2;
		__arm.set(_cood(__c1.rf(rad), rad));		//アームをセットします。	開始位置はいつも0.とする

		fnc = &__c1;							//最初はc1をセットします。
	};             //ここで初期値をセットしてください。ランダムに決まる値もここで行う。
    virtual arm_DQN_param& operator=( const arm_DQN_param&s ) =default;
};

static int __d;
//アームの動作を実装してみる。
//必要な実装を行っていきます。
class arm_DQN : public DQN<arm_DQN_param>
{
private:
	/// <summary>
	///アーム動作の最高速はＰＩ／２とします
	/// </summary>
	const double	max_arm_rad_in_ms = PI / 4000;			//これが最大変位量.(msあたり)
	const double	max_acc_t = 100;					//最高速に到達する時間(ms)
	const double	max_acc = max_arm_rad_in_ms / max_acc_t;	//

public:
	_cood tgt;									//目標となるアームの位置です。
	//入力は:	armの先端の姿勢、位置、と速度 ： 
	//				pos(x,y,t) , spd(Δx ,Δy);
	//出力は：	最大出力の10-100%までの選択
	// 
	//	
	bool			move_ok;	//今回動作させられた
	int				moved;		//連続的に動作できた回数
	vector<int>		actlog;		//アクトの記録です。
	
	arm_DQN() : DQN<arm_DQN_param>(4,2) , tgt(__c2.rf(0) , 0 , spd_acc(0,0) ),moved(0),actlog(128)
/*		max_arm_rad_in_ms(PI / 2000),				//アーム最大速度
		max_acc_t(100),							//加速時定数(最大速度に到達する時間[ms])
		max_acc(max_arm_rad_in_ms/max_acc_t)	//*/
	{
		;
	}	//入力4,出力2で作ります。


	_cood _debug_p(int act) {
		double rad = (2 * max_acc * (act + 1) / n_output()) - max_acc;
		double d = _state.__arm.d.spd / 1000 + _state.fnc->dist(rad, true);			//符号を有効にした距離
		_cood p(_state.fnc->progress(_state.__arm, d), _state.__arm.rad + rad);		//軌道上でd進み、姿勢もrad分変化させるように・・・
		return p;
	}

	//(debug)
	bool	move(int act);	//actが示す方向にアームを加速させます。
/*
	//actにしたがってアームを動かそうとします。
	bool debugm_ove(int act) {
		link_stat _log[16];
		//max_acc : アームが 最高速度 PI/2 (rad/sec)
		int acc_test[] = { 9,8,7,6,5,-1 };	//加速側で試すやつ
		int dec_test[] = { 0,1,2,3,-1 };	//減速側で試すやつ
		int* arr = act > 4 ? acc_test : dec_test;
		_cood p;
		for (int i = 0; arr[i] != -1; ++i) {	//試す配列を全部
			_cood p = _debug_p(arr[i]);
			if ((_log[arr[i]] = _state.__arm.move_able(p)).stat == MOV_OK) {
				goto _act_found;
			}
		}
		//ここまできたら有効なアクションがみつからなかった。
//		act = -1;
		//debugようにもう一度
		//４で試してみる。
		{	//4でダメな場合がある??無加速でダメなケース
			//アームの現在速度を維持できないってことか
			if (_state.__arm.move_able(_debug_p(4)).stat != MOV_OK) {
				_state.__arm.move_able(_debug_p(4));		//デバッグ用に
			}
		}
		//大丈夫だったら検証する。
		move(act);

		return false;

	_act_found:
		_state.__arm.move(p);
		return true;
	}
*/
	// 
	// 
	// 
	//目標となる位置は、__c2の円の一番したの部分で速度加速度ともに0となることです。
												//とりあえずアームの最大速度にたいしてどのくらいの10分割した速度（位置変位としてみる）
												//（まず離散的な行動をとるようにしてみる。）

	virtual bool do_action(int act,double &reward , double dt=0.01) {
		if (_force) {
			act = 1;	//強制的に減速です。
		}
		move_ok = move(act);	//
        //評価を決めていきます。
        //ポールの角速度と角度が、ある範囲に入っている場合には1,角度がある範囲を超えてしまったら終了とします。
        //もし角度PI/4を超えるようなら終了とします。報酬を-1とします。
        reward = 0; //報酬をリセットします。

		//もしくは、円が切り替わった場合は、movedはリセットします

		//move_okでなかったら、-1です。
		if( ! move_ok ){
            reward = -1.0;		
            return true;		//終了です。
		}
		//アーム速度がマイナスになってしまったらNGです。(逆走)
		if ( ( _state.fnc == &__c1 ) && (_state.__arm.wd.spd <0) ) {		//ある程度速度にのってしまったらにする。
			reward = -1.0;
			return true;
		}
		// 
		//時間がかかりすぎたらおわり。
		if (moved > 5000) {
			reward = -1.0;
			return true;
		}

		//次の接合円で一回でも動かせたらOKとしてみる。
		if ((_state.fnc == &__c2) && moved > 0) {
			reward = 1.0;
//			return true;
			//成功したら、現在のwを保存します。
			save();
			main_net().debug_dump();
			//ここから先は強制的にactを決めていきます。
			_force = true;
		}


		//目標位置へ速度０で到達したらOK
		if( _state.__arm == tgt ){
			reward =1.0;
			return true;
		}

		//movedを更新する。(この上でmoved==0を評価している部分があるので、これはここから動かさないこと。)
		if (move_ok) {
			actlog[moved % actlog.size()] = act;
			moved++;
		}
		else {
			moved = 0;
		}


        return false;
	}
	//入力レイヤーを作ります。
	//入力は:	armの先端の姿勢、位置、と速度 ： 
	//				pos(x,y,t) , spd(Δx ,Δy);
	//出力は：	armの次の位置指示か(x,y,t)
    virtual layer* input(arm_DQN_param *p) const {
        vector<unique_ptr<perceptron>>_tmp_in;
		//正規化しないと値が大きすぎる
		//	x , y 座標	(mm)	:	MAX		-715 ～ 715		:	/715か、若しくはmに直して/1000か。
		//	rad	(rad)			:	MAX		0	-	2*PI	:	/2PI
		//	spd	(mm/sec)		:	MAX		PI/2 (rad/sec)なんで、
		//									1.57(rad/sec)なんとかが最高値
		//									→157(mm/sec)くらいが最高値。
		//									m/secにして、/1000か。。
		const double norm_pos = 715.0;
		const double norm_rad = 2.0 * PI;
		const double norm_spd = 1000.0;
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.x:_state.__arm.x)/norm_pos ));				//x位置
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.y:_state.__arm.y)/norm_pos	));				//y位置
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.rad:_state.__arm.rad)/norm_rad ));			//アーム角度

		//これはmm/secの値。これには符号がない。
		//学習に必要なとりあえず
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.d.spd:_state.__arm.d.spd)/norm_spd));		//アーム速度
        return new layer(::move(_tmp_in));
	}	//パラメータから、入力レイヤーを作る。
};

//====================================================================================
//  cartpole実行本体です。
//====================================================================================
// ウィンドウサイズ
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 800;

// カートの初期位置
const int ORG_X = WINDOW_WIDTH / 2;
const int ORG_Y = WINDOW_HEIGHT /2;
//const int POLE_LENGTH = 100;
#define		PI		3.14159
static double	_deg(double rad) {
	return rad * 180 / PI;
}



int arm_dqn(bool learn=true)
{
    arm_DQN dqn;
#if 1
	dqn.load();	//ここでデータ読んてみる
#endif


    // ウィンドウの作成
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "arm");
	//座標軸です。
	sf::Vector2f y1(ORG_X, 0);	sf::Vector2f y2(ORG_X, ORG_Y * 2);
	sf::Vector2f x1(0, ORG_Y);	sf::Vector2f x2(ORG_X * 2, ORG_Y);
	sf::VertexArray	axis_y(sf::Lines, 2);
		axis_y[0].position = y1;			axis_y[1].position = y2;
		axis_y[0].color=sf::Color::Black;	axis_y[1].color=sf::Color::Black;
	sf::VertexArray	axis_x(sf::Lines, 2);
		axis_x[0].position = x1; 			axis_x[1].position = x2;
		axis_x[0].color=sf::Color::Black;	axis_x[1].color=sf::Color::Black;

    // 各リンクの矩形です。
   sf::RectangleShape l1(sf::Vector2f(290.f, 3.f));    l1.setFillColor(sf::Color::Blue);
   sf::RectangleShape l2(sf::Vector2f(290.f, 2.f));    l2.setFillColor(sf::Color::Green);
   sf::RectangleShape l3(sf::Vector2f(135.f, 2.f));    l3.setFillColor(sf::Color::Magenta);

    //文字列
    sf::Font font;
	if (!font.loadFromFile("C:\\Windows\\Fonts\\arial.ttf")) {
		printf("font load error");
	}// フォントファイルを指定します
	sf::Text txt_eps;		//ε
	sf::Text txt_net_used;	//ネット使ったかどうか
	sf::Text txt_n_learn;	//学習回数
	sf::Text txt_n_moved;	//動かせた回数
	sf::Text txt_moved;		//
	//速度表示してみる。
	sf::Text txt_arm_spd;
	sf::Text txt_l1_spd;
	sf::Text txt_l2_spd;
	sf::Text txt_l3_spd;
	//各角度の表示
	sf::Text txt_arm_th;
	sf::Text txt_l1_th;
	sf::Text txt_l2_th;
	sf::Text txt_l3_th;

	{	//
		txt_eps.setFont(font);
		txt_eps.setFillColor(sf::Color::Blue);		// 文字の色を設定します。
		txt_eps.setPosition(ORG_X-60, 180);
		txt_eps.setCharacterSize(15); 				// 文字サイズを設定します（ピクセル単位）。
		//
		txt_net_used.setFont(font);
		txt_net_used.setFillColor(sf::Color::Blue); // 文字の色を設定します。
		txt_net_used.setPosition(ORG_X+30, 180);
		txt_net_used.setCharacterSize(15); 			// 文字サイズを設定します（ピクセル単位）。
		//
		txt_n_learn.setFont(font);
		txt_n_learn.setFillColor(sf::Color::Blue);	// 文字の色を設定します。
		txt_n_learn.setPosition(ORG_X-20, 150);
		txt_n_learn.setCharacterSize(15); 			// 文字サイズを設定します（ピクセル単位）。
		//moved
		txt_moved.setFont(font);
		txt_moved.setFillColor(sf::Color::Cyan);	// 文字の色を設定します。
		txt_moved.setPosition(ORG_X + 60, 150);
		txt_moved.setCharacterSize(15); 			// 文字サイズを設定します（ピクセル単位）。

		txt_n_moved.setFont(font);
		txt_n_moved.setFillColor(sf::Color::Cyan);	// 文字の色を設定します。
		txt_n_moved.setPosition(ORG_X + 90, 150);
		txt_n_moved.setCharacterSize(15); 			// 文字サイズを設定します（ピクセル単位）。
		//
		txt_arm_spd.setFont(font);
		txt_arm_spd.setFillColor(sf::Color::Black);	//
		txt_arm_spd.setPosition(ORG_X-200, 100);
		txt_arm_spd.setCharacterSize(15);


		txt_l1_spd.setFont(font);
		txt_l1_spd.setFillColor(sf::Color::Black);	//
		txt_l1_spd.setPosition(ORG_X - 100 , 100);
		txt_l1_spd.setCharacterSize(15);

		txt_l2_spd.setFont(font);
		txt_l2_spd.setFillColor(sf::Color::Black);	//
		txt_l2_spd.setPosition(ORG_X , 100);
		txt_l2_spd.setCharacterSize(15);

		txt_l3_spd.setFont(font);
		txt_l3_spd.setFillColor(sf::Color::Black);	//
		txt_l3_spd.setPosition(ORG_X + 100, 100);
		txt_l3_spd.setCharacterSize(15);

		//Θ表示
		txt_arm_th.setFont(font);
		txt_arm_th.setFillColor(sf::Color::Black);	//
		txt_arm_th.setPosition(ORG_X - 200, 60);
		txt_arm_th.setCharacterSize(15);

		txt_l1_th.setFont(font);
		txt_l1_th.setFillColor(sf::Color::Black);	//
		txt_l1_th.setPosition(ORG_X - 100, 60);
		txt_l1_th.setCharacterSize(15);

		txt_l2_th.setFont(font);
		txt_l2_th.setFillColor(sf::Color::Black);	//
		txt_l2_th.setPosition(ORG_X, 60);
		txt_l2_th.setCharacterSize(15);

		txt_l3_th.setFont(font);
		txt_l3_th.setFillColor(sf::Color::Black);	//
		txt_l3_th.setPosition(ORG_X + 100, 60);
		txt_l3_th.setCharacterSize(15);

	}
    // メインループ
    while (window.isOpen()) {
        // イベント処理
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
		//目標に向かって動作させます。
		if (learn) {
			dqn.learn();
		}
		else {
			dqn.play();
		}

		txt_eps.setString("e="+::to_string( dqn.eps()));		//イプシロンです。
		if (dqn.net_used()) {
			txt_net_used.setString("NET");
			txt_net_used.setFillColor(sf::Color::Blue);
		}
		else {
			txt_net_used.setString("RND");
			txt_net_used.setFillColor(sf::Color::Red);
		}
		txt_n_learn.setString(::to_string(dqn.n_learn()));
		//軸を動かせたかどうか
		txt_n_moved.setString(::to_string( dqn.moved));
		txt_moved.setString(dqn.move_ok?"OK":"NG");

		//アームと各関節の速度です。
		txt_arm_spd.setString( ::to_string(dqn.state().__arm.wd.spd));
		txt_l1_spd.setString(::to_string(dqn.state().__arm.lnk(0).d.spd));
		txt_l2_spd.setString(::to_string(dqn.state().__arm.lnk(1).d.spd));
		txt_l3_spd.setString(::to_string(dqn.state().__arm.lnk(2).d.spd));
		//Θ
		txt_arm_th.setString(::to_string(dqn.state().__arm.rad));
		txt_l1_th.setString(::to_string(dqn.state().__arm.lnk(0).r));
		txt_l2_th.setString(::to_string(dqn.state().__arm.lnk(1).r));
		txt_l3_th.setString(::to_string(dqn.state().__arm.lnk(2).r));

        // 描画
        window.clear(sf::Color::White);

		//これは先端の画像
        l1.setPosition( ORG_X ,  ORG_Y );				//これはリンクの先端の座標。
		l1.setRotation( -1 * _deg(dqn.state().__arm.lnk(0).r));	//角度をそこにする。y軸は反対になる。
		//この場合、根本の座標なんで一つ前の軸の先端座標を指定しないといけない。
		l2.setPosition( ORG_X + dqn.state().__arm.lnk(0).linked_vect().x, ORG_Y - dqn.state().__arm.lnk(0).linked_vect().y);	//l1の先端座標にする。
		l2.setRotation(-1 *  _deg(dqn.state().__arm.lnk(0).r + dqn.state().__arm.lnk(1).r)	);								//角度をそこにする。
		l3.setPosition( ORG_X + dqn.state().__arm.lnk(1).linked_vect().x, ORG_Y - dqn.state().__arm.lnk(1).linked_vect().y);	//l1の先端座標にする。
		l3.setRotation( -1 * _deg(dqn.state().__arm.lnk(0).r + dqn.state().__arm.lnk(1).r + dqn.state().__arm.lnk(2).r)	);	//角度をそこにする。


		// Create a line from (100, 100) to (700, 500)
		sf::Vector2f p1(100, 100);
		sf::Vector2f p2(700, 500);

		sf::VertexArray line(sf::Lines, 2);
		line[0].position = p1;
		line[1].position = p2;

		//ポールの動きです。
		//ポール角度の表示位置

        window.draw(l1);				//
        window.draw(l2);				//
        window.draw(l3);				//
		window.draw(txt_eps);			//イプシロン
		window.draw(txt_net_used);		//ネットを使ったかどうか
		window.draw(txt_n_learn);		//
		window.draw(txt_moved);			//
		window.draw(txt_n_moved);			//
		//座標軸です。
		window.draw(axis_x);	window.draw(axis_y);
		window.draw(line);
		//
		window.draw(txt_arm_spd);
		window.draw(txt_l1_spd);
		window.draw(txt_l2_spd);
		window.draw(txt_l3_spd);

		window.draw(txt_arm_th);
		window.draw(txt_l1_th);
		window.draw(txt_l2_th);
		window.draw(txt_l3_th);

		window.display();

		::Sleep(1);
    }
    return 0;
}


/*        x = cp.state().cart_position;   //カートの位置(m)
		theta = cp.state().pole_angle; //ポールの角度
		txt.setString(::to_string(x)); // 描画する文字列を設定します。
		txt_pol_th.setString(::to_string(theta)); // 描画する文字列を設定します。
		//xはｾﾝﾁにしてみるか
		x *= 1000;
		//テキスト座標と内容を更新する。
		txt.setPosition(CART_X + x-25, CART_Y + 20);
		//txt_pol_th.setPosition(CART_X + x - 25, CART_Y -150);

/*
・dwitterはマスクなしやプライベートな部分のっているので親近感でていいんじゃないかと
・
*/

//arm::moveto()のコピーです。
//これを改造して、加速もしくは減速の
static int __debug_n=0;
bool arm_DQN::move(int act)//actが示す方向にアームを加速させます。
{
#if 0 //加速しかしない
	act = 0;
/*	if (__debug_n++ < 2) { act = 0; }
	else {
		act = 1;
	}*/
#endif
	int __debug;
	//==================================================================================================================
	//	動作方向を決める？とりあえず円周をまわる前提で
	//==================================================================================================================
	int dir = act == 1 ? -1 : 1;		//とりあえず2を減速側としてマイナス方向へ加速するようにします。

	//==================================================================================================================
	//アームの姿勢変化に対する変化率を決めます。
	//場所によって、移動距離にたいして、均等にアームの姿勢が変化するように角度を振り分けないとなりません。
	//めちゃくちゃ難しくないか。。。。？？？
	//==================================================================================================================
//	double d_posture = 0;		//距離当たりの姿勢変化量です。
/* {
		//円だから、角度で割り算したいが・・・距離を
		//円周上の距離です。
		double d = f.dist(*this, tgt);		//これが今回の移動距離です。

		if (d) {		//アーム姿勢変化量
			//姿勢に関する変化量は角度になってしまう。
			double dt = (tgt - *this).rad;	//これが今回の総姿勢変化量です。これは角度で大丈夫です。
			//距離当たりの角度変化量
			d_posture = dt / d;		//距離あたりの姿勢の変化量(角度)。移動したい距離をかけると、対応した姿勢の変化量になります。
		}
	}




	//これは都度計算するようにします.。

*/
//切り替えの処理
	//==================================================================================================================
	//	今回加速するのか、減速するのかを考えます。
	//==================================================================================================================
	bool acc = (act == 1) ? false : true;					//	acc 	: 目標速度が現在速度よりおおきいか小さいか(大きいとtrue)
	//	const double maxspd = PI / 2;
	//	max_arm_rad_in_ms
	//とりあえず円運動なんで、角速度部分を考えることにします。・・これで本当にいいかはちょっと考えないと
	//距離に直しておかないとなりません
	//ここが最大、C1の終わりでとまるようにしないといけないかも
	const double tgt_min = _state.fnc->dist( acc ? _state.__arm.wd.spd/ 1000	:	-1 * max_arm_rad_in_ms ,true );			//	tgt_min	:	最小速度
	const double tgt_max = _state.fnc->dist( acc ? max_arm_rad_in_ms			:	_state.__arm.wd.spd / 1000 , true); 	//	tgt_max	:	最大速度

	//
	 
	//これって、1msで進む、ラジアン.

/*	accの場合、	最高距離は、	最高速で1msで到達する距離。(= max_arm_rad_in_ms)
				最低距離は、	現在の角速度で、1msで進む距離。
								→現在マイナスの速度で進んでいるのでその速度で(_state.__arm.wd.spd)
*/

	//==================================================================================================================
	//	目標位置&速度に到達するまで続けます。
	//	
	//==================================================================================================================
	double _last_d = 0, _last_dd = 0;		//前回の距離です

//	for (; _arm != tgt;) 
	{		//とりあえず条件を速度が達成できたら抜けるにします。		//これは、座標移動の速度の最適なものを探すための探索。
		_cood last_movable;
		int i;
		//max:最大速度 , min=最低速度
		double max = tgt_max, min = tgt_min;		//目標速度をリセットします。
		//動作できる最大のものを探す二分探索です。
		int n_update = 0;	//動けるものを何回見つけられたか。
		for (i = 0; (_equal<>(min, max,0.001) != true) || (last_movable.rad == 0); ++i) {	//最後に見つけられた
			//ある程度でやめます。
			if (n_update > 2) {
				break;
			}
			//__c2円になってからのNG
			if ( (_state.fnc==&__c2) &&  _equal<>(min,max,0.001)) {	//もう探索できない状況になったら諦めます。
				//				_Assert(0, "arm::move() try overflow");
				__debug = 10;
//				return false;		//何度ためしてもできない。
				//とりあえずもう一回やってみる。
				if (act==1) {		//減速側
					move(act);
				}
				return false;
				//動かせなかった。
//				return false;
			}
			//今回試す動作距離です。
			double d		=	min + (max - min) / 2;		//これが今回ためす速度です。(1msで進む距離)
			//	=====================================================================================
			//	d	:	
			//	この距離,dir方向に進んだ位置を返す関数を用意します。
			//	now :	ある時間単位（この場合は1msです）の移動量
			//	=====================================================================================
			//1msで動作させる距離を決めます。
			//dは、方向を含んでいる。。。
			//__c2の円になったら、角度は変えない。
			double arm_rad;
			{
				if (_state.fnc == &__c1) {
					arm_rad = _state.__arm.rad + _state.fnc->angle(d);		//今回の距離を動かします。
				}
				else {	//とりあえず、ちょっと角度も変えてみる
					arm_rad = _state.__arm.rad + __c1.angle(-d);			//今回の距離を動かします。
/*					if (_state.__arm.rad <5) {
						arm_rad = _state.__arm.rad + __c1.angle(-d);			//今回の距離を動かします。
					}
					else {
						arm_rad = _state.__arm.rad;			//今回の距離を動かします。
					}
//					arm_rad = _state.__arm.rad;			//今回の距離を動かします。
*/
				}
			}
			_cood		now(_state.fnc->progress(_state.__arm, d),			arm_rad);
			//もし無効なベクタが帰ってきたらエラーだと思います。終了です。
			if (now.y==0&&now.x==0) {
				return false;
			}
			
			// 
			//_state.__arm.rad + _state.fnc->angle(d));		//今回の距離を動かします。
//			_state.__arm.rad + (_state.fnc == &__c1) ? _state.fnc->angle(d) : 0	);		//今回の距離を動かします。
			//ここで、もしC1の終わりに差し掛かってしまった場合は、C1の終点で止める必要があるかもしれない。

			link_stat res;
			if ((res = _state.__arm.move_able(now)).stat == MOV_OK) {								//動かせた
				++n_update;
				last_movable = now;		//最後にうまくいったものを記録しておきます。		
				//動かせた場合、現在の位置を最低8として、まだいけるかを				
				if (acc) { min = d; }	//加速する場合は、なるｂく大きな速度を求める
				else { max = d; }		//減速する場合は、なるべく小さい速度を求めたい。
			}
			else {	//何かしらの原因で失敗しました。

				//現在のアームポジションでの逆ヤコビから、軌道上でアームの微小変位を求め、
				//該当リンクが	マイナス側へ加速しすぎの場合には、リンクがプラスの変位
				//				プラス側へ加速しすぎた場合は、リンクがマイナスの変位
				//する方向に動く方向をみつけ、
				// 
				//	上記の考えはちょっと違う気がする。
				//	(1)
				//		現在のアームポジションから、距離d を動かそうとしたら、どこかのlinkが動けなかった。
				// 
				// (2)
				//	そのときに、
				// もっと加速したほうがいいのか、減速したほうがいいのかがリンクエラーからはわからない。
				//	→アームの加速もしくは減速の方向と、リンクの加速、減速の方向が違う場合があるため。
				//
				// (3)
				//	そこで、「今回算出した位置」での逆ヤコビをまず計算し、
				//	そこからの微小変動させたときのリンクの動きを計算する。
				//	
				// (4)
				//	その計算結果から、例えば：リンクを加速させすぎた場合には、
				//	リンクの変位が少なくなる方向のアームの動きを見つける。
				// (5)
				//	それがアームをさらに加速させる方向だとすれば、もっと大きく動作する方向へ動かすように
				//	min,maxを設定する。
				// 
				//min,maxを適切に設定する。
				//まず逆ヤコビです。
				Matrix3d invJ;
				//now分進めた状態のアームを作ります。
				{
					arm _tmp_arm(false);
					_tmp_arm.set(now);
					if (inv_jacobi(_tmp_arm.lnk(0).r, _tmp_arm.lnk(1).r, _tmp_arm.lnk(2).r, invJ) != true) {
						_Assert(0, "invJ is not found");
					}
				}
				//ここで逆ヤコビが求まったので、各間接の勾配を見ていきます。
				//ここで、該当軌道（ｆ）のdir側と、-dir側にちょっとだけ動いた位置を作ります。

				Vector3d delta_link_F, delta_link_B;
				{
					const double delta = 0.000001;	//とりあえず小さい値
					_cood _dl;
//					_cood		now(_state.fnc->progress(_state.__arm, d), _state.__arm.rad + _state.fnc->angle(d));		//今回の距離を動かします。

					//これがちょっとだけ順方向に軌道上を動作させたときの差分の座標です。
					//あ、これがc2円のときの対応になっていない。
					if (_state.fnc == &__c1) {
						_dl = _cood(_state.fnc->progress(now, dir * delta), now.rad + _state.fnc->angle(delta * dir)) - now;			//今回の距離を動かします。
					}
					else {
						//ちょっとややこしいが、c2円の場合は、順方向がマイナスなので-1をかけます。これは後で修正しないと
						_dl = _cood(_state.fnc->progress(now, dir * delta), now.rad + __c1.angle(-1* delta * dir)) - now;			//今回の距離を動かします。
					}

					Vector3d	delta_arm_F(_dl.x, _dl.y, _dl.rad);	//変位量のEigenベクタです。
					delta_link_F = invJ * delta_arm_F;
					//これが逆方向にちょっとだけ逆方向に軌道上に動作させたときの差分の座標です。
//					_dl = _cood(f.progress(*this, -1 * dir * delta), rad + (-1 * d * d_posture)) - now;	//今回の距離を動かします。
					if (_state.fnc == &__c1) {
						_dl = _cood(_state.fnc->progress(now, -1 * dir * delta), now.rad + _state.fnc->angle(-1 * delta * dir)) - now;			//今回の距離を動かします。
					}
					else {
						//c2円の場合は、順方向が-1なので、-1をかけます。
						_dl = _cood(_state.fnc->progress(now, -1 * dir * delta), now.rad + __c1.angle(-1*-1* delta * dir)) - now;			//今回の距離を動かします。
					}
					Vector3d	delta_arm_B(_dl.x, _dl.y, _dl.rad);	//変位量のEigenベクタです。
					delta_link_B = invJ * delta_arm_B;
				}
				//これが該当リンクの状態によって、どっちが合っているかを
				if (res.stat == MOV_ACC_OVER) {	//リンクがプラス側に加速しすぎてＮＧだった場合：リンクの変位が小さくなるよう方向を選択するようにします。
					if (delta_link_F(res.no) < delta_link_B(res.no)) {	//アームを順方向へ動かすと、リンクがマイナスに動きますので、より大きなほうへ探索します。
						//ΔF(アームを順方向へ)を選択。します。この場合、
						if (dir > 0) {				//dirが + （アームの角度が+増える方向へ動作）の場合		より＋側をを探索するように	minを書き換える。)
							min = d;
						} else {
							max = d;				//dirが - (アーム角度が小さくなる方向へ動作させる場合	より－側を探索するように	maxを書き換える)
						}
					}else {		//					else if (delta_link_B(res.no) < 0) {	//アームを逆方向へ動かすと、リンクがマイナスに動く。ので、より小さくなるように探索します。
						//ΔＢ（アームを逆方向へ動作させる）を選択する。
						if (dir > 0) {
							max = d;				// dir が	+ （アーム角度が増える方向へ動作）の場合	より-側へ探索させるように	maxを書き換える
						} else {
							min = d;				// dir が	-  (アーム角度が減る方向へ動作）の場合		より+側へ探索させうように	minを書き換える
						}
					}
				}
				else if (res.stat == MOV_DEC_OVER) {	//リンクがマイナス側に加速しすぎた場合は、リンクの変位が大きく（プラス側）なるような方向へアームを動かします。
					if (delta_link_F(res.no) > delta_link_B(res.no)) {
						//ΔFを（アームを順方向へ動作させる）を選択:
						if (dir > 0)				//	dir が	+	の場合、	より+側へ探索させるように		minを書き換える。
						{
							min = d;
						} else {
							max = d;				//	dir が -	の場合、	より-側へ探索させるように		maxを書き換える
						}
					}
					else {	//ΔB（アームを逆方向に動作させる）を選択:	(ΔＢのほうが、リンクの角度が+になるので）
						if (dir > 0) {				//	dir が	+	の場合、	より-側に探索させるように		maxを書き換える。
							max = d;
						} else {
							min = d;				//	dir が	-	の場合、	より+側に探索させるように		minを書き換える。
						}
					}
				}
			}
		}
		//ここで今回の最適が見つかりましたので動かします。
		_state.__arm.move(last_movable);
	}
	//ここの時点で、現在のアームの位置がC2にさしかかっていれば、
	if (_state.fnc == &__c1) {
		if (_state.__arm.rad >= (PI * 3 / 2)) {
			_state.fnc = &__c2;	//_c2の円に切り替えます。
			//この時点で、アームの角速度は、新しい軌道に合わせます。
			//これは考え直さないといけない
			_state.__arm.wd.spd *= (__c1.get_dir() * __c2.get_dir());

			moved = 0;	//動かせた回数はリセットします（報酬計算のため）
		}
	}

	return true;
}



//まず超適当な理屈でやってみる。
//	最大の変位（アーム速度）を決めてみる。円軌道の場合、角速度でもいいけど、
//	アームの最大変位量を決めて、それに対する割合を決める。(10-100%)
//	(まず簡単に、円軌道なのでradにしてみる。)→のちに円周の弧の長さにする(半径が変わっても大丈夫なように)
//	
//	最大速度を90°/secとしてみると、(PI/2 /sec)
//	1msに進む変位は PI/2000 rad
//		bool move_ok;		//動かせたかどうかです。
		//	v_min = v - max_acc
		//	v_max = v + max_acc

		//	今回の加速です。
		//	dv = v_min + { (v_max-v_min)*(act+1)/10
//        {
			//移動距離を計算します。現在どの円周起動にいるかで計算が変わります。
			//__c1→__c2の切り替わり位置の判定はどうするか

			//±最大加速度の中で１０段階に分けます。
			// 

			//方針転換：
			//(1)アームの速度が上がるにつれ、加速をしなくても等速運動を行うことが、
			//		各リンクにとってはできない加速を伴う場合がある。
			//		そのため、今回の計算はできない。
			//(2)ネットが決めるアクションを「加速」「無加速」「減速」
			//		の三種類として、その時に可能な加減速を都度見つけて、
			//		それを実行するようにする。
			//


			//ちょっと以下をやめてみます。
/*
			double rad = (2*max_acc * (act+1) / n_output()) - max_acc;
			bool		chg=false;					//
			//今回の移動で、軌跡の切り替わりを超えるようならば、そこで止めます。
			if( (_state.fnc == &__c1) && (_state.__arm.rad >= (PI*3/2)) ) {	//アームの角度で判断するか？？これもあやしいな。。。
				rad = PI*3/2 - _state.__arm.rad;							//円周の切り替わり部分はピッタリになるようにします。
																			//これはあとで考え直したい
				chg = true;		//処理が終わったら切り替える
			}
			//今回進みたい距離がちょっと違う。
			//今回の加速度を、自分の速度に足して、その上で1ms進む距離になります。
			//
			//いまはとりあえず、位置の加速、減速だけを考えて、姿勢はそれにしたがって決まるとしてみる。
			//アームの現在速度で進む距離＋加速で進む距離(1msで)

			//この時点で__armはcircle上にある
#if 1
			if (_state.fnc->in(_state.__arm) == true) {
				__d = 0;
			}
			else {
				__d = -1;
			}
#endif
			double d = _state.__arm.d.spd / 1000 + _state.fnc->dist(rad,true);	//符号を有効にした距離
//			double d = _state.fnc->dist(rad);				//これが進む距離
			//※姿勢は、現在の円周上の位置におうじて一意に決まります。
			//これが今回進ませたい距離になります。
			_cood p( _state.fnc->progress( _state.__arm , d ) ,_state.__arm.rad + rad );		//軌道上でd進み、姿勢もrad分変化させるように・・・

			//この時点で、progressの結果が、もしかしたら円周上にない可能性
#if 1
			if (_state.fnc->in(p) == false) {
				__d += 0;
				vec2 q(_state.fnc->progress(_state.__arm, d));
				if (_state.fnc->in(q) == false) {
					__d += 0;
				}
			}
#endif
			//
			//
			//動かそうとします。
			move_ok = (_state.__arm.move(p,true).stat == MOV_OK);
			if(chg) {
				_state.fnc = &__c2;		//軌跡がc2に切り替わります。
			}
#if 1
			//NGになる原因を見つけます
			if( ! move_ok )	{
//				_state.__arm.move_able(p);
				if ( ! debug_move(act)) {
					act = -1;//debug
				}
			}
#endif // 1

			//
		}
*/
