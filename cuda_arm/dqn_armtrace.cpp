#include <SFML/Graphics.hpp>

#include "dqn.h"
#include <random>

#include "arm.h"
#include "circle.h"


//debug�p�t���O�ł�
static bool _force=false;

//===========================================================================================================
//		�A�[������̊w�K������Ă݂�B
//===========================================================================================================
// �|�[���̒���
#define		PI		3.14159

//�^�[�Q�b�g�ƂȂ�~�O����
static circle __c1(100.0, 100.0, 100.0);		//���̉~�ł��B
static circle __c2(100.0, 100.0, -100.0);		//���̉~�ł��B

//�f�[�^�\�������܂��B
struct arm_DQN_param : public DQN_state
{
	arm __arm;			//�A�[�����̂��p�����[�^�Ƃ��Ă����Ă݂�B
	circle *fnc;		//
	//�����̃p�����[�^�͕ς���Ă���B
	//�A�[�����̂������Ă���̂������̂��H
    arm_DQN_param(): __arm(false) {		//���O�����Ȃ��悤�ɂ��܂��B
		__c1.set_dir(CIRCLE_DIR_CCW);		//C1�͔����v���
		__c2.set_dir(CIRCLE_DIR_CW);		//c2�͎��v���
		reset();
	}
    virtual void reset() {
		//�A�[���̏�����Ԃ����܂��B
//		__arm.set(_cood(__c1.rf(0), 0));		//�A�[�����Z�b�g���܂��B	�J�n�ʒu�͂���0.�Ƃ���
		double rad = PI * 2.98 / 2;
		__arm.set(_cood(__c1.rf(rad), rad));		//�A�[�����Z�b�g���܂��B	�J�n�ʒu�͂���0.�Ƃ���

		fnc = &__c1;							//�ŏ���c1���Z�b�g���܂��B
	};             //�����ŏ����l���Z�b�g���Ă��������B�����_���Ɍ��܂�l�������ōs���B
    virtual arm_DQN_param& operator=( const arm_DQN_param&s ) =default;
};

static int __d;
//�A�[���̓�����������Ă݂�B
//�K�v�Ȏ������s���Ă����܂��B
class arm_DQN : public DQN<arm_DQN_param>
{
private:
	/// <summary>
	///�A�[������̍ō����͂o�h�^�Q�Ƃ��܂�
	/// </summary>
	const double	max_arm_rad_in_ms = PI / 4000;			//���ꂪ�ő�ψʗ�.(ms������)
	const double	max_acc_t = 100;					//�ō����ɓ��B���鎞��(ms)
	const double	max_acc = max_arm_rad_in_ms / max_acc_t;	//

public:
	_cood tgt;									//�ڕW�ƂȂ�A�[���̈ʒu�ł��B
	//���͂�:	arm�̐�[�̎p���A�ʒu�A�Ƒ��x �F 
	//				pos(x,y,t) , spd(��x ,��y);
	//�o�͂́F	�ő�o�͂�10-100%�܂ł̑I��
	// 
	//	
	bool			move_ok;	//���񓮍삳����ꂽ
	int				moved;		//�A���I�ɓ���ł�����
	vector<int>		actlog;		//�A�N�g�̋L�^�ł��B
	
	arm_DQN() : DQN<arm_DQN_param>(4,2) , tgt(__c2.rf(0) , 0 , spd_acc(0,0) ),moved(0),actlog(128)
/*		max_arm_rad_in_ms(PI / 2000),				//�A�[���ő呬�x
		max_acc_t(100),							//�������萔(�ő呬�x�ɓ��B���鎞��[ms])
		max_acc(max_arm_rad_in_ms/max_acc_t)	//*/
	{
		;
	}	//����4,�o��2�ō��܂��B


	_cood _debug_p(int act) {
		double rad = (2 * max_acc * (act + 1) / n_output()) - max_acc;
		double d = _state.__arm.d.spd / 1000 + _state.fnc->dist(rad, true);			//������L���ɂ�������
		_cood p(_state.fnc->progress(_state.__arm, d), _state.__arm.rad + rad);		//�O�����d�i�݁A�p����rad���ω�������悤�ɁE�E�E
		return p;
	}

	//(debug)
	bool	move(int act);	//act�����������ɃA�[�������������܂��B
/*
	//act�ɂ��������ăA�[���𓮂������Ƃ��܂��B
	bool debugm_ove(int act) {
		link_stat _log[16];
		//max_acc : �A�[���� �ō����x PI/2 (rad/sec)
		int acc_test[] = { 9,8,7,6,5,-1 };	//�������Ŏ������
		int dec_test[] = { 0,1,2,3,-1 };	//�������Ŏ������
		int* arr = act > 4 ? acc_test : dec_test;
		_cood p;
		for (int i = 0; arr[i] != -1; ++i) {	//�����z���S��
			_cood p = _debug_p(arr[i]);
			if ((_log[arr[i]] = _state.__arm.move_able(p)).stat == MOV_OK) {
				goto _act_found;
			}
		}
		//�����܂ł�����L���ȃA�N�V�������݂���Ȃ������B
//		act = -1;
		//debug�悤�ɂ�����x
		//�S�Ŏ����Ă݂�B
		{	//4�Ń_���ȏꍇ������??�������Ń_���ȃP�[�X
			//�A�[���̌��ݑ��x���ێ��ł��Ȃ����Ă��Ƃ�
			if (_state.__arm.move_able(_debug_p(4)).stat != MOV_OK) {
				_state.__arm.move_able(_debug_p(4));		//�f�o�b�O�p��
			}
		}
		//���v�������猟�؂���B
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
	//�ڕW�ƂȂ�ʒu�́A__c2�̉~�̈�Ԃ����̕����ő��x�����x�Ƃ���0�ƂȂ邱�Ƃł��B
												//�Ƃ肠�����A�[���̍ő呬�x�ɂ������Ăǂ̂��炢��10�����������x�i�ʒu�ψʂƂ��Ă݂�j
												//�i�܂����U�I�ȍs�����Ƃ�悤�ɂ��Ă݂�B�j

	virtual bool do_action(int act,double &reward , double dt=0.01) {
		if (_force) {
			act = 1;	//�����I�Ɍ����ł��B
		}
		move_ok = move(act);	//
        //�]�������߂Ă����܂��B
        //�|�[���̊p���x�Ɗp�x���A����͈͂ɓ����Ă���ꍇ�ɂ�1,�p�x������͈͂𒴂��Ă��܂�����I���Ƃ��܂��B
        //�����p�xPI/4�𒴂���悤�Ȃ�I���Ƃ��܂��B��V��-1�Ƃ��܂��B
        reward = 0; //��V�����Z�b�g���܂��B

		//�������́A�~���؂�ւ�����ꍇ�́Amoved�̓��Z�b�g���܂�

		//move_ok�łȂ�������A-1�ł��B
		if( ! move_ok ){
            reward = -1.0;		
            return true;		//�I���ł��B
		}
		//�A�[�����x���}�C�i�X�ɂȂ��Ă��܂�����NG�ł��B(�t��)
		if ( ( _state.fnc == &__c1 ) && (_state.__arm.wd.spd <0) ) {		//������x���x�ɂ̂��Ă��܂�����ɂ���B
			reward = -1.0;
			return true;
		}
		// 
		//���Ԃ������肷�����炨���B
		if (moved > 5000) {
			reward = -1.0;
			return true;
		}

		//���̐ڍ��~�ň��ł�����������OK�Ƃ��Ă݂�B
		if ((_state.fnc == &__c2) && moved > 0) {
			reward = 1.0;
//			return true;
			//����������A���݂�w��ۑ����܂��B
			save();
			main_net().debug_dump();
			//���������͋����I��act�����߂Ă����܂��B
			_force = true;
		}


		//�ڕW�ʒu�֑��x�O�œ��B������OK
		if( _state.__arm == tgt ){
			reward =1.0;
			return true;
		}

		//moved���X�V����B(���̏��moved==0��]�����Ă��镔��������̂ŁA����͂������瓮�����Ȃ����ƁB)
		if (move_ok) {
			actlog[moved % actlog.size()] = act;
			moved++;
		}
		else {
			moved = 0;
		}


        return false;
	}
	//���̓��C���[�����܂��B
	//���͂�:	arm�̐�[�̎p���A�ʒu�A�Ƒ��x �F 
	//				pos(x,y,t) , spd(��x ,��y);
	//�o�͂́F	arm�̎��̈ʒu�w����(x,y,t)
    virtual layer* input(arm_DQN_param *p) const {
        vector<unique_ptr<perceptron>>_tmp_in;
		//���K�����Ȃ��ƒl���傫������
		//	x , y ���W	(mm)	:	MAX		-715 �` 715		:	/715���A�Ⴕ����m�ɒ�����/1000���B
		//	rad	(rad)			:	MAX		0	-	2*PI	:	/2PI
		//	spd	(mm/sec)		:	MAX		PI/2 (rad/sec)�Ȃ�ŁA
		//									1.57(rad/sec)�Ȃ�Ƃ����ō��l
		//									��157(mm/sec)���炢���ō��l�B
		//									m/sec�ɂ��āA/1000���B�B
		const double norm_pos = 715.0;
		const double norm_rad = 2.0 * PI;
		const double norm_spd = 1000.0;
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.x:_state.__arm.x)/norm_pos ));				//x�ʒu
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.y:_state.__arm.y)/norm_pos	));				//y�ʒu
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.rad:_state.__arm.rad)/norm_rad ));			//�A�[���p�x

		//�����mm/sec�̒l�B����ɂ͕������Ȃ��B
		//�w�K�ɕK�v�ȂƂ肠����
		_tmp_in.emplace_back(::make_unique<input_perceptron>((p?p->__arm.d.spd:_state.__arm.d.spd)/norm_spd));		//�A�[�����x
        return new layer(::move(_tmp_in));
	}	//�p�����[�^����A���̓��C���[�����B
};

//====================================================================================
//  cartpole���s�{�̂ł��B
//====================================================================================
// �E�B���h�E�T�C�Y
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 800;

// �J�[�g�̏����ʒu
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
	dqn.load();	//�����Ńf�[�^�ǂ�Ă݂�
#endif


    // �E�B���h�E�̍쐬
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "arm");
	//���W���ł��B
	sf::Vector2f y1(ORG_X, 0);	sf::Vector2f y2(ORG_X, ORG_Y * 2);
	sf::Vector2f x1(0, ORG_Y);	sf::Vector2f x2(ORG_X * 2, ORG_Y);
	sf::VertexArray	axis_y(sf::Lines, 2);
		axis_y[0].position = y1;			axis_y[1].position = y2;
		axis_y[0].color=sf::Color::Black;	axis_y[1].color=sf::Color::Black;
	sf::VertexArray	axis_x(sf::Lines, 2);
		axis_x[0].position = x1; 			axis_x[1].position = x2;
		axis_x[0].color=sf::Color::Black;	axis_x[1].color=sf::Color::Black;

    // �e�����N�̋�`�ł��B
   sf::RectangleShape l1(sf::Vector2f(290.f, 3.f));    l1.setFillColor(sf::Color::Blue);
   sf::RectangleShape l2(sf::Vector2f(290.f, 2.f));    l2.setFillColor(sf::Color::Green);
   sf::RectangleShape l3(sf::Vector2f(135.f, 2.f));    l3.setFillColor(sf::Color::Magenta);

    //������
    sf::Font font;
	if (!font.loadFromFile("C:\\Windows\\Fonts\\arial.ttf")) {
		printf("font load error");
	}// �t�H���g�t�@�C�����w�肵�܂�
	sf::Text txt_eps;		//��
	sf::Text txt_net_used;	//�l�b�g�g�������ǂ���
	sf::Text txt_n_learn;	//�w�K��
	sf::Text txt_n_moved;	//����������
	sf::Text txt_moved;		//
	//���x�\�����Ă݂�B
	sf::Text txt_arm_spd;
	sf::Text txt_l1_spd;
	sf::Text txt_l2_spd;
	sf::Text txt_l3_spd;
	//�e�p�x�̕\��
	sf::Text txt_arm_th;
	sf::Text txt_l1_th;
	sf::Text txt_l2_th;
	sf::Text txt_l3_th;

	{	//
		txt_eps.setFont(font);
		txt_eps.setFillColor(sf::Color::Blue);		// �����̐F��ݒ肵�܂��B
		txt_eps.setPosition(ORG_X-60, 180);
		txt_eps.setCharacterSize(15); 				// �����T�C�Y��ݒ肵�܂��i�s�N�Z���P�ʁj�B
		//
		txt_net_used.setFont(font);
		txt_net_used.setFillColor(sf::Color::Blue); // �����̐F��ݒ肵�܂��B
		txt_net_used.setPosition(ORG_X+30, 180);
		txt_net_used.setCharacterSize(15); 			// �����T�C�Y��ݒ肵�܂��i�s�N�Z���P�ʁj�B
		//
		txt_n_learn.setFont(font);
		txt_n_learn.setFillColor(sf::Color::Blue);	// �����̐F��ݒ肵�܂��B
		txt_n_learn.setPosition(ORG_X-20, 150);
		txt_n_learn.setCharacterSize(15); 			// �����T�C�Y��ݒ肵�܂��i�s�N�Z���P�ʁj�B
		//moved
		txt_moved.setFont(font);
		txt_moved.setFillColor(sf::Color::Cyan);	// �����̐F��ݒ肵�܂��B
		txt_moved.setPosition(ORG_X + 60, 150);
		txt_moved.setCharacterSize(15); 			// �����T�C�Y��ݒ肵�܂��i�s�N�Z���P�ʁj�B

		txt_n_moved.setFont(font);
		txt_n_moved.setFillColor(sf::Color::Cyan);	// �����̐F��ݒ肵�܂��B
		txt_n_moved.setPosition(ORG_X + 90, 150);
		txt_n_moved.setCharacterSize(15); 			// �����T�C�Y��ݒ肵�܂��i�s�N�Z���P�ʁj�B
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

		//���\��
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
    // ���C�����[�v
    while (window.isOpen()) {
        // �C�x���g����
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
		//�ڕW�Ɍ������ē��삳���܂��B
		if (learn) {
			dqn.learn();
		}
		else {
			dqn.play();
		}

		txt_eps.setString("e="+::to_string( dqn.eps()));		//�C�v�V�����ł��B
		if (dqn.net_used()) {
			txt_net_used.setString("NET");
			txt_net_used.setFillColor(sf::Color::Blue);
		}
		else {
			txt_net_used.setString("RND");
			txt_net_used.setFillColor(sf::Color::Red);
		}
		txt_n_learn.setString(::to_string(dqn.n_learn()));
		//���𓮂��������ǂ���
		txt_n_moved.setString(::to_string( dqn.moved));
		txt_moved.setString(dqn.move_ok?"OK":"NG");

		//�A�[���Ɗe�֐߂̑��x�ł��B
		txt_arm_spd.setString( ::to_string(dqn.state().__arm.wd.spd));
		txt_l1_spd.setString(::to_string(dqn.state().__arm.lnk(0).d.spd));
		txt_l2_spd.setString(::to_string(dqn.state().__arm.lnk(1).d.spd));
		txt_l3_spd.setString(::to_string(dqn.state().__arm.lnk(2).d.spd));
		//��
		txt_arm_th.setString(::to_string(dqn.state().__arm.rad));
		txt_l1_th.setString(::to_string(dqn.state().__arm.lnk(0).r));
		txt_l2_th.setString(::to_string(dqn.state().__arm.lnk(1).r));
		txt_l3_th.setString(::to_string(dqn.state().__arm.lnk(2).r));

        // �`��
        window.clear(sf::Color::White);

		//����͐�[�̉摜
        l1.setPosition( ORG_X ,  ORG_Y );				//����̓����N�̐�[�̍��W�B
		l1.setRotation( -1 * _deg(dqn.state().__arm.lnk(0).r));	//�p�x�������ɂ���By���͔��΂ɂȂ�B
		//���̏ꍇ�A���{�̍��W�Ȃ�ň�O�̎��̐�[���W���w�肵�Ȃ��Ƃ����Ȃ��B
		l2.setPosition( ORG_X + dqn.state().__arm.lnk(0).linked_vect().x, ORG_Y - dqn.state().__arm.lnk(0).linked_vect().y);	//l1�̐�[���W�ɂ���B
		l2.setRotation(-1 *  _deg(dqn.state().__arm.lnk(0).r + dqn.state().__arm.lnk(1).r)	);								//�p�x�������ɂ���B
		l3.setPosition( ORG_X + dqn.state().__arm.lnk(1).linked_vect().x, ORG_Y - dqn.state().__arm.lnk(1).linked_vect().y);	//l1�̐�[���W�ɂ���B
		l3.setRotation( -1 * _deg(dqn.state().__arm.lnk(0).r + dqn.state().__arm.lnk(1).r + dqn.state().__arm.lnk(2).r)	);	//�p�x�������ɂ���B


		// Create a line from (100, 100) to (700, 500)
		sf::Vector2f p1(100, 100);
		sf::Vector2f p2(700, 500);

		sf::VertexArray line(sf::Lines, 2);
		line[0].position = p1;
		line[1].position = p2;

		//�|�[���̓����ł��B
		//�|�[���p�x�̕\���ʒu

        window.draw(l1);				//
        window.draw(l2);				//
        window.draw(l3);				//
		window.draw(txt_eps);			//�C�v�V����
		window.draw(txt_net_used);		//�l�b�g���g�������ǂ���
		window.draw(txt_n_learn);		//
		window.draw(txt_moved);			//
		window.draw(txt_n_moved);			//
		//���W���ł��B
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


/*        x = cp.state().cart_position;   //�J�[�g�̈ʒu(m)
		theta = cp.state().pole_angle; //�|�[���̊p�x
		txt.setString(::to_string(x)); // �`�悷�镶�����ݒ肵�܂��B
		txt_pol_th.setString(::to_string(theta)); // �`�悷�镶�����ݒ肵�܂��B
		//x�;���ɂ��Ă݂邩
		x *= 1000;
		//�e�L�X�g���W�Ɠ��e���X�V����B
		txt.setPosition(CART_X + x-25, CART_Y + 20);
		//txt_pol_th.setPosition(CART_X + x - 25, CART_Y -150);

/*
�Edwitter�̓}�X�N�Ȃ���v���C�x�[�g�ȕ����̂��Ă���̂Őe�ߊ��łĂ����񂶂�Ȃ�����
�E
*/

//arm::moveto()�̃R�s�[�ł��B
//������������āA�����������͌�����
static int __debug_n=0;
bool arm_DQN::move(int act)//act�����������ɃA�[�������������܂��B
{
#if 0 //�����������Ȃ�
	act = 0;
/*	if (__debug_n++ < 2) { act = 0; }
	else {
		act = 1;
	}*/
#endif
	int __debug;
	//==================================================================================================================
	//	������������߂�H�Ƃ肠�����~�����܂��O���
	//==================================================================================================================
	int dir = act == 1 ? -1 : 1;		//�Ƃ肠����2���������Ƃ��ă}�C�i�X�����։�������悤�ɂ��܂��B

	//==================================================================================================================
	//�A�[���̎p���ω��ɑ΂���ω��������߂܂��B
	//�ꏊ�ɂ���āA�ړ������ɂ������āA�ϓ��ɃA�[���̎p�����ω�����悤�Ɋp�x��U�蕪���Ȃ��ƂȂ�܂���B
	//�߂��Ⴍ�������Ȃ����B�B�B�B�H�H�H
	//==================================================================================================================
//	double d_posture = 0;		//����������̎p���ω��ʂł��B
/* {
		//�~������A�p�x�Ŋ���Z���������E�E�E������
		//�~����̋����ł��B
		double d = f.dist(*this, tgt);		//���ꂪ����̈ړ������ł��B

		if (d) {		//�A�[���p���ω���
			//�p���Ɋւ���ω��ʂ͊p�x�ɂȂ��Ă��܂��B
			double dt = (tgt - *this).rad;	//���ꂪ����̑��p���ω��ʂł��B����͊p�x�ő��v�ł��B
			//����������̊p�x�ω���
			d_posture = dt / d;		//����������̎p���̕ω���(�p�x)�B�ړ�������������������ƁA�Ή������p���̕ω��ʂɂȂ�܂��B
		}
	}




	//����͓s�x�v�Z����悤�ɂ��܂�.�B

*/
//�؂�ւ��̏���
	//==================================================================================================================
	//	�����������̂��A��������̂����l���܂��B
	//==================================================================================================================
	bool acc = (act == 1) ? false : true;					//	acc 	: �ڕW���x�����ݑ��x��肨����������������(�傫����true)
	//	const double maxspd = PI / 2;
	//	max_arm_rad_in_ms
	//�Ƃ肠�����~�^���Ȃ�ŁA�p���x�������l���邱�Ƃɂ��܂��B�E�E����Ŗ{���ɂ������͂�����ƍl���Ȃ���
	//�����ɒ����Ă����Ȃ��ƂȂ�܂���
	//�������ő�AC1�̏I���łƂ܂�悤�ɂ��Ȃ��Ƃ����Ȃ�����
	const double tgt_min = _state.fnc->dist( acc ? _state.__arm.wd.spd/ 1000	:	-1 * max_arm_rad_in_ms ,true );			//	tgt_min	:	�ŏ����x
	const double tgt_max = _state.fnc->dist( acc ? max_arm_rad_in_ms			:	_state.__arm.wd.spd / 1000 , true); 	//	tgt_max	:	�ő呬�x

	//
	 
	//������āA1ms�Ői�ށA���W�A��.

/*	acc�̏ꍇ�A	�ō������́A	�ō�����1ms�œ��B���鋗���B(= max_arm_rad_in_ms)
				�Œ዗���́A	���݂̊p���x�ŁA1ms�Ői�ދ����B
								�����݃}�C�i�X�̑��x�Ői��ł���̂ł��̑��x��(_state.__arm.wd.spd)
*/

	//==================================================================================================================
	//	�ڕW�ʒu&���x�ɓ��B����܂ő����܂��B
	//	
	//==================================================================================================================
	double _last_d = 0, _last_dd = 0;		//�O��̋����ł�

//	for (; _arm != tgt;) 
	{		//�Ƃ肠���������𑬓x���B���ł����甲����ɂ��܂��B		//����́A���W�ړ��̑��x�̍œK�Ȃ��̂�T�����߂̒T���B
		_cood last_movable;
		int i;
		//max:�ő呬�x , min=�Œᑬ�x
		double max = tgt_max, min = tgt_min;		//�ڕW���x�����Z�b�g���܂��B
		//����ł���ő�̂��̂�T���񕪒T���ł��B
		int n_update = 0;	//��������̂����񌩂���ꂽ���B
		for (i = 0; (_equal<>(min, max,0.001) != true) || (last_movable.rad == 0); ++i) {	//�Ō�Ɍ�����ꂽ
			//������x�ł�߂܂��B
			if (n_update > 2) {
				break;
			}
			//__c2�~�ɂȂ��Ă����NG
			if ( (_state.fnc==&__c2) &&  _equal<>(min,max,0.001)) {	//�����T���ł��Ȃ��󋵂ɂȂ�������߂܂��B
				//				_Assert(0, "arm::move() try overflow");
				__debug = 10;
//				return false;		//���x���߂��Ă��ł��Ȃ��B
				//�Ƃ肠��������������Ă݂�B
				if (act==1) {		//������
					move(act);
				}
				return false;
				//�������Ȃ������B
//				return false;
			}
			//���񎎂����싗���ł��B
			double d		=	min + (max - min) / 2;		//���ꂪ���񂽂߂����x�ł��B(1ms�Ői�ދ���)
			//	=====================================================================================
			//	d	:	
			//	���̋���,dir�����ɐi�񂾈ʒu��Ԃ��֐���p�ӂ��܂��B
			//	now :	���鎞�ԒP�ʁi���̏ꍇ��1ms�ł��j�̈ړ���
			//	=====================================================================================
			//1ms�œ��삳���鋗�������߂܂��B
			//d�́A�������܂�ł���B�B�B
			//__c2�̉~�ɂȂ�����A�p�x�͕ς��Ȃ��B
			double arm_rad;
			{
				if (_state.fnc == &__c1) {
					arm_rad = _state.__arm.rad + _state.fnc->angle(d);		//����̋����𓮂����܂��B
				}
				else {	//�Ƃ肠�����A������Ɗp�x���ς��Ă݂�
					arm_rad = _state.__arm.rad + __c1.angle(-d);			//����̋����𓮂����܂��B
/*					if (_state.__arm.rad <5) {
						arm_rad = _state.__arm.rad + __c1.angle(-d);			//����̋����𓮂����܂��B
					}
					else {
						arm_rad = _state.__arm.rad;			//����̋����𓮂����܂��B
					}
//					arm_rad = _state.__arm.rad;			//����̋����𓮂����܂��B
*/
				}
			}
			_cood		now(_state.fnc->progress(_state.__arm, d),			arm_rad);
			//���������ȃx�N�^���A���Ă�����G���[���Ǝv���܂��B�I���ł��B
			if (now.y==0&&now.x==0) {
				return false;
			}
			
			// 
			//_state.__arm.rad + _state.fnc->angle(d));		//����̋����𓮂����܂��B
//			_state.__arm.rad + (_state.fnc == &__c1) ? _state.fnc->angle(d) : 0	);		//����̋����𓮂����܂��B
			//�����ŁA����C1�̏I���ɍ����|�����Ă��܂����ꍇ�́AC1�̏I�_�Ŏ~�߂�K�v�����邩������Ȃ��B

			link_stat res;
			if ((res = _state.__arm.move_able(now)).stat == MOV_OK) {								//��������
				++n_update;
				last_movable = now;		//�Ō�ɂ��܂����������̂��L�^���Ă����܂��B		
				//���������ꍇ�A���݂̈ʒu���Œ�8�Ƃ��āA�܂������邩��				
				if (acc) { min = d; }	//��������ꍇ�́A�Ȃ邂���傫�ȑ��x�����߂�
				else { max = d; }		//��������ꍇ�́A�Ȃ�ׂ����������x�����߂����B
			}
			else {	//��������̌����Ŏ��s���܂����B

				//���݂̃A�[���|�W�V�����ł̋t���R�r����A�O����ŃA�[���̔����ψʂ����߁A
				//�Y�������N��	�}�C�i�X���։����������̏ꍇ�ɂ́A�����N���v���X�̕ψ�
				//				�v���X���։������������ꍇ�́A�����N���}�C�i�X�̕ψ�
				//��������ɓ����������݂��A
				// 
				//	��L�̍l���͂�����ƈႤ�C������B
				//	(1)
				//		���݂̃A�[���|�W�V��������A����d �𓮂������Ƃ�����A�ǂ�����link�������Ȃ������B
				// 
				// (2)
				//	���̂Ƃ��ɁA
				// �����Ɖ��������ق��������̂��A���������ق��������̂��������N�G���[����͂킩��Ȃ��B
				//	���A�[���̉����������͌����̕����ƁA�����N�̉����A�����̕������Ⴄ�ꍇ�����邽�߁B
				//
				// (3)
				//	�����ŁA�u����Z�o�����ʒu�v�ł̋t���R�r���܂��v�Z���A
				//	��������̔����ϓ��������Ƃ��̃����N�̓������v�Z����B
				//	
				// (4)
				//	���̌v�Z���ʂ���A�Ⴆ�΁F�����N�����������������ꍇ�ɂ́A
				//	�����N�̕ψʂ����Ȃ��Ȃ�����̃A�[���̓�����������B
				// (5)
				//	���ꂪ�A�[��������ɉ���������������Ƃ���΁A�����Ƒ傫�����삷������֓������悤��
				//	min,max��ݒ肷��B
				// 
				//min,max��K�؂ɐݒ肷��B
				//�܂��t���R�r�ł��B
				Matrix3d invJ;
				//now���i�߂���Ԃ̃A�[�������܂��B
				{
					arm _tmp_arm(false);
					_tmp_arm.set(now);
					if (inv_jacobi(_tmp_arm.lnk(0).r, _tmp_arm.lnk(1).r, _tmp_arm.lnk(2).r, invJ) != true) {
						_Assert(0, "invJ is not found");
					}
				}
				//�����ŋt���R�r�����܂����̂ŁA�e�Ԑڂ̌��z�����Ă����܂��B
				//�����ŁA�Y���O���i���j��dir���ƁA-dir���ɂ�����Ƃ����������ʒu�����܂��B

				Vector3d delta_link_F, delta_link_B;
				{
					const double delta = 0.000001;	//�Ƃ肠�����������l
					_cood _dl;
//					_cood		now(_state.fnc->progress(_state.__arm, d), _state.__arm.rad + _state.fnc->angle(d));		//����̋����𓮂����܂��B

					//���ꂪ������Ƃ����������ɋO����𓮍삳�����Ƃ��̍����̍��W�ł��B
					//���A���ꂪc2�~�̂Ƃ��̑Ή��ɂȂ��Ă��Ȃ��B
					if (_state.fnc == &__c1) {
						_dl = _cood(_state.fnc->progress(now, dir * delta), now.rad + _state.fnc->angle(delta * dir)) - now;			//����̋����𓮂����܂��B
					}
					else {
						//������Ƃ�₱�������Ac2�~�̏ꍇ�́A���������}�C�i�X�Ȃ̂�-1�������܂��B����͌�ŏC�����Ȃ���
						_dl = _cood(_state.fnc->progress(now, dir * delta), now.rad + __c1.angle(-1* delta * dir)) - now;			//����̋����𓮂����܂��B
					}

					Vector3d	delta_arm_F(_dl.x, _dl.y, _dl.rad);	//�ψʗʂ�Eigen�x�N�^�ł��B
					delta_link_F = invJ * delta_arm_F;
					//���ꂪ�t�����ɂ�����Ƃ����t�����ɋO����ɓ��삳�����Ƃ��̍����̍��W�ł��B
//					_dl = _cood(f.progress(*this, -1 * dir * delta), rad + (-1 * d * d_posture)) - now;	//����̋����𓮂����܂��B
					if (_state.fnc == &__c1) {
						_dl = _cood(_state.fnc->progress(now, -1 * dir * delta), now.rad + _state.fnc->angle(-1 * delta * dir)) - now;			//����̋����𓮂����܂��B
					}
					else {
						//c2�~�̏ꍇ�́A��������-1�Ȃ̂ŁA-1�������܂��B
						_dl = _cood(_state.fnc->progress(now, -1 * dir * delta), now.rad + __c1.angle(-1*-1* delta * dir)) - now;			//����̋����𓮂����܂��B
					}
					Vector3d	delta_arm_B(_dl.x, _dl.y, _dl.rad);	//�ψʗʂ�Eigen�x�N�^�ł��B
					delta_link_B = invJ * delta_arm_B;
				}
				//���ꂪ�Y�������N�̏�Ԃɂ���āA�ǂ����������Ă��邩��
				if (res.stat == MOV_ACC_OVER) {	//�����N���v���X���ɉ����������Ăm�f�������ꍇ�F�����N�̕ψʂ��������Ȃ�悤������I������悤�ɂ��܂��B
					if (delta_link_F(res.no) < delta_link_B(res.no)) {	//�A�[�����������֓������ƁA�����N���}�C�i�X�ɓ����܂��̂ŁA���傫�Ȃق��֒T�����܂��B
						//��F(�A�[������������)��I���B���܂��B���̏ꍇ�A
						if (dir > 0) {				//dir�� + �i�A�[���̊p�x��+����������֓���j�̏ꍇ		���{������T������悤��	min������������B)
							min = d;
						} else {
							max = d;				//dir�� - (�A�[���p�x���������Ȃ�����֓��삳����ꍇ	���|����T������悤��	max������������)
						}
					}else {		//					else if (delta_link_B(res.no) < 0) {	//�A�[�����t�����֓������ƁA�����N���}�C�i�X�ɓ����B�̂ŁA��菬�����Ȃ�悤�ɒT�����܂��B
						//���a�i�A�[�����t�����֓��삳����j��I������B
						if (dir > 0) {
							max = d;				// dir ��	+ �i�A�[���p�x������������֓���j�̏ꍇ	���-���֒T��������悤��	max������������
						} else {
							min = d;				// dir ��	-  (�A�[���p�x����������֓���j�̏ꍇ		���+���֒T���������悤��	min������������
						}
					}
				}
				else if (res.stat == MOV_DEC_OVER) {	//�����N���}�C�i�X���ɉ������������ꍇ�́A�����N�̕ψʂ��傫���i�v���X���j�Ȃ�悤�ȕ����փA�[���𓮂����܂��B
					if (delta_link_F(res.no) > delta_link_B(res.no)) {
						//��F���i�A�[�����������֓��삳����j��I��:
						if (dir > 0)				//	dir ��	+	�̏ꍇ�A	���+���֒T��������悤��		min������������B
						{
							min = d;
						} else {
							max = d;				//	dir �� -	�̏ꍇ�A	���-���֒T��������悤��		max������������
						}
					}
					else {	//��B�i�A�[�����t�����ɓ��삳����j��I��:	(���a�̂ق����A�����N�̊p�x��+�ɂȂ�̂Łj
						if (dir > 0) {				//	dir ��	+	�̏ꍇ�A	���-���ɒT��������悤��		max������������B
							max = d;
						} else {
							min = d;				//	dir ��	-	�̏ꍇ�A	���+���ɒT��������悤��		min������������B
						}
					}
				}
			}
		}
		//�����ō���̍œK��������܂����̂œ������܂��B
		_state.__arm.move(last_movable);
	}
	//�����̎��_�ŁA���݂̃A�[���̈ʒu��C2�ɂ����������Ă���΁A
	if (_state.fnc == &__c1) {
		if (_state.__arm.rad >= (PI * 3 / 2)) {
			_state.fnc = &__c2;	//_c2�̉~�ɐ؂�ւ��܂��B
			//���̎��_�ŁA�A�[���̊p���x�́A�V�����O���ɍ��킹�܂��B
			//����͍l�������Ȃ��Ƃ����Ȃ�
			_state.__arm.wd.spd *= (__c1.get_dir() * __c2.get_dir());

			moved = 0;	//���������񐔂̓��Z�b�g���܂��i��V�v�Z�̂��߁j
		}
	}

	return true;
}



//�܂����K���ȗ����ł���Ă݂�B
//	�ő�̕ψʁi�A�[�����x�j�����߂Ă݂�B�~�O���̏ꍇ�A�p���x�ł��������ǁA
//	�A�[���̍ő�ψʗʂ����߂āA����ɑ΂��銄�������߂�B(10-100%)
//	(�܂��ȒP�ɁA�~�O���Ȃ̂�rad�ɂ��Ă݂�B)���̂��ɉ~���̌ʂ̒����ɂ���(���a���ς���Ă����v�Ȃ悤��)
//	
//	�ő呬�x��90��/sec�Ƃ��Ă݂�ƁA(PI/2 /sec)
//	1ms�ɐi�ޕψʂ� PI/2000 rad
//		bool move_ok;		//�����������ǂ����ł��B
		//	v_min = v - max_acc
		//	v_max = v + max_acc

		//	����̉����ł��B
		//	dv = v_min + { (v_max-v_min)*(act+1)/10
//        {
			//�ړ��������v�Z���܂��B���݂ǂ̉~���N���ɂ��邩�Ōv�Z���ς��܂��B
			//__c1��__c2�̐؂�ւ��ʒu�̔���͂ǂ����邩

			//�}�ő�����x�̒��łP�O�i�K�ɕ����܂��B
			// 

			//���j�]���F
			//(1)�A�[���̑��x���オ��ɂ�A���������Ȃ��Ă������^�����s�����Ƃ��A
			//		�e�����N�ɂƂ��Ă͂ł��Ȃ������𔺂��ꍇ������B
			//		���̂��߁A����̌v�Z�͂ł��Ȃ��B
			//(2)�l�b�g�����߂�A�N�V�������u�����v�u�������v�u�����v
			//		�̎O��ނƂ��āA���̎��ɉ\�ȉ�������s�x�����āA
			//		��������s����悤�ɂ���B
			//


			//������ƈȉ�����߂Ă݂܂��B
/*
			double rad = (2*max_acc * (act+1) / n_output()) - max_acc;
			bool		chg=false;					//
			//����̈ړ��ŁA�O�Ղ̐؂�ւ��𒴂���悤�Ȃ�΁A�����Ŏ~�߂܂��B
			if( (_state.fnc == &__c1) && (_state.__arm.rad >= (PI*3/2)) ) {	//�A�[���̊p�x�Ŕ��f���邩�H�H��������₵���ȁB�B�B
				rad = PI*3/2 - _state.__arm.rad;							//�~���̐؂�ւ�蕔���̓s�b�^���ɂȂ�悤�ɂ��܂��B
																			//����͂��Ƃōl����������
				chg = true;		//�������I�������؂�ւ���
			}
			//����i�݂���������������ƈႤ�B
			//����̉����x���A�����̑��x�ɑ����āA���̏��1ms�i�ދ����ɂȂ�܂��B
			//
			//���܂͂Ƃ肠�����A�ʒu�̉����A�����������l���āA�p���͂���ɂ��������Č��܂�Ƃ��Ă݂�B
			//�A�[���̌��ݑ��x�Ői�ދ����{�����Ői�ދ���(1ms��)

			//���̎��_��__arm��circle��ɂ���
#if 1
			if (_state.fnc->in(_state.__arm) == true) {
				__d = 0;
			}
			else {
				__d = -1;
			}
#endif
			double d = _state.__arm.d.spd / 1000 + _state.fnc->dist(rad,true);	//������L���ɂ�������
//			double d = _state.fnc->dist(rad);				//���ꂪ�i�ދ���
			//���p���́A���݂̉~����̈ʒu�ɂ������Ĉ�ӂɌ��܂�܂��B
			//���ꂪ����i�܂����������ɂȂ�܂��B
			_cood p( _state.fnc->progress( _state.__arm , d ) ,_state.__arm.rad + rad );		//�O�����d�i�݁A�p����rad���ω�������悤�ɁE�E�E

			//���̎��_�ŁAprogress�̌��ʂ��A������������~����ɂȂ��\��
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
			//���������Ƃ��܂��B
			move_ok = (_state.__arm.move(p,true).stat == MOV_OK);
			if(chg) {
				_state.fnc = &__c2;		//�O�Ղ�c2�ɐ؂�ւ��܂��B
			}
#if 1
			//NG�ɂȂ錴���������܂�
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
