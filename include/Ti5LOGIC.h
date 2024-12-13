#ifndef _TI5LOGIC_H_
#define _TI5LOGIC_H_
#include <mathfunc.h>
#include <iostream>
#include <time.h>
#include <cmath>
#include <tool.h>
using namespace std;

extern "C"
{ // 添加extern "C"
	class pos_trans
	{
	protected:
		float orij[7];
		float prcj = 0.017;
		float scd = sin(prcj) * sin(prcj);
		float prc1 = float(int(10000 * sin(prcj) + 1)) / 10000;
		float prc2 = prc1 * (prc1 + 2);
		float prc3 = prc1 * (3 + prc1 + prc1 * prc1) + prc2;
		float P_0[19][3] = {{0, 0, 0}}; // 机械臂按线性顺序的点集
		struct ROD
		{
			int i1, i2; // 连杆端点为P_0[i1]和P_0[i2]
			float r;	// 连杆半径
			float l;	// 连杆长度
		};
		ROD rod[18]; // 机械臂按线性顺序的连杆集
		float len[18]; // 数学模型的长度参数（与rod.l独立）
		int rodnum; // 连杆总数
		// 检测两连杆是否碰撞，P1、P2构成一根连杆，半径为r1；P3、P4构成另一根连杆，半径为r2
		bool LLcolsp(float P1[3], float P2[3], float P3[3], float P4[3], float r1, float r2);
		void init_rodindex();
		// 若点距和连杆长度一致则认为点坐标值正确
		bool check_Points();
		// 根据变换矩阵计算位姿
		void MatrixT2ypr(float T[4][4]);
		// 根据位姿计算变换矩阵
		void Pos2MatrixT(float T[4][4]);
		// 修正角度并检测关节范围是否合法（相邻连杆间是否碰撞）
		bool mendjoints();
		bool checkcalj();
		// 检测所有连杆间的碰撞
		bool check_colsp();
		
	public:
		float jr1[7] = {-pi, -pi, -pi, -pi, -pi, -pi, -pi}; // 关节静态角度范围
		float jr2[7] = {pi, pi, pi, pi, pi, pi, pi};
		float gap0, gap;
		float ypr[6] = {0, 0, 0};
		float j[7] = {0, 0, 0, 0, 0, 0, 0};
		void showpointsinfo();
	};

	class humanoidLeftArm : public pos_trans
	{
	protected:
		// 初始化固有的机械结构
		void init_arm_structure()
		{
			rodnum = 3;
			rod[0].r = 10, rod[1].r = 10, rod[2].r = 10;	// 连杆半径
			rod[0].l = 171, rod[1].l = 250, rod[2].l = 250; // 连杆长度
			init_rodindex();
		}
		// 初始化数学模型参数
		void init_model_structure()
		{
			float M = 1.57;
			jr1[0] = -M, jr1[1] = -M, jr1[2] = -0.5, jr1[3] = -M, jr1[4] = -M, jr1[5] = -M, jr1[6] = -pi/6;
			jr2[0] = M, jr2[1] = M, jr2[2] = 0.5, jr2[3] = 0, jr2[4] = M, jr2[5] = M, jr2[6] = M;
			gap = abs(jr1[0]);
			for (int i = 1; i < 7; i++)
				if (abs(jr1[i]) > gap)
					gap = abs(jr1[i]);
			for (int i = 0; i < 7; i++)
				if (abs(jr2[i]) > gap)
					gap = abs(jr2[i]);
			gap0 = gap;
			len[0] = rod[0].l, len[1] = rod[1].l, len[2] = rod[2].l;
			P_0[1][0] = 0, P_0[1][1] = len[0], P_0[1][2] = 0;
			float goal_j[7] = {0, 0, 0, 0, 0, 0, 0};
			l_forward_move(goal_j); // show_value("pos",pos,6);
			ypr[0]=ypr[2]=0;
		}

		// 点或向量在不同坐标系转换（p=true表示点，p=false表示向量）
		void fromS0toS1(float P0[3], float P[3], bool p);
		void fromS1toS2(float P0[3], float P[3], bool p);
		void fromS2toS3(float P0[3], float P[3], bool p);
		void fromS3toS4(float P0[3], float P[3], bool p);
		void fromS4toS5(float P0[3], float P[3], bool p);

		void fromS5toS6(float P0[3], float P[3], bool p);
		void fromS6toS7(float P0[3], float P[3], bool p);

		void fromS1toS0(float P0[3], float P[3], bool p);
		void fromS2toS1(float P0[3], float P[3], bool p);
		void fromS3toS2(float P0[3], float P[3], bool p);
		void fromS4toS3(float P0[3], float P[3], bool p);
		void fromS5toS4(float P0[3], float P[3], bool p);
		void fromS6toS5(float P0[3], float P[3], bool p);
		void fromS7toS6(float P0[3], float P[3], bool p);
		bool solve_in_S3(float j0,float *d23_2);
		bool solve_in_S2();
		// 逆运动解算器
		bool Points2J();
		// 根据角度计算点变换矩阵
		void J2MatrixT(float T[4][4]);

	public:
		humanoidLeftArm();
		// 检测逆运动解出的角是否满足原始位姿
		bool checkacc();
		// 正运动
		bool l_forward_move(float goal_j[7]);

		// 获取当前角度
		void get_crt_j(float joints[7]);

		// 获取当前位置
		void get_crt_pos(float pos[6]);

		/*
		  参数：
			pos:目标位置
			value：dim的值
			dim：-1~2 代表x,y,z，当值是-1的时候忽略臂角值，只接收末端位姿的值
			absolute：true的时候是绝对位置（以胸部原点位置），false的时候是相对位置（以当前点胳膊轴位置）
		*/
		bool l_backward_move(float pos[6], float value, int dim, bool absolute);
		// 测试fromSi2Sj函数是否正确（是否抄错）
		void testj2p();
		void show_crt_pos();
		void show_crt_j();
	};
	class humanoidRightArm : public humanoidLeftArm
	{
	private:
		void get_r_ypr(float r_ypr[3]);
	public:
		humanoidRightArm();
		bool r_forward_move(float goal_j[7]);
		void get_crt_j(float joints[7]);
		void get_crt_pos(float postrue[6]);
		bool r_backward_move(float postrue[6], float value, int dim, bool absolute);
		void show_crt_pos();
		void show_crt_j();
		void showpointsinfo();
	};

} // 添加extern "C"
#endif
