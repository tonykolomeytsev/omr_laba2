#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include <iostream>
#include <math.h>

#define TASK_NUMBER 	2
#define LOG_FILE_NAME 	"./log.csv"
#define EPSILON_2 		0.01

#define TIME_START 	0.0
#define TIME_END	10.0
#define TIME_STEP 	0.1

#define PI 		3.14159265
#define SQR(x)  ((x)*(x))

#define X_A1 	0.15
#define D1  	0.033
#define L1  	0.075
#define L2 		0.155
#define L3 		0.135
#define L4 		0.081
#define L5 		0.137

using namespace youbot;

class Angles
{
private:
	YouBotManipulator *ybArm;
	JointAngleSetpoint j2;
	JointAngleSetpoint j3;
	JointAngleSetpoint j4;
public:
	Angles(YouBotManipulator *&ybArm) {
		this->ybArm = ybArm;
	}
	~Angles() { /* nothing */ }
	void setAngles(double A2, double A3, double A4) {
		j2.angle = A2 * radian;
		j3.angle = A3 * radian;
		j4.angle = A4 * radian;
		ybArm->getArmJoint(2).setData(j2);
		ybArm->getArmJoint(3).setData(j3);
		ybArm->getArmJoint(4).setData(j4);
	}
};

/**
 * Определяем конфигурацию робота (наличие колес или манипулятора)
 * */
void initYoubotConfiguration(YouBotBase *&ybBase, YouBotManipulator *&ybArm) 
{
	try {
		ybBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		ybBase->doJointCommutation();
	} catch (std::exception& e) {
		LOG(warning) << "Unable to define robot base: ";
		LOG(warning) << e.what();
	}

	try {
		ybArm = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		ybArm->doJointCommutation();
		ybArm->calibrateManipulator();
	} catch (std::exception& e) {
		LOG(warning) << "Unable to define robot arm: ";
		LOG(warning) << e.what();
	}
}

/**
 * Создаем файлик для логирования углов поворота звеньев
 * */
void createLog() 
{
	FILE *l = fopen(LOG_FILE_NAME, "w");
    fclose(l);
}

/**
 * Логирование углов
 * */
void logAngles(double time, double phi2, double phi3, double phi4) 
{
	FILE *l = fopen(LOG_FILE_NAME, "a");	
	fprintf(l, "%3.2f, %8.6f, %8.6f, %8.6f \n", time, phi2, phi3, phi4);
    fclose(l); 
}

void toTechAngles(
	const double phi2, 
	const double phi3, 
	const double phi4, 
	double &A2, 
	double &A3, 
	double &A4
) {
	A2 = phi2 + 1.1345;
	A3 = phi3 - 2.5654;
	A4 = phi4 + 1.829;
}

/**
 * РЕШЕНИЕ 1: АНАЛИТИЧЕСКОЕ 😄
 * */
void task1(YouBotBase *&ybBase, YouBotManipulator *&ybArm) 
{
	createLog();
	auto angles = new Angles(ybArm);
	auto z = [=](double t) -> double { return 0.2 + 0.1 * cos((2 * PI * t) / 10); };

	for (double t = TIME_START; t <= TIME_END; t += TIME_STEP) 
	{
		double X_ = X_A1 - D1;
		double Z_ = z(t) - L1;
		double D_ = (SQR(X_) + SQR(Z_) - (SQR(L3) + SQR(L2))) / (2 * L2 * L3);
		double phi3 = acos(D_);

		double beta_ = atan2(Z_, X_);
		double L_ = sqrt(SQR(X_) + SQR(Z_));
		double gamma_ = acos((SQR(L_) + SQR(L2) - SQR(L3)) / (2 * L_ * L2));
		double phi2 = (PI/2) - gamma_ - beta_;

		double phi4 = (PI/2) - phi3 - phi2;

		logAngles(t, phi2, phi3, phi4);

		// переходим к техническим углам, они нужны для управления
		double A2, A3, A4;
		toTechAngles(phi2, phi3, phi4, A2, A3, A4);
		angles->setAngles(A2, A3, A4);
	}

	delete angles;
	angles = nullptr;
}


/**
 * РЕШЕНИЕ 2: ЧИСЛЕННЫЙ МЕТОД, КООРДИНАТЫ
 * */
void task2(YouBotBase *&ybBase, YouBotManipulator *&ybArm) 
{
	createLog();
	auto angles = new Angles(ybArm);
	auto z = [=](double t) -> double { return 0.2 + 0.1 * cos((2 * PI * t) / 10); };
	// рабочие величины
	double J[2][2] = {{0, 0}, {0, 0}};
	double invJ[2][2] = {{0, 0}, {0, 0}};
	double X[2] = {0, 0};
	double E[2] = {1, 1}; 
	// вычисление Якобиана
	auto calcJ = [&](double J[2][2], double phi2, double phi3) -> void { 
		const double c2 = cos(phi2), c23 = cos(phi2 + phi3);
		const double s2 = sin(phi2), s23 = sin(phi2 + phi3);
		J[0][0] = c2*L2+c23*L3; J[0][1] = c23*L3;
		J[1][0] =-s2*L2-s23*L3; J[1][1] =-s23*L3;
	};
	// решение прямой задачи
	auto calcX = [&](double* X, double phi2, double phi3) -> void {
		const double c2 = cos(phi2), c23 = cos(phi2 + phi3);
		const double s2 = sin(phi2), s23 = sin(phi2 + phi3);
		X[0] = D1 + L2*s2 + L3*s23;
    	X[1] = L1 + L2*c2 + L3*c23;
	};
	// вычисление псевдообратной матрицы
	auto calcInvJ = [&](double J[2][2], double invJ[2][2]) -> void { 
		double 	a = J[0][0], b = J[0][1],
				c = J[1][0], d = J[1][1];
		invJ[0][0] = d / (a*d - b*c); invJ[0][1] = b / (b*c - a*d);
		invJ[1][0] = c / (b*c - a*d); invJ[1][1] = a / (a*d - b*c);
	};
	// норма 2
	auto norm2 = [](double* E) -> double { return sqrt(SQR(E[0]) + SQR(E[1])); };
	
	for (double t = TIME_START; t <= TIME_END; t += TIME_STEP) 
	{
		E[0] = 1; E[1] = 1; // сбрасываем ошибку
		double phi2 = 1;
		double phi3 = 1;

		int i = 0; // на всякий случай ограничим кол-во итераций
		while (norm2(E) > 0.01 && i++ < 10) {
			// находим якобиан
			calcJ(J, phi2, phi3);
			// решаем прямую задачу для текущих углов
			calcX(X, phi2, phi3); 
			// сравниваем решение прямой задачи с требуемыми координатами схвата
			E[0] = X[0] - X_A1; E[1] = X[1] - z(t);
			// найдем псевдообратную матрицу якобиана
			calcInvJ(J, invJ);
			// находим компоненты векторы полного шага 
			double pPhi2 = invJ[0][0] * E[0] + invJ[0][1] * E[1];
			double pPhi3 = invJ[1][0] * E[0] + invJ[1][1] * E[1];

			// делаем приращение к искомым значениям углов
			phi2 -= pPhi2;
			phi3 -= pPhi3;
		}

		double phi4 = (PI/2) - phi3 - phi2;
		logAngles(t, phi2, phi3, phi4);

		// переходим к техническим углам, они нужны для управления
		double A2, A3, A4;
		toTechAngles(phi2, phi3, phi4, A2, A3, A4);
		angles->setAngles(A2, A3, A4);
	}


	delete angles;
	angles = nullptr;
}


/**
 * РЕШЕНИЕ 3: ЧИСЛЕННЫЙ МЕТОД, СКОРОСТИ
 * */
void task3(YouBotBase *&ybBase, YouBotManipulator *&ybArm) {

}

int main() 
{
	/* create handles for youBot base and manipulator (if available) */
	YouBotBase *ybBase = nullptr;
	YouBotManipulator *ybArm = nullptr;
	initYoubotConfiguration(ybBase, ybArm);

	try {
		// if ybBase != null
		if (ybBase) {
			// mock
		}

		// if ybArm != null
		if (ybArm) {
			switch (TASK_NUMBER)
			{
			case 1:
				task1(ybBase, ybArm);
				break;
			case 2:
				task2(ybBase, ybArm);
				break;
			case 3:
				task3(ybBase, ybArm);
				break;
			
			default:
				break;
			}
		}

	} catch (std::exception& e) {
		std::cout << "Unhandled exception:" << std::endl;
		std::cout << e.what() << std::endl;
	}

	/* clean up memory */
	if (ybBase) {
		delete ybBase;
		ybBase = nullptr;
	}
	if (ybArm) {
		delete ybArm;
		ybArm = nullptr;
	}
	return 0;
}

