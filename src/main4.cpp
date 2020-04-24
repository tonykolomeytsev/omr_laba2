#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include <iostream>
#include <math.h>

#define LOG_FILE_NAME   "./log.csv"

// from source
#define TIME_START  0.0 
#define TIME_END    2.080 // 2080 ms
#define TIME_STEP   0.020 // 20 ms (for data collecting)

#define X_START     0.439 // m
#define X_END       0.335 // m
#define Z_START     0.173 // m
#define Z_END       0.263 // m
#define PHI_4       0.9 // rad

#define PI          3.14159265
#define SQR(x)      ((x)*(x))

#define X_A1    0.15  // m
#define D1      0.033 // m
#define L1      0.075 // m
#define L2      0.155 // m
#define L4      0.081 // m
#define L5      0.137 // m
#define L3      0.135/*+L4+L5*/ // m

using namespace youbot;

class Angles
{
private:
    YouBotManipulator *ybArm;
    JointAngleSetpoint j2;
    JointAngleSetpoint j3;
    JointAngleSetpoint j4;
    JointSensedAngle jsa2;
    JointSensedAngle jsa3;
    JointSensedAngle jsa4;
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
    void getAngles(double &A2, double &A3, double &A4) {
        ybArm->getArmJoint(2).getData(jsa2);
        ybArm->getArmJoint(3).getData(jsa3);
        ybArm->getArmJoint(4).getData(jsa4);
        
        A2 = jsa2.angle / radian;
        A3 = jsa3.angle / radian;
        A4 = jsa4.angle / radian;
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
void logAll(double time, double phi2, double phi3, double phi4, double X, double Z) 
{
    FILE *l = fopen(LOG_FILE_NAME, "a");    
    fprintf(l, "%3.2f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f \n", time, phi2, phi3, phi4, X, Z);
    fclose(l); 
}

/**
 * Переход от кинематических углов к "техническим"
 * */
void toTechAngles(double phi2, double phi3, double phi4, double &A2, double &A3, double &A4) 
{
    A2 = phi2 + 1.1345;
    A3 = phi3 - 2.5654;
    A4 = phi4 + 1.829;
}

/**
 * Переход от "технических" углов к кинематическим
 * */
void fromTechAngles(double A2, double A3, double A4, double &phi2, double &phi3, double &phi4) 
{
    phi2 = A2 - 1.1345;
    phi3 = A3 + 2.5654;
    phi4 = A4 - 1.829;
}

/**
 * Закон изменения Z
 * */
double z(const double t) 
{
    // линейное движение от начальной до конечной точки
    return Z_START + (t / TIME_END) * (Z_END - Z_START);
}

/**
 * Закон изменения X
 * */
double x(const double t)
{
    // линейное движение от начальной до конечной точки
    return X_START + (t / TIME_END) * (X_END - X_START);
}

// вычисление Якобиана
void calcJ(double J[2][2], double phi2, double phi3) 
{ 
    const double c2 = cos(phi2), c23 = cos(phi2 + phi3);
    const double s2 = sin(phi2), s23 = sin(phi2 + phi3);
    J[0][0] = c2*L2+c23*L3; J[0][1] = c23*L3;
    J[1][0] =-s2*L2-s23*L3; J[1][1] =-s23*L3;
}

// решение прямой задачи
void calcX(double* X, double phi2, double phi3) 
{
    const double c2 = cos(phi2), c23 = cos(phi2 + phi3);
    const double s2 = sin(phi2), s23 = sin(phi2 + phi3);
    X[0] = D1 + L2*s2 + L3*s23 + (L4+L5)*sin(PHI_4);
    X[1] = L1 + L2*c2 + L3*c23 + (L4+L5)*cos(PHI_4);
}

// вычисление псевдообратной матрицы 💦💦💦
void calcInvJ(double J[2][2], double invJ[2][2]) 
{ 
    double  a = J[0][0], b = J[0][1],
            c = J[1][0], d = J[1][1];
    invJ[0][0] = d / (a*d - b*c); invJ[0][1] = b / (b*c - a*d);
    invJ[1][0] = c / (b*c - a*d); invJ[1][1] = a / (a*d - b*c);
}

// норма
double norm2(double* E) { return sqrt(SQR(E[0]) + SQR(E[1])); }

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
            createLog();
            Angles angles(ybArm); // для упрощенного управления углами
            // рабочие величины
            double J[2][2] = {{0, 0}, {0, 0}};
            double invJ[2][2] = {{0, 0}, {0, 0}};
            double X[2] = {0, 0};
            double E[2] = {1, 1}; 

            // движение вперед
            for (double t = TIME_START; t <= TIME_END; t += TIME_STEP) 
            {
                E[0] = 1; E[1] = 1; // сбрасываем ошибку
                double phi2 = PI/6;
                double phi3 = PI/6;

                int i = 0; // на всякий случай ограничим кол-во итераций
                while (norm2(E) > 0.001 && i++ < 10) {
                    // находим якобиан
                    calcJ(J, phi2, phi3);
                    // решаем прямую задачу для текущих углов
                    calcX(X, phi2, phi3); 
                    // сравниваем решение прямой задачи с требуемыми координатами схвата
                    E[0] = X[0] - x(t); E[1] = X[1] - z(t);
                    // найдем псевдообратную матрицу якобиана
                    calcInvJ(J, invJ);
                    // находим компоненты векторы полного шага 
                    double pPhi2 = invJ[0][0] * E[0] + invJ[0][1] * E[1];
                    double pPhi3 = invJ[1][0] * E[0] + invJ[1][1] * E[1];

                    // делаем приращение к искомым значениям углов
                    phi2 -= pPhi2;
                    phi3 -= pPhi3;
                }

                double phi4 = PHI_4;

                double A2, A3, A4;
                toTechAngles(phi2, phi3, phi4, A2, A3, A4); // переходим к техническим углам
                angles.setAngles(A2, A3, A4); // поворачиваем приводы на нужные углы
                SLEEP_MILLISEC(TIME_STEP); // ждем

                // angles.getAngles(A2, A3, A4); // измеряем реальные углы поворота приводов
                // fromTechAngles(A2, A3, A4, phi2, phi3, phi4); // переходим к кинематическим углам
                logAll(t, phi2, phi3, phi4, X[0], X[1]); // выводим в лог время и измеренные углы
            }

            // движение назад
            for (double t = TIME_END; t >= TIME_START; t -= TIME_STEP) 
            {
                E[0] = 1; E[1] = 1; // сбрасываем ошибку
                double phi2 = PI/6;
                double phi3 = PI/6;

                int i = 0; // на всякий случай ограничим кол-во итераций
                while (norm2(E) > 0.001 && i++ < 10) {
                    // находим якобиан
                    calcJ(J, phi2, phi3);
                    // решаем прямую задачу для текущих углов
                    calcX(X, phi2, phi3); 
                    // сравниваем решение прямой задачи с требуемыми координатами схвата
                    E[0] = X[0] - x(t); E[1] = X[1] - z(t);
                    // найдем псевдообратную матрицу якобиана
                    calcInvJ(J, invJ);
                    // находим компоненты векторы полного шага 
                    double pPhi2 = invJ[0][0] * E[0] + invJ[0][1] * E[1];
                    double pPhi3 = invJ[1][0] * E[0] + invJ[1][1] * E[1];

                    // делаем приращение к искомым значениям углов
                    phi2 -= pPhi2;
                    phi3 -= pPhi3;
                }

                double phi4 = PHI_4;

                double A2, A3, A4;
                toTechAngles(phi2, phi3, phi4, A2, A3, A4); // переходим к техническим углам
                angles.setAngles(A2, A3, A4); // поворачиваем приводы на нужные углы
                SLEEP_MILLISEC(TIME_STEP); // ждем

                // angles.getAngles(A2, A3, A4); // измеряем реальные углы поворота приводов
                // fromTechAngles(A2, A3, A4, phi2, phi3, phi4); // переходим к кинематическим углам
                logAll(2*TIME_END-t, phi2, phi3, phi4, X[0], X[1]); // выводим в лог время и измеренные углы
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

