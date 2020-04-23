/**
 * Программа, которая работает.
 * Коломейцев А.А., Пережилова Е.Д.
 * */
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include <iostream>
#include <math.h>

#define LOG_FILE_NAME   "./log.csv"
#define EPSILON_2       0.01

// from source
#define TIME_START  0.0 
#define TIME_END    2.080 // 2080 ms
#define TIME_STEP   0.020 // 20 ms (for data collecting)

#define X_START     0.297 // m
#define X_END       0.425 // m
#define Z_START     0.384 // m
#define Z_END       0.299 // m
#define PHI_4       0.9 // rad

#define PI          3.14159265
#define SQR(x)      ((x)*(x))

#define X_A1    0.15  // m
#define D1      0.033 // m
#define L1      0.075 // m
#define L2      0.155 // m
#define L4      0.081 // m
#define L5      0.137 // m
#define L3      0.135+L4+L5 // m

using namespace youbot;

/**
 * КЛАСС_ОБЕРТКА 
 * для сокращения количества кода) 💩💩💩
 * */
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
        // ybArm->getArmJoint(2).getData(jsa2);
        // ybArm->getArmJoint(3).getData(jsa3);
        // ybArm->getArmJoint(4).getData(jsa4);
        
        // A2 = jsa2.angle / radian;
        // A3 = jsa3.angle / radian;
        // A4 = jsa4.angle / radian;
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

            // движение вперед
            for (double t = TIME_START; t <= TIME_END; t += TIME_STEP) 
            {
                double X_ = x(t) - D1;
                double Z_ = z(t) - L1;
                double D_ = (SQR(X_) + SQR(Z_) - (SQR(L3) + SQR(L2))) / (2 * L2 * L3);
                double phi3 = acos(D_);

                double beta_ = atan2(Z_, X_);
                double L_ = sqrt(SQR(X_) + SQR(Z_));
                double gamma_ = acos((SQR(L_) + SQR(L2) - SQR(L3)) / (2 * L_ * L2));
                double phi2 = (PI/2) - gamma_ - beta_;

                double phi4 = (PI/2) - phi3 - phi2;

                double A2, A3, A4;
                toTechAngles(phi2, phi3, phi4, A2, A3, A4); // переходим к техническим углам
                angles.setAngles(A2, A3, A4); // поворачиваем приводы на нужные углы
                SLEEP_MILLISEC(TIME_STEP); // ждем

                //angles.getAngles(A2, A3, A4); // измеряем реальные углы поворота приводов
                //fromTechAngles(A2, A3, A4, phi2, phi3, phi4); // переходим к кинематическим углам
                logAll(t, phi2, phi3, phi4, X_, Z_); // выводим в лог время и измеренные углы
            }
            
            // движение назад
            for (double t = TIME_END; t >= TIME_START; t -= TIME_STEP) 
            {
                double X_ = x(t) - D1;
                double Z_ = z(t) - L1;
                double D_ = (SQR(X_) + SQR(Z_) - (SQR(L3) + SQR(L2))) / (2 * L2 * L3);
                double phi3 = acos(D_);

                double beta_ = atan2(Z_, X_);
                double L_ = sqrt(SQR(X_) + SQR(Z_));
                double gamma_ = acos((SQR(L_) + SQR(L2) - SQR(L3)) / (2 * L_ * L2));
                double phi2 = (PI/2) - gamma_ - beta_;

                double phi4 = (PI/2) - phi3 - phi2;

                double A2, A3, A4;
                toTechAngles(phi2, phi3, phi4, A2, A3, A4); // переходим к техническим углам
                angles.setAngles(A2, A3, A4); // поворачиваем приводы на нужные углы
                SLEEP_MILLISEC(TIME_STEP); // ждем

                //angles.getAngles(A2, A3, A4); // измеряем реальные углы поворота приводов
                //fromTechAngles(A2, A3, A4, phi2, phi3, phi4); // переходим к кинематическим углам
                logAll(t, phi2, phi3, phi4, X_, Z_); // выводим в лог время и измеренные углы
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

