/**
 * THIS IS THE MOCK FILE
 **/
#include <vector>
#include <iostream>
#include "Constants.hpp"
#define ARM_DOF 5

namespace youbot {
    

    class JointAngleSetpoint
    {
    private:
        /* data */
    public:
        double angle = 0;
        JointAngleSetpoint(/* args */) { /* mock */ }
        ~JointAngleSetpoint() { /* mock */ }
    };


    class JointSensedAngle
    {
    private:
        /* data */
    public:
        double angle = 0;
        JointSensedAngle(/* args */) { /* mock */ }
        ~JointSensedAngle() { /* mock */ }
    };


    class ArmJoint
    {
    private:
        int number;
        double angle;
    public:
        ArmJoint(const int number) {
            this->number = number;
        }
        ~ArmJoint() { /* mock */ }
        void setData(const JointAngleSetpoint setpoint) {
            this->angle = setpoint.angle;
            LOG(info) << "JOINT #" << this->number << " SET ANGLE " << this->angle << std::endl;
        }
        void getData(JointSensedAngle sensed) {
            sensed.angle = this->angle;
        }
    };
    
    
    class YouBotManipulator
    {
    private:
        std::vector<ArmJoint> *joints;
    public:
        YouBotManipulator(const char* mockName, const char* mockFileName) { 
            joints = new std::vector<ArmJoint>();
            for (int i = 0; i< ARM_DOF; i++)
                joints->push_back(ArmJoint(i));
        }
        ~YouBotManipulator() { 
            if (joints) {
                delete joints;
                joints = nullptr;
            }
        }
        void doJointCommutation() { /* mock */ }
        void calibrateManipulator() { /* mock */ }
        ArmJoint getArmJoint(const int jointNumber) {
            if (jointNumber >= 1 && jointNumber <= 5)
                return joints->at(jointNumber - 1);
        }
    };
}
