#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

using namespace youbot;

void initYoubotConfiguration(YouBotBase *ybBase, YouBotManipulator *ybArm) 
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

int main() 
{
	/* create handles for youBot base and manipulator (if available) */
	YouBotBase *ybBase = nullptr;
	YouBotManipulator *ybArm = nullptr;
	initYoubotConfiguration(ybBase, ybArm);

	/* Variable for the arm. */
	JointAngleSetpoint desiredJointAngle1;
	JointAngleSetpoint desiredJointAngle2;
	JointAngleSetpoint desiredJointAngle3;
	JointAngleSetpoint desiredJointAngle4;
	JointSensedAngle   phi1a;
	JointSensedAngle   phi2a;
	JointSensedAngle   phi3a;
	JointSensedAngle   phi4a;

	try {
		// if ybBase != null
		if (ybBase) {
			// mock
		}

		// if ybArm != null
		if (ybArm) {

			/* unfold arm 
			 * all of the following constants are empirically determined to move the arm into the desired position 
			 */
			desiredJointAngle1.angle = 1.1 * radian;
			ybArm->getArmJoint(1).setData(desiredJointAngle1);

			desiredJointAngle2.angle = 1.05 * radian;
			ybArm->getArmJoint(2).setData(desiredJointAngle2);

			desiredJointAngle3.angle = -1.05 * radian;
			ybArm->getArmJoint(3).setData(desiredJointAngle3);


			desiredJointAngle4.angle = 1.05 * radian;
			ybArm->getArmJoint(4).setData(desiredJointAngle4);
				
			SLEEP_MILLISEC(1000);

			desiredJointAngle1.angle = 1.55 * radian;
			ybArm->getArmJoint(1).setData(desiredJointAngle1);

			desiredJointAngle2.angle = 1.05 * radian;
			ybArm->getArmJoint(2).setData(desiredJointAngle2);

			desiredJointAngle3.angle = -1.05 * radian;
			ybArm->getArmJoint(3).setData(desiredJointAngle3);

			desiredJointAngle4.angle = 1.55 * radian;
			ybArm->getArmJoint(4).setData(desiredJointAngle4);
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

