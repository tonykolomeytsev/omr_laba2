#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

using namespace youbot;

void initYoubotConfiguration(bool &youBotHasBase, bool &youBotHasArm) 
{
	try {
		ybBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		ybBase->doJointCommutation();

		youBotHasBase = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	try {
		ybArm = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		ybArm->doJointCommutation();
		ybArm->calibrateManipulator();

		youBotHasArm = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}
}

int main2() 
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

	/* clean up */
	if (ybBase) {
		delete ybBase;
		ybBase = 0;
	}
	if (ybArm) {
		delete ybArm;
		ybArm = 0;
	}

	LOG(info) << "Done.";

	return 0;
}

