#include "YouBotKDLInterface.hpp"

YouBotKDLInterface::YouBotKDLInterface(const std::string name)
			: YouBotManipulator(name)
{

}

YouBotKDLInterface::~YouBotKDLInterface()
{
}

void YouBotKDLInterface::doInitialize()
{

    YouBotManipulator::doJointCommutation();
    YouBotManipulator::calibrateManipulator();

}

void YouBotKDLInterface::setJointData(const KDL::JntArray& JointData)
{
	std::vector<youbot::JointAngleSetpoint> jointSetAngle;
	jointSetAngle.resize(5);

	jointSetAngle[0].angle = JointData(0) * radian;
	jointSetAngle[1].angle = JointData(1) * radian;
	jointSetAngle[2].angle = JointData(2) * radian;
	jointSetAngle[3].angle = JointData(3) * radian;
	jointSetAngle[4].angle = JointData(4) * radian;

	YouBotManipulator::setJointData(jointSetAngle);
}

