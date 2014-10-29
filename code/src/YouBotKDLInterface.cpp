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

void YouBotKDLInterface::setJointPosition(const KDL::JntArray& JointData)
{
	std::vector<youbot::JointAngleSetpoint> jointSetAngle;
	jointSetAngle.resize(ARMJOINTS);

	jointSetAngle[0].angle = JointData(0) * radian;
	jointSetAngle[1].angle = JointData(1) * radian;
	jointSetAngle[2].angle = JointData(2) * radian;
	jointSetAngle[3].angle = JointData(3) * radian;
	jointSetAngle[4].angle = JointData(4) * radian;

	YouBotManipulator::setJointData(jointSetAngle);
}


///commands positions or angles to all manipulator joints
///all positions will be set at the same time
///@param JointData the to command positions
void YouBotKDLInterface::setJointVelocity(const KDL::JntArray& JointData)
{

	std::vector<youbot::JointVelocitySetpoint> jointSetVelocity;
	jointSetVelocity.resize(ARMJOINTS);

	jointSetVelocity[0].angularVelocity = JointData(0) * radian_per_second;
	jointSetVelocity[1].angularVelocity = JointData(1) * radian_per_second;
	jointSetVelocity[2].angularVelocity = JointData(2) * radian_per_second;
	jointSetVelocity[3].angularVelocity = JointData(3) * radian_per_second;
	jointSetVelocity[4].angularVelocity = JointData(4) * radian_per_second;

	YouBotManipulator::setJointData(jointSetVelocity);
}
    


///commands positions or angles to all manipulator joints
///all positions will be set at the same time
///@param JointData the to command positions
void YouBotKDLInterface::setJointCurrent(const KDL::JntArray& JointData)
{

	std::vector<youbot::JointCurrentSetpoint> jointSetCurrent;
	jointSetCurrent.resize(ARMJOINTS);

	jointSetCurrent[0].current = JointData(0) * si::ampere;
	jointSetCurrent[1].current = JointData(1) * si::ampere;
	jointSetCurrent[2].current = JointData(2) * si::ampere;
	jointSetCurrent[3].current = JointData(3) * si::ampere;
	jointSetCurrent[4].current = JointData(4) * si::ampere;

	YouBotManipulator::setJointData(jointSetCurrent);
}


///reads Current of all manipulator joints
///@param data returns the current per joint
void YouBotKDLInterface::getJointCurrent(KDL::JntArray& data)
{
    data.resize(ARMJOINTS);
	std::vector<youbot::JointSensedCurrent> jointSensedCurrent;
	jointSensedCurrent.resize(ARMJOINTS);  
	YouBotManipulator::getJointData(jointSensedCurrent);
	
	data(0) = (double) jointSensedCurrent[0].current.value();
	data(1) = (double) jointSensedCurrent[1].current.value();
	data(2) = (double) jointSensedCurrent[2].current.value();
	data(3) = (double) jointSensedCurrent[3].current.value();
	data(4) = (double) jointSensedCurrent[4].current.value();
	
}

///reads Position of all manipulator joints
///@param data returns the Position per joint
void YouBotKDLInterface::getJointPosition(KDL::JntArray& data)
{
    data.resize(ARMJOINTS);
	std::vector<youbot::JointSensedAngle> jointSensedAngle;
	jointSensedAngle.resize(ARMJOINTS);  
	YouBotManipulator::getJointData(jointSensedAngle);
	
	data(0) = (double) jointSensedAngle[0].angle.value();
	data(1) = (double) jointSensedAngle[1].angle.value();
	data(2) = (double) jointSensedAngle[2].angle.value();
	data(3) = (double) jointSensedAngle[3].angle.value();
	data(4) = (double) jointSensedAngle[4].angle.value();
	
}

///reads Velocity of all manipulator joints
///@param data returns the Velocity per joint
void YouBotKDLInterface::getJointVelocity(KDL::JntArray& data)
{
    data.resize(ARMJOINTS);
	std::vector<youbot::JointSensedVelocity> jointSensedVelocity;
	jointSensedVelocity.resize(ARMJOINTS);  
	YouBotManipulator::getJointData(jointSensedVelocity);
	
	data(0) = (double) jointSensedVelocity[0].angularVelocity.value();
	data(1) = (double) jointSensedVelocity[1].angularVelocity.value();
	data(2) = (double) jointSensedVelocity[2].angularVelocity.value();
	data(3) = (double) jointSensedVelocity[3].angularVelocity.value();
	data(4) = (double) jointSensedVelocity[4].angularVelocity.value();
	
}
