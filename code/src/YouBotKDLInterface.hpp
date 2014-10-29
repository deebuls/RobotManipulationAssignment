
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbot/DataTrace.hpp"

 
using namespace KDL;
using namespace youbot;

#define ARMJOINTS 5

class YouBotKDLInterface : public YouBotManipulator
{
	public:
	

   	    
		YouBotKDLInterface(const std::string name);
		virtual ~YouBotKDLInterface();

        ///Intialize the variables of the Manipulator
	    virtual void doInitialize();
	    
		///commands positions or angles to all manipulator joints
		///all positions will be set at the same time
		///@param JointData the to command positions
		virtual void setJointPosition(const KDL::JntArray& JointData);

		///commands positions or angles to all manipulator joints
		///all positions will be set at the same time
		///@param JointData the to command positions
		virtual void setJointCurrent(const KDL::JntArray& JointData);

		///commands positions or angles to all manipulator joints
		///all positions will be set at the same time
		///@param JointData the to command positions
		virtual void setJointVelocity(const KDL::JntArray& JointData);


		///reads Current of all manipulator joints
		///@param data returns the current per joint
		virtual void getJointCurrent(KDL::JntArray& data);
 
		///reads Position of all manipulator joints
		///@param data returns the Position per joint
		virtual void getJointPosition(KDL::JntArray& data);
		
		///reads Velocity of all manipulator joints
		///@param data returns the Velocity per joint
		virtual void getJointVelocity(KDL::JntArray& data);
		
	

};
