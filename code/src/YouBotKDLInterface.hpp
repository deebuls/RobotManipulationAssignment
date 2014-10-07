
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

class YouBotKDLInterface : public YouBotManipulator
{
	public:
		YouBotKDLInterface(const std::string name);
		virtual ~YouBotKDLInterface();

		///commands positions or angles to all manipulator joints
		///all positions will be set at the same time
		///@param JointData the to command positions
		virtual void setJointData(const KDL::JntArray& JointData);
		 
 
        virtual void doInitialize();
};
