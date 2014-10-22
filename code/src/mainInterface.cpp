
#include <iostream>
#include <stdio.h>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbot/DataTrace.hpp"
#include "YouBotKDLInterface.hpp"

using namespace KDL;

int main( int argc, const char* argv[] ){
	YouBotKDLInterface *youBotArm;
    youBotArm = new YouBotKDLInterface("youbot-manipulator");
    youBotArm->doInitialize();
    SLEEP_SEC(5);
    KDL::JntArray jointpositions = JntArray(5);

    //Assigning to candle position
	 jointpositions(0) = (double) 2.96244 ;
	 jointpositions(1) = (double)1.04883 ;
	 jointpositions(2) = (double)-2.43523;
	 jointpositions(3) = (double)1.73184 ;
	 jointpositions(4) = (double)2.91062;
    
    
    youBotArm->setJointData(jointpositions);	

    SLEEP_SEC(5);
    
    
    KDL::JntArray readJointValues = JntArray(5);
    youBotArm->getJointCurrent(readJointValues);
    cout<<"Current : "<<readJointValues(0);
    
    youBotArm->getJointCurrent(readJointValues);
    cout<<"Current : "<<readJointValues(0);
    
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(0);
    
    youBotArm->getJointVelocity(readJointValues);
    cout<<"Velocity : "<<readJointValues(0);    
    
    
    return 0;

}
