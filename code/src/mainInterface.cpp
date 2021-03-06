
#include <iostream>
#include <stdio.h>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbot/DataTrace.hpp"
#include "YouBotKDLInterface.hpp"

using namespace KDL;
using namespace youbot;
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
    
    
    youBotArm->setJointPosition(jointpositions);	

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
    
    KDL::Frame endfactorframe;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe;

    SLEEP_SEC(5);
    int i; 
	jointpositions(4) = jointpositions(4) + M_PI/2;
   
    cout<<"------rotation +ve ---------"<<std::endl;
    cout<<"Rotation Joint 4 to :"<<jointpositions(4)<<std::endl;

    youBotArm->setJointPosition(jointpositions);	
    SLEEP_SEC(5);
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(4)<<std::endl;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe<<std::endl;
    cout<<"Please enter for next Test"<<std::endl;
    cin>>i;

	jointpositions(4) = jointpositions(4) - M_PI/4;
   
    cout<<"------rotation -ve ---------"<<std::endl;
    cout<<"Rotation Joint 4 to :"<<jointpositions(4)<<std::endl;

    youBotArm->setJointPosition(jointpositions);	
    SLEEP_SEC(5);
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(4)<<std::endl;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe<<std::endl;
    cout<<"Please enter for next Test"<<std::endl;
    cin>>i;

	jointpositions(2) = jointpositions(2) + M_PI/2;
   
    cout<<"------rotation +ve ---------"<<std::endl;
    cout<<"Rotation Joint 2 to :"<<jointpositions(2)<<std::endl;

    youBotArm->setJointPosition(jointpositions);	
    SLEEP_SEC(5);
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(2)<<std::endl;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe<<std::endl;
    cout<<"Please enter for next Test"<<std::endl;
    cin>>i;

	jointpositions(2) = jointpositions(2) - M_PI/4;
   
    cout<<"------rotation -ve ---------"<<std::endl;
    cout<<"Rotation Joint 2 to :"<<jointpositions(2)<<std::endl;

    youBotArm->setJointPosition(jointpositions);	
    SLEEP_SEC(5);
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(2)<<std::endl;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe<<std::endl;
    cout<<"Please enter for next Test"<<std::endl;
    cin>>i;

	jointpositions(3) = jointpositions(3) + M_PI/2;
   
    cout<<"------rotation +ve ---------"<<std::endl;
    cout<<"Rotation Joint 3 to :"<<jointpositions(3)<<std::endl;

    youBotArm->setJointPosition(jointpositions);	
    SLEEP_SEC(5);
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(3)<<std::endl;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe<<std::endl;
    cout<<"Please enter for next Test"<<std::endl;
    cin>>i;

	jointpositions(3) = jointpositions(3) - M_PI/4;
   
    cout<<"------rotation -ve ---------"<<std::endl;
    cout<<"Rotation Joint 3 to :"<<jointpositions(3)<<std::endl;

    youBotArm->setJointPosition(jointpositions);	
    SLEEP_SEC(5);
    youBotArm->getJointPosition(readJointValues);
    cout<<"Position : "<<readJointValues(3)<<std::endl;
    youBotArm->getEndFactorPose(jointpositions, endfactorframe);
    cout<<"Frame end factor"<<endfactorframe<<std::endl;
    cout<<"Please enter for next Test"<<std::endl;
    cin>>i;



    return 0;

}
