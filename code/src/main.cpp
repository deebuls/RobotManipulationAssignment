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

double joint[5];
double lastJoint[5];
double jointDelta[5];
double jointMax[] = {5.840139, 2.617989, -0.0157081, 3.42919, 5.641589};
double jointMin[] = {0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062};
std::vector<double> JointAngles;
std::vector<double> JointVelocities;
				


char getch(void)
{
	char ch;
	struct termios oldt;
	struct termios newt;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt; 
	newt.c_lflag &= ~(ICANON | ECHO); 
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
	ch = getchar(); 
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
	return ch;
}


int main( int argc, const char* argv[] ){
	
	static const int numberOfArmJoints = 5;
	char keyboardInput = '0';
	int readValue = 0;
	JointAngles.resize(5);	
	JointVelocities.resize(5);

	for(int i = 0; i < numberOfArmJoints; i++)
	{
		jointDelta[i] = (jointMax[i] - jointMin[i]) * 0.02;
	}	
	


	youbot::YouBotManipulator *youBotArm;
	youBotArm = new youbot::YouBotManipulator("youbot-manipulator");
	youBotArm->doJointCommutation();
	youBotArm->calibrateManipulator();
	std::vector<youbot::JointSensedAngle> jointangles;
	jointangles.resize(numberOfArmJoints);
	std::vector<youbot::JointAngleSetpoint> jointSetAngle;
	std::vector<youbot::JointVelocitySetpoint> jointSetVel;
	jointSetAngle.resize(5);
	jointSetVel.resize(5);
	SLEEP_SEC(5);
	// Home position
	jointSetAngle[0].angle =  2.96244 * radian;
	jointSetAngle[1].angle = 1.04883 * radian;
	jointSetAngle[2].angle =-2.43523* radian;
	jointSetAngle[3].angle = 1.73184  * radian;
	jointSetAngle[4].angle =  2.91062 * radian;
//	youBotArm->setJointData(jointSetAngle);
	SLEEP_SEC(5);
	youbot::JointSensedAngle angle;
	youbot::JointSensedVelocity vel;	

	while (true) {	
		keyboardInput = '0';
		readValue = 0;

		std::cout << "Please provide an arm joint or the gripper to operate on.\n\tType number between 1 and 5, to control joints 1 - 5.\n\tOr hit 9 to quit:" << std::endl;
		std::cin >> readValue;
		if((readValue >= 1)&&(readValue <= 5))
		{
			cout << "Selected arm joint " << readValue << endl;
		}
		if(readValue == 9)
		{
			cout << "Exiting..." << endl;
			break;
		}
		cout << "Hit w/s to increase/decrease joint angle." << endl;
		cout << "If you want to stop manipulating this joint, hit 'm' to switch to new joint or exit." << endl;
		cout << "Hit o/l to open/close gripper." << endl;
		cout << "To quit, hit Control + C  or 'm'." << endl;

		keyboardInput = '0';
	
		while((keyboardInput != 'm'))
		{	
			
			
			keyboardInput = getch();
			
			//std::vector<youbot::JointSensedVelocity> jointSensedVel;
			//jointSensedVel.resize(5);
			//youBotArm->getJointData(jointSensedVel);
			
			for(std::size_t i=0;i<5;i++){
				youBotArm->getArmJoint(i+1).getData(angle);
				JointAngles[i] = (double)angle.angle.value();
				youBotArm->getArmJoint(i+1).getData(vel);
				cout <<" Angles "<<i<<JointAngles[i]<<endl;

			}
			
			//////////////////////////////////////////////////////////////////////////////
			///Sensing the velocities of the Joints
			///and storing it in JointVelocities
			//////////////////////////////////////////////////////////////////////////////
			for(std::size_t i=0;i<5;i++){
				youBotArm->getArmJoint(i+1).getData(vel);
				JointVelocities[i] = (double)vel.angularVelocity.value();
				cout <<" Vel "<<i<<(double)vel.angularVelocity.value()<<endl;
			}
			
			
			if (keyboardInput == 'w'){
				
				if((readValue >= 1)&&(readValue <= 3))
				{	
					//////////////////////////////////////////////////////////////
					///Updating angles of Joints 1 to 3
					//////////////////////////////////////////////////////////////		
					JointAngles[readValue - 1] += jointDelta[readValue - 1];
				} else if ((readValue >= 4)&&(readValue <= 5))
				{
					//////////////////////////////////////////////////////////////
					///Updating velocity of Joints 4 to 5
					//////////////////////////////////////////////////////////////
					JointVelocities[readValue - 1] += jointDelta[readValue - 1];					
				}
				
			}
			if (keyboardInput == 's'){
						
				if((readValue >= 1)&&(readValue <= 3))
				{
					//////////////////////////////////////////////////////////////
					///Updating angles of Joints 1 to 3	
					//////////////////////////////////////////////////////////////		
					JointAngles[readValue - 1] -= jointDelta[readValue - 1];
				} else if ((readValue >= 4)&&(readValue <= 5))
				{
					//////////////////////////////////////////////////////////////
					///Updating velocity of Joints 4 to 5
					//////////////////////////////////////////////////////////////
					JointVelocities[readValue - 1] -= jointDelta[readValue - 1];					
				}
				
			}			
			if(keyboardInput == 'o')
			{
				youBotArm->getArmGripper().open();
			}
			if(keyboardInput == 'l')
			{
				youBotArm->getArmGripper().close();
			}

			if(keyboardInput != '0')
			{
				
				for(std::size_t i=0;i<5;i++){
					jointSetAngle[i].angle = JointAngles[i]*radian;
					jointSetVel[i].angularVelocity = JointVelocities[i]*radian_per_second;
				}
				
				for(std::size_t i=0;i<5;i++){

					//////////////////////////////////////////////////////////////
					// Setting the data as per the Joints
					//////////////////////////////////////////////////////////////
					if((readValue >= 1)&&(readValue <= 3))
					{		
						youBotArm->getArmJoint(i+1).setData(jointSetAngle[i]);
					} else if ((readValue >= 4)&&(readValue <= 5))
					{
						youBotArm->getArmJoint(i+1).setData(jointSetVel[i]);
					}
				}


				
			}
		}
	}
	return 0;

}
