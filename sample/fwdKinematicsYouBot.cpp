
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
 
using namespace KDL;
 
 
int main( int argc, char** argv )
{
    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
    KDL::Frame arm_base_frame;

    KDL::Joint joint_1;
    KDL::Frame frame_1;

    KDL::Joint joint_2;
    KDL::Frame frame_2;

    KDL::Joint joint_3;
    KDL::Frame frame_3;

    KDL::Joint joint_4;
    KDL::Frame frame_4;

    KDL::Joint joint_5;
    KDL::Frame frame_5;

    KDL::Frame end_factor;
    double offset[5] = {
        (-169 * M_PI/180),
        (-65 * M_PI/180),
        (151 * M_PI/180),
        (-102.5 * M_PI/180),
        (-165 * M_PI/180)
    };
    
    //Base frame 
    arm_base_frame = KDL::Frame::DH(0.0, M_PI, 0.147, 0.0);

    //Frame 1 and Joint 1
    joint_1 = KDL::Joint(KDL::Joint::RotZ);
    frame_1 = KDL::Frame::DH(0.033, +M_PI_2, 0.000, offset[0]);

    //Frame 2 and Joint 2
    joint_2 = KDL::Joint( KDL::Joint::RotZ);
    frame_2 = KDL::Frame::DH(0.155, 0, 0.000, offset[1]-M_PI_2);

    //Frame 3 and Joint 3
    joint_3 = KDL::Joint(KDL::Joint::RotZ);
    frame_3 =  KDL::Frame::DH(0.135, 0, 0.000, offset[2]+ 0);

    //Frame 4 and Joint 4
    joint_4 = KDL::Joint(KDL::Joint::RotZ);
    frame_4 =  KDL::Frame::DH(0.0, 0, 0.000,offset[3]+ M_PI_2 );

    //Frame 5 and Joint 5
    joint_5 = KDL::Joint(KDL::Joint::RotZ);
    frame_5 =  KDL::Frame::DH(0.0, -M_PI_2, 0.000, offset[4]+ 0);


    //Frame 6
    end_factor = KDL::Frame::DH(0.0, +M_PI, -0.218,  0);

    //Adding the chain
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), arm_base_frame));
    chain.addSegment(KDL::Segment(joint_1,frame_1));
    chain.addSegment(KDL::Segment(joint_2,frame_2));
    chain.addSegment(KDL::Segment(joint_3,frame_3));
    chain.addSegment(KDL::Segment(joint_4,frame_4));
    chain.addSegment(KDL::Segment(joint_5,frame_5));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), end_factor));

 
    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
 
    // Assign some values to the joint positions
    //Assigning to candle position
    
	 jointpositions(0) = -offset[0] ;
	 jointpositions(1) = -offset[1];
	 jointpositions(2) = -offset[2];
	 jointpositions(3) = - offset[3];
	 jointpositions(4) = -offset[4];
    /*
	 jointpositions(0) = (double)0 ;
	 jointpositions(1) = (double)0;
	 jointpositions(2) = (double)0;
	 jointpositions(3) = (double)0;
	 jointpositions(4) = (double)0;
   */ 
    // Create the frame that will contain the resul
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    for(int i =0 ; i<8 ; i++)
    {
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos, i);
    if(kinematics_status>=0){
        std::cout << cartpos.M<<std::endl;
        std::cout << cartpos.p <<std::endl;
        printf("Joint %i \n", i);
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
    }
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos.M<<std::endl;
        std::cout << cartpos.p <<std::endl;
        printf("End Joint  \n" );
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

}
