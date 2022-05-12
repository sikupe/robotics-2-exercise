#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

MatrixNd CalcOrientationEulerXYZ( double x, double y, double z ) {
    return rotx(x) * roty(y) * rotz(z);
}


int main(int argc, char *argv[]) {

	//Initial values
	Model humanoid;
    //Load your model
    RigidBodyDynamics::Addons::LuaModelReadFromFile("humanoid_model.lua", &humanoid);

	//prepare data structures and compute CoM
	double mass;
    Vector3d com;
    int dof;
    ---> CODE missing
    
	
    //compute kinematics 
    ---> CODE missing

    std::cout << "The humanoid robot has " << dof << " degrees of freedom and ways " << mass << " kilograms" << std::endl;
    std::cout << "Center of mass position: " << com.transpose() << std::endl;
    std::cout << "Feet distance: " << (l_foot - r_foot)[1] << std::endl;
    std::cout << "Pelvis height: " << (pelv - r_foot)[2] << std::endl;
    
    // Compute inverse kinematic
	InverseKinematicsConstraintSet CS;
    
    // Change this parameter for infeasible configurations and see what happens
    CS.lambda = 0.00001;

    VectorNd q_init (VectorNd::Zero(humanoid.dof_count));
    
    // Prepare data structures for Inverse kinematics
    ---> CODE missing
    
    // Hint: unsigned int pelvis = CS.AddFullConstraint(....)

    //Prepare animation output
    std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");


    // Run inverse kinematics and save animation
    for (unsigned int i = 0; i < 100; i ++){
        
        double Pelvis_height = 0.75 + 0.4 * sin( M_PI * i / 10. );
        double Pelvis_angle = sin( M_PI * i / 10. );
        
        //override target position and orientation
        CS.target_positions[pelvis] = Vector3d(0.0, 0.0, Pelvis_height);
        CS.target_orientations[pelvis] = CalcOrientationEulerXYZ( 0.0, 0.0, Pelvis_angle);
        
        //calculate IK
        bool ret = InverseKinematics(humanoid, q_init, CS, q_res);
        if (!ret) {
            std::cout << "InverseKinematics did not find a solution" << std::endl;
        }
        
        //compute CoM for animation
        of << i / 10. << ", ";
        ff << i / 10. << ", ";
        for (unsigned int j = 0; j < dof; j++){
            of << q_res[j] << ", ";
        }
        of << i << ",\n";
        ff << com[0] << ", " << com[1] << ", " << com[2] << ", 1000, 0, 0, 0, 0, 0\n";
        
    }
    
	of.close();
	return 0;
}
