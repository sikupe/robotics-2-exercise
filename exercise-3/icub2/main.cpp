#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// Define a few types to make it easier
typedef VectorNd (*rhsFuncPtr) (const VectorNd&, const VectorNd&);
typedef VectorNd (*integratorFuncPtr) (const VectorNd&, const VectorNd&, const double, rhsFuncPtr);

MatrixNd CalcOrientationEulerXYZ( double x, double y, double z ) {
    return rotx(x) * roty(y) * rotz(z);
}

Model humanoid;

VectorNd rk4_integrator (const VectorNd &x, const VectorNd &u, const double h, rhsFuncPtr rhs) {

--- copy and paste your rk4 here

}

VectorNd rhs_constraint_set_feet (const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());
    
    //update robot states 
    VectorNd q = VectorNd::Zero(humanoid.dof_count);
    VectorNd qdot = VectorNd::Zero(humanoid.dof_count);
    VectorNd qddot = VectorNd::Zero(humanoid.dof_count);
    VectorNd tau = VectorNd::Zero(humanoid.dof_count);
    
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        q[i] = x[i];
        qdot[i] = x[i + humanoid.dof_count];
    }
    for (unsigned int i = 0; i < humanoid.dof_count -3; i++) {
        tau[i+3] = u[i];
    }
            
    
    int left_foot = humanoid.GetBodyId("l_sole");
    int right_foot = humanoid.GetBodyId("r_sole");
    ConstraintSet fix_feet;

 --- Add your constraints here
    
    fix_feet.Bind(humanoid);
    //compute ForwardDynamics
    ForwardDynamicsConstraintsDirect(humanoid,q,qdot,tau,fix_feet,qddot);
    
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        res[i] = qdot[i];
        res[i + humanoid.dof_count] = qddot[i];
    }     
    
    return res;
}


int main(int argc, char *argv[]) {
    //load model
	if (! RigidBodyDynamics::Addons::LuaModelReadFromFile( "../models/iCubHeidelberg01.lua" , &humanoid, false)) {
		std::cerr << "Error loading model - aborting" << std::endl;
		abort();
	}
	//get DoF
    int dof = humanoid.dof_count;
    int actuated_dof = dof - 3;
	
    //compute feet distance and pelvis hight
    VectorNd q (VectorNd::Zero(dof));
    Vector3d r_foot = ___
    Vector3d l_foot = ___
    Vector3d pelv = ___
    
    double feet_dist = (l_foot - r_foot)[1];
    double pelvis_hight = (pelv - r_foot)[2];	

	InverseKinematicsConstraintSet CS;
    
    // Change this parameter for infeasible configurations
    CS.lambda = 0.01;
    VectorNd q_init (VectorNd::Zero(dof));
    VectorNd q_res (VectorNd::Zero(dof));
    
    // Set Bent knees in the same direction
    q_init[3] = 0.45;
    q_init[4] = -0.7;
    q_init[5] = 0.4;
     
    q_init[6] = -0.48;
    q_init[7] = -0.7;
    q_init[8] = -0.4;
    
    // Add target positions
    Vector3d R_foot_pos = ___
    Vector3d L_foot_pos = ___
    Vector3d Pelvis_pos = ___
    
    //model feet are turned
    Matrix3d R_foot_ort = CalcOrientationEulerXYZ(0,0,M_PI);
    Matrix3d L_foot_ort = CalcOrientationEulerXYZ(0,0,M_PI);
    
    //add constrains
    CS.AddFullConstraint(humanoid.GetBodyId("r_sole") ....... )
    unsigned int pelvis = CS.AddPointConstraint(humanoid.GetBodyId("chest"), Vector3dZero, Pelvis_pos);
    
    //compute IK for beginning and end pose
    bool ret = InverseKinematics(humanoid, q_init, CS, q_res);
    if (!ret) {
        std::cout << "InverseKinematics did not find a solution" << std::endl;
    }
    
    q_init = q_res;

    //Update target position and run IK again
    CS.target_positions[pelvis] = ___
    
    ret = InverseKinematics(humanoid, q_init, CS, q_res);
    if (!ret) {
        std::cout << "InverseKinematics did not find a solution" << std::endl;
    }    
    
    //save animation
    std::ofstream an("animation.csv");
    an << 0. << ", ";
    for (int j = 0; j < dof; j++){
        an << q_res[j] << ", ";
    }
    an << "\n";
    an << 0.4 << ", ";
    for (int j = 0; j < dof; j++){
        an << q_init[j] << ", ";
    }
    an << "\n";
    
    //compute ForwardDynamics with constraints
    // define start and end position
    VectorNd q_start = q_res;
    VectorNd q_end = q_init;
    
	// States: x = [ q q_dot ]
    VectorNd x (VectorNd::Zero(2 * dof));
    
    // Control vector u = [ tau ]
    VectorNd u = VectorNd::Zero(actuated_dof);
    
	double t = 0.;
	double tf = 0.4;
	double h = 0.00001;
    
    //close enough initial guess
   // u[0] = __;
   // u[1] = __;
   // u[2] = __;
   //  u[3] = __;
   // u[4] = __;
   // u[5] = __;

	std::ofstream of("computed_motion.csv");
    
    //iterations for single shooting, set to 0 for only one forward simulation
    int n = 0;
    
    for (unsigned int i = 0; i <= n; i++){
        //reset data to 
        t = 0;
        x = VectorNd::Zero(2 * dof);
        for (unsigned int j = 0; j < dof; j++){
            x[j] = q_start[j];
            x[j + dof] = 0;
        }
        
        //perform single shot
        while (t <= tf) { 
            x = x + h * rk4_integrator (x, u, h, rhs_constraint_set_feet);
            t = t + h;
            
            //save result after last iteration
            if (i == n){
                of << t << ", ";
                for (unsigned int j = 0; j < dof; j++){
                    of << x[j] << ", ";
                }
                of << "\n";
            }
        }
        
        //compute difference of resulting position to target
        VectorNd dif (q_end.size());

        --- implement update for u
        
        //print difference
        cout << "Difference to target: " << dif.norm()  << endl;
        
        //update torques according to difference, Newton step, only works if we are (really close to the solution)
        for (unsigned int j = 0; j < actuated_dof; j++){
            u[j] = ....
        }
    }
    cout << "Torques: " << u.transpose() << endl;
    an.close();
	of.close();
	return 0;
}
