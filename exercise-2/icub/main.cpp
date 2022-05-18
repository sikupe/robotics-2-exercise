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

Model humanoid;

VectorNd rk4_integrator (const VectorNd &x, const VectorNd &u, const double h, rhsFuncPtr rhs) {
    return 0.0;
}


VectorNd rhs (const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());
    
    //allocate robot states update kinematics
    VectorNd q = ....
    
    //compute ForwardDynamics
    ForwardDynamics(___);
    
    //copy to res vector
    
    return res;
}

VectorNd rhs_constraint_set (const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());

    //allocate robot states update kinematics
    VectorNd q = ....
    
    //create constraints here
    int body_Id = humanoid.GetBodyId("root_link");
    ConstraintSet fixed_root_link;

    // Add constraints first, then bind it to the model
    
    fixed_root_link.Bind(humanoid);

    //compute ForwardDynamics
    ForwardDynamicsConstraintsDirect(___);
    
    //copy to res vector
    return res;
}

VectorNd rhs_contact_force (const VectorNd &x, const VectorNd &u) {
    
    //set up external forces use RBDL API
    std::vector< SpatialVector > fext;
    ____
    
    int body_Id = humanoid.GetBodyId("root_link");

    //compute forces and torque
    Vector3d force (___, ___, ___);
    Vector3d torque = VectorCrossMatrix(___)*force;

    fext[body_Id][0] = torque[0];
    fext[body_Id][1] = torque[1];
    fext[body_Id][2] = torque[2];
    
    fext[body_Id][3] = force[0];
    fext[body_Id][4] = force[1];
    fext[body_Id][5] = force[2];
    
    
    //compute ForwardDynamics
    ForwardDynamics(___, &fext);
    

    return res;
}

VectorNd rhs_contact_force_damping (const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());
    

    
    return res;
}


int main(int argc, char *argv[]) {

	if (! RigidBodyDynamics::Addons::LuaModelReadFromFile( "../models/iCubHeidelberg01.lua" , &humanoid, false)) {
		std::cerr << "Error loading model - aborting" << std::endl;
		abort();
	}
	
    // humanoid.gravity = Vector3dZero;
	
	// States: x = [ q q_dot ]
    
    // State derivatives xd = [ qdot qddot ]
    
    // Control vector u = [ tau ]
    
	double t = 0.;
	double tf = 10;
	double h = 0.001;

	std::ofstream of("animation.csv");

	while (t <= tf) {
        //add joint torques in u if needed
		of << t << ", ";
        for (unsigned int i = 0; i < humanoid.dof_count; i++){
            of << x[i] << ", ";
        }
        of << "\n";
		x = x + h * rk4_integrator (x, u, h, rhs_constraint_set);
		t = t + h;
	}

	of.close();
	return 0;
}
