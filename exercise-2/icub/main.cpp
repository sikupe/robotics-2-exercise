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
typedef VectorNd (*rhsFuncPtr)(const VectorNd &, const VectorNd &);

typedef VectorNd (*integratorFuncPtr)(const VectorNd &, const VectorNd &, const double, rhsFuncPtr);

Model humanoid;


VectorNd rk4_integrator(const VectorNd &x, const VectorNd &u, const double h, rhsFuncPtr rhs) {
    auto k_1 = rhs(x, u);
    auto k_2 = rhs(x + 0.5 * h * k_1, u);
    auto k_3 = rhs(x + 0.5 * h * k_2, u);
    auto k_4 = rhs(x + h * k_3, u);
    auto rk4_xd = (1. / 6.) * (k_1 + 2. * k_2 + 2. * k_3 + k_4);

    return rk4_xd;
}


VectorNd rhs(const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2 * humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());

    //allocate robot states update kinematics
    VectorNd q = VectorNd(humanoid.dof_count);
    VectorNd qd = VectorNd(humanoid.dof_count);
    VectorNd tau = VectorNd(humanoid.dof_count);

    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        q[i] = x[i];
        qd[i] = x[i + humanoid.dof_count];
        tau[i] = u[i];
    }

    //compute ForwardDynamics
    VectorNd qdd = VectorNd(humanoid.dof_count);

    ForwardDynamics(humanoid, q, qd, tau, qdd);

    //copy to res vector
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        res[i] = qd[i];
        res[i + humanoid.dof_count] = qdd[i];
    }

    return res;
}

VectorNd rhs_constraint_set(const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2 * humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());

    //allocate robot states update kinematics
    VectorNd q = VectorNd(humanoid.dof_count);
    VectorNd qd = VectorNd(humanoid.dof_count);
    for (int i = 0; i < humanoid.dof_count; ++i) {
        q[i] = x[i];
        qd[i] = x[i + humanoid.dof_count];
    }
    VectorNd tau = VectorNd(u.size());
    for (int i = 0; i < u.size(); ++i) {
        tau[i] = u[i];
    }

    VectorNd qdd = VectorNd(humanoid.dof_count);

    //create constraints here
    int body_Id = humanoid.GetBodyId("root_link");
    ConstraintSet fixed_root_link;

    // Add constraints first, then bind it to the model
//    fixed_root_link.AddContactConstraint();
    fixed_root_link.Bind(humanoid);

    //compute ForwardDynamics
    ForwardDynamicsConstraintsDirect(humanoid, q, qd, tau, fixed_root_link, qdd);

    //copy to res vector
    for (int i = 0; i < humanoid.dof_count; ++i) {

    }
    return res;
}

//VectorNd rhs_contact_force (const VectorNd &x, const VectorNd &u) {
//
//    //set up external forces use RBDL API
//    std::vector< SpatialVector > fext;
//    ____
//
//    int body_Id = humanoid.GetBodyId("root_link");
//
//    //compute forces and torque
//    Vector3d force (___, ___, ___);
//    Vector3d torque = VectorCrossMatrix(___)*force;
//
//    fext[body_Id][0] = torque[0];
//    fext[body_Id][1] = torque[1];
//    fext[body_Id][2] = torque[2];
//
//    fext[body_Id][3] = force[0];
//    fext[body_Id][4] = force[1];
//    fext[body_Id][5] = force[2];
//
//
//    //compute ForwardDynamics
//    ForwardDynamics(___, &fext);
//
//
//    return res;
//}

VectorNd rhs_contact_force_damping(const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2 * humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());


    return res;
}

int main(int argc, char *argv[]) {

    if (!RigidBodyDynamics::Addons::LuaModelReadFromFile("./models/iCubHeidelberg01.lua", &humanoid, false)) {
        std::cerr << "Error loading model - aborting" << std::endl;
        abort();
    }

//     humanoid.gravity = Vector3dZero;

    // States: x = [ q q_dot ]
    auto q = VectorNd::Zero(humanoid.dof_count);
    auto qdot = VectorNd::Zero(humanoid.dof_count);
    auto x = VectorNd(2 * humanoid.dof_count);
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        x[i] = q[i];
        x[i + humanoid.dof_count] = qdot[i];
    }

    // State derivatives xd = [ qdot qddot ]
    auto xdot = VectorNd(2 * humanoid.dof_count);


    // Control vector u = [ tau ]
    auto tau = VectorNd::Zero(humanoid.dof_count);
    auto u = VectorNd(humanoid.dof_count);
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        u[i] = tau[i];
    }

    double t = 0.;
    double tf = 10;
    double h = 0.001;

    std::ofstream of("animation.csv");

    while (t <= tf) {
        //add joint torques in u if needed
        of << t;
        for (unsigned int i = 0; i < humanoid.dof_count; i++) {
            of << ", " << x[i];
        }
        of << "\n";
        x = x + h * rk4_integrator(x, u, h, rhs);
        t = t + h;
    }

    of.close();
    return 0;
}
