#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

MatrixNd CalcOrientationEulerXYZ(double x, double y, double z) {
    return rotx(x) * roty(y) * rotz(z);
}

int main(int argc, char *argv[]) {

    // Initial values
    Model humanoid;
    // Load your model
    RigidBodyDynamics::Addons::LuaModelReadFromFile("../humanoid_model_giorgos.lua", &humanoid);

    // prepare data structures and compute CoM
    double mass;
    Vector3d com;
    int dof;

    // Getting DoF
    dof = humanoid.dof_count;
    std::cout << "DoF: " << dof << std::endl;

    // Calculating Mass and COM
    auto zero = RigidBodyDynamics::Math::Vector3dZero;
    auto q = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    auto qd = RigidBodyDynamics::Math::VectorNd::Zero(dof);

    RigidBodyDynamics::Utils::CalcCenterOfMass(humanoid, q, qd, NULL, mass, com);

    std::cout << "Mass: " << mass << std::endl;
//    std::cout << "Center of Mass: " << com << std::endl;

    // Calculate distance between feet
    auto r_foot_id = humanoid.GetBodyId("foot_right");
    auto l_foot_id = humanoid.GetBodyId("foot_left");
    auto r_thigh_id = humanoid.GetBodyId("thigh_right");
    auto l_thigh_id = humanoid.GetBodyId("thigh_left");
    auto pelv_id = humanoid.GetBodyId("pelvis");
    auto base_link_id = humanoid.GetBodyId("base_link");

    std::cout << humanoid.IsBodyId(r_foot_id) << std::endl;
    std::cout << humanoid.IsBodyId(l_foot_id) << std::endl;

    //compute kinematics
    auto l_foot = RigidBodyDynamics::CalcBodyToBaseCoordinates(humanoid, q, l_foot_id, zero);
    auto r_foot = RigidBodyDynamics::CalcBodyToBaseCoordinates(humanoid, q, r_foot_id, zero);
    auto pelv = RigidBodyDynamics::CalcBodyToBaseCoordinates(humanoid, q, pelv_id, zero);

    std::cout << "The humanoid robot has " << dof << " degrees of freedom and ways " << mass << " kilograms"
              << std::endl;
    std::cout << "Center of mass position: " << com.transpose() << std::endl;
    std::cout << "Feet distance: " << (l_foot - r_foot)[1] << std::endl;
    std::cout << "Pelvis height: " << (pelv - r_foot)[2] << std::endl;

    // Compute inverse kinematic
    InverseKinematicsConstraintSet CS;

    // Change this parameter for infeasible configurations and see what happens
    CS.lambda = 0.00001;

    VectorNd q_init(VectorNd::Zero(humanoid.dof_count));

    // Prepare data structures for Inverse kinematics
    q_init[12] = 0.2;
    q_init[18] = 0.2;

    // Hint: unsigned int pelvis = CS.AddFullConstraint(....)
    Vector3d right_foot_target_pos = Vector3d(0., -0.25, 0.);
    Vector3d left_foot_target_pos = Vector3d(0., 0.25, 0.);
    Vector3d pelvis_target_pos = Vector3d(0., 0., 0.6);

    Matrix3d right_foot_target_orientation = Matrix3dIdentity;
    Matrix3d left_foot_target_orientation = Matrix3dIdentity;


    std::cout << "Pelvis position: " << pelvis_target_pos.transpose() << std::endl;
    std::cout << "Right foot target position: " << right_foot_target_pos.transpose() << std::endl;
    std::cout << "Left foot target position: " << left_foot_target_pos.transpose() << std::endl;

    Matrix3d pelvis_target_orientation = RigidBodyDynamics::Math::Matrix3d(0, -1, 0, 1, 0, 0, 0, 0, 1);
    Matrix3d foot_target_orientation = RigidBodyDynamics::Math::Matrix3dIdentity;

    int pelvis = CS.AddFullConstraint(pelv_id, zero, pelvis_target_pos, Matrix3dIdentity);
    CS.AddOrientationConstraint(base_link_id, RigidBodyDynamics::Math::Matrix3dIdentity);
    CS.AddFullConstraint(r_foot_id, Vector3d(0.0, 0.0, -0.1), right_foot_target_pos, right_foot_target_orientation);
    CS.AddFullConstraint(l_foot_id, Vector3d(0.0, 0.0, -0.1), left_foot_target_pos, left_foot_target_orientation);

    //Prepare animation output
    std::ofstream of("animation.csv");
    std::ofstream ff("arrows.ff");

    // Run inverse kinematics and save animation
    for (unsigned int i = 0; i < 100; i++) {

        double Pelvis_height = 0.75 + 0.4 * sin(M_PI * i / 10.);
        double Pelvis_angle = sin(M_PI * i / 10.);

        //override target position and orientation
        CS.target_positions[pelvis] = Vector3d(0.0, 0.0, Pelvis_height);
        CS.target_orientations[pelvis] = CalcOrientationEulerXYZ(0.0, 0.0, Pelvis_angle);

        //calculate IK
        auto q_res = RigidBodyDynamics::Math::VectorNd(humanoid.dof_count);
        bool ret = InverseKinematics(humanoid, q_init, CS, q_res);
        if (!ret) {
            std::cout << "InverseKinematics did not find a solution" << std::endl;
        }
        RigidBodyDynamics::Utils::CalcCenterOfMass(humanoid, q_res, qd, NULL ,mass, com);
        q_init = q_res;

        //compute CoM for animation
        of << i / 10. << ", ";
        ff << i / 10. << ", ";
        for (unsigned int j = 0; j < dof; j++) {
            of << q_res[j] << ", ";
        }
        of << i << ",\n";
        ff << com[0] << ", " << com[1] << ", " << com[2] << ", 1000, 0, 0, 0, 0, 0\n";

    }

    of.close();
    return 0;
}
