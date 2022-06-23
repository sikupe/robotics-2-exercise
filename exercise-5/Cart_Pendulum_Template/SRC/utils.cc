void updateState (const double *sd, const double *u, const double *p_in, const ConstraintSetName cs_name) {
    for (unsigned int i = 0; i < PosNameLast; i++) {
        q[i] = sd[i];
        qdot[i] = sd[i + PosNameLast];
    }

    for (unsigned int i = 0; i < ControlNameLast; i++) {
        tau[i + nDof - nActuatedDof] = u[i];
    }

    for (unsigned int i = 0; i < ParamNameLast; i++) {
        p[i] = p_in[i];
    }
}

bool WalkerModel::loadFromFile (const char* filename, bool verbose) {
    if (!RigidBodyDynamics::Addons::LuaModelReadFromFile (filename, &model, verbose)) {
        cerr << "Error loading LuaModel: " << filename << endl;
        abort();
    }

    nDof = model.dof_count;
    nActuatedDof = nDof - 3;

    assert (nActuatedDof >= 1 && nActuatedDof <= nDof);

    if (nDof != PosNameLast) {
        cerr << "Error: Number of model degrees of freedom (" << nDof << ") does not match number of positional variables (" << PosNameLast << ")!" << endl;
        abort();
    }

    if (nActuatedDof != ControlNameLast) {
        cerr << "Error: #nActuatedDof does not match the number of controls!" << endl;
        abort();
    }

    q = VectorNd::Zero (nDof);
    qdot = VectorNd::Zero (nDof);
    qddot = VectorNd::Zero (nDof);
    tau = VectorNd::Zero (nDof);
    springDamperForces = VectorNd::Zero (nDof);

    return true;
}
