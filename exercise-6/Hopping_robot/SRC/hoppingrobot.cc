/*
 *  hoppingrobot.cc
 *  (c) Martin Felis <martin.felis@iwr.uni-heidelberg.de>, 2013
 */

#include <cmath>

#include "def_usrmod.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

#include <iostream>
#include <fstream>

const int NMOS = 3;  /* Number of phases (Model Stages) */
const int NP   = 1;  /* Number of parameters */
const int NRC  = 4;  /* Number of coupled constraints */
const int NRCE = 4;  /* Number of coupled equality constraints */

const int NXD  = 4;  /* Number of differential states */
const int NXA  = 0;  /* Number of algebraic states */
const int NU   = 1;  /* Number of controls */
const int NPR  = 0;  /* Number of local parameters */

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Model *model = NULL;
std::string model_file="hoppingrobot.lua";
std::string result_file="RES/result.csv";

unsigned int contact_body_id;
Vector3d contact_point_local;
ConstraintSet constraint_set;

VectorNd Q, QDot, QDDot, Tau;

unsigned int nDof;
unsigned int nActuatedDof;

void update_generalized_variables (double *xd, double *u, VectorNd &Q, VectorNd &QDot, VectorNd &Tau) {
	for (unsigned int i = 0; i < nDof; i++) {
		Q[i] = xd[i];
		QDot[i] = xd[i + nDof];
	}

	Tau[0] = 0.;
	Tau[1] = 0.;

	if (u != NULL)
		Tau[1] = u[0];
}

/** \brief Objective function (Lagrangian type) */
static void lfcn(double *t, double *xd, double *xa, double *u,
  double *p, double *lval, double *rwh, long *iwh, InfoPtr *info)
{
	*lval = u[0] * u[0];
}


/** \brief Objective function (Mayer type) */
static void mfcn(double *ts, double *xd, double *xa, double *p, double *pr,
double *mval,  long *dpnd, InfoPtr *info) {
  if (*dpnd) {
    *dpnd = MFCN_DPND(0, *xd, 0, 0, 0);
    return;
  }

	update_generalized_variables (xd, NULL, Q, QDot, Tau);

    VectorNd QDotPlus (VectorNd::Zero(nDof));

    // Compute Contact Impulses with RBDL
	// hint: Define vector QDotPlus (all zeros) first;
    ComputeConstraintImpulsesDirect(*model, Q, QDot, constraint_set, QDotPlus);


	double impulse_x = constraint_set.impulse[0];

	*mval = impulse_x * impulse_x;
}

/** \brief Right hand side of the differential equation */
static void ffcn_flight(double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
	update_generalized_variables (xd, u, Q, QDot, Tau);

	// Compute Forward Dynamics with RBDL;
    ForwardDynamics(*model, Q, QDot, Tau, QDDot);

    for (unsigned int i = 0; i < nDof; i++) {
        rhs[i]        = QDot[i];
        rhs[i + nDof] = QDDot[i];
    }
}

static void ffcn_touchdown(double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
	update_generalized_variables (xd, u, Q, QDot, Tau);

    // Compute Contact Impulses with RBDL
	// hint: Define vector QDotPlus (all zeros) first;
    VectorNd QDotPlus (VectorNd::Zero(nDof));

    ComputeConstraintImpulsesDirect(*model, Q, QDot, constraint_set, QDotPlus);

	for (unsigned int i = 0; i < nDof; i++) {
		rhs[i] = Q[i];
		rhs[i + nDof] = QDotPlus[i];
	}

}

static void ffcn_contact(double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
	update_generalized_variables (xd, u, Q, QDot, Tau);

	double spring_force = p[0] * (0. - Q[1]);
	
	// Define Tau as the sum of control and spring force
	Tau[1] = spring_force + Tau[1];

    // Compute Forward Dynamics with contacts using RBDL;
    ForwardDynamicsConstraintsDirect(*model, Q, QDot, Tau, constraint_set, QDDot);

	for (unsigned int i = 0; i < nDof; i++) {
		rhs[i] = Q[i];
		rhs[i + nDof] = QDot[i];
	}
}


static void rdfcn_flight_i(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);

	// Compute contact point in base coordinates
    auto contact_point = CalcBodyToBaseCoordinates(*model, Q, contact_body_id, contact_point_local);

	res[0] = contact_point[2];
}

static void rdfcn_touchdown_s(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);

	// Compute contact point in base coordinates and velocity of contact point
    auto contact_point = CalcBodyToBaseCoordinates(*model, Q, contact_body_id, contact_point_local);
    auto contact_point_velocity = CalcPointVelocity(*model, Q, QDot, contact_body_id, contact_point_local);

	res[0] = contact_point[2];
	res[1] = -contact_point_velocity[2];
}

static void rdfcn_contact_i(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, *u, 0, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);

	// Compute Forward Dynamics with contacts using RBDL;
	// Determine force

    ForwardDynamicsConstraintsDirect(*model, Q, QDot, Tau, constraint_set, QDDot);

	res[0] = constraint_set.force[0]; // force greater zero
}

/** \brief Constraints at the start point */
static void rdfcn_contact_e (double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, *u, 0, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);

	// Compute Forward Dynamics with contacts using RBDL;
	// Determine force
    ForwardDynamicsConstraintsDirect(*model, Q, QDot, Tau, constraint_set, QDDot);
    auto contact_point_velocity = CalcPointVelocity(*model, Q, QDot, contact_body_id, contact_point_local);

	res[0] = constraint_set.force[0]; // force greater zero
	res[1] = contact_point_velocity[2] - 5; // velocity greater five
}

static void rcfcn_periodic_s(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);


	res[0] = Q[0];
	res[1] = Q[1];
	res[2] = QDot[0];
	res[3] = QDot[1];
}

static void rcfcn_periodic_e(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);


    res[0] = -Q[0];
    res[1] = -Q[1];
    res[2] = -QDot[0];
    res[3] = -QDot[1];
}

vector<double> t_values;
vector<VectorNd> sd_values;
vector<VectorNd> u_values;

static void data_out( double *t, double *sd, double *sa, double *u, double *p, double *rwh, long *iwh, InfoPtr *info ) {
	if (*t == 0.) {
		ofstream meshup_csv_stream;

		meshup_csv_stream.open (result_file.c_str(), ios_base::trunc);

		if (!meshup_csv_stream) {
			cerr << "Error opening file " << result_file << endl;
			abort();
		}

		meshup_csv_stream << "COLUMNS:" << endl;
		meshup_csv_stream << "Time, Body:T:z, Leg:T:z" << endl;
		meshup_csv_stream << "DATA:" << endl;

		if (t_values.size() > 0) {
			for (unsigned int i = 0; i < t_values.size(); i ++) {
				meshup_csv_stream << t_values[i] << ", ";
				for (unsigned int j = 0; j < sd_values[i].size(); j++) {
					meshup_csv_stream << sd_values[i][j];
					if (j < sd_values[i].size() -1 )
						meshup_csv_stream << ", ";
				}

				// output of the constraint force
				update_generalized_variables (&(sd_values[i][0]), &(u_values[i][0]), Q, QDot, Tau);
				ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau, constraint_set, QDDot);
				meshup_csv_stream << ", " << constraint_set.force[0];

				meshup_csv_stream << endl;
			}
		}

		t_values.clear();
		sd_values.clear();
		u_values.clear();

		meshup_csv_stream.close();
	}

	t_values.push_back (*t);

	VectorNd sd_vec (NXD);
	for (unsigned i = 0; i < NXD; i++)
		sd_vec[i] = sd[i];
	sd_values.push_back (sd_vec);

	VectorNd u_vec (NU);
	for (unsigned i = 0; i < NU; i++)
		u_vec[i] = u[i];
	u_values.push_back (u_vec);
}

static void mout
(
  long   *imos,      ///< index of model stage (I)
  long   *imsn,      ///< index of m.s. node on current model stage (I)
  double *ts,        ///< time at m.s. node (I)
  double *te,        ///< time at end of m.s. interval (I)
  double *sd,        ///< differential states at m.s. node (I)
  double *sa,        ///< algebraic states at m.s. node (I)
  double *u,         ///< controls at m.s. node (I)
  double *udot,      ///< control slopes at m.s. node (I)
  double *ue,        ///< controls at end of m.s. interval (I)
  double *uedot,     ///< control slopes at end of m.s. interval (I)
  double *p,         ///< global model parameters (I)
  double *pr,        ///< local i.p.c. parameters (I)
  double *ccxd,
  double *mul_ccxd,  ///< multipliers of continuity conditions (I)
#if defined(PRSQP) || defined(EXTPRSQP)
  double *ares,
  double *mul_ares,
#endif
  double *rd,
  double *mul_rd,    ///< multipliers of decoupled i.p.c. (I)
  double *rc,
  double *mul_rc,    ///< multipliers of coupled i.p.c. (I)
  double *obj,
  double *rwh,       ///< real work array (I)
  long   *iwh        ///< integer work array (I)
) {
	InfoPtr info(0, *imos, *imsn);
	data_out( ts, sd, sa, u, p, rwh, iwh, &info);
}

/** \brief Entry point for the muscod application */
extern "C" void def_model(void);
void def_model(void)
{
	model = new Model();
	
	printf("Load model %s!\n",model_file.c_str());

	if (!Addons::LuaModelReadFromFile (model_file.c_str(), model, true)) {
		cerr << "Error loading RBDL model '" << model_file << "'!" << endl;
		abort();
	}
	
	printf("Model loaded!\n");

	assert (model->dof_count == 2);

	nDof = model->dof_count;
	nActuatedDof = 1;

	Q = VectorNd::Zero(model->dof_count);
	QDot = VectorNd::Zero(model->dof_count);
	QDDot = VectorNd::Zero(model->dof_count);
	Tau = VectorNd::Zero(model->dof_count);

	// Initialization of the constraint set
	unsigned int Leg_id = model->GetBodyId("Leg");
	unsigned int LegEnd_id = model->GetBodyId("LegEnd");

	Vector3d Leg_origin = CalcBodyToBaseCoordinates (*model, Q, Leg_id, Vector3d (0., 0., 0.));
	Vector3d LegEnd_origin = CalcBodyToBaseCoordinates (*model, Q, LegEnd_id, Vector3d (0., 0., 0.));

	contact_body_id = Leg_id;
	contact_point_local = LegEnd_origin - Leg_origin;

	cout << "Contact point (local) = " << contact_point_local.transpose() << endl;

	constraint_set.AddContactConstraint (contact_body_id, contact_point_local, Vector3d (0., 0., 1.));

	constraint_set.Bind (*model);
	constraint_set.SetSolver (LinearSolverColPivHouseholderQR);

	/* Define problem dimensions */
	def_mdims(NMOS, NP, NRC, NRCE);
	
	def_mstage(
			0,
			NXD, NXA, NU,
            mfcn, NULL,
			0, 0, 0, NULL, ffcn_flight, NULL,
			NULL, NULL
			);

	def_mstage(
			1,
			NXD, NXA, NU,
			NULL, NULL,
			0, 0, 0, NULL, ffcn_touchdown, NULL,
			NULL, NULL
			);

	def_mstage(
			2,
			NXD, NXA, NU,
            mfcn, NULL,
			0, 0, 0, NULL, ffcn_contact, NULL,
			NULL, NULL
			);

	/* Point Constraints Contact phase */
	def_mpc(0, "Start Point", NPR, 0, 0, NULL, rcfcn_periodic_s);
	def_mpc(0, "Interior Point", NPR, 1, 0, rdfcn_flight_i, NULL);

	def_mpc(1, "Start Point", NPR, 2, 1, rdfcn_touchdown_s, NULL);
	
	def_mpc(2, "Interior Point", NPR, 1, 0, rdfcn_contact_i, NULL);
	def_mpc(2, "End Point", NPR, 2, 0, rdfcn_contact_e, rcfcn_periodic_e);

	def_mio (NULL, mout, data_out);
}
