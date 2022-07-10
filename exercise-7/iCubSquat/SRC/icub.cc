/*
 *  icub.cc
 *  (c) Kevin Stein <Kevin.Stein@ziti.uni-heidelberg.de>, 2019
 */

#include <cmath>

#include "def_usrmod.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

#include <iostream>
#include <fstream>

const int NMOS = _;  /* Number of phases (Model Stages) */
const int NP   = _;  /* Number of parameters */
const int NRC  = 0;  /* Number of coupled constraints */
const int NRCE = 0;  /* Number of coupled equality constraints */

const int NXD  = __;  /* Number of differential states */
const int NXA  = 0;  /* Number of algebraic states */
const int NU   = __;  /* Number of controls */
const int NPR  = 0;  /* Number of local parameters */

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Model humanoid;
std::string model_file="iCubHeidelberg01.lua";
std::string result_file="RES/squat.csv";
std::string force_file="RES/squat_forces.ff";

unsigned int contact_body_id_r;
unsigned int contact_body_id_l;
unsigned int chest_body_id;

ConstraintSet constraint_set;

VectorNd Q, QDot, QDDot, Tau;

unsigned int nDof;

void update_generalized_variables (double *xd, double *u, VectorNd &Q, VectorNd &QDot, VectorNd &Tau) {
// Implement Function
}

/** \brief Objective function (Lagrangian type) */
static void lfcn(double *t, double *xd, double *xa, double *u,
  double *p, double *lval, double *rwh, long *iwh, InfoPtr *info)
{
//implement min Torque
}


/** \brief Objective function (Mayer type) */
void mfcn_max_height(double *ts, double *xd, double *xa, double *p, double *pr,
double *mval,  long *dpnd, InfoPtr *info) {
  if (*dpnd) { *dpnd = MFCN_DPND(*ts, 0, 0, *p, 0); return; }
//Implement Mayer Term
}

/** \brief Right hand side of the differential equation */
static void ffcn_contact(double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
// Implement forward dynamics
}

static int rdfcn_contact_s_n = 6, rdfcn_contact_s_ne = 6;
static void rdfcn_contact_s(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, *u, *p, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);
    

    res[0] = ___
	res[1] = ___
	res[2] = ___
 	res[3] = ___
	res[4] = ___
    res[5] = ___
    
}


static int rdfcn_contact_i_n = 4, rdfcn_contact_i_ne = 1;
static void rdfcn_contact_i(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, *u, *p, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);
    
    res[0] = __
    res[1] = __
	res[2] = __
	res[3] = __
}

static int rdfcn_contact_e_n = 1, rdfcn_contact_e_ne = 1;
static void rdfcn_contact_e(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(0, *sd, 0, *u, *p, 0);
    return;
  }

	update_generalized_variables (sd, u, Q, QDot, Tau);
    
    res[0] =  ___
}


vector<double> t_values;
vector<VectorNd> sd_values;
vector<VectorNd> u_values;

static void data_out( double *t, double *sd, double *sa, double *u, double *p, double *rwh, long *iwh, InfoPtr *info ) {
	if (*t == 0.) {
		ofstream meshup_csv_stream;
        ofstream force_stream;
        

		meshup_csv_stream.open (result_file.c_str(), ios_base::trunc);

		if (!meshup_csv_stream) {
			cerr << "Error opening file " << result_file << endl;
			abort();
		}
		
        force_stream.open (force_file.c_str(), ios_base::trunc);

		if (!force_stream) {
			cerr << "Error opening file " << force_file << endl;
			abort();
		}
		
		if (t_values.size() > 0) {
			for (unsigned int i = 0; i < t_values.size(); i ++) {
				meshup_csv_stream << t_values[i] << ", ";
				force_stream << t_values[i] << ", ";
                
                
				for (unsigned int j = 0; j < 6; j++) {
					meshup_csv_stream << sd_values[i][j] << ", ";
				}
				
                meshup_csv_stream << -sd_values[i][3] << ", ";
                meshup_csv_stream << sd_values[i][4] << ", ";
                meshup_csv_stream << -sd_values[i][5] << ", ";
                
				// output of the constraint force
				update_generalized_variables (&(sd_values[i][0]), &(u_values[i][0]), Q, QDot, Tau);
				ForwardDynamicsConstraintsDirect (humanoid, Q, QDot, Tau, constraint_set, QDDot);
                
    
                Vector3d contact_point_r = CalcBodyToBaseCoordinates (humanoid, Q, contact_body_id_r, Vector3dZero);
                Vector3d contact_point_r2 = CalcBodyToBaseCoordinates (humanoid, Q, contact_body_id_r, Vector3d(0.1,0.0,0.0));
                Vector3d contact_point_l = CalcBodyToBaseCoordinates (humanoid, Q, contact_body_id_l, Vector3dZero);
                Vector3d contact_point_l2 = CalcBodyToBaseCoordinates (humanoid, Q, contact_body_id_l, Vector3d(0.1,0.0,0.0));
                
                for (unsigned int f = 0; f < 3; f ++){
                    force_stream << contact_point_r[f] << ", ";
                }
                force_stream << constraint_set.force[0] <<  ", 0.0, " << constraint_set.force[1] << ", 0.0, 0.0, 0.0, ";
                
                for (unsigned int f = 0; f < 3; f ++){
                    force_stream << contact_point_r2[f] << ", ";
                }
                force_stream << "0.0, 0.0, " << constraint_set.force[2] << ", 0.0, 0.0, 0.0, ";    
                
                for (unsigned int f = 0; f < 3; f ++){
                    force_stream << contact_point_l[f] << ", ";
                }
                force_stream << constraint_set.force[3] <<  ", 0.0, " << constraint_set.force[4] << ", 0.0, 0.0, 0.0, ";
                
                for (unsigned int f = 0; f < 3; f ++){
                    force_stream << contact_point_l2[f] << ", ";
                }
                force_stream << "0.0, 0.0, " << constraint_set.force[5] << ", 0.0, 0.0, 0.0, ";  
                
// 				force_stream << ", " << constraint_set.force[0];
//                 
//                 cout << constraint_set.force << endl;

                force_stream << endl;
				meshup_csv_stream << endl;
			}
		}

		t_values.clear();
		sd_values.clear();
		u_values.clear();

		meshup_csv_stream.close();
        force_stream.close();
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
	printf("Load model %s!\n",model_file.c_str());

	if (!Addons::LuaModelReadFromFile (model_file.c_str(), &humanoid, false)) {
		cerr << "Error loading RBDL model '" << model_file << "'!" << endl;
		abort();
	}
	
	printf("Model loaded!\n");

	assert (humanoid.dof_count == 9);

	nDof = humanoid.dof_count;

	Q = VectorNd::Zero(humanoid.dof_count);
	QDot = VectorNd::Zero(humanoid.dof_count);
	QDDot = VectorNd::Zero(humanoid.dof_count);
	Tau = VectorNd::Zero(humanoid.dof_count);

	// Initialization of the constraint set

    contact_body_id_l = humanoid.GetBodyId("l_sole");
    contact_body_id_r = humanoid.GetBodyId("r_sole");
    chest_body_id = humanoid.GetBodyId("chest");
    
    constraint_set.AddContactConstraint(contact_body_id_l, Vector3d(0.0,0.0,0.0), Vector3d (1., 0., 0.));
    constraint_set.AddContactConstraint(contact_body_id_l, Vector3d(0.0,0.0,0.0), Vector3d (0., 0., 1.));
    constraint_set.AddContactConstraint(contact_body_id_l, Vector3d(0.1,0.0,0.0), Vector3d (0., 0., 1.));
    
    constraint_set.AddContactConstraint(contact_body_id_r, Vector3d(0.0,0.0,0.0), Vector3d (1., 0., 0.));
    constraint_set.AddContactConstraint(contact_body_id_r, Vector3d(0.0,0.0,0.0), Vector3d (0., 0., 1.));
    constraint_set.AddContactConstraint(contact_body_id_r, Vector3d(0.1,0.0,0.0), Vector3d (0., 0., 1.));
    
    constraint_set.Bind(humanoid);
// 	constraint_set.SetSolver (LinearSolverColPivHouseholderQR);

	/* Define problem dimensions */
	def_mdims(NMOS, NP, NRC, NRCE);
	
	def_mstage(
			0,
			NXD, NXA, NU,
			mfcn_max_height, lfcn,
			//mfcn_max_height, NULL,
			0, 0, 0, NULL, ffcn_contact, NULL,
			NULL, NULL
			);

	/* Point Constraints Contact phase */
   	def_mpc(0, "Start Point", NPR, rdfcn_contact_s_n, rdfcn_contact_s_ne, rdfcn_contact_s, NULL);
    def_mpc(0, "Interior Point", NPR, rdfcn_contact_i_n, rdfcn_contact_i_ne, rdfcn_contact_i, NULL);
    def_mpc(0, "End Point", NPR, rdfcn_contact_e_n, rdfcn_contact_e_ne, rdfcn_contact_e, NULL);

	def_mio (NULL, mout, data_out);
}
