#include <cmath>

#include "def_usrmod.hpp"

#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "rbdl/rbdl.h"
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

// datfileutils.h needs VectorNd and therefore RigidBodyDynamics::Math
#include "datfileutils.h"

// Global Definitions

int NMOS   =  1;  /* Number of phases (MOdel Stages) */
int NP     =  0;  /* Number of parameters */
int NRC    =  0;  /* Number of coupled constraints */
int NRCE   =  0;  /* Number of coupled equality constraints */

int NXD    =  4;  /* Number of differential states */
int NXA    =  0;  /* Number of algebraic states */
int NU     =  1;  /* Number of controls */
int NPR    =  0;  /* Number of local parameters */

// Variables

Model *model = NULL;
string model_file_name  ="model.lua";
string meshup_file_name = "meshup_cart_pendulum.csv";

VectorNd Q, QDOT, QDDOT, TAU;

unsigned int nDof;
unsigned int nActuatedDof;

// ********************************o
// *
// * Utilities
// *
// ********************************o

// update RBDL vectors
static void updateState (const double *sd, const double *u=NULL)
{
    Q   [0] = sd[0];
    Q   [1] = sd[1];
    QDOT[0] = sd[2];
    QDOT[1] = sd[3];

    TAU[0]  = 0.0;
    TAU[1]  = 0.0;

    if (u)
    {
        TAU[0] = u[0];
    }
}

// ********************************o
// *
// * Objective Funtions
// *
// ********************************o

/** \brief Objective function (Lagrangian type) */
static void lfcn_energy(double *t, double *xd, double *xa, double *u,
  double *p, double *lval, double *rwh, long *iwh, InfoPtr *info) {
    int i;

    *lval = u[0] * u[0]; // Same objective as in the rocket car example

}

/** \brief Objective function (Mayer type) */
static void mfcn_end_time(double *ts, double *xd, double *xa, double *p, double *pr,
    double *mval,  long *dpnd, InfoPtr *info) {
  if (*dpnd) {
      *dpnd = MFCN_DPND(*ts, 0, 0, 0, 0);
      return;
  }
  *mval = *ts;
}

// ********************************o
// *
// * Right Hand Sides
// *
// ********************************o

static void ffcn (double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
    updateState (xd, u);

    // Compute Forward Dynamics with RBDL
    ForwardDynamics(*model, Q, QDOT, TAU, QDDOT);

    for (unsigned int i = 0; i < nDof; i++) {
        rhs[i]        = QDOT[i];
        rhs[i + nDof] = QDDOT[i];
    }
}

// ********************************o
// *
// * Decoupled Constraints
// *
// ********************************o

//         # of all constraints       # of equality constraints
static int RDFCN_S_N = 4, RDFCN_S_NE = 4;
static void rdfcn_s(double *ts, double *sd, double *sa, double *u,
        double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
    if (*dpnd) {
        *dpnd = RFCN_DPND(NULL, *sd, 0, 0, 0, 0);
        return;
    }
    updateState (sd, NULL);
    res[0] = ( sd[0] ) * 100; // = 0 (* 100 just for scaling reasons)
    res[1] = ( sd[1] - M_PI ) * 100; // = 0
    res[2] = ( sd[2] ) * 100; // = 0
    res[3] = ( sd[3] ) * 100; // = 0
}

/// \brief Constraints at end point */
static int RDFCN_E_N = 4, RDFCN_E_NE = 4;
static void rdfcn_e(double *ts, double *sd, double *sa, double *u,
        double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
    if (*dpnd) {
        *dpnd = RFCN_DPND(NULL, *sd, 0, 0, 0, 0);
        return;
    }
    updateState (sd, NULL);
    res[0] = ( sd[0] ) * 100; // = 0 (* 100 just for scaling reasons)
    res[1] = ( sd[1] ) * 100; // = 0
    res[2] = ( sd[2] ) * 100; // = 0
    res[3] = ( sd[3] ) * 100; // = 0
}


// ********************************o
// *
// * Data Output
// *
// ********************************o

static void data_output( double *t, double *sd, double *sa, double *u, double *p, double *rwh, long *iwh, InfoPtr* info ) {
    FILE *fp;

    stringstream output_filename ("");

    const char* meshup_header = "COLUMNS: \n\
    time,\n\
    cart:T:X,\n\
    pendulum:R:Y:rad";

    // define output path
    output_filename << "./RES/" << meshup_file_name;

    // if in first MS node then create file, else append data
    // get imsn and imos from InfoPtr
    if ((info->cnode == 0) && (info->cimos == 0)) {
        fp = fopen (output_filename.str().c_str(), "w");

        if (!fp) {
            fprintf (stderr, "Error: Could not open file '%s'!\n", output_filename.str().c_str());
            return;
        }

        // write the header of the meshup file
        fprintf (fp, "%s\n", meshup_header);
        fprintf (fp, "DATA:\n");
    }
    else {
        fp = fopen (output_filename.str().c_str(), "a");

        if (!fp) {
            fprintf (stderr, "Error: Could not open file '%s'!\n", output_filename.str().c_str());
            return;
        }
    }

    // write data
    int i;

    // print time
    fprintf (fp, "%e,\t", *t);

    // print degrees of freedom
    for (i = 0; i < nDof; i ++) {
        fprintf (fp, "%e,\t", sd[i]);
    }
    fprintf (fp, "\n");

    // close file
    fclose (fp);
}

void meshup_output
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
)
{
    InfoPtr info(0, *imos, *imsn);
    data_output( ts, sd, sa, u, p, rwh, iwh, &info);
}

// \brief Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void)
{
    // load LUA model
    model = new Model;

    if (!Addons::LuaModelReadFromFile (model_file_name.c_str(), model, false)) {
        cerr << "Error loading RBDL model '" << model_file_name << "'!" << endl;
        abort();
    }

    assert (model->dof_count == 2);

    nDof = model->dof_count;
    nActuatedDof = 2;

    Q     = VectorNd::Zero(model->dof_count);
    QDOT  = VectorNd::Zero(model->dof_count);
    QDDOT = VectorNd::Zero(model->dof_count);
    TAU   = VectorNd::Zero(model->dof_count);

    def_mdims(NMOS, NP, NRC, NRCE);

    // right_flat
    def_mstage(
            0,
            NXD, NXA, NU,
            NULL, lfcn_energy,   // mayer term, lagrange term
            0, 0, 0, NULL, ffcn, NULL,
            NULL, NULL
            );

    def_mpc(0, "s", NPR, RDFCN_S_N, RDFCN_S_NE, rdfcn_s, NULL);
    def_mpc(0, "e", NPR, RDFCN_E_N, RDFCN_E_NE, rdfcn_e, NULL);

    def_mio (NULL , meshup_output, data_output);

#ifdef HAVE_USERPLOT
    def_userplot (plot_start, plot_data, plot_end);
#endif
}
