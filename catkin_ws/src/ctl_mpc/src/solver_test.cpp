#include <iostream>

#include "acados_solver_astrobee.h"
#include "acados_c/ocp_nlp_interface.h"

// global data
ocp_nlp_in* nlp_in;
ocp_nlp_out* nlp_out; 
ocp_nlp_config* nlp_config;
ocp_nlp_dims* nlp_dims;

// TODO: Is there a way to retrieve this automatically? (e.g. for models with different number of states)
// Number of intervals in the horizon
#define N 10
// Number of state variables
#define NX 12
// Number of control inputs
#define NU 6

class MPC
{
    // Defining acados structs
    struct solver_output{
        double u0[NU];
        double x1[NX];
    };

    struct solver_input{
        double x0[NX];
        double xref[N][NX];
    };

    solver_input acados_in;
    solver_output acados_out;
    int acados_status;

    astrobee_solver_capsule* capsule;

    public:
        MPC()
        {
            capsule = astrobee_acados_create_capsule(); 
            int status = astrobee_acados_create(capsule);
            std::cout << "Printing in C++. Output status " << status << "\n";
            if(status != 0) {
                // TODO: Error handling
            }
        };

        void control() {
            acados_in.x0[0] = 0;
            acados_in.x0[1] = 0;
            acados_in.x0[2] = 0;
            acados_in.x0[3] = 0;
            acados_in.x0[4] = 0;
            acados_in.x0[5] = 0;
            acados_in.x0[6] = 0;
            acados_in.x0[7] = 0;
            acados_in.x0[8] = 0;
            acados_in.x0[9] = 0;
            acados_in.x0[10] = 0;
            acados_in.x0[11] = 0;

            for(int stage=0; stage<=N; stage++) {
                acados_in.xref[stage][0] = 0;
                acados_in.xref[stage][1] = 0;
                acados_in.xref[stage][2] = 0;
                acados_in.xref[stage][3] = 0;
                acados_in.xref[stage][4] = 0;
                acados_in.xref[stage][5] = 0;
                acados_in.xref[stage][6] = 0;
                acados_in.xref[stage][7] = 0;
                acados_in.xref[stage][8] = 0;
                acados_in.xref[stage][9] = 0;
                acados_in.xref[stage][10] = 0;
                acados_in.xref[stage][11] = 0;
            }

            // Set initial state
            void* test = (void*) acados_in.x0;
            std::cout << "At least we got here! \n";
            const char* test2 = (const char*) "lbx";
            std::cout << "At least we got here! \n";
            int status1 = ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, (const char*) "lbx", (void*) acados_in.x0);
            int status2 = ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, (const char*) "ubx", (void*) acados_in.x0);
            std::cout << "At least we got here! \n";

            // For loop to set reference for each stage (xref are params)
            for(int stage=0; stage<=N; stage++) {
                int status = astrobee_acados_update_params(capsule, stage, acados_in.xref[stage], NX);
                if(status != 0) {
                    // TODO: Error handling   
                }
            }

            // Solve
            acados_status = astrobee_acados_solve(capsule);
            if(acados_status != 0) {
                // TODO: Error handling
            }

            // Extract control from output. Saved to our acados_out struct
            nlp_out = astrobee_acados_get_nlp_out(capsule);
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void*) acados_out.u0);

            std::cout << acados_out.u0 << "\n"; 
        }
};

int main()
{
    // TODO: Set up ROS node

    // Set up solver
    MPC mpc;

    // TODO: In while loop of ros node, keep calling mpc.control()
    mpc.control();

    return 0;
}