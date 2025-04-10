void dynamics(const double * const x, /* primal vars */
const double * const p, /* runtime parameters */
double * const c, /* Zero order function */
double * const nabla_c , /* first order Fcn */
const int stage, /* stage number (0 indexed ) */
const int threadID /* thread number */
)
{
    // Parameters É POSSÍVEL QUE SEJA NECESSÁRIO ALTERAR OS INDICES PORQUE ESTÁ A MULTIPLICAR PELO HORIZONTE
    const float l_f = p[9];
    const float l_r = p[10];
    const float g = p[20];
    const float m = p[11];
    const float rho = p[22];
    const float C_l = p[26];
    const float downforce_front = p[30];
    const float downforce_rear = p[31];
    const float T_max_front = p[13];
    const float T_max_rear = p[14];
    const float T_brake_front = p[15];
    const float T_brake_rear = p[16];
    const float GR = p[17];
    const float r_wheel = p[19];
    const float eta_motor = p[18];
    const float C_roll = p[21];
    const float C_d = p[25];
    const float D = p[29];
    const float C = p[28];
    const float B = p[27];
    const float I_z = p[12];
    const float h_cog = p[33];
    const float N = p[46];
    const float controller_freq = p[47];

    const float integrator_stepsize = 1.0/controller_freq;

    if (stage == N-1){
        c = dynamics_eq_N();
        nabla_c = dynamics_eq_N_nabla();
    }else{
        c = dynamics_eq();
        nabla_c = dynamics_eq_nabla();
    }
}

double * const c = dynamics_eq(){
    double * const c = (double *)malloc(6 * sizeof(double));
    c[0] = x[3]*cos(x[2]) - x[4]*sin(x[2]);
    c[1] = x[3]*sin(x[2]) + x[4]*cos(x[2]);
    c[2] = x[5];
    c[3] = (T_max_front*U[0] - T_brake_front*U[1] - C_d*pow(x[3],2)*sign(x[3]) - C_roll*m*g*sign(x[3]) - downforce_front)/m;
    c[4] = (T_max_rear*U[2] - T_brake_rear*U[3] - C_d*pow(x[4],2)*sign(x[4]) - C_roll*m*g*sign(x[4]) - downforce_rear)/m;
    c[5] = (l_f*T_max_front*U[0] - l_r*T_max_rear*U[2] - l_f*T_brake_front*U[1] + l_r*T_brake_rear*U[3] - C_d*pow(x[5],2)*sign(x[5]) - C_roll*m*g*h_cog*sign(x[5]))/I_z;
    return c;
}

double * const nabla_c = dynamics_eq_nabla(){
    double * const nabla_c = (double *)malloc(6 * sizeof(double));
    nabla_c[0] = 0;
    nabla_c[1] = 0;
    nabla_c[2] = 0;
    nabla_c[3] = 0;
    nabla_c[4] = 0;
    nabla_c[5] = 0;
    return nabla_c;
}

double * const c = dynamics_eq_N(){
    double * const c = (double *)malloc(6 * sizeof(double));
    c[0] = x[3]*cos(x[2]) - x[4]*sin(x[2]);
    c[1] = x[3]*sin(x[2]) + x[4]*cos(x[2]);
    c[2] = x[5];
    c[3] = (T_max_front*U[0] - T_brake_front*U[1] - C_d*pow(x[3],2)*sign(x[3]) - C_roll*m*g*sign(x[3]) - downforce_front)/m;
    c[4] = (T_max_rear*U[2] - T_brake_rear*U[3] - C_d*pow(x[4],2)*sign(x[4]) - C_roll*m*g*sign(x[4]) - downforce_rear)/m;
    c[5] = (l_f*T_max_front*U[0] - l_r*T_max_rear*U[2] - l_f*T_brake_front*U[1] + l_r*T_brake_rear*U[3] - C_d*pow(x[5],2)*sign(x[5]) - C_roll*m*g*h_cog*sign(x[5]))/I_z;
    return c;
}

double * const nabla_c = dynamics_eq_N_nabla(){
    double * const nabla_c = (double *)malloc(6 * sizeof(double));
    nabla_c[0] = 0;
    nabla_c[1] = 0;
    nabla_c[2] = 0;
    nabla_c[3] = 0;
    nabla_c[4] = 0;
    nabla_c[5] = 0;
    return nabla_c;
}

double dX = vehicle_dynamics(){

}

double y = euler_step(x,dX,h){
    y = x + dX*h;
}