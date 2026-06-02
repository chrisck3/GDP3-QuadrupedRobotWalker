// Scale factor: positive gait z input -> foot moves away from body (lifts in swing).
// Change this value to tune effective lift height without editing the gait table.
// Effective lift = March_table_value * stepHeightScale  (must stay < maxReach - hleg)
double stepHeightScale = -2.2;

// Fills q with foot target positions for gait phase 'iter'.
// Result is the sum of the lift arc (March) + forward/back (FB*a) +
// lateral (LR*b) + yaw (RT*c) + weight-shift (WS*ws) tables.
void create_mat(double q[legn][coor],gait_struct *gait,float a,float b,float c,float ws,int iter){
    a = (double) a;
    b = (double) b;
    c = (double) c;
    ws = (double) ws;
    for(int ii=0;ii<legn;ii++){
        for(int jj=0;jj<coor;jj++){
        q[ii][jj] = (gait->March[iter][ii][jj]+a*gait->FB[iter][ii][jj]+b*gait->LR[iter][ii][jj]+c*gait->RT[iter][ii][jj]+ws*gait->WS[iter][ii][jj]);
      }
    }
}

// Per-leg configuration table
// ax0_phi / ax1_phi : 1=phi1, 2=phi2  (which IK angle drives which ODrive axis)
// ax0_gain / ax1_gain: +/-4            (flip sign if leg moves wrong direction)
// sh_sign           : +1 or -1        (flip if shoulder swings wrong way)
// foot_mirror       : true for front legs. assembly is front-back mirrored vs back legs.
//                     Swaps off1/off2 in the height adjustment only (NOT the main IK),
//                     so that Motor A/B use the correct phi for extension without
//                     disturbing the gait formula.
// Offsets are NOT stored here. computed dynamically from hleg[i] in kinematics_mat()
struct LegConfig {
    int   ax0_phi;
    float ax0_gain;
    int   ax1_phi;
    float ax1_gain;
    float sh_sign;
    bool  foot_mirror;
};

LegConfig legCfg[4] = {
//   ax0_phi  ax0_gain  ax1_phi  ax1_gain  sh_sign  foot_mirror
    {  1,      -4.0f,    2,      +4.0f,    +1.0f,   true  },  // FR (mirrored assembly)
    {  1,      +4.0f,    2,      -4.0f,    -1.0f,   false },  // BL  (phi+gain swapped vs FR, validated on rig)
    {  2,      -4.0f,    1,      +4.0f,    -1.0f,   true  },  // FL (mirrored assembly)
    {  1,      -4.0f,    2,      +4.0f,    +1.0f,   false },  // BR
};


// Converts foot position targets (x/y/z offsets from neutral) into ODrive
// encoder turn commands for all four legs.
//
// Coordinate system (body frame): x = lateral, y = forward, z = up.
// Leg geometry: two equal links (leglen) + foot extension (feetlen).
//   phi1 = upper link angle (hip joint)
//   phi2 = lower link angle (knee joint)
//   phi3 = shoulder abduction angle (lateral swing)
//
// Output: ODrive encoder positions in turns (not radians).
// The 0.25 - phi/(2*PI) formula maps the IK angle to the encoder zero
// defined by actuate2()'s home position.
void kinematics_mat(double inp[legn][coor], double out[legn][coor], double pos[legn][coor]){

    double phi1, phi2;     // hip and knee joint angles (radians) from 2-link planar IK
    double phi3;           // shoulder abduction angle (radians)
    double c1, c2;         // reach to foot, and inter-joint distance
    double theta, beta, alpha, gamma, s, z; // intermediate IK geometry values


    for(int i=0; i<legn; i++){

        double x   = inp[i][0];
        double y   = inp[i][1];
        double z_in = inp[i][2] * stepHeightScale;  // stepHeightScale defined at top of file

        z = hleg[i] - z_in;
        if(z < 1e-6) z = 1e-6;
        s = sqrt(x*x + z*z);
        if(s < 1e-6) s = 1e-6;

        phi3 = atan2(x, z);

        c1 = sqrt(y*y + s*s);
        double maxReach = 2*leglen + feetlen - 5; //5mm safety margin
        if(c1 > maxReach) c1 = maxReach;

        double cosTheta = (c1*c1 - 2*leglen*leglen - 2*leglen*feetlen - feetlen*feetlen)
                        / (-2*leglen*leglen - 2*leglen*feetlen);
        cosTheta = constrain(cosTheta, -1.0, 1.0);
        theta = acos(cosTheta);

        c2    = sqrt(2*leglen*leglen - 2*leglen*leglen*cos(theta));
        beta  = acos(constrain(c2/(2*leglen), -1.0, 1.0));
        alpha = acos(constrain((feetlen*feetlen - c1*c1 - c2*c2)/(-2*c1*c2), -1.0, 1.0));
        gamma = fabs(atan2(z, fabs(y)));

        if(y <= 0){ phi1 = gamma - alpha - beta; }
        else      { phi1 = M_PI - gamma - alpha - beta; }
        phi2 = M_PI - phi1 - 2*beta;

        double phi_ax0 = (legCfg[i].ax0_phi == 1) ? phi1 : phi2;
        double phi_ax1 = (legCfg[i].ax1_phi == 1) ? phi1 : phi2;

        // Gait offsets from target height hleg[i], gives correct swing shape around hleg.
        float dyn_off1, dyn_off2;
        compute_offsets(hleg[i], dyn_off1, dyn_off2);
        float ax0_offset = (legCfg[i].ax0_phi == 1) ? dyn_off1 : dyn_off2;
        float ax1_offset = (legCfg[i].ax1_phi == 1) ? dyn_off1 : dyn_off2;

        // Reference offsets at inipos capture height (hleg_base, never changes).
        // Height adjustment = gain * (base_off - hleg_off): moves leg physically to hleg standing height.
        // Front legs (foot_mirror=true) are a front-back mirror of back legs: Motor A/B swap their
        // kinematic roles for pure vertical extension, so off1 and off2 are swapped here only.
        // The main IK offset (ax0_offset/ax1_offset above) is unchanged to preserve gait correctness.
        float base_off1, base_off2;
        compute_offsets(hleg_base[i], base_off1, base_off2);
        float h_b1 = legCfg[i].foot_mirror ? base_off2 : base_off1;
        float h_b2 = legCfg[i].foot_mirror ? base_off1 : base_off2;
        float h_d1 = legCfg[i].foot_mirror ? dyn_off2  : dyn_off1;
        float h_d2 = legCfg[i].foot_mirror ? dyn_off1  : dyn_off2;
        float ax0_base = (legCfg[i].ax0_phi == 1) ? h_b1 : h_b2;
        float ax1_base = (legCfg[i].ax1_phi == 1) ? h_b1 : h_b2;
        float ax0_height_adj = legCfg[i].ax0_gain * (ax0_base - ((legCfg[i].ax0_phi==1) ? h_d1 : h_d2));
        float ax1_height_adj = legCfg[i].ax1_gain * (ax1_base - ((legCfg[i].ax1_phi==1) ? h_d1 : h_d2));

        out[i][0] = constrain(
            legCfg[i].ax0_gain * (0.25 - constrain(phi_ax0/(2*M_PI),-0.25,0.25) - ax0_offset),
            -0.3, 1.0) + pos[i][0] + ax0_height_adj;

        out[i][1] = constrain(
            legCfg[i].ax1_gain * (0.25 - constrain(phi_ax1/(2*M_PI),-0.25,0.25) - ax1_offset),
            -1.0, 0.3) + pos[i][1] + ax1_height_adj;

        out[i][2] = constrain(
            legCfg[i].sh_sign * 40.0 * constrain(phi3/(2*M_PI),-0.25,0.25),
            -2.5, 2.5) + pos[i][2];
    }
}

// Compute encoder offsets from physical standing height (hleg).
// Runs neutral-pose kinematics (x=0, y=0, z_in=0) → derives phi1, phi2 → derives offsets.
// Call in setup() after hleg[] is set but before pos_est().
// If leglen or feetlen change, offsets update automatically
void compute_offsets(float h, float &off1, float &off2) {
    // Neutral pose: x=0, y=0 → z=h, s=h, c1=h
    float c1 = h;
    float cosTheta = (c1*c1 - 2*leglen*leglen - 2*leglen*feetlen - feetlen*feetlen)
                   / (-2*leglen*leglen - 2*leglen*feetlen);
    cosTheta = constrain(cosTheta, -1.0f, 1.0f);
    float theta = acos(cosTheta);
    float c2    = sqrt(2*leglen*leglen - 2*leglen*leglen*cos(theta));
    float beta  = acos(constrain(c2 / (2*leglen), -1.0f, 1.0f));
    float alpha = acos(constrain((feetlen*feetlen - c1*c1 - c2*c2) / (-2*c1*c2), -1.0f, 1.0f));
    float gamma = M_PI / 2.0f;          // y=0 → gamma = pi/2
    float phi1  = gamma - alpha - beta; // y<=0 branch
    float phi2  = M_PI - phi1 - 2*beta;
    off1 = 0.25f - phi1 / (2*M_PI);
    off2 = 0.25f - phi2 / (2*M_PI);
}
