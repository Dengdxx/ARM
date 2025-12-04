#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

// Use standard math functions for the mock
inline float arm_cos_f32(float x) { return std::cos(x); }
inline float arm_sin_f32(float x) { return std::sin(x); }

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

// Mock memory.h
#include <string.h>

// Forward declaration needed for the include
class DOF6Kinematic;

// Rename the inline functions in the source file to avoid conflict with standard library
#define cosf custom_cosf
#define sinf custom_sinf

#include "2.Firmware/Core-STM32F4-fw/Robot/algorithms/kinematic/6dof_kinematic.cpp"

#undef cosf
#undef sinf

void print_joints(DOF6Kinematic::Joint6D_t j) {
    printf("[%f, %f, %f, %f, %f, %f]\n", j.a[0], j.a[1], j.a[2], j.a[3], j.a[4], j.a[5]);
}

void print_pose(DOF6Kinematic::Pose6D_t p) {
    printf("X: %f, Y: %f, Z: %f, A: %f, B: %f, C: %f\n", p.X, p.Y, p.Z, p.A, p.B, p.C);
}

int main() {
    // Parameters from dummy_robot.cpp
    DOF6Kinematic solver(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);

    // Case 3: Singularity J5=0, but J4+J6 != 0.
    // [0,0,0,10,0,10] -> J4=10, J5=0, J6=10. Sum=20.
    DOF6Kinematic::Joint6D_t target_joints(0, 0, 0, 10, 0, 10);
    DOF6Kinematic::Pose6D_t target_pose;

    solver.SolveFK(target_joints, target_pose);

    printf("Target Pose (FK output, meters): \n");
    print_pose(target_pose);

    DOF6Kinematic::Pose6D_t ik_target_pose = target_pose;
    ik_target_pose.X *= 1000.0f;
    ik_target_pose.Y *= 1000.0f;
    ik_target_pose.Z *= 1000.0f;

    DOF6Kinematic::Joint6D_t last_joints(0, 0, 0, 10, 0, 10);
    DOF6Kinematic::IKSolves_t solutions;

    solver.SolveIK(ik_target_pose, last_joints, solutions);

    bool found_bad = false;
    for(int i=0; i<8; i++) {
        printf("Solution %d: ", i);
        print_joints(solutions.config[i]);

        // Check FK for this solution
        DOF6Kinematic::Pose6D_t sol_pose;
        solver.SolveFK(solutions.config[i], sol_pose);

        float dist_sq = pow(sol_pose.X - target_pose.X, 2) +
                        pow(sol_pose.Y - target_pose.Y, 2) +
                        pow(sol_pose.Z - target_pose.Z, 2);

        float r_err = 0;
        for(int k=0; k<9; k++) r_err += fabs(sol_pose.R[k] - target_pose.R[k]);

        printf("  FK Error: DistSq=%f, R_err=%f\n", dist_sq, r_err);

        if (dist_sq > 0.0001 || r_err > 0.01) {
             printf("  INVALID solution returned by IK!\n");
             // Check if this is one of the uninitialized ones (Index odd: 1, 3, 5, 7)
             if (i % 2 != 0) {
                 found_bad = true;
             }
        }
    }

    if (found_bad) {
        printf("FAIL: IK returned invalid solutions (likely uninitialized).\n");
        return 1;
    } else {
        printf("PASS: All returned solutions are valid.\n");
        return 0;
    }
}
