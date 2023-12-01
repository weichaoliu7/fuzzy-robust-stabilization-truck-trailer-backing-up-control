#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// reference: Tanaka K, Sano M. A robust stabilization problem of fuzzy control systems and its application to backing up control of a truck-trailer[J].
// IEEE Transactions on Fuzzy systems, 1994, 2(2): 119-134.
// global variables declaration
#define PI 3.14159
#define ARRAY_SIZE 150000 // sampling times

static double Ts = 0.001;             // sampling period
static double t0 = 0.0;               // start time
static double t1 = 150.0;             // end time
static double smallnumber;            // small number
static double velocity = -1.0;        // constant speed of backing up, m/s
static double width_truck = 2;        // width of truck, m
static double width_trailer = 2;      // width of trailer, m
static double length_truck = 2.8;     // length of truck, m
static double length_trailer = 6.5;   // length of trailer, m
static double length_jackknife = 0.7; // length of jackknife, m
double keypoint[10][2];

struct _archive{
    double x0_archive[ARRAY_SIZE]; // x0 :angle of truck
    double x1_archive[ARRAY_SIZE]; // x1: angle difference between truck and trailer
    double x2_archive[ARRAY_SIZE]; // x2: angle of trailer
    double x3_archive[ARRAY_SIZE]; // x3: vertical position of rear end of trailer
    double x4_archive[ARRAY_SIZE]; // x4: horizontal position of rear end of trailer
    double point_archive[10 * 2][ARRAY_SIZE];
    double control_archive[ARRAY_SIZE];
} archive;

struct _system_state{
    double x[5];            // state variable
    double x_derivative[5]; // derivative of state variable
} system_state;

struct _controller{
    int scenario; // four cases of truck-trailer initial positions
    int tempflag; // sign of x2(k) + v*t/(2*L)-x1(k) about pi (rad) or -pi (rad) and about 0
    double controller_u[5];
    double controller_out[2];
    double x[3];           // state variable x1,x2,x3
    double fuzzy_gain1[3]; // feedback gain
    double fuzzy_gain2[3]; // feedback gain
    double temp;           // x2(k) + v*t/(2*L)-x1(k)
    double control;        // control volume, truck steering angle
    double membership[2];  // membership function of x2(k) + v*t/(2*L)-x1(k) about pi (rad) or -pi (rad) and about 0
} controller;

void CONTROLLER_init(){
    controller.scenario = 0; // select truck-trailer initial position
    double scenarionumber[4][5] = {{0.0, 0.0, 0.0, 20.0, 0.0}, {PI, 0.0, PI, 20.0, 0.0}, 
    {PI / 4, -PI / 4, PI / 2, -10.0, 0.0}, {PI / 4, -PI / 2, 3 * PI / 4, -10.0, 0.0}};
    switch (controller.scenario){
    case 0:
        for (int j = 0; j < 5; j++){
            system_state.x[j] = scenarionumber[0][j];
        } break;
    case 1:
        for (int j = 0; j < 5; j++){
            system_state.x[j] = scenarionumber[1][j];
        } break;
    case 2:
        for (int j = 0; j < 5; j++){
            system_state.x[j] = scenarionumber[2][j];
        } break;
    case 3:
        for (int j = 0; j < 5; j++){
            system_state.x[j] = scenarionumber[3][j];
        } break;
    }

    double f1[] = {1.2 * 1.2837, 1.2 * -0.4139, 1.2 * 0.0201};
    double f2[] = {1.2 * 0.9773, 1.2 * -0.0709, 1.2 * 0.0005};
    for (int j = 0; j < 3; j++) {
        controller.fuzzy_gain1[j] = f1[j];
        controller.fuzzy_gain2[j] = f2[j];
    }
    
}

double CONTROLLER_realize(int i){
    controller.controller_u[0] = system_state.x[0]; // angle of truck
    controller.controller_u[1] = system_state.x[1]; // angle difference between truck and trailer
    controller.controller_u[2] = system_state.x[2]; // angle of trailer
    controller.controller_u[3] = system_state.x[3]; // vertical position of rear end of trailer
    controller.controller_u[4] = system_state.x[4]; // horizontal position of rear end of trailer

    archive.x0_archive[i] = controller.controller_u[0];
    archive.x1_archive[i] = controller.controller_u[1];
    archive.x2_archive[i] = controller.controller_u[2];
    archive.x3_archive[i] = controller.controller_u[3];
    archive.x4_archive[i] = controller.controller_u[4];

    for (int j = 0; j < 3; j++){
        controller.x[j] = controller.controller_u[j + 1]; // state variable x1,x2,x3
    }

    // x2(k) + v*t/(2*L)*x1(k)
    double time = i * Ts + t0;
    controller.temp = system_state.x[2] + velocity * time / (2 * length_trailer) * system_state.x[1];

    // if x2(k) + v*t/(2*L)*x1(k) is about pi (rad) or -pi (rad)
    if (controller.temp >= -PI && controller.temp <= 0)
        controller.membership[0] = -controller.temp / PI;
    else if (controller.temp > 0 && controller.temp <= PI)
        controller.membership[0] = controller.temp / PI;
    else
        controller.membership[0] = 0;

    // if x2(k) + v*t/(2*L)*x1(k) is about 0 (rad)
    if (controller.temp >= -PI && controller.temp <= 0)
        controller.membership[1] = (controller.temp + PI) / PI;
    else if (controller.temp > 0 && controller.temp <= PI)
        controller.membership[1] = (PI - controller.temp) / PI;
    else
        controller.membership[1] = 0;

    // rule1: if x2(k) + v*t/(2*L)-x1(k) is about pi (rad) or -pi (rad), then u(k) = f2*x(k)
    if (controller.membership[0] > controller.membership[1]){
        controller.tempflag = 0;
        controller.control = 0.0;
        for (int j = 0; j < 3; j++){
            controller.control += controller.fuzzy_gain2[j] * controller.x[j]; // truck steering angle
        }
    }

    // rule2: if x2(k) + v*t/(2*L)-x1(k) is about 0 (rad), then u(k) = f1*x(k)
    else if (controller.membership[0] < controller.membership[1]){
        controller.tempflag = 1;
        controller.control = 0.0;
        for (int j = 0; j < 3; j++){
            controller.control += controller.fuzzy_gain1[j] * controller.x[j]; // truck steering angle
        }
    }
    else{
        controller.tempflag = 2;
        controller.control = 0.0;
    }
    controller.controller_out[0] = controller.tempflag;
    controller.controller_out[1] = controller.control;
}

struct _plant{
    double plant_u[2];
    double plant_out[5];
    int simplifiedmodel;
} plant;

void PLANT_init(){
    for (int j = 0; j < 5; j++){
        plant.plant_out[j] = system_state.x[j];
    }
    smallnumber = pow(10, -2) / PI;
}

double PLANT_realize(int i){
    plant.plant_u[0] = controller.controller_out[0];
    plant.plant_u[1] = controller.controller_out[1];
    plant.simplifiedmodel = 0;
    double time = i * Ts + t0;

    switch (plant.simplifiedmodel){

    // simplified truck-trailer model
    case 0:
        if (plant.plant_u[0] == 0){
            // derivative of angle difference between truck and trailer, Eq. 34
            system_state.x_derivative[1] = -(velocity / length_trailer) * system_state.x[1] + (velocity / length_truck) * controller.control;
            // derivative of angle of trailer
            system_state.x_derivative[2] = (velocity / length_trailer) * system_state.x[1];
            // derivative of vertical position of rear end of trailer
            system_state.x_derivative[3] = (smallnumber * velocity * velocity * time) / (2 * length_trailer) * system_state.x[1] + smallnumber * velocity * system_state.x[2];
            // derivative of angle of truck, Eq. 19's difference equation
            system_state.x_derivative[0] = system_state.x_derivative[1] + system_state.x_derivative[2];
        }
        else if (plant.plant_u[0] == 1){
            // derivative of angle difference between truck and trailer, Eq. 34
            system_state.x_derivative[1] = -(velocity / length_trailer) * system_state.x[1] + (velocity / length_truck) * controller.control;
            // derivative of angle of trailer
            system_state.x_derivative[2] = (velocity / length_trailer) * system_state.x[1];
            // derivative of vertical position of rear end of trailer
            system_state.x_derivative[3] = (velocity * velocity * time) / (2 * length_trailer) * system_state.x[1] + velocity * system_state.x[2];
            // derivative of angle of truck, Eq. 19's difference equation
            system_state.x_derivative[0] = system_state.x_derivative[1] + system_state.x_derivative[2];
        }
        else if (plant.plant_u[0] == 2){
            // derivative of angle of truck, Eq. 18
            system_state.x_derivative[0] = velocity / length_truck * controller.control;
            // derivative of angle of trailer, Eq. 20
            system_state.x_derivative[2] = velocity / length_trailer * sin(system_state.x[1]);
            // derivative of angle difference between truck and trailer, Eq. 19's difference equation
            system_state.x_derivative[1] = system_state.x_derivative[0] - system_state.x_derivative[2];
            // derivative of vertical position of rear end of trailer, Eq. 21
            system_state.x_derivative[3] = velocity * cos(system_state.x[1]) * sin(system_state.x[2] + (velocity * time / length_trailer * sin(system_state.x[1])) / 2);
        } break;

    // Ichihashi truck-trailer model
    case 1:
        // derivative of angle of truck, Eq. 18
        system_state.x_derivative[0] = velocity / length_truck * controller.control;
        // derivative of angle of trailer, Eq. 20
        system_state.x_derivative[2] = velocity / length_trailer * sin(system_state.x[1]);
        // derivative of angle difference between truck and trailer, Eq. 19's difference equation
        system_state.x_derivative[1] = system_state.x_derivative[0] - system_state.x_derivative[2];
        // derivative of vertical position of rear end of trailer, Eq. 21
        system_state.x_derivative[3] = velocity * cos(system_state.x[1]) * sin(system_state.x[2] + (velocity * time / length_trailer * sin(system_state.x[1])) / 2);
    }

    // derivative of horizontal position of rear end of trailer, Eq. 22
    system_state.x_derivative[4] = velocity * cos(system_state.x[1]) * cos(system_state.x[2] + (velocity * time / length_trailer * sin(system_state.x[1])) / 2);

    keypoint[0][0] = system_state.x[4] - (width_trailer / 2) * sin(system_state.x[2]); // rear left corner of trailer
    keypoint[0][1] = system_state.x[3] + (width_trailer / 2) * cos(system_state.x[2]);

    keypoint[1][0] = system_state.x[4] + width_trailer / 2 * sin(system_state.x[2]); // rear right corner of trailer
    keypoint[1][1] = system_state.x[3] - width_trailer / 2 * cos(system_state.x[2]);

    keypoint[3][0] = keypoint[0][0] + length_trailer * cos(system_state.x[2]); // left front corner of trailer
    keypoint[3][1] = keypoint[0][1] + length_trailer * sin(system_state.x[2]);

    keypoint[2][0] = keypoint[1][0] + length_trailer * cos(system_state.x[2]); // right front corner of trailer
    keypoint[2][1] = keypoint[1][1] + length_trailer * sin(system_state.x[2]);

    keypoint[4][0] = system_state.x[4] + length_trailer * cos(system_state.x[2]); // midpoint of front side of trailer
    keypoint[4][1] = system_state.x[3] + length_trailer * sin(system_state.x[2]);

    keypoint[5][0] = keypoint[4][0] + length_jackknife * cos(system_state.x[0]); // midpoint of front side of trailer
    keypoint[5][1] = keypoint[4][1] + length_jackknife * sin(system_state.x[0]);

    keypoint[6][0] = keypoint[5][0] - width_truck / 2 * sin(system_state.x[0]); // rear left corner of truck
    keypoint[6][1] = keypoint[5][1] + width_truck / 2 * cos(system_state.x[0]);

    keypoint[7][0] = keypoint[5][0] + width_truck / 2 * sin(system_state.x[0]); // rear right corner of truck
    keypoint[7][1] = keypoint[5][1] - width_truck / 2 * cos(system_state.x[0]);

    keypoint[9][0] = keypoint[6][0] + length_truck * cos(system_state.x[0]); // left front corner of truck
    keypoint[9][1] = keypoint[6][1] + length_truck * sin(system_state.x[0]);

    keypoint[8][0] = keypoint[7][0] + length_truck * cos(system_state.x[0]); // right front corner of truck
    keypoint[8][1] = keypoint[7][1] + length_truck * sin(system_state.x[0]);

    for (int j = 0; j < 10; j++){
        for (int k = 0; k < 2; k++){
            archive.point_archive[2 * j + k][i] = keypoint[j][k];
        }
    }

    for (int j = 0; j < 5; j++){
        system_state.x[j] += system_state.x_derivative[j] * Ts;
    }

    for (int j = 0; j < 5; j++){
        plant.plant_out[j] = system_state.x[j];
    }

}

void saveArchiveToTxt(double *archive, int rows, int cols, const char *filename){

    FILE *file = fopen(filename, "w+");

    if (file == NULL){
        perror("Failed to open file");
        exit(1);
    } else{
        for (int j = 0; j < rows; j++){
            for (int k = 0; k < cols; k++){
                fprintf(file, "%lf ", archive[j * cols + k]);
            }
            fprintf(file, "\n");
        }
        fclose(file);
        printf("Saved to file %s\n", filename);
    }
}

void saveArchive(){
    saveArchiveToTxt((double *)archive.x0_archive, ARRAY_SIZE, 1, "../report/x0.txt");
    saveArchiveToTxt((double *)archive.x1_archive, ARRAY_SIZE, 1, "../report/x1.txt");
    saveArchiveToTxt((double *)archive.x2_archive, ARRAY_SIZE, 1, "../report/x2.txt");
    saveArchiveToTxt((double *)archive.x3_archive, ARRAY_SIZE, 1, "../report/x3.txt");
    saveArchiveToTxt((double *)archive.x4_archive, ARRAY_SIZE, 1, "../report/x4.txt");
    saveArchiveToTxt((double *)archive.control_archive, ARRAY_SIZE, 1, "../report/control.txt");
    saveArchiveToTxt((double *)archive.point_archive, 20, ARRAY_SIZE,"../report/point.txt");
}

int main(){

    CONTROLLER_init(); // initialize controller parameter
    PLANT_init();      // initialize plant parameter

    for (int i = 0; i < ARRAY_SIZE; i++){
        double time = i * Ts + t0;
        printf("time at step %d: %f\n", i, time);
        CONTROLLER_realize(i);
        PLANT_realize(i);
    }

    saveArchive();

    return 0;
}
