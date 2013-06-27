/**
 * @file 04-gripVis.cpp
 * @author Can Erdogan
 * @date June 18, 2013
 * @brief This file demonstrates how to visualize the motion of the arms in grip.
 * NOTE Although I wanted to change this, we had to make the GRIP/wxWidget the main program
 * (the surrounding thread) and send data within a timer... This could be bad if for some reason
 * visualization halts and we want to stop the arms right then.
 */


#define protected public
#define private public

#include <imud.h>

#include "simTab.h"
#include "GRIPApp.h"

#include "helpers.h"
#include "initModules.h"
#include "motion.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, \
	&protobuf_c_system_allocator, 1024, &x, &abstime))

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

/* ********************************************************************************************* */
// Communication

somatic_d_t daemon_cx;
ach_channel_t imuChan;
ach_channel_t waistChan;		///< the state channel for the waist module
ach_channel_t leftArmChan;		///< the state channel for the left arm modules
ach_channel_t rightArmChan;		///< the state channel for the right arm modules
ach_channel_t ftLeftChan;
ach_channel_t ftRightChan;

/* ********************************************************************************************* */
// Gnuplot

FILE* gnuplot;
int windowSize = 100;		// "int" because we need to use it in subtraction
float** data;					///< windowSize x 6 matrix with each row contains force values for left/right

/* ********************************************************************************************* */
double getImu () {

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Prepare the ssdmu structure 
	ssdmu_sample_t imu_sample;
	imu_sample.x  = imu_msg->data[0];
	imu_sample.y  = imu_msg->data[1];
	imu_sample.z  = imu_msg->data[2];
	imu_sample.dP = imu_msg->data[3];
	imu_sample.dQ = imu_msg->data[4];
	imu_sample.dR = imu_msg->data[5];

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// Make the calls to extract the pitch and rate of extraction
	return -ssdmu_pitch(&imu_sample) + M_PI_2;				 
}

/* ********************************************************************************************* */
/// Gets the data from the channels
void getData (Somatic__MotorState** waist, Somatic__MotorState** leftArm, Somatic__MotorState** 
		rightArm, Somatic__ForceMoment** leftFt, Somatic__ForceMoment** rightFt, double* imu) {

	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	assert(((*waist = getMotorMessage(waistChan)) != NULL) && "Waist call failed");
	assert(((*leftArm = getMotorMessage(leftArmChan)) != NULL) && "leftArm call failed");
	assert(((*rightArm = getMotorMessage(rightArmChan)) != NULL) && "rightArm call failed");

	// Get the data from imu
	*imu = getImu ();

	// Get the data from ft sensors
	*leftFt = SOMATIC_WAIT_LAST_UNPACK( r, somatic__force_moment, 
		&protobuf_c_system_allocator, 1024, &ftLeftChan, &abstime);
	*rightFt = SOMATIC_WAIT_LAST_UNPACK( r, somatic__force_moment, 
		&protobuf_c_system_allocator, 1024, &ftRightChan, &abstime);
}

/* ********************************************************************************************* */
/// Given the sensor messages, makes changes to the kinematic model
void changeKinematics (simulation::World* world, Somatic__MotorState* waist, 
		Somatic__MotorState* leftArm, Somatic__MotorState* rightArm, double imu) {

	// Set the imu and waist positions
	vector <int> imuWaist_ids;
	imuWaist_ids.push_back(5);
	imuWaist_ids.push_back(8);
	Vector2d imuWaist_vals (imu, waist->position->data[0]);
	world->getSkeleton(0)->setConfig(imuWaist_ids, imuWaist_vals);

	// Set the indices for the left/right arms
	vector <int> left_arm_ids, right_arm_ids;
	for(size_t i = 10; i < 23; i += 2) {
		left_arm_ids.push_back(i);  
		right_arm_ids.push_back(i + 1);  
	}

	// Prepare the left/right values to set
	VectorXd left_vals (7), right_vals (7);
	for(size_t i = 0; i < 7; i++) {
		left_vals(i) = leftArm->position->data[i];
		right_vals(i) = rightArm->position->data[i];
	}

	// Set the values
	world->getSkeleton(0)->setConfig(left_arm_ids, left_vals);
	world->getSkeleton(0)->setConfig(right_arm_ids, right_vals);
}

/* ********************************************************************************************* */
void drawForces (Somatic__ForceMoment* leftArm, Somatic__ForceMoment* rightArm) {

	// Set the data
	static int k = 0;
	k++;
	for(size_t p = 0; p < 3; p++) data[p][k % windowSize] = leftArm->force->data[p];
	for(size_t p = 0; p < 3; p++) data[p+3][k % windowSize] = rightArm->force->data[p];

	// Update the range
	fprintf(gnuplot, "set xrange [%d:%d]\n", max(0, k-windowSize+1), k);

	// Plot the force values
	fprintf(gnuplot, "set term x11 0\n");
	fprintf(gnuplot, "set title 'Left Arm Force Readings'\n");
	fprintf(gnuplot, "set yrange [-10:10]\n");
	fprintf(gnuplot, "plot '-' with points title 'fx'");	
	fprintf(gnuplot, ", '-' with points title 'fy'");	
	fprintf(gnuplot, ", '-' with points title 'fz'\n");	
	for(size_t p = 0; p < 3; p++) {
		for(int i = max(0, k - windowSize + 1); i <= k; i++)
			fprintf(gnuplot, "%d %f\n", i, data[p][i % windowSize]);
		fprintf(gnuplot, "e\n");
	}

	// Plot the torque values
	fprintf(gnuplot, "set term x11 1\n");
	fprintf(gnuplot, "set title 'Right Arm Force Readings'\n");
	fprintf(gnuplot, "set yrange [-10:10]\n");
	fprintf(gnuplot, "plot '-' with points title 'fx'");	
	fprintf(gnuplot, ", '-' with points title 'fy'");	
	fprintf(gnuplot, ", '-' with points title 'fz'\n");	
	for(size_t p = 3; p < 6; p++) {
		for(int i = max(0, k - windowSize + 1); i <= k; i++)
			fprintf(gnuplot, "%d %f\n", i, data[p][i % windowSize]);
		fprintf(gnuplot, "e\n");
	}
	fflush(gnuplot);
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

	// Get the data
	double imu = 0.0;
	Somatic__MotorState* waist, *leftArm, *rightArm;
	Somatic__ForceMoment* leftFt, *rightFt;
	getData(&waist, &leftArm, &rightArm, &leftFt, &rightFt, &imu);

	// Reflect the changes in the dart kinematics and update the viewer
	changeKinematics(world, waist, leftArm, rightArm, imu);
	viewer->DrawGLScene();

	// Draw the force values
	drawForces(leftFt, rightFt);

	// Restart the timer for the next start
	Start(0.005 * 1e4);	
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// Initialize grip stuff

  sizerFull = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(0.0, 0.0, -0.7);
	viewer->UpdateCamera();
	SetSizer(sizerFull);
	frame->DoLoad("../scenes/01-World-Robot.urdf");

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->world = mWorld;
	timer->Start(1);	

	// ============================================================================
	// Initialize f/t visualization (gnuplot) stuff

	// Initialize the interface to the gnuplot setting some options
	gnuplot = popen("gnuplot -persist", "w");
	// gnuplot = fopen("bla", "w");
	fprintf(gnuplot, "set xlabel 'Time ()'\n");
	fprintf(gnuplot, "set xrange [0:100]\n");

	// Initialize the plotted data
	data = new float* [6];
	for(size_t i =0; i < 6; i++) {
		data[i] = new float [windowSize];
		for(size_t j = 0; j < windowSize; j++)
			data[i][j] = 0.0f;
	}

	// ============================================================================
	// Initialize robot stuff

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the channels to the sensors
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &leftArmChan, "llwa-state", NULL);
	somatic_d_channel_open(&daemon_cx, &rightArmChan, "rlwa-state", NULL);
	somatic_d_channel_open(&daemon_cx, &ftLeftChan, "llwa_ft", NULL);
	somatic_d_channel_open(&daemon_cx, &ftRightChan, "rlwa_ft", NULL);
	
	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	cout << "Init finished!" << endl;
}

/* ********************************************************************************************* */
SimTab::~SimTab() {
	
	// Close the pipe to gnuplot
	pclose(gnuplot);

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Destroy the channel and daemon resources
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_channel_close(&daemon_cx, &waistChan);	 
	somatic_d_channel_close(&daemon_cx, &leftArmChan); 
	somatic_d_channel_close(&daemon_cx, &rightArmChan);
	somatic_d_channel_close(&daemon_cx, &ftLeftChan);
	somatic_d_channel_close(&daemon_cx, &ftRightChan);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
	EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
END_EVENT_TABLE()

/* ********************************************************************************************* */
// Class constructor for the tab: Each tab will be a subclass of GRIPTab

IMPLEMENT_DYNAMIC_CLASS(SimTab, GRIPTab)

/* ********************************************************************************************* */
// Necessary interface call to create a GRIP executable 

/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new SimTab(tabView), wxT("Inverse Kinematics"));
	}
};

IMPLEMENT_APP(mainApp)
