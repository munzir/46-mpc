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

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
ach_channel_t imu_chan;
somatic_motor_t llwa, waist;

/* ********************************************************************************************* */
double getImu () {

	// Get the value
	int r;
	struct timespec abstime = aa_tm_future( aa_tm_sec2timespec( 1.0 / 30.0 ));
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imu_chan, &abstime );
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME, 
			"Ach wait failure %s on IMU data receive (%s, line %d)\n",
			 ach_result_to_string(static_cast<ach_status_t>(r)),  __FILE__, __LINE__);

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
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

	// Move the arm to any position with the joystick
	setJoystickInput(daemon_cx, js_chan, llwa, llwa);
	somatic_motor_update(&daemon_cx, &llwa);

	// Get the imu and waist positions
	somatic_motor_update(&daemon_cx, &waist);
	double imu = getImu();
	vector <int> imuWaist_ids;
	imuWaist_ids.push_back(5);
	imuWaist_ids.push_back(8);
	cout << waist.pos[0] << endl;
	Vector2d imuWaist_vals (imu, waist.pos[0]);
	world->getSkeleton(0)->setConfig(imuWaist_ids, imuWaist_vals);

	// Update the arm position in gui with the real arm positions
	VectorXd vals (7);
	for(size_t i = 0; i < 7; i++) vals(i) = llwa.pos[i];
	vector <int> arm_ids;
	for(size_t i = 4; i < 17; i+=2) arm_ids.push_back(i + 6);  
	world->getSkeleton(0)->setConfig(arm_ids, vals);

	// Restart the timer for the next start
	viewer->DrawGLScene();
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
	// Initialize robot stuff

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	initArm(daemon_cx, llwa, "llwa");

	// Initialize the waist motors	
	somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state" );
	somatic_motor_update(&daemon_cx, &waist);
	usleep(1e5);

	// Initialize the IMU channel
	int r  = ach_open(&imu_chan, "imu-data" , NULL );
	aa_hard_assert(r == ACH_OK, "Ach failure %s on opening IMU channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Initialize the joystick channel
	r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Halt the motors
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
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
