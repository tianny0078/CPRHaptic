#include "CPRViewer.h"

#include <QKeyEvent>

using namespace std;
// Draws a spiral
void Viewer::draw()
{
	const float nbSteps = 200.0;

	glBegin(GL_QUAD_STRIP);
	for (int i=0; i<nbSteps; ++i)
	{
		const float ratio = i/nbSteps;
		const float angle = 21.0*ratio;
		const float c = cos(angle);
		const float s = sin(angle);
		const float r1 = 1.0 - 0.8f*ratio;
		const float r2 = 0.8f - 0.8f*ratio;
		const float alt = ratio - 0.5f;
		const float nor = 0.5f;
		const float up = sqrt(1.0-nor*nor);
		glColor3f(1.0-ratio, 0.2f , ratio);
		glNormal3f(nor*c, up, nor*s);
		glVertex3f(r1*c, alt, r1*s);
		glVertex3f(r2*c, alt+0.05f, r2*s);
	}
	glEnd();

	
}

int Viewer::initHaptic()
{

	// required to change asynchronous operation mode
	dhdEnableExpertMode ();

	if (dhdOpen () >= 0) {

		// default config: 1 finger, no wrist, no gripper
		switch (dhdGetSystemType ()) {
		case DHD_DEVICE_OMEGA3:
			printf ("%s device detected\n", dhdGetSystemName());
			break;
		}
	}

	else {
		printf ("no device detected\n");
		dhdSleep (2.0);
		exit (0);
	}

	printf ("\n");

	return 0;
}

void Viewer::MatTranspose (const double a[3][3],
	double       m[3][3])
{
	m[0][0] = a[0][0];  m[0][1] = a[1][0];  m[0][2] = a[2][0];
	m[1][0] = a[0][1];  m[1][1] = a[1][1];  m[1][2] = a[2][1];
	m[2][0] = a[0][2];  m[2][1] = a[1][2];  m[2][2] = a[2][2];
}

// haptic thread
void*
	 Viewer::HapticsLoop (void* pUserData)
{
	Viewer * viewer = (Viewer *)pUserData;
	/*
	int       i;
	cVector3d newFingerPosGlobal;
	cVector3d newFingerPosLocal;
	cVector3d forceLocal;
	cVector3d forceGlobal[2];

	// start haptic simulation
	SimulationOn       = true;
	SimulationFinished = false;

	// start with no force
	forceGlobal[0].zero ();
	forceGlobal[1].zero ();

	*/
	double px, py, pz;
	double vx, vy, vz;
	double fx, fy, fz;
	double j0, j1, j2;
	double g0, g1, g2;
	double q0, q1, q2;
	double J[3][3];
	double Jt[3][3];
	double t1,t0  = dhdGetTime ();
	double freq   = 0.0;
	int sat;
	bool spring = false;
	// enable force
	dhdEnableForce (DHD_ON);
	// main haptic simulation loop
	while (viewer->SimulationOn) {

		dhdGetPosition (&px, &py, &pz);

		//printf("%f %f %f", px, py, pz);

		// retrieve joint angles
		if (dhdGetPosition (&px, &py, &pz) < DHD_NO_ERROR) {
			printf ("error: cannot get joint angles (%s)\n", dhdErrorGetLastStr());
			viewer->SimulationOn = false;
		}

		// compute spring force to apply
		//if (spring)
		//{

		dhdGetLinearVelocity (&vx, &vy, &vz);
		fx = (- viewer->Kx * px) - LINEAR_VISCOSITY * vx;
		fy = (- viewer->Ky * py) - LINEAR_VISCOSITY * vy;
		fz = (- viewer->Kz * pz) - LINEAR_VISCOSITY * vz;
		//}
		//else
		//{
		//	fx = 0;
		//	fy = 0;
		//	fz = 0;
		//}
		
		// retrieve joint angles
		if (dhdGetDeltaJointAngles (&j0, &j1, &j2) < DHD_NO_ERROR) {
			printf ("error: cannot get joint angles (%s)\n", dhdErrorGetLastStr());
			viewer->SimulationOn = false;
		}

		// compute jacobian
		if (dhdDeltaJointAnglesToJacobian (j0, j1, j2, J) < DHD_NO_ERROR) {
			printf ("error: cannot compute jacobian (%s)\n", dhdErrorGetLastStr());
			viewer->SimulationOn = false;
		}

		// compute joint torques required for gravity compensation
		if (dhdDeltaGravityJointTorques (j0, j1, j2, &g0, &g1, &g2) < DHD_NO_ERROR) {
			printf ("error: cannot compute gravity compensation joint torques (%s)\n", dhdErrorGetLastStr());
			viewer->SimulationOn = false;
		}

		// compute joint torques Q = ((J)T) * F
		viewer->MatTranspose (J, Jt);
		q0 = Jt[0][0]*fx + Jt[0][1]*fy + Jt[0][2]*fz;
		q1 = Jt[1][0]*fx + Jt[1][1]*fy + Jt[1][2]*fz;
		q2 = Jt[2][0]*fx + Jt[2][1]*fy + Jt[2][2]*fz;

		// combine gravity compensation and requested force
		q0 += g0;
		q1 += g1;
		q2 += g2;

		// apply joint torques
		if ((sat = dhdSetDeltaJointTorques (q0, q1, q2)) < DHD_NO_ERROR) {
			printf ("error: cannot set joint torques (%s)\n", dhdErrorGetLastStr());
			viewer->SimulationOn = false;
		}

		// display refresh rate and position at 10Hz
		t1 = dhdGetTime ();
		//printf("%f", (t1-t0));
		if ((t1-t0) > REFRESH_INTERVAL) {

			// retrieve information to display
			freq = dhdGetComFreq ();
			t0   = t1;

			// write down position
			if (dhdGetPosition (&px, &py, &pz) < 0) {
				printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
				viewer->SimulationOn = false;
			}
			if (sat == DHD_MOTOR_SATURATED) printf ("[*] ");
			else                            printf ("[-] ");
			printf ("q = (%+0.03f, %+0.03f, %+0.03f) [Nm]  |  freq = %0.02f [kHz]       \r", q0, q1, q2, freq);
			
			//test for exit condition
			//if (dhdGetButtonMask()) spring = true;
			//else                    spring = false;
			//if (dhdKbHit()) {
			//	switch (dhdKbGet()) {
			//	case 'v': K += 10; printf("%f", K); break;
			//	case 's': dhdSetComMode (DHD_COM_MODE_SYNC);  break;
			//	case 'a': dhdSetComMode (DHD_COM_MODE_ASYNC); break;
			//	}
			//}
		}

	}

	// close connection with haptic device
	dhdClose ();

	// simulation is now exiting
	viewer->SimulationFinished = true;

	// return
	return NULL;
}

void Viewer::startSimulation()
{
	// create a high priority haptic thread
#if defined(WIN32) || defined(WIN64)
	DWORD ThreadId;
	CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(HapticsLoop), this, NULL, &ThreadId);
	SetThreadPriority(&ThreadId, THREAD_PRIORITY_ABOVE_NORMAL);
#else
	pthread_t handle;
	pthread_create (&handle, NULL, HapticsLoop, NULL);
	struct sched_param sp;
	memset (&sp, 0, sizeof(struct sched_param));
	sp.sched_priority = 10;
	pthread_setschedparam (handle, SCHED_RR, &sp);
#endif
}

void Viewer::init()
{
	SimulationOn = true;
	SimulationFinished = false;
	Kx = 60;
	Ky = 60;
	Kz = 60;
	//init haptic
	initHaptic();
	startSimulation();
	// Restore previous viewer state.
	restoreStateFromFile();

	// Opens help window
	//help();
}

QString Viewer::helpString() const
{
	QString text("<h2>S i m p l e V i e w e r</h2>");
	text += "Use the mouse to move the camera around the object. ";
	text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
	text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
	text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
	text += "Simply press the function key again to restore it. Several keyFrames define a ";
	text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
	text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
	text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
	text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
	text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
	text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
	text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
	text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
	text += "Press <b>Escape</b> to exit the viewer.";
	return text;
}

void Viewer::displayText()
{
	//qglColor(foregroundColor());
	//glDisable(GL_LIGHTING);
	//drawText(10,height()-30, "TRANSLATION :");
	//displayDir(transDir, 190, height()-30, 'G');
	//displayType(constraints[activeConstraint]->translationConstraintType(), 10, height()-60, 'T');

	//drawText(width()-220,height()-30, "ROTATION :");
	//displayDir(rotDir, width()-100, height()-30, 'D');
	//displayType(constraints[activeConstraint]->rotationConstraintType(), width()-220, height()-60, 'R');

	//switch (activeConstraint)
	//{
	//case 0 : drawText(20,20, "Constraint direction defined w/r to WORLD (SPACE)"); break;
	//case 1 : drawText(20,20, "Constraint direction defined w/r to CAMERA (SPACE)"); break;
	//}

	//glEnable(GL_LIGHTING);
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
	// Get event modifiers key
	const Qt::KeyboardModifiers modifiers = e->modifiers();

	// A simple switch on e->key() is not sufficient if we want to take state key into account.
	// With a switch, it would have been impossible to separate 'F' from 'CTRL+F'.
	// That's why we use imbricated if...else and a "handled" boolean.
	bool handled = false;
	if ((e->key()==Qt::Key_W) && (modifiers==Qt::NoButton))
	{
		wireframe_ = !wireframe_;
		if (wireframe_)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		handled = true;
		updateGL();
	}
	else if ((e->key()==Qt::Key_S) && (modifiers==Qt::NoButton))
	{
		//flatShading_ = !flatShading_;
		//if (flatShading_)
		//	glShadeModel(GL_FLAT);
		//else
		//	glShadeModel(GL_SMOOTH);
		//handled = true;
		//updateGL();
		saveStateToFile();
		handled = true;
	}
	else if ((e->key()==Qt::Key_B) && (modifiers==Qt::NoButton))
	{
		 Kx = 1000;
		 Ky = 1000;
		 Kz = 300;
		 handled = true;
	}	
	else if ((e->key()==Qt::Key_Q) && (modifiers==Qt::NoButton))
	{
		SimulationOn = false;
		while (!SimulationFinished) dhdSleep (0.01);
		exit(0);
	}
	// ... and so on with other else/if blocks.

	if (!handled)
		QGLViewer::keyPressEvent(e);
}

Viewer::~Viewer()
{
	SimulationOn = false;
	while (!SimulationFinished) dhdSleep (0.01);
}
