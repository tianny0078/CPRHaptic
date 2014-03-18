#include "CPRViewer.h"

#include <QKeyEvent>

using namespace std;
// Draws a spiral
void Viewer::draw()
{
	/*
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
	*/
	renderMesh(p_kernel->p_mesh);
	displayText();
}

int Viewer::initHaptic()
{
	HapticLoopOn = true;
	SimulationFinished = false;
	Kx = 60;
	Ky = 60;
	Kz = 60;

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
	HapticLoopOn       = true;
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
	while (viewer->HapticLoopOn) {

		dhdGetPosition (&px, &py, &pz);

		//printf("%f %f %f", px, py, pz);

		// retrieve joint angles
		if (dhdGetPosition (&px, &py, &pz) < DHD_NO_ERROR) {
			printf ("error: cannot get joint angles (%s)\n", dhdErrorGetLastStr());
			viewer->HapticLoopOn = false;
		}

		// compute spring force to apply
		//if (spring)
		//{

		dhdGetLinearVelocity (&vx, &vy, &vz);
		fx = (- viewer->Kx * px) - LINEAR_VISCOSITY * vx;
		fy = (- viewer->Ky * py) - LINEAR_VISCOSITY * vy;
		fz = (- viewer->Kz * pz) - LINEAR_VISCOSITY * vz;

		if (fz > 0.0)		
			viewer->F = (-0.1) * fz;
		else
			viewer->F = 0.0;
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
			viewer->HapticLoopOn = false;
		}

		// compute jacobian
		if (dhdDeltaJointAnglesToJacobian (j0, j1, j2, J) < DHD_NO_ERROR) {
			printf ("error: cannot compute jacobian (%s)\n", dhdErrorGetLastStr());
			viewer->HapticLoopOn = false;
		}

		// compute joint torques required for gravity compensation
		if (dhdDeltaGravityJointTorques (j0, j1, j2, &g0, &g1, &g2) < DHD_NO_ERROR) {
			printf ("error: cannot compute gravity compensation joint torques (%s)\n", dhdErrorGetLastStr());
			viewer->HapticLoopOn = false;
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
			viewer->HapticLoopOn = false;
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
				viewer->HapticLoopOn = false;
			}
			if (sat == DHD_MOTOR_SATURATED) printf ("[*] ");
			else                            printf ("[-] ");
				//printf ("q = (%+0.03f, %+0.03f, %+0.03f) [Nm]  |  freq = %0.02f [kHz]       \r", q0, q1, q2, freq);
				printf ("q = (%+0.03f, %+0.03f, %+0.03f) [Nm]  |  freq = %0.02f [kHz]       \r", fx, fy, fz, freq);
			
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

// create a high priority haptic thread
#if defined(WIN32) || defined(WIN64)
	DWORD ThreadId_simulator;
	CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(VisualLoop), this, NULL, &ThreadId_simulator);
	SetThreadPriority(&ThreadId_simulator, THREAD_PRIORITY_ABOVE_NORMAL);
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
	//init haptic
	initHaptic();
	//init visual
	initVisual();
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
	qglColor(foregroundColor());
	glDisable(GL_LIGHTING);
	QFont sansFont("Times", 14, QFont::Bold);
	drawText(20,20, "Push down in the center of the chest 2 inches 30 times. Pump hard and fast at the rate of at least 100 per minute, faster than once per second.", sansFont);
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

	glEnable(GL_LIGHTING);
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
		HapticLoopOn = false;
		while (!SimulationFinished) dhdSleep (0.01);
		VisualLoopOn = false;
		exit(0);
	}
	// ... and so on with other else/if blocks.

	if (!handled)
		QGLViewer::keyPressEvent(e);
}

Viewer::~Viewer()
{
	HapticLoopOn = false;
	while (!SimulationFinished) dhdSleep (0.01);
	VisualLoopOn = false;
}

int Viewer::initVisual()
{
	//render
	F = 0.0;
	VisualLoopOn = true;
	p_kernel = new Kernel;
	if (!loadMesh())
		printf("Error! cannot load mesh!");
	if(!initShapeMatching()) 
		printf("Error! Shape Matching cannot be initialized!");
	return 1;
}

void Viewer::renderMesh(const Mesh* m)
{
	glColor3f(0.0, 162.0/255.0, 232.0/255.0);
	vector<Face>::iterator fi = p_kernel->p_mesh->face_list.begin();
	for (; fi!=p_kernel->p_mesh->face_list.end(); ++fi)
	{
		Vector3d n0, n1, n2, n01, n02, n;

		n0 = fi->node0->coordinate + fi->node0->displacement;
		n1 = fi->node1->coordinate + fi->node1->displacement;
		n2 = fi->node2->coordinate + fi->node2->displacement;

		n01 = n0 - n1;
		n02 = n0 - n2;

		n = n01.cross(n02);
		n.normalize();
		fi->normal = n;

		//display a line mesh
		//glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		glBegin(GL_TRIANGLES);
		glNormal3d(n[0], n[1], n[2]);
		glVertex3d(n0[0], n0[1], n0[2]);
		glVertex3d(n1[0], n1[1], n1[2]);
		glVertex3d(n2[0], n2[1], n2[2]);
		glEnd();
	}
}

bool Viewer::loadMesh()
{
	p_kernel->p_mesh->read("human_hand.obj");
	p_kernel->flag_mesh_ready = true;

	if (p_kernel->p_mesh->flag_normalized)
	{
		p_kernel->mark_preprocess4Voxel(p_kernel->p_mesh, p_kernel->p_voxel, p_kernel->grid_density);
	}
	else
	{
		p_kernel->p_mesh->scale();
		p_kernel->mark_preprocess4Voxel(p_kernel->p_mesh, p_kernel->p_voxel, p_kernel->grid_density);
	}
	printf("read %d nodes and %d faces", p_kernel->p_mesh->number_node, p_kernel->p_mesh->number_face);
	
	
	if (!loadLevel())
		return false;

	if (!loadAnchor())
		return false;

	if (!loadConstraints())
		return false;
	
	return true;
}

bool Viewer::loadLevel()
{
	//load level info
	p_kernel->clearAllLevel();
	ifstream ifs("level.txt");
	char line[1024];
	char * token;
	while (!ifs.eof())
	{
		ifs.getline(line, 1024);
		if (strlen(line) == 0)
			break;
		token = strtok(line, " ");
		int level = atoi(token);
		token = strtok(NULL, " ");
		int d = atoi(token);
		cout << level << " " << d << endl;
		if(level > 0)
		{
			p_kernel->addLevel();
		}
		p_kernel->generateVoxMesh4Level(level++, d);
	}

	return true;
}

bool Viewer::loadAnchor()
{
	string filename;
	if (p_kernel->level_list.size() == 1)
		filename = "anchor4naive.txt";
	else
		filename = "anchor.txt";
	ifstream ifs(filename);
	char line[1024];
	char * token;
	while (!ifs.eof())
	{
		ifs.getline(line, 1024);
		if (strlen(line) == 0)
			break;
		token = strtok(line, " ");
		int level = atoi(token);
		token = strtok(NULL, " ");
		int z = atoi(token);
		token = strtok(NULL, " ");
		int y = atoi(token);
		token = strtok(NULL, " ");
		int x = atoi(token);
		token = strtok(NULL, " ");
		int nodei = atoi(token);
		//traverse level vox, find the vox, and find corresponding node
		vector<Vox>::iterator vi = p_kernel->level_list[level]->voxmesh_level->vox_list.begin();
		for (; vi != p_kernel->level_list[level]->voxmesh_level->vox_list.end(); vi++)
		{
			if (z == vi->coord_grid(0) && y == vi->coord_grid(1) && x == vi->coord_grid(2))
			{
				Node * temp = NULL;
				if (nodei == 0)
					temp = vi->node_0;
				else if (nodei == 1)
					temp = vi->node_1;
				else if (nodei == 2)
					temp = vi->node_2;
				else if (nodei == 3)
					temp = vi->node_3;
				else if (nodei == 4)
					temp = vi->node_4;
				else if (nodei == 5)
					temp = vi->node_5;
				else if (nodei == 6)
					temp = vi->node_6;
				else if (nodei == 7)
					temp = vi->node_7;
				temp->flag_anchor_node = true;
				p_kernel->level_list[level]->voxmesh_level->anchor_node_list.push_back(temp);

				break;
			}
		}
	}
	char msg[1024];
	int cx = 0;
	for (int i = 0; i < p_kernel->level_list.size(); i++)
	{
		cx += sprintf(msg+cx, "Level %d:  %d anchor nodes have been chosen\n", i, p_kernel->level_list[i]->voxmesh_level->anchor_node_list.size());
	}
	printf(msg);

	return true;
}

bool Viewer::loadConstraints()
{
	string filename;
	if (p_kernel->level_list.size() == 1)
		filename = "constraints4naive.txt";
	else
		filename = "constraints.txt";
	ifstream ifs(filename);
	char line[1024];
	char * token;
	while (!ifs.eof())
	{
		ifs.getline(line, 1024);
		if (strlen(line) == 0)
			break;
		token = strtok(line, " ");
		int level = atoi(token);
		token = strtok(NULL, " ");
		int z = atoi(token);
		token = strtok(NULL, " ");
		int y = atoi(token);
		token = strtok(NULL, " ");
		int x = atoi(token);
		token = strtok(NULL, " ");
		int nodei = atoi(token);
		//traverse level vox, find the vox, and find corresponding node
		vector<Vox>::iterator vi = p_kernel->level_list[level]->voxmesh_level->vox_list.begin();
		for (; vi != p_kernel->level_list[level]->voxmesh_level->vox_list.end(); vi++)
		{
			if (z == vi->coord_grid(0) && y == vi->coord_grid(1) && x == vi->coord_grid(2))
			{
				Node * temp = NULL;
				if (nodei == 0)
					temp = vi->node_0;
				else if (nodei == 1)
					temp = vi->node_1;
				else if (nodei == 2)
					temp = vi->node_2;
				else if (nodei == 3)
					temp = vi->node_3;
				else if (nodei == 4)
					temp = vi->node_4;
				else if (nodei == 5)
					temp = vi->node_5;
				else if (nodei == 6)
					temp = vi->node_6;
				else if (nodei == 7)
					temp = vi->node_7;
				temp->flag_constraint_node = true;
				p_kernel->level_list[level]->voxmesh_level->constraint_node_list.push_back(temp);
				for (int k=0; k < temp->duplicates.size(); ++k)
				{
					temp->duplicates[k]->flag_constraint_node = true;
					temp->prescribed_position = temp->target_position;
					temp->prescribed_preposition = temp->target_position;
				}
				for(int j=0; j < temp->incident_cluster.size(); ++j)
				{
					temp->incident_cluster[j]->flag_constrained = true;
					temp->incident_cluster[j]->constraint_node = NULL;
					p_kernel->level_list[level]->voxmesh_level->constraint_cluster_list.push_back(temp->incident_cluster[j]);
				}
				break;
			}
		}
	}
	int l = p_kernel->level_list.size() - 1;
	for(int i = 0; i < p_kernel->level_list.size(); i ++)
	{
		int size_k = p_kernel->level_list[l]->voxmesh_level->constraint_node_list.size();
		Vector3d sum = Vector3d::Zero();
		for(int k = 0; k < size_k; k ++)
		{
			sum += p_kernel->level_list[l]->voxmesh_level->constraint_node_list[k]->prescribed_position;
		}
		p_kernel->level_list[i]->voxmesh_level->constraint_center = sum / size_k;
		p_kernel->level_list[i]->voxmesh_level->constraint_displacement.clear();
		for(int j = 0; j < p_kernel->level_list[i]->voxmesh_level->constraint_node_list.size(); j++)
		{
			Vector3d displacement = p_kernel->level_list[i]->voxmesh_level->constraint_node_list[j]->coordinate 
				+ p_kernel->level_list[i]->voxmesh_level->constraint_node_list[j]->displacement - p_kernel->level_list[i]->voxmesh_level->constraint_center;
			p_kernel->level_list[i]->voxmesh_level->constraint_displacement.push_back(displacement);
		}
	}
	p_kernel->constraint_first = p_kernel->level_list[l]->voxmesh_level->constraint_center;
	char msg[1024];
	int cx = 0;
	for (int i = 0; i < p_kernel->level_list.size(); i++)
	{
		cx += sprintf(msg+cx, "Level %d:  %d constraint nodes have been chosen\n", i, p_kernel->level_list[i]->voxmesh_level->constraint_node_list.size());
	}
	printf(msg);

	return true;
}

bool Viewer::initShapeMatching()
{
	for(int i = 0; i < p_kernel->level_list.size(); i ++)
	{
		vector<Cluster>::iterator ci;
		for( ci = p_kernel->level_list[i]->voxmesh_level->cluster_list.begin(); ci != p_kernel->level_list[i]->voxmesh_level->cluster_list.end(); ci++)
		{
			ci->kappa = 0.5;
		}
	}
	p_kernel->used_simulator = Kernel::SHAPE_MATCHING;
	p_kernel->initializeSimulator();

	return true;
}

void * Viewer::VisualLoop(void * pUserData)
{
	Viewer * viewer = (Viewer *)pUserData;
	//viewer->p_kernel->flag_redo = true;
	//Vector3d force = Vector3d(0.0, 0.0, 0.5);
	//for (int i = 0; i < 100; i++)
	//	viewer->p_kernel->force_list.push_back(force);
	//force = Vector3d(0.0, 0.0, -0.5);
	//for (int i = 0; i < 100; i++)
	//	viewer->p_kernel->force_list.push_back(force);
	while (viewer->VisualLoopOn)
	{
		viewer->setForce();
		if (!viewer->p_kernel->simulateNextStep())
		{
			viewer->VisualLoopOn = false;
		}
		viewer->update();
	}
	return NULL;
}

void Viewer::setForce()
{
	int size_k = p_kernel->level_list[0]->voxmesh_level->constraint_node_list.size();
	Vector3d f(0.0, 0.0, F);
	for(int k = 0; k < size_k; k ++)
	{
		p_kernel->level_list[0]->voxmesh_level->constraint_node_list[k]->force = f;
		for(int i=0; i<p_kernel->level_list[0]->voxmesh_level->constraint_node_list[k]->duplicates.size(); ++i)
		{
			p_kernel->level_list[0]->voxmesh_level->constraint_node_list[k]->duplicates[i]->force = f;
		}
	}
}