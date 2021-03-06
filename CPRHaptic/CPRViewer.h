#include <qglviewer.h>
#include "dhdc.h"
#if defined(WIN32) || defined(WIN64)
#include "windows.h"
#endif
#include "mesh.h"
#include "kernel.h"
#include <QTimer>

#define REFRESH_INTERVAL  0.1   // sec
#define LINEAR_VISCOSITY   50.0   //  N/(m/s)

class Viewer : public QGLViewer
{
protected :
	virtual void draw();
	virtual void init();
	virtual QString helpString() const;
	virtual void keyPressEvent(QKeyEvent *e);

	int initGL();
	int initHaptic();
	int initVisual();

	void startSimulation();
	void displayText();
	void MatTranspose (const double a[3][3],
		double       m[3][3]);

	//render
	bool loadMesh();
	bool loadLevel();
	bool loadAnchor();
	bool loadConstraints();
	bool initShapeMatching();
	void setForce();
	void renderMesh(const Mesh* m);
	void renderSpiral();

	void updateDeformableMesh();
private:
	bool wireframe_;
public:
	bool HapticLoopOn;
	bool VisualLoopOn;
	bool SimulationFinished;
	double Kx, Ky, Kz;
	double F;
	double Stiffness;
	Kernel * p_kernel;

public:
	~Viewer();
	static void * VisualLoop (void* pUserData);
	static void * HapticsLoop (void* pUserData);
};
