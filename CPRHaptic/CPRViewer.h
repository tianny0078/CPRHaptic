#include <qglviewer.h>
#include "dhdc.h"
#if defined(WIN32) || defined(WIN64)
#include "windows.h"
#endif


#define REFRESH_INTERVAL  0.1   // sec
#define LINEAR_VISCOSITY   20.0   //  N/(m/s)

class Viewer : public QGLViewer
{
protected :
	virtual void draw();
	virtual void init();
	virtual QString helpString() const;
	virtual void keyPressEvent(QKeyEvent *e);

	int initHaptic();

	void startSimulation();
	void displayText();
	void MatTranspose (const double a[3][3],
		double       m[3][3]);

public:
	bool SimulationOn;
	bool SimulationFinished;
	double Kx, Ky, Kz;
protected:
	bool wireframe_;
public:
	~Viewer();
	static void * HapticsLoop (void* pUserData);
};
