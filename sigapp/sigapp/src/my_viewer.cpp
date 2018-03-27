
# include "my_viewer.h"

# include <sigogl/ui_button.h>
# include <sigogl/ui_radio_button.h>
# include <sig/sn_primitive.h>
# include <sig/sn_transform.h>
# include <sig/sn_manipulator.h>

# include <sigogl/ws_run.h>

MyViewer::MyViewer ( int x, int y, int w, int h, const char* l ) : WsViewer(x,y,w,h,l)
{
	_nbut=0;
	_animating=false;
	build_ui ();
	build_scene ();
}

void MyViewer::build_ui ()
{
	UiPanel *p, *sp;
	UiManager* uim = WsWindow::uim();
	p = uim->add_panel ( "", UiPanel::HorizLeft );
	p->add ( new UiButton ( "View", sp=new UiPanel() ) );
	{	UiPanel* p=sp;
		p->add ( _nbut=new UiCheckButton ( "Normals", EvNormals ) ); 
	}
	p->add ( new UiButton ( "Animate", EvAnimate ) );
	p->add ( new UiButton ( "Exit", EvExit ) ); p->top()->separate();
}


static bool rotateAboutLeftShoulder = false;
static bool rotateAboutLeftElbow = false; 
//Relative to shoulder
static GsVec translateUpperLeftArm_Sh; 
static GsVec translateLeftElbow_Sh; 
static GsVec translateLowerLeftArm_Sh; 

//Relative to Elbow
static GsVec translateLowerLeftArm_El;

static float theta = 0.0f; 

GsVec calculatDeltas(const GsMat& A, const GsMat& B) {
	//A should be part1, B should be part2, C should be part3, if applicable
	//Example: A is shoulder joint, B is elbow joint

	float x1 = A.e14;
	float x2 = B.e14;

	float y1 = A.e24;
	float y2 = B.e24;

	float z1 = A.e34;
	float z2 = B.e34;

	float dX =  x1 - x2;
	float dY =  y1 - y2;
	float dZ =	z1 - z2;

	/*if (y1 < y2) {
		dY = y1 - y2;
	}*/


	return GsVec(dX, dY, dZ);
}

void rotateAboutShoulderX(GsMat &A, GsVec &t, const float theta, const float phi) {
	GsMat T; 
	GsMat R;
	T.translation(t);
	R.rotx(theta);
	A.mult(A, T);
	A.mult(A, R);

	T.translation(-t);
	A.mult(A, T);

}

void rotateAboutShoulderY(GsMat& A, GsVec &t, const float theta, const float phi ) {

}

void rotateAboutShoulderZ(GsMat& A, GsVec &t, const float theta, const float phi) {

}

void rotateAboutElbowX(GsMat &A, GsVec &t, const float theta) {
	GsMat T; 
	GsMat R; 
	T.translation(t);
	R.rotx(theta);
	A.mult(A, T);
	A.mult(A, R);

	T.translation(-t);
	A.mult(A, T);
}

void MyViewer::add_model ( SnPrimitive* s, GsVec p, SnTransform *t )
{
	GsModel &a = *s->model(); 
	SnGroup *g = new SnGroup;
	g->separator(true);
	
	GsMat origin; 
	origin.translation(p);
	a.transform(origin, true);

	g->add(t);
	g->add(s);
	
	rootg()->add(g);
}
void MyViewer::buildTorso() {
	SnPrimitive* p;

	p = new SnPrimitive(GsPrimitive::Cylinder, 0.5f, 0.6f, 0.5f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), torso_T = new SnTransform);

	GsMat &t = torso_T->get();
	GsMat R;
	GsMat T;

	R.rotx(GS_TORAD(0.0f));
	T.translation(GsVec(0, 0, 0));

	t.mult(t, R);
	t.mult(t, T);
}
void MyViewer::buildNeck() {
	SnPrimitive *p;

	p = new SnPrimitive(GsPrimitive::Cylinder, 0.2f, 0.2f, 0.1f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), neck_T = new SnTransform);

	GsMat &t = neck_T->get();
	GsMat T;

	T.translation(GsVec(0.0f, 0.6f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildHead() {
	SnPrimitive *p; 
	p = new SnPrimitive(GsPrimitive::Capsule, 0.23f, 0.25f, 0.1f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), head_T = new SnTransform);

	GsMat &t = head_T->get();
	GsMat T;

	T.translation(GsVec(0.0f, 1.0f, 0.0f));
	t.mult(t, T);

}
void MyViewer::buildPelvis() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.4f, 0.5f, 0.2f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), pelvis_T = new SnTransform);
	GsMat &t = pelvis_T->get();
	GsMat T;

	T.translation(GsVec(0.0f, -0.7f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildLeftShoulderJoint() {
	SnPrimitive* p; 
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftShoulderJoint_T = new SnTransform);

	GsMat &t = leftShoulderJoint_T->get();
	GsMat T;

	T.translation(GsVec(-0.7f, 0.4f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightShoulderJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightShoulderJoint_T = new SnTransform);

	GsMat &t = rightShoulderJoint_T->get();
	GsMat T;

	T.translation(GsVec(0.7f, 0.4f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildLeftUpperArm() {
	SnPrimitive *p; 
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.15f, 0.15f, 0.25f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftUpperArm_T = new SnTransform);

	GsMat &t = leftUpperArm_T->get();
	GsMat T;

	T.translation(GsVec(-0.7f, 0.0f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightUpperArm() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.15f, 0.15f, 0.25f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightUpperArm_T = new SnTransform);

	GsMat &t = rightUpperArm_T->get();
	GsMat T;

	T.translation(GsVec(0.7f, 0.0f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildLeftElbowJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftElbowJoint_T = new SnTransform);

	GsMat &t = leftElbowJoint_T->get();
	GsMat T;

	T.translation(GsVec(-0.7f, -0.4f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightElbowJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightElbowJoint_T = new SnTransform);

	GsMat &t = rightElbowJoint_T->get();
	GsMat T;

	T.translation(GsVec(0.7f, -0.4f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildLeftLowerArm() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.10f, 0.15f, 0.25f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftLowerArm_T = new SnTransform);

	GsMat &t = leftLowerArm_T->get();
	GsMat T;

	T.translation(GsVec(-0.7f, -0.8f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightLowerArm() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.10f, 0.15f, 0.25f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightLowerArm_T = new SnTransform);


	GsMat &t = rightLowerArm_T->get();
	GsMat T;

	T.translation(GsVec(0.7f, -0.8f, 0.0f));
	t.mult(t, T);

}
void MyViewer::buildLeftLegJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftLegJoint_T = new SnTransform);


	GsMat &t = leftLegJoint_T->get();
	GsMat T;

	T.translation(GsVec(-0.5f, -1.0f, 0.0f));
	t.mult(t, T);
	
}
void MyViewer::buildRightLegJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightLegJoint_T = new SnTransform);

	GsMat &t = rightLegJoint_T->get();
	GsMat T;

	T.translation(GsVec(0.5f, -1.0f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildLeftUpperLeg() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.15f, 0.15f, 0.3f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftUpperLeg_T = new SnTransform);

	GsMat &t = leftUpperLeg_T->get();
	GsMat T;

	T.translation(GsVec(-0.5f, -1.4f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightUpperLeg() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.15f, 0.15f, 0.3f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightUpperLeg_T = new SnTransform);

	GsMat &t = rightUpperLeg_T->get();
	GsMat T;

	T.translation(GsVec(0.5f, -1.4f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildLeftKneeJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftKneeJoint_T = new SnTransform);

	GsMat &t = leftKneeJoint_T->get();
	GsMat T;

	T.translation(GsVec(-0.5f, -1.8f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightKneeJoint() {
	SnPrimitive* p;
	p = new SnPrimitive(GsPrimitive::Capsule, 0.15f, 0.15f, 0.01f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightKneeJoint_T = new SnTransform);

	GsMat &t = rightKneeJoint_T->get();
	GsMat T;

	T.translation(GsVec(0.5f, -1.8f, 0.0f));
	t.mult(t, T);

}
void MyViewer::buildLeftLowerLeg() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.10f, 0.15f, 0.3f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), leftLowerLeg_T = new SnTransform);

	GsMat &t = leftLowerLeg_T->get();
	GsMat T;

	T.translation(GsVec(-0.5f, -2.2f, 0.0f));
	t.mult(t, T);
}
void MyViewer::buildRightLowerLeg() {
	SnPrimitive *p;
	p = new SnPrimitive(GsPrimitive::Cylinder, 0.10f, 0.15f, 0.3f);
	p->prim().material.diffuse = GsColor::darkblue;
	add_model(p, GsVec(0, 0, 0), rightLowerLeg_T = new SnTransform);

	GsMat &t = rightLowerLeg_T->get();
	GsMat T;

	T.translation(GsVec(0.5f, -2.2f, 0.0f));
	t.mult(t, T);
}
void MyViewer::build_scene ()
{
	buildTorso();//Index 0
	buildNeck();//Index 1
	buildHead();//Index 2
	buildPelvis();//Index 3
	buildLeftShoulderJoint();//Index 4
	buildRightShoulderJoint();//Index 5
	buildLeftUpperArm();//Index 6
	buildRightUpperArm();//Index 7
	buildLeftElbowJoint();//Index 8
	buildRightElbowJoint();//Index 9
	buildLeftLowerArm();//Index 10
	buildRightLowerArm();
	buildLeftLegJoint();
	buildRightLegJoint();
	buildLeftUpperLeg();
	buildRightUpperLeg();
	buildLeftKneeJoint();
	buildRightKneeJoint();
	buildLeftLowerLeg();
	buildRightLowerLeg();

	translateUpperLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftUpperArm_T->get());
	translateLeftElbow_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftElbowJoint_T->get());
	translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());

	translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());
}

// Below is an example of how to control the main loop of an animation:
void MyViewer::run_animation ()
{
	if ( _animating ) return; // avoid recursive calls
	_animating = true;
	
	int ind = gs_random ( 0, rootg()->size()-1 ); // pick one child
	SnManipulator* manip = rootg()->get<SnManipulator>(ind); // access one of the manipulators
	GsMat m = manip->mat();

	double frdt = 1.0/30.0; // delta time to reach given number of frames per second
	double v = 4; // target velocity is 1 unit per second
	double t=0, lt=0, t0=gs_time();
	do // run for a while:
	{	while ( t-lt<frdt ) { ws_check(); t=gs_time()-t0; } // wait until it is time for next frame
		double yinc = (t-lt)*v;
		if ( t>2 ) yinc=-yinc; // after 2 secs: go down
		lt = t;
		m.e24 += (float)yinc;
		if ( m.e24<0 ) m.e24=0; // make sure it does not go below 0
		manip->initial_mat ( m );
		render(); // notify it needs redraw
		ws_check(); // redraw now
	}	while ( m.e24>0 );
	_animating = false;
}
//Left Shoulder Joint
void MyViewer::negRotXAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();
	GsMat R;
	GsMat T;


	rotateAboutShoulderX(leftUpperArmMat, translateUpperLeftArm_Sh, -6.0f, theta);
	rotateAboutShoulderX(leftElbowRotMat, translateLeftElbow_Sh, -6.0f, theta);
	rotateAboutShoulderX(leftLowerArmMat, translateLowerLeftArm_Sh, -6.0f, theta);


	render(); // notify it needs redraw
	ws_check(); // redraw now		
}
void MyViewer::posRotXAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();
	GsMat R;
	GsMat T;

	
	rotateAboutShoulderX(leftUpperArmMat, translateUpperLeftArm_Sh, 6.0f, theta);
	rotateAboutShoulderX(leftElbowRotMat, translateLeftElbow_Sh, 6.0f, theta);
	rotateAboutShoulderX(leftLowerArmMat, translateLowerLeftArm_Sh, 6.0f, theta);


	render(); // notify it needs redraw
	ws_check(); // redraw now		
}
void MyViewer::negRotYAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);
	R.roty(GS_TORAD(-6.0f));
	leftUpperArmMat.mult(leftUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	leftElbowRotMat.mult(leftElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);
	leftLowerArmMat.mult(leftLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now		
}
void MyViewer::posRotYAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);
	R.roty(GS_TORAD(6.0f));
	leftUpperArmMat.mult(leftUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	leftElbowRotMat.mult(leftElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);
	leftLowerArmMat.mult(leftLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now		

}
void MyViewer::posRotZAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);
	R.rotz(GS_TORAD(6.0f));
	leftUpperArmMat.mult(leftUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	leftElbowRotMat.mult(leftElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);
	leftLowerArmMat.mult(leftLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now		
}
void MyViewer::negRotZAboutLSJ() {

	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);
	R.rotz(GS_TORAD(-6.0f));
	leftUpperArmMat.mult(leftUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	leftUpperArmMat.mult(leftUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	leftElbowRotMat.mult(leftElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	leftElbowRotMat.mult(leftElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);
	leftLowerArmMat.mult(leftLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	leftLowerArmMat.mult(leftLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now		

}

//Right Shoulder Joint
void MyViewer::negRotXAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);
	R.rotx(GS_TORAD(-6.0f));
	rightUpperArmMat.mult(rightUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	rightElbowRotMat.mult(rightElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	rightLowerArmMat.mult(rightLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now	
}
void MyViewer::posRotXAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);
	R.rotx(GS_TORAD(6.0f));
	rightUpperArmMat.mult(rightUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	rightElbowRotMat.mult(rightElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	rightLowerArmMat.mult(rightLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now	

}
void MyViewer::negRotYAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);
	R.roty(GS_TORAD(-6.0f));
	rightUpperArmMat.mult(rightUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	rightElbowRotMat.mult(rightElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	rightLowerArmMat.mult(rightLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now	
}
void MyViewer::posRotYAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);
	R.roty(GS_TORAD(6.0f));
	rightUpperArmMat.mult(rightUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	rightElbowRotMat.mult(rightElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	rightLowerArmMat.mult(rightLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now	
}
void MyViewer::posRotZAboutRSJ() {

	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);
	R.rotz(GS_TORAD(6.0f));
	rightUpperArmMat.mult(rightUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	rightElbowRotMat.mult(rightElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	rightLowerArmMat.mult(rightLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	

	render(); // notify it needs redraw
	ws_check(); // redraw now		

}
void MyViewer::negRotZAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	T.translation(GsVec(0.0f, 0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);
	R.rotz(GS_TORAD(-6.0f));
	rightUpperArmMat.mult(rightUpperArmMat, R);

	T.translation(GsVec(0.0f, -0.4f, 0.0f));
	rightUpperArmMat.mult(rightUpperArmMat, T);

	T.translation(GsVec(0.0f, 0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	rightElbowRotMat.mult(rightElbowRotMat, R);

	T.translation(GsVec(0.0f, -0.8f, 0.0f));
	rightElbowRotMat.mult(rightElbowRotMat, T);

	T.translation(GsVec(0.0f, 1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);
	rightLowerArmMat.mult(rightLowerArmMat, R);
	T.translation(GsVec(0.0f, -1.2f, 0.0f));
	rightLowerArmMat.mult(rightLowerArmMat, T);


	render(); // notify it needs redraw
	ws_check(); // redraw now		
}

//Left Elbow Joint

void MyViewer::negRotXAboutLEJ() {
	GsMat &leftShoulder = leftShoulderJoint_T->get();
	GsMat &leftElbowJoint = leftElbowJoint_T->get();
	GsMat &lowerLeftArm = leftLowerArm_T->get();
	GsMat T; 
	GsMat R;
	
	//GsVec translateElbow = calculatDeltas(leftShoulder, leftElbowJoint);
	//GsVec translateLowerArm = calculatDeltas(leftElbowJoint, lowerLeftArm);

	//R.rotx(GS_TORAD(-6.0f));

	//theta += 6.0f; 
	//T.translation(translateLowerLeftArm_El);//This time, it's negative becuase the lower arm is in the negative y-axis!!!!
	//lowerLeftArm.mult(lowerLeftArm, T);

	//lowerLeftArm.mult(lowerLeftArm, R);

	//T.translation(-translateLowerLeftArm_El);

	//lowerLeftArm.mult(lowerLeftArm, T);
	theta += 6.0f; 

	rotateAboutElbowX(lowerLeftArm, translateLowerLeftArm_El, -6.0f);

	render();
	ws_check();
}
void MyViewer::posRotXAboutLEJ() {
	GsMat &leftShoulder = leftShoulderJoint_T->get();
	GsMat &leftElbowJoint = leftElbowJoint_T->get();
	GsMat &lowerLeftArm = leftLowerArm_T->get();
	GsMat T;
	GsMat R;

	//GsVec translateLowerArm = calculatDeltas(leftElbowJoint, lowerLeftArm);

	//R.rotx(GS_TORAD(6.0f));
	//theta -= 6.0f; 
	//T.translation(translateLowerLeftArm_El);//This time, it's negative becuase the lower arm is in the negative y-axis!!!!
	//lowerLeftArm.mult(lowerLeftArm, T);

	//lowerLeftArm.mult(lowerLeftArm, R);

	//T.translation(-translateLowerLeftArm_El);

	//lowerLeftArm.mult(lowerLeftArm, T);
	theta -= 6.0f;
	rotateAboutElbowX(lowerLeftArm, translateLowerLeftArm_El, 6.0f);

	render();
	ws_check();

}
void MyViewer::negRotYAboutLEJ() {

}
void MyViewer::posRotYAboutLEJ() {

}
void MyViewer::negRotZAboutLEJ() {

}
void MyViewer::posRotZAboutLEJ() {

}

void MyViewer::hello_animation() {
	if (_animating) return; // avoid recursive calls
	_animating = true;

	double frdt = 1.0 / 30.0; // delta time to reach given number of frames per second
	double v = 4; // target velocity is 1 unit per second
	double t = 0, lt = 0, t0 = gs_time();

	double start = 0.0;
	double seconds = 0.0;
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get(); 
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R; 
	GsMat T;
	int theta = 0;
	
	do // run for a while:
	{
		while (t - lt<frdt) { ws_check(); t = gs_time() - t0; } // wait until it is time for next frame
		
		double end = t; 

		double elapsed_seconds = end - start; 
		//float x = 0.01f; 
		//
		//float y1 = parabolicCurve(x, 0.001f, 0.4f); //right shoulder
		//float y2 = parabolicCurve(x, 0.1f, 0.001f); //right arm
		//float y3 = parabolicCurve(x, 0.01f, -0.4f);//right elbow
		/*float x = 0.05f * cosf(GS_TORAD(theta));
		float y = 0.05f * sinf(GS_TORAD(theta));*/

		if (elapsed_seconds >= 1.0) {
			seconds += 1.0; 

			T.translation(GsVec(0.0f, 0.4f, 0.0f));
			rightUpperArmMat.mult(rightUpperArmMat, T); 
			R.rotz(GS_TORAD(6.0f));	
			rightUpperArmMat.mult(rightUpperArmMat, R);
			
			T.translation(GsVec(0.0f, -0.4f, 0.0f));
			rightUpperArmMat.mult(rightUpperArmMat, T);
			
			T.translation(GsVec(0.0f, 0.8f, 0.0f));
			rightElbowRotMat.mult(rightElbowRotMat, T);

			rightElbowRotMat.mult(rightElbowRotMat, R);

			T.translation(GsVec(0.0f, -0.8f, 0.0f));
			rightElbowRotMat.mult(rightElbowRotMat, T);

			T.translation(GsVec(0.0f, 1.2f, 0.0f));
			rightLowerArmMat.mult(rightLowerArmMat, T);
			rightLowerArmMat.mult(rightLowerArmMat, R);
			T.translation(GsVec(0.0f, -1.2f, 0.0f));
			rightLowerArmMat.mult(rightLowerArmMat, T);
		
			start = end;
		}


		lt = t;

		
		

		render(); // notify it needs redraw
		ws_check(); // redraw now

	} while (true);
	_animating = false;
}

void MyViewer::show_normals ( bool b )
{
	// Note that primitives are only converted to meshes in GsModel
	// at the first draw call.
	GsArray<GsVec> fn;
	SnGroup* r = (SnGroup*)root();
	for ( int k=0; k<r->size(); k++ )
	{	SnManipulator* manip = r->get<SnManipulator>(k);
		SnShape* s = manip->child<SnGroup>()->get<SnShape>(0);
		SnLines* l = manip->child<SnGroup>()->get<SnLines>(1);
		if ( !b ) { l->visible(false); continue; }
		l->visible ( true );
		if ( !l->empty() ) continue; // build only once
		l->init();
		if ( s->instance_name()==SnPrimitive::class_name )
		{	GsModel& m = *((SnModel*)s)->model();
			m.get_normals_per_face ( fn );
			const GsVec* n = fn.pt();
			float f = 0.33f;
			for ( int i=0; i<m.F.size(); i++ )
			{	const GsVec& a=m.V[m.F[i].a]; l->push ( a, a+(*n++)*f );
				const GsVec& b=m.V[m.F[i].b]; l->push ( b, b+(*n++)*f );
				const GsVec& c=m.V[m.F[i].c]; l->push ( c, c+(*n++)*f );
			}
		}  
	}
}

int MyViewer::handle_keyboard ( const GsEvent &e )
{
	int ret = WsViewer::handle_keyboard ( e ); // 1st let system check events
	if ( ret ) return ret;

	switch ( e.key )
	{	case GsEvent::KeyEsc : gs_exit(); return 1;
		case 'h': {
			
			hello_animation();
			return 1; 
		}
		case 'i':{
			posRotZAboutRSJ();
			return 1; 

		}
		case 'u': {
			negRotZAboutRSJ();
			return  1; 
		}
		case 'q': {

			if (rotateAboutLeftElbow == false && rotateAboutLeftShoulder == false) {
				rotateAboutLeftShoulder = true;
				negRotZAboutLSJ();

			}
			else if (rotateAboutLeftShoulder && rotateAboutLeftElbow == false) {
				negRotZAboutLSJ();
			}
			else if (rotateAboutLeftElbow && rotateAboutLeftShoulder == false) {
				rotateAboutLeftElbow = false;
				rotateAboutLeftShoulder = true; 
				/*translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());*/

				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			}
			//translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());

			return 1; 
		}
		case 'w': {
			posRotZAboutLSJ();
			return 1; 
		}
		case 'j': {
			negRotXAboutRSJ();
			return 1; 
		}
		case 'k': {
			posRotXAboutRSJ();
			return 1; 
		}
		case 'a': {

			if (rotateAboutLeftElbow == false && rotateAboutLeftShoulder == false) {
				rotateAboutLeftShoulder = true;
				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
				negRotXAboutLSJ();
				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);

				ws_check();
				render();
				//translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());
			}
			else if (rotateAboutLeftShoulder && rotateAboutLeftElbow == false) {
				
				//translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());
				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
				negRotXAboutLSJ();
				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);

				ws_check();
				render();
				//translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());
			}
			else if (rotateAboutLeftElbow && rotateAboutLeftShoulder == false) {
				rotateAboutLeftElbow = false;

				rotateAboutLeftShoulder = true;
				

				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
				negRotXAboutLSJ();
				rotateAboutElbowX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);
				/*GsMat &lowerLeftArm = leftLowerArm_T->get();
				GsMat T;
				GsMat R; 
				T.translation(translateLowerLeftArm_El);
				R.rotx(theta);
				
				lowerLeftArm.mult(lowerLeftArm, T);
				lowerLeftArm.mult(lowerLeftArm, R);

				T.translation(-translateLowerLeftArm_El);
				lowerLeftArm.mult(lowerLeftArm, T);*/

				ws_check();
				render();
				//translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());
				//negRotXAboutLSJ();
				/*translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());*/
			}
			
			return 1; 
		}
		case 's': {
			posRotXAboutLSJ();
			return 1; 
		}
		case 'z': {
			negRotYAboutLSJ();
			return 1; 
		}
		case 'x': {
			posRotYAboutLSJ();
			return 1; 
		}
		case 'm': {
			negRotYAboutRSJ();
			return 1; 
		}
		case ',': {
			posRotYAboutRSJ();
			return 1; 
		}
		case 'e': {
			if (rotateAboutLeftShoulder == false && rotateAboutLeftElbow == false) {
				rotateAboutLeftElbow = true;
				
				negRotXAboutLEJ();
				/*translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());*/

			}
			else if (rotateAboutLeftElbow == true && rotateAboutLeftShoulder == false) {
				
				negRotXAboutLEJ();
				/*translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());*/
			}
			else if (rotateAboutLeftShoulder && rotateAboutLeftElbow == false) {
				rotateAboutLeftElbow = true;
				rotateAboutLeftShoulder = false;

				//translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());
				
				negRotXAboutLEJ();
				/*translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());*/
			}

			//translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());
			
			return 1; 
		}
		case 'r': {
			posRotXAboutLEJ();
			return 1; 
		}
		/*case 'n' : { bool b=!_nbut->value(); _nbut->value(b); show_normals(b); return 1; }*/
		default: gsout<<"Key pressed: "<<e.key<<gsnl;
	}

	return 0;
}

int MyViewer::uievent ( int e )
{
	switch ( e )
	{	case EvNormals: show_normals(_nbut->value()); return 1;
		case EvAnimate: run_animation(); return 1;
		case EvExit: gs_exit();
	}
	return WsViewer::uievent(e);
}
