
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

static GsVec translateUpperRightArm_Sh;
static GsVec translateRightElbow_Sh;
static GsVec translateLowerRightArm_Sh;

//Relative to Elbow
static GsVec translateLowerLeftArm_El;
static GsVec translateLowerRightArm_El;

//Relative to Leg Joint
static GsVec translateUpperLeftLeg_LJ;
static GsVec translateLeftKnee_LJ;
static GsVec translateLowerLeftLeg_LJ;

static GsVec translateUpperRightLeg_LJ;
static GsVec translateRightKnee_LJ; 
static GsVec translateLowerRightLeg_LJ; 

//Relative to Knee

static GsVec translateLowerLeftLeg_KJ;
static GsVec translateLowerRightLeg_KJ;

static float theta = 0.0f; 
static float phi = 0.0f; 
static float alpha = 0.0f; 
static float beta = 0.0f; 

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

	return GsVec(dX, dY, dZ);
}

void rotateAboutJointX(GsMat &A, GsVec &t, const float theta) {
	GsMat T; 
	GsMat R;
	T.translation(t);
	R.rotx(theta);
	A.mult(A, T);
	A.mult(A, R);

	T.translation(-t);
	A.mult(A, T);

}

void rotateAboutJointY(GsMat& A, GsVec &t, const float theta) {
	GsMat T;
	GsMat R;
	T.translation(t);
	R.roty(theta);
	A.mult(A, T);
	A.mult(A, R);

	T.translation(-t);
	A.mult(A, T);
}

void rotateAboutJointZ(GsMat& A, GsVec &t, const float theta) {
	GsMat T;
	GsMat R;
	T.translation(t);
	R.rotz(theta);
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
	buildTorso();
	buildNeck();
	buildHead();
	buildPelvis();
	buildLeftShoulderJoint();
	buildRightShoulderJoint();
	buildLeftUpperArm();
	buildRightUpperArm();
	buildLeftElbowJoint();
	buildRightElbowJoint();
	buildLeftLowerArm();
	buildRightLowerArm();
	buildLeftLegJoint();
	buildRightLegJoint();
	buildLeftUpperLeg();
	buildRightUpperLeg();
	buildLeftKneeJoint();
	buildRightKneeJoint();
	buildLeftLowerLeg();
	buildRightLowerLeg();


	//Arms
	translateUpperLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftUpperArm_T->get());
	translateLeftElbow_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftElbowJoint_T->get());
	translateLowerLeftArm_Sh = calculatDeltas(leftShoulderJoint_T->get(), leftLowerArm_T->get());

	translateUpperRightArm_Sh = calculatDeltas(rightShoulderJoint_T->get(), rightUpperArm_T->get());
	translateRightElbow_Sh = calculatDeltas(rightShoulderJoint_T->get(), rightElbowJoint_T->get());
	translateLowerRightArm_Sh = calculatDeltas(rightShoulderJoint_T->get(), rightLowerArm_T->get());

	translateLowerLeftArm_El = calculatDeltas(leftElbowJoint_T->get(), leftLowerArm_T->get());
	translateLowerRightArm_El = calculatDeltas(rightElbowJoint_T->get(), rightLowerArm_T->get());

	//Legs

	translateUpperLeftLeg_LJ = calculatDeltas(leftLegJoint_T->get(), leftUpperLeg_T->get());
	translateLeftKnee_LJ = calculatDeltas(leftLegJoint_T->get(), leftKneeJoint_T->get());
	translateLowerLeftLeg_LJ = calculatDeltas(leftLegJoint_T->get(), leftLowerLeg_T->get());

	translateUpperRightLeg_LJ = calculatDeltas(rightLegJoint_T->get(), rightUpperLeg_T->get());
	translateRightKnee_LJ = calculatDeltas(rightLegJoint_T->get(), rightKneeJoint_T->get());
	translateLowerRightLeg_LJ = calculatDeltas(rightLegJoint_T->get(), rightLowerLeg_T->get());


	translateLowerLeftLeg_KJ = calculatDeltas(leftKneeJoint_T->get(), leftLowerLeg_T->get());
	translateLowerRightLeg_KJ = calculatDeltas(rightKneeJoint_T->get(), rightLowerLeg_T->get());


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


	rotateAboutJointX(leftUpperArmMat, translateUpperLeftArm_Sh, -6.0f);
	rotateAboutJointX(leftElbowRotMat, translateLeftElbow_Sh, -6.0f);
	rotateAboutJointX(leftLowerArmMat, translateLowerLeftArm_Sh, -6.0f);
	
}
void MyViewer::posRotXAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();

	
	rotateAboutJointX(leftUpperArmMat, translateUpperLeftArm_Sh, 6.0f);
	rotateAboutJointX(leftElbowRotMat, translateLeftElbow_Sh, 6.0f);
	rotateAboutJointX(leftLowerArmMat, translateLowerLeftArm_Sh, 6.0f);
}
void MyViewer::negRotYAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();



	rotateAboutJointY(leftUpperArmMat, translateUpperLeftArm_Sh, -6.0f);
	rotateAboutJointY(leftElbowRotMat, translateLeftElbow_Sh, -6.0f);
	rotateAboutJointY(leftLowerArmMat, translateLowerLeftArm_Sh, -6.0f);

}
void MyViewer::posRotYAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();


	rotateAboutJointY(leftUpperArmMat, translateUpperLeftArm_Sh, 6.0f);
	rotateAboutJointY(leftElbowRotMat, translateLeftElbow_Sh, 6.0f);
	rotateAboutJointY(leftLowerArmMat, translateLowerLeftArm_Sh, 6.0f);

}
void MyViewer::posRotZAboutLSJ() {
	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();


	rotateAboutJointZ(leftUpperArmMat, translateUpperLeftArm_Sh, 6.0f);
	rotateAboutJointZ(leftElbowRotMat, translateLeftElbow_Sh, 6.0f);
	rotateAboutJointZ(leftLowerArmMat, translateLowerLeftArm_Sh, 6.0f);

}
void MyViewer::negRotZAboutLSJ() {

	GsMat& leftShoulderRotMat = leftShoulderJoint_T->get();
	GsMat& leftElbowRotMat = leftElbowJoint_T->get();
	GsMat& leftUpperArmMat = leftUpperArm_T->get();
	GsMat& leftLowerArmMat = leftLowerArm_T->get();

	rotateAboutJointZ(leftUpperArmMat, translateUpperLeftArm_Sh, -6.0f);
	rotateAboutJointZ(leftElbowRotMat, translateLeftElbow_Sh, -6.0f);
	rotateAboutJointZ(leftLowerArmMat, translateLowerLeftArm_Sh, -6.0f);

}

//Right Shoulder Joint
void MyViewer::negRotXAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();

	rotateAboutJointX(rightUpperArmMat, translateUpperLeftArm_Sh, -6.0f);
	rotateAboutJointX(rightElbowRotMat, translateLeftElbow_Sh, -6.0f);
	rotateAboutJointX(rightLowerArmMat, translateLowerLeftArm_Sh, -6.0f);
}
void MyViewer::posRotXAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	
	rotateAboutJointX(rightUpperArmMat, translateUpperLeftArm_Sh, 6.0f);
	rotateAboutJointX(rightElbowRotMat, translateLeftElbow_Sh, 6.0f);
	rotateAboutJointX(rightLowerArmMat, translateLowerLeftArm_Sh, 6.0f);
}
void MyViewer::negRotYAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();

	rotateAboutJointY(rightUpperArmMat, translateUpperLeftArm_Sh, -6.0f);
	rotateAboutJointY(rightElbowRotMat, translateLeftElbow_Sh, -6.0f);
	rotateAboutJointY(rightLowerArmMat, translateLowerLeftArm_Sh, -6.0f);
}
void MyViewer::posRotYAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	GsMat R;
	GsMat T;


	rotateAboutJointY(rightUpperArmMat, translateUpperLeftArm_Sh, 6.0f);
	rotateAboutJointY(rightElbowRotMat, translateLeftElbow_Sh, 6.0f);
	rotateAboutJointY(rightLowerArmMat, translateLowerLeftArm_Sh, 6.0f);
}
void MyViewer::posRotZAboutRSJ() {

	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();
	rotateAboutJointZ(rightUpperArmMat, translateUpperLeftArm_Sh, 6.0f);
	rotateAboutJointZ(rightElbowRotMat, translateLeftElbow_Sh, 6.0f);
	rotateAboutJointZ(rightLowerArmMat, translateLowerLeftArm_Sh, 6.0f);
}
void MyViewer::negRotZAboutRSJ() {
	GsMat& rightShoulderRotMat = rightShoulderJoint_T->get();
	GsMat& rightElbowRotMat = rightElbowJoint_T->get();
	GsMat& rightUpperArmMat = rightUpperArm_T->get();
	GsMat& rightLowerArmMat = rightLowerArm_T->get();

	rotateAboutJointZ(rightUpperArmMat, translateUpperLeftArm_Sh, -6.0f);
	rotateAboutJointZ(rightElbowRotMat, translateLeftElbow_Sh, -6.0f);
	rotateAboutJointZ(rightLowerArmMat, translateLowerLeftArm_Sh, -6.0f);
}

//Left Elbow Joint

void MyViewer::negRotXAboutLEJ() {
	GsMat &leftShoulder = leftShoulderJoint_T->get();
	GsMat &leftElbowJoint = leftElbowJoint_T->get();
	GsMat &lowerLeftArm = leftLowerArm_T->get();
	GsMat T; 
	GsMat R;
	
	theta += 6.0f; 

	rotateAboutJointX(lowerLeftArm, translateLowerLeftArm_El, -6.0f);

}
void MyViewer::posRotXAboutLEJ() {
	GsMat &leftShoulder = leftShoulderJoint_T->get();
	GsMat &leftElbowJoint = leftElbowJoint_T->get();
	GsMat &lowerLeftArm = leftLowerArm_T->get();
	GsMat T;
	GsMat R;

	
	theta -= 6.0f;
	rotateAboutJointX(lowerLeftArm, translateLowerLeftArm_El, 6.0f);

}

void MyViewer::negRotXAboutREJ() {
	GsMat &rightShoulder = rightShoulderJoint_T->get();
	GsMat &rightElbowJoint = rightElbowJoint_T->get();
	GsMat &lowerRightArm = rightLowerArm_T->get();
	GsMat T;
	GsMat R;

	phi += 6.0f;

	rotateAboutJointX(lowerRightArm, translateLowerRightArm_El, -6.0f);

}

void MyViewer::posRotXAboutREJ() {
	GsMat &rightShoulder = rightShoulderJoint_T->get();
	GsMat &rightElbowJoint = rightElbowJoint_T->get();
	GsMat &lowerRightArm = rightLowerArm_T->get();
	GsMat T;
	GsMat R;

	phi -= 6.0f;

	rotateAboutJointX(lowerRightArm, translateLowerRightArm_El, 6.0f);
}

void MyViewer::posRotXAboutLLJ() {

	rotateAboutJointX(leftUpperLeg_T->get(), translateUpperLeftLeg_LJ, 6.0f);
	rotateAboutJointX(leftKneeJoint_T->get(), translateLeftKnee_LJ, 6.0f);
	rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_LJ, 6.0f);
}

void MyViewer::posRotYAboutLLJ() {
	rotateAboutJointY(leftUpperLeg_T->get(), translateUpperLeftLeg_LJ, 6.0);
	rotateAboutJointY(leftKneeJoint_T->get(), translateLeftKnee_LJ, 6.0);
	rotateAboutJointY(leftLowerLeg_T->get(), translateLowerLeftLeg_LJ, 6.0);
}

void MyViewer::posRotyZAboutLLJ() {
	rotateAboutJointZ(leftUpperLeg_T->get(), translateUpperLeftLeg_LJ, 6.0);
	rotateAboutJointZ(leftKneeJoint_T->get(), translateLeftKnee_LJ, 6.0);
	rotateAboutJointZ(leftLowerLeg_T->get(), translateLowerLeftLeg_LJ, 6.0);
}

void MyViewer::negRotXAboutLLJ() {
	rotateAboutJointX(leftUpperLeg_T->get(), translateUpperLeftLeg_LJ, -6.0);
	rotateAboutJointX(leftKneeJoint_T->get(), translateLeftKnee_LJ, -6.0);
	rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_LJ, -6.0);
}

void MyViewer::negRotYAboutLLJ() {
	rotateAboutJointY(leftUpperLeg_T->get(), translateUpperLeftLeg_LJ, -6.0);
	rotateAboutJointY(leftKneeJoint_T->get(), translateLeftKnee_LJ, -6.0);
	rotateAboutJointY(leftLowerLeg_T->get(), translateLowerLeftLeg_LJ, -6.0);
}
void MyViewer::negRotZAboutLLJ() {
	rotateAboutJointZ(leftUpperLeg_T->get(), translateUpperLeftLeg_LJ, -6.0);
	rotateAboutJointZ(leftKneeJoint_T->get(), translateLeftKnee_LJ, -6.0);
	rotateAboutJointZ(leftLowerLeg_T->get(), translateLowerLeftLeg_LJ, -6.0);
}

void MyViewer::posRotXAboutLKJ() {

	alpha -= 6.0f;
	rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, 6.0f);
}

void MyViewer::negRotXAboutLKJ() {
	alpha += 6.0f;
	rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -6.0f);
}

void MyViewer::posRotXAboutRLJ() {
	rotateAboutJointX(rightUpperLeg_T->get(), translateUpperRightLeg_LJ, 6.0f);
	rotateAboutJointX(rightKneeJoint_T->get(), translateRightKnee_LJ, 6.0f);
	rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_LJ, 6.0f);
}
void MyViewer::posRotYAboutRLJ() {
	rotateAboutJointY(rightUpperLeg_T->get(), translateUpperRightLeg_LJ, 6.0f);
	rotateAboutJointY(rightKneeJoint_T->get(), translateRightKnee_LJ, 6.0f);
	rotateAboutJointY(rightLowerLeg_T->get(), translateLowerRightLeg_LJ, 6.0f);
}
void MyViewer::posRotZAboutRLJ() {
	rotateAboutJointZ(rightUpperLeg_T->get(), translateUpperRightLeg_LJ, 6.0f);
	rotateAboutJointZ(rightKneeJoint_T->get(), translateRightKnee_LJ, 6.0f);
	rotateAboutJointZ(rightLowerLeg_T->get(), translateLowerRightLeg_LJ, 6.0f);
}

void MyViewer::negRotXAboutRLJ() {
	rotateAboutJointX(rightUpperLeg_T->get(), translateUpperRightLeg_LJ, -6.0f);
	rotateAboutJointX(rightKneeJoint_T->get(), translateRightKnee_LJ, -6.0f);
	rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_LJ, -6.0f);
}
void MyViewer::negRotYAboutRLJ() {
	rotateAboutJointY(rightUpperLeg_T->get(), translateUpperRightLeg_LJ, -6.0f);
	rotateAboutJointY(rightKneeJoint_T->get(), translateRightKnee_LJ, -6.0f);
	rotateAboutJointY(rightLowerLeg_T->get(), translateLowerRightLeg_LJ, -6.0f);
}
void MyViewer::negRotZAboutRLJ() {
	rotateAboutJointZ(rightUpperLeg_T->get(), translateUpperRightLeg_LJ, -6.0f);
	rotateAboutJointZ(rightKneeJoint_T->get(), translateRightKnee_LJ, -6.0f);
	rotateAboutJointZ(rightLowerLeg_T->get(), translateLowerRightLeg_LJ, -6.0f);
}

void MyViewer::posRotXAboutRKJ() {
	beta -= 6.0f;
	rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, 6.0f);
}
void MyViewer::negRotXAboutRKJ() {
	beta += 6.0f;
	rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -6.0f);
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
		case 'o':{
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, phi);
			posRotZAboutRSJ();
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, -phi);
			ws_check();
			render();
			return 1; 

		}
		case 'p': {
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, phi);
			negRotZAboutRSJ();
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, -phi);

			ws_check();
			render();
			return  1; 
		}
		case 'q': {
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			posRotZAboutLSJ();
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);
			ws_check();
			render();

			return 1; 
		}
		case 'w': {
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			negRotZAboutLSJ();
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);

			ws_check();
			render();
			return 1; 
		}
		case 'j': {
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, phi);
			negRotXAboutRSJ();
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, -phi);
			ws_check();
			render();
			return 1; 
		}
		case 'k': {
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, phi);
			posRotXAboutRSJ();
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, -phi);
			ws_check();
			render();
			return 1; 
		}
		case 'a': {

			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			negRotXAboutLSJ();
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);

			ws_check();
			render();
			
			return 1; 
		}
		case 's': {
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			posRotXAboutLSJ();
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);

			ws_check();
			render();
			return 1; 
		}
		case 'z': {
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			negRotYAboutLSJ();
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);
			ws_check();
			render();
			return 1; 
		}
		case 'x': {
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, theta);
			posRotYAboutLSJ();
			rotateAboutJointX(leftLowerArm_T->get(), translateLowerLeftArm_El, -theta);
			ws_check();
			render();
			return 1; 
		}
		case 'm': {
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, phi);
			negRotYAboutRSJ();
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, -phi);
			ws_check();
			render();
			return 1; 
		}
		case ',': {
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, phi);
			posRotYAboutRSJ();
			rotateAboutJointX(rightLowerArm_T->get(), translateLowerRightArm_El, -phi);
			ws_check();
			render();
			return 1; 
		}
		case 'e': {
			negRotXAboutLEJ();

			ws_check();
			render();
			
			return 1; 
		}
		case 'r': {
			posRotXAboutLEJ();

			ws_check();
			render();
			return 1; 
		}
		case 'u': {
			posRotXAboutREJ();
			ws_check();
			render();

			return 1; 
		}
		case 'i': {
			negRotXAboutREJ();
			ws_check();
			render();

			return 1;
		}
		case 'd': {

			
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, alpha);
			posRotXAboutLLJ();
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -alpha);
			ws_check();
			render();

			return 1; 
		}
		case 'c': {
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, alpha);
			posRotYAboutLLJ();
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -alpha);
			ws_check();
			render();
			return 1; 
		}
		case 't': {
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, alpha);
			posRotyZAboutLLJ();
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -alpha);
			ws_check();
			render();
			return 1; 
		}
		case 'f': {
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, alpha);
			negRotXAboutLLJ();
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -alpha);
			ws_check();
			render();
			return 1; 
		}
		case 'v': {
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, alpha);
			negRotYAboutLLJ();
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -alpha);
			ws_check();
			render();
			return 1; 
		}
		case 'b': {
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, alpha);
			negRotZAboutLLJ();
			ws_check();
			rotateAboutJointX(leftLowerLeg_T->get(), translateLowerLeftLeg_KJ, -alpha);
			render();
			return 1; 
		}
		case 'g': {
			posRotXAboutLKJ();
			ws_check();
			render();
			return 1; 
		}
		case '5': {
			negRotXAboutLKJ();
			ws_check();
			render();
			return 1; 
		}
		case '1': {
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, beta);
			posRotXAboutRLJ();
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -beta);

			ws_check();
			render();
			return 1; 
		}
		case '2': {
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, beta);
			posRotYAboutRLJ();
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -beta);

			ws_check();
			render();
			return 1; 
		}
		case '3': {
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, beta);
			posRotZAboutRLJ();
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -beta);

			ws_check();
			render();
			return 1;
		}
		case '4': {
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, beta);
			negRotXAboutRLJ();
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -beta);

			ws_check();
			render();
			return 1;
		}
		case '6': {
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, beta);
			negRotYAboutRLJ();
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -beta);

			ws_check();
			render();
			return 1;
		}
		case '7': {
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, beta);
			negRotZAboutRLJ();
			rotateAboutJointX(rightLowerLeg_T->get(), translateLowerRightLeg_KJ, -beta);

			ws_check();
			render();
			return 1;
		}
		case '8': {
			posRotXAboutRKJ();
			ws_check();
			render();
			return 1; 
		}
		case '9': {
			negRotXAboutRKJ();
			ws_check();
			render();
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
