# pragma once

# include <sig/sn_poly_editor.h>
# include <sig/sn_lines2.h>

# include <sigogl/ui_button.h>
# include <sigogl/ws_viewer.h>

// Viewer for this example:
class MyViewer : public WsViewer
{  protected :
	enum MenuEv { EvNormals, EvAnimate, EvExit };
	UiCheckButton* _nbut;
	bool _animating;
	SnTransform *torso_T;
	SnTransform *neck_T;
	SnTransform *head_T;
	SnTransform *pelvis_T;
	SnTransform *leftShoulderJoint_T;
	SnTransform *rightShoulderJoint_T;
	SnTransform *leftUpperArm_T;
	SnTransform *rightUpperArm_T;
	SnTransform *leftElbowJoint_T;
	SnTransform *rightElbowJoint_T;
	SnTransform *leftLowerArm_T;
	SnTransform *rightLowerArm_T;
	SnTransform *leftLegJoint_T;
	SnTransform *rightLegtJoint_T;
	SnTransform *leftUpperLeg_T;
	SnTransform *rightUpperLeg_T;
	SnTransform *leftKneeJoint_T;
	SnTransform *rightKneeJoint_T;
	SnTransform *leftLowerLeg_T;
	SnTransform *rightLowerLeg_T;
   public :
	MyViewer ( int x, int y, int w, int h, const char* l );
	void build_ui ();
	void add_model ( SnShape* s, GsVec p, SnTransform *t );
	void build_scene ();
	void buildTorso();
	void buildNeck();
	void buildHead();
	void buildPelvis();
	void buildLeftShoulderJoint();
	void buildRightShoulderJoint();
	void buildLeftUpperArm();
	void buildRightUpperArm();
	void buildLeftElbowJoint();
	void buildRightElbowJoint();
	void buildLeftLowerArm();
	void buildRightLowerArm();
	void buildLeftLegJoint();
	void buildRightLegJoint();
	void buildLeftUpperLeg();
	void buildRightUpperLeg();
	void buildLeftKneeJoint();
	void buildRightKneeJoint();
	void buildLeftLowerLeg();
	void buildRightLowerLeg();
	void show_normals ( bool b );
	void run_animation ();
	virtual int handle_keyboard ( const GsEvent &e ) override;
	virtual int uievent ( int e ) override;
};

