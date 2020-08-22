#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChBarrelShape.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

#define MAX_ROT_SPEED 0.8
#define MAX_XZ_SPEED 10

double STATIC_rot_speed = 0;
double STATIC_x_speed = 0;
double STATIC_z_speed = 0;
float STATIC_wheelfriction = 0.6f;

// This small function creates a Mecanum wheel, made with many ChBodySceneNode rigid bodies (a central
// wheel and the many radial rollers, already lined to the wheel with revolute joints.)
// The function returns the pointer to the central wheel.
std::shared_ptr<ChBody> create_mecanum_wheel(ChSystemNSC& mphysicalSystem,
                                             ChVector<> shaft_position,
                                             ChQuaternion<> shaft_alignment,
                                             double wheel_radius,
                                             double wheel_width,
                                             int n_rollers,
                                             double roller_angle,
                                             double roller_midradius,
                                             double roller_density,
                                             double spindle_density) {
    ChFrameMoving<> ftot(shaft_position, shaft_alignment);  // will be used to transform pos & rot of all objects

    auto mCentralWheel = chrono_types::make_shared<ChBodyEasyCylinder>(wheel_radius / 2, wheel_width,  // radius, height
                                                                       spindle_density,                // density
                                                                       true,                           // visualize
                                                                       false);                         // no collision
    mCentralWheel->SetPos(shaft_position);
    mCentralWheel->SetRot(shaft_alignment);
	mphysicalSystem.Add(mCentralWheel);

	auto mtexturepw = chrono_types::make_shared<ChTexture>();
    mtexturepw->SetTextureFilename(GetChronoDataFile("pinkwhite.png"));
    mCentralWheel->AddAsset(mtexturepw);

    auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    wheel_mat->SetFriction(STATIC_wheelfriction);

    double half_length_roller = 0.5 * wheel_width * 1.0 / (cos(roller_angle));
    double roller_elliptical_rad_Hor = wheel_radius;
    double roller_elliptical_rad_Vert = wheel_radius * 1.0 / (cos(roller_angle));

    for (int iroller = 0; iroller < n_rollers; iroller++) {
        double pitch = CH_C_2PI * ((double)iroller / (double)n_rollers);

		double Roffset = -(wheel_radius - roller_midradius);

        // Create the roller
		auto mRoller = chrono_types::make_shared<ChBody>();
		mphysicalSystem.Add(mRoller);

		// move it to slanted aligment
		ChFrameMoving<> f1( ChVector<>(0, 0, -(wheel_radius - roller_midradius)),
							Q_from_AngAxis(roller_angle, ChVector<>(0, 0, 1)));
        ChFrameMoving<> f2( ChVector<>(0, 0, 0), 
							Q_from_AngAxis(pitch, ChVector<>(0, 1, 0)));
        ChFrameMoving<> f3 = f1 >> f2 >> ftot;
        mRoller->ConcatenatePreTransformation(f3);

		// approximate mass & inertia to a cylinder:
		mRoller->SetMass(utils::CalcCylinderVolume(roller_elliptical_rad_Hor + Roffset, 2 * half_length_roller) * roller_density);
		mRoller->SetInertia(utils::CalcCylinderGyration(roller_elliptical_rad_Hor + Roffset, 2 * half_length_roller) * roller_density);

        // add collision shape
        mRoller->GetCollisionModel()->ClearModel();
        mRoller->GetCollisionModel()->AddBarrel(wheel_mat,                                              //
                                                -half_length_roller, +half_length_roller,               //
                                                roller_elliptical_rad_Vert, roller_elliptical_rad_Hor,  //
                                                Roffset);
        mRoller->GetCollisionModel()->BuildModel();
        mRoller->SetCollide(true);

        // add visualization shape
        auto mrollershape =
            chrono_types::make_shared<ChBarrelShape>(-half_length_roller, +half_length_roller,               //
                                                     roller_elliptical_rad_Vert, roller_elliptical_rad_Hor,  //
                                                     Roffset);
        mRoller->AddAsset(mrollershape);

        // Make the revolute joint between the roller and the central wheel
        // (preconcatenate rotation 90 degrees on X, to set axis of revolute joint)
        ChFrameMoving<> fr(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI / 2.0, ChVector<>(1, 0, 0)));
        ChFrameMoving<> frabs = fr >> f3;
        auto my_link_roller = chrono_types::make_shared<ChLinkLockRevolute>();
        my_link_roller->Initialize(mRoller, mCentralWheel, frabs.GetCoord());
        mphysicalSystem.AddLink(my_link_roller);

    }

    return mCentralWheel;
}

class MySimpleCar
{
public:

    // Speed of each wheel
    double speed_FL;
    double speed_BR;
    double speed_BL;
    double speed_FR;

    // The parts making the car, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    std::shared_ptr<ChBody> mTrussPlatform;

    std::shared_ptr<ChBody> spindle_A;
    std::shared_ptr<ChBody> spindle_B;
    std::shared_ptr<ChBody> spindle_C;
    std::shared_ptr<ChBody> spindle_D;

    std::shared_ptr<ChLinkMotorRotationSpeed> my_link_shaftA;   //FL
    std::shared_ptr<ChLinkMotorRotationSpeed> my_link_shaftB;   //BR
    std::shared_ptr<ChLinkMotorRotationSpeed> my_link_shaftC;   //BL
    std::shared_ptr<ChLinkMotorRotationSpeed> my_link_shaftD;   //FR

    // THE FUNCTIONS

    // Build and initialize the car, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleCar(ChSystemNSC& mphysicalSystem,        ///< the Chrono physical system
                ISceneManager* msceneManager,  ///< the Irrlicht scene manager for 3d shapes
                IVideoDriver* mdriver          ///< the Irrlicht video driver
    ){
        // Initially, speed set as 0
        speed_FL = speed_BR = speed_BL = speed_FR = 0;
        
        // Car Body: Length, Width and Height. 
        // Offset: the distance between wheel COM and car COM
        double L = 23;
        double W = 15.2;
        double H = 1;
        double offset = 18 / 2;

        // Wheel property
        double wheel_radius = 3;
        double roller_angle = CH_C_PI / 4;

        // --- The Car Body ---
        // Create the robot truss, as a circular platform
        mTrussPlatform = chrono_types::make_shared<ChBodyEasyBox>(L, H, W,  // x,y,z
                                                                    1000,                      // density
                                                                    true,                      // visualize
                                                                    false);                    // no collision
        mphysicalSystem.Add(mTrussPlatform);

        // --- Wheels ---
        ChFrame<> f0(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI  / 2.0, ChVector<>(1, 0, 0)));
        ChFrame<> f1(ChVector<>(6, 0, offset), QUNIT);
        ChFrame<> f11(ChVector<>(-6, 0, offset), QUNIT);
        ChFrame<> f2_wA(VNULL, Q_from_AngAxis(0 * (CH_C_PI), ChVector<>(0, 1, 0)));
        ChFrame<> f2_wB(VNULL, Q_from_AngAxis(1 * (CH_C_PI), ChVector<>(0, 1, 0)));

        ChFrame<> f2_wC(VNULL, Q_from_AngAxis(0 * (CH_C_PI), ChVector<>(0, 1, 0)));
        ChFrame<> f2_wD(VNULL, Q_from_AngAxis(1 * (CH_C_PI), ChVector<>(0, 1, 0)));

        ChFrame<> ftot_wA = f0 >> f1 >> f2_wA;
        ChFrame<> ftot_wB = f0 >> f1 >> f2_wB;

        ChFrame<> ftot_wC = f0 >> f11 >> f2_wC;
        ChFrame<> ftot_wD = f0 >> f11 >> f2_wD;

        spindle_A =
            create_mecanum_wheel(mphysicalSystem, 
                                ftot_wA.GetCoord().pos,  // wheel position
                                ftot_wA.GetCoord().rot,  // wheel alignment
                                wheel_radius,            // wheel radius
                                2.2,                     // wheel width
                                8,                       // n. of rollers
                                -roller_angle,            // angle of rollers
                                0.65,                    // max rad. of roller
                                1000,                    // density of roller
                                1000);                   // density of the spindle

        my_link_shaftA = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_link_shaftA->Initialize(spindle_A, mTrussPlatform, (f1 >> f2_wA));
        my_link_shaftA->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
        mphysicalSystem.AddLink(my_link_shaftA);

        spindle_B =
            create_mecanum_wheel(mphysicalSystem, 
                                ftot_wB.GetCoord().pos,  // wheel position
                                ftot_wB.GetCoord().rot,  // wheel alignment
                                wheel_radius,            // wheel radius
                                2.2,                     // wheel width
                                8,                       // n. of rollers
                                -roller_angle,            // angle of rollers
                                0.65,                    // max rad. of roller
                                1000,                    // density of roller
                                1000);                   // density of the spindle

        my_link_shaftB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_link_shaftB->Initialize(spindle_B, mTrussPlatform, (f1 >> f2_wB));
        my_link_shaftB->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
        mphysicalSystem.AddLink(my_link_shaftB);

        spindle_C =
            create_mecanum_wheel(mphysicalSystem, 
                                ftot_wC.GetCoord().pos,  // wheel position
                                ftot_wC.GetCoord().rot,  // wheel alignment
                                wheel_radius,            // wheel radius
                                2.2,                     // wheel width
                                8,                       // n. of rollers
                                roller_angle,            // angle of rollers
                                0.65,                    // max rad. of roller
                                1000,                    // density of roller
                                1000);                   // density of the spindle

        my_link_shaftC = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_link_shaftC->Initialize(spindle_C, mTrussPlatform, (f11 >> f2_wC));
        my_link_shaftC->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
        mphysicalSystem.AddLink(my_link_shaftC);

        spindle_D =
            create_mecanum_wheel(mphysicalSystem, 
                                ftot_wD.GetCoord().pos,  // wheel position
                                ftot_wD.GetCoord().rot,  // wheel alignment
                                wheel_radius,            // wheel radius
                                2.2,                     // wheel width
                                8,                       // n. of rollers
                                roller_angle,            // angle of rollers
                                0.65,                    // max rad. of roller
                                1000,                    // density of roller
                                1000);                   // density of the spindle

        my_link_shaftD = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_link_shaftD->Initialize(spindle_D, mTrussPlatform, (f11 >> f2_wD));
        my_link_shaftD->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
        mphysicalSystem.AddLink(my_link_shaftD);


    }

    // Delete the car object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleCar()
    {
        ChSystem* mysystem = mTrussPlatform->GetSystem();  // trick to get the system here

        mysystem->Remove(my_link_shaftA);
        mysystem->Remove(my_link_shaftB);
        mysystem->Remove(my_link_shaftC);
        mysystem->Remove(my_link_shaftD);
        mysystem->Remove(mTrussPlatform);
        mysystem->Remove(spindle_A);
        mysystem->Remove(spindle_B);
        mysystem->Remove(spindle_C);
        mysystem->Remove(spindle_D);
    }
};


// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* myapp, MySimpleCar* acar) {
        // store pointer application
        application = myapp;
        // store pointer to other stuff
        mcar = acar;

        // ..add a GUI slider to control wheel left via mouse
        scrollbar_FL =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 20, 650, 35), 0, 101);
        scrollbar_FL->setMax(100);
        scrollbar_FL->setPos(50);
        text_FL =
            application->GetIGUIEnvironment()->addStaticText(L"Front Left Wheel ", rect<s32>(650, 20, 750, 35), false);

        // ..add a GUI slider to control wheel right via mouse
        scrollbar_BR =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 45, 650, 60), 0, 102);
        scrollbar_BR->setMax(100);
        scrollbar_BR->setPos(50);
        text_BR =
            application->GetIGUIEnvironment()->addStaticText(L"Back Right Wheel", rect<s32>(650, 45, 750, 60), false);

        // ..add a GUI slider to control wheel right via mouse
        scrollbar_BL =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 70, 650, 85), 0, 103);
        scrollbar_BL->setMax(100);
        scrollbar_BL->setPos(50);
        text_BL =
            application->GetIGUIEnvironment()->addStaticText(L"Back Left Wheel", rect<s32>(650, 70, 750, 85), false);

        // ..add a GUI slider to control wheel right via mouse
        scrollbar_FR =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 95, 650, 110), 0, 104);
        scrollbar_FR->setMax(100);
        scrollbar_FR->setPos(50);
        text_FR =
            application->GetIGUIEnvironment()->addStaticText(L"Front Right Wheel", rect<s32>(650, 95, 750, 110), false);

        sprintf(this->message, "FL: %4.4f, FR: %4.4f, , BL: %4.4f, BR: %4.4f\n", mcar->speed_FL, mcar->speed_FR, mcar->speed_BL, mcar->speed_BR);
        text_vec = application->GetIGUIEnvironment()->addStaticText(
            core::stringw(this->message).c_str(), rect<s32>(150, 10, 430, 40), false);
    }

    void OnChangeScreenInfo(){
        sprintf(this->message, "FL: %4.4f, FR: %4.4f, BL: %4.4f, BR: %4.4f\n", this->mcar->speed_FL, this->mcar->speed_FR, this->mcar->speed_BL, this->mcar->speed_BR);
        this->text_vec->setText(core::stringw(this->message).c_str());
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();
            IGUIEnvironment* env = application->GetIGUIEnvironment();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 101) {  // id of 'FL: Wheel A' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newspeed = ((double)(pos)-50) / 50.0;

                        // FL Minus
                        newspeed = newspeed * -1.0;

                        this->mcar->speed_FL = newspeed;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mcar->my_link_shaftA->GetSpeedFunction());
                        mfun->Set_yconst(newspeed);
                        this->OnChangeScreenInfo();
                        return true;
                    }
                    if (id == 102) {  // id of 'BR: Wheel B' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newspeed = ((double)(pos)-50) / 50.0;
                        this->mcar->speed_BR = newspeed;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mcar->my_link_shaftB->GetSpeedFunction());
                        mfun->Set_yconst(newspeed);
                        this->OnChangeScreenInfo();
                        return true;
                    }
                    if (id == 103) {  // id of 'BL: Wheel C' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newspeed = ((double)(pos)-50) / 50.0;

                        // BL Minus
                        newspeed = newspeed * -1.0;

                        this->mcar->speed_BL = newspeed;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mcar->my_link_shaftC->GetSpeedFunction());
                        mfun->Set_yconst(newspeed);
                        this->OnChangeScreenInfo();
                        return true;
                    }
                    if (id == 104) {  // id of 'FR: Wheel D' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newspeed = ((double)(pos)-50) / 50.0;
                        this->mcar->speed_FR = newspeed;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mcar->my_link_shaftD->GetSpeedFunction());
                        mfun->Set_yconst(newspeed);
                        this->OnChangeScreenInfo();
                        return true;
                    }
                    break;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChIrrAppInterface* application;
    MySimpleCar* mcar;

    IGUIStaticText* text_FL;
    IGUIScrollBar* scrollbar_FL;
    
    IGUIStaticText* text_BR;
    IGUIScrollBar* scrollbar_BR;
    
    IGUIStaticText* text_BL;
    IGUIScrollBar* scrollbar_BL;

    IGUIStaticText* text_FR;
    IGUIScrollBar* scrollbar_FR;

    char message[100];
    IGUIStaticText* text_vec;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Mecanum robot simulator", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    // 2- Create the rigid bodies of the simpified tank suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(STATIC_wheelfriction);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(200, 1, 200,  // size
                                                           1000,         // density
                                                           true,         // visualize
                                                           true,         // collide
                                                           ground_mat);  // contact material
    // ground->SetPos(ChVector<>(0, -1, 0));
    ground->SetPos(ChVector<>(0, -1*3-0.01, 0));
    ground->SetBodyFixed(true);
    mphysicalSystem.Add(ground);

	auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mtexture->SetTextureScale(100, 100);

    ground->AddAsset(mtexture);
    // ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("blu.png")));
    // mphysicalSystem.AddBody(ground);


    // ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleCar* mycar = new MySimpleCar(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object.
    MyEventReceiver receiver(&application, mycar);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    //
    // SETTINGS
    //
    // system.Set_G_acc(ChVector<>(0, -10, 0));
    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(30);  // the higher, the easier to keep the constraints satisfied.

    //
    // Simulation loop
    //

    application.SetTimestep(0.03);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. draw also a grid (rotated so that it's horizontal)
        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30, 30,
                             ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 60, 60, 60), true);
        ChIrrTools::drawAllCOGs(mphysicalSystem, application.GetVideoDriver(), 5.0);

        // HERE CHRONO INTEGRATION IS PERFORMED:
        application.DoStep();

        application.EndScene();
    }

    if (mycar)
        delete mycar;

    return 0;
}


