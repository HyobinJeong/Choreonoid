/**
   Sample walking motion controller for the SR1 robot model.
   This program was ported from the "SamplePD" sample of OpenHRP3.
   @author Shin'ichiro Nakaoka
*/

/**

  Connect to RBQ-3 Project

  by Hyobin Jeong
 */


// #include "/home/rainbow/Desktop/RBQuad_Controller/utils/SimVariable.h"
#include "/home/rbq/Desktop/RBQuad_PODO/share/Headers/SimVariable.h"


#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/Sensor>
#include <cnoid/Device>
#include <cnoid/ForceSensor>

//for marker
#include <cnoid/Plugin>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneView>
#include <cnoid/RootItem>
#include <cnoid/EigenUtil> // Eigen 변환 관련 함수들
#include <cnoid/MeshGenerator> // 메쉬 생성기

pSIM_VARIABLE simData;

const static double D2R = 1.0/180.0*3.1415926535897;
const static double R2D = 1.0/D2R;


using namespace std;
using namespace cnoid;



static double pgain[] = {
   200.0, 200.0, 200.0,
   200.0, 200.0, 200.0,
   200.0, 200.0, 200.0,
   200.0, 200.0, 200.0,
};

static double dgain[] = {
   1.0, 1.0, 1.0,
   1.0, 1.0, 1.0,
   1.0, 1.0, 1.0,
   1.0, 1.0, 1.0,
};

static double initPos[] = {
   0.0, 30.0, -60.0,
   0.0, 30.0, -60.0,
   0.0, 30.0, -60.0,
   0.0, 30.0, -60.0,
};

static double lowerJLim[] = {
   -42.0, -360.0, -165.0,
   -31.0, -360.0, -165.0,
   -42.0, -360.0, -165.0,
   -31.0, -360.0, -165.0,
};

static double upperJLim[] = {
   31.0, 360.0, -24.0,
   42.0, 360.0, -24.0,
   31.0, 360.0, -24.0,
   42.0, 360.0, -24.0,
};

static double TauLim[] = {
   100.0, 100.0, 140.0,
   100.0, 100.0, 140.0,
   100.0, 100.0, 140.0,
   100.0, 100.0, 140.0,
};


enum JointSequentialNumberBYNAME
{
    HRR, HRP, HRK,
    HLR, HLP, HLK,
    FRR, FRP, FRK,
    FLR, FLP, FLK,
    NO_OF_JOINTS
};




class RBQController : public cnoid::SimpleController
{
    BodyPtr body;
    vector<double> qref;
    vector<double> q;
    vector<double> qold;
    vector<double> dq;
    vector<double> q0;
    vector<double> q0ref;

    ForceSensor* pHRFT;
    ForceSensor* pHLFT;
    ForceSensor* pFRFT;
    ForceSensor* pFLFT;
    AccelerationSensor* pAcc;
    RateGyroSensor* pGyro;

    SimpleControllerIO* pio;

    bool shm_opened;

    int currentFrame;
    double dt;
    int initcnt;

    double Tau_ref[12];
    double Vel_ref[12];
    double Pos_ref[12];
    double pGain[12], dGain[12];
    double Pos_ref_old[12];

    double jointInput[12];
    int cnt;

    double Quat_IMU[4];

//    SgPointSetPtr marker; // 마커 추가
    SgPosTransformPtr zmpRef, comRef; //sphereTransform;
    SgPosTransformPtr pfoot_final[4];

public:
    const static int NUM_INIT_STEP = 1000;
    virtual bool initialize(SimpleControllerIO* io) override {
        try {
            // Core Shared Memory Creation [SharedData]===================================
            shm_opened = false;
            int shmFD;
            shmFD = shm_open(SIM_DATA, O_RDWR, 0666);
            if(shmFD == -1){
                io->os() << "Fail to open core shared memory [simData]";
            }else{
                if(ftruncate(shmFD, sizeof(SIM_VARIABLE)) == -1){
                    io->os() << "Fail to truncate core shared memory [simData]";
                }else{
                    simData = (pSIM_VARIABLE)mmap(0, sizeof(SIM_VARIABLE), PROT_WRITE, MAP_SHARED, shmFD, 0);
                    if(simData == (void*)-1){
                        io->os() << "Fail to mapping core shared memory [simData]";
                    }
                }
            }
            io->os() << "Core shared memory creation = OK [simData]";
            // =========================================================================


            pio = io;
            io->os() << "init start" << endl;
            dt = io->timeStep();
            io->os() << "time step: " <<dt<<endl;
            body = io->body();
            qref.resize(body->numJoints());
            q.resize(body->numJoints());
            qold.resize(body->numJoints());
            dq.resize(body->numJoints());
            q0.resize(body->numJoints());
            q0ref.resize(body->numJoints());

            io->os()<<"Device num: "<<body->numDevices()<<endl;
            //Device list?
            pGyro = body->findDevice<RateGyroSensor>("WaistGyro");
            pAcc = body->findDevice<AccelerationSensor>("WaistAccelSensor");

    //        DeviceList<ForceSensor> forceSensors(body->devices());
    //        forceSensors<< body->devices();

    //        pHRFT = forceSensor[0];

            pHRFT = body->findDevice<ForceSensor>("HRFT");
            pHLFT = body->findDevice<ForceSensor>("HLFT");
            pFRFT = body->findDevice<ForceSensor>("FRFT");
            pFLFT = body->findDevice<ForceSensor>("FLFT");

            io->enableInput(pGyro);
            io->enableInput(pAcc);
            io->enableInput(pHRFT);
            io->enableInput(pHLFT);
            io->enableInput(pFRFT);
            io->enableInput(pFLFT);
            io->enableInput(body->rootLink(), LINK_POSITION);

            io->enableInput(body->link("HIND_R_FT"), LINK_POSITION);
            io->enableInput(body->link("HIND_L_FT"), LINK_POSITION);
            io->enableInput(body->link("FRONT_R_FT"), LINK_POSITION);
            io->enableInput(body->link("FRONT_L_FT"), LINK_POSITION);


            for(int i=0; i < body->numJoints(); ++i){

                Link* joint = body->joint(i);
                joint->setActuationMode(Link::JointTorque);
                io->enableIO(joint);
                qref[i] = joint->q();

                q0[i] = body->joint(i)->q();
                Pos_ref[i] = 0;
                Pos_ref_old[i] = 0;
                Tau_ref[i] = 0;
            }

            io->os() << "init done" << endl;

            Quat_IMU[0] = 1;
            Quat_IMU[1] = 0;
            Quat_IMU[2] = 0;
            Quat_IMU[3] = 0;

            cnt = 0;

            initcnt = NUM_INIT_STEP;

            // 마커 초기화
    //        initializeMarker(io);
            initializeSphereMarker(io);


            return true;
        } catch (const std::exception& e) {
            io->os() << "Exception in initialize: " << e.what() << std::endl;
            return false;
        }

    }

//    void initializeMarker(SimpleControllerIO* io) {
//            // 표시할 점의 초기 좌표
//            Vector3f position(0.5, 0.5, 0.5);

//            // 점의 색상 (R, G, B)
//            SgColorArrayPtr colors = new SgColorArray;
//            colors->resize(1);
//            (*colors)[0] = Vector3f(1.0f, 0.0f, 0.0f); // 빨간색


//            // 점의 크기
//            float size = 15.0f;

//            // 마커 생성
//            marker = new SgPointSet;
//            marker->setColors(colors);
//            marker->setPointSize(size);
//            marker->getOrCreateVertices()->push_back(position);

//            // 뷰어에 SceneGraph 추가
//            SceneView::instance()->scene()->addChild(marker);

//            io->os() << "Marker initialized at position: " << position.transpose() << endl;
//        }

//    void updateMarkerPosition(const Vector3f& position) {
//        marker->vertices()->at(0) = position;
//        marker->notifyUpdate();
//    }

    void initializeSphereMarker(SimpleControllerIO* io) {
        // 구의 반지름
        float radius = 0.03f;

        // 구의 색상 (R, G, B)
        Vector3f color_zmpRef(1.0f, 0.0f, 0.0f); // red
        Vector3f color_comRef(0.0f, 1.0f, 0.0f); // green
        Vector3f color_pfoot_final(0.0f, 0.0f, 1.0f); // blue

        // 구 메쉬 생성
        MeshGenerator meshGenerator;
        SgShapePtr sphere_zmpRef = new SgShape;
        sphere_zmpRef->setMesh(meshGenerator.generateSphere(radius));
        SgShapePtr sphere_comRef = new SgShape;
        sphere_comRef->setMesh(meshGenerator.generateSphere(0.08));

        SgShapePtr sphere_pfoot_final[4];
        for(int i=0; i<4; i++){
            sphere_pfoot_final[i] = new SgShape;
            sphere_pfoot_final[i]->setMesh(meshGenerator.generateSphere(radius));
        }


        // 구 재질 설정
        SgMaterialPtr material_zmpRef = new SgMaterial;
        material_zmpRef->setDiffuseColor(color_zmpRef);
        sphere_zmpRef->setMaterial(material_zmpRef);

        SgMaterialPtr material_comRef = new SgMaterial;
        material_comRef->setDiffuseColor(color_comRef);
        sphere_comRef->setMaterial(material_comRef);

        SgMaterialPtr material_pfoot_final = new SgMaterial;
        material_pfoot_final->setDiffuseColor(color_pfoot_final);
        for(int i=0; i<4; i++){
            sphere_pfoot_final[i]->setMaterial(material_pfoot_final);
        }


        // 변환 노드에 추가
        zmpRef = new SgPosTransform;
        zmpRef->addChild(sphere_zmpRef);
        comRef = new SgPosTransform;
        comRef->addChild(sphere_comRef);

        for(int i=0; i<4; i++){
            pfoot_final[i] = new SgPosTransform;
            pfoot_final[i]->addChild(sphere_pfoot_final[i]);
        }

        // 뷰어에 SceneGraph 추가
        SceneView::instance()->scene()->addChild(zmpRef);
        SceneView::instance()->scene()->addChild(comRef);
        for(int i=0; i<4; i++){
            SceneView::instance()->scene()->addChild(pfoot_final[i]);
        }


        io->os() << "Sphere marker initialized with radius: " << radius << endl;
    }
    void updatezmpRefPosition(const Vector3& position) {
        // 변환 행렬을 설정하여 위치 업데이트
        zmpRef->setTranslation(position);
    }
    void updatecomRefPosition(const Vector3& position) {
        // 변환 행렬을 설정하여 위치 업데이트
        comRef->setTranslation(position);
    }
    void updatefootPosition0(const Vector3& position) {
        // 변환 행렬을 설정하여 위치 업데이트
        pfoot_final[0]->setTranslation(position);
//        pfoot_final[0]->notifyUpdate();
    }
    void updatefootPosition1(const Vector3& position) {
        // 변환 행렬을 설정하여 위치 업데이트
        pfoot_final[1]->setTranslation(position);
//        pfoot_final[1]->notifyUpdate();
    }
    void updatefootPosition2(const Vector3& position) {
        // 변환 행렬을 설정하여 위치 업데이트
        pfoot_final[2]->setTranslation(position);
//        pfoot_final[2]->notifyUpdate();
    }
    void updatefootPosition3(const Vector3& position) {
        // 변환 행렬을 설정하여 위치 업데이트
        pfoot_final[3]->setTranslation(position);
//        pfoot_final[3]->notifyUpdate();
    }

    double LimitFunction(double _input, double _upperlim, double _lowerlim){
        double output_ = _input;
        if(_input >= _upperlim) output_ = _upperlim;
        else if(_input <= _lowerlim) output_ = _lowerlim;
        else output_ = _input;

        return output_;
    }

    void getPosRef(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            Pos_ref[i] = simData->choreonoid_refJointPos[i];
        }
    }

    void getVelRef(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            Vel_ref[i] = simData->choreonoid_refJointVel[i];
        }
    }

    void getTorqueRef(){
        for(int i=0; i<NO_OF_JOINTS ; i++){
            Tau_ref[i] = simData->choreonoid_refJointTorque[i];
        }
    }

    void getGains(){
        for(int i=0; i<NO_OF_JOINTS ; i++){
            pGain[i] = simData->Joint_pGain[i];
            dGain[i] = simData->Joint_dGain[i];
        }
    }

    void setPosNow(){
        for(int i=0; i<NO_OF_JOINTS ; i++){
            simData->choreonoid_JointPosNow[i] = q[i];
        }
    }
    void setVelNow(){
        for(int i=0; i<NO_OF_JOINTS ; i++){
            simData->choreonoid_JointVelNow[i] = dq[i];
        }
    }

    virtual bool control() override {
        try {
            Vector3 Gyro = pGyro->w();
            Vector3 Acc = pAcc->dv();

    //        Vector6 rfwrench = pRFFT->F();
    //        Vector6 lfwrench = pLFFT->F();

            Isometry3 T = body->rootLink()->position();
            Vector3 P = T.translation();
            Matrix3 R = T.rotation();
            Quaterniond Qt(R);

            Eigen::Matrix3d rotationMatrix = R;
            Eigen::Vector3d euler = rotationMatrix.eulerAngles(2, 1, 0);

    //        io->os() << P << endl;
            //pio->os() << Qt.w() <<" "<<Qt.x() <<" "<<Qt.y() <<" "<<Qt.z() <<" "<<endl;


            //test_give force to robot
            //Link* WLptr= body->link("WAIST");
            //io->os()<<WLptr->index()<< " f_ext "<<WLptr->f_ext()  <<endl;
            //WLptr->f_ext() =Vector3(200,0,0);
            //not working......why???????

            //setExternalForce(body,WLptr,Vector3(0,0,0),Vector3(200,0,0),0.1);
            //how to apply????

            for(int i=0; i<3 ; i++){
                simData->choreonoid_acc[i] = Acc[i];
                simData->choreonoid_gyro[i] = Gyro[i];
                simData->choreonoid_W_body_pos[i] = P[i];
            }


           simData->choreonoid_quat[0] = Qt.w();
           simData->choreonoid_quat[1] = Qt.x();
           simData->choreonoid_quat[2] = Qt.y();
           simData->choreonoid_quat[3] = Qt.z();



            //pio->os() <<Gyro[0] <<" "<<Gyro[1] <<" "<<Gyro[2] <<" "<<endl;
            //pio->os() <<Acc[0] <<" "<<Acc[1] <<" "<<Acc[2] <<" "<<endl;

            ///----- Go to Initial Position during INIT steptime--------
            if(initcnt==NUM_INIT_STEP)
            {
                getPosRef();
                for(int i=0; i < body->numJoints(); ++i){
                    Link* joint = body->joint(i);
                    q0ref[i] = Pos_ref[i];
                    q0[i] = joint->q();

                    //pio->os()<<"posRef["<<i<<"]: "<<Pos_ref[i]<<endl;
                }



            }
            initcnt--;
            if(initcnt>0)
            {
                double alpha = (NUM_INIT_STEP*1.0-initcnt*1.0)/(NUM_INIT_STEP*1.0);

                for(int i=0; i < body->numJoints(); ++i){
                    Link* joint = body->joint(i);
                    qref[i] = q0[i]+(alpha)*(q0ref[i]-q0[i]);
                    q[i] = joint->q();

                    double dq_ref = 0;
                    dq[i] = (q[i] - qold[i]) / dt;
                    joint->u() = (qref[i] - q[i]) * pgain[i] + (dq_ref - dq[i]) * dgain[i];

                    qold[i] = q[i];
                    Pos_ref_old[i] = Pos_ref[i];
                }

                //io->os() << "alpha: "<<alpha<<endl;
                return true;
            }
            ///------------------------------------------------------------


            ///------Joint Position Control ---------------------------------
            getPosRef();
            getGains();
            for(int i=0; i < body->numJoints(); ++i){

                Link* joint = body->joint(i);
                qref[i] = Pos_ref[i];
                q[i] = joint->q();
                dq[i] = (q[i] - qold[i]) / dt;

                double dq_ref = (qref[i] - Pos_ref_old[i]) / dt;
    //            double JointToque = (qref[i] - q[i]) * pGain[i] + (dq_ref - dq[i]) * dGain[i];
                double JointToque = (qref[i] - q[i]) * pGain[i] + (- dq[i]) * dGain[i];

                jointInput[i] = JointToque;


                qold[i] = q[i];
                Pos_ref_old[i] = Pos_ref[i];



            }

            ///------ Torque FeedForward --------------------------------------
            getTorqueRef();


            for(int i=0; i < body->numJoints(); ++i){
                jointInput[i] += Tau_ref[i];
            }


            ///------ Total Joint input------------------------------------------------
            for(int i=0; i<body->numJoints(); i++){
                Link* joint = body->joint(i);

                double jointTorque;

                if(q[i] >= lowerJLim[i]*D2R && q[i] <= upperJLim[i]*D2R){
                    jointTorque = LimitFunction(jointInput[i], TauLim[i], -TauLim[i]);
                }
                else if(q[i] < lowerJLim[i]*D2R){
                    jointTorque = (- dq[i])*20.0 + ((lowerJLim[i]*D2R + 0.0*D2R) - q[i])*1500;
                }
                else if(q[i] > upperJLim[i]*D2R){
                    jointTorque = (- dq[i])*20.0 + ((upperJLim[i]*D2R - 0.0*D2R) - q[i])*1500;
                }
                joint->u() = jointTorque;
            }


            setPosNow();
            setVelNow();

            ///-------------Force Sensor Calc ------------------------------------
            Isometry3 T_HRFT = body->link("HIND_R_FT")->position();
            Matrix3 R_HRFT = T_HRFT.rotation();
            Isometry3 T_HLFT = body->link("HIND_L_FT")->position();
            Matrix3 R_HLFT = T_HLFT.rotation();
            Isometry3 T_FRFT = body->link("FRONT_R_FT")->position();
            Matrix3 R_FRFT = T_FRFT.rotation();
            Isometry3 T_FLFT = body->link("FRONT_L_FT")->position();
            Matrix3 R_FLFT = T_FLFT.rotation();

            Vector6 FT_HRFT = pHRFT->F();
            Vector3 F_HRFT;
            Vector6 FT_HLFT = pHLFT->F();
            Vector3 F_HLFT;
            Vector6 FT_FRFT = pFRFT->F();
            Vector3 F_FRFT;
            Vector6 FT_FLFT = pFLFT->F();
            Vector3 F_FLFT;

            for(int i=0 ; i<3; i++){
                F_HRFT[i] = FT_HRFT[i];
                F_HLFT[i] = FT_HLFT[i];
                F_FRFT[i] = FT_FRFT[i];
                F_FLFT[i] = FT_FLFT[i];
            }

            Vector3 HRFT_global = R_HRFT*F_HRFT;
            Vector3 HLFT_global = R_HLFT*F_HLFT;
            Vector3 FRFT_global = R_FRFT*F_FRFT;
            Vector3 FLFT_global = R_FLFT*F_FLFT;

            for(int i=0 ; i<3; i++){
                simData->chorednoid_ReactionF_HR[i] = HRFT_global[i];
                simData->chorednoid_ReactionF_HL[i] = HLFT_global[i];
                simData->chorednoid_ReactionF_FR[i] = FRFT_global[i];
                simData->chorednoid_ReactionF_FL[i] = FLFT_global[i];

            }


            //pio->os()<< HRFT_global.x()<<", "<<HRFT_global.y()<<", "<<HRFT_global.z()<<endl;
            //pio->os()<< R_HRFT<<endl;

            if(cnt%2==0 && !(simData == (void*)-1)) // 500Hz
            {
                simData->TimeSyncFlag = 1;
                //pio->os()<<"1 tic"<<endl;
            }
            cnt++;

            // 마커 위치 업데이트

            Vector3 newPosition_zmpRef(simData->pos_zmpRef[0], simData->pos_zmpRef[1], simData->pos_zmpRef[2]); // 원하는 위치로 설정
            updatezmpRefPosition(R*newPosition_zmpRef + P);

            Vector3 newPosition_comRef(simData->pos_comRef[0], simData->pos_comRef[1], simData->pos_comRef[2]);
            updatecomRefPosition(R*newPosition_comRef + P);

            if(simData->CommandRef_swingState[0] > 0.2 && simData->CommandRef_swingState[0] < 0.6 && simData->CommandRef_contact[0] == 0){
                Vector3 newPosition_foot(simData->pos_pfoot0[0], simData->pos_pfoot0[1], simData->pos_pfoot0[2]);
                updatefootPosition0(R*newPosition_foot + P);
            }
            if(simData->CommandRef_swingState[1] > 0.2 && simData->CommandRef_swingState[1] < 0.6 && simData->CommandRef_contact[1] == 0){
                Vector3 newPosition_foot(simData->pos_pfoot1[0], simData->pos_pfoot1[1], simData->pos_pfoot1[2]);
                updatefootPosition1(R*newPosition_foot + P);
            }
            if(simData->CommandRef_swingState[2] > 0.2 && simData->CommandRef_swingState[2] < 0.6 && simData->CommandRef_contact[2] == 0){
                Vector3 newPosition_foot(simData->pos_pfoot2[0], simData->pos_pfoot2[1], simData->pos_pfoot2[2]);
                updatefootPosition2(R*newPosition_foot + P);
            }
            if(simData->CommandRef_swingState[3] > 0.2 && simData->CommandRef_swingState[3] < 0.6 && simData->CommandRef_contact[3] == 0){
                Vector3 newPosition_foot(simData->pos_pfoot3[0], simData->pos_pfoot3[1], simData->pos_pfoot3[2]);
                updatefootPosition3(R*newPosition_foot + P);
            }

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Exception in control: " << e.what() << std::endl;
            return false;
        }

    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RBQController)
