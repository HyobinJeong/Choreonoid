/**
   Sample walking motion controller for the SR1 robot model.
   This program was ported from the "SamplePD" sample of OpenHRP3.
   @author Shin'ichiro Nakaoka
*/

/**

  try connect to PODO

  OKKEE SIM
 */
//#include "../Desktop/OKKEEWALK_0526/SHARE/Headers/RBSharedMemory.h"
//#include "../../../Desktop/podo_nrl/src/ALPrograms/ALTutorial/BasicFiles/BasicSetting.h"

#include "../../../Desktop/podo_nrl/share/Headers/RBSharedMemory.h"
#include "../../../Desktop/podo_nrl/share/Headers/UserSharedMemory.h"

#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/Sensor>

const static double D2R = 1.0/180.0*3.1415926535897;
const static double R2D = 1.0/D2R;

    enum joint_testrobot
    {
        RLEG_HIP_R,
        RLEG_HIP_P,
        RLEG_HIP_Y,
        RLEG_KNEE,
        RLEG_ANKLE_P,
        RLEG_ANKLE_R,

        RARM_SHOULDER_P,
        RARM_SHOULDER_R,
        RARM_SHOULDER_Y,
        RARM_ELBOW,
        RARM_WRIST_Y,
        RARM_WRIST_P,
        RARM_WRIST_Y2,

        LLEG_HIP_R,
        LLEG_HIP_P,
        LLEG_HIP_Y,
        LLEG_KNEE,
        LLEG_ANKLE_P,
        LLEG_ANKLE_R,

        LARM_SHOULDER_P,
        LARM_SHOULDER_R,
        LARM_SHOULDER_Y,
        LARM_ELBOW,
        LARM_WRIST_Y,
        LARM_WRIST_P,
        LARM_WRIST_Y2,

        CHEST,
    };
    enum JNums
    {
        RHY = 0, RHR, RHP, RKN, RAP, RAR,
        LHY, LHR, LHP, LKN, LAP, LAR,
        RSP, RSR, RSY, REB, RWY, RWP,
        LSP, LSR, LSY, LEB, LWY, LWP,
        WST,
        RF1, RF2, LF1, LF2,
        RWH, LWH, NO_OF_JOINTS
    };


   const struct {
       int id;
       int ch;
   } MC_ID_CH_Pairs[NO_OF_JOINTS] = {
       {0,0}, {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
       {6,0}, {7,0}, {8,0}, {9,0}, {10,0},{11,0},
       {13,0}, {14,0}, {15,0}, {15,1}, {16,0},{16,1},
       {17,0},{18,0},{19,0},{19,1},{20,0},{20,1},
       {12,0},
       {21,0},{21,1},{22,0},{22,1},
       {4,1}, {10,1}
   };



using namespace std;
using namespace cnoid;

static double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0 };

static double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };


//pRBCORE_SHM sharedData;
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
using namespace std;
#include <iostream>
class HuboController : public cnoid::SimpleController
{
    BodyPtr body;
    //MultiValueSeqPtr qseq;
    vector<double> q0;
    int currentFrame;
    double timeStep_;
    int initcnt;

    ForceSensorPtr pLFFT, pRFFT;
    RateGyroSensorPtr pRG;
    AccelerationSensorPtr  pAS;

    bool shm_opened;
    int *GAZEBO_FLAG;

    double t0refs[100];
    double t0pos[100];
    double trefq[100];
    double refFromPODO[100];
    double oldrefFromPODO[100];
    int cnt;
public:
    const static int NUM_INIT_STEP = 1000;
    virtual bool initialize() {
        //===========================shared memory=================================//
        shm_opened = false;

        // Core Shared Memory Creation [Reference]==================================
        int shmFD = shm_open(RBCORE_SHM_NAME_REFERENCE, O_RDWR, 0666);
        if(shmFD == -1){
            os() << "Fail to open core shared memory [Reference]";
           // return false;
        }else{
            if(ftruncate(shmFD, sizeof(RBCORE_SHM_REFERENCE)) == -1){
                os() << "Fail to truncate core shared memory [Reference]";
               // return false;
            }else{
                sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
                if(sharedREF == (void*)-1){
                    os() << "Fail to mapping core shared memory [Reference]";
                    //return false;
                }
            }
        }
        os() << "Core shared memory creation = OK [Reference]";
        // =========================================================================

        // Core Shared Memory Creation [Sensor]=====================================
        shmFD = shm_open(RBCORE_SHM_NAME_SENSOR, O_RDWR, 0666);
        if(shmFD == -1){
            os() << "Fail to open core shared memory [Sensor]";
           // return false;
        }else{
            if(ftruncate(shmFD, sizeof(RBCORE_SHM_SENSOR)) == -1){
                os() << "Fail to truncate core shared memory [Sensor]";
                //return false;
            }else{
                sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
                if(sharedSEN == (void*)-1){
                    os() << "Fail to mapping core shared memory [Sensor]";
                   // return false;
                }
            }
        }
        os() << "Core shared memory creation = OK [Sensor]";
        // =========================================================================

        // Core Shared Memory Creation [Command]====================================
        shmFD = shm_open(RBCORE_SHM_NAME_COMMAND, O_RDWR, 0666);
        if(shmFD == -1){
            os() << "Fail to open core shared memory [Command]";
           // return false;
        }else{
            if(ftruncate(shmFD, sizeof(RBCORE_SHM_COMMAND)) == -1){
                os() << "Fail to truncate core shared memory [Command]";
               // return false;
            }else{
                sharedCMD = (pRBCORE_SHM_COMMAND)mmap(0, sizeof(RBCORE_SHM_COMMAND), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
                if(sharedCMD == (void*)-1){
                    os() << "Fail to mapping core shared memory [Command]";
                    //return false;
                }
            }
        }
        os() << "Core shared memory creation = OK [Command]";
        // =========================================================================



        // User Shared Memory Creation ============================================
        shmFD = shm_open(USER_SHM_NAME, O_RDWR, 0666);
        if(shmFD == -1){
            os() << "Fail to open user shared memory";
            //return false;
        }else{
            if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
                os() << "Fail to truncate user shared memory";
                //return false;
            }else{
                userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
                if(userData == (void*)-1){
                    os() << "Fail to mapping user shared memory";
                   //return false;
                }
            }
        }
        if(userData != (void*)-1 && sharedCMD != (void*)-1 && sharedSEN != (void*)-1 &&sharedREF != (void*)-1)
        {
            shm_opened = true;
        }
        os() << "User shared memory creation = OK";
        // =========================================================================

//        shmFD = shm_open("GAZEBO_SHM", O_RDWR, 0666);
//        if(shmFD == -1){
//            os() << "Fail to open GAZEBO shared memory" <<endl;
//        }else{
//            if(ftruncate(shmFD, sizeof(int)) == -1){
//                os()<< "Fail to truncate GAZEBO shared memory"<<endl;
//            }else{
//                GAZEBO_FLAG = (int *)mmap(0, sizeof(int), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
//                if(GAZEBO_FLAG == (void*)-1){
//                    os() << "Fail to mapping GAZEBO shared memory"<<endl;
//                }
//            }
//        }
        os() << "GAZEBO shared memory creation = OK"<<endl;
       // shm_opened = false;
        //========================================================================//
        if(shm_opened)
        {
            //sharedData->IMURoll[0]=0;
            //sharedData->IMUPitch[0]=0;
            //sharedData->IMUYaw[0]=0;

            sharedSEN->IMU[0].Roll = 0;
            sharedSEN->IMU[0].Pitch = 0;
            sharedSEN->IMU[0].Yaw = 0;

        }

        os() << "init start" << endl;


        timeStep_ = timeStep();


        body = ioBody();
        q0.resize(body->numJoints());

        os()<<"Device num: "<<body->numDevices()<<endl;
        //Device list?
        pRG = body->findDevice<RateGyroSensor>("gyrometer");
        pAS = body->findDevice<AccelerationSensor>("gsensor");
        pLFFT = body->findDevice<ForceSensor>("lfsensor");
        pRFFT = body->findDevice<ForceSensor>("rfsensor");

        for(int i=0; i < body->numJoints(); ++i){
            q0[i] = body->joint(i)->q();
            refFromPODO[i] = 0;
            oldrefFromPODO[i] = 0;

        }

        os() << "init done" << endl;

        os() <<"CHEST: "<<CHEST <<" "<<body->numJoints()<<endl;
        initcnt = NUM_INIT_STEP;
        return true;
    }

    double getRefPODO(int in)
    {
         //double a = sharedData->CurrentJointReference[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch]*M_PI/180.;
         double a = sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentReference*M_PI/180.;
         if(in==LEB) a+=-20*M_PI/180.;
         if(in==REB) a+=-20*M_PI/180.;
         if(in==RSR) a+=-15*M_PI/180.;
         if(in==LSR) a+=+15*M_PI/180.;
         return a;
    }
    void setPosPODO(int in, double Pos)
    {
         double a = Pos*180.0/M_PI;
         if(in==LEB) a+=+20;
         if(in==REB) a+=+20;
         if(in==RSR) a+=+15;
         if(in==LSR) a+=-15;
         //sharedData->CurrentPosition[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch] = a;
         sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentPosition = a;
    }
    void PODO2ref()
    {
        refFromPODO[RLEG_HIP_Y] = getRefPODO(RHY);
        refFromPODO[RLEG_HIP_R] = getRefPODO(RHR);
        refFromPODO[RLEG_HIP_P] = getRefPODO(RHP);
        refFromPODO[RLEG_KNEE] = getRefPODO(RKN);
        refFromPODO[RLEG_ANKLE_P] = getRefPODO(RAP);
        refFromPODO[RLEG_ANKLE_R] = getRefPODO(RAR);

        refFromPODO[LLEG_HIP_Y] = getRefPODO(LHY);
        refFromPODO[LLEG_HIP_R] = getRefPODO(LHR);
        refFromPODO[LLEG_HIP_P] = getRefPODO(LHP);
        refFromPODO[LLEG_KNEE] = getRefPODO(LKN);
        refFromPODO[LLEG_ANKLE_P] = getRefPODO(LAP);
        refFromPODO[LLEG_ANKLE_R] = getRefPODO(LAR);

        refFromPODO[RARM_SHOULDER_P] = getRefPODO(RSP);
        refFromPODO[RARM_SHOULDER_R] = getRefPODO(RSR);
        refFromPODO[RARM_SHOULDER_Y] = getRefPODO(RSY);
        refFromPODO[RARM_ELBOW] = getRefPODO(REB);
        refFromPODO[RARM_WRIST_Y] = getRefPODO(RWY);
        refFromPODO[RARM_WRIST_P] = getRefPODO(RWP);
        refFromPODO[RARM_WRIST_Y2] = getRefPODO(RF1);//actually not same

        refFromPODO[LARM_SHOULDER_P] = getRefPODO(LSP);
        refFromPODO[LARM_SHOULDER_R] = getRefPODO(LSR);
        refFromPODO[LARM_SHOULDER_Y] = getRefPODO(LSY);
        refFromPODO[LARM_ELBOW] = getRefPODO(LEB);
        refFromPODO[LARM_WRIST_Y] = getRefPODO(LWY);
        refFromPODO[LARM_WRIST_P] = getRefPODO(LWP);
        refFromPODO[LARM_WRIST_Y2] = getRefPODO(LF1);//actually not same

        refFromPODO[CHEST] = getRefPODO(WST);


    }
    void pos2PODO()
    {
        setPosPODO(RHY,body->joint(RLEG_HIP_Y)->q());
        setPosPODO(RHR,body->joint(RLEG_HIP_R)->q());
        setPosPODO(RHP,body->joint(RLEG_HIP_P)->q());
        setPosPODO(RKN,body->joint(RLEG_KNEE)->q());
        setPosPODO(RAP,body->joint(RLEG_ANKLE_P)->q());
        setPosPODO(RAR,body->joint(RLEG_ANKLE_R)->q());

        setPosPODO(LHY,body->joint(LLEG_HIP_Y)->q());
        setPosPODO(LHR,body->joint(LLEG_HIP_R)->q());
        setPosPODO(LHP,body->joint(LLEG_HIP_P)->q());
        setPosPODO(LKN,body->joint(LLEG_KNEE)->q());
        setPosPODO(LAP,body->joint(LLEG_ANKLE_P)->q());
        setPosPODO(LAR,body->joint(LLEG_ANKLE_R)->q());

        setPosPODO(RSP,body->joint(RARM_SHOULDER_P)->q());
        setPosPODO(RSR,body->joint(RARM_SHOULDER_R)->q());
        setPosPODO(RSY,body->joint(RARM_SHOULDER_Y)->q());
        setPosPODO(REB,body->joint(RARM_ELBOW)->q());
        setPosPODO(RWY,body->joint(RARM_WRIST_Y)->q());
        setPosPODO(RWP,body->joint(RARM_WRIST_P)->q());
        setPosPODO(RF1,body->joint(RARM_WRIST_Y2)->q());

        setPosPODO(LSP,body->joint(LARM_SHOULDER_P)->q());
        setPosPODO(LSR,body->joint(LARM_SHOULDER_R)->q());
        setPosPODO(LSY,body->joint(LARM_SHOULDER_Y)->q());
        setPosPODO(LEB,body->joint(LARM_ELBOW)->q());
        setPosPODO(LWY,body->joint(LARM_WRIST_Y)->q());
        setPosPODO(LWP,body->joint(LARM_WRIST_P)->q());
        setPosPODO(LF1,body->joint(LARM_WRIST_Y2)->q());

        setPosPODO(WST,body->joint(CHEST)->q());

    }
    virtual bool control() {
        //here, convert PODO to refFromPODO


        Vector3 Rot = pRG->w();
        Vector3 Acc = pAS->dv();
        Vector6 rfwrench = pRFFT->F();
        Vector6 lfwrench = pLFFT->F();


        //test_give force to robot
        //Link* WLptr= body->link("WAIST");
        //os()<<WLptr->index()<< " f_ext "<<WLptr->f_ext()  <<endl;
        //WLptr->f_ext() =Vector3(200,0,0);
        //not working......why???????

        //setExternalForce(body,WLptr,Vector3(0,0,0),Vector3(200,0,0),0.1);
        //how to apply????


        //
        if(shm_opened)
        {
            PODO2ref();
            //and put encoder value here;
            pos2PODO();
            //no time sync yet
            //and some sensor values
//            sharedData->FTFx[0] = rfwrench[0];
//            sharedData->FTFy[0] = rfwrench[1];
//            sharedData->FTFz[0] = rfwrench[2];
//            sharedData->FTMx[0] = rfwrench[3];
//            sharedData->FTMy[0] = rfwrench[4];
//            sharedData->FTMz[0] = rfwrench[5];

//            sharedData->FTFx[1] = lfwrench[0];
//            sharedData->FTFy[1] = lfwrench[1];
//            sharedData->FTFz[1] = lfwrench[2];
//            sharedData->FTMx[1] = lfwrench[3];
//            sharedData->FTMy[1] = lfwrench[4];
//            sharedData->FTMz[1] = lfwrench[5];

            sharedSEN->FT[0].Fx = rfwrench[0];
            sharedSEN->FT[0].Fy = rfwrench[1];
            sharedSEN->FT[0].Fz = rfwrench[2];
            sharedSEN->FT[0].Mx = rfwrench[3];
            sharedSEN->FT[0].My = rfwrench[4];
            sharedSEN->FT[0].Mz = rfwrench[5];

            sharedSEN->FT[1].Fx = lfwrench[0];
            sharedSEN->FT[1].Fy = lfwrench[1];
            sharedSEN->FT[1].Fz = lfwrench[2];
            sharedSEN->FT[1].Mx = lfwrench[3];
            sharedSEN->FT[1].My = lfwrench[4];
            sharedSEN->FT[1].Mz = lfwrench[5];


            double velX = Rot[0];
            double velY = Rot[1];
            double velZ = Rot[2];

//            double delX = velX + velY*sin((sharedData->IMURoll[0])*D2R)*tan(sharedData->IMUPitch[0]*D2R) + velZ*cos(sharedData->IMURoll[0]*D2R)*tan(sharedData->IMUPitch[0]*D2R);
//            double delY = velY*cos(sharedData->IMURoll[0]*D2R) - velZ*sin(sharedData->IMURoll[0]*D2R);
//            double delZ = velY*sin(sharedData->IMURoll[0]*D2R)/(cos(sharedData->IMUPitch[0]*D2R)) + velZ*cos(sharedData->IMURoll[0]*D2R)/(cos(sharedData->IMUPitch[0]*D2R));

            double delX = velX + velY*sin((sharedSEN->IMU[0].Roll)*D2R)*tan(sharedSEN->IMU[0].Pitch*D2R) + velZ*cos(sharedSEN->IMU[0].Roll*D2R)*tan(sharedSEN->IMU[0].Pitch*D2R);
            double delY = velY*cos(sharedSEN->IMU[0].Roll*D2R) - velZ*sin(sharedSEN->IMU[0].Roll*D2R);
            double delZ = velY*sin(sharedSEN->IMU[0].Roll*D2R)/(cos(sharedSEN->IMU[0].Pitch*D2R)) + velZ*cos(sharedSEN->IMU[0].Roll*D2R)/(cos(sharedSEN->IMU[0].Pitch*D2R));

            if(isnan(delX)) delX = 0;
            if(isnan(delY)) delY = 0;
            if(isnan(delZ)) delZ = 0;


//            sharedData->IMURollVel[0] = delX;
//            sharedData->IMUPitchVel[0] = delY;
//            sharedData->IMUYawVel[0] = delZ;
            sharedSEN->IMU[0].RollVel = delX;
            sharedSEN->IMU[0].PitchVel = delY;
            sharedSEN->IMU[0].YawVel = delZ;

//            sharedData->IMURoll[0]+=delX*R2D*0.001;
//            sharedData->IMUPitch[0]+=delY*R2D*0.001;
//            sharedData->IMUYaw[0]+=delZ*R2D*0.001;

            sharedSEN->IMU[0].Roll +=delX*R2D*0.001;
            sharedSEN->IMU[0].Pitch +=delY*R2D*0.001;
            sharedSEN->IMU[0].Yaw +=delZ*R2D*0.001;



//            sharedData->IMUAccX[0] = Acc[0];
//            sharedData->IMUAccY[0] = Acc[1];
//            sharedData->IMUAccZ[0] = Acc[2];

            sharedSEN->IMU[0].AccX = Acc[0];
            sharedSEN->IMU[0].AccY = Acc[1];
            sharedSEN->IMU[0].AccZ = Acc[2];

//	    sharedData->FOGRollVel = sharedData->IMURollVel[0];
//	    sharedData->FOGPitchVel = sharedData->IMUPitchVel[0];
//	    sharedData->FOGYawVel = sharedData->IMUYawVel[0];
//	    sharedData->FOGRoll = sharedData->IMURoll[0];
//	    sharedData->FOGPitch = sharedData->IMUPitch[0];
//	    sharedData->FOGYaw = sharedData->IMUYaw[0];

            sharedSEN->FOG.RollVel = sharedSEN->IMU[0].RollVel;
            sharedSEN->FOG.PitchVel = sharedSEN->IMU[0].PitchVel;
            sharedSEN->FOG.YawVel = sharedSEN->IMU[0].YawVel;
            sharedSEN->FOG.Roll = sharedSEN->IMU[0].Roll;
            sharedSEN->FOG.Pitch = sharedSEN->IMU[0].Pitch;
            sharedSEN->FOG.Yaw = sharedSEN->IMU[0].Yaw;

        }
        else
        {
            for(int i=0;i<body->numJoints();i++)
            {
                refFromPODO[i] = 0.0;
            }
        }
        if(initcnt==NUM_INIT_STEP)
        {
            for(int i=0; i < body->numJoints(); ++i){
                Link* joint = body->joint(i);
                t0refs[i] = refFromPODO[i];
                t0pos[i] = joint->q();
                oldrefFromPODO[i] = refFromPODO[i];                
            }
            os()<<t0refs[CHEST]<<refFromPODO[CHEST]<<endl;
        }
        initcnt--;
        if(initcnt>0)
        {
            double alpha = (NUM_INIT_STEP*1.0-initcnt*1.0)/(NUM_INIT_STEP*1.0);

            for(int i=0; i < body->numJoints(); ++i){
                Link* joint = body->joint(i);
                double q_ref = t0pos[i]+(alpha)*(t0refs[i]-t0pos[i]);
                double q = joint->q();
                double dq_ref = (q_ref - oldrefFromPODO[i]) / timeStep_;
                double dq = (q - q0[i]) / timeStep_;
                dq_ref = 0;
                joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
                q0[i] = q;
                trefq[i] = q_ref;
            }
            return true;
        }



        for(int i=0; i < body->numJoints(); ++i){

            Link* joint = body->joint(i);
            double q_ref = refFromPODO[i];
            double q = joint->q();
            double dq_ref = (q_ref - oldrefFromPODO[i]) / timeStep_;
            double dq = (q - q0[i]) / timeStep_;
            joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            q0[i] = q;

        }

        for(int i=0; i<100; i++)
        {
               oldrefFromPODO[i] = refFromPODO[i];
        }

//        if(cnt%5==0&&!(GAZEBO_FLAG == (void*)-1))
//        {
//            *GAZEBO_FLAG = 1;
//        }
        cnt++;
        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HuboController)
