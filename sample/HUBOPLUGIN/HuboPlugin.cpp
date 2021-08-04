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

#include "/home/rainbow/Desktop/Humanoid_Research/share/Headers/RBSharedMemory.h"
#include "/home/rainbow/Desktop/Humanoid_Research/share/Headers/UserSharedMemory.h"
#include "/home/rainbow/Desktop/Humanoid_Research/share/Headers/JointInformation.h"

#include <iostream>
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


using namespace std;
using namespace cnoid;



   static double pgain[] = {
       8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
       8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
       8000.0, 8000.0, 8000.0,
       3000.0, 3000.0, 3000.0, 3000.0,
       3000.0, 3000.0, 3000.0, 3000.0,
        };

   static double dgain[] = {
       30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
       30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
       40.0, 40.0, 40.0,
       40.0, 40.0, 40.0, 40.0,
       40.0, 40.0, 40.0, 40.0,
        };
//pRBCORE_SHM sharedData;
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
using namespace std;

class HuboController : public cnoid::SimpleController
{
    BodyPtr body;
    //MultiValueSeqPtr qseq;
    vector<double> q0;
    vector<double> dq;
    int currentFrame;
    double timeStep_;
    int initcnt;

    Vector6 rfwrench_old;
    Vector6 lfwrench_old;


    ForceSensorPtr pLFFT, pRFFT;
    RateGyroSensorPtr pRG;
    AccelerationSensorPtr  pAS;

    bool shm_opened = false;
    int *GAZEBO_FLAG;

    double t0refs[100];
    double t0pos[100];
    double trefq[100];
    double refFromPODO[100];
    double oldrefFromPODO[100];
    int cnt;

    double Quat_IMU[4];

public:
    const static int NUM_INIT_STEP = 1000;
    virtual bool initialize(SimpleControllerIO* io) {

        for(int i=0; i<6;i++){
            rfwrench_old[0] = 0;
            lfwrench_old[0] = 0;
        }



        io->os() << "init start" << endl;


        timeStep_ = io->timeStep();


        body = io->body();
        q0.resize(body->numJoints());
        dq.resize(body->numJoints());

        io->os()<<"Device num: "<<body->numDevices()<<endl;
        //Device list?
        pRG = body->findDevice<RateGyroSensor>("gyrometer");
        pAS = body->findDevice<AccelerationSensor>("gsensor");
        pLFFT = body->findDevice<ForceSensor>("lfsensor");  //HSW
        pRFFT = body->findDevice<ForceSensor>("rfsensor");
        
        io->setLinkInput(body->rootLink(),LINK_POSITION);
        io->setJointInput(JOINT_ANGLE|JOINT_VELOCITY);
        io->setJointOutput(JOINT_TORQUE);

        for(int i=0; i < body->numJoints(); ++i){
            q0[i] = body->joint(i)->q();
            refFromPODO[i] = 0;
            oldrefFromPODO[i] = 0;

        }

        io->os() << "init done" << endl;

        Quat_IMU[0] = 1;
        Quat_IMU[1] = 0;
        Quat_IMU[2] = 0;
        Quat_IMU[3] = 0;

        double Y = 0;//90deg offset

        Quat_IMU[0] = cos(Y/2);
        Quat_IMU[1] = 0;
        Quat_IMU[2] = 0;
        Quat_IMU[3] = sin(Y/2);

        cnt = 0;

//        io->os() <<"CHEST: "<<CHEST <<" "<<body->numJoints()<<endl;  //HSW
        initcnt = NUM_INIT_STEP;
        return true;
    }

    double getRefPODO(int in)
    {
        double a;

        if(sharedCMD->Choreonoid.TorqueConOnOff == 1){
            //a = sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentTorqueRef;
            a = sharedCMD->Choreonoid.Torque[in];

        }
        else{
            a = sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentReference*M_PI/180.;
//            if(in==LEB) a+=-20*M_PI/180.;
//            if(in==REB) a+=-20*M_PI/180.;
//            if(in==RSR) a+=-15*M_PI/180.;
//            if(in==LSR) a+=+15*M_PI/180.;
        }
         return a;
    }
    void setPosPODO(int in, double Pos)
    {
         double a = Pos*180.0/M_PI;
//         if(in==LEB) a+=+20;
//         if(in==REB) a+=+20;
//         if(in==RSR) a+=+15;
//         if(in==LSR) a+=-15;
         //sharedData->CurrentPosition[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch] = a;
         sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentPosition = a;
    }
    void setVelPODO(int in, double Vel)
    {
        double a =Vel*180.0/M_PI;

        sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentVelocity = a;
    }
    void PODO2ref()
    {
        for(int i = 0; i<NO_OF_JOINTS ; i++){
            refFromPODO[i] = 0;//getRefPODO(i);
        }

    }
    void pos2PODO()
    {
        for(int i = 0; i<NO_OF_JOINTS ; i++){
            setPosPODO(i,body->joint(i)->q());
            setVelPODO(i,dq[i]);
        }
    }
    virtual bool control() {

        Vector3 Rot = pRG->w();
        Vector3 Acc = pAS->dv();
        Vector6 rfwrench = pRFFT->F();  //HSW
        Vector6 lfwrench = pLFFT->F();
        
        Position T = body->rootLink()->position();
        Vector3 P = T.translation();
        Matrix3 R = T.rotation();
        Quaterniond Qt(R);
//        io->os() << P << endl;
        //io->os() << Qt.w() <<" "<<Qt.x() <<" "<<Qt.y() <<" "<<Qt.z() <<" "<<endl;


        //test_give force to robot
        //Link* WLptr= body->link("WAIST");
        //io->os()<<WLptr->index()<< " f_ext "<<WLptr->f_ext()  <<endl;
        //WLptr->f_ext() =Vector3(200,0,0);
        //not working......why???????

        //setExternalForce(body,WLptr,Vector3(0,0,0),Vector3(200,0,0),0.1);
        //how to apply????


        //
        if(shm_opened)
        {
            
            sharedCMD->Choreonoid.pPel_3x1[0] = P[0];
            sharedCMD->Choreonoid.pPel_3x1[1] = P[1];
            sharedCMD->Choreonoid.pPel_3x1[2] = P[2];
            
            sharedCMD->Choreonoid.qPel_4x1[0] = Qt.w();
            sharedCMD->Choreonoid.qPel_4x1[1] = Qt.x();
            sharedCMD->Choreonoid.qPel_4x1[2] = Qt.y();
            sharedCMD->Choreonoid.qPel_4x1[3] = Qt.z();

            PODO2ref();
            //and put encoder value here;
            pos2PODO();
            //no time sync yet
            //and some sensor values
            double alpha = 1/(1 + 2*3.14*0.001*90);

            sharedSEN->FT[0].Fx = alpha*rfwrench_old[0] + (1-alpha)*rfwrench[0];  //HSW
            sharedSEN->FT[0].Fy = alpha*rfwrench_old[1] + (1-alpha)*rfwrench[1];
            sharedSEN->FT[0].Fz = alpha*rfwrench_old[2] + (1-alpha)*rfwrench[2];
            sharedSEN->FT[0].Mx = alpha*rfwrench_old[3] + (1-alpha)*rfwrench[3];
            sharedSEN->FT[0].My = alpha*rfwrench_old[4] + (1-alpha)*rfwrench[4];
            sharedSEN->FT[0].Mz = alpha*rfwrench_old[5] + (1-alpha)*rfwrench[5];

            sharedSEN->FT[1].Fx = alpha*lfwrench_old[0] + (1-alpha)*rfwrench[0];
            sharedSEN->FT[1].Fy = alpha*lfwrench_old[1] + (1-alpha)*lfwrench[1];
            sharedSEN->FT[1].Fz = alpha*lfwrench_old[2] + (1-alpha)*lfwrench[2];
            sharedSEN->FT[1].Mx = alpha*lfwrench_old[3] + (1-alpha)*lfwrench[3];
            sharedSEN->FT[1].My = alpha*lfwrench_old[4] + (1-alpha)*lfwrench[4];
            sharedSEN->FT[1].Mz = alpha*lfwrench_old[5] + (1-alpha)*lfwrench[5];

            rfwrench_old = rfwrench;
            lfwrench_old = lfwrench;


            double velX = Rot[0];
            double velY = Rot[1];
            double velZ = Rot[2];


            double delX = velX + velY*sin((sharedSEN->IMU[0].Roll)*D2R)*tan(sharedSEN->IMU[0].Pitch*D2R) + velZ*cos(sharedSEN->IMU[0].Roll*D2R)*tan(sharedSEN->IMU[0].Pitch*D2R);
            double delY = velY*cos(sharedSEN->IMU[0].Roll*D2R) - velZ*sin(sharedSEN->IMU[0].Roll*D2R);
            double delZ = velY*sin(sharedSEN->IMU[0].Roll*D2R)/(cos(sharedSEN->IMU[0].Pitch*D2R)) + velZ*cos(sharedSEN->IMU[0].Roll*D2R)/(cos(sharedSEN->IMU[0].Pitch*D2R));

            if(isnan(delX)) delX = 0;
            if(isnan(delY)) delY = 0;
            if(isnan(delZ)) delZ = 0;

            // quaternion(20170411)
            double l = sqrt(velX*velX + velY*velY + velZ*velZ);
            double p[4] = {cos(0.001*l/2.0),velX*sin(0.001*l/2.0)/l,velY*sin(0.001*l/2.0)/l,velZ*sin(0.001*l/2.0)/l};
            double Q[4];

            for(int i=0;i<4;i++)
            {
                    if(isnan(p[i]))
                    {
                            p[0] = 1;p[1] = 0;p[2] = 0;p[3] = 0;
                            //io->os() << "p - isnan" << endl;
                            break;
                    }
                    if(isnan(Quat_IMU[i]))
                    {
                            Quat_IMU[0] = 1;Quat_IMU[1] = 0;Quat_IMU[2] = 0;Quat_IMU[3] = 0;
                            //io->os() << "QQQQ - isnan" << endl;
                            break;
                    }
            }

            Q[0] = Quat_IMU[0]*p[0] - Quat_IMU[1]*p[1] - Quat_IMU[2]*p[2] - Quat_IMU[3]*p[3];//hamiltonian form
            Q[1] = Quat_IMU[0]*p[1] + Quat_IMU[1]*p[0] + Quat_IMU[2]*p[3] - Quat_IMU[3]*p[2];
            Q[2] = Quat_IMU[0]*p[2] - Quat_IMU[1]*p[3] + Quat_IMU[2]*p[0] + Quat_IMU[3]*p[1];
            Q[3] = Quat_IMU[0]*p[3] + Quat_IMU[1]*p[2] - Quat_IMU[2]*p[1] + Quat_IMU[3]*p[0];

            //Q[0] = Quat_IMU[0]*p[0] - Quat_IMU[1]*p[1] - Quat_IMU[2]*p[2] - Quat_IMU[3]*p[3];//JPL form
            //Q[1] = Quat_IMU[0]*p[1] + Quat_IMU[1]*p[0] - Quat_IMU[2]*p[3] + Quat_IMU[3]*p[2];
            //Q[2] = Quat_IMU[0]*p[2] + Quat_IMU[1]*p[3] + Quat_IMU[2]*p[0] - Quat_IMU[3]*p[1];
            //Q[3] = Quat_IMU[0]*p[3] - Quat_IMU[1]*p[2] + Quat_IMU[2]*p[1] + Quat_IMU[3]*p[0];

            double ll = sqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);

            Quat_IMU[0] = Q[0]/ll;
            Quat_IMU[1] = Q[1]/ll;
            Quat_IMU[2] = Q[2]/ll;
            Quat_IMU[3] = Q[3]/ll;

            sharedSEN->IMU[0].RollVel = delX;
            sharedSEN->IMU[0].PitchVel = delY;
            sharedSEN->IMU[0].YawVel = delZ;   //gloabal Gyro velocity

            //Quaternion Integration
            sharedSEN->IMU[0].Yaw = R2D*atan2(2.0*(Q[1]*Q[2] + Q[0]*Q[3]),Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3]);//Q[1];//delX*0.002;//0.001;
            sharedSEN->IMU[0].Pitch = R2D*asin(-2.0*(Q[1]*Q[3] - Q[0]*Q[2]));//Q[2];//delY*0.002;//0.001;
            sharedSEN->IMU[0].Roll =R2D*atan2(2.0*(Q[2]*Q[3] + Q[0]*Q[1]) , Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3]);//Q[3];//delZ*0.002;//0.001;

            sharedSEN->IMU[0].AccX = Acc[0];
            sharedSEN->IMU[0].AccY = Acc[1];
            sharedSEN->IMU[0].AccZ = Acc[2];

            sharedSEN->FOG.RollVel = sharedSEN->IMU[0].RollVel;
            sharedSEN->FOG.PitchVel = sharedSEN->IMU[0].PitchVel;
            sharedSEN->FOG.YawVel = sharedSEN->IMU[0].YawVel;
            sharedSEN->FOG.Roll = sharedSEN->IMU[0].Roll;
            sharedSEN->FOG.Pitch = sharedSEN->IMU[0].Pitch;
            sharedSEN->FOG.Yaw = sharedSEN->IMU[0].Yaw;

            sharedSEN->IMU[0].Q[0] = Q[0]/ll;
            sharedSEN->IMU[0].Q[1] = Q[1]/ll;
            sharedSEN->IMU[0].Q[2] = Q[2]/ll;
            sharedSEN->IMU[0].Q[3] = Q[3]/ll;


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
//            io->os()<<t0refs[CHEST]<<refFromPODO[CHEST]<<endl;  //HSW
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
                double _dq = (q - q0[i]) / timeStep_;
                dq_ref = 0;
                //joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];   //okkee
                joint->u() = (q_ref - q) * pgain[i] + (dq_ref - _dq) * dgain[i];
                q0[i] = q;
                trefq[i] = q_ref;
            }
            return true;
        }



        for(int i=0; i < body->numJoints(); ++i){


            Link* joint = body->joint(i);
            double q_ref = 0;//refFromPODO[i];
            double q = joint->q();
            dq[i] = (q - q0[i]) / timeStep_;

            double dq_ref = 0;//(q_ref - oldrefFromPODO[i]) / timeStep_;
            double _dq = dq[i];
            joint->u() = (q_ref - q) * pgain[i] + (dq_ref - _dq) * dgain[i];

            q0[i] = q;

        }

        for(int i=0; i<100; i++)
        {
               oldrefFromPODO[i] = refFromPODO[i];
        }




//        if(cnt%5==0&&!(GAZEBO_FLAG == (void*)-1)) // 200Hz
//        {
//            *GAZEBO_FLAG = 1;
//        }
        cnt++;
        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HuboController)
