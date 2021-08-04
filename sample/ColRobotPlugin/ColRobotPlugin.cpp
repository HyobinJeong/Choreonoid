/**
   Sample walking motion controller for the SR1 robot model.
   This program was ported from the "SamplePD" sample of OpenHRP3.
   @author Shin'ichiro Nakaoka
*/

/**

  try connect to PODO

  OKKEE SIM
 */

#include "../../../Desktop/podo_gazelle_choreonoid//share/Headers/RBSharedMemory.h"
#include "../../../Desktop/podo_gazelle_choreonoid//share/Headers/UserSharedMemory.h"

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


pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;

class ColRobotController : public cnoid::SimpleController
{
    BodyPtr body;
    //MultiValueSeqPtr qseq;
    vector<double> q0;
    int currentFrame;
    double timeStep_;
    int pushcount, pushstate;
    int cnt;
    double pushforce;
    ForceSensorPtr pFT;

    bool shm_opened;
    int *GAZEBO_FLAG;

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

        shmFD = shm_open("GAZEBO_SHM", O_RDWR, 0666);
        if(shmFD == -1){
            os() << "Fail to open GAZEBO shared memory" <<endl;
        }else{
            if(ftruncate(shmFD, sizeof(int)) == -1){
                os()<< "Fail to truncate GAZEBO shared memory"<<endl;
            }else{
                GAZEBO_FLAG = (int *)mmap(0, sizeof(int), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
                if(GAZEBO_FLAG == (void*)-1){
                    os() << "Fail to mapping GAZEBO shared memory"<<endl;
                }
            }
        }
        os() << "GAZEBO shared memory creation = OK"<<endl;
        //========================================================================//

        pushcount = -1;
        pushstate = 0;
        os() << "init start" << endl;


        timeStep_ = timeStep();


        body = ioBody();
        q0.resize(body->numJoints());
        pFT = body->findDevice<ForceSensor>("fsensor");

        for(int i=0; i < body->numJoints(); ++i){
            q0[i] = body->joint(i)->q();

        }

        os() << "init done_colrobot" <<" "<<body->numJoints()<< endl;
        cnt = 0;
        return true;
    }

    virtual bool control() {
        //here, convert PODO to refFromPODO
        Vector6 fwrench = pFT->F();
        //
       // os()<<fwrench[0]<<" "<<fwrench[1]<<" "<<fwrench[2]<<" "
         //             <<fwrench[3]<<" "<<fwrench[4]<<" "<<fwrench[5]<<" "<<endl;
        if(shm_opened)
        {
            if(sharedCMD->COMMAND[MAX_AL-1].USER_COMMAND==222)
            {
                pushcount = sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[0];
                pushforce = sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[1];
                pushstate = 2;
                sharedCMD->COMMAND[MAX_AL-1].USER_COMMAND = 0;
            }
            sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[2] = fwrench[0];
            sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[3] = fwrench[1];
            sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[4] = fwrench[2];
            sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[5] = fwrench[3];
            sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[6] = fwrench[4];
            sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[7] = fwrench[5];
        }
        else
        {

        }

        if(pushstate==2)
        {
            cnt++;
            double speed = 5.0;//5m/s
            Link* joint = body->joint(1);

            joint->u() = 10.0*(speed*cnt*timeStep_-joint->q()) + 10.0*(speed-(joint->q()-q0[1])/timeStep_);//pd control
            //10N/m spring?
            q0[1] = joint->q();

            if(joint->q()>0.3&&(fabs(fwrench[0])>20||fabs(fwrench[1])>20))//not good enough
            {
                pushstate = 1;
                 os() << "pushforce" <<" "<<pushcount<< endl;
            }
        }
        else if(pushstate==1)
        {
            pushcount--;
            Link* joint = body->joint(1);
            joint->u() =pushforce;
            if(pushcount<0)
            {
                pushstate = 0;
            }
        }
        else
        {
            cnt--;

            double speed = 5.0;//1m/s
            if(cnt<0)
            {
                cnt = 0;
                speed = 0;
            }
            Link* joint = body->joint(1);
            joint->u() = 10.0*(speed*cnt*timeStep_-joint->q()) + 10.0*(-speed-(joint->q()-q0[1])/timeStep_);//pd control
            //slow comback
            q0[1] = joint->q();

        }
        if(pushcount<-100)
        {pushcount = -100;}

        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ColRobotController)
