// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email: valentina.gaggero@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __mcParserh__
#define __mcParserh__



#include <string>
#include <vector>
#include <map>

//  Yarp stuff
#include <yarp/os/Bottle.h>
//#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/ControlBoardHelper.h>

#include <yarp/dev/PidEnums.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include "EoMotionControl.h"
#include <yarp/os/LogStream.h>


// - public #define  --------------------------------------------------------------------------------------------------


namespace yarp {
    namespace dev  {
        namespace eomc {


//typedef enum
//{
//    PidAlgo_simple = 0,
//    PIdAlgo_velocityInnerLoop = 1,
//    PidAlgo_currentInnerLoop =2
//} PidAlgorithmType_t;


class PidInfo
{
public:

    yarp::dev::Pid pid;
    yarp::dev::PidFeedbackUnitsEnum fbk_PidUnits;
    yarp::dev::PidOutputUnitsEnum   out_PidUnits;
    eOmc_ctrl_out_type_t            out_type;

    //PidAlgorithmType_t controlLaw;

    std::string usernamePidSelected;
    bool enabled;

    PidInfo()//:usernamePidSelected("none")
    {
        enabled = false;
        //controlLaw = PidAlgo_simple;

        out_type = eomc_ctrl_out_type_n_a;
        fbk_PidUnits = yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
        out_PidUnits = yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS;
    }
    ~PidInfo()
    {
        //delete (usernamePidSelected);
    }

    //void dumpdata(void);

};

class TrqPidInfo : public PidInfo
{
public:
    double kbemf;                             /** back-emf compensation parameter */
    double ktau;                              /** motor torque constant */
    int    filterType;
};

class ControlLaw
{
public:
    yarp::dev::Pid pid;
    yarp::dev::Pid bc_pid;
    bool back_comp_2FOC_speed = false;

    yarp::dev::PidFeedbackUnitsEnum fbk_PidUnits;
    yarp::dev::PidOutputUnitsEnum   out_PidUnits;
    eOmc_ctrl_out_type_t            out_type;
    bool                            compliant = false;
    bool                            back_compatible = false;

    std::string boardname;
    std::string controlmode;

    double Kbemf = 0.0;
    double Ktau = 0.0;
    int filter_type = 0;

    ControlLaw(std::string& bn, std::string& cm) : boardname(bn), controlmode(cm)
    {
    }

    virtual ~ControlLaw() {}

    bool parseParameter(const std::string& key, yarp::os::Bottle& bottle, int joint, yarp::os::Value& value)
    {
        yarp::os::Bottle row = bottle.findGroup(key.c_str());
        
        if (row.isNull())
        {
            yError() << key + " parameter not found for controlmode "+ controlmode +" in board " + boardname + " in bottle " + bottle.toString();
            
            return false;
        }
        
        if (joint + 1 >= (int)row.size())
        {
            yError() << key + " incorrect number of entries for controlmode " + controlmode + " in board " + boardname + " in bottle " + bottle.toString();

            return false;
        }

        value = row.get(joint + 1);

        return true;
    }

    virtual bool parse(yarp::os::Bottle& control_law_bot, const int joint)
    {
        if (control_law_bot.isNull())
        {
            yError() << controlmode + " not found in board " + boardname + " configuration";

            return false;
        }

        back_compatible = false;
        
        compliant = false;

        // controlLaw

        yarp::os::Bottle cl_bot = control_law_bot.findGroup("controlLaw");

        if (cl_bot.isNull())
        {
            yError() << " controlLaw parameter missing for controlmode " + controlmode + " for board " + boardname + " joint " << joint;

            return false;
        }

        std::string cl = cl_bot.get(1).asString();
        std::string prefix = "";

        out_type = eomc_ctrl_out_type_pwm;

        if (cl == "minjerk")
        {
        }
        else if (cl == "direct")
        {
        }
        else if (cl == "torque")
        {
            compliant = true;
        }
        else if (cl == "low_lev_current")
        {
        }
        else if (cl == "low_lev_velocity")
        {
        }
        // legacy
        else if (cl == "Pid_inPos_outPwm")
        {
            back_compatible = true;
            prefix = "pos_";
        }
        else if (cl == "Pid_inTrq_outPwm")
        {
            back_compatible = true;
            prefix = "trq_";
            compliant = true;
        }
        else if (cl == "PidPos_withInnerVelPid")
        {
            out_type = eomc_ctrl_out_type_vel;
            back_comp_2FOC_speed = true;
            back_compatible = true;
            prefix = "pos_";
        }
        else if (cl == "limitscurrent")
        {
            back_compatible = true;
            prefix = "cur_";
        }
        //else if (cl == "PidTrq_withInnerVelPid")
        //{
        //}

        // outputType

        yarp::os::Bottle ot_bot = control_law_bot.findGroup("outputType");

        if (!ot_bot.isNull())
        {
            std::string ot = ot_bot.get(1).asString();

            if (ot == "pwm")
            {
                out_type = eomc_ctrl_out_type_pwm;
            }
            else if (ot == "current")
            {
                out_type = eomc_ctrl_out_type_cur;
            }
            else if (ot == "velocity")
            {
                out_type = eomc_ctrl_out_type_vel;
            }
            else
            {
                yError() << ot + " unknown outputType for controlmode " + controlmode + " for board " + boardname + " joint " << joint;

                return false;
            }
        }
        else
        {
            yWarning() << " outputType parameter missing for controlmode " + controlmode + " for board " + boardname + " joint " << joint << ", pwm assumed";
        }

        // fbkControlUnits

        yarp::os::Bottle cu_bot = control_law_bot.findGroup("fbkControlUnits");

        if (cu_bot.isNull())
        {
            yError() <<  " fbkControlUnits parameter missing for controlmode " + controlmode + " for board " + boardname + " joint " << joint;
        
            return false;
        }

        std::string cu = cu_bot.get(1).asString();

        if (cu == "metric_units")
        {
            fbk_PidUnits = PidFeedbackUnitsEnum::METRIC;
        }
        else if (cu == "machine_units")
        {
            fbk_PidUnits = PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
        }
        else
        {
            yError() << cu + " unknown fbkControlUnits for controlmode " + controlmode + " for board " + boardname + " joint " << joint;
            
            return false;
        }

        // outputControlUnits

        cu_bot = control_law_bot.findGroup("outputControlUnits");

        if (cu_bot.isNull())
        {
            yError() << " outputControlUnits parameter missing for controlmode " + controlmode + " for board " + boardname + " joint " << joint;

            return false;
        }

        cu = cu_bot.get(1).asString();

        if (cu == "machine_units")
        {
            out_PidUnits = PidOutputUnitsEnum::RAW_MACHINE_UNITS;
        }
        else if (cu == "dutycycle_percent")
        {
            out_PidUnits = PidOutputUnitsEnum::DUTYCYCLE_PWM_PERCENT;
        }
        else if (cu == "metric_units")
        {
            switch (out_type)
            {
                case eomc_ctrl_out_type_pwm:
                    out_PidUnits = PidOutputUnitsEnum::DUTYCYCLE_PWM_PERCENT;
                    break;
                
                case eomc_ctrl_out_type_cur:
                    out_PidUnits = PidOutputUnitsEnum::CURRENT_METRIC;
                    break;
                    
                case eomc_ctrl_out_type_vel:
                    out_PidUnits = PidOutputUnitsEnum::VELOCITY_METRIC;
                    break;
            }
        }
        else
        {
            yError() << cu + " unknown outputControlUnits for controlmode " + controlmode + " for board " + boardname + " joint " << joint;
            
            return false;
        }

        yarp::os::Value param;

        if (!parseParameter(prefix + "kff", control_law_bot, joint, param)) return false;
        pid.kff = param.asDouble();

        if (!parseParameter(prefix + "kp", control_law_bot, joint, param)) return false;
        pid.kp = param.asDouble();

        if (!parseParameter(prefix + "kd", control_law_bot, joint, param)) return false;
        pid.kd = param.asDouble();

        if (!parseParameter(prefix + "maxOutput", control_law_bot, joint, param)) return false;
        pid.max_output = param.asDouble();

        if (back_comp_2FOC_speed)
        {
            if (!parseParameter("vel_kff", control_law_bot, joint, param)) return false;
            bc_pid.kff = param.asDouble();

            if (!parseParameter("vel_kp", control_law_bot, joint, param)) return false;
            bc_pid.kp = param.asDouble();

            if (!parseParameter("vel_kd", control_law_bot, joint, param)) return false;
            bc_pid.kd = param.asDouble();

            if (!parseParameter("vel_maxOutput", control_law_bot, joint, param)) return false;
            bc_pid.max_output = param.asDouble();

            if (!parseParameter("vel_ki", control_law_bot, joint, param)) return false;
            bc_pid.ki = param.asDouble();

            if (!parseParameter("vel_maxInt", control_law_bot, joint, param)) return false;
            bc_pid.max_int = param.asDouble();

            if (!parseParameter("vel_shift", control_law_bot, joint, param)) return false;
            bc_pid.scale = param.asDouble();
        }

        if (out_type == eomc_ctrl_out_type_vel) return true;

        if (!parseParameter(prefix + "ki", control_law_bot, joint, param)) return false;
        pid.ki = param.asDouble();
        
        if (!parseParameter(prefix + "shift", control_law_bot, joint, param)) return false;
        pid.scale = param.asDouble();

        if (!parseParameter(prefix + "maxInt", control_law_bot, joint, param)) return false;
        pid.max_int = param.asDouble();

        if (!parseParameter(prefix + "stictionUp", control_law_bot, joint, param)) return false;
        pid.stiction_up_val = param.asDouble();

        if (!parseParameter(prefix + "stictionDown", control_law_bot, joint, param)) return false;
        pid.stiction_down_val = param.asDouble();


        if (!compliant) return true;

        if (!parseParameter(prefix + "kbemf", control_law_bot, joint, param)) return false;
        Kbemf = param.asDouble();

        if (!parseParameter(prefix + "ktau", control_law_bot, joint, param)) return false;
        Ktau = param.asDouble();

        if (!parseParameter(prefix + "filterType", control_law_bot, joint, param)) return false;
        filter_type = param.asInt();

        return true;
    }
};

typedef struct
{
    bool hasHallSensor;
    bool hasTempSensor;
    bool hasRotorEncoder;
    bool hasRotorEncoderIndex;
    int  rotorIndexOffset;
    int  motorPoles;
    bool hasSpeedEncoder ; //facoltativo
    bool verbose;
} twofocSpecificInfo_t;


class JointsSet
{
public:
    int           id; //num of set. it can be between 0 and max number of joint (_njoints)
    std::vector<int>   joints; //list of joints belongig to this set2joint

    eOmc_jointset_configuration_t cfg;

    JointsSet(int num=0)
    {
        id=num;
        joints.resize(0);
        cfg.candotorquecontrol=0;
        cfg.usespeedfeedbackfrommotors=0;
        cfg.pidoutputtype=eomc_pidoutputtype_unknown;
        cfg.dummy=0;
        cfg.constraints.type=eomc_jsetconstraint_unknown;
        cfg.constraints.param1=0;
        cfg.constraints.param2=0;
    }
public:
    int getNumberofJoints(void) {return (joints.size());}
    eOmc_jointset_configuration_t* getConfiguration(void) {return &cfg;}
    void setUseSpeedFeedbackFromMotors(bool flag){cfg.usespeedfeedbackfrommotors = flag;}
    void setPidOutputType(eOmc_pidoutputtype_t type){cfg.pidoutputtype = type;}
    void setCanDoTorqueControl(bool flag) {cfg.candotorquecontrol = flag;}
    void dumpdata();
};

typedef struct
{
    int velocity;
} timeouts_t;

typedef struct
{
    double nominalCurrent;
    double peakCurrent;
    double overloadCurrent;
} motorCurrentLimits_t;


typedef struct
{
    double posMin;                         /** user joint limits, max*/
    double posMax;                         /** user joint limits, min*/
    double posHwMax;                       /** hardaware joint limits, max */
    double posHwMin;                       /** hardaware joint limits, min */
    double velMax;
} jointLimits_t;

typedef struct
{
    double posMin;
    double posMax;
    double pwmMax;
} rotorLimits_t;


typedef struct
{
    std::vector<double>                  matrixJ2M;
    std::vector<double>                  matrixM2J;
    std::vector<double>                  matrixE2J;
} couplingInfo_t;

typedef struct
{
    int             mappedto;
    std::string     name;
    JointTypeEnum   type;
} axisInfo_t;

typedef struct
{
    double min_stiff;
    double max_stiff;
    double min_damp;
    double max_damp;
    double param_a;
    double param_b;
    double param_c;
} impedanceLimits_t;


typedef struct
{
    double stiffness;
    double damping;
    impedanceLimits_t limits;
} impedanceParameters_t;

//template <class T>

class Parser
{

private:
    int _njoints;
    std::string _boardname;
    bool _verbosewhenok;

    std::vector<ControlLaw*> positionControlLaw;
    std::vector<ControlLaw*> velocityControlLaw;
    std::vector<ControlLaw*> mixedControlLaw;
    std::vector<ControlLaw*> posDirectControlLaw;
    std::vector<ControlLaw*> velDirectControlLaw;
    std::vector<ControlLaw*> torqueControlLaw;
    std::vector<ControlLaw*> currentControlLaw;
    std::vector<ControlLaw*> speedControlLaw;

    //PID parsing functions
    void copyPids(PidInfo* pids, std::vector<ControlLaw*>& controlLaws);
    void copyPids(TrqPidInfo* pids, std::vector<ControlLaw*>& controlLaws);
    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size, bool mandatory = true);
    //bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size, bool mandatory = true);
    bool parseControlsGroup(yarp::os::Searchable &config, bool lowLevPidisMandatory);
    bool parseControlLaw(yarp::os::Searchable& config, yarp::os::Bottle& control_group_bot, std::vector<ControlLaw*>& controlLaw, bool mandatory = false);
    
    //bool getCorrectPidForEachJoint(PidInfo *ppids/*, PidInfo *vpids*/, TrqPidInfo *tpids);
    bool parsePidUnitsType(yarp::os::Bottle &bPid, yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits);


    bool convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror);
    bool convert(yarp::os::Bottle &bottle, std::vector<double> &matrix, bool &formaterror, int targetsize);

    //general utils functions
    template <class T>
    bool checkAndSetVectorSize(std::vector<T> &vec, int size, const std::string &funcName)
    {
        if(size > (int)vec.capacity())
        {
            yError() << "embObjMC BOARD " << _boardname << " in " <<  funcName.c_str() << ": try to insert " << size << "element in vector with " << vec.capacity() << " elements";
            return false;
        }

        vec.resize(size);
        return true;
    }

    ///////// DEBUG FUNCTIONS
    void debugUtil_printControlLaws(void);


public:
    Parser(int numofjoints, std::string boardname);
    ~Parser();

    bool parsePids(yarp::os::Searchable &config, PidInfo *ppids, PidInfo *dpids, TrqPidInfo *tpids, PidInfo *cpids, PidInfo *spids, bool lowLevPidisMandatory);
    //bool parsePids(yarp::os::Searchable &config, bool lowLevPidisMandatory);
    bool parse2FocGroup(yarp::os::Searchable &config, twofocSpecificInfo_t *twofocinfo);
    bool parseJointsetCfgGroup(yarp::os::Searchable &config, std::vector<JointsSet> &jsets, std::vector<int> &jointtoset);
    bool parseTimeoutsGroup(yarp::os::Searchable &config, std::vector<timeouts_t> &timeouts, int defaultVelocityTimeout);
    bool parseCurrentLimits(yarp::os::Searchable &config, std::vector<motorCurrentLimits_t> &currLimits);
    bool parseJointsLimits(yarp::os::Searchable &config, std::vector<jointLimits_t> &jointsLimits);
    bool parseRotorsLimits(yarp::os::Searchable &config, std::vector<rotorLimits_t> &rotorsLimits);
    bool parseCouplingInfo(yarp::os::Searchable &config, couplingInfo_t &couplingInfo);
    bool parseMotioncontrolVersion(yarp::os::Searchable &config, int &version);
    bool parseBehaviourFalgs(yarp::os::Searchable &config, bool &useRawEncoderData, bool  &pwmIsLimited );
    bool isVerboseEnabled(yarp::os::Searchable &config);
    bool parseAxisInfo(yarp::os::Searchable &config, int axisMap[], std::vector<axisInfo_t> &axisInfo);
    bool parseEncoderFactor(yarp::os::Searchable &config, double encoderFactor[]);
    bool parsefullscalePWM(yarp::os::Searchable &config, double dutycycleToPWM[]);
    bool parseAmpsToSensor(yarp::os::Searchable &config, double ampsToSensor[]);
    bool parseGearboxValues(yarp::os::Searchable &config, double gearbox_M2J[], double gearbox_E2J[]);
    bool parseMechanicalsFlags(yarp::os::Searchable &config, int useMotorSpeedFbk[]);
    bool parseImpedanceGroup(yarp::os::Searchable &config,std::vector<impedanceParameters_t> &impedance);
    bool parseDeadzoneValue(yarp::os::Searchable &config, double deadzone[], bool *found);
};

}}}; //close namespaces

#endif // include guard
