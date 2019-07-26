// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Valentina Gaggero
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <string.h>
#include <iostream>



#include <eomcParser.h>

#include <yarp/os/LogStream.h>

#include "EoCommon.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include <vector>


using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::dev::eomc;

yarp::dev::eomc::Parser::Parser(int numofjoints, string boardname)
{
    _njoints = numofjoints;
    _boardname = boardname;
    _verbosewhenok = 0;

    positionControlLaw.resize(numofjoints);
    velocityControlLaw.resize(numofjoints);
    mixedControlLaw.resize(numofjoints);
    posDirectControlLaw.resize(numofjoints);
    velDirectControlLaw.resize(numofjoints);
    torqueControlLaw.resize(numofjoints);
    currentControlLaw.resize(numofjoints);
    speedControlLaw.resize(numofjoints);

    for (int j = 0; j < numofjoints; ++j)
    {
        positionControlLaw[j] = nullptr;
        velocityControlLaw[j] = nullptr;
        mixedControlLaw[j] = nullptr;
        posDirectControlLaw[j] = nullptr;
        velDirectControlLaw[j] = nullptr;
        torqueControlLaw[j] = nullptr;
        currentControlLaw[j] = nullptr;
        speedControlLaw[j] = nullptr;
    }
}

Parser::~Parser()
{
    for (int j = 0; j < _njoints; ++j)
    {
        if (positionControlLaw[j]) delete positionControlLaw[j];
        if (velocityControlLaw[j]) delete velocityControlLaw[j];
        if (mixedControlLaw[j]) delete mixedControlLaw[j];
        if (posDirectControlLaw[j]) delete posDirectControlLaw[j];
        if (velDirectControlLaw[j]) delete velDirectControlLaw[j];
        if (torqueControlLaw[j]) delete torqueControlLaw[j];
        if (currentControlLaw[j]) delete currentControlLaw[j];
        if (speedControlLaw[j]) delete speedControlLaw[j];

        positionControlLaw[j] = nullptr;
        velocityControlLaw[j] = nullptr;
        mixedControlLaw[j] = nullptr;
        posDirectControlLaw[j] = nullptr;
        velDirectControlLaw[j] = nullptr;
        torqueControlLaw[j] = nullptr;
        currentControlLaw[j] = nullptr;
        speedControlLaw[j] = nullptr;
    }
}

bool Parser::extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size, bool mandatory)
{
    size++;
    Bottle &tmp = input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        std::string message = key1 + " parameter not found for board " + _boardname + " in bottle " + input.toString();
        if (mandatory)
            yError() << message.c_str();
        else
            yWarning() << message.c_str();
        return false;
    }

    if (tmp.size() != size)
    {
        yError() << key1.c_str() << " incorrect number of entries in BOARD " << _boardname;
        return false;
    }

    out = tmp;
    return true;
}

bool Parser::parsePids(yarp::os::Searchable &config, PidInfo *ppids, PidInfo *dpids, TrqPidInfo *tpids, PidInfo *cpids, PidInfo *spids, bool lowLevPidisMandatory)
{
    if(!parseControlsGroup(config, lowLevPidisMandatory)) return false;

    copyPids(ppids, positionControlLaw);
    copyPids(dpids, posDirectControlLaw);
    copyPids(tpids, torqueControlLaw);
    copyPids(cpids, currentControlLaw);
    copyPids(spids, speedControlLaw);
    
    for (int j=0; j<_njoints; ++j)
    {
        yDebug() << "speed[" << j << "]  " << speedControlLaw[j]->pid.kp << "  " << speedControlLaw[j]->pid.ki << "  " << speedControlLaw[j]->pid.scale;
        yDebug() << "veloc[" << j << "]  " << positionControlLaw[j]->pid.kp << "  " << positionControlLaw[j]->pid.ki << "  " << positionControlLaw[j]->pid.kff; 
    }

    return true;
}

bool Parser::parseControlsGroup(yarp::os::Searchable &config, bool lowLevPidisMandatory)
{
    yDebug() << "oooooooooooooooo 1";
    
    Bottle controls = config.findGroup("CONTROLS", "Configuration of used control laws ");
    if (controls.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " no CONTROLS group found in config file, returning";
        return false;
    }

    yDebug() << "oooooooooooooooo 2";

    Bottle control_group_bot = controls.findGroup("positionControl");

    if (control_group_bot.isNull())
    {
        yError() << " positionControl law not found for board " + _boardname + " in bottle " + controls.toString();

        return false;
    }
    
    yDebug() << "oooooooooooooooo 3";

    if (!parseControlLaw(config, control_group_bot, positionControlLaw, true)) return false;

    control_group_bot = controls.findGroup("velocityControl");

    yDebug() << "oooooooooooooooo 4";

    if (!control_group_bot.isNull())
    {
        if (!parseControlLaw(config, control_group_bot, velocityControlLaw)) return false;
    }
    else
    {
        yWarning() << " velocityControl law not found for board " + _boardname + " in bottle " + controls.toString();
    }

    control_group_bot = controls.findGroup("mixedControl");

    yDebug() << "oooooooooooooooo 5";

    if (!control_group_bot.isNull())
    {
        if (!parseControlLaw(config, control_group_bot, mixedControlLaw)) return false;
    }
    else
    {
        yWarning() << " mixedControlLaw law not found for board " + _boardname + " in bottle " + controls.toString();
    }

    control_group_bot = controls.findGroup("posDirectControl");

    yDebug() << "oooooooooooooooo 6";

    if (!control_group_bot.isNull())
    {
        if (!parseControlLaw(config, control_group_bot, posDirectControlLaw)) return false;
    }
    else
    {
        yWarning() << " posDirectControl law not found for board " + _boardname + " in bottle " + controls.toString();
    }

    control_group_bot = controls.findGroup("velDirectControl");

    yDebug() << "oooooooooooooooo 7";

    if (!control_group_bot.isNull())
    {
        if (!parseControlLaw(config, control_group_bot, velDirectControlLaw)) return false;
    }
    else
    {
        //yWarning() << " velDirectControl law not found for board " + _boardname + " in bottle " + controls.toString();
    }

    control_group_bot = controls.findGroup("torqueControl");

    yDebug() << "oooooooooooooooo 8";

    if (!control_group_bot.isNull())
    {
        if (!parseControlLaw(config, control_group_bot, torqueControlLaw)) return false;
    }
    else
    {
        yWarning() << " torqueControl law not found for board " + _boardname + " in bottle " + controls.toString();
    }

    if (!lowLevPidisMandatory) return true;

    control_group_bot = controls.findGroup("currentPid");

    yDebug() << "oooooooooooooooo 9";

    if (!control_group_bot.isNull())
    {
        if (!parseControlLaw(config, control_group_bot, currentControlLaw, true)) return false;
    }
    else
    {
        yError() << " currentPid law not found for board " + _boardname + " in bottle " + controls.toString();

        return false;
    }

    yDebug() << "oooooooooooooooo 10";

    if (!positionControlLaw[0]->back_compatible)
    {
        control_group_bot = controls.findGroup("speedPid");

        if (!control_group_bot.isNull())
        {
            if (!parseControlLaw(config, control_group_bot, speedControlLaw, true)) return false;
        }
        else
        {
            yError() << " speedPid law not found for board " + _boardname + " in bottle " + controls.toString();
        
            return false;
        }

        yDebug() << "oooooooooooooooo 11";

        return true;
    }

    // back compatibility junk

    yDebug() << "oooooooooooooooo 12";

    for (int j = 0; j < _njoints; ++j)
    {
        if (positionControlLaw[j])
        {
            if (positionControlLaw[j]->back_comp_2FOC_speed)
            {
                std::string sControlLaw("low_lev_velocity");

                if (speedControlLaw[j]) delete speedControlLaw[j];

                speedControlLaw[j] = new ControlLaw(_boardname, sControlLaw);
                speedControlLaw[j]->pid = positionControlLaw[j]->bc_pid;
                speedControlLaw[j]->back_compatible = true;
                speedControlLaw[j]->compliant = false;
                speedControlLaw[j]->fbk_PidUnits = PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
                speedControlLaw[j]->out_PidUnits = PidOutputUnitsEnum::RAW_MACHINE_UNITS;
            }
        }
        else
        {
            if (speedControlLaw[j]) delete speedControlLaw[j];

            speedControlLaw[j] = nullptr;
        }
    }

    yDebug() << "oooooooooooooooo 13";

    return true;
}

bool Parser::parseControlLaw(yarp::os::Searchable& config, yarp::os::Bottle& control_group_bot, std::vector<ControlLaw*>& controlLaw, bool mandatory)
{

    yDebug() << "................. 1";

    if (_njoints != control_group_bot.size() - 1)
    {
        yError() << " incorrect number of entries in BOARD " + _boardname + " in bottle " + control_group_bot.toString();

        return false;
    }

    yDebug() << "................. 2";

    for (int j = 0; j < _njoints; ++j)
    {
        std::string sControlLaw = control_group_bot.get(j + 1).asString();

        yDebug() << "................. 3," << j;

        if (sControlLaw == "none" || sControlLaw == "NONE")
        {
            if (mandatory)
            {
                return false;
            }
            else
            {
                continue;
            }
        }
        
        yDebug() << "................. 4," << j;

        controlLaw[j] = new ControlLaw(_boardname, sControlLaw);

        if (!controlLaw[j]->parse(config.findGroup(sControlLaw), j)) return false;
        
        yDebug() << "................. 5," << j;
    }

    yDebug() << "................. 6";

    return true;
}

void Parser::copyPids(PidInfo* pids, std::vector<ControlLaw*>& controlLaws)
{
    for (int j = 0; j < _njoints; ++j)
    {
        if (controlLaws[j])
        {
            pids[j].pid = controlLaws[j]->pid;
            pids[j].fbk_PidUnits = controlLaws[j]->fbk_PidUnits;
            pids[j].out_PidUnits = controlLaws[j]->out_PidUnits;
            pids[j].out_type = controlLaws[j]->out_type;
            pids[j].usernamePidSelected = controlLaws[j]->controlmode;
        }
        else
        {
            pids[j].enabled = false;
        }
    }
}

void Parser::copyPids(TrqPidInfo* pids, std::vector<ControlLaw*>& controlLaws)
{
    copyPids((PidInfo*)pids, controlLaws);
    
    for (int j = 0; j < _njoints; ++j)
    {
        if (controlLaws[j])
        {
            pids[j].ktau  = controlLaws[j]->Ktau;
            pids[j].kbemf = controlLaws[j]->Kbemf;
            pids[j].filterType = controlLaws[j]->filter_type;
        }
    }
}

#if 0
bool Parser::getCorrectPidForEachJoint(PidInfo *ppids/*, PidInfo *vpids*/, TrqPidInfo *tpids)
{
    Pid_Algorithm *minjerkAlgo_ptr = NULL;
    //Pid_Algorithm *directAlgo_ptr = NULL;
    Pid_Algorithm *torqueAlgo_ptr = NULL;

    //since some joints could not have all pid configured, reset pid values to 0.
    memset(ppids, 0, sizeof(PidInfo)*_njoints);
    //memset(vpids, 0, sizeof(PidInfo)*_njoints);
    memset(tpids, 0, sizeof(TrqPidInfo)*_njoints);

    map<string, Pid_Algorithm*>::iterator it;

    for (int i = 0; i < _njoints; i++)
    {
        //get position pid
        it = minjerkAlgoMap.find(_positionControlLaw[i]);
        if (it == minjerkAlgoMap.end())
        {
            yError() << "embObjMC BOARD " << _boardname << "Cannot find " << _positionControlLaw[i].c_str() << "in parsed pos pid";
            return false;
        }

        minjerkAlgo_ptr = minjerkAlgoMap[_positionControlLaw[i]];

        ppids[i].pid = minjerkAlgo_ptr->getPID(i);
        ppids[i].fbk_PidUnits = minjerkAlgo_ptr->fbk_PidUnits;
        ppids[i].out_PidUnits = minjerkAlgo_ptr->out_PidUnits;
        //ppids[i].controlLaw =  minjerkAlgo_ptr->type;
        ppids[i].out_type = minjerkAlgo_ptr->out_type;
        ppids[i].usernamePidSelected = _positionControlLaw[i];
        ppids[i].enabled = true;

        /*
        //get velocity pid
        if (_posDirectControlLaw[i] == "none")
        {
            directAlgo_ptr = NULL;
        }
        else
        {
            it = directAlgoMap.find(_posDirectControlLaw[i]);
            if (it == directAlgoMap.end())
            {
                yError() << "embObjMC BOARD " << _boardname  << "Cannot find " << _posDirectControlLaw[i].c_str() << "in parsed vel pid";
                return false;
            }

            directAlgo_ptr = directAlgoMap[_posDirectControlLaw[i]];
        }

        if (directAlgo_ptr)
        {
            vpids[i].pid = directAlgo_ptr->getPID(i);
            vpids[i].fbk_PidUnits = directAlgo_ptr->fbk_PidUnits;
            vpids[i].out_PidUnits = directAlgo_ptr->out_PidUnits;
            //vpids[i].controlLaw = directAlgo_ptr->type;
            vpids[i].out_type = directAlgo_ptr->out_type;
            vpids[i].usernamePidSelected = _posDirectControlLaw[i];
            vpids[i].enabled = true;
        }
        else
        {
            vpids[i].enabled = false;
            vpids[i].usernamePidSelected = "none";
        }
        */

        //get torque pid
        if (_torqueControlLaw[i] == "none")
        {
            torqueAlgo_ptr = NULL;
        }
        else
        {
            it = torqueAlgoMap.find(_torqueControlLaw[i]);
            if (it == torqueAlgoMap.end())
            {
                yError() << "embObjMC BOARD " << _boardname << "Cannot find " << _torqueControlLaw[i].c_str() << "in parsed trq pid";
                return false;
            }

            torqueAlgo_ptr = torqueAlgoMap[_torqueControlLaw[i]];
        }

        if (torqueAlgo_ptr)
        {
            tpids[i].pid = torqueAlgo_ptr->getPID(i);
            tpids[i].fbk_PidUnits = torqueAlgo_ptr->fbk_PidUnits;
            tpids[i].out_PidUnits = torqueAlgo_ptr->out_PidUnits;
            //tpids[i].controlLaw = torqueAlgo_ptr->type;
            tpids[i].out_type = torqueAlgo_ptr->out_type;
            tpids[i].usernamePidSelected = _torqueControlLaw[i];
            tpids[i].enabled = true;
            tpids[i].kbemf = _kbemf[i];
            tpids[i].ktau = _ktau[i];
            tpids[i].filterType = _filterType[i];
        }
        else
        {
            tpids[i].enabled = false;
            tpids[i].usernamePidSelected = "none";
        }
    }

        //eomc_ctrl_out_type_n_a = 0,
        //eomc_ctrl_out_type_pwm = 1,
        //eomc_ctrl_out_type_vel = 2,
        //eomc_ctrl_out_type_cur = 3

    return true;


    //Here i would check that all joints have same type units in order to create torquehelper with correct factor.

    //get first joint with enabled torque
    int firstjoint = -1;
    for(int i=0; i<_njoints; i++)
    {
        if(tpids[i].enabled)
            firstjoint = i;
    }

    if(firstjoint==-1)
    {
        // no joint has torque enabed
        return true;
    }

    for(int i=firstjoint+1; i<_njoints; i++)
    {
        if(tpids[i].enabled)
        {
            if(tpids[firstjoint].fbk_PidUnits != tpids[i].fbk_PidUnits ||
               tpids[firstjoint].out_PidUnits != tpids[i].out_PidUnits)
            {
                yError() << "embObjMC BOARD " << _boardname << "all joints with torque enabled should have same controlunits type. Joint " << firstjoint << " differs from joint " << i;
                return false;
            }
        }
    }

    return true;
}
#endif

bool Parser::parsePidUnitsType(Bottle &bPid, yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits)
{

    Value &fbkControlUnits=bPid.find("fbkControlUnits");
    Value &outControlUnits = bPid.find("outputControlUnits");
    if(fbkControlUnits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " missing fbkControlUnits parameter";
        return false;
    }
    if(!fbkControlUnits.isString())
    {
        yError() << "embObjMC BOARD " << _boardname << " fbkControlUnits parameter is not a string";
        return false;
    }
    if (outControlUnits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " missing outputControlUnits parameter";
        return false;
    }
    if (!outControlUnits.isString())
    {
        yError() << "embObjMC BOARD " << _boardname << " outputControlUnits parameter is not a string";
        return false;
    }

    if(fbkControlUnits.toString()==string("metric_units"))
    {
        fbk_pidunits = yarp::dev::PidFeedbackUnitsEnum::METRIC;
    }
    else if(fbkControlUnits.toString()==string("machine_units"))
    {
        fbk_pidunits = yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
    }
    else
    {
        yError() << "embObjMC BOARD " << _boardname << "invalid fbkControlUnits value: " << fbkControlUnits.toString().c_str();
        return false;
    }

    if (outControlUnits.toString() == string("dutycycle_percent"))
    {
        out_pidunits = yarp::dev::PidOutputUnitsEnum::DUTYCYCLE_PWM_PERCENT;
    }
    else if (outControlUnits.toString() == string("machine_units"))
    {
        out_pidunits = yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS;
    }
    else
    {
        yError() << "embObjMC BOARD " << _boardname << "invalid outputControlUnits value: " << outControlUnits.toString().c_str();
        return false;
    }
    return true;
}


bool Parser::parse2FocGroup(yarp::os::Searchable &config, eomc::twofocSpecificInfo_t *twofocinfo)
{
     Bottle &focGroup=config.findGroup("2FOC");
     if (focGroup.isNull() )
     {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group 2FOC is not found in configuration file";
        return false;
     }

    Bottle xtmp;
    unsigned int i;

    if (!extractGroup(focGroup, xtmp, "HasHallSensor", "HasHallSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
           twofocinfo[i - 1].hasHallSensor = xtmp.get(i).asInt() != 0;
    }
    if (!extractGroup(focGroup, xtmp, "HasTempSensor", "HasTempSensor 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasTempSensor = xtmp.get(i).asInt() != 0;
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoder", "HasRotorEncoder 0/1 ", _njoints))
    {
        return false;
    }
    else
    {

        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasRotorEncoder = xtmp.get(i).asInt() != 0;
    }
    if (!extractGroup(focGroup, xtmp, "HasRotorEncoderIndex", "HasRotorEncoderIndex 0/1 ", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasRotorEncoderIndex = xtmp.get(i).asInt() != 0;
    }

    if (!extractGroup(focGroup, xtmp, "Verbose", "Verbose 0/1 ", _njoints, false))
    {
        //return false;
        yWarning() << "In " << _boardname << " there isn't 2FOC.Verbose filed. For default it is enabled" ;
        for (i = 0; i < (unsigned)_njoints; i++)
            twofocinfo[i].verbose = 1;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].verbose = xtmp.get(i).asInt() != 0;
    }

	std::vector<int> AutoCalibration (_njoints);
    if (!extractGroup(focGroup, xtmp, "AutoCalibration", "AutoCalibration 0/1 ", _njoints, false))
    {
        //return false;
        yWarning() << "In " << _boardname << " there isn't 2FOC.AutoCalibration filed. For default it is disabled" ;
        for (i = 0; i < (unsigned)_njoints; i++)
            AutoCalibration[i] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            AutoCalibration[i - 1] = xtmp.get(i).asInt();
    }


    if (!extractGroup(focGroup, xtmp, "RotorIndexOffset", "RotorIndexOffset", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
        {
            if(AutoCalibration[i-1] == 0)
            {
                twofocinfo[i - 1].rotorIndexOffset = xtmp.get(i).asInt();
                if (twofocinfo[i - 1].rotorIndexOffset <0 ||  twofocinfo[i - 1].rotorIndexOffset >359)
                {
                    yError() << "In " << _boardname << "joint " << i-1 << ": rotorIndexOffset should be in [0,359] range." ;
                    return false;
                }
            }
            else
            {
                yWarning() <<  "In " << _boardname << "joint " << i-1 << ": motor autocalibration is enabled!!! ATTENTION!!!" ;
                twofocinfo[i - 1].rotorIndexOffset = -1;
            }
        }
    }


    //Now I verify if rotor encoder hasn't index, then  rotor offset must be zero.
    for (i = 0; i < (unsigned)_njoints; i++)
    {
        if((0 == twofocinfo[i].hasRotorEncoderIndex) && (0 != twofocinfo[i].rotorIndexOffset))
        {
            yError() << "In " << _boardname << "joint " << i << ": inconsistent configuration: if rotor encoder hasn't index then its offset should be 0." ;
            return false;
        }
    }

    // Number of motor poles
    if (!extractGroup(focGroup, xtmp, "MotorPoles", "MotorPoles", _njoints))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].motorPoles = xtmp.get(i).asInt();
    }

    if (!extractGroup(focGroup, xtmp, "HasSpeedEncoder", "HasSpeedEncoder 0/1 ", _njoints))
    {
        yWarning () << "missing param HasSpeedEncoder";
        // optional by now
        //return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
            twofocinfo[i - 1].hasSpeedEncoder = xtmp.get(i).asInt() != 0;
    }

    return true;

}




bool Parser::parseJointsetCfgGroup(yarp::os::Searchable &config, std::vector<JointsSet> &jsets, std::vector<int> &joint2set)
{
    Bottle jointsetcfg = config.findGroup("JOINTSET_CFG");
    if (jointsetcfg.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing JOINTSET_CFG group";
        return false;
    }


    Bottle xtmp;
    int numofsets = 0;

    if(!extractGroup(jointsetcfg, xtmp, "numberofsets", "number of sets ", 1))
    {
        return  false;
    }

    numofsets = xtmp.get(1).asInt();

    if((0 == numofsets) || (numofsets > _njoints))
    {
        yError() << "embObjMC BOARD " << _boardname << "Number of jointsets is not correct. it should belong to (1, " << _njoints << ")";
        return false;
    }



    if(!checkAndSetVectorSize(jsets, numofsets, "parseJointsetCfgGroup"))
        return false;

    if(!checkAndSetVectorSize(joint2set, _njoints, "parseJointsetCfgGroup"))
        return false;

    for(unsigned int s=0;s<(unsigned)numofsets;s++)
    {
        char jointset_string[80];
        sprintf(jointset_string, "JOINTSET_%d", s);
        bool formaterror = false;


        Bottle &js_cfg = jointsetcfg.findGroup(jointset_string);
        if(js_cfg.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "cannot find " << jointset_string;
            return false;
        }

        //1) id of set
        jsets.at(s).id=s;


        //2) list of joints
        Bottle &b_listofjoints=js_cfg.findGroup("listofjoints", "list of joints");
        if (b_listofjoints.isNull())
        {
            yError() << "embObjMC BOARD " << _boardname << "listofjoints parameter not found";
            return false;
        }

        int numOfJointsInSet = b_listofjoints.size()-1;
        if((numOfJointsInSet < 1) || (numOfJointsInSet>_njoints))
        {
            yError() << "embObjMC BOARD " << _boardname << "numof joints of set " << s << " is not correct";
            return false;
        }


        for (int j = 0; j <numOfJointsInSet; j++)
        {
            int jointofthisset = b_listofjoints.get(j+1).asInt();

            if((jointofthisset< 0) || (jointofthisset>_njoints))
            {
                yError() << "embObjMC BOARD " << _boardname << "invalid joint number for set " << s;
                return false;
            }

            jsets.at(s).joints.push_back(jointofthisset);

            //2.1) fill map joint to set
            joint2set.at(jointofthisset) = s;
        }

        // 3) constraints
        if(!extractGroup(js_cfg, xtmp, "constraint", "type of jointset constraint ", 1))
        {
            return  false;
        }

        eOmc_jsetconstraint_t constraint;
        if(!convert(xtmp.get(1).asString(), constraint, formaterror))
        {
            return false;
        }
        jsets.at(s).cfg.constraints.type = constraint;

        //param1
        if(!extractGroup(js_cfg, xtmp, "param1", "param1 of jointset constraint ", 1))
        {
            return  false;
        }
        jsets.at(s).cfg.constraints.param1 = (float)xtmp.get(1).asDouble();

        //param2
        if(!extractGroup(js_cfg, xtmp, "param2", "param2 of jointset constraint ", 1))
        {
            return  false;
        }
        jsets.at(s).cfg.constraints.param2 = (float)xtmp.get(1).asDouble();


    }
    return true;
}

bool Parser::parseTimeoutsGroup(yarp::os::Searchable &config, std::vector<timeouts_t> &timeouts, int defaultVelocityTimeout)
{
    if(!checkAndSetVectorSize(timeouts, _njoints, "parseTimeoutsGroup"))
        return false;

    unsigned int i;

    Bottle timeoutsGroup =config.findGroup("TIMEOUTS");
    if(timeoutsGroup.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " no TIMEOUTS group found in config file.";
        return false;
    }

    Bottle xtmp;
    xtmp.clear();
    if (!extractGroup(timeoutsGroup, xtmp, "velocity", "a list of timeout to be used in the vmo control", _njoints))
    {
        yError() << "embObjMC BOARD " << _boardname << " no velocity parameter found in TIMEOUTS group in motion control config file.";
        return false;
    }
    else
    {
        for(i=1; i<xtmp.size(); i++)
            timeouts[i-1].velocity = xtmp.get(i).asInt();
    }


    return true;

}

bool Parser::parseCurrentLimits(yarp::os::Searchable &config, std::vector<motorCurrentLimits_t> &currLimits)
{
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group LIMITS is not found in configuration file";
        return false;
    }

    currLimits.resize(_njoints);
    unsigned int i;
    Bottle xtmp;

    // current limit
    if (!extractGroup(limits, xtmp, "motorOverloadCurrents","a list of current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) currLimits[i-1].overloadCurrent=xtmp.get(i).asDouble();

    // nominal current
    if (!extractGroup(limits, xtmp, "motorNominalCurrents","a list of nominal current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) currLimits[i-1].nominalCurrent =xtmp.get(i).asDouble();

    // peak current
    if (!extractGroup(limits, xtmp, "motorPeakCurrents","a list of peak current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) currLimits[i-1].peakCurrent=xtmp.get(i).asDouble();

    return true;

}

bool Parser::parseJointsLimits(yarp::os::Searchable &config, std::vector<jointLimits_t> &jointsLimits)
{
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group LIMITS is not found in configuration file";
        return false;
    }

    jointsLimits.resize(_njoints);
    unsigned int i;
    Bottle xtmp;

    // max limit
    if (!extractGroup(limits, xtmp, "jntPosMax","a list of user maximum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posMax = xtmp.get(i).asDouble();

    // min limit
    if (!extractGroup(limits, xtmp, "jntPosMin","a list of user minimum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posMin = xtmp.get(i).asDouble();

    // max hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMax","a list of hardware maximum angles (in degrees)", _njoints))
        return false;
    else
    {
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posHwMax = xtmp.get(i).asDouble();

        //check hardware limits are bigger then user limits
        for(i=0; i<(unsigned)_njoints; i++)
        {
            if(jointsLimits[i].posMax > jointsLimits[i].posHwMax)
            {
                yError() << "embObjMotionControl: user has set a limit  bigger then hardware limit!. Please check jntPosMax.";
                return false;
            }
        }
    }

    // min hardware limit
    if (!extractGroup(limits, xtmp, "hardwareJntPosMin","a list of hardware minimum angles (in degrees)", _njoints))
    {
        return false;
    }
    else
    {
        for(i=1; i<xtmp.size(); i++) jointsLimits[i-1].posHwMin = xtmp.get(i).asDouble();

        //check hardware limits are bigger then user limits
        for(i=0; i<(unsigned)_njoints; i++)
        {
            if(jointsLimits[i].posMin < jointsLimits[i].posHwMin)
            {
                yError() << "embObjMotionControl: user has set a limit  bigger then hardware limit!. Please check jntPosMin.";
                return false;
            }
        }

    }

    // joint Velocity command max limit
    if (!extractGroup(limits, xtmp, "jntVelMax", "a list of maximum velocities for the joints (in degrees/s)", _njoints))
        return false;
    else
        for (i = 1; i<xtmp.size(); i++)     jointsLimits[i - 1].velMax = xtmp.get(i).asDouble();

    return true;
}


bool Parser::parseRotorsLimits(yarp::os::Searchable &config, std::vector<rotorLimits_t> &rotorsLimits)
{
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << " detected that Group LIMITS is not found in configuration file";
        return false;
    }

    if(!checkAndSetVectorSize(rotorsLimits, _njoints, "parseRotorsLimits"))
        return false;

    Bottle xtmp;
    unsigned int i;

    // Rotor max limit
    if (!extractGroup(limits, xtmp, "rotorPosMax","a list of maximum rotor angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) rotorsLimits[i-1].posMax = xtmp.get(i).asDouble();



    // Rotor min limit
    if (!extractGroup(limits, xtmp, "rotorPosMin","a list of minimum roto angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) rotorsLimits[i-1].posMin = xtmp.get(i).asDouble();

    // Motor pwm limit
    if (!extractGroup(limits, xtmp, "motorPwmLimit","a list of motor PWM limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++)
        {
            rotorsLimits[i-1].pwmMax = xtmp.get(i).asDouble();
            if(rotorsLimits[i-1].pwmMax<0)
            {
                yError() << "motorPwmLimit should be a positive value";
                return false;
            }
        }

    return true;

}




bool Parser::parseCouplingInfo(yarp::os::Searchable &config, couplingInfo_t &couplingInfo)
{
    Bottle coupling_bottle = config.findGroup("COUPLINGS");
    if (coupling_bottle.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname <<  "Missing Coupling group";
        return false;
    }
    Bottle xtmp;
    int  fixedMatrix4X4Size = 16;
    int  fixedMatrix4X6Size = 24;
    bool formaterror =false;

    // matrix J2M
    if (!extractGroup(coupling_bottle, xtmp, "matrixJ2M", "matrixJ2M ", fixedMatrix4X4Size))
    {
        return false;
    }

    if(false == convert(xtmp, couplingInfo.matrixJ2M, formaterror, fixedMatrix4X4Size))
    {
       yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixJ2M";
       return false;
    }


    // matrix E2J
    if (!extractGroup(coupling_bottle, xtmp, "matrixE2J", "matrixE2J ", fixedMatrix4X6Size))
    {
        return false;
    }

    formaterror = false;
    if(false == convert(xtmp, couplingInfo.matrixE2J, formaterror, fixedMatrix4X6Size))
    {
        yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixE2J";
        return false;
    }


    // matrix M2J
    if (!extractGroup(coupling_bottle, xtmp, "matrixM2J", "matrixM2J ", fixedMatrix4X4Size))
    {
        return false;
    }

    formaterror = false;
    if( false == convert(xtmp, couplingInfo.matrixM2J, formaterror, fixedMatrix4X4Size))
    {
        yError() << "embObjMC BOARD " << _boardname << " has detected an illegal format for some of the values of CONTROLLER.matrixM2J";
        return false;
    }

    return true;
}


bool Parser::parseMotioncontrolVersion(yarp::os::Searchable &config, int &version)
{
    if (!config.findGroup("GENERAL").find("MotioncontrolVersion").isInt())
    {
        yError() << "Missing MotioncontrolVersion parameter. RobotInterface cannot start. Please contact icub-support@iit.it";
        return false;
    }

    version = config.findGroup("GENERAL").find("MotioncontrolVersion").asInt();
    return true;

}

bool Parser::isVerboseEnabled(yarp::os::Searchable &config)
{
    bool ret = false;
    if(!config.findGroup("GENERAL").find("verbose").isBool())
    {
        yError() << "embObjMotionControl::open() detects that general->verbose bool param is different from accepted values (true / false). Assuming false";
        ret = false;
    }
    else
    {
       ret = config.findGroup("GENERAL").find("verbose").asBool();
    }
    _verbosewhenok = ret;
    return ret;
}

bool Parser::parseBehaviourFalgs(yarp::os::Searchable &config, bool &useRawEncoderData, bool  &pwmIsLimited )
{

    // Check useRawEncoderData = do not use calibration data!
    Value use_raw = config.findGroup("GENERAL").find("useRawEncoderData");

    if(use_raw.isNull())
    {
        useRawEncoderData = false;
    }
    else
    {
        if(!use_raw.isBool())
        {
            yWarning() << "embObjMotionControl::open() detected that useRawEncoderData bool param is different from accepted values (true / false). Assuming false";
            useRawEncoderData = false;
        }
        else
        {
            useRawEncoderData = use_raw.asBool();
            if(useRawEncoderData)
            {
                yWarning() << "embObjMotionControl::open() detected that it is using raw data from encoders! Be careful  See 'useRawEncoderData' param in config file";
                yWarning() << "DO NOT USE OR CALIBRATE THE ROBOT IN THIS CONFIGURATION!";
                yWarning() << "CHECK IF THE FAULT BUTTON IS PRESSED and press ENTER to continue";
                getchar();
            }
        }
    }

    // Check useRawEncoderData = do not use calibration data!
    Value use_limitedPWM = config.findGroup("GENERAL").find("useLimitedPWM");
    if(use_limitedPWM.isNull())
    {
        pwmIsLimited = false;
    }
    else
    {
        if(!use_limitedPWM.isBool())
        {
            pwmIsLimited = false;
        }
        else
        {
            pwmIsLimited = use_limitedPWM.asBool();
        }
    }

    return true;
}



bool Parser::parseAxisInfo(yarp::os::Searchable &config, int axisMap[], std::vector<axisInfo_t> &axisInfo)
{

    Bottle xtmp;
    unsigned int i;
    axisInfo.resize(_njoints);

    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }


    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
    {
        int user_joint =  xtmp.get(i).asInt();
        if(user_joint>= _njoints)
        {
            yError() << "embObjMC BOARD " << _boardname << "In AxisMap param: joint " << i-1 << "has been mapped to not-existing joint ("<< user_joint <<"). Here there are only "<< _njoints <<"joints";
            return false;
        }
        axisMap[i-1] = user_joint;
    }
    

    if (!extractGroup(general, xtmp, "AxisName", "a list of strings representing the axes names", _njoints))
        return false;

    //beware: axis name has to be remapped here because they are not set using the toHw() helper function
    for (i = 1; i < xtmp.size(); i++)
    {
        int mappedto = axisInfo[i-1].mappedto;
        axisInfo[axisMap[i - 1]].name = xtmp.get(i).asString();
    }

    if (!extractGroup(general, xtmp, "AxisType", "a list of strings representing the axes type (revolute/prismatic)", _njoints))
        return false;

    //beware: axis type has to be remapped here because they are not set using the toHw() helper function
    for (i = 1; i < xtmp.size(); i++)
    {
        string s = xtmp.get(i).asString();
        int mappedto = axisInfo[i-1].mappedto;
        if (s == "revolute")  axisInfo[axisMap[i - 1]].type = VOCAB_JOINTTYPE_REVOLUTE;
        else if (s == "prismatic")  axisInfo[axisMap[i - 1]].type = VOCAB_JOINTTYPE_PRISMATIC;
        else
        {
            yError("Unknown AxisType value %s!", s.c_str());
            axisInfo[axisMap[i - 1]].type = VOCAB_JOINTTYPE_UNKNOWN;
            return false;
        }
    }

    return true;
}




bool Parser::parseEncoderFactor(yarp::os::Searchable &config, double encoderFactor[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }
    Bottle xtmp;
    unsigned int i;
    double tmp_A2E;

    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
    {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmp_A2E = xtmp.get(i).asDouble();
        if (tmp_A2E<0)
        {
            yWarning() << "embObjMC BOARD " << _boardname << "Encoder parameter should be positive!";
        }
        encoderFactor[i - 1] = tmp_A2E;
    }

    return true;
}

bool Parser::parsefullscalePWM(yarp::os::Searchable &config, double dutycycleToPWM[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing General group";
        return false;
    }
    Bottle xtmp;
    unsigned int i;
    double tmpval;

    // fullscalePWM
    if (!extractGroup(general, xtmp, "fullscalePWM", "a list of scales for the fullscalePWM conversion factor", _njoints))
    {
        yError("fullscalePWM param not found in config file. Please update robot configuration files or contact https://github.com/robotology/icub-support");
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmpval = xtmp.get(i).asDouble();
        if (tmpval<0)
        {
            yError() << "embObjMC BOARD " << _boardname << "fullscalePWM parameter should be positive!";
            return false;
        }
        dutycycleToPWM[i - 1] = tmpval / 100.0;
    }

    return true;
}


bool Parser::parseAmpsToSensor(yarp::os::Searchable &config, double ampsToSensor[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
        yError() << "embObjMC BOARD " << _boardname << "Missing General group";
        return false;
    }
    Bottle xtmp;
    unsigned int i;
    double tmpval;

    // ampsToSensor
    if (!extractGroup(general, xtmp, "ampsToSensor", "a list of scales for the ampsToSensor conversion factor", _njoints))
    {
        yError("ampsToSensor param not found in config file. Please update robot configuration files or contact https://github.com/robotology/icub-support");
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        tmpval = xtmp.get(i).asDouble();
        if (tmpval<0)
        {
            yError() << "embObjMC BOARD " << _boardname << "ampsToSensor parameter should be positive!";
            return false;
        }
        ampsToSensor[i - 1] = tmpval;
    }

    return true;
}

bool Parser::parseGearboxValues(yarp::os::Searchable &config, double gearbox_M2J[], double gearbox_E2J[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }

    Bottle xtmp;
    unsigned int i;

    // Gearbox_M2J
    if (!extractGroup(general, xtmp, "Gearbox_M2J", "The gearbox reduction ratio", _njoints))
    {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        gearbox_M2J[i-1] = xtmp.get(i).asDouble();
        if (gearbox_M2J[i-1]==0)
        {
            yError()  << "embObjMC BOARD " << _boardname << "Using a gearbox value = 0 may cause problems! Check your configuration files";
            return false;
        }
    }


    //Gearbox_E2J
    if (!extractGroup(general, xtmp, "Gearbox_E2J", "The gearbox reduction ratio between encoder and joint", _njoints))
    {
        return false;
    }

    int test = xtmp.size();
    for (i = 1; i < xtmp.size(); i++)
    {
        gearbox_E2J[i-1] = xtmp.get(i).asDouble();
        if (gearbox_E2J[i-1]==0)
        {
            yError()  << "embObjMC BOARD " << _boardname << "Using a gearbox value = 0 may cause problems! Check your configuration files";
            return false;
        }
    }


    return true;
}

bool Parser::parseDeadzoneValue(yarp::os::Searchable &config, double deadzone[], bool *found)
{
//     Bottle general = config.findGroup("GENERAL");
//     if (general.isNull())
//     {
//         yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
//         return false;
//     }

    Bottle general = config.findGroup("OTHER_CONTROL_PARAMETERS");
    if (general.isNull())
    {
        yWarning() << "embObjMC BOARD " << _boardname << "Missing OTHER_CONTROL_PARAMETERS.DeadZone parameter. I'll use default value. (see documentation for more datails)";
        *found = false;
        return true;
    }    
    Bottle xtmp;
    unsigned int i;
    
    // DeadZone
    if (!extractGroup(general, xtmp, "deadZone", "The deadzone of joint", _njoints, false))
    {
        yWarning() << "embObjMC BOARD " << _boardname << "Missing OTHER_CONTROL_PARAMETERS group.DeadZone parameter. I'll use default value. (see documentation for more datails)";
        *found = false;
        return true;
    }
 
    *found = true;
    for (i = 1; i < xtmp.size(); i++)
    {
        deadzone[i-1] = xtmp.get(i).asDouble();
    }
    
    return true;
}


bool Parser::parseMechanicalsFlags(yarp::os::Searchable &config, int useMotorSpeedFbk[])
{
    Bottle general = config.findGroup("GENERAL");
    if (general.isNull())
    {
       yError() << "embObjMC BOARD " << _boardname << "Missing General group" ;
       return false;
    }
    Bottle xtmp;
    unsigned int i;

    if(!extractGroup(general, xtmp, "useMotorSpeedFbk", "Use motor speed feedback", _njoints))
    {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++)
    {
        useMotorSpeedFbk[i-1] = xtmp.get(i).asInt();
    }

    return true;

}






bool Parser::parseImpedanceGroup(yarp::os::Searchable &config,std::vector<impedanceParameters_t> &impedance)
{
    Bottle impedanceGroup;
    impedanceGroup=config.findGroup("IMPEDANCE","IMPEDANCE parameters");

    if(impedanceGroup.isNull())
    {
        yError() <<"embObjMC BOARD " << _boardname << "fromConfig(): Error: no IMPEDANCE group found in config file, returning";
        return false;
    }



    if(_verbosewhenok)
    {
        yDebug()  << "embObjMC BOARD " << _boardname << ":fromConfig() detected that IMPEDANCE parameters section is found";
    }

    if(!checkAndSetVectorSize(impedance, _njoints, "parseImpedanceGroup"))
        return false;


    int j=0;
    Bottle xtmp;
    if (!extractGroup(impedanceGroup, xtmp, "stiffness", "stiffness parameter", _njoints))
        return false;

    for (j=0; j<_njoints; j++)
        impedance[j].stiffness = xtmp.get(j+1).asDouble();

    if (!extractGroup(impedanceGroup, xtmp, "damping", "damping parameter", _njoints))
        return false;

    for (j=0; j<_njoints; j++)
        impedance[j].damping = xtmp.get(j+1).asDouble();

    if(_verbosewhenok)
    {
        yInfo() << "embObjMC BOARD " << _boardname << "IMPEDANCE section: parameters successfully loaded";
    }
    return true;

}

bool Parser::convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    jsetconstraint = eomc_string2jsetconstraint(t, usecompactstring);

    if(eomc_jsetconstraint_unknown == jsetconstraint)
    {
        usecompactstring = eobool_true;
        jsetconstraint = eomc_string2jsetconstraint(t, usecompactstring);
    }

    if(eomc_jsetconstraint_unknown == jsetconstraint)
    {
        yError() << "embObjMC BOARD " << _boardname << "String" << t << "cannot be converted into a proper eOmc_jsetconstraint_t";
        formaterror = true;
        return false;
    }

    return true;
}



bool Parser::convert(Bottle &bottle, vector<double> &matrix, bool &formaterror, int targetsize)
{
    matrix.resize(0);

    int tmp = bottle.size();
    int sizeofmatrix = tmp - 1;    // first position of bottle contains the tag "matrix"

    // check if there are really the target number of elements in matrix.
    if(targetsize != sizeofmatrix)
    {
        yError() << "embObjMC BOARD " << _boardname << " in converting string do matrix.In the matrix there are not" << targetsize << "elements";
        return false;
    }

    formaterror = false;
    for(int i=0; i<sizeofmatrix; i++)
    {
        double item = 0;

        // ok, i use the standard converter ... but what if it is not a double format? so far we dont check.
        item = bottle.get(i+1).asDouble();
        matrix.push_back(item);
    }

    // in here we could decide to return false if any previous conversion function has returned error

    return true;
}

//////////////////////////////////////////////////////////////////////////////
/////////////////// DEBUG FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void Parser::debugUtil_printControlLaws(void)
{
    /*
    //////// debug prints
    yError() << "position control law: ";
    for(int x=0; x<_njoints; x++)
    {
        yError() << " - j " << x << _positionControlLaw[x].c_str();
    }

    yError() << "velocity control law: ";
    for(int x=0; x<_njoints; x++)
    {
        yError() << "- j " << x << _velocityControlLaw[x].c_str();
    }


    yError() << "torque control law: ";
    for(int x=0; x<_njoints; x++)
    {
        yError() << " - j " << x << _torqueControlLaw[x].c_str();
    }
    //////end
    */
}

/*
void PidInfo::dumpdata(void)
{

    cout <<  "Is enabled " << enabled;
    cout <<  ". Username pid selected is " << usernamePidSelected;
    switch(controlLaw)
    {
        case PidAlgo_simple:
            cout <<  ". Control law is " << "PidAlgo_simple";
            break;

        case PIdAlgo_velocityInnerLoop:
            cout <<  ". Control law is " << "PIdAlgo_velocityInnerLoop";
            break;

        case PidAlgo_currentInnerLoop:
            cout <<  ". Control law is " << "PidAlgo_currentInnerLoop";
            break;
        default :
            cout <<  ". Control law is " << "unknown";
    }

    cout << ". PID fbk Unit type is " << (int)fbk_PidUnits;
    cout << ". PID out Unit type is " << (int)out_PidUnits;

    cout << " kp is " << pid.kp;
    cout << endl;

}
*/
void JointsSet::dumpdata(void)
{
    switch(cfg.constraints.type)
    {
        case eomc_jsetconstraint_none:
            cout <<  "constraint is " << "eomc_jsetconstraint_none";
            break;
        case eomc_jsetconstraint_cerhand:
            cout <<  "constraint is " << "eomc_jsetconstraint_cerhand";
             break;
        case eomc_jsetconstraint_trifid:
            cout <<  "constraint is " << "eomc_jsetconstraint_trifid";
             break;
      default :
            cout <<  ". constraint is " << "unknown";
    }

    cout << " param1="<< cfg.constraints.param1 << " param2=" << cfg.constraints.param2 << endl;

}

