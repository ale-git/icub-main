// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_H__
#define __GTKMM_ICUB_BOARD_H__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>

////////////////////////////////////

#include "iCubBoardChannel.h"

////////////////////////////////////

class iCubBoard
{
public:
    iCubBoard()
    {
    }

    virtual ~iCubBoard()
    {
    }

    virtual yarp::dev::LoggerDataRef* getDataReference(std::string addr)=0;
    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data)=0;
    virtual bool findAndRead(std::string addr,yarp::os::Value& data)=0;
    
    virtual yarp::os::Bottle toBottle(bool bConfig=false)=0;
    virtual void fromBottle(yarp::os::Bottle& bot)=0;

    virtual bool hasAlarm(){ return false; }
};

class iCubBLLBoard : public iCubBoard
{
public:
    iCubBLLBoard() : iCubBoard()
    {
    }

    iCubBLLBoard(int ID,int j0,int j1) : iCubBoard(),mID(ID)
    {
        mChannel[0]=new iCubBLLChannel(0,j0);
        mChannel[1]=new iCubBLLChannel(1,j1);

        mData.write(STRING_Board_Type,yarp::os::Value("BLL"));
        mData.write(INT_Board_ID,yarp::os::Value(ID));
    }

    virtual ~iCubBLLBoard()
    {
        if (mChannel[0]!=NULL) delete mChannel[0];
        if (mChannel[1]!=NULL) delete mChannel[1];
    }

    enum Index
    {
        STRING_Board_Type,  // ="BLL"
        INT_Board_ID        //The id with which the board is identified on the canbus
    };

    virtual yarp::os::Bottle toBottle(bool bConfig)
    {
        yarp::os::Bottle bot;

        yarp::os::Bottle data=mData.toBottle(bConfig);
        if (data.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(-1);
            addList.append(data);
        }

        for (int c=0; c<2; ++c)
        {
            yarp::os::Bottle chan=mChannel[c]->toBottle(bConfig);
            if (chan.size())
            {
                yarp::os::Bottle &addList=bot.addList();
                addList.addInt(c);
                addList.append(chan);
            }
        }

        return bot;
    }

    virtual void fromBottle(yarp::os::Bottle& bot)
    {
        for (int i=1; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *list=bot.get(i).asList();

            if (list->get(0).asInt()==-1)
            {
                mData.fromBottle(*list);
            }
            else
            {
                mChannel[list->get(0).asInt()]->fromBottle(*list);
            }
        }
    }

    virtual yarp::dev::LoggerDataRef* getDataReference(std::string addr);
    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data);
    virtual bool findAndRead(std::string addr,yarp::os::Value& data);

protected:
    iCubBLLChannel *mChannel[2];
     
    int mID;
    RawData mData;
    static const char *mRowNames[];
};

class iCubAnalogBoard : public iCubBoard
{
public:
    iCubAnalogBoard() : iCubBoard()
    {
    }

    iCubAnalogBoard(int ID,int channels) : iCubBoard(),mID(ID)
    {
        printf("########## iCubAnalogBoard(%d,%d) ##############\n",ID,channels); 
        nChannels=channels;

        mChannel=new iCubAnalogChannel*[nChannels];

        for (int c=0; c<nChannels; ++c)
        {
            mChannel[c]=new iCubAnalogChannel(c);
        }

        mData.write(STRING_Board_Type,yarp::os::Value("analog"));
        mData.write(INT_Board_ID,yarp::os::Value(ID));
        mData.write(INT_Num_Channels,yarp::os::Value(nChannels));
    }

    virtual ~iCubAnalogBoard()
    {
        if (mChannel)
        {
            for (int c=0; c<nChannels; ++c)
            {
                if (mChannel[c]!=NULL)
                {
                    delete mChannel[c];
                    mChannel[c]=NULL;
                }
            }

            delete [] mChannel;
            mChannel=NULL;
        }
    }

    enum Index
    {
        STRING_Board_Type,  // ="BLL"
        INT_Board_ID,       //The id with which the board is identified on the canbus
        INT_Num_Channels
    };

    virtual yarp::os::Bottle toBottle(bool bConfig)
    {
        yarp::os::Bottle bot;

        yarp::os::Bottle data=mData.toBottle(bConfig);
        if (data.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(-1);
            addList.append(data);
        }

        for (int c=0; c<nChannels; ++c)
        {
            yarp::os::Bottle chan=mChannel[c]->toBottle(bConfig);
            if (chan.size())
            {
                yarp::os::Bottle &addList=bot.addList();
                addList.addInt(c);
                addList.append(chan);
            }
        }

        return bot;
    }

    virtual void fromBottle(yarp::os::Bottle& bot)
    {
        for (int i=1; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *list=bot.get(i).asList();

            if (list->get(0).asInt()==-1)
            {
                mData.fromBottle(*list);
            }
            else
            {
                mChannel[list->get(0).asInt()]->fromBottle(*list);
            }
        }
    }

    virtual yarp::dev::LoggerDataRef* getDataReference(std::string addr);
    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data);
    virtual bool findAndRead(std::string addr,yarp::os::Value& data);

protected:
    iCubAnalogChannel **mChannel;
    
    int mID;
    int nChannels;

    RawData mData;
    static const char *mRowNames[];
};

#endif
