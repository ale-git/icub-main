/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_calmot CalMot
 *
 *This endclient module CalMot provides a quick method to calibrate the pix2deg variables in the RecServer. It simply places (repeatedly) a cross over the point where it intents to saccade to, then initiates the saccade.  The pix2deg parameters in the rec.cfg config file can be varied to ensure the saccade is accurate.
 *
 * This module should be called in the following way:\n \b calmot l \b\n
 *\n
 * where 'l' states that the left camera/axis is being tested. 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/left_ye
 * <li> /recserver/output/right_ye
 * <li> /recserver/output/rec_params
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /calmot/input/left_ye
 * <li> /calmot/input/right_ye
 * <li> /calmot/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /calmot/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/calmot/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */ 


#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include <display.h>
#include <mydraw.h>
#include <recio.h>
 
#define BLACK 0.01
#define GREY  0.3
#define WHITE 0.99

#define PIX 30.0
#define SAC_GAIN_X 1.0
#define SAC_GAIN_Y 1.0


using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  if (argc==1){
    printf("usage: %s [l,r] [sim]\n\n",argv[0]);
    exit(0);
  }

  QString arg1 = argv[1];
  QString arg2 = argv[2];

  QString cam;
  if (arg1=="l" || arg2=="l"){
    cam = "l";
  }
  if (arg1=="r" || arg2=="r"){
    cam = "r";
  }

  QString sim;
  if (arg1=="sim" || arg2=="sim"){
    sim = "Sim";
  }



  //probe recserver:
  Port inPort_s;
  inPort_s.open("/calmot"+sim+"/input/serv_params");
  Network::connect("/calmot"+sim+"/input/serv_params","/recserver"+sim+"/output/serv_params");
  Network::connect("/recserver"+sim+"/output/serv_params","/calmot"+sim+"/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;

  //for image input:
  BufferedPort<Bottle> inPort_y;
  Bottle *inBot_y;
  if (cam == "l"){
    inPort_y.open("/calmot"+sim+"/input/rec_y");
    Network::connect("/recserver"+sim+"/output/left_ye" , "/calmot"+sim+"/input/rec_y");
  }
  else {
    inPort_y.open("/calmot"+sim+"/input/rec_y");
    Network::connect("/recserver"+sim+"/output/right_ye" , "/calmot"+sim+"/input/rec_y");
  }
  Ipp8u* rec_im_y;
  
  

  //REQUEST MOTION LIKE THIS!
  Port outPort_mot;
  outPort_mot.open("/calmot"+sim+"/output/mot");
  Network::connect("/calmot"+sim+"/output/mot", "/recserver"+sim+"/input/motion");
  Network::connect("/recserver"+sim+"/input/motion", "/calmot"+sim+"/output/mot");
  BinPortable<RecMotionRequest> motion_request;
  sleep(1);

  //initalise:
  motion_request.content().pix_y    = 0.0;
  motion_request.content().pix_xl   = 0.0;
  motion_request.content().pix_xr   = 0.0;
  motion_request.content().deg_r    = 0.0;
  motion_request.content().relative = false; //absolute just to init.
  motion_request.content().suspend = 0;
  motion_request.content().lockto = NO_LOCK;
  motion_request.content().unlock = true;
  outPort_mot.write(motion_request);



  int width = rsp.width;
  int height = rsp.height;
  int mos_width = rsp.mos_width;
  int mos_height = rsp.mos_height;
  int psb = rsp.psb;
  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;
  double des_x=0.0,des_y=0.0,des_xl,des_xr;


  iCub::contrib::primateVision::Display *d_y = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"Y_REC");



  des_y  = PIX;
  des_x  = PIX;
  if (cam == "l"){
    des_xl = des_x;
    des_xr = 0;
  }
  else{
    des_xr = des_x;
    des_xl = 0;
  }



  motion_request.content().relative = true; //relative pixel moves.



  int frames = 0;

  while (1){

    inBot_y = inPort_y.read();   //blocking
    rec_im_y = (Ipp8u*) inBot_y->get(0).asBlob();


    MyDrawLine(rec_im_y,psb,srcsize,width/2+des_x,height/2+des_y,WHITE);
    MyDrawLine(rec_im_y,psb,srcsize,width/2,height/2,BLACK);
    d_y->display(rec_im_y);
    
    if (frames%50==0){

	  // first move this direction
      motion_request.content().pix_xl = des_xl*SAC_GAIN_X;
      motion_request.content().pix_xr = des_xr*SAC_GAIN_X;
      motion_request.content().pix_y  = des_y*SAC_GAIN_Y;
      outPort_mot.write(motion_request);

      //if endpoint is far from mosaic centre,
      //need to amplify motion.
      //conversely, if endpoint is in centre, amplification reduced. 

#if 1
     for (int i=0;i<=50;i++){
       inBot_y = inPort_y.read();   //blocking
       rec_im_y = (Ipp8u*) inBot_y->get(0).asBlob();
        MyDrawLine(rec_im_y,psb,srcsize,width/2+des_x,height/2+des_y,WHITE);
        MyDrawLine(rec_im_y,psb,srcsize,width/2,height/2,BLACK);
      	d_y->display(rec_im_y); 
        frames++;
        if (frames%50==0){

		    //second move this direction
            motion_request.content().pix_xl = des_xl*SAC_GAIN_X;
         	motion_request.content().pix_xr = des_xr*SAC_GAIN_X;
      		motion_request.content().pix_y  = des_y*SAC_GAIN_Y;
      		outPort_mot.write(motion_request);

        }
      }
#endif

	  //swap direction for next time!:
      des_x  = -des_x;
      des_xl = -des_xl;
      des_xr = -des_xr;
      des_y  = -des_y;
      

    }

    frames++;

  }


  //never here!
  
}
