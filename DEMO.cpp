#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include <stdlib.h>  
#include <windows.h>

/*
*This Controller Code Simualtes the first scan of our robot as well as 
*its Power off button an Emergency STOP button funcionalities
*/


#define TIME_STEP 64

#define MAX_SPEED 7.5
//Rad/s

using namespace webots;

int main(int argc, char **argv) {
  //Set up Devices Instances
  Robot *robot = new Robot();
  
  Keyboard kb;
  kb.enable(TIME_STEP);

  Motor *arm;
  arm=robot->getMotor("linear");
  
  Motor *exts;
  exts = robot->getMotor("extension");
  
  Motor *pan_m;
  pan_m=robot->getMotor("RM");
  
  Motor *tilt_m;
  tilt_m=robot->getMotor("QM");
  
  Camera *cm;
  cm=robot->getCamera("CAM");
  cm->enable(TIME_STEP);
  
  GPS *gps_b;//BODY GPS
  gps_b = robot->getGPS("bot_center");
  gps_b->enable(TIME_STEP);
  
  GPS *gps_c;//CAMERA GPS
  gps_c = robot->getGPS("camera_gps_1");
  gps_c->enable(TIME_STEP);
  
  
  Motor *wheels[2];
  char wheels_names[2][8] = {"wheel1", "wheel2"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  //Initiate Joint Values
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  double linear=0.0;
  double extension=0.0;
  double pan = 0.55;
  double tilt=0.0;
  
  //Initiate Process Flags
  int pan_flag= 0; 
  int tilt_flag= 0;
  int extension_flag=0;
  int scan_flag = 1;
  int turn_off = 0;
  
  //Specify Object Dimension 
  //In Reality this would be an user Input
  double bot_width = 0.3;//m  
  double radius = 0.75;//m
  double height = 0.8;//m
  
  double cm_initial_height; //initial arm height
  cm_initial_height = height/2 - 0.15;  
  
  //Calcualte speeds for a counterclockwise concentric route
  double max_left_speed;
  
  double max_right_speed;
  
  double drive_ratio;
  
  max_right_speed = MAX_SPEED; 
 
  drive_ratio = ((1+(bot_width/(2*radius)))/(1-(bot_width/(2*radius))));
  
  max_left_speed = max_right_speed/drive_ratio; 
  
  //Iniatiate Robot Position to imitate VR Tracking Sensors
  double robot_x = 0;
  double robot_y = 0;
  double robot_z = 0;
  
  double scale = 0.01;
  
    
  /////////////////////////////////
  //Controller Infinite Loop Starts
  ///////////////////////////////
  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    

    if(robot_x == 0.0 && robot_y!=0.0){
      scan_flag = 1;
    }
    
    if(robot_x != 0.0 && robot_y== 0.0){
      scan_flag = 1;
    }
    
    if (key==32){//space -> scan
    scan_flag =1;
    turn_off = 0;
    }
    
    if (key==71){//g -> GO
    scan_flag =0;
    turn_off = 0;
    }
    
    
    if (scan_flag == 1){
        if(leftSpeed > 0.0 && rightSpeed > 0.0){
            
           leftSpeed += -max_left_speed/20;
           rightSpeed += -max_right_speed/20;
        
        } else if(linear < cm_initial_height ){
              linear += 0.01;
              
             } else if(pan < 1 && pan_flag == 0){
                  
                      pan += 0.025; 
                      if(pan >= 1){
                          pan_flag = 1;
                          }
                          
                  }else if(pan> 0.1 && pan_flag == 1){
                  
                      pan += -0.025; 
                      if(pan <= 0.1 ){
                          pan =0.55;
                          pan_flag = 2;
                          }
                          
                  }else if(extension > -0.1 && extension_flag == 0){
                  
                              extension += -0.005; 
                              if(extension <= -0.1){
                                extension_flag = 1;
                              }
                          
                     }else if(extension < 0 && extension_flag == 1){
                                          
                              extension += 0.005;      
                                           
                      }else if(tilt > -0.5 && tilt_flag == 0){
                      
                              tilt += -0.025; 
                              if(tilt <= -0.5){
                                tilt_flag = 1;
                              }
                              
                          }else if(tilt < 0.2 && tilt_flag == 1){
                      
                              tilt += 0.025; 
                              if(tilt >= 0.2 ){
                                tilt = 0;
                                tilt_flag = 2;
                                scan_flag = 0; 
                              } 
    
                          }   
      
      
        } else if (scan_flag == 0){
    
            if(leftSpeed < max_left_speed && rightSpeed < max_right_speed){
            
               leftSpeed += max_left_speed/20;
               rightSpeed += max_right_speed/20;
        
             }
        
             pan_flag = 0;
             tilt_flag = 0;
             extension_flag = 0;
                 
             pan = 0.55;
         
             tilt = 0.0;
         
             extension = 0.0;
         
          }
    
    wheels[0]->setVelocity(leftSpeed); //left_Wheel
    wheels[1]->setVelocity(rightSpeed);//Right_Wheel
    
    //Turn_off Button
    if (key==84 || turn_off==1){/// T_KEY
    scan_flag = 2;  
    turn_off = 1;
    pan = 0.55;     
    tilt = 0.0;
    extension = 0.0;
                 
        if(leftSpeed > 0.0 && rightSpeed > 0.0){           
           leftSpeed += -max_left_speed/20;
           rightSpeed += -max_right_speed/20;  
        } else if(linear > 0.01 ){
              linear += -0.01;   
          }
        
    }
   
    
    //Emergency Stop BUTTON
    if (key== 90 ){///Z_KEY
    scan_flag = 2;        
    leftSpeed = 0;
    rightSpeed = 0;
    }
    
    //Keyboard Control
    if (key==87 && linear<2){///W
    linear += 0.01;
    } else if (key==83 && linear>0){///S
    linear += -0.01;
    }else {
    linear+=0;
    }
    arm->setPosition(linear);
    
    //axis is inverted for this joint
    if (key==74 && extension<0){///J
    extension += 0.01;
    } else if (key==75 && extension>-0.1){///K
    extension += -0.01;
    }else {
    extension+=0;
    }
    exts->setPosition(extension); 
   
    if (key==65){////A
    pan += 0.05;
    } else if (key==68){///D
    pan += -0.05;
    }else {
    pan += 0;
    }
    pan_m->setPosition(pan);
    
    if (key==79){/////O
    tilt += 0.05;
    } else if (key==76){/////L
    tilt += -0.05;
    }else {
    tilt += 0;
    }
    tilt_m->setPosition(tilt);
    
    //Read and Scale Robot Coordinates
    robot_x = (int)(gps_b->getValues()[0] / scale) * scale;
    
    robot_y = (int)(gps_b->getValues()[1] / scale) * scale;
    
    robot_z = (int)(gps_b->getValues()[2] / scale) * scale;
    
    
    std::cout << "X :" <<robot_x<< std::endl;
    std::cout << "Y :" <<robot_y<< std::endl;
    std::cout << "Z :" <<robot_z<< std::endl; 
    
    std::cout << "X_C :" <<gps_c->getValues()[0]<< std::endl;
    std::cout << "Y_C :" <<gps_c->getValues()[1]<< std::endl;
    std::cout << "Z_C :" <<gps_c->getValues()[2]<< std::endl;
    
    std::cout << "Press T to Turn OFF" << std::endl;  
    std::cout << "Press Z for the Emergency Stop" << std::endl;
    std::cout << "Press SPACE to perform an scan at anytime" << std::endl;        
    
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}
