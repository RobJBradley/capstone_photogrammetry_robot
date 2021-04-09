#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include <stdlib.h>  
#include <windows.h>


#define TIME_STEP 64

#define MAX_SPEED 7.5
//Rad/s

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;

  Motor *lr;
  lr=robot->getMotor("linear");
  
  Motor *exts;
  exts = robot->getMotor("extension");
  
  Motor *rm;
  rm=robot->getMotor("RM");
  
  Motor *qm;
  qm=robot->getMotor("QM");
  
  Camera *cm;
  cm=robot->getCamera("CAM");
  cm->enable(TIME_STEP);
  
  GPS *gps_b;
  gps_b = robot->getGPS("bot_center");
  gps_b->enable(TIME_STEP);
  
  GPS *gps_c;
  gps_c = robot->getGPS("camera_gps_1");
  gps_c->enable(TIME_STEP);
  
  
  Motor *wheels[2];
  char wheels_names[2][8] = {"wheel1", "wheel2"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  kb.enable(TIME_STEP);///
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  double linear=0.0;
  double extension=0.0;
  double rotate = 0.55;
  double rotateq=0.0;
  
  
  int rotate_flag= 0;
  
  int tilt_flag= 0;
  
  int extension_flag=0;
  
  int scan_flag = 1;
  
  int turn_off = 0;
  
  double bot_width = 0.3;//
    
  double radius = 0.75;//m
  
  //Calcualte speeds for a counterclockwise concentric route
  
  double max_left_speed;
  
  double max_right_speed;
  
  double drive_ratio;
  
  max_right_speed = MAX_SPEED; 
 
  drive_ratio = ((1+(bot_width/(2*radius)))/(1-(bot_width/(2*radius)))); //Robot's route radius is doble the raidius of the object to scan 
  
  max_left_speed = max_right_speed/drive_ratio; 
  
  ///WEBOTS LOOP STARTS
  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    
    
    if (key==32){//space -> scan
    scan_flag =1;
    turn_off = 0;
    }
    
    if (key==71){//g -> GO
    scan_flag =0;
    turn_off = 0;
    }
    
    ////SCAN PROCESS
    if (scan_flag == 1){
        if(leftSpeed > 0.0 && rightSpeed > 0.0){
           /// Wheels speed is decreased gradually so as to avoid a sudden stop
           leftSpeed += -max_left_speed/20;
           rightSpeed += -max_right_speed/20;
        
        } else{
           
             if(linear < 0.25 ){
              linear += 0.01;
              
             } else {
              
                  if(rotate < 1 && rotate_flag == 0){
                  
                      rotate += 0.025; 
                      if(rotate >= 1){
                          rotate_flag = 1;
                          }
                          
                  }else if(rotate> 0.1 && rotate_flag == 1){
                  
                      rotate += -0.025; 
                      if(rotate <= 0.1 ){
                          rotate =0.55;
                          rotate_flag = 2;
                          }
                          
                  }else {
              
                      if(extension > -0.1 && extension_flag == 0){
                  
                              extension += -0.005; 
                              if(extension <= -0.1){
                                extension_flag = 1;
                              }
                          
                     }else if(extension < 0 && extension_flag == 1){
                                          
                              extension += 0.005;      
                                           
                      }else {
                      
                          if(rotateq > -0.5 && tilt_flag == 0){
                      
                              rotateq += -0.025; 
                              if(rotateq <= -0.5){
                                tilt_flag = 1;
                              }
                              
                          }else if(rotateq < 0.2 && tilt_flag == 1){
                      
                              rotateq += 0.025; 
                              if(rotateq >= 0.2 ){
                                rotateq = 0;
                                tilt_flag = 2;
                                scan_flag = 0; 
                              } 
    
                          }   
                      }     
                   }         
               }
           }
             
    } else if (scan_flag == 0){
    
         if(leftSpeed < max_left_speed && rightSpeed < max_right_speed){
            
           leftSpeed += max_left_speed/20;
           rightSpeed += max_right_speed/20;
        
        }
        
         rotate_flag = 0;
         tilt_flag = 0;
         extension_flag = 0;
                 
         rotate = 0.55;
         
         rotateq = 0.0;
         
         extension = 0.0;
         
         if(linear >= 0.3 ){
              linear = 0.3;
              
         }
        
    }
    
    wheels[0]->setVelocity(leftSpeed); //left
    wheels[1]->setVelocity(rightSpeed);//right
    
    //Turn_off Button
    if (key==84 || turn_off==1){///t
    stop_flag = 2; 
    turn_off = 1;
    rotate = 0.55;     
    rotateq = 0.0;
    extension = 0.0;              
      if(leftSpeed > 0.0 && rightSpeed > 0.0){           
           leftSpeed += -max_left_speed/20;
           rightSpeed += -max_right_speed/20;  
      } else if(linear > 0.01 ){
              linear += -0.01;   
        }
    }
   
    
    //Emergency Stop BUTTON
    if (key== 90 ){///Z
    stop_flag = 2;        
    leftSpeed = 0;
    rightSpeed = 0;
    }
    
    //Keyboard Control
    if (key==87 && linear<2){///W
    linear += 0.01;//0.005
    } else if (key==83 && linear>0){///S
    linear += -0.01;
    }else {
    linear+=0;
    }
    lr->setPosition(linear);
    
    //axis is inverted for this joint
    if (key==74 && extension<0){///J
    extension += 0.01;//0.005
    } else if (key==75 && extension>-0.1){///K
    extension += -0.01;
    }else {
    extension+=0;
    }
    exts->setPosition(extension); 
   
    if (key==65){////A
    rotate += 0.05;
    } else if (key==68){///D
    rotate += -0.05;
    }else {
    rotate += 0;
    }
    rm->setPosition(rotate);
    
    if (key==79){/////O
    rotateq += 0.05;
    } else if (key==76){/////L
    rotateq += -0.05;
    }else {
    rotateq += 0;
    }
    qm->setPosition(rotateq); 
    
    std::cout << "X :" <<gps_b->getValues()[0]<< std::endl;
    std::cout << "Y :" <<gps_b->getValues()[1]<< std::endl;
    std::cout << "Z :" <<gps_b->getValues()[2]<< std::endl; 
    
    std::cout << "X_C :" <<gps_c->getValues()[0]<< std::endl;
    std::cout << "Y_C :" <<gps_c->getValues()[1]<< std::endl;
    std::cout << "Z_C :" <<gps_c->getValues()[2]<< std::endl;
    
    std::cout << "left_speed :" <<max_left_speed<< std::endl;      
    
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}
