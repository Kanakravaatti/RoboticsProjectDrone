/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */
#define DELAY_TIME 20.0

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/lidar.h>


#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  
   WbDeviceTag RM = wb_robot_get_device("RM");
  WbDeviceTag RM1 = wb_robot_get_device("RM1");
  WbDeviceTag RM2 = wb_robot_get_device("RM2");
  WbDeviceTag RM3 = wb_robot_get_device("RM3");
  
 
  
  wb_motor_set_position(RM, INFINITY);
  wb_motor_set_velocity(RM, 0.0);
  wb_motor_set_position(RM1, INFINITY);
  wb_motor_set_velocity(RM1, 0.0);
  wb_motor_set_position(RM2, INFINITY);
  wb_motor_set_velocity(RM2, 0.0);
  
  
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag brake = wb_robot_get_device("brake");
  
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps(1)");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gps2 = wb_robot_get_device("gps(2)");
  wb_gps_enable(gps2, timestep);
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  
  WbDeviceTag lidar2 = wb_robot_get_device("lidar(1)");
  wb_lidar_enable(lidar, timestep);
  wb_lidar_enable(lidar2, timestep);
 
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  
  
  
   WbDeviceTag distance_sensor = wb_robot_get_device("distance");

  double speed= -1.0;
  double leftright = 0.0;
    
    
  
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Display the welcome message.
  printf("Start the drone...\n");

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Display manual control message.
  printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");
  
  
  
  // Constants, empirically found.
  const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
  const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;           // P constant of the roll PID.
  const double k_pitch_p = 30.0;          // P constant of the pitch PID.

  // Variables.
 double target_altitude = 1.0;
 double target_velocity = 0.1;
 double target_position = -0.03;
  bool home = false;

 bool start_handled = false;
    bool detected = false;
    const double k_descend_altitude = 0.6;
  

     double start_time = wb_robot_get_time();
     
     
      bool search = false; 
  bool in_destination = false;
  int n = 0;
  double search1[26][2];
  //luodaan kartta etsimistä varten.
  void searchh(){
    double o = 6.5;
    double s = 5.0;
    int size = sizeof(search1)/sizeof(search1[0]);
    for ( int i=0; i < size; ++i){
    
      search1[i][1] = s;
      search1[i][0] = o;
      printf("etsi %f\n", search1[i][1]);
      printf("etsi %f\n", search1[i][0]);
      s = s * -1;
      o = o - 0.5;
    };
    n = 25; 
  }
  searchh();
  double layer_1[20];
  double layer_2[20];
  double layer_3[20];
  double layer_4[20];
  
  //lidar2
  double lay1[20];
  double lay2[20];
  double lay3[20];
  double lay4[20];
  double lay5[20];
  double lay6[20];
  double lay7[20];
  double lay8[20];
  
  double detectionvert = 0;
 
  
  int d = 0;
  bool over = false;
  bool ro = false;
  bool lidarsensors = false;
  
  bool onobject = false;
  bool wait = true;
  bool diff = false;
  
  // Main loop
 while (wb_robot_step(timestep) != -1) {
 //object detection starts here. 
    
    const float *layer1 = wb_lidar_get_layer_range_image(lidar, 0);
    const float *layer2 = wb_lidar_get_layer_range_image(lidar, 2);
    const float *layer3 = wb_lidar_get_layer_range_image(lidar, 5);
    const float *layer4 = wb_lidar_get_layer_range_image(lidar, 6);
    
    const float *layer41 = wb_lidar_get_layer_range_image(lidar2, 1);
    const float *layer42 = wb_lidar_get_layer_range_image(lidar2, 2);
    const float *layer43 = wb_lidar_get_layer_range_image(lidar2, 3);
    const float *layer44 = wb_lidar_get_layer_range_image(lidar2, 4);
    const float *layer45 = wb_lidar_get_layer_range_image(lidar2, 5);
    const float *layer46 = wb_lidar_get_layer_range_image(lidar2, 6);
    const float *layer47 = wb_lidar_get_layer_range_image(lidar2, 7);
    const float *layer48 = wb_lidar_get_layer_range_image(lidar2, 8);
    
    double distance_sensor_value = wb_distance_sensor_get_value(distance_sensor);
    
    
    if (lidarsensors == true){
      /*Alustan layerien listat, jonne laitan lidarin jokaisen
      layerin alkuarvon.*/
      if (d == 0){
        for ( int i = 0; i < 20; i++){
          printf("lidar layer1 %f", *(layer1+i));
          layer_1[i] = *(layer1+i);                      
        }
        printf("\n");
        for ( int i = 0; i < 20; i++){
          printf("lidar layer2 %f", *(layer2+i));
          layer_2[i] = *(layer2+i);
        }
        printf("\n");
        for ( int i = 0; i < 20; i++){
          printf("lidar layer3 %f", *(layer3+i));
          layer_3[i] = *(layer3+i);
      
        }
        for ( int i = 0; i < 20; i++){
          printf("lidar layer4 %f", *(layer4+i));
          layer_4[i] = *(layer4+i);                      
        }
        
                
              for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer1 %f", *(layer41+i));
                  lay1[i] = *(layer41+i);
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer2 %f", *(layer42+i));
                  lay2[i] = *(layer42+i);
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer3 %f", *(layer43+i));
                  lay3[i] = *(layer43+i);
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer4 %f", *(layer44+i));
                  lay4[i] = *(layer44+i);
              
                }
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer5 %f", *(layer45+i));
                  lay5[i] = *(layer45+i);
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer6 %f", *(layer46+i));
                  lay6[i] = *(layer46+i);
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer7 %f", *(layer47+i));
                  lay7[i] = *(layer47+i);
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){
                  printf("lidar2 layer8 %f", *(layer48+i));
                  lay8[i] = *(layer48+i);
                }
      }else{
      /*havaitaan objekti ja pysähdytään/liikutaan taaksepäin 
      samaan linjaan objektin kanssa.*/
        for ( int i = 0; i < 20; i++){
        
          float m = layer_1[i] - *(layer1+i);
          
          if (m > 0.05 || m < -0.05){
            speed = 0.0;
            search = false;
            ro = true;
            target_velocity = 0.5;
            target_position = -1;
            
          }
        }
        printf("\n");
        for ( int i = 0; i < 20; i++){
         
          float m = layer_2[i] - *(layer2+i);
          
          if (m > 0.05 || m < -0.05){
            speed = 0.1;
            search = false;
            ro = true;
            target_velocity = 0.5;
            target_position = -1;
          }
        }
        printf("\n");
        for ( int i = 0; i < 20; i++){
       
          float m = layer_3[i] - *(layer3+i);
          
          if (m > 0.05 || m < -0.05){
            speed = 0.13;                       
            printf("lidar layer3 %f", *(layer4+i));
          }
        }
        printf("\n");
        for ( int i = 0; i < 20; i++){
         
          float m = layer_4[i] - *(layer4+i);
          
          if (m > 0.05 || m < -0.05){
            speed = 0.2;
            leftright = 0.07;         
            over = true;
            printf("lidar layer4 %f", *(layer4+i));
           if (onobject == false){
              wait = false;
              start_time = wb_robot_get_time();
              onobject = true;
              
            }
          }
      
        }
         printf("\n");
         
        //odotetaan että            
         printf("speed %f", speed);
        //move left and right
        if ( over == true ){
                for ( int i = 0; i < 20; i++){     
                        float m = lay1[i] - *(layer41+i);                      
                        if (m > 0.05 || m < -0.05){
                          leftright = -0.6;
                          printf("lidar2 layer1 %f", m);
                        }
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){                  
                      float m = lay2[i] - *(layer42+i);                     
                      if (m > 0.05 || m < -0.05){
                        leftright = -0.4;
                        printf("lidar2 layer2 %f", m);
                      }
                }
                 printf("\n");
                for ( int i = 0; i < 20; i++){                  
                      float m = lay3[i] - *(layer43+i);                     
                      if (m > 0.05 || m < -0.05){
                        leftright = -0.2;
                        printf("lidar2 layer3 %f", m);
                      }
                }
                 printf("\n");
                for ( int i = 0; i < 20; i++){                
                      float m = lay4[i] - *(layer44+i);                    
                      if (m > 0.05 || m < -0.05){
                        leftright = -0.1;
                        printf("lidar2 layer4 %f", m);
                      }
                  
                    }
                printf("\n");
                for ( int i = 0; i < 20; i++){                     
                      float m = lay5[i] - *(layer45+i);                     
                      if (m > 0.05 || m < -0.05){
                        leftright = 0.1;
                         printf("lidar2 layer5 %f", m);
                      }
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){                     
                      float m = lay6[i] - *(layer46+i);                     
                      if (m > 0.05 || m < -0.05){
                        leftright = 0.2;
                        printf("lidar2 layer6 %f", m);
                      }
                }
               printf("\n");
                for ( int i = 0; i < 20; i++){                     
                      float m = lay7[i] - *(layer47+i);                      
                      if (m > 0.05 || m < -0.05){
                        leftright = 0.4;
                        
                        printf("lidar2 layer7 %f", m);                        
                      }
                }
                printf("\n");
                for ( int i = 0; i < 20; i++){                      
                      float m = lay8[i] - *(layer48+i);                     
                      if (m > 0.05|| m < -0.05){
                        leftright = 0.6;
                        printf("lidar2 layer8 %f", m);
                      }              
                }
                printf("\n");
          //liikutaan objektin päälle sivuttaissuunnassa. 
              
        
        }
                                   
      }
      printf("rolling %f", leftright);
      d = 1;
      
    }
    //object detection ends here. 
        const double time = wb_robot_get_time();  // in seconds.
        
        printf("Distance to ground: %f\n", distance_sensor_value);

        
        detected = true;
        
        if(time - start_time >= 20 && !wait){
          
          detected = false;
          
        }
        
        // Check the distance condition
        if (distance_sensor_value < 780.0 && !detected) {
            lidarsensors = false;
            leftright = 0.065; 
            speed = 0.14; 
           
            target_altitude = 0.4;
            detected = true;
            if (distance_sensor_value < 500.0){
              target_altitude = 0.7;
             
              
            }
            if (distance_sensor_value < 400.0){
              target_altitude = 0.7;
              target_position = -0.03;
              target_velocity = 0.3;
              
            }
            if (distance_sensor_value < 300.0){
              target_altitude = 0.5;
             
              
            }
            if (distance_sensor_value < 100.0){
              target_altitude =0.2;
              
              
            }
            if (time - start_time >= 35){
               target_altitude = 1.0;
               wb_distance_sensor_disable(distance_sensor);
            }
            
            
        }
        
        
    



    // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];
   
    // Blink the front LEDs alternatively with a 1 second rate.
     // kotiin
    double x = wb_gps_get_values(gps)[0];
    double y = wb_gps_get_values(gps)[1];
    double x2 = wb_gps_get_values(gps2)[0];
    double y2 = wb_gps_get_values(gps2)[1];
    double e = pow(x2,2);
    double u = pow(y2,2);
    double x1 = x;
    double y1 = y;
          
    if ( search == true){
      if (search1[n][0] == 6.5 && search1[n][1] == 5.0){
        search = false;
        searchh();
        home = true;
      }
      x1 = x - search1[n][0];
      y1 = y - search1[n][1];
      printf("distance %f\n", search1[n][0]);
      printf("distance %f\n", search1[n][1]);
      e = pow(x1,2);
      u = pow(y1,2);
    }
    double dy= y-y2;
    double dx= x-x2;
    
    double direction_to_home = atan2(y1,x1)*(180.0 / 3.14159265);
    double direction = atan2(dy,dx)*(180.0 / 3.14159265)+180;
    
    if (direction_to_home < 0){
      direction_to_home = atan2(y1,x1)*(180.0 / 3.14159265)+180;
      direction = atan2(dy,dx)*(180.0 / 3.14159265);
    };
    if (direction < 0){
      direction = atan2(dy,dx)*(180.0 / 3.14159265)+360;
    }
    
    double nu= e + u;
    double distance = sqrt(nu);
   
    int direction_r = ceil(direction);
    int direction_home = ceil(direction_to_home);
    printf("angle %d\n", direction_r);
    printf("angle2 %d\n", direction_home);
    printf("distance %f\n", distance);
    int difference = direction_r - direction_home;
    if(difference < 0){
      difference = difference * -1;
    }
    
    // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);

    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    
    if (ro == true){
      pitch_disturbance = speed;
    }
    roll_disturbance = leftright;
    
    
    if (home == true){
      wait = true;
      target_altitude = 1;
      printf("speed %f", speed);
      /*wb_lidar_disable(lidar);
      wb_lidar_disable(lidar2);*/
      speed = -0.5;
      d = 0;
      over = false;
      ro = false;
      if(direction_r > direction_home && difference < 15){
        yaw_disturbance = 0.04;
      }else if(direction_r < direction_home && difference < 80){
        yaw_disturbance = 0.1;
      }else if(direction_r < direction_home){
        yaw_disturbance = 0.7;
      };
      
      if(direction_r > direction_home && difference < 15){
        yaw_disturbance = -0.04;
      }else if(direction_r > direction_home && difference < 80){
        yaw_disturbance = -0.1;
      }else if(direction_r > direction_home){
        yaw_disturbance = -0.7;
      };
      if( difference < 5){
        if (distance > 2){
          pitch_disturbance = -2.0;
        }else if(distance > 0.3){
          pitch_disturbance = -0.5;
        }else if(distance > 0.0){
          pitch_disturbance = 0.13;
        };
       };
       
       if (distance < 0.2){
         home = false;
         
       };
        
    };
    if (search == true){
      target_altitude = 1.0;
      printf("speed %f", speed);
    
      if(direction_r > direction_home && difference < 15){
        yaw_disturbance = 0.04;
      }else if(direction_r < direction_home && difference < 80){
        yaw_disturbance = 0.1;
      }else if(direction_r < direction_home){
        yaw_disturbance = 0.7;
      };
      
      if(direction_r > direction_home && difference < 15){
        yaw_disturbance = -0.04;
      }else if(direction_r > direction_home && difference < 80){
        yaw_disturbance = -0.1;
      }else if(direction_r > direction_home){
        yaw_disturbance = -0.7;
      };
      
      if( difference < 10){
        if (distance > 2){
          pitch_disturbance = speed;
         
         
        }
        else if(distance > 0.3){
          
          
          pitch_disturbance = -0.5;
        }else if(distance > 0.0){
        
          pitch_disturbance = 0.0;
        };
       };
       
       if (distance < 0.4 && in_destination == false){
         n = n - 1; 
           
         onobject = false;
         lidarsensors = true; 
         in_destination = true;
         distance_sensor_value =1000;
           detected = false;
            /*wb_lidar_enable(lidar, timestep);
            wb_lidar_enable(lidar2, timestep);*/
            wb_distance_sensor_enable(distance_sensor, timestep);
       };
       if (distance > 0.4 && in_destination == true){
          
         in_destination = false;
       };
    }
    
    
      
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          pitch_disturbance = -2.0;
          
          break;
        case WB_KEYBOARD_DOWN:
          pitch_disturbance = 2.0;
          break;
        case WB_KEYBOARD_ALT:
          
          break;
        case WB_KEYBOARD_RIGHT:
          yaw_disturbance = -1.3;
          break;
        case WB_KEYBOARD_LEFT:
          yaw_disturbance = 1.3;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          target_position = 0.2;
          target_velocity = 0.3;
          printf("pos: %f\n", target_position);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          target_velocity = 0.3;
          target_position = -1;
          printf("pos: %f\n", target_position);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          
          printf("target altitude: %f [m]\n", target_altitude);
          target_altitude = 1.0;
          home = true;
            speed=-1.0;
      
      
          search = false;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          search = true;
        
          printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = wb_keyboard_get_key();
    }

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
    
    wb_motor_set_velocity(RM, target_velocity);
    wb_motor_set_position(RM, target_position);
    
    wb_motor_set_velocity(RM1, target_velocity);
    wb_motor_set_position(RM1, target_position);
    
    wb_motor_set_velocity(RM2, target_velocity);
    wb_motor_set_position(RM2, target_position);
    
    wb_motor_set_velocity(RM3, target_velocity);
    wb_motor_set_position(RM3, target_position);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}