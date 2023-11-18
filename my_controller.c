#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/gps.h>
#include <stdlib.h>
#include <time.h>


#define TIME_STEP 32
#define QtddSensoresProx 8
#define QtddLeds 10
#define MAX_SPEED 6.28
#define MOVE_THRESHOLD 0.03
#define CHECK_THRESHOLD 9

double calculateMean(int array[], int size);
double calculateStandardDeviation(int array[], int size);

int main(int argc, char **argv) {

  int i=0;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;

  for (i=0; i < 256; i++) texto[i]='0';
  wb_robot_init();

  //configurei MOTORES

  WbDeviceTag MotorEsquerdo, MotorDireito;
  // instacia os motores
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  // inicializa posicoes dos motores
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  // inicializa velocidades
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);

  //configurei Sensores de Proximidade
  WbDeviceTag SensorProx[QtddSensoresProx];

  SensorProx[0] = wb_robot_get_device("ps0");
  SensorProx[1] = wb_robot_get_device("ps1");
  SensorProx[2] = wb_robot_get_device("ps2");
  SensorProx[3] = wb_robot_get_device("ps3");
  SensorProx[4] = wb_robot_get_device("ps4");
  SensorProx[5] = wb_robot_get_device("ps5");
  SensorProx[6] = wb_robot_get_device("ps6");
  SensorProx[7] = wb_robot_get_device("ps7");

  wb_distance_sensor_enable(SensorProx[0],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[1],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[2],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[3],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[4],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[5],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[6],TIME_STEP);
  wb_distance_sensor_enable(SensorProx[7],TIME_STEP);


  //config leds
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0],-1);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  bool collision = false;
  bool wall_collision = false;
  bool spinning = false;
  bool stop = false;
  int nex_direction = -1;
  double distance = 0.0;
  double old_gps_sum = 0.0;
  int steps_to_check = 0;
  double distances[CHECK_THRESHOLD];
  double speed = 0.0;
  bool lateral_collision = false;

  for (i = 0; i < CHECK_THRESHOLD; i++) {
    distances[i] = 0.0;
  }

  const double* gps_value = wb_gps_get_values(gps);
  double old_gps_value[3];
  for (i = 0; i < 3; i++) {
    old_gps_value[i] = gps_value[i];
  }

  while (wb_robot_step(TIME_STEP) != -1) {

    for (i = 0; i < 256; i++) texto[i] = 0;

    gps_value = wb_gps_get_values(gps);


    if (!spinning) {
      // adiciona aleatoriedade na rotacao
      // para evitar que o robo fique preso
      int directions[3] = {3, 4, 5};
      srand(time(NULL));
      int random_idx = rand() % 3;
      nex_direction = directions[random_idx];
    }

    collision = LeituraSensorProx[0] > 100 
      || LeituraSensorProx[1] > 100
      || LeituraSensorProx[7] > 100;

    int count_collisions = 0;
    for (i=0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i])-60;
      if (i == 0 || i == 1 || i == 7) {
        if (LeituraSensorProx[i] > 100) {
          count_collisions++;
        }
        sprintf(texto,"%s| ps%d: %5.2f ", texto, i, LeituraSensorProx[i]);
      }
    }
    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);
    
    if (collision && count_collisions >= 2) {

      printf("COLLISION\n");
      
      distance = sqrt(pow(gps_value[0] - old_gps_value[0], 2) + pow(gps_value[1] - old_gps_value[1], 2));
      printf("DISTANCE = %f\n", distance);
      distances[steps_to_check] = distance;
      double new_speed = wb_gps_get_speed(gps);
      printf("speed on collision = %f\n", new_speed);
      printf("speed before collision = %f\n", speed);
      if (!stop) {
        stop = speed - new_speed > 0.025;
        steps_to_check == CHECK_THRESHOLD + 1;
      }
      printf("steps_to_check = %d\n", steps_to_check);
      if (steps_to_check > CHECK_THRESHOLD) {
        printf("CHECKING\n");
        printf("stop = %s\n", stop ? "true" : "false");
        if (!stop) {
          printf("GOAL\n");
          printf("%s\n", texto);
          // desliga motores
          wb_motor_set_velocity(MotorEsquerdo,0);
          wb_motor_set_velocity(MotorDireito, 0);
          // apaga leds
          wb_led_set(Leds[0], 0);
          break;
        }
        spinning = true;
      }

      // encurta o tempo de rotacao 
      // ao evitar uma rotacao maior que 180 graus
      if (nex_direction <= 4 && spinning) {
        // gira para esquerda
        AceleradorDireito = 1;
        AceleradorEsquerdo = -1;
      } else if (spinning) {
        // gira para direita
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
      }

      if (steps_to_check > CHECK_THRESHOLD || steps_to_check == 0) {
        for (i = 0; i < 3; i++) {
          old_gps_value[i] = gps_value[i];
        }
        steps_to_check = 0;
      }
      steps_to_check++;

    } else if (nex_direction != -1 && LeituraSensorProx[nex_direction] > 100 && !collision) {
      spinning = false;
      wall_collision = false;
      stop = false;
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
      for (i = 0; i < CHECK_THRESHOLD; i++) {
        distances[i] = 0.0;
      }
      steps_to_check = 0;
    } else {
      speed = wb_gps_get_speed(gps);
      printf("speed = %f\n", speed);
      steps_to_check = 0;
    }

    //printf("%s\n", texto);
    
    wb_motor_set_velocity(MotorEsquerdo,6.28*AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);

  };

  wb_robot_cleanup();

  return 0;
}

double calculateMean(int array[], int size) {
    double sum = 0;
    for (int i = 0; i < size; i++) {
        sum += array[i];
    }
    return sum / size;
}

double calculateStandardDeviation(int array[], int size) {
    double mean = calculateMean(array, size);
    double sumSquaredDifferences = 0;

    for (int i = 0; i < size; i++) {
        double difference = array[i] - mean;
        sumSquaredDifferences += difference * difference;
    }

    double variance = sumSquaredDifferences / size;
    double standardDeviation = sqrt(variance);
    return standardDeviation;
}
