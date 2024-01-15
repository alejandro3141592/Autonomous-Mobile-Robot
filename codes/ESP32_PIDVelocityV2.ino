//Código para el control de la velocidad del AMR1
//Elaborado para ESP32

/*-----------Inclusión de Librerías-----------*/
#include <mcp2515.h>      //Librería del modulo CAN
#include "driver/pcnt.h"  // Librería para contar los pulsos del sensor
#include <PID_v1.h>       //Librería para implementar un PID
/*--------------Inicializaciones--------------*/
MCP2515 mcp2515(5);     //Inicialización del bus CAN
TaskHandle_t taskRFBT;  //Crea una Task dedicada al calculo del control PID

/*----------Definición de Variables-----------*/
//Variables usadas para el conteo de pulsos
#define PCNT_FREQ_UNIT PCNT_UNIT_0  //Unidad del módulo que se usará para el conteo de pulsos
#define HALL_SENSOR_PIN 14          // Pin del sensor de efecto Hall
#define PCNT_H_LIM_VAL 10000        //Limite máximo al que puede llegar el contador
#define PCNT_FILTER_VAL 1000        // Define el valor para el filtro del contador, el màximo es 1023

//Variables usadas para controlar el tren motriz
#define SPEED_PIN_2 32    //Low Gear
#define SPEED_PIN_1 33    //High Gear
#define BACKWARDS_PIN 25  //Reversa
#define LED_PIN 16        //Led
#define ENABLE_PIN 17     //Enable

#define CANIDReceivePWM 84  //ID del CAN del modulo de tren motriz con lazo abierto 84
#define CANIDSendPWM 116    //ID del CAN del modulo de tren motriz con lazo abierto de respuesta 116

#define CANIDReceiveVelocity 85  //ID del CAN del modulo de frenos de respuesta 85
#define CANIDSendVelocity 117    //ID del CAN del modulo de frenos de respuesta 117

#define CANIDParamsPID 1  //ID del CAN del modulo de frenos 66

//Variables usadas para configurar el PWM
#define LEDC_CHANNEL_0 0      //Canal de pwm
#define LEDC_TIMER_12_BIT 12  //Resolución del pwm
#define LEDC_BASE_FREQ 5000   //Frecuencia del PWM
#define PWM_PIN 12            //Pin del pwm


QueueHandle_t setPointQueue;  //Declaración de una cola
QueueHandle_t pidQueue;       //Declaraicón de una cola para el PID

hw_timer_t* timer = NULL;  //Define un timer

const float CONST_MPS = 0.0758;  //Constante para convertir de pulsos a velocidad
//const float CONST_MPS = 0.2878;

bool flagInterrupt = false;  //Bandera para saber si ya se actuvo la interrupción del timer


//Variables usadas para mensajes CAN
struct can_frame msgSend;     //Estructura de mensaje para almacenar salientes
struct can_frame msgRecieve;  //Estructura de mensaje para almacenar entrantes
bool CANEnable = 1;
bool lazoCerrado = 0;
//Variables usadas para controlar el Tren Motriz
int pwmValue;
bool enableValue, backwardsValue, speedValue1, speedValue2;
bool ledState = false;


//Variables usadas para el PID
double Kp = 0.2;  // Proportional gain
double Ki = 1.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

struct PIDParameters {
  float Kp;
  float Ki;
  float Kd;
  bool enable;
};


int MPS = 0;                        //Metros por segundo
int CMPS = 0;                       //Centimetros por segundo
double pidOutput = 0.0;             //Output del PID
double speedSetpoint_pulses = 0.0;  //Set Point en pulsos
double speedSetpoint_mps = 5.0;     //Set Point en metros por segundo
volatile float Hz = 0;              //Heartz
double metersPerSecond = 0;         //Metros por segundo


//Variables para control
float SetPoint = 0;  //Valor deseado del sistema de control 
int16_t Sensor = 0;    // Valor medido del sensor de retroalimentacion 
int Output = 0;    // Valor de control al actuador (Rango de 0 a 255)
float Error = 0;     //Diferencia entre el SetPoint y el valor del Sensor


/*-------------- Funciones dedicadas al contador de pulsos--------------*/
void initPCNT() {
  //Rutina que inicializa el contador de pulsos
  pcnt_config_t pcntFreqConfig = {};
  pcntFreqConfig.pulse_gpio_num = HALL_SENSOR_PIN;         //Define el pin donde lee los pulsos
  pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                //Define el modo de conteo, en este caso va a ser incremental
  pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;           //Define el límite máximo del contador
  pcntFreqConfig.unit = PCNT_FREQ_UNIT;                    //Define la unidad del módulo encargada del conteo
  pcntFreqConfig.channel = PCNT_CHANNEL_1;                 //Define el canal dedicado al conteo
  pcnt_unit_config(&pcntFreqConfig);                       //Actualiza la configuración
  pcnt_counter_pause(PCNT_FREQ_UNIT);                      //Pausa el contador
  pcnt_counter_clear(PCNT_FREQ_UNIT);                      //Limpia el contador
  pcnt_set_filter_value(PCNT_FREQ_UNIT, PCNT_FILTER_VAL);  //Define una valor de filtro
  pcnt_filter_enable(PCNT_FREQ_UNIT);                      //Habilita el contador
  pcnt_counter_resume(PCNT_FREQ_UNIT);                     //Quita la pausa del contador
}

void readResetPCNT() {
  //Función que lee el contador y lo resetea el contador
  pcnt_get_counter_value(PCNT_FREQ_UNIT, &Sensor);  //Guarda el calor del contador en Sensor
  pcnt_counter_clear(PCNT_FREQ_UNIT);                     // Limpia el contador
}

/*--------------Funciones dedicadas al Timer--------------*/
void IRAM_ATTR onTimer() {  //Executes on Core 1
  //Rutina que se ejecuta con la interrupción del timer, habilita la bandera y manda a llamar el reseteo del contador
  flagInterrupt = true;  //Habilita la bandera de que ya pasaron 0.3s
  readResetPCNT();       //Llama a la funcion para resetear el conteo
}

void initTimer() {
  //Rutina que configura la interrupción por timer cada 300 mS
  timer = timerBegin(3, 80, true);              //Inicializa el timer dividiendolo entre 80
  timerAttachInterrupt(timer, &onTimer, true);  //Declara la interrupción asociada al timer
  timerAlarmWrite(timer, 300000, true);         //Define cuando se va a llamar a la interrupción, cada 0.3s
  timerAlarmEnable(timer);                      //Habilita la alarma
}

/*--------------Set up--------------*/
void setup() {
  // Inicializacion del Serial
  Serial.begin(115200);
  Serial.println("STEPUPSTART");
  Serial.setTimeout(200);  //Timeout de lectura

  initPCNT();   // Inicializacion del contador de pulsos
  initTimer();  // Inicializacion del timer

  const int QUEUE_DEPTH = 10;                                   //Tamaño de la cola
  pidQueue = xQueueCreate(QUEUE_DEPTH, sizeof(PIDParameters));  //Crea la cola para el PID, de tamaño 10
  setPointQueue = xQueueCreate(1, sizeof(double));              //Crea una cola de tamaño 1

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);  //Inicializa el PWM del canal 0
  ledcAttachPin(PWM_PIN, LEDC_CHANNEL_0);                        //Asigna el canal 0 al pin PWM_PIN


  //Inicialización de módulo CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);  //Bitrate del CAN Bus
  mcp2515.setNormalMode();          //Modo del CAN
  xTaskCreatePinnedToCore(taskRFBT_code, "TaskRFBT", 10000, NULL, tskIDLE_PRIORITY, &taskRFBT, 0);

  //Declaración de pines
  pinMode(SPEED_PIN_2, OUTPUT);
  pinMode(SPEED_PIN_1, OUTPUT);
  pinMode(BACKWARDS_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  //Como señales de entrada para el tren motriz, si no se manda el tren motirz no acepta otros valores de PWM
  analogWrite(PWM_PIN, 0);
  digitalWrite(ENABLE_PIN, 1);
  digitalWrite(BACKWARDS_PIN, 1);
  delay(500);
  analogWrite(PWM_PIN, 0);

  delay(500);
  Serial.println("SETUPEND");
}


/*--------------Task que calcula el PID--------------*/
void taskRFBT_code(void* parameter) {
  //Tarea dedicada al calculo del PID
  
  while (true) {
    LeerQueues();
    if (lazoCerrado == 1) { //Si está en modo de lazo cerrado, ocupa el PID
      if (flagInterrupt) {
        //Si pasaron 300mS
        ComputeOutput();
        flagInterrupt = false;  //Desactiva la bandera del timer

      }
    }
  }
}


/*--------------Loop--------------*/
void loop() {  //Executes on Core 1
  //Revisa si llego un nuevo mensaje CAN
  if (CANEnable) {
    if (mcp2515.readMessage(&msgRecieve) == MCP2515::ERROR_OK) {
      ReadCAN();
    }
  }

  if (Serial.available() > 0) {
    ReadSerial();
  }
}



void ReadCAN() {
  //Rutina que lee el mensaje de CAN y filtra el mensaje deseado
  if (msgRecieve.can_id == CANIDParamsPID) {
    ConfigureParamsPID();  //Configura las constantes del PID, y si se va a trabajar o no en lazo cerrado
  }

  if (lazoCerrado) {
    if (msgRecieve.can_id == CANIDReceiveVelocity) {
      ConfigureVelocitySetPoint();
    }
  }

  if (lazoCerrado == 0) {
    if (msgRecieve.can_id == CANIDReceivePWM) {
      ConfigurePWM();
    }
  }
}


void ReadSerial() {
  //Rutina que lee el mensaje del Serial y sirve para debuggear

  char lectura = Serial.read();
  if (lectura == '*') {
    if (lazoCerrado) {
      Serial.print("TC: ");
      Serial.print(SetPoint);
      Serial.print(", ");
      Serial.print(Sensor);
      Serial.print(", ");
      Serial.println(Error);

    } else {
      Serial.print("TA: ");
      Serial.print(pwmValue);
      Serial.print(", ");
      Serial.print(enableValue);
      Serial.print(", ");
      Serial.println(backwardsValue);
      Serial.print(", ");
      Serial.println(speedValue1);
      Serial.print(", ");
      Serial.println(speedValue2);
    }
  }
  if (lectura == 's') {
    //Si recibe una s espera 3 valores para indicar el setPoint
    if (lazoCerrado) {
      while (Serial.available() == 0);
      char centenas = Serial.read() - '0';
      while (Serial.available() == 0);
      char decenas = Serial.read() - '0';
      while (Serial.available() == 0);
      char unidades = Serial.read() - '0';
      SetPoint = centenas * 100 + decenas * 10 + unidades;     //Obtiene el setpoint en metros por segundo
      speedSetpoint_pulses = SetPoint / (CONST_MPS);           //Convierte el setpoint de metros por segundo a nunero de pulsos
      double setpointToSend = speedSetpoint_pulses;
      xQueueSend(setPointQueue, &setpointToSend, portMAX_DELAY);  //Manda los parametros a la cola setPointQueue
    }else {
      while (Serial.available() == 0);
      char centenas = Serial.read() - '0';
      while (Serial.available() == 0);
      char decenas = Serial.read() - '0';
      while (Serial.available() == 0);
      char unidades = Serial.read() - '0';
      pwmValue = centenas * 100 + decenas * 10 + unidades;
    }
  }
  if (lectura == 'e') {
    //Si recibe una e habilita la lectura del CAN
    CANEnable = 1;
  }
  if (lectura == 'd') {
    //Si recibe una d deshabilita la lectura del CAN
    CANEnable = 0;
  }
    if (lectura == 'c') {
    //Si recibe una c habilita el lazo cerrado
    lazoCerrado = 1;
  }
  if (lectura == 'd') {
    //Si recibe una a  habilita el lazo abierto
    lazoCerrado = 0;
  }
}

void ConfigureParamsPID() {
  PIDParameters pidParams;  // Estructura que guarda los parámetros del PID

  lazoCerrado = msgRecieve.data[0];       //Define si se usará lazo cerrado o abierto
  pidParams.enable = msgRecieve.data[0];  //Define si se usará lazo cerrado o abierto, y se manda al otro Task
  pidParams.Kp = msgRecieve.data[1];      //Kp
  pidParams.Ki = msgRecieve.data[2];      //Ki
  pidParams.Kd = msgRecieve.data[3];      //Kd


  if (xQueueSend(pidQueue, &pidParams, portMAX_DELAY) != pdPASS) {  //Manda los parametros a la cola PIDqueue
    // Error Handling
  }
}


void ConfigureVelocitySetPoint() {
  digitalWrite(8, HIGH);
  int units = int(msgRecieve.data[0]);     //Lee las unidades
  int decimals = int(msgRecieve.data[1]);  //Lee los decimales
  if (decimals > 99) {
    decimals = 99;  //Para que los decimales no sobrpasen 99
  }
  speedSetpoint_mps = units + decimals * 0.01;             //Obtiene el setpoint en metros por segundo
  speedSetpoint_pulses = speedSetpoint_mps / (CONST_MPS);  //Convierte el setpoint de metros por segundo a nunero de pulsos

  double setpointToSend = speedSetpoint_pulses;
  xQueueSend(setPointQueue, &setpointToSend, portMAX_DELAY);  //Manda los parametros a la cola setPointQueue

  //Guarda los valores recibidos por CAN en su respectiva variable
  enableValue = (msgRecieve.data[2] > 0) ? HIGH : LOW;
  backwardsValue = (msgRecieve.data[3] > 0) ? HIGH : LOW;
  speedValue1 = (msgRecieve.data[4] > 0) ? HIGH : LOW;
  speedValue2 = (msgRecieve.data[5] > 0) ? HIGH : LOW;

  //Activa o desactiva el resto de pines del tren motriz
  digitalWrite(ENABLE_PIN, enableValue);
  digitalWrite(BACKWARDS_PIN, backwardsValue);
  digitalWrite(SPEED_PIN_1, speedValue1);
  digitalWrite(SPEED_PIN_2, speedValue2);

  //Calcula la velocidad medida por el sensor de efecto Hall
  metersPerSecond = (Sensor * CONST_MPS) / 10.0;
  int MPS = int(metersPerSecond);
  int CMPS = int((metersPerSecond - MPS) * 100);

  //Envia la velocidad y el resto de datos por CAN
  msgSend.can_id = CANIDSendVelocity;
  msgSend.can_dlc = 6;
  msgSend.data[0] = MPS;
  msgSend.data[1] = CMPS;
  msgSend.data[2] = backwardsValue;
  msgSend.data[3] = speedValue1;
  msgSend.data[4] = speedValue2;
  msgSend.data[5] = Sensor;

  mcp2515.sendMessage(&msgSend);
}


void ConfigurePWM() {
  //Rutina que funciona en lazo abierto, le manda directo el valor de PWM que recibe por CAN
  digitalWrite(8, HIGH);
  //Guarda los valores recibidos por CAN en su respectiva variable
  pwmValue = int(msgRecieve.data[0]);
  enableValue = (msgRecieve.data[2] > 0) ? HIGH : LOW;
  backwardsValue = (msgRecieve.data[3] > 0) ? HIGH : LOW;
  speedValue1 = (msgRecieve.data[4] > 0) ? HIGH : LOW;
  speedValue2 = (msgRecieve.data[5] > 0) ? HIGH : LOW;

  //Manda las señales al tren motriz
  analogWrite(PWM_PIN, pwmValue);
  digitalWrite(ENABLE_PIN, enableValue);
  digitalWrite(BACKWARDS_PIN, backwardsValue);
  digitalWrite(SPEED_PIN_1, speedValue1);
  digitalWrite(SPEED_PIN_2, speedValue2);

  //Envia los datos del Tren motriz por CAN
  msgSend.can_id = CANIDSendPWM;
  msgSend.can_dlc = 5;
  msgSend.data[0] = pwmValue;
  msgSend.data[1] = enableValue;
  msgSend.data[2] = backwardsValue;
  msgSend.data[3] = speedValue1;
  msgSend.data[4] = speedValue2;

  mcp2515.sendMessage(&msgSend);
}



void LeerQueues() {
  double receivedSetpoint = 0.0;
  //Queque del Set Point
  if (xQueueReceive(setPointQueue, &receivedSetpoint, pdMS_TO_TICKS(100))) {  //Lee la cola, si está vacia espera hasta 100ms para recibir un dato
    SetPoint = receivedSetpoint;                                              //Guarda el valor de la cola en SetPoint
  }
  //Queque del PID
  PIDParameters receivedParams;
  if (xQueueReceive(pidQueue, &receivedParams, pdMS_TO_TICKS(100))) {  //Lee la cola de parámetros PID, si está vacia espera hasta 100ms para recibir un dato
    //Divide los valores recibidos entre 100, porqeu fueron recibidos por CAN, y CAN no acepta decimales.
    Kp = receivedParams.Kp / 100;
    Ki = receivedParams.Ki / 100;
    Kd = receivedParams.Kd / 100;
    lazoCerrado = receivedParams.enable;
  }
}


void ComputeOutput() {
  Error = (SetPoint - Sensor);  //Calcula el error

  // Calculate the integral term (sum of errors over time)
  float integral = (Error * 0.3) + integral;  //Calcula la integral como   I = error*dt + I
  //Limita la integral a un rango entre -255 y 255
  integral = (integral >= 255) ? 255 : integral;
  integral = (integral < -255) ? -255 : integral;

  //Si el tren motriz no comenzó a girar, reiniciabas el integral para mandarle un 0 al pwm y así habilitar el giro
  if (integral > 180 && Sensor < 10) {
    integral = 0;
  }


  float Output = Kp * Error + Ki * integral;  //Hace el caluclo del PI

  //Limita que la salida sea un valor entre 0 y 255
  Output = (int(Output) > 255) ? 255 : Output;
  Output = (int(Output) < 0) ? 0 : Output;

  //Como el tren motriz comienza a moverse con un PWM de 110, le sumamos ese valor a la salida, pero si el resultado de esa suma supera los 255, entonces lo limitamos a 255
  Output = min(int(Output + 110), 255);
  analogWrite(PWM_PIN, int(Output));  //Escribe el PWM
}