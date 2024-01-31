//Sistema de direccion
//Elaboraron: Ali, Pedro, Pablo, Julio, Ana, Tom, Alejandro, Dr. Reyes


//Librerías
#include <SPI.h>
#include <mcp2515.h>
#include "Adafruit_VL53L0X.h"
// #include <avr/wdt.h>
#include <EEPROM.h>
#include "esp_task_wdt.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define backwardsPin 32  //backwardsPin - Dirección Pololu
#define CsPololuPin 34   //CsPolulu - Corriente Pololu
#define pwmPin 25        //pwmPin - PWM Pololu by Frenos


//Inicialización de variables
int SetPoint = 0;  //Valor deseado del sistema de control (Rango de 0 a 100)
int Sensor = 0;    // Valor medido del sensor de retroalimentacion (Rango de 0 a 100)
int Output = 0;    // Valor de control al actuador (Rango de 0 a 100)
int Error = 0;     //Diferencia entre el SetPoint y el valor del Sensor


int CANEnable = 1;  // Bandera para habilitar la lectura del CAN

bool FlagEnable;
int Pos = 0;
#define vel 100



struct can_frame msgSend;     //Estructura de mensaje para almacenar datos salientes
struct can_frame msgRecieve;  //Estructura de mensaje para almacenar datos entrantes



MCP2515 mcp2515(5);  // Indica en que pin de arduino está conectado CS del modulo CANA

TaskHandle_t taskRead_tof;
QueueHandle_t distance_queue;  //Declaración de una cola para comunicar los 2 cores

void setup() {
  //wdt_disable();

  //Declaración de pines
  pinMode(backwardsPin, OUTPUT);  //backwardsPin - Dirección Pololu
  pinMode(CsPololuPin, INPUT);    //CsPolulu - Corriente Pololu
  pinMode(pwmPin, OUTPUT);        //pwmPin - PWM Pololu by dirección


    Serial.begin(115200);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(pwmPin, 0);
  //Inicialización de módulo CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);  //Bitrate del CAN Bus
  mcp2515.setNormalMode();          //Modo del CAN

  //Test para verificar la inicialización del sensor
  delay(1000);
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1) {
      Serial.println("Adafruit VL53L0X test");
    }  // Bucle infinito
  }
  Wire.setTimeout(300);

  //Configuración del tiempo de espera max. para la comunicación I2C utilizando la calse Wire


  xTaskCreatePinnedToCore(taskRead_sensor, "TaskRead_tof", 10000, NULL, tskIDLE_PRIORITY, &taskRead_tof, 0);
  distance_queue = xQueueCreate(1, sizeof(Sensor));


  //wdt_enable(WDTO_120MS);
}

void taskRead_sensor(void* parameter) {
  esp_task_wdt_init(1, true);
  //Task dedicada a la lectura del sensor de tiempo de vuelo
  int mapped_distance = 0;
  VL53L0X_RangingMeasurementData_t measure;
  while (true) {
    lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!
                                       //Se guarda la medición hecha y mapeada a un rango de entre 0-100 en la var. "Pos"
    Pos = measure.RangeMilliMeter;
    mapped_distance = map(Pos, 39, 104, 0, 100);
    xQueueSend(distance_queue, &mapped_distance, portMAX_DELAY);
    Serial.println(Pos);
    //Wire.setTimeout(3);
    esp_task_wdt_reset();
  }
}

void loop() {
  //Espera mensaje CAN del módulo de Control
if (CANEnable){
  if (mcp2515.readMessage(&msgRecieve) == MCP2515::ERROR_OK) {
    //Verifica que sea un mensaje para el sistema de dirección
    if (msgRecieve.can_id == 0x53) {

      //Se establece el SetPoint
      SetPoint = int(msgRecieve.data[0]);
      //Se activa la vandera para iniciar lazo de control
      FlagEnable = true;

      //Envia la posición medida guardada en void setup
      msgSend.can_id = 115;
      msgSend.can_dlc = 1;
      msgSend.data[0] = Sensor;
      mcp2515.sendMessage(&msgSend);

      //Se reinicia el módulo de CAN
      mcp2515.reset();
      mcp2515.setBitrate(CAN_125KBPS);  //Bitrate del CAN Bus
      mcp2515.setNormalMode();          //Modo del CAN
      delay(100);
    }
  }
}

  if (Serial.available() > 0) {
    ReadSerial();
  }


  int receivedDistance = 0;
  if (xQueueReceive(distance_queue, &receivedDistance, pdMS_TO_TICKS(100))) {  //Lee la cola, si está vacia espera hasta 100ms para recibir un dato
    Sensor = receivedDistance;                                                //Guarda el valor de la cola en SetPoint
  }
  //Calcula el error entre el SetPoint y la posición medida
  Error = SetPoint - Sensor;
  //    EEPROM.write(direccion, valor);

  //Acción de control


  if (Error > 5) {
    //Serial.println("Errors 5");3
    ledcWrite(0, vel);                 //pin 9 velocidad
    digitalWrite(backwardsPin, HIGH);  //Izquierda
  }

  if (Error < -5) {
    //Serial.println("Errors -5");
    ledcWrite(0, vel);
    digitalWrite(backwardsPin, LOW);  //Derecha
  }

  if (5 > Error && Error > -5) {
    ledcWrite(0, 0);
    digitalWrite(backwardsPin, HIGH);
    FlagEnable = false;
  }



  //wdt_reset();
}




void ReadSerial() {
  char lectura = Serial.read();
  if (lectura == '*') {
    Serial.print("B: ");
    Serial.print(SetPoint);
    Serial.print(", ");
    Serial.print(Sensor);
    Serial.print(", ");
    Serial.println(Error);
  }
  if (lectura == 's') {
    //Si recibe una s espera 3 valores para indicar el setPoint
    while (Serial.available() == 0)
      ;
    char centenas = Serial.read() - '0';
    while (Serial.available() == 0)
      ;
    char decenas = Serial.read() - '0';
    while (Serial.available() == 0)
      ;
    char unidades = Serial.read() - '0';

    SetPoint = centenas * 100 + decenas * 10 + unidades;
  }

  if (lectura == 'e') {
    //Si recibe una e habilita la lectura del CAN
    CANEnable = 1;
  }
  if (lectura == 'd') {
    //Si recibe una d deshabilita la lectura del CAN
    CANEnable = 0;
  }
}
