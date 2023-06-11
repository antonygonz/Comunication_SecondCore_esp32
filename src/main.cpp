#include <Arduino.h>

// Variables de la comunicacion
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
int contadorFiltro=0;
int Modo = 1;  // Modo de operacion 1: Modo manual (lazo abierto)
               //                   2: Modo automatico (lazo cerrado)
               
// Parametros para analisis a lazo abierto
float Potencia_1 = 0;     // valor inicial del cambio escalon. (min = 0)
float Potencia_2 = 50;   // Valor final del cambio escalon. (max = 100)

// Parametros para analisis a lazo cerrado
float Setpoint = 30;       // Setpoint
float Kc = 8; float Tao_I = 80;// Constantes de PID

int Tiempo0 = 0;     // Retardo
int SensorLM35 = 32;           // Pin A0 de entrada analogica para sensor LM35 (Variable de salida)
float Potencia = 0;  // Potencia inicial enviada al dimmer en rango de 0 a 100 (Variable de entrada)
int pin_disparo = 17; //Pin para disparar
int pin_cruce_cero = 16;//pin para detectar el cruce por zero
int last_CH1_state = 0;//anterior estado
int detectado = 0;//variable para indicar si se detecto un cruce por cero
int valor=0;

unsigned long Tiempo_Previo_loop1=0;
unsigned long Tiempo_Actual_loop1=0;
unsigned long Tiempo_previo = 0; //variable para medir tiempo inicial
unsigned long Tiempo_actual = 0; //variable para medir tiempo actual
int Read_Delay = 1000;     // Periodo de muestreo en milisegundos
int Temperatura = 0;       // Variable para medir la temperatura //CAMBIAR A DOUBLE SI SE PUEDE
int TempFiltro=0;//temperatura usada mientras se filtra
float sp = 0;    //probablemendte start point CAMBIAR
// Variables para PID
float PID_error = 0;
float previous_error = 0;
float PID_value = 0;
float Error_INT = 0;

void InterrupcionCruceZero(){
  detectado=1;
}


TaskHandle_t Task1;
void loop2(void *parameter){
  for(;;){
    
      Tiempo_actual = millis(); // Tiempo Actual    
      
      valor = map(Potencia,0,100,7600,10);
      if (detectado)
      {
        delayMicroseconds(valor);
        digitalWrite(pin_disparo,HIGH);
        delayMicroseconds(100);
        digitalWrite(pin_disparo,LOW);
        detectado=0;

      if(Tiempo_actual - Tiempo_previo >= Read_Delay){
        Tiempo_previo += Read_Delay;                
      
      //Temperatura = 5.0*100.0*analogRead(A)/1024.0;         //Lectura del sensor LM35

      if(Modo == 1){
        // Modo manual (lazo abierto)
        if(Tiempo_actual < Tiempo0) {
          Potencia = Potencia_1;
        }
        else if(Tiempo_actual >= Tiempo0) {
          Potencia = Potencia_2;
        } 
        
      }
      else if(Modo == 2){
      // Modo automatico (lazo cerrado)
      if(Tiempo_actual <= Tiempo0) {
        PID_value = 0;
      }
      else if (Tiempo_actual >= Tiempo0){
        PID_error = Setpoint - Temperatura;                   //Calculo del error    
        Error_INT = Error_INT + PID_error*(1000/Read_Delay);  //Calculo de la integral del error
        PID_value = Kc*(PID_error + (1/Tao_I)*Error_INT);     //Calculo de la salida del controlador PI

        sp = Setpoint;
        
      }
      // Limite de salida del controlador
      if(PID_value < 0)
      {      PID_value = 0;       }
      if(PID_value > 100)
      {      PID_value = 100;    }

      Potencia = PID_value;   //Asignacion a la entrada de la planta.
      }    
      
          Serial.print(Potencia);
          Serial.print(" ");
          Serial.print(Temperatura);
          Serial.print(" ");
          Serial.println(sp);    
      }
      
      } //Serial.println("sEGUNDO CORE");
      vTaskDelay(1); //NECESARIO SI LA PRIORIDAD DE LA TAREA NO ES 0//resulta que de ahuevo se ocupa
}
  
}
//

void setup() {
  Serial.begin(115200);//Inicializa la comunicacion
  inputString.reserve(200);//recerba hasta 200 caracteres en la memoria para los strings de entrada
  pinMode(LED_BUILTIN, OUTPUT);//inicializa el led
  
  pinMode (pin_disparo,OUTPUT); 
  pinMode (pin_cruce_cero,INPUT);
  attachInterrupt(pin_cruce_cero,InterrupcionCruceZero,RISING);

  /*
  */
  
}

void loop() {
  Tiempo_Actual_loop1=micros();
  if (stringComplete) {//entra si hay un string completo
    Serial.println(inputString);//Escribe en la consola el ultimo string
    if (inputString.equals("ON")){
      digitalWrite(LED_BUILTIN, HIGH);  
    }else if (inputString=="OFF"){
      digitalWrite(LED_BUILTIN, LOW);
    }else if (inputString=="TEMP"){
      Serial.println(String(Temperatura));
    }else if (inputString=="OLSTART"){
      //vTaskSuspend(Task1);
      xTaskCreatePinnedToCore(loop2,"Task_1",2000,NULL,1,/*intenta cambiar prioridad a 0, original 1*/&Task1,0);
    }else if (inputString=="CLSTART"){
      //vTaskSuspend(Task1);
      xTaskCreatePinnedToCore(loop2,"Task_1",2000,NULL,1,/*intenta cambiar prioridad a 0, original 1*/&Task1,0);

    }else if (inputString=="STOP"){
      vTaskSuspend(Task1);
    }
    inputString = "";// Borra el string:
    stringComplete = false;//Resetea la recepcion
  }
  if ((Tiempo_Actual_loop1-Tiempo_Previo_loop1)>=100&&contadorFiltro<=1000)
  {
    TempFiltro=TempFiltro+analogReadMilliVolts(SensorLM35);
    contadorFiltro++;
    //
  }if (contadorFiltro>1000)
  {
    
    TempFiltro=TempFiltro/1000;
    Temperatura=TempFiltro/10;
    contadorFiltro=0;
    Tiempo_Previo_loop1=Tiempo_Actual_loop1;
    //Serial.println(Temperatura);
  }
  
  



}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();// get the new byte:
    inputString += inChar;    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      inputString.remove(inputString.lastIndexOf("\n"));//Elimina el caracter salto de linea
      inputString.remove(inputString.lastIndexOf("\r"));//Elimina el caracter Carriage return
      stringComplete = true;
    }
  }
}
