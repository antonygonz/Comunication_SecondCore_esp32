#include <Arduino.h>

// put function declarations here:
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
/*
TaskHandle_t Task1;
void loop2(void *parameter){
  for(;;){

  }
  vTaskDelay(10);
}*/
//

void setup() {
  /*xTaskCreatePinnedToCore(
    loop2,
    "Task_1",
    1000,
    NULL,
    1,
    &Task1,
    0
  );
  */
  Serial.begin(9600);//Inicializa la comunicacion
  inputString.reserve(200);//recerba hasta 200 caracteres en la memoria para los strings de entrada
  pinMode(LED_BUILTIN, OUTPUT);//inicializa el led
}

void loop() {
  if (stringComplete) {//entra si hay un string completo
    if (inputString.equals("ON")){
      digitalWrite(LED_BUILTIN, HIGH);  
    }else if (inputString=="OFF"){
      digitalWrite(LED_BUILTIN, LOW);
    }
    Serial.println(inputString);//Escribe en la consola el ultimo string
    inputString = "";// Borra el string:
    stringComplete = false;//Resetea la recepcion
  }




}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      inputString.remove(inputString.lastIndexOf("\n"));//Elimina el caracter salto de linea
      inputString.remove(inputString.lastIndexOf("\r"));//Elimina el caracter Carriage return
      stringComplete = true;
      
    }
  }
}
