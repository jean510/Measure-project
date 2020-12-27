#include <Arduino.h>
#include <Stream.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <PCD8544.h>

/*******************************************************************************/
// Macros **********************************************************************/
/*******************************************************************************/
#define RESISTENCIA_BASE 100

// Tasks id
#define READ_DC_VOLTAGE  0
#define READ_AC_VOLTAGE  1
#define READ_RESISTANCE  2
#define READ_INDUCTANCE  3
#define READ_CAPACITANCE 4
#define READ_BETHA       5

// Pines
#define PIN_DC_VOLTAGE  A0
#define PIN_AC_VOLTAGE  A1
#define PIN_RESISTANCE  A2
#define PIN_INDUCTANCE  A3

#define PIN_CAPACITANCE A4
#define PIN_CHARGER     13
#define PIN_DISCHARGER  12  

#define PIN_BETHA       A5
#define PIN_BATTERY_V   A6

/*******************************************************************************/
// Declaración de structs ******************************************************/
/*******************************************************************************/
typedef struct{
    uint8_t line0;
    uint8_t line1;
    uint8_t line2;
    uint8_t line3;
    uint8_t line4;
    uint8_t line5;
    String  str_line0;
    String  str_line1;
    String  str_line2;
    String  str_line3;
    String  str_line4;
    String  str_line5;
}lcdLines;

/*******************************************************************************/
/* Declaranción de tareas ******************************************************/
/*******************************************************************************/
void TaskReadACVoltage   ( void *pvParameters );
void TaskReadDCVoltage   ( void *pvParameters );
void TaskReadResistance  ( void *pvParameters );
void TaskReadInductance  ( void *pvParameters );

void TaskReadCapacitance ( void *pvParameters );

void TaskReadBetha       ( void *pvParameters );
void TaskLcdPrint        ( void *pvParameters );
void TaskLowPower        ( void *pvParameters );
void TaskSleep           ( void *pvParameters );

void buttonPressed();

/*******************************************************************************/
// Variables globales **********************************************************/
/*******************************************************************************/
TaskHandle_t      g_TaskReadDCVoltage_Handler;
TaskHandle_t      g_TaskReadACVoltage_Handler;
TaskHandle_t      g_TaskReadResistance_Handler;
TaskHandle_t      g_TaskReadInductance_Handler;

TaskHandle_t      g_TaskReadCapacitance_Handler;

TaskHandle_t      g_TaskReadBetha_Handler;
TaskHandle_t      g_TaskLowPower_Handler;
TaskHandle_t      g_TaskLcdPrint_Handler;
TaskHandle_t      g_TaskSleep_Handler;

SemaphoreHandle_t g_ADConverter_Semaphore;
QueueHandle_t     g_lcdPrint_Queue;
uint8_t           g_selected_meter;
uint8_t           g_sleepCounter;
uint8_t           f_suspended;
static PCD8544    g_lcd;

/*******************************************************************************/
// Setup ***********************************************************************/
/*******************************************************************************/
void setup()
{
    pinMode(2, INPUT_PULLUP);

    // Inicialización del puerto serial
    Serial.begin(9600);

    // Interrupción pin 2
    attachInterrupt(digitalPinToInterrupt(2), buttonPressed, FALLING);

    // LCD 
    g_lcd.begin(84, 48);
    pinMode(8, OUTPUT);

    g_selected_meter = 0;
    f_suspended      = 0;

    // Queue
    g_lcdPrint_Queue = xQueueCreate(8, sizeof(lcdLines));

    // Semaphore
    if ( g_ADConverter_Semaphore == NULL )
    {
        g_ADConverter_Semaphore = xSemaphoreCreateMutex();
        xSemaphoreGive( g_ADConverter_Semaphore );
    }

    // Instanciación de tareas
    xTaskCreate(TaskReadDCVoltage  ,"TaskReadDCVoltage"   , 512, NULL, 1, &g_TaskReadDCVoltage_Handler);
    xTaskCreate(TaskReadACVoltage  ,"TaskReadACVoltage"   , 512, NULL, 1, &g_TaskReadACVoltage_Handler);
    xTaskCreate(TaskReadResistance ,"TaskReadResistance"  , 512, NULL, 1, &g_TaskReadResistance_Handler);
    xTaskCreate(TaskReadInductance ,"TaskReadInductance"  , 512, NULL, 1, &g_TaskReadInductance_Handler);   
    
    xTaskCreate(TaskReadCapacitance,"TaskReadCapacitance" , 512, NULL, 1, &g_TaskReadCapacitance_Handler);   
    
    xTaskCreate(TaskReadBetha      ,"TaskReadBetha"       , 512, NULL, 1, &g_TaskReadBetha_Handler);
    xTaskCreate(TaskLowPower       ,"TaskLowPower"        , 512, NULL, 0, &g_TaskLowPower_Handler);
    xTaskCreate(TaskLcdPrint       ,"TaskLcdPrint"        , 512, NULL, 0, &g_TaskLcdPrint_Handler);
    xTaskCreate(TaskSleep          ,"TaskSleep"           , 512, NULL, 0, &g_TaskSleep_Handler);

    vTaskSuspend(g_TaskReadACVoltage_Handler);
    vTaskSuspend(g_TaskReadResistance_Handler);
    vTaskSuspend(g_TaskReadInductance_Handler);
    vTaskSuspend(g_TaskReadCapacitance_Handler);
    vTaskSuspend(g_TaskReadBetha_Handler);

    // Iniciacion del planificador
    vTaskStartScheduler();

}


/*******************************************************************************/
// Loop ************************************************************************/
/*******************************************************************************/
void loop()
{

}

/*******************************************************************************/
// Implementación de funciones *************************************************/
/*******************************************************************************/

void buttonPressed()
{
    g_sleepCounter = 0;

    if(f_suspended)
    {
        vTaskResume(g_TaskLcdPrint_Handler);
        vTaskResume(g_TaskLowPower_Handler);
        vTaskResume(g_TaskSleep_Handler);
        f_suspended = 0;
    }
    else{
        if(g_selected_meter == READ_CAPACITANCE){
            g_selected_meter = READ_DC_VOLTAGE;
        }else{
            g_selected_meter++;
        }
    }

    vTaskSuspend(g_TaskReadDCVoltage_Handler);
    vTaskSuspend(g_TaskReadACVoltage_Handler);
    vTaskSuspend(g_TaskReadResistance_Handler);
    vTaskSuspend(g_TaskReadInductance_Handler);
    vTaskSuspend(g_TaskReadCapacitance_Handler);
    vTaskSuspend(g_TaskReadBetha_Handler);

    switch (g_selected_meter)
    {
    case READ_DC_VOLTAGE:
        vTaskResume(g_TaskReadDCVoltage_Handler);
        break;
    case READ_AC_VOLTAGE:
        vTaskResume(g_TaskReadACVoltage_Handler);
        break;
    case READ_RESISTANCE:
        vTaskResume(g_TaskReadResistance_Handler);
        break;
    case READ_INDUCTANCE:
        vTaskResume(g_TaskReadInductance_Handler);
        break;
    case READ_CAPACITANCE:
        vTaskResume(g_TaskReadCapacitance_Handler);
        break;
    case READ_BETHA:
        vTaskResume(g_TaskReadBetha_Handler);
        break;
    default:
        Serial.println("default");
        break;
    }
}

/*******************************************************************************/
// Implementación de tareas ****************************************************/
/*******************************************************************************/

void TaskReadDCVoltage( void *pvParameters )
{  
    pinMode(PIN_DC_VOLTAGE, INPUT_PULLUP);
    int    t_sensorValue   = 0;
    float  t_sensorVoltage = 0;
    float  t_vdcValue      = 0;
    lcdLines t_lcdLines    = {};
    t_lcdLines.line1 = 1;
    t_lcdLines.line3 = 1;

    while(1)
    { 
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {   
            // Lectura del pin analógico
            t_sensorValue   = analogRead(PIN_DC_VOLTAGE);
            vTaskDelay(1);
            xSemaphoreGive( g_ADConverter_Semaphore );

            // Calculo de la tension
            t_sensorVoltage = t_sensorValue * (5.0/1023.0);
            t_vdcValue      = (t_sensorVoltage - 2.35)/(0.341463*0.471264);

            t_lcdLines.str_line1 = "Tension DC";
            t_lcdLines.str_line3 = String(t_vdcValue);
            xQueueSend(g_lcdPrint_Queue, &t_lcdLines, 100);
        }
        vTaskDelay(132);
    }
}


void TaskReadACVoltage( void *pvParameters )
{  
    pinMode(PIN_AC_VOLTAGE, INPUT_PULLUP);
    int    t_sensorValue   = 0;
    float  t_sensorVoltage = 0;
    float  t_vacValue      = 0;
    lcdLines t_lcdLines = {};
    t_lcdLines.line1 = 1;
    t_lcdLines.line3 = 1;

    while(1)
    { 
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            t_sensorValue  = analogRead(PIN_AC_VOLTAGE);
            vTaskDelay(1);
            xSemaphoreGive( g_ADConverter_Semaphore );

            t_sensorVoltage = t_sensorValue * (5.0/1023.0);
            t_vacValue      = (t_sensorVoltage/0.34146) + 1.6;
            
            t_lcdLines.str_line1 = "Tension AC";
            t_lcdLines.str_line3 = String(t_vacValue);
            xQueueSend(g_lcdPrint_Queue, &t_lcdLines, 100);
        }
        vTaskDelay(132);
    }
}

void TaskReadResistance( void *pvParameters )
{  
    pinMode(PIN_RESISTANCE, INPUT_PULLUP);
    int   t_sensorValue;
    float t_sensorVoltage;
    float t_resistanceValue;
    lcdLines t_lcdLines = {};
    t_lcdLines.line1 = 1;
    t_lcdLines.line3 = 1;

    while(1)
    { 
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            t_sensorValue     = analogRead(PIN_RESISTANCE);
            vTaskDelay(1);
            xSemaphoreGive( g_ADConverter_Semaphore );
            t_sensorVoltage   = t_sensorValue * (5.0/1023.0);
            t_resistanceValue = (50000/t_sensorVoltage) - 10000;
            t_lcdLines.str_line1 = "Resistencia";
            t_lcdLines.str_line3 = String(t_resistanceValue);
            xQueueSend(g_lcdPrint_Queue, &t_lcdLines, 100);
        }
        vTaskDelay(132);
    }
}

void TaskReadInductance( void *pvParameters )
{  
    pinMode(PIN_INDUCTANCE, INPUT_PULLUP);
    lcdLines l_lcdLines = {};
    l_lcdLines.line1 = 1;
    l_lcdLines.line3 = 1;

    while(1)
    { 
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            int sensorValue  = analogRead(PIN_INDUCTANCE);
            vTaskDelay(1);
            xSemaphoreGive( g_ADConverter_Semaphore );
            l_lcdLines.str_line1 = "Inductancia";
            l_lcdLines.str_line3 = String(sensorValue);
            xQueueSend(g_lcdPrint_Queue, &l_lcdLines, 100);
        }
        vTaskDelay(132);
    }
}

void TaskReadCapacitance( void *pvParameters )
{  
    pinMode(PIN_CAPACITANCE, INPUT_PULLUP);
    unsigned long t_startTime;
    unsigned long t_elapsedTime;
    int           t_sensorValue;
    float         t_capacitanceValue;
    lcdLines      t_lcdLines = {};
    t_lcdLines.line1 = 1;
    t_lcdLines.line3 = 1;

    while(1)
    { 
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            xSemaphoreGive( g_ADConverter_Semaphore );
            t_lcdLines.str_line1 = "Capacitancia";
            t_lcdLines.str_line3 = String(t_capacitanceValue);
            xQueueSend(g_lcdPrint_Queue, &t_lcdLines, 100);
            /*
            taskENTER_CRITICAL();
            digitalWrite(PIN_CHARGER, HIGH);  
            t_startTime   = millis();
            t_sensorValue = analogRead(PIN_CAPACITANCE);
            while(t_sensorValue < 400){
                t_sensorValue = analogRead(PIN_CAPACITANCE);
                Serial.println(t_sensorValue);
            }
            t_elapsedTime = millis() - t_startTime;
            taskEXIT_CRITICAL();

            vTaskDelay(1);
            xSemaphoreGive( g_ADConverter_Semaphore );
            
            // Se inicia la descarga del capacitor
            digitalWrite(PIN_CHARGER, LOW);            
            pinMode(PIN_DISCHARGER, OUTPUT);            
            digitalWrite(PIN_DISCHARGER, LOW); 
            
            // Se calcula la capacitancia utilizando la constante de tiempo
            t_capacitanceValue = ((float) t_elapsedTime / 10000) * 1000;  

            // Se ponen los resultados en la cola del display
            t_lcdLines.str_line1 = "Capacitancia";
            t_lcdLines.str_line3 = String(t_capacitanceValue);
            xQueueSend(g_lcdPrint_Queue, &t_lcdLines, 100);
         
            // Se comprueba que el capacitor se descargue
            while(analogRead(PIN_CAPACITANCE) > 0){}
            pinMode(PIN_DISCHARGER, INPUT); 
           */
        }
        vTaskDelay(132);
    }
}

void TaskReadBetha( void *pvParameters )
{  
    pinMode(PIN_BETHA, INPUT_PULLUP);
    int   t_sensorValue;
    float t_bethaValue;
    lcdLines l_lcdLines = {};
    l_lcdLines.line1 = 1;
    l_lcdLines.line3 = 1;

    while(1)
    {
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            t_sensorValue    = analogRead(PIN_BETHA);
            t_bethaValue     = 1.0 + ( ( 1023.0*330.0*(2.4 - 0.8) ) / (5.0 * t_sensorValue) );
            vTaskDelay(1);
            xSemaphoreGive( g_ADConverter_Semaphore );
            l_lcdLines.str_line1 = "Betha";
            l_lcdLines.str_line3 = String(t_bethaValue);
            xQueueSend(g_lcdPrint_Queue, &l_lcdLines, 100);
        }
        vTaskDelay(132);
    }
}


void TaskLowPower( void *pvParameters )
{
    pinMode(PIN_BATTERY_V, INPUT_PULLUP);
    int      t_sensorValue;
    float    t_sensorVoltage;
    lcdLines t_lcdLines = {};
    t_lcdLines.line5 = 1;

    while (1)
    {
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            t_sensorValue = analogRead(PIN_BATTERY_V);
            xSemaphoreGive( g_ADConverter_Semaphore );
            t_sensorVoltage = t_sensorValue * (5.0/1023);
            if(t_sensorVoltage > 7)
            {   
                t_lcdLines.str_line5 = "";      
            }
            else
            {
                t_lcdLines.str_line5 = "Low Power";  
            }
            xQueueSend(g_lcdPrint_Queue, &t_lcdLines, 100);
        }
        vTaskDelay(132);
    }    
}

void TaskLcdPrint( void *pvParameters )
{
    lcdLines l_lcdLines;
    while(1)
    {
        if(!xQueueReceive(g_lcdPrint_Queue, &l_lcdLines, 132)) {
            puts("Failed to receive item within 1000 ms");
        }
        else {
            if(l_lcdLines.line0 == 1){
                g_lcd.setCursor(0,0);
                g_lcd.clearLine();
                g_lcd.print(l_lcdLines.str_line0);
            }
            if(l_lcdLines.line1 == 1){
                g_lcd.setCursor(0,1);
                g_lcd.clearLine();
                g_lcd.print(l_lcdLines.str_line1);
            }
            if(l_lcdLines.line2 == 1){
                g_lcd.setCursor(0,2);
                g_lcd.clearLine();
                g_lcd.print(l_lcdLines.str_line2);
            }
            if(l_lcdLines.line3 == 1){
                g_lcd.setCursor(0,3);
                g_lcd.clearLine();
                g_lcd.print(l_lcdLines.str_line3);
            }
            if(l_lcdLines.line4 == 1){
                g_lcd.setCursor(0,4);
                g_lcd.clearLine();
                g_lcd.print(l_lcdLines.str_line4);
            }
            if(l_lcdLines.line5 == 1){
                g_lcd.setCursor(0,5);
                g_lcd.clearLine();
                g_lcd.print(l_lcdLines.str_line5);
            }
        }
    }
}

void TaskSleep    ( void *pvParameters )
{
    while (1)
    {
        vTaskDelay(66);
        if(g_sleepCounter >= 4)
        {
            g_lcd.clear();
            f_suspended = 1;
            vTaskSuspend(g_TaskReadDCVoltage_Handler);
            vTaskSuspend(g_TaskReadACVoltage_Handler);
            vTaskSuspend(g_TaskReadResistance_Handler);
            vTaskSuspend(g_TaskReadInductance_Handler);
            vTaskSuspend(g_TaskReadCapacitance_Handler);
            vTaskSuspend(g_TaskReadBetha_Handler);
            vTaskSuspend(g_TaskLcdPrint_Handler);
            vTaskSuspend(g_TaskLowPower_Handler);
            vTaskSuspend(g_TaskSleep_Handler);
        }else{
            g_sleepCounter++;
        }
    }
}

/*
double pulso, frecuencia, capacidad, inductancia;
void setup(){
    Serial.begin(9600);
    pinMode(9, INPUT);
    pinMode(13, OUTPUT);
    Serial.println("Medidor de Inductancias!!!");
    delay(200);
}

void loop(){
    digitalWrite(13, HIGH);
    delay(5);                                                                  // Dar algun timepo para cargar la bobina.
    digitalWrite(13,LOW);
    delayMicroseconds(100);                                                    // verificar que mide la resonancia.
    pulso = pulseIn(9,HIGH,5000);                                             // returna 0 si esta fuera de tiempo.
    if(pulso > 0.1){                                                           // Si no se activo el fuera de tiempo ... tomo una lectura:
        capacidad = 2.E-6;                                                       // Insertar la capacidad de los condensadores aqui. Actualmente se usa 2 uF.
        frecuencia = 1.E6/(2*pulso);
        inductancia = 1./(capacidad*frecuencia*frecuencia*4.*3.14159*3.14159);
        inductancia *= 1E6;
        Serial.print("Pulso en uS:");
        Serial.print( pulso );
        Serial.print("\tfrecuencia Hz:");
        Serial.print( frecuencia );
        Serial.print("\tinductancia uH:");
        Serial.println( inductancia );
    delay(20);
    }                                                        // Tenga en cuenta que esto es lo mismo que decir  inductance = inductance*1E6
}*/

/*
#define analogPin      0          
#define chargePin      13         
#define dischargePin   11        
#define resistorValue  10000.0F   

unsigned long startTime;
unsigned long elapsedTime;
float microFarads;                
float nanoFarads;

void setup(){
  pinMode(chargePin, OUTPUT);     
  digitalWrite(chargePin, LOW);  
  Serial.begin(9600);             
}

void loop(){
    digitalWrite(chargePin, HIGH);  
  
    // Se carga el capacitor y se mide la constate de tiempo
    startTime = millis(); 
    while(analogRead(analogPin) < 648){}
    elapsedTime= millis() - startTime;

    // Se calcula la capacitancia utilizando la constante de tiempo
    microFarads = ((float)elapsedTime / resistorValue) * 1000;   
    Serial.print(elapsedTime);       
    Serial.print(" mS    ");         

    // Se imprime la capacitancia
    if (microFarads > 1){
        Serial.print((long)microFarads);       
        Serial.println(" microFarads");         
    }
    else{
        nanoFarads = microFarads * 1000.0;      
        Serial.print((long)nanoFarads);         
        Serial.println(" nanoFarads");          
        delay(500); 
    }

    // Se descarga el capacitor
    digitalWrite(chargePin, LOW);            
    pinMode(dischargePin, OUTPUT);            
    digitalWrite(dischargePin, LOW);          
    while(analogRead(analogPin) > 0){}
    pinMode(dischargePin, INPUT);            
} 
*/

/**
double pulso, frecuencia, capacidad, inductancia;
void setup(){
    Serial.begin(9600);
    pinMode(11, INPUT);
    pinMode(13, OUTPUT);
    Serial.println("Medidor de Inductancias!!!");
    delay(200);
}

void loop(){
    digitalWrite(13, HIGH);
    delay(5);                                                                  // Dar algun timepo para cargar la bobina.
    digitalWrite(13,LOW);
    delayMicroseconds(100);                                                    // verificar que mide la resonancia.
    pulso = pulseIn(11,HIGH,5000);                                             // returna 0 si esta fuera de tiempo.
    if(pulso > 0.1){                                                           // Si no se activo el fuera de tiempo ... tomo una lectura:
        capacidad = 2.E-6;                                                       // Insertar la capacidad de los condensadores aqui. Actualmente se usa 2 uF.
        frecuencia = 1.E6/(2*pulso);
        inductancia = 1./(capacidad*frecuencia*frecuencia*4.*3.14159*3.14159);
        inductancia *= 1E6;                                                        // Tenga en cuenta que esto es lo mismo que decir  inductance = inductance*1E6
        Serial.print("Pulso en uS:");
        Serial.print( pulso );
        Serial.print("\tfrecuencia Hz:");
        Serial.print( frecuencia );
        Serial.print("\tinductancia uH:");
        Serial.println( inductancia );
        delay(20);
    }
}
*/
/*

#define R_meter      3          

int lecture = 0;
int Ve = 5; // Tensión en el Arduino.
float VR2 = 0;
float R1 = 10000;
float R2 = 0;
float I = 0;
float relacion = 0;

void setup(){  
  Serial.begin(9600); 
  pinMode(R_meter, INPUT);            
}

void loop(){
   lecture = analogRead(R_meter);
if(lecture) 
{
relacion = lecture * Ve;
VR2 = (relacion)/1024.0;
relacion = (Ve/VR2) -1;
R2= R1 * relacion;

Serial.print("R2: ");
Serial.println(R2);
delay(1000);            
} }
*/

/*
void TaskReadA0( void *pvParameters )
{
    pinMode(A0, INPUT_PULLUP);
    lcdLines l_lcdLines = {};
    l_lcdLines.line1 = 1;
    l_lcdLines.line3 = 1;
    while(1)
    {
        if ( xSemaphoreTake(g_ADConverter_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
            int sensorValue  = analogRead(A0);
            xSemaphoreGive( g_ADConverter_Semaphore );
            l_lcdLines.str_line1 = "Amperimetro";
            l_lcdLines.str_line3 = String(sensorValue);
            xQueueSend(g_lcdPrint_Queue, &l_lcdLines, 100);
        }
        vTaskDelay(100);
    }
}
*/