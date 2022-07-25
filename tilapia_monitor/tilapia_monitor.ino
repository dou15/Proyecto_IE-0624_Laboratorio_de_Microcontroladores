/***************************************************************************
            Importar bibliotecas
***************************************************************************/

#include <Arduino.h>
#include <DHT.h>

/***************************************************************************
            Sensor temperatura dht22 definiciones - variables
***************************************************************************/

#define DHTPIN A2         // Puerto conectar sensor DHT
#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE); // Instancia biblioteca DHT(PUERTO,TypoSensor)

float temperaturadht22;

/***************************************************************************
            Sensor pH SEN0161 definiciones - variables
***************************************************************************/

#define SensorPin A0            //la interfas del sensor de pH se conecta por el puerto A0
#define Offset 0.00            //margen de la medicion de pH
#define samplingInterval 20
#define printInterval 2000
#define ArrayLenth  40    //frecuencia de muestreo
int pHArray[ArrayLenth];   //se almacena el promedio de las mediciones de pH
int pHArrayIndex=0;

/***************************************************************************
            Sensor oxigeno SEN0237 definiciones - variables
***************************************************************************/

#define DO_PIN A1

#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //resolucion de medición ADC

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (temperaturadht22) //Temperatura del agua ℃

//punto de calibración CAL1_V y CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//puntos de calibracion CAL2_V and CAL2_T
//CAL1 calibra altas temperaturas, CAL2 calibra bajas temperaturas
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
uint16_t DissolvedOxygen;

/***************************************************************************
            Sensor oxigeno SEN0237 funciones
***************************************************************************/

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

/***************************************************************************
            Sensor pH SEN0161 funciones
***************************************************************************/

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //menos de 5 mediciones, se almacena directamente
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

/***************************************************************************
            SETUP
***************************************************************************/
void setup(void){
  Serial.begin(9600);
  dht.begin();               // Inicializa biblioteca DHT22
}

/***************************************************************************
            LOOP
***************************************************************************/
void loop(void){
  
  /*************************************************************************
            Sensor temperatura dht22 loop 
  *************************************************************************/  
  // Lectura Humedad DHT22
  // Espera mientras realiza la medición de humedad
  delay(2000);
  temperaturadht22 = dht.readTemperature();  
    
  /*************************************************************************
            Sensor pH SEN0237 loop 
  *************************************************************************/
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  DissolvedOxygen = (readDO(ADC_Voltage, Temperaturet))/1000;
    
  /*************************************************************************
            Sensor pH SEN0161 loop 
  *************************************************************************/  
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1023;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  // Cada 1000 milesegundos se imprime los valores registrados
  if(millis() - printTime > printInterval)
  {
    // Sensor temperatura
    Serial.println("Temperatura:\t" + String(temperaturadht22) + String(char(176)) + "C"); 
    Serial.println("Oxigeno:\t" + String(DissolvedOxygen) + "mg/L");
    // Sensor pH
    Serial.println("pH value:\t" + String(pHValue));
    Serial.println();
        printTime=millis();
  }
}
