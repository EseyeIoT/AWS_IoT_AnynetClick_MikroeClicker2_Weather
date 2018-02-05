// This project uses Mikro Electronica CLICKER2 and is designed for use with
// their Mikro Pro C compiler and libraries
// CLICK modules used :
// Weather Click utilising the BOSCH BME280
// Anynet 2G Click.  Cellular AWS gateway / client

#include "BME280_Driver.h"
#include "BME280_Defs.h"
#include "resources.h"

char tmp;
char text[200];
char shtext[50];
int count = 1;
float tmp_f, my_temperature, my_humidity, my_pressure;

/* init hardware */
static void MCU_Init(void)
{
  I2C2_Init(100000);
  delay_ms(200);
  UART2_Init(9600);                                                       // Initialize I2C on pins RA2 and RA3
  delay_ms(200);
}

/* init bme */
void BME280_INIT(void) {
  BME280_SetStandbyTime(BME280_STANDBY_TIME_1_MS);                          // Standby time 1ms
  BME280_SetFilterCoefficient(BME280_FILTER_COEFF_16);                      // IIR Filter coefficient 16
  BME280_SetOversamplingPressure(BME280_OVERSAMP_16X);                      // Pressure x16 oversampling
  BME280_SetOversamplingTemperature(BME280_OVERSAMP_2X);                    // Temperature x2 oversampling
  BME280_SetOversamplingHumidity(BME280_OVERSAMP_1X);                       // Humidity x1 oversampling
  BME280_SetOversamplingMode(BME280_NORMAL_MODE);
}

/* setup and calibrate weather click */
void BME280_Setup(void) {
  BME280_ReadCalibrationParams();                                           //Read calibration parameters
  BME280_SetOversamplingPressure(BME280_OVERSAMP_1X);
  BME280_SetOversamplingTemperature(BME280_OVERSAMP_1X);
  BME280_SetOversamplingHumidity(BME280_OVERSAMP_1X);
  BME280_SetOversamplingMode(BME280_FORCED_MODE);
  while(BME280_IsMeasuring());
  BME280_ReadMeasurements();
  BME280_INIT();
}

/* read response from AWS */
void waitForAwsResponse(char *response) {
  while(1) {
    char output[255];
    if (UART2_Data_Ready() == 1) {          // if data is received
      UART2_Read_Text(output, response, 255);    // reads text until response string is found
      break;
    }
  }
}

/* publish weather data to AWS */
void publishAwsWeatherData(void) {
  int len;
  /* read weather */
  while(BME280_IsMeasuring());
  BME280_ReadMeasurements();
  my_temperature = BME280_GetTemperature();
  my_humidity = BME280_GetHumidity();
  my_pressure = BME280_GetPressure();

  /* create json message */
  len = sprintf((char*)text, "{\"EventNo.\":\n{\"count\": %d,\"temperature\": %.2f,\"humidity\": %.2f, \"pressure\": %.2f}}",
                                        count, my_temperature, my_humidity, my_pressure);
  count += 1;

  /* send to aws */
  sprintf(shtext, "AT+AWSPUBLISH=0,%d\r\n", len);
  UART2_Write_Text(shtext);
  waitForAwsResponse(">");
  UART2_Write_Text(text);
}

/* setup connection to aws */
void setupAws(void) {
  /* echo off simplified the response handling */
  UART2_Write_Text("ATE0\r\n");
  waitForAwsResponse("OK");

  // OPEN an SSL encryped AWS IoT connection to publish the data
  sprintf(shtext, "AT+AWSPUBOPEN=0,\"weather\"\r\n");
  UART2_Write_Text(shtext);
  waitForAwsResponse("OK");
}

void main(void) {
  PLLEN_bit = 1;            // Enable PLL
  Delay_ms(150);

  ADSHR_bit = 1;            // Alternate SFR is selected
  ANCON0 = 0xFF;            // All pins to digital
  ANCON1 = 0xFF;
  TRISD4_bit=0;             // LED for feedback

  /* init hardware */
  MCU_Init();
  LATD4_bit=1;
  delay_ms(1000);
  // Check we can see the BME280
  tmp = BME280_GetID();
  delay_ms(1000);
  LATD4_bit=0;
  delay_ms(1000);
  if (tmp != BME280_CHIP_ID) {
    while (1);                                                              //stop program
  } else {
    // LED on once we have the BME under our control
    LATD4_bit=1;
    delay_ms(1000);
  }

  BME280_Setup();

  LATD4_bit=0;
  delay_ms(1000);

  // Initialise Anynet CLICK module for AWS
  setupAws();

  LATD4_bit=1;
  delay_ms(2000);
  while(1) {
     // Every 15 seconds update weather data and publish
     publishAwsWeatherData();
     BME280_INIT();
     delay_ms(15000);
  }
}