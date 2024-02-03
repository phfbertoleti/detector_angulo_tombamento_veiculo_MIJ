/* Exemplo: uso do algoritmo Mahony AHRS para determinar, via leitura do acelerômetro
            e giroscópio, os ângulo de tombamento (roll). Ainda, envia periodicamente,
            via LoRa, as leitura obtida.
   Autor: Pedro Bertoleti
*/

/* Includes */
#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Wire.h>
#include <math.h>

/* Defines - serial de debug */
#define SERIAL_DEBUG_BAUDRATE      115200

/* Defines - LoRa */
#define SCK                      18
#define MISO                     19
#define MOSI                     23
#define SS                       5
#define RST                      14
#define DI00                     26
#define BAND_END_DEVICE          918E6    // Band01 = 919MHz
#define PA_SET                   true     // true = RFO / false = PABOOST
#define SF                       9        // Spread factor
#define PA_Value                 15
#define BW                       125E6    //Largura de banda
#define CR                       5
#define TEMPO_ENTRE_ENVIOS_LORA  2000   //ms

/* Definições - LED RGB */
#define NUMPIXELS             1
#define RED                   pixels1.Color(255, 0, 0)
#define GREEN                 pixels1.Color(0, 255, 0)
#define BLUE                  pixels1.Color(0, 0, 255)
#define WHITE                 pixels1.Color(255, 255, 255)
#define OFF_COLORS            pixels1.Color(0, 0, 0)
#define MAXCOLORS             5
#define GPIO_LED_RGB          2
#define TEMPO_ENTRE_ALTERNANCIA_CORES    500 //ms

/* Definições - IMU */
#define VALOR_GRAVIDADE              9.80665  // m/s²
#define NUM_LEITURAS_CALIBRACAO      200
#define GPIO_INTERRUPT_BMI270        27
#define TEMPO_ENTRE_LEITURAS_IMU     10   //ms
#define TEMPO_ENTRE_MOSTRAR_ANGULOS  1000 //ms
#define sampleFreq                   100  //frequencia de amostragem (em Hz)
#define twoKpDef                     (256.0f * 1.0f)
#define twoKiDef                     (32.0f * 1.0f)

/* Estrutura de envio de dados LoRa */
typedef struct __attribute__((__packed__))  
{
  char byte_1_sincronia;
  char byte_2_sincronia;
  char byte_3_sincronia;
  float roll;  
  char checksum;
}TDadosLora;

/* Variáveis e objetos - LED RGB */
Adafruit_NeoPixel pixels1(NUMPIXELS, GPIO_LED_RGB, NEO_GRB + NEO_KHZ800);
uint32_t cores[3] = {BLUE, RED, GREEN};
int contador_cores = 0;
unsigned long timestamp_led_rgb = 0;

/* Variáveis - LoRa */
unsigned long timestamp_envio_LoRa = 0;

/* Variáveis e objetos - IMU */
unsigned long timestamp_imu = 0;
unsigned long timestamp_mostra_angulos = 0;
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;  //0x68
bool imu_interrupt = false;
float offset_ax = 0.0;
float offset_ay = 0;
float offset_az = 0;
float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;

/* Variáveis do filtro Mahony */
volatile float twoKp = twoKpDef; //2 * ganho proporcional (Kp)
volatile float twoKi = twoKiDef; //2 * ganho integral (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; //erros da parte integral

/* Protótipos */
unsigned long diferenca_tempo(unsigned long tref);
void le_acc_gyro(float * pt_acc_x, float * pt_acc_y, float * pt_acc_z, float * pt_gyro_x, float * pt_gyro_y, float * pt_gyro_z);
void calibra_acc_giroscopio(void);
float invSqrt(float x);
void Mahony_AHS_update(float gx, float gy, float gz, float ax, float ay, float az);
void obtem_angulos_de_euler(float * ptr_roll, float * ptr_pitch, float * ptr_yaw);

/* Handler para quando ocorrer interrupção do acelerômetro */
void myInterruptHandler()
{
  imu_interrupt = true;
}

/* Função: calcula diferença de tempo entre instante atual e referência de tempo
   Parametros: referência de tempo
   Retorno: diferença calculada
*/
unsigned long diferenca_tempo(unsigned long tref)
{
  return (millis() - tref);
}

/* Função: faz a leitura do acelerômetro e giroscópio
   Parametros: ponteiro para leituras do acelerômetro e giroscópio
   Retorno: nenhum
*/
void le_acc_gyro(float * pt_acc_x, float * pt_acc_y, float * pt_acc_z, float * pt_gyro_x, float * pt_gyro_y, float * pt_gyro_z)
{
  imu.getSensorData();

  *pt_acc_x = (imu.data.accelX - offset_ax) * VALOR_GRAVIDADE;
  *pt_acc_y = (imu.data.accelY - offset_ay) * VALOR_GRAVIDADE;
  *pt_acc_z = (imu.data.accelZ - offset_az) * VALOR_GRAVIDADE;
  *pt_gyro_x = (imu.data.gyroX - offset_gx) * VALOR_GRAVIDADE;
  *pt_gyro_y = (imu.data.gyroY - offset_gy) * VALOR_GRAVIDADE;
  *pt_gyro_z = (imu.data.gyroZ - offset_gz) * VALOR_GRAVIDADE;
}

/* Função: faz a calibração do acelerômetro e giroscópio
   Parametros: nenhum
   Retorno: nenhum
*/
void calibra_acc_giroscopio(void)
{
  int i;
  float accx, accy, accz, gyrox, gyroy, gyroz;
  float soma_ax = 0;
  float soma_ay = 0;
  float soma_az = 0;
  float soma_gx = 0;
  float soma_gy = 0;
  float soma_gz = 0;

  for (i = 0; i < NUM_LEITURAS_CALIBRACAO; i++)
  {
    le_acc_gyro(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);

    soma_ax = soma_ax + accx;
    soma_ay = soma_ay + accy;
    soma_az = soma_az + accz;

    soma_gx = soma_gx + gyrox;
    soma_gy = soma_gy + gyroy;
    soma_gz = soma_gz + gyroz;
  }

  offset_ax = soma_ax / NUM_LEITURAS_CALIBRACAO;
  offset_ay = soma_ay / NUM_LEITURAS_CALIBRACAO;
  offset_az = VALOR_GRAVIDADE - (soma_az / NUM_LEITURAS_CALIBRACAO);

  offset_gx = soma_gx / NUM_LEITURAS_CALIBRACAO;
  offset_gy = soma_gy / NUM_LEITURAS_CALIBRACAO;
  offset_gz = soma_gz / NUM_LEITURAS_CALIBRACAO;
}

/* Função: fast inverse square-root */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/* Função: filtro Mahony
   Parâmetros: - giroscópio nos 3 eixos (gx, gy e gz)
               - acelerômetro nos 3 eixos (ax, ay e az)
   Retorno: nenhum
*/
void Mahony_AHS_update(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  gx *= (0.5f * (1.0f / sampleFreq));
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/* Função: obtem angulos de euler a partir dos quaternions
   Parâmetros: - Ponteiro para roll
               - Ponteiro para pitch
               - Ponteiro para yaw
   Retorno: nenhum
*/
void obtem_angulos_de_euler(float * ptr_roll, float * ptr_pitch, float * ptr_yaw)
{
  *ptr_roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * (180.0 / M_PI);
  *ptr_pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * (180.0 / M_PI);
  *ptr_yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * (180.0 / M_PI);
}

void setup()
{
  /* Configura serial de debug */
  Serial.begin(SERIAL_DEBUG_BAUDRATE);
  Serial.println("Exemplo - angulos roll, pitch e yaw com Mahony AHRS");

  /* Configura LED RGB */
  Serial.println("Setup: LED RGB");
  contador_cores = 0;
  pixels1.begin();
  pixels1.clear();
  pixels1.setPixelColor(0, BLUE);
  pixels1.show();

  /* Configura acelerômetro */
  Serial.println("Setup: IMU");
  imu_interrupt = false;
  Wire.begin();

  if (imu.beginI2C(i2cAddress) != BMI2_OK)
  {
    bmi2_int_pin_config intPinConfig;

    intPinConfig.pin_type = BMI2_INT1;
    intPinConfig.int_latch = BMI2_INT_NON_LATCH;
    intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    imu.setInterruptPinConfig(intPinConfig);
    attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_BMI270), myInterruptHandler, RISING);

    Serial.println("IMU em calibracao...");
    calibra_acc_giroscopio();
    Serial.println("IMU calibrada");
  }

  /* Configura rádio LoRa */
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI00);                                    // Setup pins LoRa

  if (!LoRa.begin(BAND_END_DEVICE))
  {
    LoRa.setSignalBandwidth(BW);
    LoRa.setSpreadingFactor(SF);
    LoRa.setTxPower(PA_Value, PA_SET);
    LoRa.setFrequency(BAND_END_DEVICE);
    LoRa.setCodingRate4(CR);
    LoRa.enableCrc();
  }

  /* Inicializa temporizações */
  timestamp_led_rgb = millis();
  timestamp_imu = millis();
  timestamp_mostra_angulos = millis();
  timestamp_envio_LoRa = millis();
}

void loop()
{
  char msg_acc_gyro[150];
  TDadosLora dados_lora;
  float ax, ay, az, gx, gy, gz;
  float roll_atual, pitch_atual, yaw_atual;
  int i;
  char * pt_dados_lora;

  /* Muda periodicamente a cor do LED RGB */
  if (diferenca_tempo(timestamp_led_rgb) >= TEMPO_ENTRE_ALTERNANCIA_CORES)
  {
    contador_cores++;

    if (contador_cores > 2)
      contador_cores = 0;

    pixels1.setPixelColor(0, cores[contador_cores]);
    pixels1.show();
    timestamp_led_rgb = millis();
  }

  /* Le periodicamente o IMU. Em seguida, calcula os ângulos de Euler (roll, pitch e yaw) */
  if (diferenca_tempo(timestamp_imu) >= TEMPO_ENTRE_LEITURAS_IMU)
  {
    /* Le IMU */
    memset(msg_acc_gyro, 0x00, sizeof(msg_acc_gyro));
    le_acc_gyro(&ax, &ay, &az, &gx, &gy, &gz);

    /* Calcula ângulos de Euler */
    /* Converte leituras do giroscópio para rad/s */
    gx = gx * (M_PI / 180.0);
    gy = gy * (M_PI / 180.0);
    gz = gz * (M_PI / 180.0);
    Mahony_AHS_update(gx, gy, gz, ax, ay, az);
    obtem_angulos_de_euler(&roll_atual, &pitch_atual, &yaw_atual);
    timestamp_imu = millis();
  }

  /* Mostra periodicamente os ângulos de Euler (roll, pitch e yaw) */
  if (diferenca_tempo(timestamp_mostra_angulos) >= TEMPO_ENTRE_MOSTRAR_ANGULOS)
  {
    memset(msg_acc_gyro, 0x00, sizeof(msg_acc_gyro));
    snprintf(msg_acc_gyro, sizeof(msg_acc_gyro), "Roll: %.1f graus", roll_atual);
    Serial.println(msg_acc_gyro);
    timestamp_mostra_angulos = millis();
  }

  /* Envia periodicamente, via rádio LoRa, uma string com os ângulos de Euler */
  if (diferenca_tempo(timestamp_envio_LoRa) >= TEMPO_ENTRE_ENVIOS_LORA)
  {
    /* Popula estrutura de dados */
    dados_lora.checksum = 0x00;
    dados_lora.byte_1_sincronia = 'A';
    dados_lora.byte_1_sincronia = 'N';
    dados_lora.byte_1_sincronia = 'G';
    dados_lora.roll = roll_atual;

    /* Calcula checksum */
    pt_dados_lora = (char *)&dados_lora;
    for (i=0; i<sizeof(TDadosLora)-1; i++)
    {
      dados_lora.checksum = dados_lora.checksum + *pt_dados_lora;
      pt_dados_lora++;
    }
    dados_lora.checksum = (~dados_lora.checksum) + 1;
                
    LoRa.beginPacket();
    LoRa.write((unsigned char *)&dados_lora, sizeof(TDadosLora));
    LoRa.endPacket();

    Serial.println("*** Envio LoRa realizado!");
    
    timestamp_envio_LoRa = millis();
  }
}
