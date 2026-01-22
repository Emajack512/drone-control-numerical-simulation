#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// Variables DMP
bool dmpReady = false;
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

// Cuaterniones y ángulos de Euler
Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // ESP32 SDA, SCL
  Wire.setClock(400000); // I2C rápido

  Serial.println(F("Iniciando MPU6050..."));
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // Rango del giroscopio: ±250 °/s  (máxima sensibilidad)
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // FS_SEL = 0  → 131.0 LSB/(°/s)

  // (Opcional pero recomendado) Filtro DLPF y tasa
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);  // ~42 Hz BW (ruido bajo, retardo moderado)
  mpu.setRate(4);  // SMPLRT_DIV=4 → Sample Rate = 1000/(1+4)=200 Hz (con DLPF activo)


  // Calibraciones básicas (ajustá según tus valores)
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("DMP iniciado correctamente!"));
  } else {
    Serial.print(F("Error al iniciar DMP (código "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convertir a grados
    float yaw = ypr[0] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float roll = ypr[2] * 180/M_PI;

    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Roll: ");
    Serial.print(roll);
    
    }
    //Leer las velocidades angulares 
    // 1) Leer crudo del gyro (LSB)
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);  // gx = rotación sobre eje X del chip

    // 2) Convertir a °/s (FS=±250 → 131.0 LSB/(°/s))
    const float G_SENS = 131.0f;       // sensibilidad para ±250 °/s
    float p_dps = gx / G_SENS;         // roll rate en grados/seg
    float q_dps= gy / G_SENS;
    float r_dps= gz / G_SENS;

    // 3) Pasar a rad/s 
    float p_rad = p_dps * (PI / 180.0f);
    float q_rad = q_dps * (PI / 180.0f);
    float r_rad = r_dps * (PI / 180.0f);

    Serial.print(" | Vel. ang. P: ");
    Serial.print(p_rad);
    Serial.print(" | Vel. ang. Q: ");
    Serial.print(q_rad);
    Serial.print(" | Vel. ang. R: ");
    Serial.println(r_rad);
}