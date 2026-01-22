#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ESP32Servo.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

Servo esc1, esc2, esc3, esc4; 
MPU6050 mpu;

// --- Fix para el autoproto de Arduino ---
    struct Polinomio2;  // declaración adelantada del struct
    struct MixerInverse; 
    struct MixerParametros;
    static inline int pwm_from_w(float w, const Polinomio2& p, int us_min, int us_max);
    struct DebugVars;                            // forward-declaration
    static inline void dbg_print_all(const DebugVars& d);  // prototipo
// ----------------------------------------

  //Estructura de datos de referencia desde el control

    typedef struct {
      float roll_ref;        // rad
      float pitch_ref;       // rad
      float r_ref;           // rad/s
      float control_altura;  // [-1, +1]
    } ControlData;

  
  // Variables fisicas del DRON
    float masa = 0.953f; //Masa en Kg
    float gravedad = 9.81f; // Gravedad de la tierra

    //Para el calculo T = J * αdes​ + w x (J * w )
      struct Mat3 {float m[3][3];}; //Creamos el tensor de inercia
      struct Vec3 {float x, y, z;}; //Creamos el vector de 3x3 a multiplicar con el tensor de inercia

      // Tensor de inercia en Kg m^2
      const Mat3 J = {{
        { 0.007853f, 0.00000083306f, 0.000040056681f },      //  { /*Jxx*/, /*Jxy*/, /*Jxz*/ }
        { 0.00000083306f, 0.008419f, -0.000000940363f },      //  { /*Jxy*/, /*Jyy*/, /*Jyz*/ }
        { 0.000040056681f, -0.000000940363f, 0.01538f }       //  { /*Jxz*/, /*Jyz*/, /*Jzz*/ }
      }};

      //Vector que devuelve la multiplicacion matricial de una matriz 3x3 por una 3x1 (J*w y J*αdes)
      static inline Vec3 mat3_mul_vec(const Mat3& A, Vec3 v) {
        Vec3 out;
        out.x = A.m[0][0]*v.x + A.m[0][1]*v.y + A.m[0][2]*v.z;
        out.y = A.m[1][0]*v.x + A.m[1][1]*v.y + A.m[1][2]*v.z;
        out.z = A.m[2][0]*v.x + A.m[2][1]*v.y + A.m[2][2]*v.z;
        return out;   // La salida es un vector llamado out
      }

      //Vector que devuelve el producto cruz de dos vectores 3x3 (Para multiplicar w x (J*w))
      static inline Vec3 cross(Vec3 a, Vec3 b) {
        Vec3 out;
        out.x = a.y*b.z - a.z*b.y;
        out.y = a.z*b.x - a.x*b.z;
        out.z = a.x*b.y - a.y*b.x;
        return out;
      }

      //Funcion que computa la formula completa T = J * αdes​ + w x (J * w )
      static inline Vec3 computed_torque(const Mat3& J, Vec3 omega, Vec3 alpha_des) {
        Vec3 Jw   = mat3_mul_vec(J, omega);       // 1) J*ω
        Vec3 giro = cross(omega, Jw);             // 2) ω × (J*ω)
        Vec3 Ja   = mat3_mul_vec(J, alpha_des);   // 3) J*α_des

        Vec3 tau;                                 // 4) T = (J*α_des) + [ω × (J*ω)]
        tau.x = Ja.x + giro.x;
        tau.y = Ja.y + giro.y;
        tau.z = Ja.z + giro.z;
        return tau;                               // N·m
      }

  //====== Variables MPU6050 ======
    // Variables fisicas medidas
      float roll=0.0f;
      float pitch=0.0f;
      float yaw=0.0f;

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
      float PQR[3]; //Velocidades angulares PQR

    //                  CONSTANTES CONTROL TOQUE EN X
    // ===== Lazo externo (Ángulo φ = roll) =====
    float Kp_roll = 3.6642f;      // ganancia proporcional (tuneada)
    float Ki_roll = 0.0060f;      // ganancia integral (tuneada)

    float I_roll     = 0.0f;      // estado del integrador
    float p_ref      = 0.0f;      // salida del PI externo (rad/s)
    const float p_ref_max = 4.0f; // límite de referencia de velocidad (≈229°/s)
    float roll_ref   = 0.0f;      // queremos nivelado: roll=0 rad

    // Anti-windup: límites del integrador (en unidades de la salida p_ref)
    const float I_roll_min = -1.0f;
    const float I_roll_max = +1.0f;

    //===== Lazo interno (Velocidad p = roll rate) =====
    float Kp_p = 75.6397f;   // [rad/s -> rad/s^2] (tuneado)
    float Ki_p = 5.1850f;
    float Kd_p = 0.5932f;

    float I_p      = 0.0f;
    float ep_prev  = 0.0f;

    const float alphaX_max = 8.0f;   // Máxima aceleración angular (≈ u_max del PID p)
    const float I_p_min = -2.0f;
    const float I_p_max =  2.0f;

  //                      CONSTANTES CONTROL TOQUE EN Y
    // ===== Lazo externo (Ángulo pitch) =====
    float Kp_pitch = 7.4330f;     // ganancia proporcional (tuneada)
    float Ki_pitch = 0.0153f;     // ganancia integral (tuneada)

    float I_pitch    = 0.0f;      // estado del integrador
    float q_ref      = 0.0f;      // salida del PI externo (rad/s)
    const float q_ref_max = 4.0f; // límite de referencia de velocidad (≈229°/s)
    float pitch_ref  = 0.0f;      // queremos nivelado: pitch=0 rad

    // Anti-windup: límites del integrador (en unidades de la salida q_ref)
    const float I_pitch_min = -1.0f;
    const float I_pitch_max = +1.0f;

    //===== Lazo interno (Velocidad q = pitch rate) =====
    float Kp_q = 21.1333f;   // [rad/s -> rad/s^2] (tuneado)
    float Ki_q = 3.9098f;
    float Kd_q = 0.2328f;

    float I_q      = 0.0f;
    float eq_prev  = 0.0f;

    const float alphaY_max = 8.0f;   // Máxima aceleración angular (≈ u_max del PID q)
    const float I_q_min = -2.0f;
    const float I_q_max =  2.0f;

  //                      CONSTANTES CONTROL TOQUE EN Z
    // ===== Único lazo PID en yaw (r) =====

    float r_ref = 0.0f;           // Referencia desde el control remoto (rad/s)
    const float r_ref_max = 1.0f; // límite de referencia de velocidad (lo podés mapear desde el TX)

    // Constantes Kp, Ki, Kd para el PID de r
    float Kp_r = 5.8820f;   // [rad/s -> rad/s^2] (tuneado)
    float Ki_r = 6.4370f;
    float Kd_r = 0.17f;

    // Anti-windup (se suele usar ±I_r_max internamente)
    float I_r = 0.0f;
    const float I_r_max = +1.5f;  // ≈ |I_max| de la simulación

    // Declaración del error previo
    float er_prev = 0.0f;

    const float alphaZ_max = 3.0f;   // Máxima aceleración angular en yaw (≈ u_max del PID r)

  // ============================ Mixer 4 motores ===================

    int   us_min  = 1000;
    int   us_max  = 2000;
    int   us_base  = 1000;

    struct MixerInverse { float B[4][4]; }; //Creamos la matriz de 4X4 B=M^-1
    struct MixerParametros{ float kT, kQ, l;};
                        
    // Asignamos cada valor a la matriz B=M^-1, devolviendola llena para usar nomas
    //Aplicamos una sola vez en el setup()
inline MixerInverse build_inverse(const MixerParametros& p) {
  MixerInverse inv{};
  const float a = p.kT;
  const float b = p.kT * p.l * 0.7071067811865476f; // l/√2
  const float c = p.kQ;
  const float q = 0.25f;

  // Orden M1(14 CCW), M2(25 CW), M3(26 CW), M4(27 CCW)
  //        U = [ T,   Tx,   Ty,   Tz ]
  inv.B[0][0]= q/a; inv.B[0][1]=-q/b; inv.B[0][2]=-q/b; inv.B[0][3]= q/c;  // M1
  inv.B[1][0]= q/a; inv.B[1][1]= q/b; inv.B[1][2]=-q/b; inv.B[1][3]=-q/c;  // M2
  inv.B[2][0]= q/a; inv.B[2][1]=-q/b; inv.B[2][2]= q/b; inv.B[2][3]=-q/c;  // M3  <-- cambia
  inv.B[3][0]= q/a; inv.B[3][1]= q/b; inv.B[3][2]= q/b; inv.B[3][3]= q/c;   // M4  <-- cambia
  return inv;
}



    //aplicamos esta funcion en el loop() devuelve el resultado de el vector W
    inline void apply_inverse(const MixerInverse& inv, const float U[4], float w2[4]) {
      for (int i=0;i<4;++i) {
        float s = 0.f; for (int j=0;j<4;++j) s += inv.B[i][j]*U[j];
        w2[i] = (s>0.f ? s : 0.f); // clamp >= 0 para evitar NaN en sqrt
      }
    }

    MixerParametros mixP{ 0.00001551f, 0.0000002061f, 0.21379f };  // kT, kQ, l
    MixerInverse   mixInv; 

    struct Polinomio2 { float a,b,c; };             //Estructura de constantes para uS = a + b*w + c*w^2

    //Funcion para que genere solo los PWM en uS con las constantes de la estructura polinnomio2 de manera uS = a + b*w + c*w^2, para cada motor
    inline int pwm_from_w(float w, const Polinomio2& p, int us_min, int us_max) {
      float u = p.a + p.b*w + p.c*(w*w);
      if (u < us_min) u = us_min;
      if (u > us_max) u = us_max;
      return (int)lroundf(u);
    }

   Polinomio2 pwm_poly[4] = {
        {1041.f, 0.08145f, 0.00219f}, // Motor 1
        {1041.f, 0.08145f, 0.00219f}, // Motor 2
        {1041.f, 0.08145f, 0.00219f}, // Motor 3
        {1041.f, 0.08145f, 0.00219f}  // Motor 4
      };

    // En configuracion de vuelo X
    static const int PWM_PIN_MOTOR_1  = 14; // Motor arriba a la izquierda    
    static const int PWM_PIN_MOTOR_2   = 25;    // Motor arriba a la derecha
    static const int PWM_PIN_MOTOR_3  = 26;  // Motor abajo a la izquierda    
    static const int PWM_PIN_MOTOR_4   = 27;     //Motor abajo a la derecha

  //Constantes de U
    float Empuje = 0.0f;
    float control_altura = 0.0f;

// Toma de tiempo
uint32_t t_prev_us = 0;

inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi ? hi : x);
}
  //======================= Comunicacion=====================================
    // --- LINK & DEBUG RX ---
    volatile bool     g_have_rx = false;
    volatile uint32_t g_rx_pkt_count = 0;
    volatile uint8_t  g_rx_src_mac[6] = {0};

    const uint32_t RX_TIMEOUT_MS = 500;     // failsafe a 500 ms (tu pedido)
    
    // Último paquete recibido + timestamp (failsafe)
    volatile ControlData g_rx_raw;
    volatile uint32_t    g_rx_last_ms = 0;

    void onReceive(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
      if (len >= (int)sizeof(ControlData)) {
        memcpy((void*)&g_rx_raw, incomingData, sizeof(ControlData));
        g_rx_last_ms = millis();
        g_have_rx = true;              // marca que ya hay primer paquete

        if (info && info->src_addr) {
          memcpy((void*)g_rx_src_mac, info->src_addr, 6);
        }
        g_rx_pkt_count++;
      }
    }

  // ================= DEBUG GLOBALS =================
    #define DEBUG_ALL        1        // 1=on, 0=off
    #define DEBUG_ALL_CSV    0        // 1=CSV, 0=humano multilínea
    const uint32_t DBG_ALL_PERIOD_MS = 2000; // cada 200 ms

    volatile uint32_t g_ctrl_tick = 0;
    uint32_t g_dbg_all_last_ms = 0;

    struct DebugVars {
      // tiempo / link
      uint32_t now_ms, age_ms, tick;
      bool rx_ok;

      // NUEVO: info de radio
        uint32_t rx_pkts;
        uint8_t  rx_mac[6];

      // referencias recibidas
      float roll_ref, pitch_ref, r_ref, control_altura;

      // medidas IMU
      float roll, pitch, yaw;   // rad
      float p, q, r;            // rad/s

      // control X (roll)
      float e_roll, p_ref, I_roll_state, ep, I_p_state, D_p, alphaX;

      // control Y (pitch)
      float e_pitch, q_ref, I_pitch_state, eq, I_q_state, D_q, alphaY;

      // control Z (yaw rate)
      float er, I_r_state, D_r, alphaZ;

      // torque / mezclado / salida
      float Empuje;
      float Tx, Ty, Tz;   // torques recortados
      float U[4], w2[4], w[4];
      int   us[4];

      // timing
      float Ts;
    };

    DebugVars g_dbg{};

    static inline void dbg_print_all(const DebugVars& d) {
      #if DEBUG_ALL
      #if DEBUG_ALL_CSV
      // --- CSV: una sola línea, ideal para log a archivo ---
      // (Opcional) imprimir la cabecera una sola vez:
      static bool header_printed = false;
      if (!header_printed) {
        Serial.println(
          "now_ms,age_ms,tick,rx_ok,rx_pkts,rx_mac,"
          "roll_ref,pitch_ref,r_ref,control_altura,"
          "roll,pitch,yaw,"
          "p,q,r,"
          "e_roll,p_ref,I_roll,ep,I_p,D_p,alphaX,"
          "e_pitch,q_ref,I_pitch,eq,I_q,D_q,alphaY,"
          "er,I_r,D_r,alphaZ,"
          "Empuje,Tx,Ty,Tz,"
          "U0,U1,U2,U3,"
          "w2_0,w2_1,w2_2,w2_3,"
          "w_0,w_1,w_2,w_3,"
          "us0,us1,us2,us3,"
          "Ts"
        );
        header_printed = true;
      }

      Serial.printf(
        "%lu,%lu,%lu,%d,%lu,%02X:%02X:%02X:%02X:%02X:%02X,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%d,%d,%d,%d,"
        "%.6f\n",

        // -------- argumentos en el mismo orden --------
        (unsigned long)d.now_ms,
        (unsigned long)d.age_ms,
        (unsigned long)d.tick,
        (int)d.rx_ok,
        (unsigned long)d.rx_pkts,
        d.rx_mac[0], d.rx_mac[1], d.rx_mac[2], d.rx_mac[3], d.rx_mac[4], d.rx_mac[5],

        // refs
        (double)d.roll_ref, (double)d.pitch_ref, (double)d.r_ref, (double)d.control_altura,

        // IMU (rad)
        (double)d.roll, (double)d.pitch, (double)d.yaw,

        // gyro (rad/s)
        (double)d.p, (double)d.q, (double)d.r,

        // control X
        (double)d.e_roll, (double)d.p_ref, (double)d.I_roll_state,
        (double)d.ep, (double)d.I_p_state, (double)d.D_p, (double)d.alphaX,

        // control Y
        (double)d.e_pitch, (double)d.q_ref, (double)d.I_pitch_state,
        (double)d.eq, (double)d.I_q_state, (double)d.D_q, (double)d.alphaY,

        // control Z
        (double)d.er, (double)d.I_r_state, (double)d.D_r, (double)d.alphaZ,

        // mixer / torques / empuje
        (double)d.Empuje, (double)d.Tx, (double)d.Ty, (double)d.Tz,

        // U[]
        (double)d.U[0], (double)d.U[1], (double)d.U[2], (double)d.U[3],

        // w2[]
        (double)d.w2[0], (double)d.w2[1], (double)d.w2[2], (double)d.w2[3],

        // w[]
        (double)d.w[0], (double)d.w[1], (double)d.w[2], (double)d.w[3],

        // PWM us[]
        d.us[0], d.us[1], d.us[2], d.us[3],

        // Ts
        (double)d.Ts
      );
      #else
     // --- Humano multilínea, ordenado por bloques ---
        Serial.printf(
          "\n===== DEBUG ALL @ %lums (age=%lums, tick=%lu, rx_ok=%d) =====\n",
          (unsigned long)d.now_ms, (unsigned long)d.age_ms, (unsigned long)d.tick, (int)d.rx_ok
        );
        Serial.printf("LINK   : pkts=%lu | MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
          (unsigned long)d.rx_pkts,
          d.rx_mac[0], d.rx_mac[1], d.rx_mac[2], d.rx_mac[3], d.rx_mac[4], d.rx_mac[5]
        );
        Serial.printf("REFS   : roll=%.3f rad | pitch=%.3f rad | r=%.3f rad/s | alt=%.3f\n",
                      (double)d.roll_ref,(double)d.pitch_ref,(double)d.r_ref,(double)d.control_altura);
        Serial.printf("IMU    : roll=%.3f | pitch=%.3f | yaw=%.3f (rad)\n",
                      (double)d.roll,(double)d.pitch,(double)d.yaw);
        Serial.printf("GYRO   : p=%.3f | q=%.3f | r=%.3f (rad/s)\n",
                      (double)d.p,(double)d.q,(double)d.r);

        Serial.printf("CTRL X : e=%.3f | p_ref=%.3f | I_roll=%.3f || ep=%.3f | I_p=%.3f | D_p=%.3f | alphaX=%.3f\n",
                      (double)d.e_roll,(double)d.p_ref,(double)d.I_roll_state,
                      (double)d.ep,(double)d.I_p_state,(double)d.D_p,(double)d.alphaX);

        Serial.printf("CTRL Y : e=%.3f | q_ref=%.3f | I_pitch=%.3f || eq=%.3f | I_q=%.3f | D_q=%.3f | alphaY=%.3f\n",
                      (double)d.e_pitch,(double)d.q_ref,(double)d.I_pitch_state,
                      (double)d.eq,(double)d.I_q_state,(double)d.D_q,(double)d.alphaY);

        Serial.printf("CTRL Z : e=%.3f | I_r=%.3f | D_r=%.3f | alphaZ=%.3f\n",
                      (double)d.er,(double)d.I_r_state,(double)d.D_r,(double)d.alphaZ);

        Serial.printf("MIXER  : Empuje=%.3f | Tx=%.3f | Ty=%.3f | Tz=%.3f\n",
                      (double)d.Empuje,(double)d.Tx,(double)d.Ty,(double)d.Tz);

        Serial.printf("U[]    : [%.3f, %.3f, %.3f, %.3f]\n",
                      (double)d.U[0],(double)d.U[1],(double)d.U[2],(double)d.U[3]);

        Serial.printf("w2[]   : [%.3f, %.3f, %.3f, %.3f]\n",
                      (double)d.w2[0],(double)d.w2[1],(double)d.w2[2],(double)d.w2[3]);

        Serial.printf("w[]    : [%.3f, %.3f, %.3f, %.3f]\n",
                      (double)d.w[0],(double)d.w[1],(double)d.w[2],(double)d.w[3]);

        Serial.printf("PWM us : [%d, %d, %d, %d]\n", d.us[0], d.us[1], d.us[2], d.us[3]);
        Serial.printf("TIMING : Ts=%.6f s\n", (double)d.Ts);

      #endif
      #endif
      }

      static inline void dbg_maybe_print() {
      #if DEBUG_ALL
        uint32_t now = millis();
        if (now - g_dbg_all_last_ms >= DBG_ALL_PERIOD_MS) {
          g_dbg_all_last_ms = now;
          dbg_print_all(g_dbg);
        }
      #endif
      }

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // ESP32 SDA, SCL
    Wire.setClock(400000); // I2C rápido

      //Iniciamos comunicacion
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
      Serial.println("[ERR] ESP-NOW init");
    } else {
      esp_now_register_recv_cb(onReceive);
      Serial.println("[OK] ESP-NOW receptor listo");
    }

    // === BLOQUEANTE: esperar PRIMER paquete ===
    Serial.print("[WAIT] Esperando primer paquete ESP-NOW");
    while (!g_have_rx) {
      Serial.print(".");
      delay(50); // respira WDT y no satura USB
    }
    Serial.println("\n[LINK] Primer paquete recibido.");

    Serial.println(F("Iniciando MPU6050..."));
    mpu.initialize();

    // Rango del giroscopio: ±250 °/s  (máxima sensibilidad)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // FS_SEL = 0  → 131.0 LSB/(°/s)

    // (Opcional pero recomendado) Filtro DLPF y tasa
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);  // ~42 Hz BW (ruido bajo, retardo moderado)
    mpu.setRate(4);  // SMPLRT_DIV=4 → Sample Rate = 1000/(1+4)=200 Hz (con DLPF activo)

    devStatus = mpu.dmpInitialize();

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

      t_prev_us = micros(); //Inicializamos micros para clock excato de F. control

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    esc1.setPeriodHertz(50);
    esc2.setPeriodHertz(50);
    esc3.setPeriodHertz(50);
    esc4.setPeriodHertz(50);
    
    // === ESC: adjuntar a 50 Hz con límites de 1000–2000 us ===

    bool okFL = esc1.attach(PWM_PIN_MOTOR_1, us_min, us_max);
    bool okFR = esc2.attach(PWM_PIN_MOTOR_2, us_min, us_max);
    bool okRR = esc3.attach(PWM_PIN_MOTOR_3, us_min, us_max);
    bool okRL = esc4.attach(PWM_PIN_MOTOR_4, us_min, us_max);

    if (!okFL || !okFR || !okRR || !okRL) {
      Serial.println("[ERR] attach ESC");
    }

    mixInv = build_inverse(mixP);

  // Arrancar en seguro y despues saltar a la base 
      // Armado básico: ambos a MIN por ~1.5 s, luego a base
      esc1.writeMicroseconds(us_min);
      esc2.writeMicroseconds(us_min);
      esc3.writeMicroseconds(us_min);
      esc4.writeMicroseconds(us_min);
      delay(3000);

}

void loop() {

    g_ctrl_tick++;
  //=========== COMUNICACION =======================
      constexpr float ROLL_MAX_RAD  = 15.0f * 3.14159265358979323846f / 180.0f;
      constexpr float PITCH_MAX_RAD = 15.0f * 3.14159265358979323846f / 180.0f;

      // snapshot
      ControlData rx;
      memcpy(&rx, (const void*)&g_rx_raw, sizeof(rx));
      bool rx_ok = (millis() - g_rx_last_ms) <= RX_TIMEOUT_MS;
      
      g_dbg.now_ms = millis();
      g_dbg.age_ms = g_dbg.now_ms - g_rx_last_ms;
      g_dbg.tick   = g_ctrl_tick;
      g_dbg.rx_ok  = rx_ok;

      g_dbg.rx_pkts = g_rx_pkt_count;
      memcpy(g_dbg.rx_mac, (const void*)g_rx_src_mac, 6);

      if (!rx_ok) {
        // opcional: resetear integradores/refs
        roll_ref = 0.0f; pitch_ref = 0.0f; r_ref = 0.0f; control_altura = 0.0f;
        I_roll = I_pitch = I_p = I_q = I_r = 0.0f;

        Serial.println("Comunicacion perdida");
        esc1.writeMicroseconds(us_min);
        esc2.writeMicroseconds(us_min);
        esc3.writeMicroseconds(us_min);
        esc4.writeMicroseconds(us_min);

          // reflejar en el DEBUG lo que acabás de forzar
        g_dbg.roll_ref = 0.0f;
        g_dbg.pitch_ref = 0.0f;
        g_dbg.r_ref = 0.0f;
        g_dbg.control_altura = 0.0f;

        dbg_maybe_print();

        return;
        }
      // mapear a tus variables internas usadas por el controlador
      roll_ref       = rx_ok ? clampf(rx.roll_ref,  -ROLL_MAX_RAD,  ROLL_MAX_RAD)  : 0.0f;
      pitch_ref      = rx_ok ? clampf(rx.pitch_ref, -PITCH_MAX_RAD, PITCH_MAX_RAD) : 0.0f;
      r_ref          = rx_ok ? rx.r_ref : 0.0f;  // ya viene en rad/s (si querés, también clamp a ±r_ref_max)
      r_ref = clampf(r_ref, -r_ref_max, r_ref_max);
      control_altura = rx_ok ? clampf(rx.control_altura, -1.0f, 1.0f) : 0.0f;

  if (!dmpReady) return;
  // ====== (A) SENSADO: leer y guardar lo último disponible ======
    // DMP (roll/pitch/yaw) cuando haya paquete:
      fifoCount = mpu.getFIFOCount();
      if (fifoCount == 1024) { mpu.resetFIFO(); }
      else if (fifoCount >= packetSize) {
      // TIP: drenar a lo último por si se acumularon varios paquetes PENSAR EN LA CINTA CON LOS TORNILLOS
        while (fifoCount >= packetSize) {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;}
      //Calculo dependiente de las librerias
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Guardar las últimas mediciones (en rad)
        yaw = ypr[0]; pitch = ypr[1]; roll = ypr[2];
      }

   // Gyro crudo (rápido): siempre que puedas
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz); //libreria que lee el giro en crudo
      const float G_SENS = 131.0f;      // FS=±250 parametro de conversion cada 131 raw data hay 1°/s
      const float DEG2RAD = PI/180.0f;  //Pasaje de °/s a rad/s
      PQR[0] = (gx / G_SENS) * DEG2RAD; // p
      PQR[1] = (gy / G_SENS) * DEG2RAD; // q
      PQR[2] = (gz / G_SENS) * DEG2RAD; // r

  // ====== (B) CONTROL-TICK: correr el control a Ts fijo ======
      static const float Ts_target = 0.004f;         // 4 ms → 250 Hz
      uint32_t t_now = micros();
      float Ts = (t_now - t_prev_us) * 1e-6f;
      if (Ts < Ts_target) return;                    // todavía no toca
      t_prev_us = t_now;                             // cierra periodo

      // Control de estado de altura

      Empuje = masa * gravedad + control_altura * 0.1 * masa * gravedad;

      //================================= CONTROL DE TORQUE EN X ===================================================
      // --- Lazo externo (PI sobre roll) usando las últimas variables ---
      //1) Cálculo del error de ángulo
      float e_roll = roll_ref - roll;  // roll en rad

      //2) Integrador externo del ángulo (I_roll)
      I_roll += Ki_roll * e_roll * Ts;
      if (I_roll > I_roll_max) I_roll = I_roll_max;
      if (I_roll < I_roll_min) I_roll = I_roll_min;

      //3) Salida del PI externo: referencia de velocidad p_ref
      p_ref = Kp_roll * e_roll + I_roll;   // [rad/s]
      if (p_ref >  p_ref_max) p_ref =  p_ref_max;
      if (p_ref < -p_ref_max) p_ref = -p_ref_max;

      //=== Lazo interno (PID sobre p) para calcular aceleración angular alphaX ===
      //1) Cálculo del error de velocidad
      float p  = PQR[0];          // [rad/s]
      float ep = p_ref - p;       // error de tasa

      //2) Cálculo de términos derivador, integrador y señal alphaX_base
      float D_p = Kd_p * (ep - ep_prev) / Ts;   // Derivada discreta

      // Integrador interno con límites (I_p)
      I_p += Ki_p * ep * Ts;
      if (I_p > I_p_max) I_p = I_p_max;
      if (I_p < I_p_min) I_p = I_p_min;

      // Salida base del PID (antes de saturar en alphaX_max)
      float alphaX_base = Kp_p * ep + I_p + D_p;

      //3) Saturación en aceleración angular (alphaX)
      float alphaX = alphaX_base;
      if (alphaX_base >=  alphaX_max) alphaX =  alphaX_max;
      if (alphaX_base <= -alphaX_max) alphaX = -alphaX_max;

      //4) Anti-windup condicional extra
      // Si estamos saturados y el error empuja hacia afuera, deshacemos la integración de este tick.
      bool sat_hix = (alphaX_base >  alphaX_max);
      bool sat_lox = (alphaX_base < -alphaX_max);
      if ((sat_hix && ep > 0) || (sat_lox && ep < 0)) {
        I_p -= Ki_p * ep * Ts;   // Quitamos lo que acabamos de sumar
      }

      // Clamp final por seguridad (por si el undo lo sacó de rango)
      if (I_p > I_p_max) I_p = I_p_max;
      if (I_p < I_p_min) I_p = I_p_min;

      //5) Actualizamos ep_prev
      ep_prev = ep;

    //======================================== CONTROL DE TORQUE EN  Y =================================================
        // --- Lazo externo (PI sobre pitch) usando las últimas variables ---
        //1) Cálculo del error de ángulo
        float e_pitch = pitch_ref - pitch;   // pitch en rad

        //2) Integrador externo del ángulo (I_pitch)
        I_pitch += Ki_pitch * e_pitch * Ts;
        if (I_pitch > I_pitch_max) I_pitch = I_pitch_max;
        if (I_pitch < I_pitch_min) I_pitch = I_pitch_min;

        //3) Salida del PI externo: referencia de velocidad q_ref
        q_ref = Kp_pitch * e_pitch + I_pitch;   // [rad/s]
        if (q_ref >  q_ref_max) q_ref =  q_ref_max;
        if (q_ref < -q_ref_max) q_ref = -q_ref_max;

        //=== Lazo interno (PID sobre q) para calcular aceleración angular alphaY ===
        //1) Cálculo del error de velocidad
        float q  = PQR[1];          // [rad/s]
        float eq = q_ref - q;       // error de tasa

        //2) Cálculo de términos derivador, integrador y señal alphaY_base
        float D_q = Kd_q * (eq - eq_prev) / Ts;   // Derivada discreta

        // Integrador interno con límites (I_q)
        I_q += Ki_q * eq * Ts;
        if (I_q > I_q_max) I_q = I_q_max;
        if (I_q < I_q_min) I_q = I_q_min;

        // Salida base del PID (antes de saturar en alphaY_max)
        float alphaY_base = Kp_q * eq + I_q + D_q;

        //3) Saturación en aceleración angular (alphaY)
        float alphaY = alphaY_base;
        if (alphaY_base >=  alphaY_max) alphaY =  alphaY_max;
        if (alphaY_base <= -alphaY_max) alphaY = -alphaY_max;

        //4) Anti-windup condicional extra
        bool sat_hiy = (alphaY_base >  alphaY_max);
        bool sat_loy = (alphaY_base < -alphaY_max);
        if ((sat_hiy && eq > 0) || (sat_loy && eq < 0)) {
          I_q -= Ki_q * eq * Ts;   // Quitamos lo que acabamos de sumar
        }

        // Clamp final por seguridad
        if (I_q > I_q_max) I_q = I_q_max;
        if (I_q < I_q_min) I_q = I_q_min;

        //5) Actualizamos eq_prev
        eq_prev = eq;


      // =================================== CONTROL DE TORQUE EN Z ===============================================
              //===Lazo interno (PID sobre r) para calcular el torque
          //1)Calculo del error
            float r = PQR[2];
            float er = r_ref - r;

          //2)Calculo de terminos derivador, integrador y señal Tz de control
            float D_r= Kd_r  *(er-er_prev) / Ts; //Calculo de la derivada
            I_r += Ki_r* er * Ts;

            if (I_r > I_r_max) I_r = I_r_max;
            if (I_r < -I_r_max) I_r = -I_r_max;
          
          //PID

          float alphaZ_base = Kp_r * er + I_r + D_r;
          
          //3)Establecer los limites seguros del toque X de control
            float alphaZ = alphaZ_base;
            if (alphaZ_base >=  alphaZ_max) alphaZ =  alphaZ_max;
            if (alphaZ_base <= -alphaZ_max) alphaZ = -alphaZ_max;

          //4) Anti-windup para no sobrecargar el termino integrador 
            // === Anti-windup básico (evitar que el integrador se infle en saturación) ===
            // Si estamos saturados y el error empuja hacia afuera, deshacemos la integración de este tick.
            bool sat_hiz = (alphaZ_base >  alphaZ_max); //Devuelve un true si esta saturado para arriba (>1)
            bool sat_loz = (alphaZ_base < -alphaZ_max);  //Devuelve un true si esta saturado para abajo (<1)
            if ((sat_hiz && er > 0) || (sat_loz && er < 0)) {
              I_r -= Ki_r * er * Ts;   // Quitamos lo que acabamos de sumar 
            }
          //5)Actualizamos er_prev
          er_prev=er;

    //Pasar de señales de control a Torques reales 

        Vec3 omega = { PQR[0], PQR[1], PQR[2] }; // (p,q,r)
        Vec3 alphaDes = { alphaX, alphaY, alphaZ };   //Vector de aceleraciones angulares
        Vec3 tau = computed_torque(J, omega, alphaDes);  //Vector de Torques

      const float Tx_max = 0.20f;
      const float Ty_max = 0.20f;
      const float Tz_max = 0.12f;

      float Tx = clampf(tau.x, -Tx_max, Tx_max);
      float Ty = clampf(tau.y, -Ty_max, Ty_max);
      float Tz = clampf(tau.z, -Tz_max, Tz_max);

    //=====MATRIZ MIXER Y SALIDA A LOS MOTORES========

      // U físico
       float U[4] = { Empuje, Tx, Ty, Tz };

      // ω^2 = B·U
      float w2[4]; apply_inverse(mixInv, U, w2);

      //  ω = sqrt(ω²)
      float w[4]  = { sqrtf(w2[0]), sqrtf(w2[1]), sqrtf(w2[2]), sqrtf(w2[3]) };

      // PWM por motor con tu regresión cuadrática (w en rad/s)
        int us[4];

        for (int i=0; i<4; i++){
          //La funcion pwm_from_w ya me devulve los uS con el polinomio, y pwm_poly tiene cargadas las ctes a,b,c de cada motor
          us[i]= pwm_from_w(w[i], pwm_poly[i], us_min, us_max);
        }

      // Escribir a los 4 ESC (mismo orden)
        esc1.writeMicroseconds(us[0]);
        esc2.writeMicroseconds(us[1]);
        esc3.writeMicroseconds(us[2]);
        esc4.writeMicroseconds(us[3]);

    //Degubs
      g_dbg.roll_ref       = roll_ref;
      g_dbg.pitch_ref      = pitch_ref;
      g_dbg.r_ref          = r_ref;
      g_dbg.control_altura = control_altura;
      g_dbg.roll = roll; g_dbg.pitch = pitch; g_dbg.yaw = yaw;
      g_dbg.p = PQR[0]; g_dbg.q = PQR[1]; g_dbg.r = PQR[2];
      g_dbg.Ts = Ts;
      g_dbg.e_roll       = e_roll;
      g_dbg.p_ref        = p_ref;
      g_dbg.I_roll_state = I_roll;
      g_dbg.ep           = ep;
      g_dbg.I_p_state    = I_p;
      g_dbg.D_p          = D_p;
      g_dbg.alphaX       = alphaX;
      g_dbg.e_pitch       = e_pitch;
      g_dbg.q_ref         = q_ref;
      g_dbg.I_pitch_state = I_pitch;
      g_dbg.eq            = eq;
      g_dbg.I_q_state     = I_q;
      g_dbg.D_q           = D_q;
      g_dbg.alphaY        = alphaY;
      g_dbg.er        = er;
      g_dbg.I_r_state = I_r;
      g_dbg.D_r       = D_r;
      g_dbg.alphaZ    = alphaZ;
      g_dbg.Empuje = Empuje;
      g_dbg.Tx = Tx; g_dbg.Ty = Ty; g_dbg.Tz = Tz;

      g_dbg.U[0] = U[0]; g_dbg.U[1] = U[1]; g_dbg.U[2] = U[2]; g_dbg.U[3] = U[3];
      g_dbg.w2[0] = w2[0]; g_dbg.w2[1] = w2[1]; g_dbg.w2[2] = w2[2]; g_dbg.w2[3] = w2[3];
      g_dbg.w[0]  = w[0];  g_dbg.w[1]  = w[1];  g_dbg.w[2]  = w[2];  g_dbg.w[3]  = w[3];
      g_dbg.us[0] = us[0]; g_dbg.us[1] = us[1]; g_dbg.us[2] = us[2]; g_dbg.us[3] = us[3];

    dbg_maybe_print();
}