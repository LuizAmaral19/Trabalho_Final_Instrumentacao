#include <Wire.h>
#include <math.h>

// --- Mapeamento de Hardware (ESP32) ---
#define SDA_PIN 21
#define SCL_PIN 22

const uint8_t MPU = 0x68; // Endereço I2C do MPU-6050

// --- Parâmetros de Calibração Estática (Substitua pelos valores medidos na bancada) ---
// A_cal = (A_saida - A_offset) / Ganho
float GanhoX = 1.00111, OffsetX = 0.04078;
float GanhoY = 1.00205, OffsetY = -0.01777;
float GanhoZ = 1.00904, OffsetZ = -0.12113;

// --- Variáveis Globais ---
float AccX_raw, AccY_raw, AccZ_raw;
float AccX, AccY, AccZ;
float accAngleX, accAngleY;

void lerAcelerometroBruto();
void rotinaCalibracaoEstatica();

void setup() {
  Serial.begin(115200);
  delay(500);

  // Inicializa I2C na ESP32 (begin ANTES de setClock)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Acorda o MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configura fundo de escala do acelerômetro para +/- 2g (16384 LSB/g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Serial.println("\n=== Sistema de Medicao de Inclinacao (ESP32 + MPU-6050) ===");
  Serial.println("Envie 'c' no Monitor Serial nos proximos 5s para rodar a calibracao estatica (6 posicoes)...");

  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'c' || cmd == 'C') {
        rotinaCalibracaoEstatica();
        break;
      }
    }
  }

  Serial.println("Iniciando medicao continua de inclinacao...");
}

void loop() {
  lerAcelerometroBruto();

  // Aplica a compensação linear: A_cal = (A_saida - A_offset) / Ganho
  AccX = (AccX_raw - OffsetX) / GanhoX;
  AccY = (AccY_raw - OffsetY) / GanhoY;
  AccZ = (AccZ_raw - OffsetZ) / GanhoZ;

  // Cálculo do ângulo no plano X-Y alinhado ao transferidor (0° -> 90° -> 180°)
  // Se em 90° o valor de Y no Serial for NEGATIVO (~ -1.000), troque AccY por -AccY
  float anguloTransferidor = atan2(AccY, -AccX) * 180.0 / PI;

  // Evita salto de 180° para -180° nos extremos
  if (anguloTransferidor < -90.0) {
    anguloTransferidor += 360.0;
  }

  Serial.print("Acc_g [X, Y, Z]: ");
  Serial.print(AccX, 3); Serial.print(", ");
  Serial.print(AccY, 3); Serial.print(", ");
  Serial.print(AccZ, 3);
  Serial.print("  ||  Angulo Transferidor: ");
  Serial.print(anguloTransferidor, 2);
  Serial.println(" deg");

  delay(100);
}

// ============= Leitura Bruta em [g] (Corrigida para 32 bits da ESP32) =============
void lerAcelerometroBruto() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)6, true);

  // Cast explícito para int16_t preserva o sinal negativo na ESP32
  int16_t rawX = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t rawY = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t rawZ = (int16_t)((Wire.read() << 8) | Wire.read());

  AccX_raw = rawX / 16384.0;
  AccY_raw = rawY / 16384.0;
  AccZ_raw = rawZ / 16384.0;
}

// ============= Coleta de Média (Mínimo 100 amostras) =============
float coletarMediaEixo(int eixo, int numAmostras) {
  double soma = 0.0;
  for (int i = 0; i < numAmostras; i++) {
    lerAcelerometroBruto();
    if (eixo == 0) soma += AccX_raw;
    else if (eixo == 1) soma += AccY_raw;
    else soma += AccZ_raw;
    delay(5);
  }
  return (float)(soma / numAmostras);
}

void aguardarConfirmacaoSerial() {
  while (Serial.available() > 0) Serial.read(); // Limpa buffer
  while (Serial.available() == 0) {
    delay(10);
  }
  while (Serial.available() > 0) Serial.read();
}

// ============= Rotina Guiada das 6 Posições (+1g e -1g) =============
void rotinaCalibracaoEstatica() {
  const int N = 150; // > 100 leituras exigidas pelo roteiro
  float pos1g[3], neg1g[3];
  const char* nomesEixos[3] = {"X", "Y", "Z"};

  Serial.println("\n--- INICIANDO CALIBRACAO ESTATICA (6 POSICOES) ---");
  for (int eixo = 0; eixo < 3; eixo++) {
    Serial.print("Posicione o eixo "); Serial.print(nomesEixos[eixo]);
    Serial.println(" apontando para +1g (nivelado) e envie qualquer tecla...");
    aguardarConfirmacaoSerial();
    pos1g[eixo] = coletarMediaEixo(eixo, N);
    Serial.print("Media +1g: "); Serial.println(pos1g[eixo], 5);

    Serial.print("Posicione o eixo "); Serial.print(nomesEixos[eixo]);
    Serial.println(" apontando para -1g (invertido) e envie qualquer tecla...");
    aguardarConfirmacaoSerial();
    neg1g[eixo] = coletarMediaEixo(eixo, N);
    Serial.print("Media -1g: "); Serial.println(neg1g[eixo], 5);
  }

  // Resolve o sistema de 2 equações para cada eixo
  GanhoX  = (pos1g[0] - neg1g[0]) / 2.0;
  OffsetX = (pos1g[0] + neg1g[0]) / 2.0;

  GanhoY  = (pos1g[1] - neg1g[1]) / 2.0;
  OffsetY = (pos1g[1] + neg1g[1]) / 2.0;

  GanhoZ  = (pos1g[2] - neg1g[2]) / 2.0;
  OffsetZ = (pos1g[2] + neg1g[2]) / 2.0;

  Serial.println("\n=== RESULTADOS PARA A TABELA DO RELATORIO ===");
  Serial.print("Eixo X -> Ganho: "); Serial.print(GanhoX, 5); Serial.print(" | Offset [g]: "); Serial.println(OffsetX, 5);
  Serial.print("Eixo Y -> Ganho: "); Serial.print(GanhoY, 5); Serial.print(" | Offset [g]: "); Serial.println(OffsetY, 5);
  Serial.print("Eixo Z -> Ganho: "); Serial.print(GanhoZ, 5); Serial.print(" | Offset [g]: "); Serial.println(OffsetZ, 5);
  Serial.println("Copie esses valores para as variaveis globais no topo do codigo!\n");
}