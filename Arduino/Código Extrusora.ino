#include <AccelStepper.h>

// =============================================================================
// SEÇÃO 1 — PINOS DO HARDWARE
// =============================================================================
#define STEP_PIN 11
#define DIR_PIN  10
#define ENA_PIN  12
const int pinoTermistor = A0; 
const int PINO_HEATER   = 3; 

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int PASSOS_POR_VOLTA = 200; 
double rpmSelecionado = 20.0;     
double rpmMotorAtual   = 0.0;      
const double RPM_MAXIMO = 300.0;   
int sentidoGiro = 1;             
const double ZONA_RAMPA_MOTOR = 25.0; 

double fluxoAlvo = 0.0;
double aceleracaoMotor = 100.0; 

// =============================================================================
// SEÇÃO 3 — PARÂMETROS DO TERMISTOR NTC
// =============================================================================
const double beta = 3950.0;
const double r0   = 100000.0;
const double t0   = 273.0 + 25.0;
const double rx   = r0 * exp(-beta / t0);
const double vcc  = 5.0;                  
const double resistorDivisor = 1800.0;    
const int numeroAmostras = 5;

double temperaturaAlvo = 250.0;
bool   sistemaAtivo    = false; 
bool   heaterLigado    = false;
double temperaturaAtual = 0.0;
bool tracionamentoManual = false;

// PID Oficial (Controla a máquina)
double kp = 15.0; 
double ki = 0.5;  
double kd = 20.0; 

// PID Temporário (O que aparece no ecrã antes de apertar "Alterar")
double kp_temp = 15.0; 
double ki_temp = 0.5;  
double kd_temp = 20.0; 

double erroIntegral = 0.0;
double erroAnterior = 0.0;
unsigned long ultimoTempoPID = 0;

double temperaturaFiltrada = 0.0;
const double fatorFiltro = 0.15; 
bool primeiraLeitura = true;

// Variáveis de Produção
double pesoGarrafa = 13.0; 
double gramasProduzidas = 0.0;
double metrosProduzidos = 0.0;
unsigned long tempoPercorridoSegundos = 0; 
unsigned long ultimoTempoMetragem = 0;
const unsigned long intervaloMetragem = 1000; 

const int PWM_MAXIMO = 255;
int dutyCicloAtual   = 0;

unsigned long ultimaLeituraTemp = 0;
unsigned long ultimoDebugSerial = 0;
const unsigned long intervaloLeituraTemp = 315;
const unsigned long intervaloDebugSerial = 300;

void(* resetFunc) (void) = 0;

double rpmParaStepsPorSegundo(double rpm) {
  return (rpm * PASSOS_POR_VOLTA) / 60.0;
}

int calcularPID(double tempAtual, double sp) {
  unsigned long tempoAtual = millis();
  if (ultimoTempoPID == 0) {
    ultimoTempoPID = tempoAtual;
    erroAnterior = sp - tempAtual;
  }
  
  double deltaTempo = (tempoAtual - ultimoTempoPID) / 1000.0; 
  if (deltaTempo <= 0.0) return 0; 
  
  double erro = sp - tempAtual;
  double P = kp * erro;
  
  if (abs(erro) < 15.0) erroIntegral += erro * deltaTempo;
  else erroIntegral = 0.0; 
  
  double I = ki * erroIntegral;
  if (I > PWM_MAXIMO) I = PWM_MAXIMO;
  if (I < -PWM_MAXIMO) I = -PWM_MAXIMO;
  
  double derivada = (erro - erroAnterior) / deltaTempo;
  double D = kd * derivada;
  double saida = P + I + D;
  
  erroAnterior = erro;
  ultimoTempoPID = tempoAtual;
  
  if (saida > PWM_MAXIMO) saida = PWM_MAXIMO;
  if (saida < 0) saida = 0;
  return (int)saida;
}

double calcularRpmMotor(double tempAtual, double sp, double rpmAlvo, double zonaRampa) {
  double inicioRampa = sp - zonaRampa;
  if (tempAtual <= inicioRampa) return 0.0;
  double fracao = (tempAtual - inicioRampa) / zonaRampa;
  double rpmCalculado = fracao * rpmAlvo;
  if (rpmCalculado > RPM_MAXIMO) return RPM_MAXIMO;
  return rpmCalculado;
}

float calcularTempoRestante() {
  if (rpmMotorAtual <= 0 || !sistemaAtivo) return 0.0;
  
  double alvoGramas = pesoGarrafa * 0.88; 
  double gramasRestantes = alvoGramas - gramasProduzidas;
  if (gramasRestantes < 0) gramasRestantes = 0.0;

  double fatorProporcional = rpmMotorAtual / 20.0;
  if (fatorProporcional <= 0) return 0.0;
  
  double tempoBaseRestante = gramasRestantes * (56.0 / 11.0);
  return tempoBaseRestante / fatorProporcional;
}

void enviarParaNextion(String comando) {
  Serial.print(comando);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void setup() {
  Serial.begin(115200);   
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 
  motor.setMaxSpeed(rpmParaStepsPorSegundo(RPM_MAXIMO));
  motor.setAcceleration(aceleracaoMotor); 
  motor.setSpeed(0);
  pinMode(PINO_HEATER, OUTPUT);
  analogWrite(PINO_HEATER, PWM_MAXIMO); 
  ultimoTempoMetragem = millis();
}

void loop() {
  motor.runSpeed();
  unsigned long agora = millis();

  // ---------------------------------------------------------------------------
  // RECEPÇÃO DE COMANDOS DA NEXTION
  // ---------------------------------------------------------------------------
  if (Serial.available()) {
    char comando = Serial.peek();

    if (comando == 'S') {
      Serial.read();
      int estado = (int)Serial.parseFloat();
      if (estado == 1) {
        sistemaAtivo = true;
        digitalWrite(ENA_PIN, LOW);
        metrosProduzidos = 0.0; 
        gramasProduzidas = 0.0;
        tempoPercorridoSegundos = 0;
      } else if (estado == 0) {
        sistemaAtivo = false;
        dutyCicloAtual = 0;
        analogWrite(PINO_HEATER, PWM_MAXIMO); 
        rpmMotorAtual = 0.0;
        motor.setSpeed(0);  
        if (!tracionamentoManual) digitalWrite(ENA_PIN, HIGH);
        erroIntegral = 0.0;
        ultimoTempoPID = 0;                
      }
    }
    else if (comando == 'W') {
      Serial.read();
      int estado = (int)Serial.parseFloat();
      if (estado == 1) {
        tracionamentoManual = true;
        digitalWrite(ENA_PIN, LOW); 
      } else if (estado == 0) {
        tracionamentoManual = false;
        if (!sistemaAtivo) digitalWrite(ENA_PIN, HIGH); 
      }
    }
    else if (comando == 'B' || comando == 'G') {
      Serial.read();
      double val = Serial.parseFloat();
      if (val > 0.0) pesoGarrafa = val;
    }
    else if (comando == 'T') {
      Serial.read();
      double val = Serial.parseFloat();
      if (val >= 50.0 && val <= 400.0) temperaturaAlvo = val;
    } 
    else if (comando == 'R') {
      Serial.read();
      double val = Serial.parseFloat();
      if (val >= 0 && val <= RPM_MAXIMO) {
        rpmSelecionado = val;
        if (sistemaAtivo) motor.setSpeed(sentidoGiro * rpmParaStepsPorSegundo(rpmMotorAtual));
      }
    } 
    else if (comando == 'D') {
      Serial.read();
      int val = (int)Serial.parseFloat();
      if (val == 1 || val == -1) {
        sentidoGiro = val;
        if (sistemaAtivo) motor.setSpeed(sentidoGiro * rpmParaStepsPorSegundo(rpmMotorAtual));
      }
    }
    // Recebe o valor inteiro do teclado e converte para decimal
    else if (comando == 'P') {
      Serial.read();
      kp_temp = Serial.parseFloat() / 10.0; 
    }
    else if (comando == 'I') {
      Serial.read();
      ki_temp = Serial.parseFloat() / 10.0;
    }
    else if (comando == 'V') {
      Serial.read();
      kd_temp = Serial.parseFloat() / 10.0;
    }
    // Botão "Alterar" grava os valores temporários no PID oficial
    else if (comando == 'K') {
      Serial.read();
      kp = kp_temp;
      ki = ki_temp;
      kd = kd_temp;
    }
    else if (comando == 'F') {
      Serial.read();
      fluxoAlvo = Serial.parseFloat();
    }
    else if (comando == 'A') {
      Serial.read();
      aceleracaoMotor = Serial.parseFloat();
      motor.setAcceleration(aceleracaoMotor); 
    }
    else if (comando == 'C') {
      Serial.read();
      metrosProduzidos = 0.0;
      gramasProduzidas = 0.0;
      tempoPercorridoSegundos = 0;
    }
    else if (comando == 'X') {
      Serial.read();
      delay(200);
      resetFunc(); 
    }
    else {
      Serial.read(); 
    }
  }

  // ---------------------------------------------------------------------------
  // LEITURA DE TEMPERATURA E PID
  // ---------------------------------------------------------------------------
  if (agora - ultimaLeituraTemp >= intervaloLeituraTemp) {
    ultimaLeituraTemp = agora;
    int somaADC = 0;
    for (int i = 0; i < numeroAmostras; i++) {
      somaADC += analogRead(pinoTermistor);
      unsigned long inicioEspera = micros();
      while (micros() - inicioEspera < 3000UL) motor.runSpeed();
    }

    double adcMedio       = somaADC / (double)numeroAmostras;
    double tensaoNTC      = (vcc * adcMedio) / 1023.0;
    double resistenciaNTC = (vcc * resistorDivisor) / tensaoNTC - resistorDivisor;
    double temperaturaK   = beta / log(resistenciaNTC / rx);
    double temperaturaBruta = temperaturaK - 273.0; 

    if (primeiraLeitura) {
      temperaturaFiltrada = temperaturaBruta; 
      primeiraLeitura = false;
    } else {
      temperaturaFiltrada = (temperaturaBruta * fatorFiltro) + (temperaturaFiltrada * (1.0 - fatorFiltro));
    }
    temperaturaAtual = temperaturaFiltrada;

    if (sistemaAtivo) {
      dutyCicloAtual = calcularPID(temperaturaAtual, temperaturaAlvo);
      analogWrite(PINO_HEATER, PWM_MAXIMO - dutyCicloAtual);
      heaterLigado = (dutyCicloAtual > 0);
    } else {
      dutyCicloAtual = 0;
      analogWrite(PINO_HEATER, PWM_MAXIMO);
      erroIntegral = 0.0; 
      ultimoTempoPID = 0;
      heaterLigado = false;
    }

    if (tracionamentoManual) {
      rpmMotorAtual = rpmSelecionado; 
      motor.setSpeed(sentidoGiro * rpmParaStepsPorSegundo(rpmMotorAtual));
    } else if (sistemaAtivo) {
      rpmMotorAtual = calcularRpmMotor(temperaturaAtual, temperaturaAlvo, rpmSelecionado, ZONA_RAMPA_MOTOR);
      motor.setSpeed(sentidoGiro * rpmParaStepsPorSegundo(rpmMotorAtual));
    } else {
      rpmMotorAtual = 0.0;
      motor.setSpeed(0);
    }
  }

  // ---------------------------------------------------------------------------
  // LÓGICA PROPORCIONAL DE PRODUÇÃO E TEMPO
  // ---------------------------------------------------------------------------
  if (sistemaAtivo && !tracionamentoManual) {
    if (agora - ultimoTempoMetragem >= intervaloMetragem) {
      ultimoTempoMetragem = agora;

      if (rpmMotorAtual > 0) {
        tempoPercorridoSegundos++;
        
        double fatorProporcional = rpmMotorAtual / 20.0;
        metrosProduzidos += (0.065 / 60.0) * fatorProporcional;
        gramasProduzidas += (11.0 / 3600.0) * fatorProporcional;
      }
    }
  } else {
    ultimoTempoMetragem = agora;
  }

  // ---------------------------------------------------------------------------
  // ATUALIZAÇÃO DA NEXTION
  // ---------------------------------------------------------------------------
  if (agora - ultimoDebugSerial >= intervaloDebugSerial) {
    ultimoDebugSerial = agora;

    String statusTexto  = sistemaAtivo ? "Ativado" : "Parado";
    String estadoMotor  = (sistemaAtivo || tracionamentoManual) ? "Rodando" : "Parado";
    String sentidoTexto = (sentidoGiro == 1) ? "Horario" : "Anti-H.";
    String strTempAtual = String(temperaturaAtual, 1); 
    String strTempAlvo  = String(temperaturaAlvo, 1);
    String strDuty      = String((dutyCicloAtual * 100.0) / 255.0, 0);
    String strRpmAtual  = String(rpmMotorAtual, 1);
    String strRpmAlvo   = String(rpmSelecionado, 1);
    
    String strTempoPerc = String(tempoPercorridoSegundos / 60.0, 1) + "min";
    String strTempoRest = String(calcularTempoRestante(), 1) + "min";
    String strProduzido = String(metrosProduzidos, 2) + "m | " + String(gramasProduzidas, 1) + "g";
    String strAcel      = String(aceleracaoMotor, 1); 
    String strGarrafaTxt = String(pesoGarrafa, 0) + "g";

    double alvoGramas = pesoGarrafa * 0.88;
    int porcentagemProducao = 0;
    if (alvoGramas > 0) {
      porcentagemProducao = (int)((gramasProduzidas / alvoGramas) * 100.0);
      porcentagemProducao = constrain(porcentagemProducao, 0, 100);
    }

    enviarParaNextion("homepage.t8.txt=\"" + statusTexto + "\"");
    enviarParaNextion("temperatura.t8.txt=\"" + statusTexto + "\"");
    enviarParaNextion("producao.t8.txt=\"" + statusTexto + "\"");
    enviarParaNextion("configuracoes.t1.txt=\"" + statusTexto + "\"");

    enviarParaNextion("homepage.t0.txt=\"" + strTempAtual + "\"");
    enviarParaNextion("homepage.t1.txt=\"" + strTempAlvo + "\"");
    enviarParaNextion("homepage.t2.txt=\"" + strDuty + "%\"");
    enviarParaNextion("homepage.t3.txt=\"" + strTempoPerc + "\"");    
    enviarParaNextion("homepage.t4.txt=\"" + strRpmAtual + "\"");
    enviarParaNextion("homepage.t5.txt=\"" + strRpmAlvo + "\"");
    enviarParaNextion("homepage.t6.txt=\"" + sentidoTexto + "\"");
    enviarParaNextion("homepage.t7.txt=\"" + strProduzido + "\"");   

    // MUDANÇA AQUI: Mostra os valores "temp" pendentes no ecrã para confirmação
    enviarParaNextion("temperatura.t0.txt=\"" + strTempAtual + "\"");
    enviarParaNextion("temperatura.t1.txt=\"" + strTempAlvo + "\"");  
    enviarParaNextion("temperatura.t2.txt=\"" + String(kp_temp, 1) + "\""); 
    enviarParaNextion("temperatura.t3.txt=\"" + String(ki_temp, 1) + "\""); 
    enviarParaNextion("temperatura.t4.txt=\"" + String(kd_temp, 1) + "\""); 
    enviarParaNextion("temperatura.t7.txt=\"" + strDuty + "%\"");

    enviarParaNextion("producao.t0.txt=\"" + strTempoRest + "\"");     
    enviarParaNextion("producao.t1.txt=\"" + strProduzido + "\"");    
    enviarParaNextion("producao.t2.txt=\"" + sentidoTexto + "\"");
    enviarParaNextion("producao.t3.txt=\"" + estadoMotor + "\"");
    enviarParaNextion("producao.t4.txt=\"" + strGarrafaTxt + "\"");   
    enviarParaNextion("producao.t5.txt=\"" + strRpmAlvo + "\"");   
    enviarParaNextion("producao.t7.txt=\"" + strRpmAtual + "\"");

    enviarParaNextion("sptemp.t1.txt=\"" + strTempAlvo + "\"");
    enviarParaNextion("sprpm.t1.txt=\"" + strRpmAlvo + "\"");

    enviarParaNextion("configuracoes.j0.val=" + String(porcentagemProducao));
    enviarParaNextion("configuracoes.t0.txt=\"" + strAcel + "\"");
    enviarParaNextion("garrafa.t1.txt=\"" + strGarrafaTxt + "\"");
  }
}
