#include <Pixy2.h>
#include <HCSR04.h>
#include <Servo.h>

Servo servo;
#define pservo 37

// Sensor ultrassons - HCSR04 - definição de pinos
const int trigPin = 12;
const int echoPin = 13;

UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);

Pixy2 pixy;
double kp = 2.5; // é é aproximadamente proporcional à posição do seu robô em relação à linha. Ou seja, se o seu robô estiver precisamente centrado na linha, esperamos um valor proporcional de exatamente 0.
double ki = 1; //  regista o histórico do movimento do robô: é uma soma de todos os valores do termo proporcional que foram registrados desde que o robô começou a funcionar.
double kd = 22.5; // é a constante utilizada para me permitir que não haja oscilações

#define SpeedFast    100
#define SpeedSlow    50
#define SpeedStop    0

// Driver de motores - TB6612FNG - definição de pinos
//Motor Esquerdo
const int AIN1 = 9; //motor A
const int AIN2 = 8;
const int PWMA = 11;
//Motor Direito
const int BIN1 = 6; //motor B
const int BIN2 = 5;
const int PWMB = 10;

const int STBY = 7;

// definir variáveis PID
unsigned long currentTime, previousTime; // tempo atual e tempo anterior
double elapsedTime; // tempo decorrido
double error; // error que é o meu erro proporcional
double lastError; // último error
double input, output, setPoint; // setPoint vai ser a coordenada no caso que quero que o meu rôbo esteja sempre neste caso é o centro da linha
double cumError, rateError; // cumError meu erro integral que é o erro cumulativo ao longo do tempo, rate Error meu erro deritivo é a taxa de alteração do erro
int time_mili = 0; // número de milisegundos passados desde o ínicio do programa esse número excederá (retornará a zero), após aproximadamente 50 dias.
double OUT_PID, Input_PID, temp_PID;
int VetorHead = 0, VetorTail = 0; // a    corresponde há ponta do vetor e o b    corresponde há cauda do vetor

//definir variáveis do sensor ultrassons
long duration;
int distance;
int d1 = 0; // objeto há direita
int d2 = 0; // objeto há esquerda

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  setPoint = 40;
  servo.attach(pservo);
  servo.write(90);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode (AIN2, OUTPUT);
  pinMode (AIN1, OUTPUT);
  pinMode (PWMA, OUTPUT);

  pinMode (BIN2, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (PWMB, OUTPUT);

  pinMode (STBY, OUTPUT);

  pixy.init();
  pixy.setLamp(1, 1);
  Serial.println(pixy.changeProg("line"));

}

void loop()
{
  int8_t res;
  char buf[96]; //Um tipo de dados usado para armazenar um valor de caractere.

  if (distanceSensor.measureDistanceCm() <= 10)
  {
    parar();
    escolherLadoesquerdo();
    delay(1800);
    andarparafrente();
    delay(1000);
    virarparadireita();
    delay(1800);
    andarparafrente();
    delay(1200);
    virarparadireita();
    delay(1800);
    andarparafrente();
    delay(1100);
    virarparaesquerda();
    delay(1500);
  }
  /*
    if (distanceSensor.measureDistanceCm() <= 10)
    {
    parar();
    escolherLadodireito();
    delay(1800);
    andarparafrente();
    delay(1000);
    virarparaesquerda();
    delay(1800);
    andarparafrente();
    delay(1200);
    virarparaesquerda();
    delay(1800);
    andarparafrente();
    delay(1100);
    virarparadireita();
    delay(1500);
    }
  */

  else  
  {
    res = pixy.line.getMainFeatures();// Para obter últimas informações da câmara, inclui vetores, interseções e códigos de barras
    if (res <= 0)
    {
      parar();
      //Serial.print("stop  ");
      //Serial.println(res);
      return;
    }

    if (pixy.line.numVectors)
    {
      pixy.line.vectors->print();
      VetorHead = pixy.line.vectors->m_x1;
      VetorTail = pixy.line.vectors->m_x0;
      //Serial.println(a);
      //Serial.println(b);
      Input_PID = (VetorHead + VetorTail) / 2;
      //Serial.println(Input_PID);
      temp_PID = computePID(Input_PID);
      OUT_PID = constrain(temp_PID, -150, 150);
      //Serial.println(error);
      //Serial.println(OUT_PID);

      if (-5 <= error && error <= 5) // está entre estes valores anda para a frente
      {
        andarparafrente();
        //Serial.println(anda em frente);
      }
      else if (error > 4) // se o erro for maior que 5 a roda B vai ter uma velocidade diferente
      {
        error_B();
      }
      else if (error < -4) // se o erro for menor que -5 a roda A vai ter uma velocidade diferente
      {
        error_A();
      }
    }

    if (res & LINE_INTERSECTION)
    {
      pixy.line.intersections->print();
      //Serial.println("teste do ciclo");
      if (pixy.line.intersections->m_n == 4)
      {
        //Serial.println("teste do ciclo 1");
        pixy.line.setNextTurn(-90); //Virar para a direita
      }
    }
  }
}

void escolherLadoesquerdo() {

  delay(750);
  servo.write(150); // Direita
  delay(750);
  d1 = distanceSensor.measureDistanceCm();
  delay(750);
  servo.write(30); // Esquerda
  delay(750);
  d2 = distanceSensor.measureDistanceCm();;
  delay(750);
  servo.write(90); // Centro
  //Serial.print("Direita.: "); Serial.println(d1);
  //Serial.print("Esquerda: "); Serial.println(d2);
  if (d1 > d2)
  {
    //Serial.println("Virar para esquerda");
    virarparaesquerda();
  }

  else if (!d2 && !d1)
  {
    //Serial.println("Virar para direita");
    virarparaesquerda();
  }
}
/*
  void escolherLadodireito() {

  delay(750);
  servo.write(150); // Direita
  delay(750);
  d1 = distanceSensor.measureDistanceCm();
  delay(750);
  servo.write(30);// Esquerda
  delay(750);
  d2 = distanceSensor.measureDistanceCm();;
  delay(750);
  servo.write(90); // Centro
  delay(750);
  Serial.print("Direita.: "); Serial.println(d1);
  Serial.print("Esquerda: "); Serial.println(d2);
  if (d2 > d1)
  {
    Serial.println("Virar para direita");
    virarparadireita();
  }
  }
*/
double computePID(double inp) { //computePID função que permite calcular o PID
  currentTime = millis(); // obter o tempo atual
  elapsedTime = (double)(currentTime - previousTime); // calcular o tempo decorrido

  error = setPoint - inp; // calcular erro proporcional
  cumError += error * elapsedTime; // calcular erro integral
  rateError = (error - lastError) / elapsedTime; // calcular o erro derivativo

  double out = kp * error + ki * cumError + kd * rateError; // Calcular o OUTPUT PID

  lastError = error; // obter o último erro
  previousTime = currentTime; // obter o tempo anterior

  return out; // função para retornar á saída PID
}

void andarparafrente() {
  digitalWrite (AIN2, HIGH);
  digitalWrite (AIN1, LOW);
  analogWrite (PWMA, SpeedFast);
  digitalWrite (BIN2, LOW);
  digitalWrite (BIN1, HIGH);
  analogWrite  (PWMB, SpeedFast);
  digitalWrite(STBY, HIGH);
}

void virarparadireita() {
  digitalWrite (AIN2, HIGH);
  digitalWrite (AIN1, LOW);
  analogWrite (PWMA, SpeedFast);
  digitalWrite (BIN2, LOW);
  digitalWrite (BIN1, LOW);
  analogWrite  (PWMB, SpeedStop);
  digitalWrite(STBY, HIGH);
}

void virarparaesquerda() {
  digitalWrite (AIN2, LOW);
  digitalWrite (AIN1, LOW);
  analogWrite (PWMA, SpeedStop);
  digitalWrite (BIN2, LOW);
  digitalWrite (BIN1, HIGH);
  analogWrite  (PWMB, SpeedFast);
  digitalWrite(STBY, HIGH);
}

void andarparatras() {
  digitalWrite (AIN2, LOW);
  digitalWrite (AIN1, HIGH);
  analogWrite (PWMA, SpeedFast);
  digitalWrite (BIN2, HIGH);
  digitalWrite (BIN1, LOW);
  analogWrite  (PWMB, SpeedFast);
  digitalWrite(STBY, HIGH);
}

void error_B() {
  digitalWrite (AIN2, HIGH);
  digitalWrite (AIN1, LOW);
  analogWrite (PWMA, SpeedSlow);
  digitalWrite (BIN2, LOW);
  digitalWrite (BIN1, HIGH);
  analogWrite  (PWMB, OUT_PID);
  digitalWrite(STBY, HIGH);
}

void error_A() {
  digitalWrite (AIN2, HIGH);
  digitalWrite (AIN1, LOW);
  analogWrite (PWMA, OUT_PID);
  digitalWrite (BIN2, LOW);
  digitalWrite (BIN1, HIGH);
  analogWrite  (PWMB, SpeedSlow);
  digitalWrite(STBY, HIGH);
}

void parar() {
  analogWrite (PWMA, SpeedStop);
  analogWrite (PWMB, SpeedStop);
}
