void inicia_motores()
{
  pinMode(FAN_PIN , OUTPUT);
  pinMode(HEATER_0_PIN , OUTPUT);
  pinMode(HEATER_1_PIN , OUTPUT);
  pinMode(LED_PIN  , OUTPUT);

  pinMode(X_STEP_PIN  , OUTPUT);
  pinMode(X_DIR_PIN    , OUTPUT);
  pinMode(X_ENABLE_PIN    , OUTPUT);

  pinMode(Y_STEP_PIN  , OUTPUT);
  pinMode(Y_DIR_PIN   , OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  pinMode(Z_STEP_PIN  , OUTPUT);
  pinMode(Z_DIR_PIN    , OUTPUT);
  pinMode(Z_ENABLE_PIN    , OUTPUT);

  pinMode(E_STEP_PIN  , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN    , OUTPUT);

  pinMode(Q_STEP_PIN  , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN    , OUTPUT);

  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);
  digitalWrite(Z_ENABLE_PIN    , LOW);
  digitalWrite(E_ENABLE_PIN    , LOW);
  digitalWrite(Q_ENABLE_PIN    , LOW);

  digitalWrite(X_DIR_PIN    , LOW);
  digitalWrite(Y_DIR_PIN    , LOW);
  digitalWrite(Z_DIR_PIN    , LOW);
  digitalWrite(Q_DIR_PIN    , LOW);
}


void movimento_translacional(char lado_roboFT, char lado_roboDE)
{
  if (lado_roboDE=='E')
  {
    if(lado_roboFT=='F') digitalWrite(X_STEP_PIN, !digitalRead(X_STEP_PIN));
    if(lado_roboFT=='T') digitalWrite(Z_STEP_PIN, !digitalRead(Z_STEP_PIN));    
  }
  if (lado_roboDE=='D')
  {
    if(lado_roboFT=='F') digitalWrite(Y_STEP_PIN, !digitalRead(Y_STEP_PIN));
    if(lado_roboFT=='T') digitalWrite(Q_STEP_PIN, !digitalRead(Q_STEP_PIN));
  }
}

void parar(char lado_roboFT, char lado_roboDE)
{
  if (lado_roboDE=='E')
  {
    if(lado_roboFT=='F') digitalWrite(X_STEP_PIN, LOW);    // Passos  de acionamento nível abaixo
    if(lado_roboFT=='T') digitalWrite(Z_STEP_PIN, LOW);
  }
  if (lado_roboDE=='D')
  {
    if(lado_roboFT=='F') digitalWrite(Y_STEP_PIN, LOW);
    if(lado_roboFT=='T') digitalWrite(Q_STEP_PIN, LOW);
  }
}


void sentido_frente(char lado_roboFT, char lado_roboDE)
{
  if (lado_roboDE=='E')
  {
    if(lado_roboFT=='F') digitalWrite(X_DIR_PIN, LOW); // Motor dianteiro direito - sentido horário
    if(lado_roboFT=='T') digitalWrite(Z_DIR_PIN, LOW); // Motor traseiro direito - sentido horário 
  }
  if (lado_roboDE=='D')
  {
  if(lado_roboFT=='F') digitalWrite(Y_DIR_PIN, HIGH); // Motor dianteiro esquerdo - sentido horário
  if(lado_roboFT=='T') digitalWrite(Q_DIR_PIN, HIGH); // Motor traseiro esquerda - sentido horário
  }
}

void sentido_tras(char lado_roboFT, char lado_roboDE)
{
  if (lado_roboDE=='E')
  {
    if(lado_roboFT=='F') digitalWrite(X_DIR_PIN, HIGH); // Motor dianteiro direito - sentido horário
    if(lado_roboFT=='T') digitalWrite(Z_DIR_PIN, HIGH); // Motor traseiro direito - sentido horário 
  }
  if (lado_roboDE=='D')
  {
  if(lado_roboFT=='F') digitalWrite(Y_DIR_PIN, LOW); // Motor dianteiro esquerdo - sentido horário
  if(lado_roboFT=='T') digitalWrite(Q_DIR_PIN, LOW); // Motor traseiro esquerda - sentido horário
  }
}

