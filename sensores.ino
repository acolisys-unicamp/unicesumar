/*
  UNICESUMAR (EAD - Campinas/SP) : BACHARELADO EM ENGENHARIA DE SOFTWARE
  Módulo                         : 202452
  Disciplina                     : TRABALHO DE CONCLUSÃO DE CURSO II
  Aluno                          : ALEXANDRE CARLOS DE OLIVEIRA
  Matrícula                      : RA 1609142-5
  Orientadora                    : JANAINA APARECIDA DE FREITA
  Local                          : CAMPINAS / SP
  Data                           : JUNHO / 2024

  Título do projeto:
  FENÔMENOS TRANSITÓRIOS EM SISTEMAS HIDRÁULICOS: Modelo de Sistema Inteligente para Diagnóstico Diferencial de Aeração, e de Cavitação

  Resumo do projeto:
  Este estudo propõe um modelo de sistema inteligente para diagnostico diferencial de fenômenos transitórios causados por alterações físico-químicas q
  ue afetam as características dos fluidos transportados em sistemas hidráulicos, denominados aerações e cavitações. O modelo foi testado em um sistema 
  hidráulico simplificado (maquete), com a implantação de um sistema eletroeletrônico mínimo necessário para aquisição, processamento e apresentação dos 
  dados. O algoritmo proposto utiliza modelagem matemática para prever as dinâmicas de funcionamento e as variáveis de um sistema hidráulico sob condições 
  ideais e sob a manifestação dos fenômenos estudados. Os resultados obtidos foram condizentes com os valores previamente calculados, permitindo que medidas 
  corretivas fossem tomadas antes que os problemas se tornassem críticos – o que valida o modelo como uma ferramenta alternativa viável para suporte à 
  mitigação, resolução imediata ou prevenção de ocorrências futuras desses fenômenos em sistemas hidráulicos de diferentes portes, com consequente redução 
  de riscos e de custos associados a avarias ou paradas não programadas.

  Palavras-chave:
  Palavra-Chave: automação; diagnóstico; inteligente
*/

// Definição das bibliotecas necessárias.
#include <DallasTemperature.h>  // Inclui a biblioteca DallasTemperature para o sensor de temperatura DS18B20
#include <OneWire.h>            // Inclui a biblioteca OneWire para comunicação OneWire (usada pelo sensor de temperatura DS18B20)
#include <Wire.h>               // Inclui a biblioteca Wire para comunicação I2C

// Definição da utilizados.
#define submersaBomba 5    // porta para controle da bomba submersa
#define superiorBomba 6    // porta para controle da bomba superior
#define microfone A2       // porta para leitura do microfone MAX9814
#define nivelAlto 4        // porta para leitura do sensor de nível alto
#define nivelBaixo 3       // porta para leitura do sensor de nível baixo
#define oneWireBus 7       // porta para leitura do sensor de temperatura DS18B20
#define sensorCorrente A1  // porta para leitura do sensor de corrente ACS712-20A
#define sensorTensao A0    // porta para leitura do sensor de tensão 25VDC
#define sensorVazao 2      // porta para leitura do sensor de vazão YF-S201


// Definição das informações acadêmicas
const char curso[] PROGMEM = "UNICESUMAR - E SOFT";   // Instituição e curso
const char modulo[] PROGMEM = "202452 - TCC II";      // Módulo - Trabalho
const char ra[] PROGMEM = "RA 16091425";              // Registro Acadêmico (RA)
const char aluno[] PROGMEM = "ALEXANDRE C OLIVEIRA";  // Nome do Aluno

// Definição das constantes globais
const float D_cano = 0.0127;                                       // Diâmetro interno do cano em metros (1/2 polegada)
const float A_canoST = 3.14159 * (D_cano / 2.0) * (D_cano / 2.0);  // Área da seção transversal do cano em metros quadrados
const float Vazao_fator = 7.5;                                     // Fator de conversão do sensor YF-S201 para calcular a vazão em litros por minuto (L/min)

// Definição das variáveis globais
bool H_calculado = false;          // Variável para indicar se a elevação (H) real já foi calculado
volatile unsigned int pulsos = 0;  // Variável para armazenar a contagem de pulsos do sensor de vazão YF-S201
unsigned long t_inicial;           // Variável para armazenar o tempo inicial de enchimento do cano

// Definição da estrutura de dados para armazenamento das leituras dos sensores
struct dadosSensores {    //
  float corrente = 0;     // Corrente elétrica medida, em A
  float elevacao = 0;     // Elevação real (H), em m
  float som = 0;          // Volume de som medido, em V
  float temperatura = 0;  // Temperatura medida, em °C
  float tensao = 0;       // Tensão elétrica medida, em V
  float vazao = 0;        // Vazão medida, em L/h
};                        //
dadosSensores dados;

OneWire oneWire(oneWireBus);                    // Inicialização do objeto OneWire
DallasTemperature sensorTemperatura(&oneWire);  // Inicialização do objeto DallasTemperature
DeviceAddress enderecoSensorTemperatura;        // Cria uma variável para armazenar o endereço do dispositivo

// Definição dos protótipos das funções
void configurarPortas();                                // Função para configurar e inicializar as portas utilizadas no Arduino Pro Mini
void eventoRequisicao(const dadosSensores& dados);      // Função para tratar eventos de requisição I2C
String exibirConstPROGMEM(const char* constPROGMEM);    // Função para exibir os dados das constantes PROGMEM
void idSensorTemperatura();                             // Função para exibir o endereço do sensor de temperatura DS18B20
void incrementarPulsos();                               // Função para incrementar a contagem de pulsos do sensor de vazão YF-S201
void inicializarI2C();                                  // Função para inicializar o protocolo de comunicação I2C
void inicializarSensorTemperatura();                    // Função para inicializar o sensor de temperatura DS18B20
void inicializarSerial();                               // Função para inicializar a comunicação serial
float lerMicrofone();                                   // Função para ler o microfone MAX9814
float lerSensorCorrente();                              // Função para ler o sensor de corrente ACS712-20A
float lerSensorTemperatura();                           // Função para ler o sensor de temperatura DS18B20
float lerSensorTensao();                                // Função para ler o sensor de tensão 25VDC
float lerSensorVazao();                                 // Função para ler o sensor de vazão YF-S201
void mostrardadosSensores(const dadosSensores& dados);  // Função para exibir os dados dos sensores na porta serial

/**
 * @brief Função de configuração inicial do Arduino.
 *
 * Esta função é executada uma vez quando o Arduino é ligado ou reiniciado. Ela
 * inicializa a comunicação serial, o protocolo de comunicação I2C, configura as 
 * portas de entrada e saída, e inicializa os sensores.
 */
void setup() {  //
  inicializarSerial();
  inicializarI2C();                                           // Configura e inicializa as portas utilizadas no Arduino Pro Mini
  configurarPortas();                                         // Inicializa a comunicação serial.
  inicializarSensorTemperatura();                             // Inicializa o sensor de temperatura DS18B20.
  Serial.println(F("Inicialização concluída com sucesso."));  // Mensagem de depuração na porta serial.
  Serial.println();                                           //
}

/**
 * @brief Função principal de execução contínua do Arduino.
 *
 * Esta função é chamada repetidamente durante a execução do programa. Ela
 * monitora os sensores de nível, controla as bombas e calcula os dados
 * necessários para exibição.
 */
void loop() {                                                 //
  int estadoNivelBaixo = digitalRead(nivelBaixo);             // Lê o estado dos sensores de nível BAIXO
  int estadoNivelAlto = digitalRead(nivelAlto);               // Lê o estado dos sensores de nível ALTO
  if (estadoNivelBaixo == HIGH && estadoNivelAlto == HIGH) {  // Se ambos os sensores estiverem fora da água
    digitalWrite(submersaBomba, HIGH);                        // Liga a bomba submersa
  }                                                           //
  if (estadoNivelBaixo == LOW) {                              // Se o sensor de nível baixo estiver na água
    digitalWrite(superiorBomba, HIGH);                        // Liga a bomba superior
    if (!H_calculado) {                                       // Se a elevação (H) real não foi calculado
      t_inicial = millis();                                   // Inicia a contagem do tempo
    }                                                         //
    pulsos = 0;                                               // Reseta a contagem de pulsos
  }                                                           //
  if (estadoNivelAlto == LOW) {                               // Se o sensor de nível alto estiver na água
    digitalWrite(submersaBomba, LOW);                         // Desliga a bomba submersa
    if (!H_calculado) {                                       // Se a elevação (H) real não foi calculado
      unsigned long t_decorrido = millis() - t_inicial;       // Calcula o tempo decorrido (ms)
      float Q_media = lerSensorVazao();                       // Lê a vazão atual (L/min)
      float V_total = Q_media * (t_decorrido / 60000);        // Calcula a quantidade total de água (em L) utilizando a vazão média (em L/min) e o tempo decorrido (em ms)
      dados.elevacao = V_total / (1000 * A_canoST);           // Calcula a elevação (H) real baseado na quantidade total de água medida e da seção transversal do encanamento
      H_calculado = true;                                     // Sinaliza que a medida foi aferida
    }                                                         //
    dados.corrente = lerSensorCorrente();                     // Armazena a leitura do sensor de corrente ACS712-20A
    dados.som = lerMicrofone();                               // Armazena a leitura do sensor de som
    dados.temperatura = lerSensorTemperatura();               // Armazena a leitura do sensor de temperatura DS18B20
    dados.tensao = lerSensorTensao();                         // Armazena a leitura do sensor de tensão 25VDC
    dados.vazao = lerSensorVazao() * 60;                      // Armazena a vazão real (em L/h)
    mostrardadosSensores(dados);                              // Mostra os dados no Serial Plotter
  }                                                           //
  if (estadoNivelBaixo == HIGH) {                             // Se o sensor de nível baixo estiver fora da água
    digitalWrite(superiorBomba, LOW);                         // Desliga a bomba superior
  }                                                           //
}

/**
 * @brief Função para configurar e inicializar as portas utilizadas no Arduino Pro Mini.
 *
 * Esta função define os modos das portas utilizados no projeto. Ela configura
 * portas como entrada ou saída de acordo com os sensores e atuadores conectados.
 * Também desativa as bombas submersas e superiores inicialmente.
 */
void configurarPortas() {
  pinMode(submersaBomba, OUTPUT);                                                   // Define a porta porta da bomba submersa como saída.
  pinMode(superiorBomba, OUTPUT);                                                   // Define a porta da bomba superior como saída.
  pinMode(nivelAlto, INPUT_PULLUP);                                                 // Define a porta do sensor de nível alto como entrada com pull-up interno.
  pinMode(nivelBaixo, INPUT_PULLUP);                                                // Define a porta do sensor de nível baixo como entrada com pull-up interno.
  pinMode(sensorVazao, INPUT);                                                      // Define a porta do sensor de vazão YF-S201 como entrada.
  digitalWrite(submersaBomba, LOW);                                                 // Força a desativação da bomba submersa.
  digitalWrite(superiorBomba, LOW);                                                 // Força a desativação da bomba superior.
  attachInterrupt(digitalPinToInterrupt(sensorVazao), incrementarPulsos, FALLING);  // Ativa a interrupção na porta do sensor de vazão YF-S201.
}

/**
 * @brief Função de tratamento de evento de requisição I2C que envia dados pré-coletados.
 *
 * Esta função é chamada quando uma requisição I2C é recebida. Em vez de coletar dados,
 * ela recebe uma estrutura de dados previamente preenchida com leituras de vários sensores.
 * A função então envia esses dados pela interface I2C para o solicitante.
 *
 * @param dados Uma referência constante à estrutura de dados `dadosSensores` que contém
 *              as leituras dos sensores a serem enviadas.
 */
void eventoRequisicao(const dadosSensores& dados) {        //
  Wire.write((const char*)&dados, sizeof(dadosSensores));  // Envia a estrutura de dados preenchida pela interface I2C, byte a byte.
}

/**
 * @brief Função para exibir os dados das constantes armazenadas na memória PROGMEM.
 *
 * Esta função lê uma string armazenada na memória flash (PROGMEM) e a converte
 * em uma string do tipo String do Arduino. A memória PROGMEM é usada para armazenar
 * dados constantes fora da RAM, que é limitada.
 *
 * @param constPROGMEM Ponteiro para a string constante armazenada na memória PROGMEM.
 * @return A string completa convertida (tipo String).
 */
String exibirConstPROGMEM(const char* constPROGMEM) {  //
  String resultado;                                    // Inicializa uma string vazia para armazenar o resultado.
  char c;                                              // Variável para armazenar o caractere atual lido da PROGMEM.
  while ((c = pgm_read_byte(constPROGMEM++))) {        // Lê cada caractere da memória PROGMEM até encontrar o caractere nulo '\0'.
    resultado += c;                                    // Adiciona o caractere atual à string resultado.
  }                                                    //
  return resultado;                                    // Retorna a string completa.
}

/**
 * @brief Função para exibir o endereço do sensor de temperatura DS18B20.
 *
 * Esta função recebe o endereço de um sensor de temperatura DS18B20 e exibe
 * o endereço em formato hexadecimal na porta serial. É útil para identificar
 * individualmente cada sensor conectado ao barramento.
 *
 */
void idSensorTemperatura() {                                      //
  for (uint8_t i = 0; i < 8; i++) {                               // Loop para iterar por todos os bytes do endereço do sensor.
    if (enderecoSensorTemperatura[i] < 16) Serial.print(F("0"));  // Adiciona zeros à esquerda se o valor hexadecimal for menor que 16 (0x10).
    Serial.print(enderecoSensorTemperatura[i], HEX);              // Exibe o valor hexadecimal do byte atual do endereço do sensor.
  }                                                               //
}

/**
 * @brief Função de interrupção para incrementar a contagem de pulsos.
 *
 * Esta função é chamada sempre que ocorre uma interrupção na porta configurado para o sensor de vazão YF-S201.
 * Cada interrupção representa um pulso do sensor de vazão YF-S201, e a função simplesmente incrementa a variável
 * global de contagem de pulsos.
 *
 * @note Esta função deve ser configurada como uma rotina de serviço de interrupção (ISR). ISRs devem ser
 *       curtas e rápidas para evitar bloquear outras interrupções.
 */
void incrementarPulsos() {  //
  pulsos++;                 // Incrementa a variável global de contagem de pulsos.
}

/**
* @brief Função para inicializar o protocolo de comunicação I2C.
*
* Esta função configura o barramento I2C com um endereço específico e registra
* a função de tratamento de evento de solicitação I2C. Isso permite que o dispositivo
* atue como um escravo I2C, respondendo a solicitações do mestre (Placa Principal).
*/
void inicializarI2C() {              //
  Wire.begin(8);                     // Inicializa o barramento I2C com endereço 8.
  Wire.onRequest(eventoRequisicao);  // Registra a função de evento de solicitação I2C.
};

/**
 * @brief Função para inicializar e verificar os sensores de temperatura DS18B20.
 *
 * Esta função inicia a comunicação com todos os sensores de temperatura DS18B20 conectados ao barramento One-Wire.
 * Ela verifica se os sensores estão funcionando corretamente, obtém os endereços de todos os sensores e Exibe
 * a quantidade de sensores encontrados e seus endereços. Se nenhum sensor for encontrado, ela informa o erro.
 *
 * @return Número de sensores de temperatura DS18B20 encontrados.
 */
void inicializarSensorTemperatura() {                                  //
  sensorTemperatura.begin();                                           // Inicia a comunicação com os sensores de temperatura.
  Serial.println(F("Localizando sensor(es) DS18B20..."));              // Exibe uma mensagem indicando o início da localização dos sensores.
  sensorTemperatura.requestTemperatures();                             // Envia o comando para todos os sensores começarem a conversão de temperatura.
  int numSensores = sensorTemperatura.getDeviceCount();                // Obtém o número de sensores DS18B20 encontrados.
  Serial.print(numSensores);                                           // Exibe o número de sensores encontrados.
  Serial.println(F(" sensor(es) DS18B20 encontrado(s)."));             // Exibe uma mensagem indicando o fim da localização dos sensores.
  for (int i = 0; i < numSensores; i++) {                              // Itera por todos os sensores encontrados.
    if (sensorTemperatura.getAddress(enderecoSensorTemperatura, i)) {  // Verifica se o endereço do sensor pode ser obtido.
      Serial.print(F("DS18B20 ("));                                    // Exibe o nome do sensor.
      Serial.print(i + 1);                                             // Exibe o número do sensor.
      Serial.print(F(") - Endereço: "));                               // Exibe uma mensagem indicando que o endereço do sensor será impresso a seguir.
      for (uint8_t j = 0; j < 8; j++) {                                // Itera por todos os bytes do endereço do sensor.
        if (enderecoSensorTemperatura[j] < 16) Serial.print(F("0"));   // Adiciona um zero à frente se o byte for menor que 16, para manter o formato hexadecimal.
        Serial.print(enderecoSensorTemperatura[j], HEX);               // Exibe o byte do endereço do sensor em formato hexadecimal.
      }                                                                //
      Serial.println();                                                // Pula para a próxima linha após imprimir o endereço do sensor.
    } else {                                                           // Caso o endereço do sensor não possa ser obtido.
      Serial.print(F("DS18B20 ("));                                    // Exibe o nome do sensor.
      Serial.print(i + 1);                                             // Exibe o número do sensor.
      Serial.println(F(") - ERRO: sensor não encontrado"));            // Exibe uma mensagem de erro indicando que o sensor não foi encontrado.
    }                                                                  //
  }                                                                    //
}

/**
 * @brief Função para inicializar a comunicação serial.
 *
 * Esta função configura a comunicação serial com uma taxa de transmissão de 57600 bps.
 * Após inicializar a comunicação, ela Exibe informações acadêmicas (curso, módulo,
 * registro acadêmico e nome do aluno) na porta serial.
 */
void inicializarSerial() {                     //
  Serial.begin(57600);                         // Inicializa a comunicação serial com a taxa de transmissão de 57600 bps.
  Serial.println(exibirConstPROGMEM(curso));   // Exibe o nome do curso na porta serial, convertendo a constante PROGMEM para uma String.
  Serial.println(exibirConstPROGMEM(modulo));  // Exibe o número do módulo na porta serial, convertendo a constante PROGMEM para uma String.
  Serial.println(exibirConstPROGMEM(ra));      // Exibe o número de registro acadêmico (RA) na porta serial, convertendo a constante PROGMEM para uma String.
  Serial.println(exibirConstPROGMEM(aluno));   // Exibe o nome do aluno na porta serial, convertendo a constante PROGMEM para uma String.
  Serial.println();                            // Exibe uma linha em branco na porta serial para separar as informações.
}

/**
 * @brief Função para ler o sensor de corrente ACS712-20A.
 *
 * Esta função lê o valor analógico do sensor de corrente ACS712-20A, converte o valor
 * para uma tensão correspondente e, em seguida, calcula a corrente elétrica em amperes.
 *
 * @return A corrente elétrica medida pelo sensor, em amperes.
 */
float lerSensorCorrente() {                                //
  const float sensibilidade = 0.1;                         // Sensibilidade do ACS712-20A é 100mV por Ampere (0.1V/A).
  const int offset = 512;                                  // Offset de leitura do sensor no centro, geralmente 512 para 5V e 10-bit ADC.
  int valorSensor = analogRead(sensorCorrente);            // Lê o valor do sensor de corrente ACS712-20A (valor entre 0 e 1023).
  float tensao = (valorSensor - offset) * (5.0 / 1024.0);  // Converte o valor ADC para tensão (em volts).
  float corrente = tensao / sensibilidade;                 // Converte a tensão em corrente (em amperes).
  return corrente;                                         // Retorna a corrente medida.
}

/**
 * @brief Função para ler a temperatura em graus Celsius do sensor de temperatura DS18B20.
 *
 * Esta função recebe um objeto DallasTemperature como parâmetro, solicita a leitura da temperatura
 * do sensor de temperatura DS18B20 e retorna o valor em graus Celsius.
 *
 * @return A temperatura medida pelo sensor, em graus Celsius.
 */
float lerSensorTemperatura() {                               //
  sensorTemperatura.requestTemperatures();                   // Solicita a leitura da temperatura do sensor.
  float temperatura = sensorTemperatura.getTempCByIndex(0);  // Obtém a temperatura em graus Celsius do sensor.
  return temperatura;                                        // Retorna a temperatura medida.
}

/**
 * @brief Função para ler o sensor de tensão 25VDC.
 *
 * Esta função lê o valor analógico do sensor de tensão 25VDC, converte o valor para uma
 * tensão correspondente considerando o divisor de tensão utilizado.
 *
 * @return A tensão elétrica medida pelo sensor, em volts.
 */
float lerSensorTensao() {                                     //
  const float divisorTensao = 5.0 / 1024.0;                   // Conversão de ADC para tensão (assumindo 5V e 10-bit ADC).
  const float razaoDivisor = 5.0;                             // Razão do divisor de tensão (depende do sensor, frequentemente 5).
  int valorSensor = analogRead(sensorTensao);                 // Lê o valor do sensor de tensão 25VDC (valor entre 0 e 1023).
  float tensao = valorSensor * divisorTensao * razaoDivisor;  // Calcula a tensão real usando o divisor de tensão.
  return tensao;                                              // Retorna a tensão medida.
}

/**
 * @brief Função para ler o sensor de vazão YF-S201.
 *
 * Esta função calcula a vazão de água (L/min) com base na contagem de pulsos do sensor de vazão YF-S201. A função é chamada periodicamente e 
 * utiliza interrupções para contar os pulsos gerados pelo fluxo de água.
 *
 * @return A vazão de água medida pelo sensor, em litros por minuto.
 */
float lerSensorVazao() {                                                             //
  static unsigned long ultimoTempo = 0;                                              // Variável estática para armazenar o último tempo de leitura.
  static float vazao = 0;                                                            // Variável estática para armazenar a vazão calculada.
  unsigned long tempoAtual = millis();                                               // Obtém o tempo atual em milissegundos.
  unsigned long intervaloTempo = tempoAtual - ultimoTempo;                           // Calcula o intervalo de tempo desde a última leitura.
  if (intervaloTempo >= 1000) {                                                      // Atualiza a cada 1 segundo.
    detachInterrupt(digitalPinToInterrupt(sensorVazao));                             // Desabilita a interrupção temporariamente.
    vazao = ((float)pulsos / Vazao_fator) / (intervaloTempo / 1000.0);               // Calcula a vazão em L/min com base na contagem de pulsos e no intervalo de tempo.
    pulsos = 0;                                                                      // Reseta o contador de pulsos.
    ultimoTempo = tempoAtual;                                                        // Reseta o contador de tempo.
    attachInterrupt(digitalPinToInterrupt(sensorVazao), incrementarPulsos, RISING);  // Reabilita a interrupção para contar os pulsos do sensor de vazão YF-S201.
  }                                                                                  //
  return vazao;                                                                      // Retorna a vazão medida.
}

/**
 * @brief Função para ler o valor de volume de som do sensor.
 *
 * Esta função lê repetidamente os valores analógicos do microfone MAX9814
 * durante um período de amostragem especificado. Ela calcula a diferença pico a pico
 * entre os valores máximos e mínimos lidos e converte essa diferença para uma
 * tensão correspondente.
 *
 * @return A tensão correspondente ao valor de pico a pico do sinal de som, em volts.
 */
float lerMicrofone() {                                      //
  const int janelaAmostragem = 50;                          // Define a janela de amostragem em milissegundos.
  unsigned long inicioAmostragem = millis();                // Marca o início do período de amostragem.
  unsigned int picoPico = 0;                                // Inicializa a variável de diferença pico a pico.
  unsigned int sinalMaximo = 0;                             // variável para armazenar o valor máximo do sinal.
  unsigned int sinalMinimo = 1023;                          // variável para armazenar o valor mínimo do sinal.
  while (millis() - inicioAmostragem < janelaAmostragem) {  // Continua amostrando até que a janela de amostragem tenha expirado.
    int amostra = analogRead(microfone);                    // Lê o valor analógico do microfone MAX9814.
    if (amostra < 1023) {                                   // Se a amostra lida for válida (menor que 1023)
      if (amostra > sinalMaximo) {                          // Se a amostra for maior que o valor máximo atual
        sinalMaximo = amostra;                              // Atualiza o valor máximo do sinal
      } else if (amostra < sinalMinimo) {                   // Se a amostra for menor que o valor mínimo atual
        sinalMinimo = amostra;                              // Atualiza o valor mínimo do sinal
      }                                                     //
    }                                                       //
  }                                                         //
  picoPico = sinalMaximo - sinalMinimo;                     // Calcula a diferença entre os valores máximo e mínimo (pico a pico).
  float tensao = (picoPico * 5.0) / 1024.0;                 // Converte a diferença pico a pico para volts. O ADC do Arduino tem uma resolução de 10 bits (0-1023), e a tensão de referência é de 5V.
  return tensao;                                            // Retorna a tensão calculada.
}

/**
 * @brief Função para exibir os dados dos sensores no Serial Plotter.
 *
 * Esta função exibe os valores dos sensores na porta serial, formatados para
 * serem visualizados no Serial Plotter do Arduino IDE. Os dados são impressos
 * em uma linha, separados por tabulações, utilizando um loop para reduzir a redundância.
 *
 * @param dados Estrutura contendo os valores lidos dos sensores.
 */
void mostrardadosSensores(const dadosSensores& dados) {                                                                       //
  const float* campos[] = { &dados.corrente, &dados.elevacao, &dados.tensao, &dados.vazao, &dados.som, &dados.temperatura };  // Array de ponteiros para os membros da struct.
  const char* nomesCampos[] = { "I: ", "L: ", "V: ", "Q: ", "S: ", "T: " };                                                   // Array de strings para os nomes dos campos.
  const int numCampos = sizeof(campos) / sizeof(campos[0]);                                                                   // Contagem de campos na struct.
  for (int i = 0; i < numCampos; ++i) {                                                                                       // Loop para exibir os valores dos campos.
    Serial.print(nomesCampos[i]);                                                                                             // Exibe o nome do campo.
    Serial.print(*campos[i]);                                                                                                 // Exibe o valor do campo.
    Serial.print(F("\t"));                                                                                                    // Exibe uma tabulação.
  }                                                                                                                           //
}