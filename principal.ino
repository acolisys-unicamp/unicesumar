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

// Definição das bibliotecas necessárias
#include <EtherCard.h>          // Inclui a biblioteca EtherCard para comunicação Ethernet (ENC28J60)
#include <LiquidCrystal_I2C.h>  // Inclui a biblioteca LiquidCrystal_I2C para controle de displays LCD via I2C
#include "RTClib.h"             // Inclui a biblioteca RTClib para manipulação do módulo RTC DS1307
#include <SPI.h>                // Inclui a biblioteca SPI para comunicação serial (full duplex) - arquitetura Mestre - Escravo
#include <Wire.h>               // Inclui a biblioteca Wire para comunicação I2C

// Definição dos pinos utilizados.
#define alarme 3  // Alarme (buzzer) para sinalização de episódios de Cavitação

// Definição das informações acadêmicas
const char curso[] PROGMEM = "UNICESUMAR - E SOFT";   // Instituição e curso
const char modulo[] PROGMEM = "202452 - TCC II";      // Módulo - Trabalho
const char ra[] PROGMEM = "RA 16091425";              // Registro Acadêmico (RA)
const char aluno[] PROGMEM = "ALEXANDRE C OLIVEIRA";  // Nome do Aluno

// Definição de constantes globais
const float gravidade = 9.81;  // Aceleração da gravidade, em m/s²

// Definição das variáveis globais
// Parâmetros de rede
boolean conexaoRede = true;                                          // Indicador de conexão de rede
byte Ethernet::buffer[700];                                          // Buffer Ethernet (ENC28J60)
static byte enderecoMAC[] = { 0x74, 0x69, 0x69, 0x2D, 0x30, 0x31 };  // Endereço MAC da placa Ethernet (ENC28J60)
long fusoHorario = -10800L;                                          // Fuso horário (-3 horas)
static byte servidorNTP[] = { 129, 6, 15, 28 };                      // Endereço do servidor NTP
uint8_t portaNTP = 123;                                              // Porta do servidor NTP

// Parâmetros fornecidos pelo usuário
float F_cavitacao = 0;   // Fator de cavitação, adimensional
float H_maxima = 0;      // Elevação máxima do motor da bomba, em m
float P_maxima = 0.001;  // Potência elétrica máxima do motor da bomba, em W; inicializada com o valor 0.001 para prevenção da divisão por 0 (zero)
float Q_maxima = 0;      // Vazão máxima da bomba, em L/h
float V_maxima = 0;      // Tensão elétrica máxima do motor da bomba, em V

// Definição da estrutura de dados para armazenamento das leituras dos sensores
struct dadosSensores {     //
  float corrente = 0;      // Corrente elétrica medida, em A
  float elevacao = 0.001;  // Elevação (H) real, em m; ; inicializada com o valor 0.001 para prevenção da divisão por 0 (zero)
  float som = 0;           // Volume de som medido, em V
  float temperatura = 0;   // Temperatura medida, em °C
  float tensao = 0;        // Tensão elétrica medida, em V
  float vazao = 0;         // Vazão medida, em L/h
};                         //
dadosSensores dados;       // Instância da estrutura de dados dos sensores

/**
 * @brief Definição do formulário HTML.
 *
 * Esta constante contém o código HTML do formulário web usado para enviar os parâmetros nominais do motor da bomba.
 * A string HTML é armazenada na memória flash usando a palavra-chave PROGMEM para economizar espaço na memória RAM.
 * A macro R"=====(...)===" é usada para criar uma string bruta, o que permite que a string contenha aspas e barras invertidas.
 */
const char formularioHTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang='pt-BR'>
	<head>
		<meta charset='UTF-8'>
		<title>UNICESUMAR - Engenharia de Software</title>
		<script>
			window.onload = function() {
				const entradas = document.querySelectorAll('input[type=text]');
				const botao = document.getElementById('salvarParametros');
				botao.disabled = true;
				
				function verificarEntradas() {
					const todosPreenchidos = Array.from(entradas).every(input => input.value.match(/^\d+(,\d{1,2})?$/));
					botao.disabled = !todosPreenchidos;
				}
				
				function converterParaFloat(valor) {
					return parseFloat(valor.replace(',', '.'));
				}
				
				function enviarFormulario(event) {
					event.preventDefault();
					const dadosConvertidos = {};
					entradas.forEach(input => {
						dadosConvertidos[input.name] = converterParaFloat(input.value);
					});
					console.log(dadosConvertidos); // Aqui você pode substituir por código para enviar os dados via AJAX ou outra forma
				}
				
				entradas.forEach(input => input.addEventListener('input', verificarEntradas));
				document.getElementById('formularioHTML').addEventListener('submit', enviarFormulario);
				verificarEntradas();
			};
		</script>
	</head>
	<body>
		<div>
			<form id='formularioHTML' method='post' action=''>
				<h2>UNICESUMAR - Engenharia de Software</h2><p>
					Módulo 202452 - TCC II<br>
				RA 16091425 - ALEXANDRE CARLOS DE OLIVEIRA</p>
				<b>Informe os parâmetros nominais do motor da bomba</b>
				<ul>
					<li>
						<label for='H_maxima'>Elevação máxima (m)</label><br>
						<input id='H_maxima' name='H_maxima' type='text' maxlength='255' required>
					</li>
					<li>
						<label for='P_maxima'>Potência elétrica máxima (W)</label><br>
						<input id='P_maxima' name='P_maxima' type='text' maxlength='255' required>
					</li>
					<li>
						<label for='Q_maxima'>Vazão máxima (L/h)</label><br>
						<input id='Q_maxima' name='Q_maxima' type='text' maxlength='255' required>
					</li>
					<li>
						<label for='V_maxima'>Tensão elétrica máxima (V)</label><br>
						<input id='V_maxima' name='V_maxima' type='text' maxlength='255' required>
					</li>
				</ul>
				<br>
				<button id='salvarParametros' type='submit'>Salvar</button>
			</form>
		</div>
	</body>
</html>
)=====";

/**
 * @brief Definição da página web para demonstração dos resultados obtidos.
 *
 * Esta constante contém o código HTML de uma página web usada para exibir os resultados obtidos pelo sistema.
 * Os resultados são atualizados automaticamente a cada 150 milissegundos usando JavaScript.
 * A string HTML é armazenada na memória flash usando PROGMEM para economizar espaço na memória RAM.
 * A macro R"=====(...)===" é usada para criar uma string bruta, o que permite que a string contenha aspas e barras invertidas.
 */
const char resultadosHTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang='pt-BR'>
	<head>
		<meta charset='UTF-8'>
		<title>UNICESUMAR - Engenharia de Software</title>
		<h2>UNICESUMAR - Engenharia de Software</h2>
		<p>Módulo 202452 - TCC II<br> RA 16091425 - ALEXANDRE CARLOS DE OLIVEIRA</p>
		<script>
			function atualizarDados() {
				fetch('/atualizar')
				.then(response => response.json())
				.then(data => {
					document.getElementById('H').innerText = data.H + ' m';
					document.getElementById('P').innerText = data.P + ' W';
					document.getElementById('Q').innerText = data.Q + ' L/h';
					document.getElementById('T').innerText = data.T + ' C';
					document.getElementById('F').innerText = data.F;
					
					const statusElement = document.getElementById('S');
					statusElement.innerText = data.S;
					
					if (data.S === 'NORMAL') {
						statusElement.style.backgroundColor = 'lightgreen';
						statusElement.style.fontWeight = 'normal';
						statusElement.style.color = 'black';
						} else if (data.S === 'NO LIMITE') {
						statusElement.style.backgroundColor = 'yellow';
						statusElement.style.fontWeight = 'bold';
						statusElement.style.color = 'black';
						} else if (data.S === 'CAVITANDO') {
						statusElement.style.backgroundColor = 'red';
						statusElement.style.fontWeight = 'bold';
						statusElement.style.color = 'white';
						} else {
						statusElement.style.backgroundColor = 'transparent';
						statusElement.style.fontWeight = 'normal';
						statusElement.style.color = 'black';
					}
				});
			}
			setInterval(atualizarDados, 150);
		</script>
	</head>
	<body onload="atualizarDados()">
		<b>Status da bomba (S)</b><br>
		Referências: <b>NORMAL</b> (F > 1); <b>NO LIMITE</b> (F = 1); <b>CAVITANDO</b> (F < 1)
		<ul>
			<li>H: <span id='H'></span></li>
			<li>P: <span id='P'></span></li>
			<li>Q: <span id='Q'></span></li>
			<li>T: <span id='T'></span></li>
			<li>F: <span id='F'></span></li>
			<li>S: <span id='S'></span></li>
		</ul>
	</body>
</html>
)=====";

// Definição da instância do módulo LCD I2C (20x4)
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Objeto para controle do display LCD I2C (20x4)

// Definição da instância do módulo RTC DS1307
RTC_DS1307 rtc;  // Objeto para controle do módulo RTC DS1307

// Definição dos protótipos das funções
void atenderRequisicoesEthernet();                                                                                 // Função para responder as requisições ao servidor web (Ethernet)
void calcularDados(dadosSensores dados);                                                                           // Função para calcular os dados dos sensores
float calcularDensidadeAgua(float temperatura);                                                                    // Função para calcular a densidade aproximada da água em função da temperatura, em kg/m³
float calcularElevacaoEstimada(float P_maxima, float H_maxima, float P_real);                                      // Função para calcular a elevação estimada, em m
float calcularFatorCavitacao(float elevacao, float H_estimada, float Q_real, float Q_estimada, float eficiencia);  // Função para calcular o fator de cavitação
float calcularPotenciaHidraulica(float densidade, float gravidade, float Q_maxima, float H_maxima);                // Função para calcular a potência hidráulica da bomba, em W
float calcularVazaoEstimada(float P_maxima, float Q_maxima, float P_real);                                         // Função para calcular a vazão estimada, em L/h
float converterLH_M3S(float vazaoLH);                                                                              // Função para converter a vazão em L/h para m³/s
void enviarDadosJSON();                                                                                            // Função para enviar dados atualizados em formato JSON
void enviarHTML(const char* html, size_t tamanho);                                                                 // Função para enviar dados HTML para o cliente.
String exibirConstPROGMEM(const char* constPROGMEM);                                                               // Função para exibir os dados das constantes PROGMEM
void inicializarLCD();                                                                                             // Função para inicializar o display LCD I2C (20x4)
void inicializarRede();                                                                                            // Função para inicializar a rede Ethernet (ENC28J60) e mostrar o endereço MAC
void inicializarRTC();                                                                                             // Função para inicializar o módulo RTC DS1307
void inicializarSerial();                                                                                          // Função para inicializar a comunicação serial
void mostrarDadosLCD(int linha, int coluna, String grandeza, float valor, String unidade);                         // Função para mostrar os dados no LCD I2C (20x4)
String mostrarDataHora();                                                                                          // Função para mostrar a data e hora
void processarFormularioHTML(uint8_t status, uint16_t off, uint8_t* buffer);                                       // Função para processar os dados do formulário
void requisitarLeituras(dadosSensores& dados);                                                                     // Função para requisitar os dados dos sensores
unsigned long requisitarNTP();                                                                                     // Função para fazer a requisição NTP
String statusCavitacao(float F_cavitacao);                                                                         // Função para determinar o status da bomba

/**
 * @brief Função de configuração inicial do Arduino.
 *
 * Esta função é executada uma vez quando o Arduino é ligado ou reiniciado. Ela
 * inicializa a comunicação serial, configura os módulos de display e RTC DS1307,
 * configura a rede Ethernet (ENC28J60) e limpa o display LCD I2C (20x4).
 */
void setup() {                                                //
  pinMode(alarme, OUTPUT);                                    // Define o pino do alarme (buzzer) como saída.
  digitalWrite(alarme, LOW);                                  // Força a desativação inicial do alarme.
  inicializarSerial();                                        // Inicializa a comunicação serial
  inicializarLCD();                                           // Inicializa o display LCD I2C (20x4)
  inicializarRede();                                          // Inicializa a rede Ethernet (ENC28J60) e mostrar o endereço MAC
  inicializarRTC();                                           // Inicializa o módulo RTC DS1307
  Serial.println(F("Inicialização concluída com sucesso."));  // Mensagem de depuração na porta serial.
  Serial.println();                                           //
  lcd.clear();                                                // Limpa o display após inserção dos dados
}

/**
 * @brief Função principal de execução contínua do Arduino.
 *
 * Esta função é chamada repetidamente durante a execução do programa. Ela
 * responde às requisições do servidor web, requisita leituras dos sensores,
 * realiza cálculos necessários e exibe os dados no display LCD I2C (20x4).
 */
void loop() {                                                        //
  atenderRequisicoesEthernet();                                      // Responde às requisições ao servidor web.
  requisitarLeituras(dados);                                         // Requisita os dados dos sensores.
  calcularDados(dados);                                              // Realiza os cálculos necessários.
  mostrarDadosLCD(0, 0, mostrarDataHora(), 0, "");                   // Exibe Data e Hora na linha 0, coluna 0.
  mostrarDadosLCD(1, 0, "H:", dados.elevacao, "m");                  // Exibe a elevação real na linha 1, coluna 0.
  mostrarDadosLCD(1, 12, "P:", dados.corrente * dados.tensao, "W");  // Exibe a potência real na linha 1, coluna 12.
  mostrarDadosLCD(2, 0, "Q:", dados.vazao, "L/h");                   // Exibe a vazão real na linha 2, coluna 0.
  mostrarDadosLCD(2, 12, "T:", dados.temperatura, "C");              // Exibe a temperatura na linha 2, coluna 12.
  mostrarDadosLCD(3, 0, "F:", F_cavitacao, "");                      // Exibe o fator de cavitação na linha 3, coluna 0.
  mostrarDadosLCD(3, 12, "S:", 0, statusCavitacao(F_cavitacao));     // Exibe o status da bomba na linha 3, coluna 12.
}

/**
 * @brief Função para responder as requisições ao servidor web.
 *
 * Esta função é responsável por processar as requisições recebidas pelo servidor web (Ethernet). 
 * Ela verifica o tipo de requisição e responde de acordo com a solicitação, seja 
 * enviando uma página de dados, atualizando os dados em formato JSON ou respondendo 
 * a outras requisições com o formulário HTML.
 */
void atenderRequisicoesEthernet() {                                            //
  ether.packetLoop(ether.packetReceive());                                     // Executa o loop para processar os pacotes de rede recebidos.
  if (ether.packetReceive()) {                                                 // Verifica se há um pacote recebido.
    if (strncmp((char*)ether.buffer + 9, "dados", 5) == 0) {                   // Verifica se a requisição é para "dados".
      enviarHTML(resultadosHTML, sizeof(resultadosHTML));                      // Envia a página de dados ao cliente.
    } else if (strncmp((char*)ether.buffer + 9, "atualizar", 9) == 0) {        // Verifica se a requisição é para "atualizar".
      enviarDadosJSON();                                                       // Atualiza os dados em formato JSON e os envia ao cliente.
    } else {                                                                   //
      ether.browseUrl(PSTR("/"), "", ether.hisport, processarFormularioHTML);  // Responde a outras requisições com o formulário HTML.
    }                                                                          //
  }                                                                            //
}

/**
 * @brief Função para realizar os cálculos necessários.
 *
 * Esta função realiza diversos cálculos com base nos dados dos sensores,
 * incluindo a elevação estimada, vazão estimada, potência hidráulica, 
 * e o fator de cavitação. 
 *
 * @param dados Estrutura contendo os dados dos sensores.
 */
void calcularDados(dadosSensores dados) {
  float H_estimada = 0;                                                                                               // Elevação estimada, em m
  float Q_estimada = 0.001;                                                                                           // Vazão estimada, em L/h; inicializada com o valor 0.001 para prevenção da divisão por 0 (zero)                                                                        //
  float Ph_estimada = 0;                                                                                              // Potência hidráulica, em W
  H_estimada = calcularElevacaoEstimada(P_maxima, dados.elevacao, dados.corrente * dados.tensao);                     // Calcula a elevação estimada.
  Q_estimada = calcularVazaoEstimada(P_maxima, Q_maxima, dados.corrente * dados.tensao);                              // Calcula a vazão estimada.
  Ph_estimada = calcularPotenciaHidraulica(calcularDensidadeAgua(dados.temperatura), gravidade, Q_maxima, H_maxima);  // Calcula a potência hidráulica.
  F_cavitacao = calcularFatorCavitacao(dados.elevacao, H_estimada, dados.vazao, Q_estimada, Ph_estimada / P_maxima);  // Calcula o fator de cavitação, adimensional.
}

/**
 * @brief Função para processar os dados do formulário HTML.
 *
 * Esta função é responsável por processar os dados enviados através de um formulário HTML.
 * Ela extrai os parâmetros do formulário contidos no buffer e os armazena em variáveis globais.
 * Após o processamento dos dados, o usuário é redirecionado para a página de exibição de dados.
 *
 * @param status O status da requisição HTTP.
 * @param off O offset no buffer onde os dados começam.
 * @param buffer O buffer contendo os dados do formulário HTML.
 */
void processarFormularioHTML(uint8_t status, uint16_t off, uint8_t* buffer) {                                  //
  if (status == 0) {                                                                                           // Verifica se o status da requisição é igual a zero, indicando sucesso
    H_maxima = atof(strstr((char*)buffer + off, "H_maxima=") + 10);                                            // Extrai o valor de H_maxima e converte para float.
    P_maxima = atof(strstr((char*)buffer + off, "P_maxima=") + 10);                                            // Extrai o valor de P_maxima e converte para float.
    Q_maxima = atof(strstr((char*)buffer + off, "Q_maxima=") + 10);                                            // Extrai o valor de Q_maxima e converte para float.
    V_maxima = atof(strstr((char*)buffer + off, "V_maxima=") + 10);                                            // Extrai o valor de V_maxima e converte para float.
    ether.httpServerReplyAck();                                                                                // Confirma o recebimento da requisição HTTP.
    ether.browseUrl(PSTR("/dados"), "", ether.hisport, [](uint8_t status, uint16_t off, uint8_t* buffer) {});  // Redireciona para a página de exibição de dados.
  }
}

/**
 * @brief Função para calcular a densidade aproximada da água em função da temperatura.
 *
 * Esta função calcula de forma aproximada a densidade da água, em kg/m³
 * com base na temperatura fornecida em graus Celsius.
 *
 * @param temperatura A temperatura da água em graus Celsius.
 * @return A densidade aproximada da água, em kg/m³.
 */
float calcularDensidadeAgua(float temperatura) {  //
  const float A = 999.84;                         // Coeficiente aproximado por método de Regressão Não Linear (Exponencial).
  const float B = -0.0003;                        // Coeficiente aproximado por método de Regressão Não Linear (Exponencial).
  return A * exp(B * temperatura);                // Calcula e retorna a densidade aproximada da água, em kg/m³.
}

/**
 * @brief Função para calcular a elevação estimada.
 *
 * Esta função estima a elevação (em metros) com base na potência máxima,
 * elevação máxima, e potência real fornecida.
 *
 * @param P_maxima A potência elétrica máxima da bomba, em watts.
 * @param H_maxima A elevação máxima alcançada pela bomba, em metros.
 * @param P_real A potência elétrica real, em watts.
 * @return A elevação estimada, em metros.
 */
float calcularElevacaoEstimada(float P_maxima, float H_maxima, float P_real) {  //
  return exp(log(H_maxima + 1) / P_maxima * P_real) - 1;                        // Calcula e retorna a elevação estimada.
}

/**
 * @brief Função para calcular o fator de cavitação.
 *
 * Esta função calcula o fator de cavitação com base na elevação real e estimada,
 * vazão real e estimada, e eficiência da bomba.
 *
 * @param elevacao A elevação real medida, em metros.
 * @param H_estimada A elevação estimada calculada, em metros.
 * @param Q_real A vazão real medida, em litros por hora.
 * @param Q_estimada A vazão estimada calculada, em litros por hora.
 * @param eficiencia A eficiência do sistema, em percentual.
 * @return O fator de cavitação calculado, adimensional.
 */
float calcularFatorCavitacao(float elevacao, float H_estimada, float Q_real, float Q_estimada, float eficiencia) {  //
  return (H_estimada / elevacao * eficiencia) * (Q_real / Q_estimada);                                              // Calcula e retorna o fator de cavitação.
}

/**
 * @brief Função para calcular a potência hidráulica da bomba.
 *
 * Esta função calcula a potência hidráulica com base na densidade da água,
 * aceleração da gravidade, vazão máxima e elevação máxima.
 *
 * @param densidade A densidade da água, em kg/m³.
 * @param gravidade A aceleração da gravidade, em m/s².
 * @param Q_maxima A vazão máxima da bomba, em L/h.
 * @param H_maxima A elevação máxima da bomba, em m.
 * @return A potência hidráulica, em W.
 */
float calcularPotenciaHidraulica(float densidade, float gravidade, float Q_maxima, float H_maxima) {  //
  return densidade * gravidade * converterLH_M3S(Q_maxima) * H_maxima;                                // Retorna o cálculo da potência hidráulica (informação: vazão máxima convertida em m³/s).
}

/**
 * @brief Função para calcular a vazão estimada.
 *
 * Esta função estima a vazão com base na potência máxima, vazão máxima,
 * e potência real fornecida.
 *
 * @param P_maxima A potência máxima da bomba, em W.
 * @param Q_maxima A vazão máxima da bomba, em L/h.
 * @param P_real A potência real medida, em W.
 * @return A vazão estimada, em L/h.
 */
float calcularVazaoEstimada(float P_maxima, float Q_maxima, float P_real) {  //
  return exp(log(Q_maxima + 1) / P_maxima * P_real) - 1;                     // Calcula e retorna a vazão estimada.
}

/**
 * @brief Função para converter a vazão de L/h para m³/s.
 *
 * Esta função converte a vazão medida em litros por hora (L/h)
 * para metros cúbicos por segundo (m³/s).
 *
 * @param vazaoLH A vazão referencial, em L/h.
 * @return A vazão convertida, em m³/s.
 */
float converterLH_M3S(float vazaoLH) {  //
  return vazaoLH / 1000.0 / 3600.0;     // Retorna a vazão convertida.
}

/**
 * @brief Função para enviar dados atualizados em formato JSON.
 *
 * Esta função formata os dados coletados em um objeto JSON e os envia
 * como resposta HTTP ao cliente.
 */
void enviarDadosJSON() {                                                                                                               //
  char resposta[256];                                                                                                                  // Buffer para armazenar a resposta JSON.
  snprintf(resposta, sizeof(resposta), "{\"H\":%.2f,\"P\":%.2f,\"Q\":%.2f,\"T\":%.2f,\"F\":%.2f,\"S\":\"%s\"}",                        //
           dados.elevacao, dados.corrente * dados.tensao, dados.vazao, dados.temperatura, F_cavitacao, statusCavitacao(F_cavitacao));  // Formata os dados em um objeto JSON.
  ether.httpServerReply(resposta);                                                                                                     // Envia a resposta JSON ao cliente.
}

/**
 * @brief Função para enviar dados HTML para o cliente.
 *
 * Esta função percorre cada caractere da string HTML armazenada na memória flash
 * e a envia ao cliente via interface Ethernet (ENC28J60).
 *
 * @param html Uma constante do tipo char que representa a string HTML a ser enviada.
 * @param tamanho O tamanho da string HTML.
 */
void enviarHTML(const char* html, size_t tamanho) {       //
  ether.httpServerReplyAck();                             // Envia uma resposta de reconhecimento ao cliente.
  for (size_t i = 0; i < tamanho; ++i) {                  //
    ether.httpServerReply(pgm_read_byte_near(html + i));  // Lê cada byte da memória flash e o envia ao cliente.
  }                                                       //
}

/**
 * @brief Função para exibir os dados das constantes armazenadas na memória PROGMEM.
 *
 * Esta função lê uma string armazenada na memória flash (PROGMEM) e a converte
 * em uma string do tipo String do Arduino. A memória PROGMEM é usada para armazenar
 * dados constantes fora da RAM, que é limitada.
 *
 * @param constPROGMEM Ponteiro para a string constante armazenada na memória PROGMEM.
 * @return A string convertida do tipo String.
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
 * @brief Função para inicializar o display LCD I2C (20x4).
 *
 * Esta função configura e inicializa o display LCD I2C (20x4), ativa a luz de fundo,
 * limpa o display e exibe informações acadêmicas (curso, módulo, registro acadêmico
 * e nome do aluno) nas quatro linhas do display.
 */
void inicializarLCD() {                   //
  Wire.begin();                           // Inicia o barramento I2C como mestre.
  lcd.init();                             // Inicializa o LCD I2C (20x4).
  lcd.backlight();                        // Liga a luz de fundo do LCD I2C (20x4).
  lcd.clear();                            // Limpa o display LCD I2C (20x4).
  lcd.print(exibirConstPROGMEM(curso));   // Exibe o nome do curso na primeira linha do LCD I2C (20x4).
  lcd.setCursor(0, 1);                    // Define a posição do cursor para a segunda linha.
  lcd.print(exibirConstPROGMEM(modulo));  // Exibe o número do módulo.
  lcd.setCursor(0, 2);                    // Define a posição do cursor para a terceira linha.
  lcd.print(exibirConstPROGMEM(ra));      // Exibe o número de registro acadêmico (RA).
  lcd.setCursor(0, 3);                    // Define a posição do cursor para a quarta linha.
  lcd.print(exibirConstPROGMEM(aluno));   // Exibe o nome do aluno.
  delay(5000);                            // Aguarda 5 segundos para permitir a leitura da mensagem.
  lcd.clear();                            // Limpa o display LCD I2C (20x4) após mostrar a mensagem.
}

/**
 * @brief Função para inicializar a rede Ethernet (ENC28J60) e mostrar o endereço MAC.
 *
 * Esta função inicializa a conexão Ethernet (ENC28J60), exibe o endereço MAC
 * e configura o DHCP para obter um endereço IP. Em caso de sucesso na
 * configuração DHCP, serve o formulário HTML e inicia o loop do servidor
 * Ethernet (ENC28J60) para processar as solicitações do cliente.
 */
void inicializarRede() {                                         //
  Serial.print(F("Endereco MAC: "));                             // Exibe o cabeçalho do endereço MAC.
  for (byte i = 0; i < 6; ++i) {                                 // Loop para imprimir cada byte do endereço MAC.
    Serial.print(enderecoMAC[i], HEX);                           // Exibe o byte atual em hexadecimal.
    if (i < 5)                                                   // Se não for o último byte.
      Serial.print(':');                                         // Exibe um separador.
  }                                                              //
  Serial.println();                                              // Exibe uma linha em branco.
  Serial.println();                                              // Exibe uma linha em branco adicional.
  int DHCP = ether.begin(sizeof Ethernet::buffer, enderecoMAC);  // Inicia a configuração DHCP e verifica o status.
  if (DHCP == 0) {                                               // Verifica se a configuração DHCP falhou.
    Serial.println(F("DHCP: Falha na conexão"));                 // Exibe mensagem de falha.
    conexaoRede = false;                                         // Define a flag de conexão de rede como falso.
  } else if (ether.dhcpSetup()) {                                // Se a configuração DHCP foi bem-sucedida.
    Serial.print(F("DHCP: Conectado (IP "));                     // Exibe mensagem de sucesso.
    ether.printIp("", ether.myip);                               // Exibe o endereço IP atribuído via DHCP.
    Serial.println(F(")"));                                      //
    enviarHTML(formularioHTML, sizeof(formularioHTML));          // Serve o formulário HTML.
    ether.packetLoop(ether.packetReceive());                     // Loop do servidor Ethernet (ENC28J60) - para processar as solicitações do cliente.
    conexaoRede = true;                                          // Define a flag de conexão de rede como verdadeiro.
  } else {                                                       // Se a configuração DHCP falhou.
    Serial.println(F("DHCP: Falha na conexão"));                 // Exibe mensagem de falha.
    conexaoRede = false;                                         // Define a flag de conexão de rede como falso.
  }                                                              //
  Serial.println();                                              //
}

/**
 * @brief Função para inicializar o módulo RTC (Real Time Clock) DS1307.
 *
 * Esta função inicializa a comunicação I2C e o módulo RTC DS1307, verifica se
 * o módulo RTC DS1307 está funcionando corretamente e ajusta a data e hora atual. Em
 * caso de falha na obtenção da data/hora do servidor NTP, a data/hora atual do RTC é mantida.
 */
void inicializarRTC() {
  Wire.begin();                                                 // Inicializa a comunicação I2C.
  rtc.begin();                                                  // Inicializa o módulo RTC DS1307.
  if (!rtc.isrunning()) {                                       // Verifica se o RTC DS1307 está funcionando corretamente.
    Serial.println(F("RTC (DS1307): Falha na inicialização"));  // Exibe mensagem de erro.
    return;                                                     // Sai da função em caso de falha.
  } else {                                                      //
    Serial.println(F("RTC (DS1307): Inicializado"));            // Exibe mensagem de sucesso.
  }                                                             //
  if (conexaoRede) {                                            // Se houver conexão de rede.
    unsigned long novaDataHora = requisitarNTP();               // Solicita a Data/Hora atual ao servidor NTP.
    if (novaDataHora != 0) {                                    // Se a obtenção da Data/Hora for bem-sucedida.
      rtc.adjust(novaDataHora);                                 // Ajusta o RTC DS1307 com a nova Data/Hora.
    }                                                           // Se falhar, não faz nada (mantém Data/Hora atual do RTC).
  }                                                             // Se não houver conexão de rede, não faz nada (mantém Data/Hora atual do RTC).
  mostrarDataHora();                                            // Exibe Data/Hora atualizadas (ou mantidas).
  Serial.println();                                             // Exibe uma linha em branco.
}

/**
 * @brief Função para inicializar a comunicação serial.
 *
 * Esta função configura a comunicação serial com uma taxa de transmissão
 * de 57600 bps e exibe informações acadêmicas (curso, módulo, registro
 * acadêmico e nome do aluno) na porta serial.
 */
void inicializarSerial() {                     //
  Serial.begin(57600);                         // Inicializa a comunicação serial com a taxa de 57600 bps.
  Serial.println(exibirConstPROGMEM(curso));   // Exibe o nome do curso.
  Serial.println(exibirConstPROGMEM(modulo));  // Exibe o número do módulo.
  Serial.println(exibirConstPROGMEM(ra));      // Exibe o número de registro acadêmico (RA).
  Serial.println(exibirConstPROGMEM(aluno));   // Exibe o nome do aluno.
  Serial.println();                            // Exibe uma linha em branco.
}

/**
 * @brief Função para exibir os dados no display LCD I2C (20x4).
 *
 * Esta função limpa o display LCD I2C (20x4), define a posição do cursor e exibe
 * a grandeza medida, o valor e a unidade de medida no display LCD I2C (20x4).
 *
 * @param linha A linha onde os dados serão exibidos.
 * @param coluna A coluna onde os dados serão exibidos.
 * @param grandeza A grandeza medida.
 * @param valor O valor da grandeza medida.
 * @param unidade A unidade de medida da grandeza.
 */
void mostrarDadosLCD(int linha, int coluna, String grandeza, float valor, String unidade) {  //
  char buffer[10];                                                                           // Buffer para armazenar o valor float convertido em string.
  lcd.clear();                                                                               // Limpa o display LCD I2C (20x4).
  lcd.setCursor(coluna, linha);                                                              // Define a posição do cursor no display LCD I2C (20x4).
  lcd.print(grandeza + ":");                                                                 // Exibe a grandeza medida.
  dtostrf(valor, 6, 2, buffer);                                                              // Converte o valor float para uma string com duas casas decimais.
  lcd.print(buffer);                                                                         // Exibe o valor convertido.
  lcd.print(unidade);                                                                        // Exibe a unidade de medida.
  delay(500);                                                                                // Aguarda 0,5 segundo para exibição.
}

/**
 * @brief Função para retornar Data e Hora no formato "DD/MM/YYYY - hh:mm:ss".
 *
 * Esta função obtém a data e hora atual do RTC (Real Time Clock) DS1307 e
 * retorna uma string formatada no padrão "DD/MM/YYYY - hh:mm:ss".
 *
 * @return A data e hora formatada como uma string.
 */
String mostrarDataHora() {                       //
  DateTime agoraDataHora = rtc.now();            // Solicita a Data e Hora atuais ao módulo RTC DS1307
  char strDataHora[] = "DD/MM/YYYY - hh:mm:ss";  // Formatação da Data e Hora (DD/MM/YYYY - hh:mm:ss)                                                                                                                                                                                                                                                                                                                                                                                                                  // Obtém a data/hora atual do RTC DS1307
  agoraDataHora.toString(strDataHora);           // Retorna Data e Hora no formato "DD/MM/YYYY - hh:mm:ss"
}

/**
 * @brief Função para requisitar os dados calculados dos sensores.
 *
 * Esta função envia uma requisição via I2C para obter os dados
 * calculados dos sensores e armazena os dados na estrutura de dados.
 *
 * @param dados A referência para a estrutura de dados onde os dados serão armazenados.
 */
void requisitarLeituras(dadosSensores& dados) {            //
  Wire.requestFrom(9, sizeof(dadosSensores));              // Requisita os dados da placa de sensores via I2C.
  while (Wire.available()) {                               // Loop enquanto houver dados disponíveis para leitura.
    Wire.readBytes((char*)&dados, sizeof(dadosSensores));  // Lê os dados recebidos e armazena na estrutura de dados.
  }
}

/**
 * @brief Função para solicitar a Data/Hora atual a um servidor NTP.
 *
 * Esta função envia uma requisição NTP para um servidor NTP e aguarda
 * a resposta. Após receber a resposta, processa-a e retorna o timestamp
 * ajustado com o fuso horário.
 *
 * @return O timestamp ajustado com o fuso horário, em segundos desde 1970.
 */
unsigned long requisitarNTP() {                                                 //
  unsigned long respostaNTP;                                                    // Variável para armazenar a resposta do servidor NTP.
  const unsigned long ajusteSetentaAnos = 2208988800UL;                         // Ajuste para converter de NTP para Unix timestamp.
  for (int i = 0; i < 60; i++) {                                                // Loop para tentar receber resposta do servidor NTP (até 60 tentativas).
    ether.ntpRequest(servidorNTP, portaNTP);                                    // Envia requisição NTP.
    word tamanhoPacote = ether.packetReceive();                                 // Recebe o pacote de resposta.
    ether.packetLoop(tamanhoPacote);                                            // Processa o pacote recebido.
    if (tamanhoPacote > 0 && ether.ntpProcessAnswer(&respostaNTP, portaNTP)) {  // Se a resposta do servidor NTP for processada com sucesso.
      Serial.println(F("NTP: Sincronizado"));                                   // Exibe mensagem de sincronização bem-sucedida.
      return respostaNTP - ajusteSetentaAnos + fusoHorario;                     // Retorna o timestamp ajustado com o fuso horário.
    }                                                                           //
    delay(500);                                                                 // Aguarda 500 milissegundos.
  }                                                                             //
  Serial.println(F("NTP: Falha na sincronização"));                             // Exibe mensagem de falha.
  return 0;                                                                     // Retorna 0 para indicar erro na obtenção da data/hora.
}

/**
 * @brief Função para determinar o status do fator de cavitação da bomba.
 *
 * Esta função recebe o valor do fator de cavitação como entrada e determina
 * o status com base no seu valor.
 *
 * @param F_cavitacao O valor do fator de cavitação.
 * @return O status do fator de cavitação ("NORMAL", "NO LIMITE" ou "CAVITANDO").
 */
String statusCavitacao(float F_cavitacao) {  //
  if (F_cavitacao > 1) {                     // Verifica se o fator de cavitação é maior que 1.
    digitalWrite(alarme, LOW);               // Desliga o alarme (buzzer).
    return "NORMAL";                         // Retorna "NORMAL" se o fator de cavitação for maior que 1.
  } else if (F_cavitacao == 1) {             // Verifica se o fator de cavitação é igual a 1.
    return "NO LIMITE";                      // Retorna "NO LIMITE" se o fator de cavitação for igual a 1.
  } else {                                   // Se não for nenhuma das condições anteriores.
    digitalWrite(alarme, HIGH);              // Ativa o alarme (buzzer).
    return "CAVITANDO";                      // Retorna "CAVITANDO" se o fator de cavitação for menor que 1.
  }                                          //
}