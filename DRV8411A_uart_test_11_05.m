clear;
clc;

% O objetivo deste códgio é utilizar os valores Ix e Iy que se obteve no
% Simulink e transforma-los em comandos para as duas pontes H da Evaluation
% Board.
% PWM controla diretamente a tensão média aplicada à bobina e depois a
% corrente resulta da dinâmica elétrica da bobina.
% V= L*di/dt + Ri

%% CONFIGURAÇÃO GERAL

portaCOM = "COM4"; % porta série onde a EVM está ligada
baudRate = 9600; % velocidade de comunicação

% o ficheiro .mat onde estão guardadas as correntes da simulação
ficheiroCorrente = "Current_Values_modelo_cubesat_4_without_z_component.mat"; % o ficheiro .mat onde estão guardadas as correntes da simulação

% Corrente máxima e PWM máximo

I_ref = 0.6; % corrente máxima usada como referência na simulação

% Limite de PWM para teste inicial, por segurança. O máximo teórico é 255
% PWM é enviado como um uint8 (inteiro sem sinal de 8 bits). Desta forma,
% existem 2^8 = 256 valores possíveis para PWM. Como começa em 0, PWM pode
% ir ate 255 (PWM máximo)
PWM_MAX_TESTE = 40;

% A conversão é de aproximadamente 0 A -> PWM 0
% 0.3 A -> PWM 20
% 0.6 A (MÁX) -> PWM 40 (MÁX)

% Para não enviar todas as amostras da simulação.
% Exemplo: decimacao = 50 envia uma amostra a cada 50.

decimacao = 50; % A simulação pode ter milhares de pontos. Em vez de se enviar
% todos, envia-se só 1 em cada 50

% Tempo entre comandos enviados para a EVM

tempoEntreComandos = 0.05;   % segundos

%% ABRIR PORTA SÉRIE

% variável s abre a comunicação com a EVM

s = serialport(portaCOM, baudRate); %  Definem-se a serial port for connection e a communication speed
fprintf("Ligação série aberta em %s.\n", portaCOM);


%% ENDEREÇOS DO FIRMWARE

%enderecos internos do firmware que controlam as entradas da ponte H

ADDR_CTR_MODE   = hex2dec('03D8');
ADDR_AIN1       = hex2dec('03DE');%bobina X
ADDR_AIN2       = hex2dec('03DF');%bobina X
ADDR_BIN1       = hex2dec('03E0');%bobina Y
ADDR_BIN2       = hex2dec('03E1');%bobina Y
ADDR_MOTOR_TYPE = hex2dec('03E2');
ADDR_FREQ       = hex2dec('03E3');
ADDR_NSLEEP     = hex2dec('03E4');


%% CARREGAR CORRENTES DA SIMULAÇÃO

dados = load(ficheiroCorrente); % carrega os dados de corrente contidos no ficheiro

% Tenta encontrar automaticamente a matriz de corrente no ficheiro
nomes = fieldnames(dados);

if isfield(dados, "I")
    I = dados.I;
elseif isfield(dados, "current")
    I = dados.current;
elseif isfield(dados, "Current")
    I = dados.Current;
else
    error("Não encontrei uma variável de corrente chamada I, current ou Current no ficheiro MAT.");
end

Ix = I(:,1);
Iy = I(:,2);

fprintf("Foram carregadas %d amostras de corrente.\n", length(Ix));


%% CONFIGURAR A EVM

fprintf("A configurar a evaluation board...\n");

write_byte(s, ADDR_MOTOR_TYPE, 1);
write_byte(s, ADDR_CTR_MODE, 1);
write_byte(s, ADDR_FREQ, 0);

% Garantir que as saídas começam desligadas
write_byte(s, ADDR_AIN1, 0);
write_byte(s, ADDR_AIN2, 0);
write_byte(s, ADDR_BIN1, 0);
write_byte(s, ADDR_BIN2, 0);

% Ativar driver
write_byte(s, ADDR_NSLEEP, 1);
pause(0.1);

valorNSLEEP = read_byte(s, ADDR_NSLEEP);
fprintf("NSLEEP lido após ativação: %d\n", valorNSLEEP);


%% ENVIAR COMANDOS PWM

fprintf("A enviar comandos PWM para as bobinas X e Y...\n");

indices = 1:decimacao:length(Ix);

for i = indices

    correnteX = Ix(i);
    correnteY = Iy(i);

    [AIN1, AIN2, BIN1, BIN2] = currents_to_PWM( ...
        correnteX, correnteY, I_ref, PWM_MAX_TESTE);
% PWM = |Icmd| / I_ref × pwmMax (I_ref é o valor máximo de corrente
% suportado (0.6 A)

% Os valores PWM calculados são enviados para o firmware

    write_byte(s, ADDR_AIN1, AIN1);
    write_byte(s, ADDR_AIN2, AIN2);
    write_byte(s, ADDR_BIN1, BIN1);
    write_byte(s, ADDR_BIN2, BIN2);

% Ler de volta para confirmar que os valores chegaram ao firmware
    AIN1_lido = read_byte(s, ADDR_AIN1);
    AIN2_lido = read_byte(s, ADDR_AIN2);
    BIN1_lido = read_byte(s, ADDR_BIN1);
    BIN2_lido = read_byte(s, ADDR_BIN2);

    fprintf("k=%5d | Ix=%+.3f A | Iy=%+.3f A | AIN1=%3d AIN2=%3d BIN1=%3d BIN2=%3d\n", ...
        i, correnteX, correnteY, AIN1_lido, AIN2_lido, BIN1_lido, BIN2_lido);

    pause(tempoEntreComandos);
end


%% DESLIGAR SAÍDAS

fprintf("A desligar saídas...\n");

write_byte(s, ADDR_AIN1, 0);
write_byte(s, ADDR_AIN2, 0);
write_byte(s, ADDR_BIN1, 0);
write_byte(s, ADDR_BIN2, 0);

write_byte(s, ADDR_NSLEEP, 0);
pause(0.1);

% Desativar o Driver

valorNSLEEP = read_byte(s, ADDR_NSLEEP);
fprintf("NSLEEP lido após desativação: %d\n", valorNSLEEP);


%% FECHAR PORTA

clear s;

fprintf("Teste terminado. Porta série fechada.\n");


%% FUNÇÕES AUXILIARES

% Função que converte a corrente em valor absoluto em ampères num duty-cycle (PWM) - Pulse Width
% Modulation 

function [AIN1, AIN2, BIN1, BIN2] = currents_to_PWM(Ix, Iy, I_ref, pwmMax)

   % Conversão da magnitude da corrente em duty-cycle PWM
   % PWM apenas representa magnitude, o sentido da corrente é tratado a seguir
   % Round arredonda os números

    PWM_X = uint8(max(min(round(abs(Ix) / I_ref * pwmMax), pwmMax), 0));
    PWM_Y = uint8(max(min(round(abs(Iy) / I_ref * pwmMax), pwmMax), 0));

    % Bobina X ligada ao canal A

    if Ix >= 0
        AIN1 = PWM_X; %sentido positivo da corrente ao longo do eixo x
        AIN2 = uint8(0); % sentido negativo da corrente ao longo do eixo x O firmware da EVM espera receber: unsigned integer de 8 bits
        % uint8 é explicitamente um byte de 8 bits
    else
        AIN1 = uint8(0);
        AIN2 = PWM_X;
    end

    % Bobina Y ligada ao canal B

    if Iy >= 0
        BIN1 = PWM_Y;  %sentido positivo da corrente ao longo do eixo y
        BIN2 = uint8(0); %sentido negativo da corrente ao longo do eixo y
    else
        BIN1 = uint8(0);
        BIN2 = PWM_Y;
    end

    % Ix positivo -> AIN1 ativo, AIN2 zero
    % Iy negativo -> AIN1 zero, AIN2 ativo

end

function write_byte(s, addr, val)

    write(s, uint8([ ...
        hex2dec('81'), ...
        bitand(bitshift(uint32(addr), -24), 255), ...
        bitand(bitshift(uint32(addr), -16), 255), ...
        bitand(bitshift(uint32(addr), -8), 255), ...
        bitand(uint32(addr), 255), ...
        uint8(val) ...
    ]), "uint8");

end


function val = read_byte(s, addr)

    cmd = uint8([ ...
        hex2dec('C1'), ...
        bitand(bitshift(uint32(addr), -24), 255), ...
        bitand(bitshift(uint32(addr), -16), 255), ...
        bitand(bitshift(uint32(addr), -8), 255), ...
        bitand(uint32(addr), 255) ...
    ]);

    flush(s);
    write(s, cmd, "uint8");

    resposta = read(s, 2, "uint8");

    val = resposta(2);

end