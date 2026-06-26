clear;
clc;

%% PORTAS
portaKitSat = "COM7";   % KitSat: leitura do magnetómetro
portaEVM    = "COM4";   % DRV8411AEVM: envio de PWM

baudKitSat = 115200;
baudEVM    = 9600;

%% PARÂMETROS DE CONTROLO
Ts = 0.2;
duracao = 10;
N = round(duracao/Ts);

I_ref = 0.6;
PWM_MAX_TESTE = 40;     

km = 0.23/0.6;          % [A.m^2/A]
Kbdot = 5e4;            % ganho usado no Simulink, com B em Tesla

%% ENDEREÇOS DA DRV8411AEVM

ADDR_CTR_MODE   = hex2dec('03D8');
ADDR_AIN1       = hex2dec('03DE'); % Canal A -> bobina Z
ADDR_AIN2       = hex2dec('03DF'); % Canal A -> bobina Z
ADDR_BIN1       = hex2dec('03E0'); % Canal B -> bobina Y
ADDR_BIN2       = hex2dec('03E1'); % Canal B -> bobina Y
ADDR_MOTOR_TYPE = hex2dec('03E2');
ADDR_FREQ       = hex2dec('03E3');
ADDR_NSLEEP     = hex2dec('03E4');

%% ABRIR AS COMUNICAÇÕES

% Abre a comunicação com as duas placas.


sKitSat = serialport(portaKitSat, baudKitSat, "Timeout", Ts);
sEVM    = serialport(portaEVM, baudEVM);

% limpa dados antigos nas portas após aguardar 1 segundo
pause(1);
flush(sKitSat);
flush(sEVM);

fprintf("KitSat ligado em %s.\n", portaKitSat);
fprintf("DRV8411AEVM ligada em %s.\n", portaEVM);

%% CONFIGURAR a Evaluation Board DRV8411AEVM

%Configura o firmware da EVM
write_byte(sEVM, ADDR_MOTOR_TYPE, 1);
write_byte(sEVM, ADDR_CTR_MODE, 1);
write_byte(sEVM, ADDR_FREQ, 0);

% Garante que as bobinas começam desligadas
write_byte(sEVM, ADDR_AIN1, 0);
write_byte(sEVM, ADDR_AIN2, 0);
write_byte(sEVM, ADDR_BIN1, 0);
write_byte(sEVM, ADDR_BIN2, 0);

% Ativa o driver
write_byte(sEVM, ADDR_NSLEEP, 1);
pause(0.1);

fprintf("EVM configurada.\n");

%% PRIMEIRA LEITURA DO MAGNETÓMETRO

B_raw_antigo = kitsat_read_mag(sKitSat);   % leitura em Gauss do primeiro valor do campo magnético

% No caso da primeira leitura ser inválida dá print do seguinte erro:

if any(isnan(B_raw_antigo))
    error("Primeira leitura do magnetómetro inválida.");
end

B_antigo = B_raw_antigo * 1e-4;  % Conversão de Gauss -> Tesla

fprintf("Primeira leitura B_raw = [%.4f %.4f %.4f] G\n", ...
    B_raw_antigo(1), B_raw_antigo(2), B_raw_antigo(3));

fprintf("Primeira leitura B = [%.4e %.4e %.4e] T\n", ...
    B_antigo(1), B_antigo(2), B_antigo(3));

%% VETORES PARA GUARDAR DADOS
tempo_log = zeros(N,1);
B_raw_log = zeros(N,3); % campo em Gauss
B_log     = zeros(N,3); % campo em Tesla
Bdot_log  = zeros(N,3); % derivada do campo magnético
m_log     = zeros(N,3); % momento magnético comandado
I_log     = zeros(N,2); % correntes nas bobinas Z e Y (não tenho bobina em x)
PWM_log   = zeros(N,4); % valores enviados para a EVM
dt_log    = zeros(N,1); % tempo real entre amostras

%% LOOP 
fprintf("A iniciar controlo B-dot...\n");

t0 = tic;
t_antigo = toc(t0);

for k = 1:N

    %% Leitura do campo magnético
    B_raw_atual = kitsat_read_mag(sKitSat);

    % No caso de o valor lido ser inválido

    if any(isnan(B_raw_atual))
        warning("Leitura inválida em k=%d. Saídas desligadas.", k);

        write_byte(sEVM, ADDR_AIN1, 0);
        write_byte(sEVM, ADDR_AIN2, 0);
        write_byte(sEVM, ADDR_BIN1, 0);
        write_byte(sEVM, ADDR_BIN2, 0);

        pause(Ts);
        continue;
    end

    B_atual = B_raw_atual * 1e-4;  % Conversão de Gauss -> Tesla

    %% Cálculo de dt real
    t_atual = toc(t0);
    dt = t_atual - t_antigo;

    if dt <= 0
        dt = Ts;
    end

    %% B-dot
    Bdot = (B_atual - B_antigo)/dt;

    %% Lei de controlo B-dot
    m_cmd = -Kbdot * Bdot;

    % Não existe bobina no eixo X
    m_cmd(1) = 0;

    % Bobinas apenas em Y e Z
    my = m_cmd(2);
    mz = m_cmd(3);

  
    % Canal A / AOUT1-AOUT2 -> bobina Z
    % Canal B / BOUT1-BOUT2 -> bobina Y
    I_A = mz/km;
    I_B = my/km;

    %% Saturação de corrente
    I_A = max(min(I_A, I_ref), -I_ref);
    I_B = max(min(I_B, I_ref), -I_ref);

    %% Corrente -> PWM
    [AIN1, AIN2, BIN1, BIN2] = currents_to_PWM(I_A, I_B, I_ref, PWM_MAX_TESTE);

    %% Enviar comandos à EVM
    write_byte(sEVM, ADDR_AIN1, AIN1);
    write_byte(sEVM, ADDR_AIN2, AIN2);
    write_byte(sEVM, ADDR_BIN1, BIN1);
    write_byte(sEVM, ADDR_BIN2, BIN2);

    %% Guardar dados
    tempo_log(k) = t_atual;
    B_raw_log(k,:) = B_raw_atual;
    B_log(k,:) = B_atual;
    Bdot_log(k,:) = Bdot;
    m_log(k,:) = m_cmd;
    I_log(k,:) = [I_A I_B];
    PWM_log(k,:) = double([AIN1 AIN2 BIN1 BIN2]);
    dt_log(k) = dt;

    fprintf("k=%3d | dt=%.3f s | B=[%+.3e %+.3e %+.3e] T | m=[%+.3e %+.3e %+.3e] A.m^2 | I_A(Z)=%+.3f A | I_B(Y)=%+.3f A | PWM=[%3d %3d %3d %3d]\n", ...
        k, dt, ...
        B_atual(1), B_atual(2), B_atual(3), ...
        m_cmd(1), m_cmd(2), m_cmd(3), ...
        I_A, I_B, AIN1, AIN2, BIN1, BIN2);

    %% Atualizar memória
    B_antigo = B_atual;
    t_antigo = t_atual;

    pause(Ts);
end

%% DESLIGAR SAÍDAS
fprintf("A desligar saídas...\n");

write_byte(sEVM, ADDR_AIN1, 0);
write_byte(sEVM, ADDR_AIN2, 0);
write_byte(sEVM, ADDR_BIN1, 0);
write_byte(sEVM, ADDR_BIN2, 0);
write_byte(sEVM, ADDR_NSLEEP, 0);

clear sKitSat sEVM;

fprintf("Teste terminado.\n");

%% GRÁFICOS
figure;
plot(tempo_log, B_raw_log);
grid on;
xlabel("Tempo [s]");
ylabel("Campo magnético bruto [G]");
legend("B_x","B_y","B_z");

figure;
plot(tempo_log, B_log);
grid on;
xlabel("Tempo [s]");
ylabel("Campo magnético [T]");
legend("B_x","B_y","B_z");

figure;
plot(tempo_log, Bdot_log);
grid on;
xlabel("Tempo [s]");
ylabel("B-dot [T/s]");
legend("dB_x/dt","dB_y/dt","dB_z/dt");

figure;
plot(tempo_log, I_log);
grid on;
xlabel("Tempo [s]");
ylabel("Corrente comandada [A]");
legend("I_A - bobina Z","I_B - bobina Y");

PWM_A = max(PWM_log(:,1), PWM_log(:,2)); % Canal A (bobina Z)
PWM_B = max(PWM_log(:,3), PWM_log(:,4)); % Canal B (bobina Y)

figure;
plot(tempo_log, PWM_A,'LineWidth',1.5); hold on;
plot(tempo_log, PWM_B,'LineWidth',1.5);
grid on;

xlabel("Tempo [s]");
ylabel("PWM Duty Cycle");
title("PWM Duty Cycle Commands");
legend("Coil Z","Coil Y","Location","best");

figure;
plot(tempo_log, I_log,'LineWidth',1.5);
grid on;
xlabel("Tempo [s]");
ylabel("Corrente comandada [A]");
title("Commanded Current as a function of Time");
legend("I_A - bobina Z","I_B - bobina Y","Location","best");

figure;
plot(tempo_log, m_log,'LineWidth',1.5);
grid on;
xlabel("Tempo [s]");
ylabel("Momento magnético comandado [A.m^2]");
title("Commanded Magnetic Moment as a function of Time");
legend("m_x","m_y","m_z","Location","best");

%% FUNÇÕES AUXILIARES

function B = kitsat_read_mag(ser)

    packet = cmd_parser("imu_get_all");
    write(ser, packet, "uint8");

    [~, dlm_found] = read_until(ser, "packet:");

    if ~dlm_found
        B = [NaN NaN NaN];
        return;
    end

    data = get_data(ser);

    if isempty(data)
        B = [NaN NaN NaN];
        return;
    end

    imu = parseIMU(data.msg);

    % Ordem: [mag, gyr, acc]
    B = double(imu(1:3));

end

function [AIN1, AIN2, BIN1, BIN2] = currents_to_PWM(I_A, I_B, I_ref, pwmMax)

    PWM_A = uint8(max(min(round(abs(I_A)/I_ref*pwmMax), pwmMax), 0));
    PWM_B = uint8(max(min(round(abs(I_B)/I_ref*pwmMax), pwmMax), 0));

    % Canal A -> bobina Z
    if I_A >= 0
        AIN1 = PWM_A;
        AIN2 = uint8(0);
    else
        AIN1 = uint8(0);
        AIN2 = PWM_A;
    end

    % Canal B -> bobina Y
    if I_B >= 0
        BIN1 = PWM_B;
        BIN2 = uint8(0);
    else
        BIN1 = uint8(0);
        BIN2 = PWM_B;
    end
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

function packet = cmd_parser(command)

    packet = uint8([]);
    cmds = split(command);

    if cmds(1) == "ping_local"
        packet = [packet, uint8(10), uint8(3), uint8(0)];

    elseif cmds(1) == "imu_get_mag"
        packet = [packet, uint8(5), uint8(1), uint8(0)];

    elseif cmds(1) == "imu_get_all"
        packet = [packet, uint8(5), uint8(14), uint8(0)];

    elseif cmds(1) == "gs_set_network"
        packet = [packet, uint8(11), uint8(5)];
        packet = appendOption(cmds{2}, packet);
    end

    packet = appendFnvHash(packet);
end

function packet = appendOption(op, packet)

    cmdLength = length(unicode2native(op, "UTF-8"));
    packet = [packet, uint8(cmdLength)];
    packet = [packet, unicode2native(op, "UTF-8")];
end

function data = get_data(ser)

    if ser.NumBytesAvailable > 0
        data.orig_int = read(ser, 1, "uint8");
        data.cmd_id_int = read(ser, 1, "uint8");
        data.data_len_int = read(ser, 1, "uint8");
        data.timestamp = read(ser, 1, "uint32");
        data.msg = read(ser, data.data_len_int, "uint8");
        data.fnv = read(ser, 1, "uint32");
    else
        data = [];
    end
end

function out = parseIMU(data)

    data = uint8(data);
    out = typecast(data, "single");
end

function hval = fnv(byteArray)

    hval = uint32(2166136261);
    fnv32Prime = uint32(16777619);
    uint32Max = uint64(4294967296);

    for s = byteArray
        hval = bitxor(hval, uint32(s));
        hval = uint32(mod(uint64(hval)*uint64(fnv32Prime), uint32Max));
    end
end

function packet = appendFnvHash(packet)

    hashValue = fnv(packet);
    hashBytes = typecast(uint32(hashValue), "uint8");
    packet = [packet, hashBytes];
end

function [data, delimiter_found] = read_until(serialObj, delimiter)

    Nout = 400;
    data = zeros(1, Nout, "uint8");
    delimiter_found = false;

    for I = 1:Nout

        if serialObj.NumBytesAvailable > 0

            byte = read(serialObj, 1, "uint8");
            data(I) = byte;

            if I >= strlength(delimiter) && ...
                    all(data(I-strlength(delimiter)+1:I) == uint8(char(delimiter)))

                data = data(1:I);
                delimiter_found = true;
                break;
            end

        else
            pause(0.001);
        end
    end
   
end
