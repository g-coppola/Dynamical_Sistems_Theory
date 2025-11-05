clear; close all; clc;
%% PARAMETRI
m = 1000;
b = 20;
g = 9.81;
V = 4000;
rho_a = 1.225;
T_a = 288.15;
c = 12.5;
k = 15;
Ad= 60;
Cd = 0.05;

x0 = [0 0 T_a]';
u0 = 0;
y0 = x0;

%% EQUILIBRIO E LINEARIZZAZIONE
x_eq = [450 0 T_a/(1-m/(V*rho_a))];
u_eq = k*(x_eq(3)-T_a);

modello = 'ModelloNonLineare';
%[xeq1, ueq1, yeq1] = trim(modello, x0, u0, y0);
xeq = x_eq;
ueq = u_eq;
yeq = x_eq;


[A, B, C, D] = linmod(modello, xeq, ueq);

C = C(1,:);
D = D(1,1);

x0_tilde = [x0(1)-xeq(1) x0(2)-xeq(2) x0(3)-xeq(3)];

%% AUTOVALORI E AUTOVETTORI
[Evector, Evalue] = eig(A);

%% FUNZIONE DI TRASFERIMENTO E POLI DEL SISTEMA
sys = ss(A,B,C,D);
W = tf(sys);
P = pole(W);

%% RISPOSTA AL GRADINO PER IL SISTEMA NON LINEARE
modelSimulation = sim('step_NL');
x1 = modelSimulation.y.Data(:,1);
x2 = modelSimulation.y.Data(:,2);
x3 = modelSimulation.y.Data(:,3);

time = modelSimulation.y.Time;
figure;

plot(time,x1);
title('Risposta al gradino per il sistema non linearizzato')
hold on;
plot(time,x2);
plot(time,x3);
hold off;
legend('Quota [m]', 'Velocità Verticale [m/s]', 'Temperatura Elio [K]');
xlabel('time [s]');
ylabel('x(t)');
grid on;

%% RISPOSTA AL GRADINO PER IL SISTEMA LINEARE
modelSimulation2 = sim('step_L');
x12 = modelSimulation2.y.Data(:,1);
x22 = modelSimulation2.y.Data(:,2);
x32 = modelSimulation2.y.Data(:,3);

time2 = modelSimulation2.y.Time;
figure;

plot(time2,x12);
title('Risposta al gradino per il sistema linearizzato')
hold on;
plot(time2,x22);
plot(time2,x32);
hold off;
legend('Quota [m]', 'Velocità Verticale [m/s]', 'Temperatura Elio [K]');
xlabel('time [s]');
ylabel('$\tilde{x}(t)$', 'Interpreter', 'latex');
grid on;

%% RISPOSTA LIBERA
x12l = modelSimulation2.yl.Data(:,1);
x22l = modelSimulation2.yl.Data(:,2);
x32l = modelSimulation2.yl.Data(:,3);
% 
figure;
plot(time2,x12l);
title('Risposta Libera $\tilde{x}_{1,l}(t)$', 'Interpreter', 'latex')
xlabel('time [s]');
ylabel('$\tilde{x}_{1,l}(t)$', 'Interpreter', 'latex');
grid on;

figure;
plot(time2,x22l);
title('Risposta Libera $\tilde{x}_{2,l}(t)$', 'Interpreter', 'latex')
xlabel('time [s]');
ylabel('$\tilde{x}_{2,l}(t)$', 'Interpreter', 'latex');
grid on;

figure;
plot(time2,x32l);
title('Risposta Libera $\tilde{x}_{3,l}(t)$', 'Interpreter', 'latex')
xlabel('time [s]');
ylabel('$\tilde{x}_{3,l}(t)$', 'Interpreter', 'latex');
grid on;

%% RISPOSTA FORZATA
% Definizione del tempo
t = 0:0.01:100; % intervallo di tempo da 0 a 10 secondi, con passo 0.01


% RISPOSTA ALL'IMPULSO
y1 = 0.03523 + 0.00597*exp(-1.2*t)-0.3583*exp(-0.02*t);

figure;
plot(t, y1);
xlabel('time [s]');
ylabel('$y_{f,\tilde{x}_1}(t)$', 'Interpreter', 'latex');
title('Risposta Impulsiva $y_{f,\tilde{x}_1}(t)$', 'Interpreter', 'latex');
grid on;
 
 
y2 =  0.0072*exp(-0.02*t) - 0.0072*exp(-1.2*t) ;
 
figure;
plot(t, y2);
xlabel('time [s]');
ylabel('$y_{f,\tilde{x}_2}(t)$', 'Interpreter', 'latex');
title('Risposta Impulsiva $y_{f,\tilde{x}_2}(t)$', 'Interpreter', 'latex');
grid on;
 
 
y3 = 0.08*exp(-1.2*t) ;
 
figure;
plot(t, y3);
xlabel('time [s]');
ylabel('$y_{f,\tilde{x}_3}(t)$', 'Interpreter', 'latex');
title('Risposta Impulsiva $y_{f,\tilde{x}_3}(t)$', 'Interpreter', 'latex');
grid on;


% RISPOSTA AL GRADINO
y11 = 0.03523*t - 17.910025 - 0.004975*exp(-1.2*t) + 17.915*exp(-0.02*t);

figure;
plot(t, y11);
xlabel('time [s]');
ylabel('$y_{f,\tilde{x}_1}(t)$', 'Interpreter', 'latex');
title('Risposta al Gradino $y_{f,\tilde{x}_1}(t)$', 'Interpreter', 'latex');
grid on;

y21 = 0.354 + 0.006*exp(-1.2*t) - 0.36*exp(-0.02*t);

figure;
plot(t, y21);
xlabel('time [s]');
ylabel('$y_{f,\tilde{x}_2}(t)$', 'Interpreter', 'latex');
title('Risposta al Gradino $y_{f,\tilde{x}_2}(t)$', 'Interpreter', 'latex');
grid on;

y31 = 0.0667 - 0.0667*exp(-1.2*t);

figure;
plot(t, y31);
xlabel('time [s]');
ylabel('$y_{f,\tilde{x}_3}(t)$', 'Interpreter', 'latex');
title('Risposta al Gradino $y_{f,\tilde{x}_3}(t)$', 'Interpreter', 'latex');
grid on;

%% RAGGIUNGIBILITA' E CONTROLLABILITA' 

% Matrice di Controllabilita'
R = ctrb(A,B);

% Rango di R
rank_R = rank(R);

K = acker(A, B, [-0.25 -0.42 -0.037]);
Ak = A-B*K;

modelSimulation3 = sim('Contr');
x1c = modelSimulation3.y.Data(:,1);
x2c = modelSimulation3.y.Data(:,2);
x3c = modelSimulation3.y.Data(:,3);

time_c = modelSimulation3.y.Time;
figure;

plot(time_c,x1c);
title('Sistema Controllato')
hold on;
plot(time_c,x2c);
plot(time_c,x3c);
hold off;
legend('Quota [m]', 'Velocità Verticale [m/s]', 'Temperatura Elio [K]');
xlabel('time [s]');
ylabel('x(t)');
grid on;

%% OSSERVABILITA' E RICOSTRUIBILITA'
%Matrice di Osservabilità
O = obsv(A,C);

%Rango di O
rank_O = rank(O);

Lt = place(A', C', [-0.29 -0.6 -0.91]);

L = Lt';

AL = A-L*C;
BL = [B L];


modelSimulation4 = sim('Obsv');
x11o = modelSimulation4.x1.Data(:,1);
x12o = modelSimulation4.x1.Data(:,2);

x21o = modelSimulation4.x2.Data(:,1);
x22o = modelSimulation4.x2.Data(:,2);

x31o = modelSimulation4.x3.Data(:,1);
x32o = modelSimulation4.x3.Data(:,2);

e1 = modelSimulation4.e1.Data(:,1);
e2 = modelSimulation4.e2.Data(:,1);
e3 = modelSimulation4.e3.Data(:,1);

time_o = modelSimulation4.x1.Time;


% CONFRONTO VARIABILE x_1
figure;
plot(time_o,x11o);
title('Sistema Osservato')
hold on;
plot(time_o,x12o);
hold off;
legend('x_1 misurata ', 'x_1 stimata');
xlabel('time [s]');
ylabel('x(t)');
grid on;


% CONFRONTO VARIABILE x_2
figure;
plot(time_o,x21o);
title('Sistema Osservato')
hold on;
plot(time_o,x22o);
hold off;
legend('x_2 misurata ', 'x_2 stimata');
xlabel('time [s]');
ylabel('x(t)');
grid on;


% CONFRONTO VARIABILE x_3
figure;
plot(time_o,x31o);
title('Sistema Osservato')
hold on;
plot(time_o,x32o);
hold off;
legend('x_3 misurata ', 'x_3 stimata');
xlabel('time [s]');
ylabel('x(t)');
grid on;


% ERRORI DI RICOSTRUZIONE DEGLI STATI
figure;
plot(time_o,e1);
title('Errore stimato per la variabile x_1 ')
xlabel('time [s]');
ylabel('e_1(t)');
grid on;

figure;
plot(time_o,e2);
title('Errore stimato per la variabile x_2 ')
xlabel('time [s]');
ylabel('e_2(t)');
grid on;

figure;
plot(time_o,e3);
title('Errore stimato per la variabile x_3 ')
xlabel('time [s]');
ylabel('e_3(t)');
grid on;


%% COMPENSATORE DINAMICO 
modelSimulation5 = sim('DynamicCompensator');

x1d = modelSimulation5.y.Data(:,1);
x2d = modelSimulation5.y.Data(:,2);
x3d = modelSimulation5.y.Data(:,3);

time_d= modelSimulation5.y.Time;

figure; 

plot(time_d,x1d);
title('Dynamic Compensator')
hold on;
plot(time_d,x2d);
plot(time_d,x3d);
hold off;
legend('Quota [m]', 'Velocità Verticale [m/s]', 'Temperatura Elio [K]');
xlabel('time [s]');
ylabel('x(t)');
grid on;
