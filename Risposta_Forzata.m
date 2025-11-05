clear; close all; clc; 
% Definizione del tempo
t = 0:0.01:100; % intervallo di tempo da 0 a 10 secondi, con passo 0.01

%% RISPOSTA ALL'IMPULSO
% y1 = 0.03523 + 0.00597*exp(-1.2*t)-0.3583*exp(-0.02*t);
% 
% 
% figure;
% plot(t, y1);
% xlabel('time [s]');
% ylabel('$y_{f,\tilde{x}_1}(t)$', 'Interpreter', 'latex');
% title('Risposta Impulsiva $y_{f,\tilde{x}_1}(t)$', 'Interpreter', 'latex');
% grid on;
% 
% 
% y2 =  0.0072*exp(-0.02*t) - 0.0072*exp(-1.2*t) ;
% 
% figure;
% plot(t, y2);
% xlabel('time [s]');
% ylabel('$y_{f,\tilde{x}_2}(t)$', 'Interpreter', 'latex');
% title('Risposta Impulsiva $y_{f,\tilde{x}_2}(t)$', 'Interpreter', 'latex');
% grid on;
% 
% 
% y3 = 0.08*exp(-1.2*t) ;
% 
% figure;
% plot(t, y3);
% xlabel('time [s]');
% ylabel('$y_{f,\tilde{x}_3}(t)$', 'Interpreter', 'latex');
% title('Risposta Impulsiva $y_{f,\tilde{x}_3}(t)$', 'Interpreter', 'latex');
% grid on;

%% RISPOSTA AL GRADINO

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
