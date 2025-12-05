% Tarea 2: Análisis de Señales Mixtas
clc; clear; close all;


Vin = 12;       % Voltaje de entrada del escalón [V]
K = 2.5;        % Ganancia estática del motor [rad/s / V]
Tau = 0.3;      % Constante de tiempo [s]

% Creamos la Función de Transferencia oficial: G(s) = K / (Tau*s + 1)
sys = tf(K, [Tau 1]); 

disp('Función de Transferencia del Sistema:');
sys


t = 0:0.01:3;   % Vector de tiempo

% Generamos la respuesta ideal al escalón de 12V 
[y_ideal, t_out] = step(sys * Vin, t);

% Agregamos ruido
ruido = 0.8 * (rand(size(y_ideal)) - 0.5); 
y_sensor = y_ideal + ruido;


figure('Color', 'w', 'Position', [100, 100, 800, 600]);

% --- Gráfica 1: Respuesta al Escalón---
subplot(2,1,1);
plot(t, y_sensor, 'b.', 'MarkerSize', 6); hold on;
plot(t, y_ideal, 'r', 'LineWidth', 2);
title(['Caracterización: Respuesta al Escalón de ', num2str(Vin), 'V']);
xlabel('Tiempo (s)'); ylabel('Velocidad (rad/s)');
legend('Datos Experimentales (Simulados)', 'Modelo Identificado G(s)');
grid on;


subplot(2,1,2);


impulse(sys); 
title('Respuesta al Impulso');
grid on;


saveas(gcf, 'graficas_sistema.jpg');
disp('Imagen guardada exitosamente.');