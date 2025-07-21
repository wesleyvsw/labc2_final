%% Analise inicial
d1 = readtable('RP21.csv');
d2 = readtable('RP22.csv');
d3 = readtable('RP23.csv');
d4 = readtable('RP24.csv');

d3.C3InV = d3.C3InV*(-1); %invertendo os sinais para casarem com o sensor
d4.C4InV = d4.C4InV*(-1);

% Iniciar o gráfico
figure;
plot(d1.inS, d1.C1InV, 'r-', 'LineWidth', 1.5); hold on;
plot(d2.inS, d2.C2InV, 'b--', 'LineWidth', 1.5);
plot(d3.inS, d3.C3InV, 'g-.', 'LineWidth', 1.5);
plot(d4.inS, d4.C4InV, 'k:', 'LineWidth', 1.5);

%Rótulos e legendas
xlabel('Tempo (s)');
ylabel('Tensão (V)');
title('Comparação de 4 sinais');
legend('Sinal 1', 'Sinal 2','Sinal 3', 'Sinal 4');
grid on;

%% Removendo o offset
media1 = mean(d1.C1InV);
media2 = mean(d2.C2InV);
media3 = mean(d3.C3InV);
media4 = mean(d4.C4InV);

d1.C1InV = d1.C1InV - media1;
d2.C2InV = (d2.C2InV - media2);
d3.C3InV = d3.C3InV - media3;
d4.C4InV = d4.C4InV - media4;

%% Com os dados, declarando parametros do problema
k_corrente_sensor = 1.5472
h = d4.inS(2)-d4.inS(1)  %%Intervalo de amostragem
ia = movmean((d2.C2InV)/k_corrente_sensor,1000) %Valores de corrente para o dado de entrada no sensor
v_sensor = movmean(d2.C2InV,1000);
k_barra = 1.3557
kt = 0.1470;
kg = kt/k_barra;
ka = kg %Considerando que estou no SI
corrente = ia;
v_tacometro = movmean(d4.C4InV,1000);
v_armadura = movmean(d3.C3InV,1000);
ue = v_armadura -(kg/kt)*v_tacometro;
um = ka*kt*corrente;

%% Parametros do motor R, La, J,f
%% Matriz eletrica e mecânica
ia_sem_primeiro = corrente(2:end)'; %vetor corrente sem o primeiro elemento
me = [corrente(1:end-1).' ue(1:end-1).']; %matriz
xe = [ia(1:end-1) ue(1:end-1)]\ia(2:end);
%xe = (me'*me)^-1*me'*ia_sem_primeiro;
%mecanica --- semelhante ao eletrico
vt1 = v_tacometro(2:end)';
mm = [v_tacometro(1:end-1).' um(1:end-1).'];
%xm = (mm'*mm)^-1*mm'*vt1;
xe = [ia(1:end-1) ue(1:end-1)]\ia(2:end);
xm = [v_tacometro(1:end-1) um(1:end-1)]\v_tacometro(2:end);
%% Ra, La,f,J
%ra = (1-xe(1))/xe(2)
%la = - (ra*h)/log(xe(1))
%f = (1-xm(1))/xm(2)
%inercia = -f*h/(log(xm(2)))
Ra = (1-xe(1))/xe(2)
La = -Ra*h/log(xe(1))
f = (1-xm(1))/xm(2)
J = -f*h/log(xm(1))
%% Espaço de estados
A = [-Ra/La -kg/La/kt; ka*kt/J -f/J];
b = [1/La;0];
c =[0 1]
v_armadura_aj = [d3.inS v_armadura];
corrente_aj = [d3.inS corrente];
v_tacometro_aj = [d3.inS v_tacometro];
%% Controlabilidade e Observabilidade
controlabilidade = ctrb(A, b)% determinante diferente de zero, matriz controlavel
%% Tempo de acomodação e overshooting
mp = 0.1
ts = 0.2
zeta = -(log(mp))/sqrt(3.14^2 +(log(mp))^2 )
wn = 4/(zeta*ts)
%escolhendo os polos
polos = roots([1 2*zeta*wn wn^2])
kt_ee = place(A, b, polos)%%acho que preciso criar mais um polo
%% Controlador Robusto
%A_robusto = A - b*kt_ee
%d_final = A - l*c
A_nova = [[-113.5203 -34.3792 0];[27.0249 -0.3312 0];[0 -1 0]];
b_novo = [46.6079;0;0];
polos = [polos;-200] %Adicionando 
k_total = place(A_nova,b_novo,polos)%Encontrando os k's
kt1 = k_total(1:2);       % pega os dois primeiros elementos → [10 20]
ki  = k_total(3);         % pega o terceiro elemento → 30
%% Observador de estados
A_obs = A';
c_obs = c';
L_obs = place(A_obs,c_obs,[-400,-401])












