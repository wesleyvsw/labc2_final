A = [[ -113.5203 -34.3792];[27.0249 -0.3312]];
b = [46.6079 0]';
c = [0 1];% Saida velocidade
[num,den] = ss2tf(A,b,eye(2),[0 0]');
roots(den)% raizes do minha ft



mp = 0.1;
ts = 0.2;
zeta = -(log(mp))/sqrt(3.14^2 +(log(mp))^2 );
wn = 4/(zeta*ts);





%escolhendo os polos
polos = roots([1 2*zeta*wn wn^2]);
k_ackm = acker(A,b,polos);
kt_ee = place(A, b, polos)%%acho que preciso criar mais um polo;

