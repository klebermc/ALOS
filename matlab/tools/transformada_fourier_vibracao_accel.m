close all
Fs = 10;            % Sampling frequency
T = 1/Fs;             % Sampling period
pstart = 750;
pend = 1120;
L = pend-pstart;             % Length of signal
t = (0:L-1)*T;        % Time vector

vet=accelz(pstart:pend-1);
% vet2=hampel(vet);
vet2=vet;

figure
plot(t,vet2)
grid on
title('Signal')
xlabel('t (milliseconds)')
ylabel('accelx(t)')
axis([1 t(L) -20 20])

Y = fft(vet2);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure
plot(f,P1)
title('Single-Sided Amplitude Spectrum of X(t)')
grid on
xlabel('f (Hz)')
ylabel('|P1(f)|')