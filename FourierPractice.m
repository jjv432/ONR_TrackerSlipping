clc; close all; clearvars -except Data

%{

Works for some of the frequencies

%}

data = Data.foot1p0Hz;  
Fs = 1/(data.t(2) - data.t(1));
T = 1/Fs;
L = numel(data.t); %data.t(end) - data.t(1);
t = (0:L-1)*T;
S = data.x;
X = S;

plot(1000*t,X)
title("Original Signal")
xlabel("t (milliseconds)")
ylabel("X(t)")

Y = fft(X);

P2 = abs(Y/L);
temp = floor(L/2);
P1 = P2(1:temp+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs/L*(0:(L/2));
figure()
plot(f,P1,"LineWidth",3)
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")