X=Data;
L=length(Time);
Fs=L/(Time(end)-Time(1));
T=1/Fs;
t=(0:L-1)*T;


Y=fft(X);
if mod(L,2)==1
    L=L+1;
end
P2 = abs(Y/L);
P1 = P2(1:(L)/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:((L)/2))/L;
figure
plot(f,P1) 
xlabel('frequency(Hz)','FontSize',18,'FontWeight','bold');
ylabel('power(dB)','FontSize',18,'FontWeight','bold');

% X2=Data;
% L0=length(Data);
% T0=Time(1:L0);
% t2=(0:L0-1)*T;
% Y2=fft(X2);
% 
% if mod(L0,2)==1
%     L0=L0+1;
% end
% P20 = abs(Y2/L0);
% P10 = P20(1:(L0)/2+1);
% P10(2:end-1) = 2*P10(2:end-1);
% f0 = Fs*(0:((L0)/2))/L0;
% figure
% plot(f0,P10) 