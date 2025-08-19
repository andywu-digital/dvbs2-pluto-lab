function [] = plot_spec(x)

y = fft(x);

n = length(x);          % number of samples
f = (0:n-1)*(10^6/n);     % frequency range
power = abs(y).^2/n;    % power of the DFT

plot(f,power)
xlabel('Frequency')
ylabel('Power')

end