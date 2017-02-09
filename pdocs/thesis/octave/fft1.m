%
% Simple demo on using fft (fast fourier transform) 
% sample input is created using series of sini waves
% frequencies and amplitudes should match values given in f
%
% e.g. 1,5,10 give nice resuls
% 2,5,12 not so nice (5 is widened)
f=[
1,5,10; % frequency (Hz)
1,1, 1]; % amplitude
fs=256; % sampling frequency
% sample time so that one wave of slowest component is seen
st=1/min(f(1,:)); 
dt = 1/fs; % timestep 
i=1;
values=[];
for t =0:dt:st-dt
  values(i)=0;
  for fi =1:size(f)(2); % loop over columns
    values(i)+=2*f(2,fi)*sin(f(1,fi)*2*pi*t);
  endfor
  i++;
endfor  
% show generated input
subplot(2,1,1)
plot(0:dt:st-dt,values);
xlabel('time')
ylabel('value')
N = size(values)(2);
transform = fft(values)/N;
magTransform = abs(transform);
faxis = linspace(0,fs/2,N/2);
% show only positive side of fft results
subplot(2,1,2)
plot(faxis,fftshift(magTransform)(N/2+1:end));
axis([0 1.1*max(f(1,:)) 0 max(f(2,:))])
xlabel('frequency (Hz)')
ylabel('amplitude')
more off