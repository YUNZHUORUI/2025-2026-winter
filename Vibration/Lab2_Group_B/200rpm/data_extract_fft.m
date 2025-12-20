clear all, close all, clc

file1 = 'expdata.mat';
load(file1);


figure(1)
plot(tb_n,ygb,'DisplayName','Ref')
hold on
plot(tb_n,yg1_i,'DisplayName','Ch 2')
hold off
grid on
xlabel('Time (s)')
ylabel('y acceleration (g)')
title('Base excitation vibration 1DOF system')
hold off

delt = tb_n(2)-tb_n(1);

% Spectrum analysis
%
% (Fourier analysis)
% 
% What is done with the function "plotspec" below is certainly
% not the most efficient and correct way of doing spectrum analysis on
% measured vibration data, for, amongst others, the following reasons:
% - No anti-aliasing filtering was employed before sampling the data.  When
%   aliasing happens, the spectrum is contaminated below half the sampling
%   frequency by whatever happens above half the sampling frequency.  To
%   prevent aliasing, a high quality (as not to distort the phase
%   information) analogue low pass filter must be employed before sampling
%   the analogue signal, to remove any content above half the sampling
%   frequency.  Our sampling frequency is about 50 Hz.  I am fairly
%   confident that we do not have significant content in our acceleration
%   signals above half the sampling frequency (25 Hz).
% - The spectra should really be calculated through averaging a number of
%   FFT results, employing power and cross spectra.  This is not done here.
%   Instead, we shall consider a single FFT of a single time window of data
%   for each channel and use this to draw conclusions.  In this experiment
%   this procedure works well enough to be used.
% 
% The "plotspec" functions calls the "fft" function on time data
% from each measurement channel and then produce the spectra. FFT stands
% for Fast Fourier Transform.

plotspec(2048,delt,ygb,yg1_i); % Does FFT on 1st 2048 points
% plotspec(2048,delt,ygb(end-2047:end),yg1_i(end-2047:end));
  % Previous line: does FFT on last 2048 points
%plotspec_1ch(2048,delt,ygb);

