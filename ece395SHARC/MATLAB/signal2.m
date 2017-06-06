clear all; close all; clc;

Fs = 48000;

% frequencies in Hz
f1 = 21600;
f2 = 1200;

% filter
FILTER_LENGTH = 64;
cutoff = 0.05;
coeffs = fir1(FILTER_LENGTH, cutoff, 'low');

%freqz(coeffs,1);

h = impz(coeffs, 1);
h = h(1:end-1);
stem(h);


theFileName = 'C:\wudtke_sconza_nunes\ece395SHARC\coeffs.h';
theFile = fopen(theFileName,'wt');

fprintf(theFile,'#define FILTER_LENGTH %d\n\n',FILTER_LENGTH+1);
fprintf(theFile,'//cutoff is %f Hz\n\n', cutoff*Fs);
fprintf(theFile,'double coeffs[%d] = {\n\n',FILTER_LENGTH+1);
fprintf(theFile,'\t%6.6f,\n',coeffs(1:end-1));
fprintf(theFile,'\t%6.6f',coeffs(end));
fprintf(theFile,'\n\n};\n\n');

