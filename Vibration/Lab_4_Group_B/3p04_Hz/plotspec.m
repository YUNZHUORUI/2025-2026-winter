function ans1 = plotspec(npoints,delt,chref,ch2)

    npo2   = floor(npoints/2);

    timwin = npoints*delt;
    delf = 1/timwin;
    fs   = npoints*delf;
    fnq  = fs/2;
    
    f = 0:delf:fnq;

    win = flattpn(npoints);
    
    Chref  = fft(chref(1:npoints).*win)/npoints;
    Chrefm = Chref.*conj(Chref);
    Chrefm = 2*sqrt(Chrefm);

    
    Ch2  = fft(ch2(1:npoints).*win)/npoints;
    Ch2m = Ch2.*conj(Ch2);
    Ch2m = 2*sqrt(Ch2m);

    phase = rad2deg(angle(Ch2./Chref));
    f_peak1 = 3.09224
    f_peak2 = 6.15993
    
    figure()
    plot(f,Chrefm(1:npo2+1),'DisplayName','Ref')
    hold on
    plot(f,Ch2m(1:npo2+1),'DisplayName','Ch 2')
    hold off
    legend('Location','northeast')
    title('Spectrum')
    xlabel('frequency (Hz)')
    ylabel('Absolute value')
    grid on

    figure()
    plot(f,phase(1:npo2+1))
    title('Phase of Ch 2 relative to Ref')
    xlabel('frequency (Hz)')
    ylabel('Angle in degrees')
    grid on
    hold on
    %xline(f_peak1, '--r', 'LineWidth', 1, 'Label', 'Peak 1');
    %xline(f_peak2, '--r', 'LineWidth', 1, 'Label', 'Peak 2'); 
    
    ans1 = true;
