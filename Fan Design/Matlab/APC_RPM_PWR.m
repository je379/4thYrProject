function [fitresult, gof] = APC_RPM_PWR(RPMAPC, PWRAPC)
%CREATEFIT(RPM,PWR)
%  Create a fit.
%
%  Data for 'APC_RPM-PWR' fit:
%      X Input : RPM
%      Y Output: PWR
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 21-Nov-2019 15:48:09


%% Fit: 'APC_RPM-PWR'.
[xData, yData] = prepareCurveData( RPMAPC, PWRAPC );

% Set up fittype and options.
ft = fittype( 'power1' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [8.94158230624564e-10 2.88956619091865];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

end

