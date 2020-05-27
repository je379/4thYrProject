function [rc, WF, RHS] = solve_JSEcondition(payload,sig,htr)
%% Solve Eriksen condition for casing radius
% rc = solve_JSEcondition(payload,sig,htr)
% htr = hub to tip ratio
%
% W_F / W_P =< (2*sigma* (area_fan / area_prop))^(1/3)
area_prop = 0.254^2/4;

rc_low = 40e-3;
rc_high = 300e-3;
prev = 0;

while 1
    rc_mid = (rc_low + rc_high)/2;
    [mass, ~] = MassModel_SIG(sig, rc_mid, (rc_mid*htr), payload);
    area_fan = rc_mid^2*(1 - htr^2);
    WF = mass.total;
    WP = mass.prop;
   
    rc_plus = rc_mid + 0.0001;
    [mass_plus, ~] = MassModel_SIG(sig, rc_plus, (rc_plus*htr), payload);
    area_fan_plus = rc_plus^2*(1 - htr^2);
    WF_plus = mass_plus.total;
    WP_plus = mass.prop;
    
    cond = (WF/WP) - ( 2*sig*(area_fan/area_prop) )^(1/3);
    cond_plus = (WF_plus/WP_plus) - ( 2*sig*(area_fan_plus/area_prop) )^(1/3);
    
    if abs(cond-prev) < 0.001 % Tolerance
        rc = rc_mid;
        RHS = ( 2*sig*(area_fan/area_prop) )^(1/3);
        break
    elseif abs(cond_plus) < abs(cond) % Cond_plus is closer therefore rc_plus is closer therefore increase guess
        rc_low = rc_mid;
    else % Cond_plus is further away therefore rc_plus is further therefore decrease guess
        rc_high = rc_mid;
    end
    prev = cond;
end 
end