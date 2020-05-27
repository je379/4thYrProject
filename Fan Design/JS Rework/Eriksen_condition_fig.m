%% Shaft power required for prop and thrust fan can obtain
n = 100;
sig_lin = linspace(0.5,2,n);
payload = linspace(1,20,n);
[sig, payload] = meshgrid(sig_lin, payload);

htr = (1/3);

for i = 1:n
    for j=1:n
        [rc(i,j), WF(i,j), RHS(i,j)] = solve_JSEcondition(payload(i,j), sig(i,j), htr);
    end
end

nd_area = (rc.^2*(1 - htr.^2)) ./ (0.254/2).^2;
nd_payload = (WF ./ payload) - 1;
% nd_rc(sig>((rc_eq+rh_mm)/(rc_eq-rh_mm))) = NaN;

figure(14); cla; contourf(nd_payload, sig, nd_area, 'LineColor', 'None'); 
grid on; box on;
xlabel('(W_F-W_P)/W_P', 'FontSize', 12);
ylabel('Exit Duct Area Ratio / \sigma', 'FontSize', 12);
colorbar; 

clear rc WF

% Plot lines of constant payload
for pload = 0:10
    for i = 1:n
        [rc((pload+1),i), WF((pload+1),i), RHS((pload+1),i)] = solve_JSEcondition(pload, sig_lin(i), htr);
    end
    hold on;
    plot((WF(pload+1)/pload - 1),sig_lin, 'r');
end
% for s = 1:n
%     [rc, WF, RHS] = solve_JSEcondition(0, siglin(s), htr);
legend('Fan Radius / Prop Radius', 'Fyling Test Bed', 'Location', 'northwest');
axis([0 1 0.5 2]);

figure(15); cla;
for line = 1:11
    plot(rc(line,:)*1000, sig_lin, 'r');
end
xlabel('Flying Test Bed r_c / mm', 'FontSize', 12);
grid on; box on; hold on;
% set(gcf, 'Position', [0 0 300 300]);