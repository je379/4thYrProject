function fom = FOM(phi, psi)
%% Return required diffusion factor and theoretical maximum FOM
% [sigma, FOM] = SIGFOM(phi psi)

sigma   = sqrt( phi .^2 ./ (2 .* psi));
fom     = sqrt( 2 .* sigma);
end