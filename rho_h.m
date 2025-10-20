% rho_h.m
function val = rho_h(z, h)
    % Implements the bump function from Eq. (5)
    if z >= 0 && z < h
        val = 1;
    elseif z >= h && z < 1
        val = 0.5 * (1 + cos(pi * (z - h) / (1 - h)));
    else
        val = 0;
    end
end