% sigma_norm.m
function val = sigma_norm(z, epsilon)
    % Implements the sigma-norm from Eq. (4)
    val = (sqrt(1 + epsilon * norm(z)^2) - 1) / epsilon;
end
