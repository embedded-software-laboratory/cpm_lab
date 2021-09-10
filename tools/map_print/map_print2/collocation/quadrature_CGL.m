function [x,w] = quadrature_CGL(N)


    % Chebyshev-Gauss-Lobatto nodes
    x=vpa(cos(pi*(0:N)/N)');

    w = nan(size(x));
    x = flipud(x);

end