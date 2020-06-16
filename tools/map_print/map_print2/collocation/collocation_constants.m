function [nodes, w_integration, D, w_interp] = collocation_constants(node_name, N, a, b, as_sym)


    
    if strcmp(node_name, 'LGL')
        [nodes, w_integration] = quadrature_LGL(N);
    elseif strcmp(node_name, 'CGL')
        [nodes, w_integration] = quadrature_CGL(N);
    end
    
    assert(all(diff(double(nodes))>0));
    %assert(abs(sum(double(w_integration)) - 2) < 1e-14);

    
    nodes = 0.5*(b-a)*(nodes+1) + a;
    w_integration = w_integration * 0.5 * (b-a);

    w_interp = Lagrange_interpolation_weights(nodes);
    D = Lagrange_differentiation_matrix(nodes, w_interp);
    
    if nargin ~= 5 || ~as_sym
        w_integration = double(w_integration);
        nodes = double(nodes);
        D = double(D);
        w_interp = double(w_interp);
    end

end