function w = Lagrange_interpolation_weights(nodes)
    assert(size(nodes, 2) == 1);
    P = nodes - nodes' + eye(length(nodes));
    w = 1./prod(P,2);
end