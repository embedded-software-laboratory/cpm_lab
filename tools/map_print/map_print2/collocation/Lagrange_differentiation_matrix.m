function D = Lagrange_differentiation_matrix(nodes, interpolation_weights)
    deltas = nodes - nodes' + eye(length(nodes));
    Q = prod(deltas, 2) ./ deltas;
    D = interpolation_weights' .* Q + diag(interpolation_weights .* (sum(Q, 2) - 2 * diag(Q)));
end