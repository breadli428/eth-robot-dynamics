function [ pinvA ] = pseudoInverseMat(A, lambda)
% Input: Any m-by-n matrix, and a damping factor.
% Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula

% Get the number of rows (m) and columns (n) of A
[m, n] = size(A);

% TODO: complete the computation of the pseudo-inverse.
% Hint: How should we account for both left and right pseudo-inverse forms?
if m >= n
    A_inv = (A' * A + lambda^2 * eye(n)) \ A';
else
    A_inv = A' / (A * A' + lambda^2 * eye(m));
end
pinvA = A_inv;
end
