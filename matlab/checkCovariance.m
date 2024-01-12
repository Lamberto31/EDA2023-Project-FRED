function [isSymm,isSemidefpos] = checkCovariance(P)
%checkCovariance Check if matrix P is symmetric and semidefinite positive
%   Check if a covariance matrix P respect the condition of being symmetric
%   and semidefinite positive. This function works only for 2x2 matrices.

% Check if symmetric
% isSymm = issymmetric(P);
if abs(P(1,2) - P(2,1)) < eps
    isSymm = true;
end

% Check if semidefinite positive
eigenvalues = eig(P);
if any(eigenvalues < 0)
    isSemidefpos = false;
else
    isSemidefpos = true;

end