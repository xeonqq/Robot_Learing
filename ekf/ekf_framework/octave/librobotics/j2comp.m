%J2COMP Second Jacobian of the compound operator.
%   J = J2COMP(XI,XJ) returns the Jacobian matrix of the 2D composition
%   of XI and XJ derived with respect to the second operand. All X's are
%   3x1-vectors, J is a 3x3 matrix.
%
%   See also J1COMP, JINV, COMPOUND, ICOMPOUND.

% v.1.0, 30.11.02, Kai Arras, ASL-EPFL


function J2 = j2comp(xij,xjk);

J2 = [cos(xij(3)), -sin(xij(3)), 0;
      sin(xij(3)),  cos(xij(3)), 0;
          0,             0,      1];
