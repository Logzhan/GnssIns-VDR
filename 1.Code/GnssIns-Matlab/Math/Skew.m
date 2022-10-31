%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Epipolar Geometry Toolbox  (EGT) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% function f_skew(vett)
%
% Descr: 
% -----  Computes the skew matrix of a vector "vett".
%
function A = Skew(vett)
   A = zeros(3,3);
   A(2) =  vett(3);
   A(3) = -vett(2);
   A(4) = -vett(3);
   A(6) =  vett(1);
   A(7) =  vett(2);
   A(8) = -vett(1);
%    A=[    0,   -vett(3),   vett(2);
%        vett(3),    0,     -vett(1);
%       -vett(2), vett(1),     0];