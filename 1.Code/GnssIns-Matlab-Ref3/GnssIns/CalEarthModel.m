function [Rmeri,Rprim,ge] = CalEarthModel(pos)
Re=6378137;
ee=1/298.25722;

lat = pos(1);
h   = pos(3);

sin_phi=sin(lat);
sin2_phi=sin_phi*sin_phi;

Rprim=Re*(1+ee*sin2_phi);
Rmeri=Re*(1-2*ee+3*ee*sin2_phi);

sl=sin(lat);  s2l=sin(2*lat);  sl2=sl^2; s2l2=s2l^2;
g0 = 9.7803267714;  
ge = g0*(1+0.0053024*sl2-0.0000059*s2l2)/(1+pos(3)/Re)^2;






