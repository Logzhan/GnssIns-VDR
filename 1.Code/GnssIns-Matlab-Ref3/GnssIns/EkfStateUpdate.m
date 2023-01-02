function Phi = EkfStateUpdate(dTins,atti,speed,pos,accn,Phi)
%¸üÐÂ×´Ì¬¾ØÕó
Fk  = GetFk(atti,speed,pos,accn);
IFk = eye(15)+Fk*dTins;
Phi = IFk*Phi;

