function Phi = EkfStateUpdate(dTins,atti,speed,pos,accn,Phi)
%����״̬����
Fk  = GetFk(atti,speed,pos,accn);
IFk = eye(15)+Fk*dTins;
Phi = IFk*Phi;

