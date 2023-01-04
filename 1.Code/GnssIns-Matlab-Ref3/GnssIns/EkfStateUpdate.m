function Phi = EkfStateUpdate(InsDt,Qnb,Vel,Pos,Accn,Phi)
%¸üÐÂ×´Ì¬¾ØÕó
Fk  = GetFk(Qnb,Vel,Pos,Accn);
IFk = eye(15) + Fk*InsDt;
Phi = IFk*Phi;

