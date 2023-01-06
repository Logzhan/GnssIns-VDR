function Phi=stateupdate(dTins,atti,speed,pos,accn,Phi)
%¸üĞÂ×´Ì¬¾ØÕó

Fk=getfk(atti,speed,pos,accn);
IFk=eye(15)+Fk*dTins;
Phi=IFk*Phi;

