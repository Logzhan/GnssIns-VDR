function qb = QuatUpdate(qa,th)

thabs=sqrt(th(1)*th(1)+th(2)*th(2)+th(3)*th(3));

THM=zeros(4);
THM(1,2)=(-th(1));
THM(1,3)=(-th(2));
THM(1,4)=(-th(3));
THM(2,3)=(th(3));
THM(2,4)=(-th(2));
THM(3,4)=(th(1));
THM(2,1)=(th(1));
THM(3,1)=(th(2));
THM(4,1)=(th(3));
THM(3,2)=(-th(3));
THM(4,2)=(th(2));
THM(4,3)=(-th(1));

A=eye(4)*cos(thabs*0.5);

if(thabs<(1e-6))
    A=A+THM*0.5;
else
    A=A+THM*(sin(thabs*0.5)/thabs);
end
qb=A*qa;





%下面是本来的代码。为了加速，改成了上面的代码
% thabs=sqrt(th'*th);	
% THM=zeros(4,4);
% THM(1,2)=(-th(1));
% THM(1,3)=(-th(2));
% THM(1,4)=(-th(3));
% THM(2,3)=(th(3));
% THM(2,4)=(-th(2));
% THM(3,4)=(th(1));
% THM=THM-THM';
% A=eye(4)*cos(thabs*0.5);
% 
% if(thabs<(1e-6))
%     A=A+THM*0.5;
% else
%     A=A+THM*(sin(thabs*0.5)/thabs);
% end
% qb=A*qa;


