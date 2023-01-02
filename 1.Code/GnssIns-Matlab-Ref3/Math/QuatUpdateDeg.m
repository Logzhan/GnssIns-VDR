function qb = QuatUpdateDeg(qa,xdeg,ydeg,zdeg)
qb = QuatUpdate(qa,[xdeg;ydeg;zdeg]*pi/180);
end