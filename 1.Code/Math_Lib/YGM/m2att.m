function att = m2att(Cnb)
 if abs(Cnb(3,2))<=0.999999
    att = [ asin(Cnb(3,2)); -atan2(Cnb(3,1),Cnb(3,3)); -atan2(Cnb(1,2),Cnb(2,2)) ];
 else
    att = [ asin(Cnb(3,2)); atan2(Cnb(1,3),Cnb(1,1)); 0 ];
 end
