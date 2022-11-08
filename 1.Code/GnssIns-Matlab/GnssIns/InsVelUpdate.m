%----------------------------------------------------------------------
% Description : ��Ԫ��ת��ת����
% Author      : loghzan
% Date        : 2022-10-31
%----------------------------------------------------------------------
function [Vn,Ve,Vd,angle,velocity,Wien,Wenn,Winn] = InsVelUpdate(ang_1,ang_2,vel_1,vel_2, Vn, Ve, Vd, ...
                        Lati,Alti, Re, Rn, cnb, cbn, sampt0, V_last, g) 

    WIE   = 7.292115e-5;           % ������ת���ٶ�

    % ��������ƺ�û��
    Vd = 0;

    Wien = [ WIE*cos(Lati) 0 -WIE*sin(Lati)]';
    Wenn = [Ve/(Re+Alti) -Vn/(Rn+Alti) -Ve*tan(Lati)/(Re+Alti) ]';
    
    Winn = Wien + Wenn;
    Winb = cnb * Winn;
    % ˫������Чת��ʸ������
    ang_1 = ang_1 - Winb * sampt0;
    ang_2 = ang_2 - Winb * sampt0;
    angle = ang_1 + ang_2;
    velocity = vel_1+vel_2;
    
    % ��������
    Vel_scull_b = velocity + 0.5*cross(angle,velocity)...
        + (2.0/3.0)*(cross(vel_1,ang_2)-cross(vel_2,ang_1));
    vel_scull_n = cbn * Vel_scull_b;
    % �������������ٶ�
    Wien2_Wenn_V = cross((2.0*Wien + Wenn),V_last);
    
    %�ٶȸ���
    Vel_update = V_last + vel_scull_n + 2.0 * sampt0 * (g - Wien2_Wenn_V );
    Vn = Vel_update(1);
    Ve = Vel_update(2);
    Vd = Vel_update(3);
end