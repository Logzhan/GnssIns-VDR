function [ q ] = QuatUpdate2(q, TV)
NS = TV' * TV ;
if   NS<1.0e-8
    dM = [0 -TV(1) -TV(2) -TV(3)
        TV(1) 0 TV(3) -TV(2)
        TV(2) -TV(3) 0 TV(1)
        TV(3) TV(2) -TV(1) 0];
    % cos(n/2)=1-n2/8+n4/384; sin(n/2)/n=1/2-n2/48+n4/3840
    % �²�QM��һ�ֽ��ͼ������ķ�������Ϊ����sin(theta)��cos(theta)
    % ����������С��ʱ�򶼿��Լ򵥷�ʽ���Ի�������Ƕ��ʽ���ԣ�����
    % sin��cos��Ϊ�ķ���Դ
    QM = (1-NS/8.0+NS^2/384.0)*eye(4)+(0.5-NS/48.0)*dM;
    q = QM* q;
    q = q/norm(q);
else
    % ��תʸ��������Ԫ��
    Delta_Re = cos(norm(TV)/2);
    Delta_Im = sin(norm(TV)/2)/norm(TV) * TV;
    % ��תʸ��ת��Ϊ��Ԫ��
    q_new= [Delta_Re Delta_Im']';
    %����Ԫ����дΪ������ʽ,������һ������Ԫ���˷�
    Q_matrix = [  q(1)     -q(2)     -q(3)      -q(4)
        q(2)      q(1)     -q(4)      q(3)
        q(3)     q(4)      q(1)      -q(2)
        q(4)    -q(3)      q(2)       q(1)];
    q = Q_matrix* q_new;
    q = q/norm(q);
end
end

