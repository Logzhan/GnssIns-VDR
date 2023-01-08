function [X,P] = EkfStateUpdate(Z,H,P,R,X)
    K = P * H'/(H * P * H' + R);
    X = X + K * (Z - H * X);
    P = (eye(15, 15) - K * H) * P * (eye(15, 15) - K * H)' + K * R * K';
end

