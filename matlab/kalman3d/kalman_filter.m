function [ X_INS, P, BiasHat ] = kalman_filter( X_INS, Px_LS, Py_LS, Psi_COMP, BiasHat, H, Q, R, beta, P, P0, step)
%Kalman filter Propagation and Update
%   use the step as a string, 'propagation' or 'update'
% ******* Error Model ********
% X = A_E * X_E + B_E * N_INS
% Y = H * X_E + N_LS

    dtKalman = 0.1;
    %INS State and Kalman filter update
    if(strcmp(step,'propagation'))   
        % ********************** Filter Propagation **********************
        CS = [cos(X_INS(5)) sin(X_INS(5))];
        Ad_E = [ 
                1           0           0   0   0   dtKalman*CS(1)          -dtKalman*CS(2)             0;
                0           1           0   0   0   dtKalman*CS(2)          dtKalman*CS(1)              0;
             dtKalman       0           1   0   0   (dtKalman^2)*CS(1)*0.5  (dtKalman^2)*CS(2)*(-0.5)   0;
                0        dtKalman       0   1   0   (dtKalman^2)*CS(2)*0.5  (dtKalman^2)*CS(1)*0.5      0;
                0           0           0   0   1   0                        0                          dtKalman;
                0           0           0   0   0   1                        0                          0;
                0           0           0   0   0   0                        1                          0;
                0           0           0   0   0   0                        0                          1
            ];

        Bd_E = [ 
                dtKalman*CS(1)               -dtKalman*CS(2)                 0;
                dtKalman*CS(2)               dtKalman*CS(1)                  0;
                (dtKalman^2)*CS(1)*0.5       (dtKalman^2)*CS(2)*(-0.5)       0;
                (dtKalman^2)*CS(2)*0.5       (dtKalman^2)*CS(1)*0.5          0;
                0                            0                               dtKalman;
                0                            0                               0;
                0                            0                               0;
                0                            0                               0
                ];

        P = Ad_E * P * Ad_E' + Bd_E * Q * Bd_E' + beta*P0;
        % ***********************************************************
    else
        % ********************** Filter Update **********************
        Y_E = X_INS(3:5)  - [Px_LS; Py_LS; Psi_COMP];

        G = P * (H') * inv(H * P * (H') + R);

        P = (eye(8) - G*H) * P;

        X_E = [ zeros(5,1); BiasHat] + G * Y_E;

        X_INS = X_INS - X_E(1:5);

        BiasHat = X_E(6:8);
        % ***********************************************************
    end
end

