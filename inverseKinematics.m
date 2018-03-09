%% Forward component
D1 = 197.13;%275.5;
D2 = 410;
D3 = 207.3;
D4 = 74.1;
D5 = 74.1;
D6 = 160.0;
e2 = 9.8;
aa = 60*pi/360;

alpha = [pi/2 pi pi/2 (60*pi/180) (60*pi/180) pi];
a = [0 D2 0 0 0 0];
d = [D1, 0, -e2, -(D3 + D4*sin(aa)/sin(2*aa)), -(D4*sin(aa)/sin(2*aa) + D5*sin(aa)/sin(2*aa)), -(D5*sin(aa)/sin(2*aa) + D6)];

syms q1 q2 q3 q4 q5 q6;

q = [q1 q2 q3 q4 q5 q6];

A1 = [];
A2 = [];
A3 = [];
A4 = [];
A5 = [];
A6 = [];

for i = 1:size(d,2)
    
    Rz_th = [cos(q(i)), -sin(q(i)), 0 0;...
                sin(q(i)), cos(q(i)), 0, 0;...
                0, 0, 1, 0;...
                0, 0, 0, 1];
    Tz_di = [1 0 0 0;...
            0 1 0 0;...
            0 0 1 d(i);...
            0 0 0 1];
    Tx_ai = [1 0 0 a(i);...
            0 1 0 0;...
            0 0 1 0;...
            0 0 0 1];
    Rotx_ai = [1 0 0 0;...
                0 cos(alpha(i)) -sin(alpha(i)) 0;...
                0 sin(alpha(i)) cos(alpha(i)) 0;...
                0 0 0 1];
    A_i = Rz_th * Tz_di * Tx_ai * Rotx_ai;
    
    
    for m = 1:4
        for n = 1:4
            [C, T] = coeffs(vpa(A_i(m,n)));
            for x  = 1:size(C,2)
                if abs(C(x)) < 0.000001
                    C(x) = 0;
                end
            end
            A_i(m,n) = dot(C, T);
        end
    end
    
    
    
    if i == 1
        A1 = A_i;
    elseif i == 2
        A2 = A_i;
    elseif i == 3
        A3 = A_i;
    elseif i == 4
        A4 = A_i;
    elseif i == 5
        A5 = A_i;
    elseif i == 6
        A6 = A_i;
    end

    
end
A = A1*A2*A3*A4*A5*A6;
%% Simplify terms
A_56 = A5*A6;
A_56 = collect(A_56, [cos(q1), cos(q2), cos(q3), cos(q4), cos(q5), cos(q6), sin(q1), sin(q2), sin(q3), sin(q4), sin(q5), sin(q6)]);

A_inv4_rhs = (A1*A2*A3*A4)^(-1)*A;
A_inv4_rhs = collect(A_inv4_rhs, [cos(q1), cos(q2), cos(q3), cos(q4), cos(q5), cos(q6), sin(q1), sin(q2), sin(q3), sin(q4), sin(q5), sin(q6)]);

A_col = collect(A, [cos(q1), cos(q2), cos(q3), cos(q4), cos(q5), cos(q6), sin(q1), sin(q2), sin(q3), sin(q4), sin(q5), sin(q6)]);

%% Matlab implementation of moving point to point %%

p_initial = [ 0; 9.81; 814.4; 1];

P_next = [ 1000-774.7; 9.81; 762.8; 1];

% px = P_next(1);
% py = P_next(2);
% pz = P_next(3);
px = p_initial(1);
py = p_initial(2);
pz = p_initial(3);

d1 = 197.13;
d2 = D2;
d3 = D3;

q_3 = -1 * acos( (px^2 + (pz-d1)^2 - d2^2 - d3^2) / (2*d2*d3) );  

cq_2 = (px*(d2+d3*cos(q_3)) + d3*sin(q_3)*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3));
sq_2 = (-px*d3*sin(q_3) + (d2+d3*cos(q_3))*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3));
q_2 = (pi/2) - atan2(sq_2,cq_2);

q_1 = tan((P_next(2))/P_next(1));

q_3*180/pi
q_2*180/pi

q_4 = pi/4;%3*pi/2;

Rx = (px + 85.563*cosd(30)*cos(q_2+q_3) + 85.563*sind(30)*sin(q_2*q_3)) * sin(q_4);
Ry = 85.563*cosd(30)*cos(q_2+q_3)*cos(q_4) + py - 9.81;
Rz = pz + 85.563*sind(30)*cos(q_2+q_3) - 85.563*cosd(30)*sin(q_2+q_3);

R = [Rx; Ry; Rz; 1]

q_5 = pi+ pi/4;

% 1000-x added to reflect vrep real-world coords
S = [1000 + (Rx + 202.782*sin(q_5*2/5)*sin(q_4)); Ry + 202.782*sin(q_5*2/5)*cos(q_4+pi); Rz + 202.782*cos(q_5*2/5); 1]


A_new = vpa(subs(A, [q1,q2,q3,q4,q5,q6], [pi - q_1, pi/2 - q_2, -pi/2 - q_3, pi + q_4, pi + q_5, 0]));

[ A_new(1,4)+1000 ;A_new(2,4);A_new(3,4);A_new(4,4)]

A_up = vpa(subs(A, [q1,q2,q3,q4,q5,q6], [pi, pi/2, -pi/2, pi, pi, 0]));
[A_up(1,4);A_up(2,4);A_up(3,4);A_up(4,4)]


%P = A_up * [0;0;0;1];
%peter corke's toolbox has ready config qr = [3*pi/2 pi pi 0 0 0]