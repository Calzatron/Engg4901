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

% The position of Jaco_link4
p_initial = [ 72.33; -99.52; 480.016; 1];

px = p_initial(1);
py = p_initial(2);
pz = p_initial(3);

d1 = 197.13;
d2 = D2;
d3 = D3;

% q_3 +
q_3__ = 1 * acos( (px^2 + (pz-d1)^2 - d2^2 - d3^2) / (2*d2*d3) );  

cq_2 = (px*(d2+d3*cos(q_3__)) + d3*sin(q_3__)*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3__));
sq_2 = (-px*d3*sin(q_3__) + (d2+d3*cos(q_3__))*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3__));
q_2__ = (pi/2) - atan2(sq_2,cq_2);
q_3__*180/pi
q_2__*180/pi
% q_3 -
q_3 = 1 * acos( (px^2 + (pz-d1)^2 - d2^2 - d3^2) / (2*d2*d3) );  

cq_2 = (px*(d2+d3*cos(q_3)) + d3*sin(q_3)*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3));
sq_2 = (-px*d3*sin(q_3) + (d2+d3*cos(q_3))*(pz -d1)) / (d2^2 + d3^2 + 2*d2*d3*cos(q_3));
q_2 = (pi/2) - atan2(sq_2,cq_2);

% q_1 offset
q_1 = 4.01426;%= atan2((p_initial(2)), p_initial(1)) - asin(e2 / ((px^2 + py^2)^.5));

q_3*180/pi
q_2*180/pi

q_4 = -1.570769%pi/4;
q_5 = 0;%0.34828;
%%
tic;
% Geometrically move from point P, to R and then to S (tip coords)
Rx = (11 + 85.563*sind(30)*sin(q_2+q_3)) * sin(q_4+q_2+q_3);
Ry = 85.563*cosd(30)*cos(q_2+q_3)*cos(pi + q_4+q_2+q_3);
Rz = pz + (85.563*sind(30)*cos(abs(q_2+q_3)) - 85.563*cosd(30)*sin(abs(q_2+q_3)));
R = [Rx; Ry; Rz; 1];

S = [Rx + 202.782*sin(q_5*2/5)*sin(q_4+q_2+q_3); Ry + 202.782*sin(q_5*2/5)*cos(pi + q_4+q_2+q_3); Rz + 202.782*sin((pi+q_5)*2/5); 1];
vpa(S)
toc;
S_ = [px + ((S(1)^2 + S(2)^2)^0.5)*cos(q_1 - atan(S(2)/S(1))); py - 9.81 + ((S(1)^2 + S(2)^2)^0.5)*sin(q_1 - atan(S(2)/S(1))); S(3)]
%%
tic;
%Perform a straight forwards transformation using the known angles and DH
%to VREP relations
A_new = vpa(subs(A1*A2*A3*A4*A5*A6, [q1,q2,q3,q4,q5,q6], [ pi-(q_1), -pi/2 + (q_2+pi), pi/2 + (pi+q_3), q_4, -pi + q_5, pi]));
[ A_new(1,4) ; A_new(2,4); A_new(3,4); A_new(4,4)]
toc;


tic;
% Transformation matrices for angle effect of q_2+q_3 on joint4 and above
Rz_th = [cos(q_2+q_3), -sin(q_2+q_3), 0 0;...
                sin(q_2+q_3), cos(q_2+q_3), 0, 0;...
                0, 0, 1, 0;...
                0, 0, 0, 1];
Rotx_ai = [1 0 0 0;...
            0 cos(q_2+q_3) -sin(q_2+q_3) 0;...
            0 sin(q_2+q_3) cos(q_2+q_3) 0;...
            0 0 0 1];
        
% this method is too difficult, 4->6 is negative because of orientation of
% arm q=[0,0,0] -> A_456=[0,249.7,-191.472], need to flip base and tilt
% relative to q_2+q_3, may as well use full transformation.
A_456 = subs(-1*Rotx_ai*A4*A5*A6, [q2,q3,q4,q5,q6], [ -pi/2 + (q_2+pi), pi/2 + (pi+q_3), q_4, -pi + q_3, 0]);
S_456 = vpa([A_456(1,4) + p_initial(1); A_456(2,4) + p_initial(2); A_456(3,4) + p_initial(3)]);

S = [((S_456(1)^2 + S_456(2)^2)^0.5)*cos(q_1 + tan(S_456(2)/S_456(1))); ((S_456(1)^2 + S_456(2)^2)^0.5)*sin(q_1 + tan(S_456(2)/S_456(1))); S_456(3)];
toc;

% Matrix for defining the upright position, the DH parameters are about
% [pi, 3*pi/2, pi/2, pi, pi, 0], so the upright position requires movement
% of joints 2,3 and 5 as below.
A_up = vpa(subs(A, [q1,q2,q3,q4,q5,q6], [pi - 0, -pi/2 + pi, pi/2 + pi, 0, -pi + 0, pi]));
[A_up(1,4);A_up(2,4);A_up(3,4);A_up(4,4)];

%Notes:
%vpa(subs(A1*A2*A3*A4*A5*A6, [q1,q2,q3,q4,q5,q6], [pi - 0, -pi/2 + pi, pi/2 + 3.31633, 0, -pi + 0, pi]))
%P = A_up * [0;0;0;1];
%peter corke's toolbox has ready config qr = [3*pi/2 pi pi 0 0 0]
