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


%%
X = A(1,4);
Y = A(2,4);
Z = A(3,4);

syms dqdt1 dqdt2 dqdt3 dqdt4 dqdt5 dqdt6


%J = []

for j = 1:5
   J(1, j) = diff(X, q(j));
   J(2, j) = diff(Y, q(j));
   J(3, j) = diff(Z, q(j));
   %J(4, j) = 0;
   %J(5, j) = 0;
   %J(6, j) = 0;
   
end

J = simplify(J);

% J is of size 3,5, so cannot be inverted, must use pseudo-inverse

transJ = simplify(transpose(J));
Inside = transJ * J;                    % has rank of 5 -> det= 0 when size (6,6);
                                        % the math from here downwards
                                        % fails unless q_6 is ignored
                                        
Inside = simplify(Inside);              % simplified expression of J*J^T

q_1 = 0;
q_2 = 0;
q_3 = 0;
q_4 = 0;
q_5 = 0;
q_6 = 0;

z = 1

%% Need to sub into the J*J^T before inverting as there is not enough
%   memory to compute the entire inverse symbolically.
tic;

q_ = [-8.84885925, 1.846790327, 10.28148573, 4.764735222, -0.0000006894989015, pi];%[ pi/4, pi/4, pi/4, pi/4, pi/4, pi/4];
%inside = subs(Inside, [q1,q2,q3,q4,q5,q6], [ pi-(q_1), -pi/2 + (q_2+pi), pi/2 + (pi+q_3), q_4, -pi + q_5, pi])
inside = subs(Inside, q, q_);

invs = (inside)^-1;                     % (J*J^T)^(-1)

z = 2

invs = simplify(invs);                  % simplified expression of (J*J^T)^(-1)

z = 3

subbedTransJ = subs(transJ, q, q_)
J_inv = invs * subbedTransJ;


subbedA = subs(A, q, q_);
x = vpa([subbedA(1,4); subbedA(2,4); subbedA(3,4)]);

delta_x = [0; 0; 3];

delta_angles = J_inv * delta_x;
delta_angles = vpa(delta_angles);

%for k = 1:5
%    delta_angles(k) = mod(delta_angles(k), 2*pi);
%    delta_angles(k) = vpa(delta_angles(k));
%end

Final = subs(A, [q1,q2,q3,q4,q5,q6], [ q_(1) + delta_angles(1), q_(2) + delta_angles(2), q_(3) + delta_angles(3), q_(4) + delta_angles(4), q_(5) + delta_angles(5), pi]);

%check that x matches last column of Final
V = vpa([Final(1,4); Final(2,4); Final(3,4)]);

toc;
