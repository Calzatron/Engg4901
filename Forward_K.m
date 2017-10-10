function [ P_out ] = Forward_K( T, P_in, ang )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    syms q1 q2 q3 q4 q5 q6 real

    
    for a = 1:size(ang,2)
        ang(a) = ang(a)*pi/180;
    end
    T = subs(T, [q1, q2, q3, q4, q5, q6], ang);

    P_out = eval(T*P_in);


end



