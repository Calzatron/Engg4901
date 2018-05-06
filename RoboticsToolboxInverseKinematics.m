%% following the sequence of ikine.m which uses the p560 arm

mdl_jaco;

q = [pi-(0), -pi/2 + (pi), pi/2 + (pi), 0, -pi, pi]

T = jaco.fkine(q)

qi = jaco.ikine(T, 'pinv')

jaco.fkine(qi)


%% Move the arm to a specific location and angle
T = transl(0.3, 0.1, 0) * rpy2tr(0, 0, 0, 'deg')
% plot the new coordinate

%calculate the angles to move
q = jaco.ikine(T, qz, [1 1 1 0 0 0]);   % initial angles, and xyz coordinate match
% show angles
jaco.plot(q);%*pi/180)
hold on;
trplot(T)