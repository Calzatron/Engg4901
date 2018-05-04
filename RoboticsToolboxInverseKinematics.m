%% following the sequence of ikine.m which uses the p560 arm

mdl_jaco;

q = [pi-(0), -pi/2 + (pi), pi/2 + (pi), 0, -pi, pi]

T = jaco.fkine(q)

qi = jaco.ikine(T, 'pinv')

jaco.fkine(qi)