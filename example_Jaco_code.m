
JACO_D_H = [0 0 0 8.2755;  -pi/2 0 0 0; 0 16.1417 0 0; -pi/2 0 0 9.8149; 0.96 0 0 3.3307; 0.96 0 0 8.9488];
IA = [0 -pi/3 pi/4 0 0 0];

D_H = JACO_D_H

L{1}=Link([D_H(1,1:4) 0])%, 'mod');
L{2}=Link([D_H(2,1:4) 0])%, 'mod');
L{3}=Link([D_H(3,1:4) 0])%, 'mod');
L{4}=Link([D_H(4,1:4) 0])%, 'mod');
L{5}=Link([D_H(5,1:4) 0])%, 'mod');
L{6}=Link([D_H(6,1:4) 0])%, 'mod');

Robot = robot(L, 'JACO');
IP = fkine(Robot, IA);

points=[12.5400 10.0000 -14.367;
12.5400 -3.5000 -14.367;
12.5400 -13.500 -14.367;
12.5400 10.0000 -7.367;
12.5400 -3.5000 -7.367;
12.5400 -13.500 -7.367;
12.5400 10.0000 1.633;
12.5400 -3.5000 1.633;
12.5400 -13.500 1.633;
12.5400 10.0000 9.613;
12.5400 -3.5000 9.613;
12.5400 -13.500 9.613;
12.5400 10.0000 14.633;
12.5400 -3.5000 14.633;
12.5400 -13.500 14.633;
12.5400 10.0000 21.613;
12.5400 -3.5000 21.613;
12.5400 -13.500 21.613;
12.5400 10.0000 33.633;
12.5400 -3.5000 33.633;
12.5400 -13.500 33.633;
12.5400 10.0000 39.613;
12.5400 -3.5000 39.613;
12.5400 -13.500 39.613;
-1.0000 10.0000 -14.367;
-1.0000 -3.5000 -14.367;
-1.0000 -13.500 -14.367;
-1.0000 10.0000 -7.367;
-1.0000 -3.5000 -7.367;
-1.0000 -13.500 -7.367;
-1.0000 10.0000 1.633;
-1.0000 -3.5000 1.633;
-1.0000 -13.500 1.633;
-1.0000 10.0000 9.613;
-1.0000 -3.5000 9.613;
-1.0000 -13.500 9.613;
-1.0000 10.0000 14.633;
-1.0000 -3.5000 14.633;
-1.0000 -13.500 14.633;
-1.0000 10.0000 21.613;
-1.0000 -3.5000 21.613;
-1.0000 -13.500 21.613;
-1.0000 10.0000 33.633;
-1.0000 -3.5000 33.633;
-1.0000 -13.500 33.633;
-1.0000 10.0000 39.613;
-1.0000 -3.5000 39.613;
-1.0000 -13.500 39.613;
-8.0000 10.0000 -14.367;
-8.0000 -3.5000 -14.367;
-8.0000 -13.500 -14.367;
-8.0000 10.0000 -7.367;
-8.0000 -3.5000 -7.367;
-8.0000 -13.500 -7.367;
-8.0000 10.0000 1.633;
-8.0000 -3.5000 1.633;
-8.0000 -13.500 1.633;
-8.0000 10.0000 9.613;
-8.0000 -3.5000 9.613;
-8.0000 -13.500 9.613;
-8.0000 10.0000 14.633;
-8.0000 -3.5000 14.633;
-8.0000 -13.500 14.633;
-8.0000 10.0000 21.613;
-8.0000 -3.5000 21.613;
-8.0000 -13.500 21.613;
-8.0000 10.0000 33.633;
-8.0000 -3.5000 33.633;
-8.0000 -13.500 33.633;
-8.0000 10.0000 39.613;
-8.0000 -3.5000 39.613;
-8.0000 -13.500 39.613;
-14.0000 10.0000 -14.367;
-14.0000 -3.5000 -14.367;
-14.0000 -13.500 -14.367;
-14.0000 10.0000 -7.367;
-14.0000 -3.5000 -7.367;
-14.0000 -13.500 -7.367;
-14.0000 10.0000 1.633;
-14.0000 -3.5000 1.633;
-14.0000 -13.500 1.633;
-14.0000 10.0000 9.613;
-14.0000 -3.5000 9.613;
-14.0000 -13.500 9.613;
-14.0000 10.0000 14.633;
-14.0000 -3.5000 14.633;
-14.0000 -13.500 14.633;
-14.0000 10.0000 21.613;
-14.0000 -3.5000 21.613;
-14.0000 -13.500 21.613;
-14.0000 10.0000 33.633;
-14.0000 -3.5000 33.633;
-14.0000 -13.500 33.633;
-14.0000 10.0000 39.613;
-14.0000 -3.5000 39.613;
-14.0000 -13.500 39.613;
-15.0000 4.2500 15.4000;
-15.0000 6.0000 15.4000;
-15.0000 10.0000 15.4000;
-15.0000 14.0000 15.4000;
-15.0000 16.7500 15.4000;
-15.0000 -3.5000 15.4000;
-15.0000 -13.500 15.4000;
-15.0000 4.2500 28.900;
-15.0000 6.0000 28.900;
-15.0000 10.0000 28.900;
-15.0000 14.0000 28.900;
-15.0000 16.7500 28.900;
-15.0000 -3.5000 28.900;
-15.0000 -13.500 28.900;
-15.0000 4.2500 31.9000;
-15.0000 6.0000 31.9000;
-15.0000 10.0000 31.9000;
-15.0000 14.0000 31.9000;
-15.0000 16.7500 31.9000;
-15.0000 -3.5000 31.9000;
-15.0000 -13.500 31.9000;
-19.0000 4.2500 28.900;
-19.0000 6.0000 28.900;
-19.0000 10.0000 28.900;
-19.0000 14.0000 28.900;
-19.0000 16.7500 28.900;
-19.0000 -3.5000 28.900;
-19.0000 -13.500 28.900;
-19.0000 4.2500 31.9000;
-19.0000 6.0000 31.9000;
-19.0000 10.0000 31.9000;
-19.0000 14.0000 31.9000;
-19.0000 16.7500 31.9000;
-19.0000 -3.5000 31.9000;
-19.0000 -13.500 31.9000];

l=size(points);
p=l(:,1);

FP=[1 0 0 30;
 0 1 0 10;
 0 0 1 15;
 0 0 0 1];

plot(Robot,IA);
axis([-20 40 -40 20 -20 40])
hold on;
plot3([IP(1,4), FP(1,4)],[IP(2,4), FP(2,4)],[IP(3,4), FP(3,4)],'-b');
view(40,15);

set(figure (1),'WindowStyle','docked');
% drivebot(Robot);
M=[1 1 1 0 0 0];
TC=ctraj(IP,FP,100);

angles=ikine_Johnny(Robot,TC,IA,M)
Manip=maniplty(Robot,angles);
for i=1:99
 i
 angles1=angles(i,:);
 angles2=angles(i+1,:);
 delta_angles=(angles2(1,1)-angles1(1,1))^2+(angles2(1,2)-...
        angles1(1,2))^2+(angles2(1,3)-angles1(1,3))^2+(angles2(1,4)-...
        angles1(1,4))^2+(angles2(1,5)-angles1(1,5))^2+(angles2(1,6)-...
        angles1(1,6))^2;
 if delta_angles>1
    disp('Singulrty cat iz in your manipz makin dem useless')
    %Manip=0;
    flag=1;
    fprintf('Manipulability = 0\r')
     fprintf('Flag = 1\r');
    break
 end
end
pause(0.2);
plot(Robot,angles);



% %IKINE Inverse manipulator kinematics
% %
% %154
% %Appendix A Continued
% % Q = IKINE(ROBOT, T)
% % Q = IKINE(ROBOT, T, Q)
% % Q = IKINE(ROBOT, T, Q, M)
% %
% % Returns the joint coordinates corresponding to the end-effector
% %transform T.
% % Note that the inverse kinematic solution is generally not unique, and
% % depends on the initial guess Q (which defaults to 0).
% %
% % QT = IKINE(ROBOT, TG)
% % QT = IKINE(ROBOT, TG, Q)
% % QT = IKINE(ROBOT, TG, Q, M)
% %
% % Returns the joint coordinates corresponding to each of the transforms
% %in
% % the 4x4xN trajectory TG.
% % Returns one row of QT for each input transform. The initial estimate
% % of QT for each time step is taken as the solution from the previous
% % time step.
% %
% % If the manipulator has fewer than 6 DOF then this method of solution
% % will fail, since the solution space has more dimensions than can
% % be spanned by the manipulator joint coordinates. In such a case
% % it is necessary to provide a mask matrix, M, which specifies the
% % Cartesian DOF (in the wrist coordinate frame) that will be ignored
% % in reaching a solution. The mask matrix has six elements that
% % correspond to translation in X, Y and Z, and rotation about X, Y and
% % Z respectively. The value should be 0 (for ignore) or 1. The number
% % of non-zero elements should equal the number of manipulator DOF.
% 
% % Solution is computed iteratively using the pseudo-inverse of the
% % manipulator Jacobian.
% %
% % Such a solution is completely general, though much less efficient
% % than specific inverse kinematic solutions derived symbolically.
% %
% % This approach allows a solution to obtained at a singularity, but
% % the joint angles within the null space are arbitrarily assigned.
% %
% % For instance with a typical 5 DOF manipulator one would ignore
% % rotation about the wrist axis, that is, M = [1 1 1 1 1 0].
% %
% %
% % See also: FKINE, TR2DIFF, JACOB0, IKINE560.
% % Copyright (C) 1993-2002, by Peter I. Corke
% % MOD.HISTORY
% % 2/95 use new 2-argument version of tr2diff(), cleanup
% % 3/99 uses objects
% % 6/99 initialize qt before loop
% % 2/01 remove inv(base) xform, since it is included in fkine
% % 10/01 bug in mask for <6 axes
% % $Log: ikine.m,v $
% %155
% %Appendix A Continued
% % Revision 1.4 2002/04/14 10:15:41 pic
% % Fixed error message text.
% %
% % Revision 1.3 2002/04/01 11:47:13 pic
% % General cleanup of code: help comments, see also, copyright, remnant D_H/dyn
% % references, clarification of functions.
% %
% % $Revision: 1.4 $
% 
% 
% 
% function [qt, jaco] = ikine(robot, tr, q, m)
%     %
%     % solution control parameters
%     %
%     ilimit = 1000;
%     stol = 1e-4;
%     n = robot.n;
%     if nargin == 2,
%         q = zeros(n, 1);
%     else
%         q = q(:);
%     end
%     if nargin == 4,
%         m = m(:);
%         if length(m) ~= 6,
%             error('Mask matrix should have 6 elements');
%         end
%     % if length(find(m)) ~= robot.n
%     % error('Mask matrix must have same number of 1s as robot
%     % DOF')
%     % end
%     else
%         if n < 6,
%             disp('For a manipulator with fewer than 6DOF a mask matrix argument should be specified');
%         end
%         m = ones(6, 1);
%     end
% 
%     tcount = 0;
%     if ishomog(tr), % single xform case
%     nm = 1;
%         count = 0;
%         while nm > stol,
%             e = tr2diff(fkine(robot, q'), tr) .* m;
%             dq = pinv( jacob0(robot, q) ) * e;
%             q = q + dq;
%             nm = norm(dq);
%             count = count+1;
%             if count > ilimit,
% 
%                 error('Solution wouldn''t converge')
%             end
%         end
%         qt = q';
%     else % trajectory case
%         np = size(tr,3);
%         qt = [];
%         jaco = [];
%         for i=1:np
%             nm = 1;
%             T = tr(:,:,i);
%             count = 0;
%             %while nm > stol,
%             e = tr2diff(fkine(robot, q'), T) .* m;
%             jaco = (jacob0(robot, q));
%             dq = pinv( jaco(1:3,:) ) * e(1:3);
%             q = q + dq;
%             nm = norm(dq);
%             count = count+1;
%             if count > ilimit,
%                 fprintf('i=%d, nm=%f\n', i, nm);
%                 error('Solution wouldn''t converge')
%             end
%  %end
% 
%             qt = [qt; q'];
%             tcount = tcount + count;
%         end
%     end
% 






