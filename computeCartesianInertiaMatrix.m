function [ M ] = computeCartesianInertiaMatrix( B, J )
%COMPUTECARTESIANINERTIAMATRIX Summary of this function goes here
%   Detailed explanation goes here

Q = J*inv(B)*J'; % B is always invertible

b = -reshape(Q,[36,1]);
H = [Q zeros(6,30);
     zeros(6,6) Q zeros(6,24);
     zeros(6,12) Q zeros(6,18);
     zeros(6,18) Q zeros(6,12);
     zeros(6,24) Q zeros(6,6);
     zeros(6,30) Q];
 H = H*H';

Aeq = [0 1 0 0 0 0 -1 eye(1, 36-7);
       0 0 1 0 0 0 0 0 0 0 0 0 -1 eye(1,36-13);
       0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 eye(1,36-19);
       0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 eye(1,36-25);
       0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 eye(1,36-31);
       0 0 0 0 0 0 0 0 1 0 0 0 0 -1 eye(1,36-14);
       0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 -1 eye(1,36-20);
       0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 eye(1, 36-26);
       0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 eye(1,36-32);
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 -1 eye(1,36-21);
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 -1 eye(1,36-27);
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 eye(1,36-33);
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 -1 eye(1,36-28);
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 -1 eye(1,36-34);
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 -1 0];
beq = zeros(1,15);

options = qpOASES_options( 'MPC' );
m = qpOASES(H,b,Aeq,[],[],beq,beq, options);


M = reshape(m, [6,6]);


end

