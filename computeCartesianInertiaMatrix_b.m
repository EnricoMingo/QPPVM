function [ M ] = computeCartesianInertiaMatrix_b( B, J )
%COMPUTECARTESIANINERTIAMATRIX Summary of this function goes here
%   Detailed explanation goes here

Q = J*inv(B)*J'; % B is always invertible

b = -[1; 0; 0; 0; 0; 0;
      0; 1; 0; 0; 0; 0;
      0; 0; 1; 0; 0; 0;
      0; 0; 0; 1; 0; 0;
      0; 0; 0; 0; 1; 0;
      0; 0; 0; 0; 0; 1];

H = [Q zeros(6,30);
     zeros(6,6) Q zeros(6,24);
     zeros(6,12) Q zeros(6,18);
     zeros(6,18) Q zeros(6,12);
     zeros(6,24) Q zeros(6,6);
     zeros(6,30) Q];


 Aeq = [];
 for i = 1:1:6
     for j = i+1:1:6
         aeq = zeros(1,36);
         aeq(i+6*(j-1)) = 1.;
         aeq(j+6*(i-1)) = -1.;
         Aeq = [Aeq; aeq];
     end
 end
 
 beq = zeros(1,15);

options = qpOASES_options( 'reliable' );
options.numRegularisationSteps = 2;
options.epsRegularisation = 1e-2;
m = qpOASES(H,b,Aeq,[],[],beq',beq', options);


M = reshape(m, [6,6]);


end

