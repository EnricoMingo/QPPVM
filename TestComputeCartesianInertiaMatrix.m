clear all;
clc;

J = rand(6,7);

flag = true;
B = [];
while(flag)
    B = rand(7,7);
    B = B*B';
    e = eig(B);
    for i = 1:1:7
        if e(i) < 0.
            flag = true;
            break;
        else
            flag = false;
        end
    end
end

tic
M = pinv(J*inv(B)*J')
toc

tic
M1 = computeCartesianInertiaMatrix(B,J)
toc

tic
M2 = computeCartesianInertiaMatrix_b(B,J)
toc


        