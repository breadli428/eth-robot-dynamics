function phi = rotMatToRotVec(C)
% Input: a rotation matrix C
% Output: the rotational vector which describes the rotation C
th = acos(0.5*(C(1,1)+C(2,2)+C(3,3)-1));
if (abs(th)<eps)
    n = zeros(3,1);
else
    n = 1/(2*sin(th))*[C(3,2) - C(2,3);
                       C(1,3) - C(3,1);
                       C(2,1) - C(1,2)];
end
phi = th*n;
end
