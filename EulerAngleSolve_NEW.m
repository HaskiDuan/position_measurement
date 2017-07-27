%由方向余弦矩阵求解欧老角
%输入：方向余弦矩阵rotation_matrix
%输出：三个欧拉角
function [theta,phi,psi] = EulerAngleSolve_NEW(rotation_matrix)
    R11 = rotation_matrix(1,1);     R12 = rotation_matrix(1,2);    R13 = rotation_matrix(1,3);
    R21 = rotation_matrix(2,1);     R22 = rotation_matrix(2,2);    R23 = rotation_matrix(2,3);
    R31 = rotation_matrix(3,1);     R32 = rotation_matrix(3,2);    R33 = rotation_matrix(3,3);
  
        theta = asin(R23);   %theta : -pi/2 ~ pi/2
       if(pi/2 < theta && theta < pi*1.5)
            theta = pi - theta;
        elseif(theta > pi*1.5)
            theta = theta -2*pi;
        end
       if(-pi/2 > theta && theta > -pi*1.5)
            theta = -pi -theta;
       elseif(theta < -pi*1.5)
           theta = theta + 2*pi;
       end
        
        phi = -atan(R13/R33);  %phi : -pi/2 ~ pi/2
       if(phi> 0.5*pi)
           phi= phi - pi;
       end
       if(phi< -0.5*pi)
           phi = phi + pi;
       end
        
        psi = -atan(R21/R22);
        if (R21>0)
            if(psi > 0)
                psi = -psi;
            end
        elseif(R21<0)
            if(psi < 0)
                psi = -psi;
            end
        else
            psi = 0;
        end  
        if(psi>pi)
           psi = psi -pi;
        end
       if (psi < -pi)
            psi = psi + pi;
       end
end