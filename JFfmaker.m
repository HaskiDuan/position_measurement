function [Jacobian,F,f]= JFfmaker(theta,phi,psi,T1i,T2i,T3i,worldxy,pictureuv,camera)
%����������ڼ���Jacobian����
%[theta,phi,psi,T1i,T2i,T3i]�Ǵ��Ż������ĳ�ʼֵ
%worldxy�洢������������������ϵ�е�����  3 x 15 ��ʹ��ǰ15����������з������Ż���
%pictureuv�洢����������ͼ����������ϵ�е�����  3 x 15  ��ʹ��ǰ15����������з������Ż���
%camera ����ͨ���궨�õ���������ڲξ���
point_number = 15; % ��8��������ݽ��з������Ż�
pictureuv2 = camera\pictureuv;   % pictureuv2 = inv(camera) * pictureuv; ����ͼ����������ϵ������һ�α任
                                                      % pitcureuv2 : 3 x 15
I = eye(3,3);   %3 x 3��λ����
for i = 1:1:point_number
    M = pictureuv2(:,i)*[0 0 1] - I;    % ��Ӧ�Ź�ʽ��M = pictureuv2(i) - I
    Jacobian(3*i-2,1) = worldxy(1,i) * (   M(1,1)*(-sin(phi)*sin(psi)*cos(theta)) + M(1,2)*(sin(psi)*sin(theta)) + M(1,3)*(cos(phi)*sin(psi)*cos(theta))  )...
                                 + worldxy(2,i) * (   M(1,1)*(cos(psi)*sin(phi)*cos(theta)) + M(1,2)*(-cos(psi)*sin(theta)) + M(1,3)*(-cos(psi)*cos(phi)*cos(theta)) );
    Jacobian(3*i-2,2) = worldxy(1,i) * (   M(1,1)*(-sin(phi)*cos(psi) - cos(phi)*sin(psi)*sin(theta)) + M(1,3)*(cos(psi)*cos(phi) - sin(phi)*sin(psi)*sin(theta))  )...
                                 + worldxy(2,i) * (   M(1,1)*(-sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + M(1,3)*(sin(psi)*cos(phi) + cos(psi)*sin(phi)*sin(theta)) );     
    Jacobian(3*i-2,3) = worldxy(1,i) * (   M(1,1)*(-cos(phi)*sin(psi) - sin(phi)*cos(psi)*sin(theta)) - M(1,2)*(cos(psi)*cos(theta)) + M(1,3)*(-sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))  )...
                                 + worldxy(2,i) * (   M(1,1)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - M(1,2)*(sin(psi)*cos(theta)) + M(1,3)*(cos(psi)*sin(phi) + sin(psi)*cos(phi)*sin(theta)) );
    Jacobian(3*i-2,4) = M(1,1);
    Jacobian(3*i-2,5) = M(1,2);
    Jacobian(3*i-2,6) = M(1,3);
  
    Jacobian(3*i-1,1) = worldxy(1,i) * (   M(2,1)*(-sin(phi)*sin(psi)*cos(theta)) + M(2,2)*(sin(psi)*sin(theta)) + M(2,3)*(cos(phi)*sin(psi)*cos(theta))  )...
                                 + worldxy(2,i) * (   M(2,1)*(cos(psi)*sin(phi)*cos(theta)) + M(2,2)*(-cos(psi)*sin(theta)) + M(2,3)*(-cos(psi)*cos(phi)*cos(theta)) );    
    Jacobian(3*i-1,2) = worldxy(1,i) * (   M(2,1)*(-sin(phi)*cos(psi) - cos(phi)*sin(psi)*sin(theta)) + M(2,3)*(cos(psi)*cos(phi) - sin(phi)*sin(psi)*sin(theta))  )...
                                 + worldxy(2,i) * (   M(2,1)*(-sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + M(2,3)*(sin(psi)*cos(phi) + cos(psi)*sin(phi)*sin(theta)) );    
    Jacobian(3*i-1,3) = worldxy(1,i) * (   M(2,1)*(-cos(phi)*sin(psi) - sin(phi)*cos(psi)*sin(theta)) - M(2,2)*(cos(psi)*cos(theta)) + M(2,3)*(-sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))  )...
                                 + worldxy(2,i) * (   M(2,1)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - M(2,2)*(sin(psi)*cos(theta)) + M(2,3)*(cos(psi)*sin(phi) + sin(psi)*cos(phi)*sin(theta)) );
    Jacobian(3*i-1,4) = M(2,1);
    Jacobian(3*i-1,5) = M(2,2);
    Jacobian(3*i-1,6) = M(2,3);
    
    Jacobian(3*i,1) = worldxy(1,i) * (   M(3,1)*(-sin(phi)*sin(psi)*cos(theta)) + M(3,2)*(sin(psi)*sin(theta)) + M(3,3)*(cos(phi)*sin(psi)*cos(theta))  )...
                              + worldxy(2,i) * (   M(3,1)*(cos(psi)*sin(phi)*cos(theta)) + M(3,2)*(-cos(psi)*sin(theta)) + M(3,3)*(-cos(psi)*cos(phi)*cos(theta)) );    
    Jacobian(3*i,2) = worldxy(1,i) * (   M(3,1)*(-sin(phi)*cos(psi) - cos(phi)*sin(psi)*sin(theta)) + M(3,3)*(cos(psi)*cos(phi) - sin(phi)*sin(psi)*sin(theta))  )...
                              + worldxy(2,i) * (   M(3,1)*(-sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + M(3,3)*(sin(psi)*cos(phi) + cos(psi)*sin(phi)*sin(theta)) );    
    Jacobian(3*i,3) = worldxy(1,i) * (   M(3,1)*(-cos(phi)*sin(psi) - sin(phi)*cos(psi)*sin(theta)) - M(3,2)*(cos(psi)*cos(theta)) + M(3,3)*(-sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))  )...
                              + worldxy(2,i) * (   M(3,1)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - M(3,2)*(sin(psi)*cos(theta)) + M(3,3)*(cos(psi)*sin(phi) + sin(psi)*cos(phi)*sin(theta)) );
    Jacobian(3*i,4) = M(3,1);
    Jacobian(3*i,5) = M(3,2);
    Jacobian(3*i,6) = M(3,3);
    
    f(3*i - 2) = worldxy(1,i)*(   M(1,1)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - M(1,2)*(sin(psi)*cos(theta)) + M(1,3)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))   ) ...
                   + worldxy(2,i)*(  M(1,1)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + M(1,2)*(cos(psi)*cos(theta)) + M(1,3)*(sin(phi)*sin(psi) - cos(psi)*cos(phi)*sin(theta))  ) ...
                   + M(1,1)*T1i + M(1,2)*T2i + M(1,3)*T3i;
    f(3*i - 1) = worldxy(1,i)*(   M(2,1)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - M(2,2)*(sin(psi)*cos(theta)) + M(2,3)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))   ) ...
                   + worldxy(2,i)*(  M(2,1)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + M(2,2)*(cos(psi)*cos(theta)) + M(2,3)*(sin(phi)*sin(psi) - cos(psi)*cos(phi)*sin(theta))  ) ...
                   + M(2,1)*T1i + M(2,2)*T2i + M(2,3)*T3i;
    f(3*i) = worldxy(1,i)*(   M(3,1)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - M(3,2)*(sin(psi)*cos(theta)) + M(3,3)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))   ) ...
                   + worldxy(2,i)*(  M(3,1)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + M(3,2)*(cos(psi)*cos(theta)) + M(3,3)*(sin(phi)*sin(psi) - cos(psi)*cos(phi)*sin(theta))  ) ...
                   + M(3,1)*T1i + M(3,2)*T2i + M(3,3)*T3i;
end

F = 0;
for i = 1:1:point_number*3
    F = f(i)^2/2 + F;
end
f = f';

