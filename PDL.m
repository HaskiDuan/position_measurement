%���룺theta,phi,psi,T10,T20,T30 �����Ż���6������
%           worldxy������������������ϵ�е�����
%          pictureuv����������ͼ����������ϵ�е�����
%          camera����������ڲξ���
%�����output1 = x   ���Ż��������6���������ɵ�����
%           output2 = k   ����������
%           output3 = f    ���Ż���������Ŀ�꺯�����ɵ�����
%           output4 = F   ��Ŀ�꺯��
function [output1,output2,output3,output4]=PDL(theta,phi,psi,T10,T20,T30,worldxy,pictureuv,camera)
      %���ڿ��������������̵ļ�����
        k = 0;  kmax = 600;  found = 0;     %���Ƶ�������
        epsilon1 = 0.0000000001;     %���ƾ���
        epsilon2 = 0.0000000001;     %���ƾ���
        epsilon3 = 0.0000000001;     %���ƾ���
        %%%%%%%%%%%%%%%%
        x = [theta,phi,psi,T10,T20,T30]';
        delta = 10; F = 0; F0 = 0; f  = 0; 
        [J,F,f]= JFfmaker(x(1),x(2),x(3),x(4),x(5),x(6),worldxy,pictureuv,camera);   %�����ſ˱Ⱦ���
        F0 = F;        %Ŀ�꺯��
        g = J'*f;
        fnorm2 = norm2(f);   %f������2����
        gnorm2 = norm2(g);  %g������2����
        if((fnorm2 <= epsilon3) || (gnorm2 <= epsilon1))
            found = 1;
        end
        while(~found && k < kmax)
                k = k+1;    %�Ե����Ĵ������м���
                alpha = norm2(g)^2/norm2(J*g)^2;
                hsd = -alpha*g;      %�ݶ��½����ĵ������򼰲���
                hgn = -(J'*J)\(J'*f);   %��˹ţ�ٷ��ĵ������򼰲���
                %*******************����hdl********************************************************************************************
                if(norm2(hgn) <= delta)
                        hdl = hgn;
                        L0Lhdl = F;
                elseif(norm2(hsd) >= delta)
                        hdl = hsd*delta/norm2(hsd);
                        L0Lhdl = delta*(2*norm2(hsd) - delta)/(2*alpha); 
                else
                        c = hsd'*(hgn - hsd);
                        if(c <= 0)
                                beta = (-c + sqrt(c^2+norm2(hgn - hsd)^2*(delta^2 - norm2(hsd)^2)))/(norm2(hgn - hsd)^2);
                        else
                                beta = (delta^2 - norm2(hsd)^2) / (c + sqrt(c^2+norm2(hgn - hsd)^2*(delta^2 - norm2(hsd)^2)));
                        end
                        hdl = alpha * hsd + beta*(hgn - hsd);
                        L0Lhdl = alpha*(1 - beta)^2*norm2(g)^2/2 + beta*(2 - beta)*F;
                end
                %***********************************************************************************************************************
                if(norm2(hdl) <= epsilon2*(norm2(x)+epsilon2))
                        found = 1;
                else
                        x_tmp = x + hdl;
                         [J_tmp,F,f_tmp]= JFfmaker(x_tmp(1),x_tmp(2),x_tmp(3),x_tmp(4),x_tmp(5),x_tmp(6),worldxy,pictureuv,camera);
                         e = (F - F0) / L0Lhdl;
                         F0 = F;
                         if(e > 0)
                                x = x_tmp;
                                [J,F,f]= JFfmaker(x_tmp(1),x_tmp(2),x_tmp(3),x_tmp(4),x_tmp(5),x_tmp(6),worldxy,pictureuv,camera);
                                g = J'*f;
                         end
                         if((fnorm2 <= epsilon3) || (gnorm2 <= epsilon1))
                                found = 1;
                         end
                         if(e>0.75)
                                delta = max(delta,3*norm2(hdl));
                         elseif(e < 0.25)
                                delta = delta/2; 
                                if(delta <= epsilon2*(norm2(x) + epsilon2))
                                    found = 1;
                                end
                         end
                end
        end
        output1 = x;
        output2 = k;
        output3 = f;
        output4 = F;
end