%Two-pass ��ͨ�������㷨��4�ڽӶ�����򣩣���ͼ�����Ͻ���ͼ�����½Ƿ���ɨ�裩
%���룺image��һ����ֵͼ��
%�����output_args1 = label;    ��ͼ��imageͬά��һ�����󣬴��ÿһ�����صı��
%           output_args2 = array;     �ȼ۶�����
%           output_args3 = nlabel;    �Ա�Ž��м�����һ������������ñ����Ա�����͵��Դ���
%           output_args4 = label_count;  ����ͳ��ÿ����Ŷ�Ӧ�����ص�ĸ���
function [ output_args1, output_args2, output_args3, output_args4 ] = CclByTwopass_NEW2( image )
array = ones(30000,1,'uint16');   %�ȼ۶�����
[m, n] = size(image);
label = ones(m, n,'uint16');   %��ͼ��imageͬά��һ������ÿ��Ԫ�ص�ֵ��ʾ��imageͬλ�ô������صı��
nlabel = 2;
%% ONE-PASS
% �ȵ��������һ�е�һ�е�Ԫ��
if(image(1,1) == 0) % ֤����ǰ���㣬�����Ǳ�����
    label(1,1) = nlabel;
    nlabel = nlabel + 1;
end
% �ٴ����һ������Ԫ��
for j = 2:1:n
    if(image(1, j) == 0) % ֤����ǰ���㣬�����Ǳ�����
        if (image(1, j) == image(1, j-1))
            label(1, j) = label(1, j-1);
        else
            label(1, j) = nlabel;
            nlabel = nlabel + 1;
        end
    end
end
% �ٴ����һ�е�Ԫ��
for i = 2:1:m
    if(image(i, j) == 0)  % ֤����ǰ���㣬�����Ǳ�����
        if (image(i, 1) == image(i-1, 1))
            label(i, 1) = label(i-1, 1);
        else
            label(i, 1) = nlabel;
            nlabel = nlabel + 1;
        end
    end
end
% Ȼ���ٴ���ʣ������ص� %
for i = 2:1:m
    for j = 2:1:n
        if(image(i,j) == 0)  % ֤����ǰ���㣬�����Ǳ�����
            if (image(i, j) == image(i, j-1) || image(i, j) == image(i-1, j))
               if (image(i, j-1) ~= image(i-1, j))
                   if (image(i, j) == image(i, j-1))
                       label(i, j) = label(i, j-1);
                   else
                       label(i, j) = label(i-1, j);
                   end
               else
                   if (label(i, j-1) == label(i-1, j))
                       label(i, j) = label(i-1, j);
                   else
                       if (label(i, j-1) > label(i-1, j))
                           label(i, j) = label(i-1, j);
                           array(label(i,j-1)) = label(i-1, j); % setunion
                           label(i, j-1) = label(i-1, j);  %
                       else
                           label(i, j) = label(i, j-1);
                           array(label(i-1, j)) = label(i, j-1);
                       end 
                   end
               end
            else
                label(i, j) = nlabel;
                nlabel = nlabel + 1;
            end 
        end
    end
end
%% ʹ��setfind�����Եȼ۶�������·��ѹ����Path Compression��
for i = 2:1:nlabel
    if (array(i) > 1)
        array(i) = setfind(array(i), array);
    end
end
%% TWO-PASS 
for i = 1:1:m
    for j = 1:1:n
        while(array(label(i, j)) > 1)
            label(i, j) = array(label(i, j));
        end
        
    end
end
 %%ͳ��ÿ����Ŷ�Ӧ�����ص�ĸ�����׼ȷ�ؽ�����һ���ִ����Ѿ�������Two-pass CCL�㷨�ˣ�����Ϊ�����԰б����ȡ��ʶ������׼��������
label_count = zeros(nlabel, 1, 'uint16'); % ����ͳ��ÿ����Ŷ�Ӧ�����ص�ĸ���
for i = 1:1:m
    for j = 1:1:n
        if(label(i, j) ~= 1)
            label_count(label(i, j)) = label_count(label(i, j)) + 1;
        end
    end
end
output_args1 = label;
output_args2 = array;
output_args3 = nlabel;
output_args4 = label_count;
end