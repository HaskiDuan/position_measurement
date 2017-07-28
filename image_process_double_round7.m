clear all;
clc;
close all;
dbstop if error

tic
%%��������Բ���ڲ�����СԲ��Ƚ�
k0 = (93/28)^2;
%% ��ԭʼͼ����뵽Matlab��
I=imread('T73.jpg');
%I=imread('C:\Users\lenovo\Desktop\��½�ű�\object_T.jpg');
%% ʹ���Ա���㷨��ԭʼ�Ĳ�ɫͼ��ת���ɻҶ�ͼ��
[rows,columns,colors]=size(I);
gray=rgb2gray(I);
figure;
image(gray);hold off
%% ʹ���Ա�Otsu����ȷ����ֵ������ԭʼ�Ҷ�ͼ��ת���ɶ�ֵͼ��
T0 = graythresh(gray);
gray = im2bw(gray,T0);
imshow(gray);
%��ɫ��1 ��ɫ��0

%% ʹ���Ա�CclByTwopass_NEW2�����Զ�ֵͼ�������ͨ����
[gray,array,nlabel,label_count]=CclByTwopass_NEW2(gray);
figure;
imagesc(gray);
set(gca,'FontSize',18);
hold off

%% Ѱ����½�ű��Ӧ����ͨ��ı��
[target1_ID, target1_pixelnum, target2_ID, target2_pixelnum] = GMS(label_count);  % GMS : global maxium-value select
%����1�������жϲο��б��Ƿ������س�����ͼ��ƽ�棬���Ҷ�1�ź�2�Űб����������ȷ��
if ((target1_pixelnum-target2_pixelnum)>(target1_pixelnum/k0)*2 || (target1_pixelnum-target2_pixelnum)<(target1_pixelnum/k0)*0.75)
    error('��½�б겻��ȫ�������Ұ��');
end

%% ֻ��ʾ��½�ű꣬���Ѳο��б�ӱ�������ȡ����  
gray1 = zeros(rows, columns, 'uint8');
gray2 = zeros(rows, columns, 'uint8');
target_rows = 0;       % ��������
target_columns = 0;    % ��������
rows_num = 0;          % ��������
columns_num = 0;       % ��������
for i = 1:1:rows
    for j = 1:1:columns
        if ( gray(i, j) == target1_ID )
            gray1(i, j) = 1;
        elseif (gray(i, j) == target2_ID)
            gray2(i, j) = 1;
       end
    end
end

[x,y]=find(gray==target2_ID);
target_rows = min(x)+(max(x)-min(x))/2;   % ���ĵ�x����
target_columns =min(y)+(max(y)-min(y))/2;   % ���ĵ�y����

%����2�������жϲο��б��Ƿ������س�����ͼ��ƽ�棬���Ҷ�1�ź�2�Űб����������ȷ��
local_gray_average = 0;     %������Ϊ���ĵ�3x3С�����ڵĵ��ƽ���Ҷ�ֵ
for i = -1:1:1
    for j = -1:1:1
        local_gray_average = gray2(target_rows - i, target_columns - j) + local_gray_average;
    end
end
local_gray_average = local_gray_average/9;
if (local_gray_average~=0)
    error('��½�б겻����ȫ�������Ұ��');
end
% �Ѵ��׵��Բ�����İ׵�Ĩ��
k = 7;
width = round(sqrt(double(target2_pixelnum/(k-1))))/2;
gray2((target_rows - width):(target_rows + width), (target_columns - width):(target_columns + width)) = 1;
%figure; imshow(gray2);
%caxis([0,1]);

%% ��Ե���
gray1 = edge(gray1,'prewitt');
gray2 = edge(gray2,'prewitt');

figure;imshow(gray1);
figure;imshow(gray2);
%figure,imshow(gray2),hold on
%plot(target_columns, target_rows, 'x', 'LineWidth', 2, 'Color', 'g'); % ��ʾ������
gray_imshow = zeros(rows, columns, 'uint16');
figure;imshow(gray_imshow);hold on

%% ͨ��������ϵĵ���Բ���̣���������Բ����������
index = 1;
x0 = zeros(1, 2); y0 = zeros(1, 2); e_a = zeros(1, 2); e_b = zeros(1, 2); theta = zeros(1, 2);
answer = zeros(5, 2);   % ���ڴ����ϵõ���ϵ��
while(index <= 2)
    if (index == 1)
        gray = gray1;
    end
    if (index == 2)
        gray = gray2;  
    end
    %% ��С���������Բ1
    a11 = 0; a12 = 0; a13 = 0; a14 = 0; a15 = 0; b1 = 0;
    a21 = 0; a22 = 0; a23 = 0; a24 = 0; a25 = 0; b2 = 0;
    a31 = 0; a32 = 0; a33 = 0; a34 = 0; a35 = 0; b3 = 0;
    a41 = 0; a42 = 0; a43 = 0; a44 = 0; a45 = 0; b4 = 0;
    a51 = 0; a52 = 0; a53 = 0; a54 = 0; a55 = 0; b5 = 0;
    boundary_point = zeros(500, 2);
    num = 0;
    for i = 1:1:rows
        for j = 1:1:columns
            if ( gray(i, j) == 1 ) % ��ʾ����½�б�ı߽�����
                num = num + 1;
                boundary_point(num,1) = j;
                boundary_point(num,2) = i;
            end
        end
    end
   % count = 0;
    for  i= 1:1:num
       % count = count + 1;
        x1 = boundary_point(i,1); x2 = x1 * x1; x3 = x1 * x2;
        y1 = boundary_point(i,2); y2 = y1 * y1; y3 = y1 * y2; y4 = y3*y1;
        a51 = x1*y1 + a51;
        a52 = y2 + a52;
        a53 = x1 + a53;
        a54 = y1 + a54;
        a55 = 1 + a55;                                
        b5 = x2 + b5;

        a41 = x1*y2 + a41;
        a42 = y3 + a42;
        b4 = y1*x2 + b4;

        a33 = x2 + a33;
        b3 = x3 + b3;

        a21 = x1 * y3 + a21;
        a22 = y4 + a22;
        b2 = x2 * y2 + b2;

        b1 = x3 * y1 + b1;
    end
   % a55 = count;
    K = 100; %����ϵ��
    a51 = a51/K; a52 = a52/K; a53 = a53/K; a54 = a54/K; a55 = a55/K; b5 = b5/K;
    a41 = a41/K; a42 = a42/K; b4 = b4/K;
    a33 = a33/K; b3 = b3/K;
    a21 = a21/K; a22 = a22/K; b2 = b2/K;
    b1 = b1/K;

    a43 = a51; a44 = a52; a45 = a54;
    a31 = b4; a32 = a41; a34 = a43; a35 = a53;
    a23 = a32; a24 = a42; a25 = a52;

    a11= b2; a12 = a21; a13 = a31; a14 = a41; a15 = a51;
    eff_matrix1 = [a11, a12, a13, a14, a15;
                   a21, a22, a23, a24, a25;
                   a31, a32, a33, a34, a35;
                   a41, a42, a43, a44, a45;
                   a51, a52, a53, a54, a55];
    eff_matrix2 = [-b1;-b2;-b3;-b4;-b5];
    %    answer1 = det(eff_matrix1);
    answer(:, index) = eff_matrix1\eff_matrix2;
    A = answer(1,index);
    B = answer(2,index);
    C = answer(3,index);
    D = answer(4,index);
    E = answer(5,index);
                                               %        1      2
    x0(1, index) = (2*B*C-A*D)/(A.^2 - 4*B);   % x0: | x01 |  x02 |    1
    y0(1, index) = (2*D - A*C)/(A.^2 - 4*B);     % y0: | y01 |  y02 |    2
    delta = (A/2)^2-B;    
    %x0 = (B*(C/2) - (A/2)*(D/2))/delta;                       
    %y0 = ((D/2) - (A/2)*(C/2))/delta;
    phi = 0.5 * acot((B-1)/(2*(A/2)));      
    nom = 2 * ((D/2)^2 + B*(C/2)^2 + E*(A/2)^2 - 2*(A/2)*(C/2)*(D/2) - B*E);
    s = sqrt(1 + (4*(A/2)^2)/(1-B)^2);
    e_a(1, index) = sqrt(nom/(delta* ( (B-1)*s -(B+1))));
    e_b(1, index) = sqrt(nom/(delta* ( (1-B)*s -(B+1))));
    theta(1, index) = 0.5 * acot((B-1)/A);
    
    if (index == 1)
        plot(x0(1,index), y0(1,index),'x', 'LineWidth', 2, 'Color', 'r');
    end
    if (index == 2)
        plot(x0(1,index), y0(1,index),'x', 'LineWidth', 2, 'Color', 'g');
    end
%    plot(x0(1,index), y0(1,index),'x', 'LineWidth', 2, 'Color', 'r');
    PlotEllipse(x0(1,index), -y0(1,index), e_a(1,index), e_b(1,index), -theta(1,index));
    
    index = index + 1;
end

%% �������Բ�����߷��̺��е�
C1 = [1 answer(1,1)/2 answer(3,1)/2; answer(1,1)/2 answer(2,1) answer(4,1)/2; answer(3,1)/2 answer(4,1)/2 answer(5,1)];
C2 = [1 answer(1,2)/2 answer(3,2)/2; answer(1,2)/2 answer(2,2) answer(4,2)/2; answer(3,2)/2 answer(4,2)/2 answer(5,2)];
C1star = inv(C1);
C2star = inv(C2);

% ���ɹ�����
% ȷ������Բ�����������������������㣨Ҳ���е㣩
r11 = C2star(1,1); r12 = C2star(1,2); r13 = C2star(1,3);
r22 = C2star(2,2); r23 = C2star(2,3);
r33 = C2star(3,3);
const1_y = (r23 + sqrt(double(r23.^2 - r22*r33)))/r33; 
const2_y = (r23 - sqrt(double(r23.^2 - r22*r33)))/r33;
if ( const1_y > const2_y )
    down_point = const2_y;     % ���·��Ĺ�����
    up_point = const1_y;       % ���Ϸ��Ĺ�����
else
    down_point = const1_y;
    up_point = const2_y;
end

var_count = 0;
for i = down_point:0.02:down_point+1-0.02
        var_count = var_count + 1;
        rolling_point(var_count,2) = i;   % �������y����
        rolling_point(var_count,1) = (sqrt((A*i+C).^2 - 4*(i.^2*B + i*D + E))-(A*i+C))/2;  %�Ҳ�������x����
        rolling_point(var_count,3) = (-sqrt((A*i+C).^2 - 4*(i.^2*B + i*D + E))-(A*i+C))/2; %���������x����
end
for i = down_point+1:0.2:up_point-1
        var_count = var_count + 1;
        rolling_point(var_count,2) = i;   % �������y����
        rolling_point(var_count,1) = (sqrt((A*i+C).^2 - 4*(i.^2*B + i*D + E))-(A*i+C))/2;  %�Ҳ�������x����
        rolling_point(var_count,3) = (-sqrt((A*i+C).^2 - 4*(i.^2*B + i*D + E))-(A*i+C))/2; %���������x����
end
for i = up_point-1+0.02:0.02:up_point
        var_count = var_count + 1;
        rolling_point(var_count,2) = i;   % �������y����
        rolling_point(var_count,1) = (sqrt((A*i+C).^2 - 4*(i.^2*B + i*D + E))-(A*i+C))/2;  %�Ҳ�������x����
        rolling_point(var_count,3) = (-sqrt((A*i+C).^2 - 4*(i.^2*B + i*D + E))-(A*i+C))/2; %���������x����
end

rolling_line = zeros(2*var_count-2, 3);   % ������ ����2*var_count-2��
for i = 1:1:var_count      % �Ұ��ĵ� + ���·��ĵ� + ���Ϸ��ĵ�
    rolling_line(i,:) = C2*[rolling_point(i,1);rolling_point(i,2);1];  % ��Բ���е�ĳ˻�Ϊ����
end
rolling_line_count = var_count + 1;
for i = var_count-1:-1:2
    rolling_line(rolling_line_count,:) = C2*[rolling_point(i,3);rolling_point(i,2);1];
    rolling_line_count = rolling_line_count + 1;
end
bias = zeros(2*var_count-2, 1);
for i = 1:1:2*var_count-2
    bias(i,1) = abs(rolling_line(i,:)*C1star*rolling_line(i,:)');
end

tangent_line_ID = NonMinSup(bias); % �ҳ�bias�����еļ�Сֵ�����Ƕ�Ӧ�����е�����
tangent_line_num = 0;   % tangent_line_ID�еķ���Ԫ�صĸ�������Ϊ���ߵ�����
for i = 1:1:length(tangent_line_ID)
    if(tangent_line_ID(i) ~= 0)
        tangent_line_num = tangent_line_num + 1;
    end   
end
if (tangent_line_num < 4)   % �����ȡ������������ĿС��4������ô�ͶԵ�һ�������һ�����ߵ�biasֵ���Ƚϴ���
    if  ( bias(1,1) > bias(2*var_count-2,1) )
        tangent_line_ID(4) = 2*var_count-2;
        tangent_line_num = tangent_line_num + 1;
    else
        tangent_line_ID(4) = 1;
        tangent_line_num = tangent_line_num + 1;
    end
end

% ����ȡ���������߻�����
for i = 1:1:tangent_line_num
    a = tangent_line_ID(i);
    y1position = (-rolling_line(a,3)-rolling_line(a,1)*100)/rolling_line(a,2);  % x1position = 100;
    y2position = (-rolling_line(a,3)-rolling_line(a,1)*600)/rolling_line(a,2);  % x2position = 300;

    plot([100 600], [y1position y2position], 'LineWidth', 2, 'Color', 'm');  %��������ֱ��
end

tangent_point = zeros(2*tangent_line_num,3);
for i = 1:1:tangent_line_num
    tangent_point(2*i-1,:) = (C1star*rolling_line(tangent_line_ID(i),:)')';
    tangent_point(2*i,:) = (C2star*rolling_line(tangent_line_ID(i),:)')';
    tangent_point(2*i-1,:) = tangent_point(2*i-1,:)/tangent_point(2*i-1,3);  % ���������ת���ɷ����������ʽ��
    tangent_point(2*i,:) = tangent_point(2*i,:)/tangent_point(2*i,3);        % �Ա��Ӧ��IR2ƽ���ϵĵ�
end
%for i =1:1:2*tangent_line_num
%   plot(tangent_point(i,1), tangent_point(i,2),'x', 'LineWidth', 2, 'Color', 'y'); 
%end

tangent_point_rotated = zeros(2*tangent_line_num,2);   % �е���(x2,y2)��תalpha�ǶȺ�õ��ĵ�
k1 = 0; k2 = 0; k3 = 0; k4 = 0; k5 = 0; k6 = 0; L1 = 0; L2 = 0; L3 = 0; 
cos_alpha = (x0(1,1)-x0(1,2))/sqrt((x0(1,1)-x0(1,2)).^2 + (y0(1,1)-y0(1,2)).^2);
for i = 1:1:2*tangent_line_num
    k2 = tangent_point(i,1)-x0(1,2);
    k3 = tangent_point(i,2)-y0(1,2);
    k1 = cos_alpha*( k2.^2 + k3.^2 ) + k2*x0(1,2) + k3*y0(1,2); 
    k4 = tangent_point(i,1).^2 + tangent_point(i,2).^2  - 2*tangent_point(i,1)*x0(1,2) - 2*tangent_point(i,2)*y0(1,2);
    k5 = x0(1,2); k6 = y0(1,2);
    L1 = k2.^2 + k3.^2;
    L2 = 2*k2*k3*k6 - 2*k3.^2*k5 - 2*k1*k2;
    L3 = k1.^2 - 2*k1*k3*k6 - k3.^2*k4;
    delta = sqrt(L2.^2 - 4*L1*L3);
    x01 = (-L2 + delta)/(2*L1);
    y01 = (k1 - k2*x01)/k3;
    vector_product1 = y0(1,2) - y0(1,1);
    vector_product2 = k2*(y01 - y0(1,2)) - k3*(x01 - x0(1,2));
    if(vector_product1 * vector_product2 < 0)
        x01 = (-L2 - delta)/(2*L1);
        y01 = (k1 - k2*x01)/k3;
    end
    tangent_point_rotated(i,1) = x01;
    tangent_point_rotated(i,2) = y01;
end
%for i =1:1:2*tangent_line_num
%   plot(tangent_point_rotated(i,1), tangent_point_rotated(i,2),'x', 'LineWidth', 2, 'Color', 'c'); 
%end

%% ��8���е����͵���������б��
feature_point = zeros(8,2);
auto_var1 = zeros(4,2);
auto_count1 = 0;
auto_var2 = zeros(4,2);
auto_count2 = 0;
for i = 1:1:8
    if (tangent_point_rotated(i,2) < y0(1,2))
        auto_count1 = auto_count1 + 1;
        auto_var1(auto_count1,1) = i;
        auto_var1(auto_count1,2) = tangent_point_rotated(i,1);
    else
        auto_count2 = auto_count2 + 1;
        auto_var2(auto_count2,1) = i;
        auto_var2(auto_count2,2) = tangent_point_rotated(i,1);
    end
end
% ��ȡ������1��������4
max = 0; feature_point1_ID = 0; max_ID = 0;
min = auto_var1(1,2); feature_point4_ID = 0; min_ID = 0;
for i = 1:1:4
    if (auto_var1(i,2) >= max)
        max = auto_var1(i,2); max_ID = i;
        feature_point4_ID = auto_var1(i,1);
    end
    if (auto_var1(i,2) <= min)
        min = auto_var1(i,2); min_ID = i;
        feature_point1_ID = auto_var1(i,1);
    end
end
feature_point1 = [tangent_point(feature_point1_ID,1), tangent_point(feature_point1_ID,2)];
feature_point4 = [tangent_point(feature_point4_ID,1), tangent_point(feature_point4_ID,2)];
auto_var1(max_ID,2) = 0; auto_var1(min_ID,2) = 0;
% ��ȡ������2��������3
max = 0; max_ID = 0; feature_point2_ID = 0;feature_point3_ID = 0;
for i = 1:1:4
    if (auto_var1(i,2) >= max)
        max = auto_var1(i,2); max_ID = i;
        feature_point3_ID = auto_var1(i,1);
    end
end
auto_var1(max_ID, 2) = 0;
for i = 1:1:4
    if(auto_var1(i,2) ~= 0)
        feature_point2_ID = auto_var1(i,1);
    end
end
feature_point2 = [tangent_point(feature_point2_ID,1), tangent_point(feature_point2_ID,2)];
feature_point3 = [tangent_point(feature_point3_ID,1), tangent_point(feature_point3_ID,2)];
% ��ȡ������5��������8
max = 0; feature_point5_ID = 0; max_ID = 0;
min = auto_var2(1,2); feature_point8_ID = 0; min_ID = 0;
for i = 1:1:4
    if (auto_var2(i,2) >= max)
        max = auto_var2(i,2); max_ID = i;
        feature_point8_ID = auto_var2(i,1);
    end;
    if (auto_var2(i,2) <= min)
        min = auto_var2(i,2); min_ID = i;
        feature_point5_ID = auto_var2(i,1);
    end
end
feature_point5 = [tangent_point(feature_point5_ID,1), tangent_point(feature_point5_ID,2)];
feature_point8 = [tangent_point(feature_point8_ID,1), tangent_point(feature_point8_ID,2)];

auto_var2(max_ID,2) = 0; auto_var2(min_ID,2) = 0;
% ��ȡ������6��������7
max = 0; max_ID = 0; feature_point6_ID = 0;feature_point7_ID = 0;
for i = 1:1:4
    if (auto_var2(i,2) >= max)
        max = auto_var2(i,2); max_ID = i;
        feature_point7_ID = auto_var2(i,1);
    end
end
auto_var2(max_ID, 2) = 0;
for i = 1:1:4
    if(auto_var2(i,2) ~= 0)
        feature_point6_ID = auto_var2(i,1);
    end
end
feature_point6 = [tangent_point(feature_point6_ID,1), tangent_point(feature_point6_ID,2)];
feature_point7 = [tangent_point(feature_point7_ID,1), tangent_point(feature_point7_ID,2)];
%% �ѵ�1��~��8����������ͬ��ű�ע��ͼ��
%plot(feature_point1(1,1), feature_point1(1,2),'o', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point2(1,1), feature_point2(1,2),'x', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point3(1,1), feature_point3(1,2),'+', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point4(1,1), feature_point4(1,2),'*', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point5(1,1), feature_point5(1,2),'s', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point6(1,1), feature_point6(1,2),'d', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point7(1,1), feature_point7(1,2),'v', 'LineWidth', 2, 'Color', 'y');
%plot(feature_point8(1,1), feature_point8(1,2),'^', 'LineWidth', 2, 'Color', 'y');

%% ������13�������������ȡ�����(��ŷ�Χ��9 ~ 21) 
%���õ�������ʽ��ֱ�߱�ʾ��
%��������1��������4��ֱ��line1_4
line1_4 = [feature_point1(1,2) - feature_point4(1,2), feature_point4(1,1) - feature_point1(1,1), feature_point1(1,1)*feature_point4(1,2) - feature_point4(1,1)*feature_point1(1,2)];
%��������5��������8��ֱ��line5_8
line5_8 = [feature_point5(1,2) - feature_point8(1,2), feature_point8(1,1) - feature_point5(1,1), feature_point5(1,1)*feature_point8(1,2) - feature_point8(1,1)*feature_point5(1,2)];
%��������2��������7��ֱ��line2_7
line2_7 = [feature_point2(1,2) - feature_point7(1,2), feature_point7(1,1) - feature_point2(1,1), feature_point2(1,1)*feature_point7(1,2) - feature_point7(1,1)*feature_point2(1,2)];
%��������3��������6��ֱ��line3_6
line3_6 = [feature_point3(1,2) - feature_point6(1,2), feature_point6(1,1) - feature_point3(1,1), feature_point3(1,1)*feature_point6(1,2) - feature_point6(1,1)*feature_point3(1,2)];
%��������2��������5��ֱ��line2_5
line2_5 = [feature_point2(1,2) - feature_point5(1,2), feature_point5(1,1) - feature_point2(1,1), feature_point2(1,1)*feature_point5(1,2) - feature_point5(1,1)*feature_point2(1,2)];
%��������3��������8��ֱ��line3_8
line3_8 = [feature_point3(1,2) - feature_point8(1,2), feature_point8(1,1) - feature_point3(1,1), feature_point3(1,1)*feature_point8(1,2) - feature_point8(1,1)*feature_point3(1,2)];
%��������1��������6��ֱ��line1_6
line1_6 = [feature_point1(1,2) - feature_point6(1,2), feature_point6(1,1) - feature_point1(1,1), feature_point1(1,1)*feature_point6(1,2) - feature_point6(1,1)*feature_point1(1,2)];
%��������4��������7��ֱ��line4_7
line4_7 = [feature_point4(1,2) - feature_point7(1,2), feature_point7(1,1) - feature_point4(1,1), feature_point4(1,1)*feature_point7(1,2) - feature_point7(1,1)*feature_point4(1,2)];
%��������1��������2��ֱ��line1_2
line1_2 = [feature_point1(1,2) - feature_point2(1,2), feature_point2(1,1) - feature_point1(1,1), feature_point1(1,1)*feature_point2(1,2) - feature_point2(1,1)*feature_point1(1,2)];
%��������3��������4��ֱ��line3_4
line3_4 = [feature_point3(1,2) - feature_point4(1,2), feature_point4(1,1) - feature_point3(1,1), feature_point3(1,1)*feature_point4(1,2) - feature_point4(1,1)*feature_point3(1,2)];
%��������5��������6��ֱ��line5_6
line5_6 = [feature_point5(1,2) - feature_point6(1,2), feature_point6(1,1) - feature_point5(1,1), feature_point5(1,1)*feature_point6(1,2) - feature_point6(1,1)*feature_point5(1,2)];
%��������7��������8��ֱ��line7_8
line7_8= [feature_point7(1,2) - feature_point8(1,2), feature_point8(1,1) - feature_point7(1,1), feature_point7(1,1)*feature_point8(1,2) - feature_point8(1,1)*feature_point7(1,2)];

%������9 Ϊline1_4��line2_5�Ľ���
feature_point9 = [(line1_4(1,2)*line2_5(1,3) - line1_4(1,3)*line2_5(1,2))/(line1_4(1,1)*line2_5(1,2) - line1_4(1,2)*line2_5(1,1)) ...
                              (line1_4(1,3)*line2_5(1,1) - line1_4(1,1)*line2_5(1,3))/(line1_4(1,1)*line2_5(1,2) - line1_4(1,2)*line2_5(1,1)) ];
%������10 Ϊline1_4��line3_8�Ľ���
feature_point10 = [(line1_4(1,2)*line3_8(1,3) - line1_4(1,3)*line3_8(1,2))/(line1_4(1,1)*line3_8(1,2) - line1_4(1,2)*line3_8(1,1)) ...
                              (line1_4(1,3)*line3_8(1,1) - line1_4(1,1)*line3_8(1,3))/(line1_4(1,1)*line3_8(1,2) - line1_4(1,2)*line3_8(1,1)) ];
%������11 Ϊline1_6��line5_8�Ľ���
feature_point11 = [(line1_6(1,2)*line5_8(1,3) - line1_6(1,3)*line5_8(1,2))/(line1_6(1,1)*line5_8(1,2) - line1_6(1,2)*line5_8(1,1)) ...
                              (line1_6(1,3)*line5_8(1,1) - line1_6(1,1)*line5_8(1,3))/(line1_6(1,1)*line5_8(1,2) - line1_6(1,2)*line5_8(1,1)) ];
%������12 Ϊline4_7��line5_8�Ľ���
feature_point12 = [(line4_7(1,2)*line5_8(1,3) - line4_7(1,3)*line5_8(1,2))/(line4_7(1,1)*line5_8(1,2) - line4_7(1,2)*line5_8(1,1)) ...
                              (line4_7(1,3)*line5_8(1,1) - line4_7(1,1)*line5_8(1,3))/(line4_7(1,1)*line5_8(1,2) - line4_7(1,2)*line5_8(1,1)) ];
%������13 Ϊline2_7��line3_6�Ľ���
feature_point13 = [(line2_7(1,2)*line3_6(1,3) - line2_7(1,3)*line3_6(1,2))/(line2_7(1,1)*line3_6(1,2) - line2_7(1,2)*line3_6(1,1)) ...
                              (line2_7(1,3)*line3_6(1,1) - line2_7(1,1)*line3_6(1,3))/(line2_7(1,1)*line3_6(1,2) - line2_7(1,2)*line3_6(1,1)) ];
%������14 Ϊline1_6��line2_5�Ľ���
feature_point14 = [(line1_6(1,2)*line2_5(1,3) - line1_6(1,3)*line2_5(1,2))/(line1_6(1,1)*line2_5(1,2) - line1_6(1,2)*line2_5(1,1)) ...
                              (line1_6(1,3)*line2_5(1,1) - line1_6(1,1)*line2_5(1,3))/(line1_6(1,1)*line2_5(1,2) - line1_6(1,2)*line2_5(1,1)) ];
%������15 Ϊline3_8��line4_7�Ľ���
feature_point15 = [(line3_8(1,2)*line4_7(1,3) - line3_8(1,3)*line4_7(1,2))/(line3_8(1,1)*line4_7(1,2) - line3_8(1,2)*line4_7(1,1)) ...
                              (line3_8(1,3)*line4_7(1,1) - line3_8(1,1)*line4_7(1,3))/(line3_8(1,1)*line4_7(1,2) - line3_8(1,2)*line4_7(1,1)) ];
%������16 Ϊline2_7��line3_4�Ľ���
feature_point16 = [(line2_7(1,2)*line3_4(1,3) - line2_7(1,3)*line3_4(1,2))/(line2_7(1,1)*line3_4(1,2) - line2_7(1,2)*line3_4(1,1)) ...
                              (line2_7(1,3)*line3_4(1,1) - line2_7(1,1)*line3_4(1,3))/(line2_7(1,1)*line3_4(1,2) - line2_7(1,2)*line3_4(1,1)) ];
%������17 Ϊline1_2��line3_4�Ľ���
feature_point17 = [(line1_2(1,2)*line3_4(1,3) - line1_2(1,3)*line3_4(1,2))/(line1_2(1,1)*line3_4(1,2) - line1_2(1,2)*line3_4(1,1)) ...
                              (line1_2(1,3)*line3_4(1,1) - line1_2(1,1)*line3_4(1,3))/(line1_2(1,1)*line3_4(1,2) - line1_2(1,2)*line3_4(1,1)) ];
%������18 Ϊline1_2��line3_6�Ľ���
feature_point18 = [(line1_2(1,2)*line3_6(1,3) - line1_2(1,3)*line3_6(1,2))/(line1_2(1,1)*line3_6(1,2) - line1_2(1,2)*line3_6(1,1)) ...
                              (line1_2(1,3)*line3_6(1,1) - line1_2(1,1)*line3_6(1,3))/(line1_2(1,1)*line3_6(1,2) - line1_2(1,2)*line3_6(1,1)) ];
%������19 Ϊline3_6��line7_8�Ľ���
feature_point19 = [(line3_6(1,2)*line7_8(1,3) - line3_6(1,3)*line7_8(1,2))/(line3_6(1,1)*line7_8(1,2) - line3_6(1,2)*line7_8(1,1)) ...
                              (line3_6(1,3)*line7_8(1,1) - line3_6(1,1)*line7_8(1,3))/(line3_6(1,1)*line7_8(1,2) - line3_6(1,2)*line7_8(1,1)) ];
%������20 Ϊline5_6��line7_8�Ľ���
feature_point20 = [(line5_6(1,2)*line7_8(1,3) - line5_6(1,3)*line7_8(1,2))/(line5_6(1,1)*line7_8(1,2) - line5_6(1,2)*line7_8(1,1)) ...
                              (line5_6(1,3)*line7_8(1,1) - line5_6(1,1)*line7_8(1,3))/(line5_6(1,1)*line7_8(1,2) - line5_6(1,2)*line7_8(1,1)) ];
%������21 Ϊline2_7��line5_6�Ľ���
feature_point21 = [(line2_7(1,2)*line5_6(1,3) - line2_7(1,3)*line5_6(1,2))/(line2_7(1,1)*line5_6(1,2) - line2_7(1,2)*line5_6(1,1)) ...
                              (line2_7(1,3)*line5_6(1,1) - line2_7(1,1)*line5_6(1,3))/(line2_7(1,1)*line5_6(1,2) - line2_7(1,2)*line5_6(1,1)) ];
                          
%��21���������������Ϣ����װ�ص�feature_point���������                         
feature_point = zeros(21,3);    
feature_point(1,1) = feature_point1(1,1); feature_point(1,2) = feature_point1(1,2);feature_point(1,3) = 1;   %װ��������1
feature_point(2,1) = feature_point2(1,1); feature_point(2,2) = feature_point2(1,2);feature_point(2,3) = 1;   %װ��������2
feature_point(3,1) = feature_point3(1,1); feature_point(3,2) = feature_point3(1,2);feature_point(3,3) = 1;   %װ��������3
feature_point(4,1) = feature_point4(1,1); feature_point(4,2) = feature_point4(1,2);feature_point(4,3) = 1;   %װ��������4
feature_point(5,1) = feature_point5(1,1); feature_point(5,2) = feature_point5(1,2);feature_point(5,3) = 1;   %װ��������5
feature_point(6,1) = feature_point6(1,1); feature_point(6,2) = feature_point6(1,2);feature_point(6,3) = 1;   %װ��������6
feature_point(7,1) = feature_point7(1,1); feature_point(7,2) = feature_point7(1,2);feature_point(7,3) = 1;   %װ��������7
feature_point(8,1) = feature_point8(1,1); feature_point(8,2) = feature_point8(1,2);feature_point(8,3) = 1;   %װ��������8
feature_point(9,1) = feature_point9(1,1); feature_point(9,2) = feature_point9(1,2);feature_point(9,3) = 1;   %װ��������9
feature_point(10,1) = feature_point10(1,1); feature_point(10,2) = feature_point10(1,2);feature_point(10,3) = 1;   %װ��������10
feature_point(11,1) = feature_point11(1,1); feature_point(11,2) = feature_point11(1,2);feature_point(11,3) = 1;   %װ��������11
feature_point(12,1) = feature_point12(1,1); feature_point(12,2) = feature_point12(1,2);feature_point(12,3) = 1;   %װ��������12
feature_point(13,1) = feature_point13(1,1); feature_point(13,2) = feature_point13(1,2);feature_point(13,3) = 1;   %װ��������13
feature_point(14,1) = feature_point14(1,1); feature_point(14,2) = feature_point14(1,2);feature_point(14,3) = 1;   %װ��������14
feature_point(15,1) = feature_point15(1,1); feature_point(15,2) = feature_point15(1,2);feature_point(15,3) = 1;   %װ��������15
feature_point(16,1) = feature_point16(1,1); feature_point(16,2) = feature_point16(1,2);feature_point(16,3) = 1;   %װ��������16
feature_point(17,1) = feature_point17(1,1); feature_point(17,2) = feature_point17(1,2);feature_point(17,3) = 1;   %װ��������17
feature_point(18,1) = feature_point18(1,1); feature_point(18,2) = feature_point18(1,2);feature_point(18,3) = 1;   %װ��������18
feature_point(19,1) = feature_point19(1,1); feature_point(19,2) = feature_point19(1,2);feature_point(19,3) = 1;   %װ��������19
feature_point(20,1) = feature_point20(1,1); feature_point(20,2) = feature_point20(1,2);feature_point(20,3) = 1;   %װ��������20
feature_point(21,1) = feature_point21(1,1); feature_point(21,2) = feature_point21(1,2);feature_point(21,3) = 1;   %װ��������21
%imshow(I);hold on
for i = 1:1:21
    plot(feature_point(i,1), feature_point(i,2),'*', 'LineWidth', 3, 'Color', 'c');   %feature_pointװ�صľ�����������ͼ�������е������ʽ������
end
camera_parameter = [3887.9 0 516.6087;0 3876.4 275.7370; 0 0 1];   %�궨�õ�����������ڲξ���
pixel_ordinate_uv = zeros(3,21); 
for i = 1:1:21
    feature_point(i,1) = feature_point(i,1) - 1;
    feature_point(i,2) = feature_point(i,2) - 1;
end
pixel_ordinate_uv = camera_parameter\feature_point';    %��������ڲξ������ * ��������ͼ����������ϵ�еĵ������ �õ� ��ת�����������ʽ 3x21
%����������������ϵ�е�����
world_ordinate_xy = zeros(3,21);
                              %      1      2       3         4       5      6        7         8       9       10     11     12       13     14     15     16      17      18     19     20       21
%world_ordinate_xy = [ 93.0 77.0  77.0   93.0     0   16.0  16.0       0    93.0   93.0     0       0      46.5  46.5   46.5 60.0   64.5   60.0  33.0  28.5   33.0  ; ...
%                                    46.5 82.0 133.5 169.0 46.5 82.0 133.5 169.0 89.5 126.0 89.5 126.0 108.0 68.0 147.5 97.0 108.0 119.0 97.0 108.0 119.0 ;...
%                                     1       1       1        1        1       1       1         1      1         1        1       1         1       1         1      1         1        1       1        1        1    ];
world_ordinate_xy = [ 93.0 76.427  76.431   93.0     0     16.573  16.569       0     93.0      93.0         0            0           46.5      46.5      46.5       59.596   64.716     59.589  33.404  28.284   33.411  ; ...
                                    46.5 82.09    132.405 168.0   46.5  82.09  132.405 168.0 89.808 124.688 89.808 124.688 107.246 68.154 146.344 96.238   107.238  118.248 96.238 107.238 118.248 ;...
                                     1         1              1        1           1          1           1           1          1           1           1            1             1            1           1             1              1                1            1          1            1    ];
                                 
F_matrix = zeros(42,9);    
for i = 1:1:21                  
    F_matrix(2*i - 1, 1) = world_ordinate_xy(1,i); 
    F_matrix(2*i - 1, 3) = -world_ordinate_xy(1,i)*pixel_ordinate_uv(1,i); 
    F_matrix(2*i - 1, 4) = world_ordinate_xy(2,i);
    F_matrix(2*i - 1, 6) = -world_ordinate_xy(2,i)*pixel_ordinate_uv(1,i); 
    F_matrix(2*i - 1, 7) =1;
    F_matrix(2*i - 1, 9) = -pixel_ordinate_uv(1,i); 
    
    F_matrix(2*i , 2) = world_ordinate_xy(1,i); 
    F_matrix(2*i , 3) = -world_ordinate_xy(1,i)*pixel_ordinate_uv(2,i); 
    F_matrix(2*i , 5) = world_ordinate_xy(2,i); 
    F_matrix(2*i , 6) = -world_ordinate_xy(2,i)*pixel_ordinate_uv(2,i); 
    F_matrix(2*i , 8) =1;
    F_matrix(2*i , 9) = -pixel_ordinate_uv(2,i); 
end   

%��ϵ�������������ֵ�ֽ⣬�õ�������ϵ���� r1 r2 T[U0 S0 V0] = svd(F_matrix);
[U0 S0 V0] = svd(F_matrix);
translation0 = zeros(3,1);
rotation = zeros(3,3);
r1plusr2 = abs(V0(1,9))+abs(V0(2,9))+abs(V0(3,9))+abs(V0(4,9))+abs(V0(5,9))+abs(V0(6,9)) ;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        +abs(V0(6,9));
for i = 1:1:3
    translation0(i,1) = 2*V0(i+6,9)/r1plusr2;
end

if (translation0(3,1)>0)
    translation = -translation0;
else
    translation = translation0;
end

%����ת����R
coef_matrix = [V0(1,9) V0(4,9) 0;V0(2,9) V0(5,9) 0;V0(3,9) V0(6,9) 0];
[U1 S1 V1] = svd(coef_matrix);
rotation = U1*V1; 
rotation = real(rotation);
toc

%[theta1,phi1,psi1,theta2,phi2,psi2] = EulerAngleSolve(rotation);
[theta1,phi1,psi1] = EulerAngleSolve_NEW(rotation);
[output1,iterate_num,f_value,F_value] = PDL(theta1,phi1,psi1,translation(1,1),translation(2,1),translation(3,1),world_ordinate_xy,feature_point',camera_parameter);
theta = output1(1); phi = output1(2); psi = output1(3);
%�޶�ÿ���ǵķ�Χ
if(theta > pi/2)
    theta = theta - pi;
end
if(theta < -pi/2 )
   theta = theta + pi;
end

if(phi > pi/2)
    phi = phi - pi;
end
if(phi < -pi/2 )
   phi = phi + pi;
end

%psi = psi - pi;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ʹ�����Է������õ�����̬��λ�ò�������������ϵ�еĵ�ͶӰ��ͼ��ƽ��
angle1 = zeros(3,1);
angle1(1,1) = theta*180/pi;
angle1(2,1) = phi*180/pi;
angle1(3,1) = psi*180/pi;

%% ʹ�÷������Ż������̬��λ�ò�������������ϵ�еĵ�ͶӰ��ͼ��ƽ��
%������ʱ���Դ���
translation(1,1) = output1(4); translation(2,1) = output1(5); translation(3,1) = output1(6);

%coef_matrix = [output1(1) output1(4) 0; output1(2) output1(5) 0; output1(3) output1(6) 0];

rotation = [(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) (cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) -sin(phi)*cos(theta);...
                  -(sin(psi)*cos(theta)) (cos(psi)*cos(theta)) sin(theta);...
                   cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta) (sin(phi)*sin(psi) - cos(psi)*cos(phi)*sin(theta)) cos(phi)*cos(theta)];
               
%rotation= [output1(1) output1(4) 0; output1(2) output1(5) 0; output1(3) output1(6) 0];
%[U1 S1 V1] = svd(coef_matrix);
%rotation2 = U1*V1; 
%

%��1����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv1 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [93.0 46.5 0 1]';
remap_pixel_ordinate_uv01 = [remap_pixel_ordinate_uv1(1,1)/remap_pixel_ordinate_uv1(3,1) remap_pixel_ordinate_uv1(2,1)/remap_pixel_ordinate_uv1(3,1)];
plot(remap_pixel_ordinate_uv01(1,1),remap_pixel_ordinate_uv01(1,2),'s', 'LineWidth', 2, 'Color', 'y');

%��2����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv2 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [77.0 82.0 0 1]';
remap_pixel_ordinate_uv02 = [remap_pixel_ordinate_uv2(1,1)/remap_pixel_ordinate_uv2(3,1) remap_pixel_ordinate_uv2(2,1)/remap_pixel_ordinate_uv2(3,1)];
plot(remap_pixel_ordinate_uv02(1,1),remap_pixel_ordinate_uv02(1,2),'s', 'LineWidth', 2, 'Color', 'r');

%��3����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv3 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [77.0 133.5 0 1]';
remap_pixel_ordinate_uv03 = [remap_pixel_ordinate_uv3(1,1)/remap_pixel_ordinate_uv3(3,1) remap_pixel_ordinate_uv3(2,1)/remap_pixel_ordinate_uv3(3,1)];
plot(remap_pixel_ordinate_uv03(1,1),remap_pixel_ordinate_uv03(1,2),'s', 'LineWidth', 2, 'Color', 'r');

%��4����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv4 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [93.0 169.0 0 1]';
remap_pixel_ordinate_uv04 = [remap_pixel_ordinate_uv4(1,1)/remap_pixel_ordinate_uv4(3,1) remap_pixel_ordinate_uv4(2,1)/remap_pixel_ordinate_uv4(3,1)];
plot(remap_pixel_ordinate_uv04(1,1),remap_pixel_ordinate_uv04(1,2),'s', 'LineWidth', 2, 'Color', 'r');

%��5����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv5 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [0 46.5 0 1]';
remap_pixel_ordinate_uv05 = [remap_pixel_ordinate_uv5(1,1)/remap_pixel_ordinate_uv5(3,1) remap_pixel_ordinate_uv5(2,1)/remap_pixel_ordinate_uv5(3,1)];
plot(remap_pixel_ordinate_uv05(1,1),remap_pixel_ordinate_uv05(1,2),'s', 'LineWidth', 2, 'Color', 'r');

%��6����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv6 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [16 82 0 1]';
remap_pixel_ordinate_uv06 = [remap_pixel_ordinate_uv6(1,1)/remap_pixel_ordinate_uv6(3,1) remap_pixel_ordinate_uv6(2,1)/remap_pixel_ordinate_uv6(3,1)];
plot(remap_pixel_ordinate_uv06(1,1),remap_pixel_ordinate_uv06(1,2),'s', 'LineWidth', 2, 'Color', 'r');

%��7����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv7 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [16 133.5 0 1]';
remap_pixel_ordinate_uv07 = [remap_pixel_ordinate_uv7(1,1)/remap_pixel_ordinate_uv7(3,1) remap_pixel_ordinate_uv7(2,1)/remap_pixel_ordinate_uv7(3,1)];
plot(remap_pixel_ordinate_uv07(1,1),remap_pixel_ordinate_uv07(1,2),'s', 'LineWidth', 2, 'Color', 'r');

%��8����������ͶӰ��ͼ��ƽ��
remap_pixel_ordinate_uv8 = camera_parameter * [rotation(1,1) rotation(1,2) rotation(1,3) translation(1,1);rotation(2,1) rotation(2,2) rotation(2,3) translation(2,1);...
    rotation(3,1) rotation(3,2) rotation(3,3) translation(3,1)] * [0 169 0 1]';
remap_pixel_ordinate_uv08 = [remap_pixel_ordinate_uv8(1,1)/remap_pixel_ordinate_uv8(3,1) remap_pixel_ordinate_uv8(2,1)/remap_pixel_ordinate_uv8(3,1)];
plot(remap_pixel_ordinate_uv08(1,1),remap_pixel_ordinate_uv08(1,2),'s', 'LineWidth', 2, 'Color', 'r');

norm = sqrt(rotation(1,1)^2+rotation(2,1)^2+rotation(3,1)^2);

dat123 = rotation(1,1)^2 + rotation(2,1)^2 +rotation(3,1)^2;
dat123456 = rotation(1,1)*rotation(1,2) + rotation(2,1)*rotation(2,2) + rotation(3,1)*rotation(3,2);

remap_pixel_ordinate_uv(1,1) = remap_pixel_ordinate_uv01(1,1) ; remap_pixel_ordinate_uv(1,2) = remap_pixel_ordinate_uv01(1,2) ;
remap_pixel_ordinate_uv(2,1) = remap_pixel_ordinate_uv02(1,1) ; remap_pixel_ordinate_uv(2,2) = remap_pixel_ordinate_uv02(1,2) ;
remap_pixel_ordinate_uv(3,1) = remap_pixel_ordinate_uv03(1,1) ; remap_pixel_ordinate_uv(3,2) = remap_pixel_ordinate_uv03(1,2) ;
remap_pixel_ordinate_uv(4,1) = remap_pixel_ordinate_uv04(1,1) ; remap_pixel_ordinate_uv(4,2) = remap_pixel_ordinate_uv04(1,2) ;
remap_pixel_ordinate_uv(5,1) = remap_pixel_ordinate_uv05(1,1) ; remap_pixel_ordinate_uv(5,2) = remap_pixel_ordinate_uv05(1,2) ;
remap_pixel_ordinate_uv(6,1) = remap_pixel_ordinate_uv06(1,1) ; remap_pixel_ordinate_uv(6,2) = remap_pixel_ordinate_uv06(1,2) ;
remap_pixel_ordinate_uv(7,1) = remap_pixel_ordinate_uv07(1,1) ; remap_pixel_ordinate_uv(7,2) = remap_pixel_ordinate_uv07(1,2) ;
remap_pixel_ordinate_uv(8,1) = remap_pixel_ordinate_uv08(1,1) ; remap_pixel_ordinate_uv(8,2) = remap_pixel_ordinate_uv08(1,2) ;
remap_error = zeros(8,1);
for i = 1:1:8
    remap_error (i,1) = sqrt((feature_point(i,1) - remap_pixel_ordinate_uv(i,1))^2 +  (feature_point(i,2) - remap_pixel_ordinate_uv(i,2))^2);
end
