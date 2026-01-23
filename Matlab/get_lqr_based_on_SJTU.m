tic
clear all
clc

% 定义机器人机体参数
syms R_w                % 驱动轮半径
syms R_l                % 驱动轮轮距/2
syms l_l l_r            % 左右腿长
syms l_wl l_wr          % 驱动轮质心到左右腿部质心距离
syms l_bl l_br          % 髋关节中点到左右腿部质心距离
syms l_c                % 机体质心到腿部关节中心点距离
syms m_w m_l m_b        % 驱动轮质量 腿部质量 机体质量
syms I_w                % 驱动轮转动惯量           (自然坐标系法向)
syms I_ll I_lr          % 驱动轮左右腿部转动惯量    (自然坐标系法向，实际上会变化)
syms I_b                % 机体转动惯量             (自然坐标系法向)
syms I_z                % 机器人z轴转动惯量        (简化为常量)

%定义中间变量
syms Sb dSb ddSb hb dhb ddhb slr dslr ddslr sll hlr dhlr ddhlr hll dhll ddhll 
%动力学中间变量
syms Fwsr Fwsl Fbsl Fbsr Fwhl Fwhr Fbhl Fbhr fl fr
% 定义其他独立变量并补充其导数
syms theta_wl   theta_wr   % 左右驱动轮转角
syms dtheta_wl  dtheta_wr 
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b

% 定义状态向量
syms S ds dds phi dphi ddphi theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b dsll ddsll 

% 定义控制向量
syms T_lwl T_lwr T_bll T_blr

% 输入物理参数：重力加速度
syms g

%进行小角度近似后的二阶导
ddphi = R_w*(-ddtheta_wl+ddtheta_wr)/(2*R_l) - l_l*theta_ll*ddtheta_ll/(2*R_l) + l_r*ddtheta_lr/(2*R_l);
ddhll = ddhb + l_bl*theta_ll*ddtheta_ll;
ddhlr = ddhb + l_br*theta_lr*ddtheta_lr;
ddSb = R_w*(ddtheta_wl+ddtheta_wr)/2 + l_l*ddtheta_ll/2 + l_r*ddtheta_lr/2;
ddhb = -l_l*theta_ll*ddtheta_ll/2 - l_r*theta_lr*ddtheta_lr/2;
ddslr = R_w*ddtheta_wr + l_wr*ddtheta_lr;
ddsll = R_w*ddtheta_wl + l_wl*ddtheta_ll;
%计算后的动力学的中间变量
%%按照从后往前的顺序替换可以保证所有内容都被替换为各个二阶导的基本方程
fl = (T_lwl - I_w*ddtheta_wl)/R_w;
fr = (T_lwr - I_w*ddtheta_wr)/R_w;
Fwsl = fl - m_w*R_w*ddtheta_wl;
Fwsr = fr - m_w*R_w*ddtheta_wr;
Fbsl = Fwsl - m_l*ddsll;
Fbsr = Fwsr - m_l*ddslr;
Fbhl = (m_b*ddhb + m_b*g - (m_l*ddhlr - m_l*ddhll))/2;
Fbhr = (m_b*ddhb + m_b*g + (m_l*ddhlr - m_l*ddhll))/2;
Fwhl = m_l*ddhll + m_l*g + Fbhl;
Fwhr = m_l*ddhlr + m_l*g + Fbhr;

%%HKU answer
sj_eq1 = (I_z*R_w/(2*R_l)+R_l*I_w/R_w)*ddtheta_wl - (I_z*R_w/(2*R_l)+R_l*I_w/R_w)*ddtheta_wr ...
- R_l*T_lwl/R_w + R_l*T_lwr/R_w+I_z*l_l*ddtheta_ll/(2*R_l) - I_z*l_r*ddtheta_lr/(2*R_l) == 0;
sj_eq2 = (m_l*l_wl+m_b*l_l/2)*g*theta_ll + ((I_w/R_w + m_w*R_w)*l_l+m_l*R_w*l_bl)*ddtheta_wl ...
+ (m_l*l_wl*l_bl - I_ll)*ddtheta_ll - (1+l_l/R_w)*T_lwl+T_bll;
sj_eq3 = (m_l*l_wr+m_b*l_r/2)*g*theta_lr + ((I_w/R_w + m_w*R_w)*l_r+m_l*R_w*l_br)*ddtheta_wr ...
+ (m_l*l_wr*l_br - I_lr)*ddtheta_lr - (1+l_r/R_w)*T_lwr+T_blr;
sj_eq4 = -(I_w/R_w+m_w*R_w+m_l*R_w+m_b*R_w/2)*(ddtheta_wl+ddtheta_wr)-(m_l*l_wl+m_b*l_l/2)*ddtheta_ll ...
- (m_l*l_wr + m_b*l_r/2)*ddtheta_lr + T_lwl/R_w + T_lwr/R_w == 0;
sj_eq5 = l_c*(I_w/R_w + m_w*R_w+m_l*R_w)*(ddtheta_wl+ddtheta_wr) - m_l*l_wl*l_c*ddtheta_ll ...
- m_l*l_wr*l_c*ddtheta_lr - l_c*(T_lwr+T_lwl)/R_w - (T_bll+T_blr) + l_c*m_b*g*theta_b - I_b*ddtheta_b == 0;
%%STJU answer
% sj_eq1 = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl+(m_l*l_wl+m_b*l_l/2)*g*theta_ll+T_bll-T_lwl*(1+l_l/R_w) == 0;
% sj_eq2 = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr+(m_l*l_wr+m_b*l_r/2)*g*theta_lr+T_blr-T_lwr*(1+l_r/R_w) == 0;
% sj_eq3 = -(m_w*R_w^2+I_w+m_l*R_w^2+m_b*R_w^2/2)*ddtheta_wr-(m_w*R_w^2+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr...
%     -(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll-(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_lwl+T_lwr == 0;
% sj_eq4 = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl+(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr...
%     +m_l*l_wl*l_c*ddtheta_ll+m_l*l_wr*l_c*ddtheta_lr-I_b*ddtheta_lr+m_b*g*l_c*theta_b - (T_lwl+T_lwr)*l_c/R_w - (T_bll+T_blr) == 0;
% sj_eq5 = (I_z*R_w/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl-(I_z*R_w/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr...
%     +I_z*l_l*ddtheta_ll/(2*R_l)-I_z*l_r*ddtheta_lr/(2*R_l)-T_lwl*R_l/R_w+T_lwr*R_l/R_w == 0;
%%建立替换规则的元胞数组，通过管理rules来实现管理替换变量
rules = {
    Fwhr,m_l*ddhlr + m_l*g + Fbhr;
    Fwhl , m_l*ddhll + m_l*g + Fbhl;
    Fbhr , (m_b*ddhb + m_b*g + (m_l*ddhlr - m_l*ddhll))/2;
    Fbhl , (m_b*ddhb + m_b*g - (m_l*ddhlr - m_l*ddhll))/2;
    Fbsr , Fwsr - m_l*ddslr;
    Fbsl , Fwsl - m_l*ddsll;
    Fwsr , fr - m_w*R_w*ddtheta_wr;
    Fwsl , fl - m_w*R_w*ddtheta_wl;
    fr , (T_lwr - I_w*ddtheta_wr)/R_w;
    fl , (T_lwl - I_w*ddtheta_wl)/R_w;
    ddphi , R_w*(-ddtheta_wl+ddtheta_wr)/(2*R_l) - l_l*theta_ll*ddtheta_ll/(2*R_l) + l_r*ddtheta_lr/(2*R_l);
    ddhll , ddhb + l_bl*theta_ll*ddtheta_ll;
    ddhlr , ddhb + l_br*theta_lr*ddtheta_lr;
    ddSb , R_w*(ddtheta_wl+ddtheta_wr)/2 + l_l*ddtheta_ll/2 + l_r*ddtheta_lr/2;
    ddhb , -l_l*theta_ll*ddtheta_ll/2 - l_r*theta_lr*ddtheta_lr/2;
    ddslr , R_w*ddtheta_wr + l_wr*ddtheta_lr;
    ddsll , R_w*ddtheta_wl + l_wl*ddtheta_ll;
    sin(theta_wr),theta_wr;
    sin(theta_wl),theta_wl;
    sin(theta_ll),theta_ll;
    sin(theta_lr),theta_lr;
    sin(theta_b),theta_b;
    cos(theta_wr),1;
    cos(theta_wl),1;
    cos(theta_ll),1;
    cos(theta_lr),1;
    cos(theta_b),1;
    dtheta_wr, 0;
    dtheta_wl, 0;
    dtheta_ll, 0;
    dtheta_lr, 0;
    dtheta_b, 0;
};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 此处输入机器人的实际参数
% 此处机器人参数是经过理想化处理的
sym_2_real = {
    R_w, 0.06; %轮半径
    R_l, 0.165; %机体半径
    l_wl, l_l/2; %驱动轮质心到左右腿质心距离
    l_wr, l_r/2; %默认为腿长一半
    l_bl, l_l/2; %髋关节中点到腿质心距离
    l_br, l_r/2; %默认为腿长一半
    I_ll, 847/1000/1000; %腿的转动惯量，轴心为髋关节中点
    I_lr, 847/1000/1000;
    I_b , 7020.46/1000/1000; %车体的转动惯量
    l_c , 18.17/1000; %身体质心到髋关节中点连线距离
    m_w , 0.495; %轮的质量
    m_l , 0.585; %腿的质量
    m_b , 3; %机体质量
    I_w , 1047.352/1000/1000; %轮的转动惯量
    I_z , 96173.7/1000/1000; %整个车的转动惯量 
    g, 9.8;
    };
R_w_ac = 0.06;
R_l_ac = 0.165;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
eqations = [sj_eq1, sj_eq2, sj_eq3, sj_eq4, sj_eq5];
target = [ddtheta_wl,ddtheta_wr,ddtheta_ll, ddtheta_lr, ddtheta_b];
for i = 1:size(rules,1)
    eqations = subs(eqations,rules(i,1),rules(i,2));
end
answer = solve(eqations,target);

%%将高阶小量消除
small = [theta_ll, theta_lr, theta_b];
ddtheta_wl_sol = answer.ddtheta_wl;
ddtheta_wr_sol = answer.ddtheta_wr;
ddtheta_ll_sol = answer.ddtheta_ll;
ddtheta_lr_sol = answer.ddtheta_lr;
ddtheta_b_sol = answer.ddtheta_b;
ddtheta_wl_sol = taylor(ddtheta_wl_sol, [theta_ll, theta_lr, theta_b], 'Order', 2);
ddtheta_wr_sol = taylor(ddtheta_wr_sol, [theta_ll, theta_lr, theta_b], 'Order', 2);
ddtheta_ll_sol = taylor(ddtheta_ll_sol, [theta_ll, theta_lr, theta_b], 'Order', 2);
ddtheta_lr_sol = taylor(ddtheta_lr_sol, [theta_ll, theta_lr, theta_b], 'Order', 2);
ddtheta_b_sol = taylor(ddtheta_b_sol, [theta_ll, theta_lr, theta_b], 'Order', 2);

formulas = [ddtheta_wl_sol,ddtheta_wr_sol,ddtheta_ll_sol,ddtheta_lr_sol,ddtheta_b_sol];

x = [theta_ll; theta_lr; theta_b];
u = [T_lwl; T_lwr; T_bll; T_blr];
%%修改Q矩阵和R矩阵
%S dot_S phi dot_phi tehta_ll dot_theta_ll theta_lr dot_theta_lr theta_b dot_theta_b
%T_lwl T_lwr T_bll T_blr
matrix_Q = diag([4 2 15 3 4 1 4 1 8 1]);  % diag函数用于产生对角矩阵
matrix_R = diag([0.8 0.8 0.5 0.5]);

%到此为止，我们输出的是一个保留与腿长相关的变量的方程式
%取消注释到275行就是实现了简化模型控制的求解
%{
for i = 1:size(sym_2_real,1)
    formulas = subs(formulas,sym_2_real(i,1),sym_2_real(i,2));
end
for i = 1:length(formulas)
    disp(formulas(i));
end
disp(formulas);


A_sym = jacobian(formulas, x);
B_sym = jacobian(formulas, u);

A_function = matlabFunction(A_sym,'Vars',[l_l,l_r]);
B_function = matlabFunction(B_sym,'Vars',[l_l,l_r]);
jacobi_A = A_function(0.1,0.1);
jacobi_B = B_function(0.1,0.1);
temp3 = write_data_a(jacobi_A,1,1,0.1,0.1);
temp4 = write_data_b(jacobi_B,1,1,0.1,0.1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%定腿长
static_a = write_data_a(jacobi_A,R_w_ac,R_l_ac,0.12,0.12);
static_b = write_data_b(jacobi_B,R_w_ac,R_l_ac,0.12,0.12);
[K,S,E] = lqr(static_a,static_b,matrix_Q,matrix_R);
disp(K);
mat2c(K,'myMat');
% leg_length
% 设置腿的长度和步长
start_num = 0.1;
step = 0.01;
end_num = 0.25;
leg_length = start_num:step:end_num
%%均不取到end_num
times = floor((end_num-start_num)/step);
result = zeros(4,10,times^2);
for i = 1:times
    for j = 1:times
        temp_a = write_data_a(jacobi_A,R_w_ac,R_l_ac,leg_length(i),leg_length(j));
        temp_b = write_data_b(jacobi_B,R_w_ac,R_l_ac,leg_length(i),leg_length(j));
        [K, S, E] = lqr(temp_a,temp_b,matrix_Q,matrix_R);
        result(:,:,(i-1)*times+j) = K;
    end
end
%%求二维拟合的曲面函数
coeffMatrix = zeros(40, 6);
for ptr_i = 1:4
    for ptr_j = 1:10
        z = result(ptr_i,ptr_j,:);
        z = z(:);
        x = zeros(times^2,1);
        y = zeros(times^2,1);
        for add_ll = 1:15
            for add_lr = 1:15
                x((add_ll-1)*15+add_lr,1) = add_ll/10;
                y((add_ll-1)*15+add_lr,1) = add_lr/10;
            end
        end
        ft = fittype('poly22');   % poly11 poly22 poly33 都可以
        fit_answer = fit([x, y], z,ft);
        coeffs = coeffvalues(fit_answer);
        for i = 1:6
            coeffMatrix((ptr_i-1)*10+ptr_j,i) = coeffs(i);
        end
    end
end
disp(coeffMatrix);
dynamic_formula(40,6,10,coeffMatrix)
numFits = size(coeffMatrix,1); % 40
cppStrings = strings(numFits,1);

% poly22 对应变量名顺序（假设按照你需要的命名）
varNames = {'x0_y0','x1_y0','x0_y1','x2_y0','x1_y1','x0_y2'};
for k = 1:numFits
    str = "dynamic[" + (k-1) + "] = "; 
    for c = 1:6
        coef = coeffMatrix(k,c);
        % 保留3位小数并加 f
        coefStr = sprintf('%.3ff', coef);
        str = str + coefStr + "*" + varNames{c};
        if c < 6
            str = str + " + ";
        else
            str = str + ";";
        end
    end
    cppStrings(k) = str;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[K, S, E] = lqr(temp3,temp4,matrix_Q,matrix_R);
toc;

%{
最后data_ary输出的是六个一组的系数，x是左腿长度，y是右腿长度
分别为'x0y0','x0_y0','x1_y0','x0_y1','x2_y0','x1_y1','x0_y2'
%}
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function matrix_a = write_data_a(matrix_j,R_w_ac,R_l_ac,l_l_ac,l_r_ac)
    temp = zeros(10);
    temp(1,2) = 1;
    temp(3,4) = 1;
    temp(5,6) = 1;
    temp(7,8) = 1;
    temp(9,10) = 1;
    for i = 3:5
        for j = 1:3
            temp(i*2,j*2+3) = matrix_j(i,j);
        end
    end
    for i = 1:3
        temp(2,i*2+3) = R_w_ac*(matrix_j(1,i)+matrix_j(2,i))/2;
        temp(4,i*2+3) = R_w_ac*(-matrix_j(1,i)+matrix_j(2,i))/2 ...
        - l_l_ac*matrix_j(3,i)/(2*R_l_ac)+l_r_ac*matrix_j(4,i)/(2*R_l_ac);
    end
    matrix_a = temp;
end

function matrix_b = write_data_b(matrix_j,R_w_ac,R_l_ac,l_l_ac,l_r_ac)
    temp = zeros(10,4);
    for i = 3:5
        for j = 1:4
            temp(i*2,j) = matrix_j(i,j);
        end
    end
    for i = 1:4
        temp(2,i) = R_w_ac*(matrix_j(1,i)+matrix_j(2,i))/2;
        temp(4,i) = R_w_ac*(-matrix_j(1,i)+matrix_j(2,i))/2 ...
        - l_l_ac*matrix_j(3,i)/(2*R_l_ac)+l_r_ac*matrix_j(4,i)/(2*R_l_ac);
    end
    matrix_b = temp; 
end

function mat2c(M, varname)
% M: 输入矩阵
% varname: 输出变量名（字符）
    % 按行优先展开（ROW-MAJOR）
    data = reshape(M.', 1, []);  % 先转置再按列取，效果等于按行取
    % 打印开头
    fprintf('%s[%d] = {', varname, numel(data));
    % 遍历输出
    for i = 1:numel(data)
        if i == numel(data)
            fprintf('%gf', data(i));  % 最后一个不加逗号
        else
            fprintf('%gf, ', data(i));
        end
    end
    fprintf('};\n');
end

function dynamic_formula(rows,clons,line_length,matrix)
    itemsPerLine = line_length;

fprintf("data_ary[%d] = {\n",rows*clons);
for i = 1:rows
    for j = 1:clons
        fprintf("%.4ff",matrix(i,j));
        if (i < rows || j < clons)
            fprintf(',');
        end
        if (mod((i-1)*clons + j, itemsPerLine) == 0)
            fprintf('\n')
        end
    end
end
fprintf("\n};\n");
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_data_const = {
    R_w, 0.06; %轮半径
    R_l, 0.165; %机体半径
    I_b , 7020.46/1000/1000; %车体的转动惯量
    l_c , 18.17/1000; %身体质心到髋关节中点连线距离
    m_w , 0.495; %轮的质量
    m_l , 0.585; %腿的质量
    m_b , 3; %机体质量
    I_w , 1047.352/1000/1000; %轮的转动惯量
    I_z , 96173.7/1000/1000; %整个车的转动惯量 
    g, 9.8;
};
leg_times = 4;%有多少个腿长
T = readtable("robot_data.xlsx");
matrix_Ks = zeros(4,10,leg_times^2);%所有的腿长组合下的K矩阵数值
leg_min = 1;%腿长最小值
delta = 0.05;%变化步长
w_ac = 0.6; %轮子半径
Rl_ac = 0.4; %轮轮间距的一半
real_formula = formulas;%定义变量：接近实际情况的方程式
for i = 1:size(robot_data_const)
    real_formula = subs(real_formula,robot_data_const(i,1),robot_data_const(i,2));
end
for i = 1:leg_times
    for j = 1:leg_times
        robot_data_dynamic = {
          l_wl, T.L_w(i);
          l_wr, T.L_w(j);
          l_bl, T.L_b(i);
          l_br, T.L_b(j);
          I_ll, T.I_l(i);
          I_lr, T.I_l(j);
          l_l, T.leg(i);
          l_r, T.leg(j);
        };
        temp = real_formula;
        for c = 1:size(robot_data_dynamic,1)
            temp = subs(temp,robot_data_dynamic(c,1),robot_data_dynamic(c,2));
        end
        A_temp_sym = jacobian(temp,x);
        B_temp_sym = jacobian(temp,u);
        A_matrix = write_data_a(A_temp_sym,w_ac,Rl_ac,T.leg(i),T.leg(j));
        B_matrix = write_data_b(B_temp_sym,w_ac,Rl_ac,T.leg(i),T.leg(j));
        [K, S, E] = lqr(A_matrix,B_matrix,matrix_Q,matrix_R);
        matrix_Ks(:,:,(i-1)*4+j) = K;
    end
end
%接下来对矩阵K进行拟合
coeffMatrix = zeros(40,6);
x = zeros(leg_times^2,1);%左腿腿长
y = zeros(leg_times^2,1);%右腿腿长
ft = fittype('poly22');
for i = 1:leg_times
    for j = 1:leg_times
        k = (i-1)*leg_times+j;
        x(k,1) = leg_min+delta*(i-1);
        y(k,1) = leg_min+delta*(j-1);
    end
end
for i = 1:4
    for j = 1:10
        z = squeeze(matrix_Ks(i,j,:));
        fit_answer = fit([x,y],z,ft);
        coeffs = coeffvalues(fit_answer);
        for k =1:6
            coeffMatrix((i-1)*10+j,k) = coeffs(k);
        end
    end
end
