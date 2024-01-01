clear
close all
clc
%% 一阶积分器多智能体事件触发机制控制算法(无向图，随时间变化的阈值，分布式控制，仅在自身触发时主动采样邻居当前状态，邻居触发时自身不触发)
x = [1,8,3,-2,10,4]; % 6个智能体的初始位置
L = [4 -1 0 -1 -1 -1;  % 定义L = D-A ，D是入度矩阵，A是邻接矩阵, L是拉普拉斯矩阵
    -1 5 -1 -1 -1 -1;     % 这里定义的是-L
    0 -1 4 -1 -1 -1;
    -1 -1 -1 5 -1 -1;
    -1 -1 -1 -1 5 -1;
    -1 -1 -1 -1 -1 5];     %注：全连接的无向图，即每两个智能体之间均连通，x最终趋向于所有智能体初始状态的平均值。
dt = 0.01; % 时间分辨率
Ts = 0:dt:3; % 时间跨度
xhat = x; % xhat表示智能体当前位置
x1 = []; % 存放x向量的所有预期状态，最新数据向下存放
x1hat = []; % 存放xhat向量的所有已知状态，最新数据向下存放
s = 1; % 代表第几次循环，每次循环 s+1
E = []; % 存放误差数据，最新数据向下存放
c = 0.052 ; % 触发函数中的比例增益项系数
alpha = 1.2; % 触发函数中的指数增益项系数
U = [];
event_1 = [];
event_2 = [];
event_3 = [];
event_4 = [];
event_5 = [];
event_6 = [];
point_x = 0;
point_y = 0;
u = -L * [1;10;15;5;20;4];  %[0;3;2;5;7;8]
u = u.';

for t = 0:dt:3

    e = [];
    % 计算测量误差 e = xhat - x
    e1 = xhat(1,1) - x(1,1);
    e2 = xhat(1,2) - x(1,2);
    e3 = xhat(1,3) - x(1,3);
    e4 = xhat(1,4) - x(1,4);
    e5 = xhat(1,5) - x(1,5);
    e6 = xhat(1,6) - x(1,6);
    e = [e1 e2 e3 e4 e5 e6];


    % 计算事件触发函数
    m = c * exp(-alpha*t); % 触发函数阈值
    f1 = abs(e1) - m;
    f2 = abs(e2) - m;
    f3 = abs(e3) - m;
    f4 = abs(e4) - m;
    f5 = abs(e5) - m;
    f6 = abs(e6) - m;

    
    % 判断是否发生事件触发
    if f1>=0
        xhat(1,1) = x(1,1);
        u(1,1) = -L(1,:) * ([xhat(1,1);xhat(1,2);xhat(1,3);xhat(1,4);xhat(1,5);xhat(1,6)]);
        if point_x ==0
        event_1 = [event_1;t];
        end
    end
    if f2>=0
        xhat(1,2) = x(1,2);
        u(1,2) = -L(2,:) * ([xhat(1,1);xhat(1,2);xhat(1,3);xhat(1,4);xhat(1,5);xhat(1,6)]);  
        if point_x ==0
        event_2 = [event_2;t];
        end
    end
    if f3>=0
        xhat(1,3) = x(1,3);
        u(1,3) = -L(3,:) * ([xhat(1,1);xhat(1,2);xhat(1,3);xhat(1,4);xhat(1,5);xhat(1,6)]);
        if point_x ==0
        event_3 = [event_3;t];
        end
    end
    if f4>=0
        xhat(1,4) = x(1,4);
        u(1,4) = -L(4,:) * ([xhat(1,1);xhat(1,2);xhat(1,3);xhat(1,4);xhat(1,5);xhat(1,6)]);
        if point_x ==0
        event_4 = [event_4;t];
        end
    end
    if f5>=0
        xhat(1,5) = x(1,5);
        u(1,5) = -L(5,:) * ([xhat(1,1);xhat(1,2);xhat(1,3);xhat(1,4);xhat(1,5);xhat(1,6)]);
        if point_x ==0
        event_5 = [event_5;t];
        end
    end
    if f6>=0
        xhat(1,6) = x(1,6);
        u(1,6) = -L(6,:) * ([xhat(1,1);xhat(1,2);xhat(1,3);xhat(1,4);xhat(1,5);xhat(1,6)]);
        if point_x ==0
        event_6 = [event_6;t];
        end
    end

    % 更新实际状态，采用一阶系统：x导 = u
    x(1,1) = x(1,1) + dt * u(1,1);
    x(1,2) = x(1,2) + dt * u(1,2);
    x(1,3) = x(1,3) + dt * u(1,3);
    x(1,4) = x(1,4) + dt * u(1,4);
    x(1,5) = x(1,5) + dt * u(1,5);
    x(1,6) = x(1,6) + dt * u(1,6); 
    x1 = [x1;x];

    x1hat = [x1hat;xhat];
    E = [E;e];
    U = [U;u];
    % 找出一致性误差达到0.01时的坐标点
%     minDifference = inf;  % 初始化为正无穷大
% 
%     for i = 1:5
%       for j = i+1:6
%         difference = abs(x(i) - x(j));
%         if difference < minDifference
%           minDifference = difference;
%         end
%       end
%     end
% 
% disp(minDifference);
     if (abs(x(1,1)-x(1,2))<=0.01)&&(abs(x(1,2)-x(1,3))<=0.01)&&(abs(x(1,3)-x(1,4))<=0.01)&&(abs(x(1,4)-x(1,5))<=0.01)&&(abs(x(1,5)-x(1,6))<=0.01)&&(abs(x(1,1)-x(1,3))<=0.01)&&(abs(x(1,1)-x(1,4))<=0.01)&&(abs(x(1,1)-x(1,5))<=0.01)&&(abs(x(1,1)-x(1,6))<=0.01)&&(abs(x(1,2)-x(1,4))<=0.01)&&(abs(x(1,2)-x(1,5))<=0.01)&&(abs(x(1,2)-x(1,6))<=0.01)&&(abs(x(1,3)-x(1,5))<=0.01)&&(abs(x(1,3)-x(1,6))<=0.01)&&(abs(x(1,4)-x(1,6))<=0.01)&&(point_x==0)
         point_x = s * dt
         point_y = x(1,1)
     end
    s = s + 1;
end
%% 画图
% %每次循环更新的期望位置
% figure(1)
% plot(Ts,x1);
% xlabel('时间')
% ylabel('期望位置')
% % text(point_x,point_y,'*','color','b')
% grid on
% %每次触发执行的输入
% figure(2)
% plot(Ts,U);
% xlabel('时间')
% ylabel('输入')
% grid on
%智能体实际的状态变化
figure(3)
plot(Ts,x1hat);
hold on;
xlabel('t(s)','FontSize',18)
ylabel('state','FontSize',18)
legend('agent 1','agent 2','agent 3','agent 4','agent 5','agent 6','Location', 'southeast' , 'FontSize',12)

% 绘制竖线
verticalLineX = point_x;  % 竖线的横坐标
minY = min(-2);
maxY = max(10);
verticalLineY = [minY, maxY];  % 竖线的纵坐标范围
line([verticalLineX, verticalLineX], verticalLineY, 'Color', 'r', 'LineStyle', '--');  % 绘制红色虚线竖线

hold off;
grid on
%触发时刻散点图
% figure(4)
% y1 = 1.0 * ones(1, length(event_1));
% scatter(event_1, y1, '*');
% hold on
% 
% y2 = 2.0 * ones(1, length(event_2));
% scatter(event_2, y2, '*');
% hold on
% 
% y3 = 3.0 * ones(1, length(event_3));
% scatter(event_3, y3, '*');
% hold on
% 
% y4 = 4.0 * ones(1, length(event_4));
% scatter(event_4, y4, '*');
% 
% y5 = 5.0 * ones(1, length(event_5));
% scatter(event_5, y5, '*');
% hold on
% 
% y6 = 6.0 * ones(1, length(event_6));
% scatter(event_6, y6, '*');
% hold on
% 
% ylim([0, 7.5]);
% xlabel('时间')
% ylabel('触发时刻')
% grid on
% 
% figure(5)
% plot(Ts,E);
% xlabel('时间')
% ylabel('误差')
% grid on

% zp = BaseZoom();
% zp.plot;
sum1 = length(event_1)
sum2 = length(event_2)
sum3 = length(event_3)
sum4 = length(event_4)
sum5 = length(event_5)
sum6 = length(event_6)
sum = (length(event_1)+length(event_2)+length(event_3)+length(event_4)+length(event_5)+length(event_6))