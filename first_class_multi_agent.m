clear;
clc;
%% 一阶积分器多智能体一致性算法
% 初始化
x0 = [1,8,3,-2,10,4];
dt = 0.01;
T = 3;
t = 0:dt:3;
n = length(t);
x = x0';
point_x=0;
% A = [
%     0 1 0 1 1 0;
%     1 0 1 0 0 1;
%     0 1 0 0 0 1;
%     1 0 0 0 1 0;
%     1 0 0 1 0 1;
%     0 1 1 0 1 0
%     ];
% % 计算拉普拉斯阵
% D = diag(sum(A,2));
% L = D - A;
L = [4 -1 0 -1 -1 -1;  % 定义L = D-A ，D是入度矩阵，A是邻接矩阵, L是拉普拉斯矩阵
    -1 5 -1 -1 -1 -1;     % 这里定义的是-L
    0 -1 4 -1 -1 -1;
    -1 -1 -1 5 -1 -1;
    -1 -1 -1 -1 5 -1;
    -1 -1 -1 -1 -1 5];

% 更新状态
for i = 1:n
    u(:,i) = -L*x;
    X(:,i) = x;
    x = dt*u(:,i) + x;
    if (abs(x(1)-x(2))<=0.01)&&(abs(x(2)-x(3))<=0.01)&&(abs(x(3)-x(4))<=0.01)&&(abs(x(4)-x(5))<=0.01)&&(abs(x(5)-x(6))<=0.01)&&(point_x==0)
        point_x = i*dt
        point_y = x(1)
    end
end

%智能体实际的状态变化
plot(t,X);
hold on;
xlabel('t(s)','FontSize',18)
ylabel('state','FontSize',18)
legend('agent 1','agent 2','agent 3','agent 4','agent 5','agent 6','Location', 'southeast' , 'FontSize',12)
% scatter(point_x, point_y, 'r', 'filled');

% 绘制竖线
verticalLineX = point_x;  % 竖线的横坐标
minY = min(-2);
maxY = max(10);
verticalLineY = [minY, maxY];  % 竖线的纵坐标范围
line([verticalLineX, verticalLineX], verticalLineY, 'Color', 'r', 'LineStyle', '--');  % 绘制红色虚线竖线
% 
% hold off;
% 
% zp = BaseZoom();
% zp.plot;
