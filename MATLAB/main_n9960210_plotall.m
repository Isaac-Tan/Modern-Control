clc
clear all

%% Constants

use_state_variables = 0;
controller_enabled = 1;

constant = [0.1 0.1 1 9.81 1 1 1 1];

m = constant(1);
m0 = constant(2);
k = constant(3);
g = constant(4);
a = constant(5);
alpha = constant(6);
L0 = constant(7);
R = constant(8);

lambda = sqrt(g/a);
b = k/(lambda*m);
beta = lambda*L0/R;
c = m0/m;
I = sqrt(8*alpha*m0*g)/L0;

% %% u_bar = 1
A = [0 1 0;
    1 -b -2;
    0 0.16667 -0.2129];

B = [0;
    0;
    0.1*b/2];

C = [1 0 0];

D = 0;
u_bar = 1;
x_bar = [2*u_bar-1;
    0;
    u_bar];

x0 = [0.8 0 0];

%% Controlability

q = 20; % keeps Voltage < 12
Q = q*eye(3);
R = 1;

[K,S,e] = lqr(A,B,Q,R);

%% Observability

p = e*10;

L = place(A', C', p)'; % Closed Loop A'-C'L' == A-BK

%% Ode solver

h = 0.01;
stoptime = 10;

Lin = sim('Linear_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('nonLinearised_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');

%% Plots


labels = {'x_1 [m]', 'x_2 [m/s]','x_3 [i/I]',...
    'Control Voltage [v]'};
figure(1)
sgtitle(strcat('Maglev Model State-Feedback Response'))
%no hats plotted
for i=1:3
    subplot(4,1,i)
    plot(NL.tout, NL.x(:,i), 'k-',... % NL.tout, NL.x_hat(:,i), 'g-',...
        Lin.tout, Lin.x(:,i),'k--'); %, Lin.tout, Lin.x_hat(:,i), 'k--');
    xlabel('time [s]')
    ylabel(labels{i})
    legend(strcat('x_', num2str(i), ' Non-linear'),... % strcat('x_', num2str(i), ' hat Non-Linear'),...
        strcat('x_', num2str(i), ' Linearised')); %,... %strcat('x_', num2str(i), ' hat Linearised'))
    legend('Location','eastoutside');
end 
% with hats plotted
% for i=1:3
%     subplot(4,1,i)
%     plot(NL.tout, NL.x(:,i), 'b-', NL.tout, NL.x_hat(:,i), 'g-',...
%         Lin.tout, Lin.x(:,i),'r--'  , Lin.tout, Lin.x_hat(:,i), 'k--');
%     xlabel('time [s]')
%     ylabel(labels{i})
%     legend(strcat('x_', num2str(i), ' Non-linear'), strcat('x_', num2str(i), ' hat Non-Linear'),...
%         strcat('x_', num2str(i), ' Linearised'), strcat('x_', num2str(i), ' hat Linearised'));
%     legend('Location','eastoutside');
% end 
subplot(4,1,4)
plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
xlabel('Time [s]'); 
ylabel(labels{4});
legend("v Non Linear", "v Linear");
legend('Location','eastoutside');

%% Fig 5.2
use_state_variables = 1;

Lin = sim('Linear_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('nonLinearised_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');

figure(2)
sgtitle(strcat('Maglev Model Output-Feedback Response'))
%no hats plotted
for i=1:3
    subplot(4,1,i)
    plot(NL.tout, NL.x(:,i), 'k-',... % NL.tout, NL.x_hat(:,i), 'g-',...
        Lin.tout, Lin.x(:,i),'k--'); %, Lin.tout, Lin.x_hat(:,i), 'k--');
    xlabel('time [s]')
    ylabel(labels{i})
    legend(strcat('x_', num2str(i), ' Non-linear'),... % strcat('x_', num2str(i), ' hat Non-Linear'),...
        strcat('x_', num2str(i), ' Linearised')); %,... %strcat('x_', num2str(i), ' hat Linearised'))
    legend('Location','eastoutside');
end 
subplot(4,1,4)
plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
xlabel('Time [s]'); 
ylabel(labels{4});
legend("v Non Linear", "v Linear");
legend('Location','eastoutside');

%% Fig 5.3

figure(3)
sgtitle(strcat('Maglev Model Output-Feedback Response with hats'))
for i=1:3
    subplot(4,1,i)
    plot(NL.tout, NL.x(:,i), 'b-', NL.tout, NL.x_hat(:,i), 'g-',...
        Lin.tout, Lin.x(:,i),'r--'  , Lin.tout, Lin.x_hat(:,i), 'k--');
    xlabel('time [s]')
    ylabel(labels{i})
    legend(strcat('x_', num2str(i), ' Non-linear'), strcat('x_', num2str(i), ' hat Non-Linear'),...
        strcat('x_', num2str(i), ' Linearised'), strcat('x_', num2str(i), ' hat Linearised'));
    legend('Location','eastoutside');
end
subplot(4,1,4)
plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
xlabel('Time [s]'); 
ylabel(labels{4});
legend("v Non Linear", "v Linear");
legend('Location','eastoutside');

%% Fig 5.4

x0 = [0 0 0];

Lin = sim('Linear_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('nonLinearised_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');

figure(4)
sgtitle(strcat('Maglev Model Output-Feedback Response x(0) = 0'))
%no hats plotted
for i=1:3
    subplot(4,1,i)
    plot(NL.tout, NL.x(:,i), 'k-',... % NL.tout, NL.x_hat(:,i), 'g-',...
        Lin.tout, Lin.x(:,i),'k--'); %, Lin.tout, Lin.x_hat(:,i), 'k--');
    xlabel('time [s]')
    ylabel(labels{i})
    legend(strcat('x_', num2str(i), ' Non-linear'),... % strcat('x_', num2str(i), ' hat Non-Linear'),...
        strcat('x_', num2str(i), ' Linearised')); %,... %strcat('x_', num2str(i), ' hat Linearised'))
    legend('Location','eastoutside');
end 
subplot(4,1,4)
plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
xlabel('Time [s]'); 
ylabel(labels{4});
legend("v Non Linear", "v Linear");
legend('Location','eastoutside');

%% Fig 5.5

x0 = [10 0 0];

Lin = sim('Linear_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('nonLinearised_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');

figure(5)
sgtitle(strcat('Maglev Model Output-Feedback Response x(0) = 10'))
%no hats plotted
for i=1:3
    subplot(4,1,i)
    plot(NL.tout, NL.x(:,i), 'k-',... % NL.tout, NL.x_hat(:,i), 'g-',...
        Lin.tout, Lin.x(:,i),'k--'); %, Lin.tout, Lin.x_hat(:,i), 'k--');
    xlabel('time [s]')
    ylabel(labels{i})
    legend(strcat('x_', num2str(i), ' Non-linear'),... % strcat('x_', num2str(i), ' hat Non-Linear'),...
        strcat('x_', num2str(i), ' Linearised')); %,... %strcat('x_', num2str(i), ' hat Linearised'))
    legend('Location','eastoutside');
end 
subplot(4,1,4)
plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
xlabel('Time [s]'); 
ylabel(labels{4});
legend("v Non Linear", "v Linear");
legend('Location','eastoutside');

%% Fig 5.6

x0 = [1.5 0 0];

Lin = sim('Linear_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('nonLinearised_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');

figure(6)
sgtitle(strcat('Maglev Model Output-Feedback Response x(0) = 1.5'))
%no hats plotted
for i=1:3
    subplot(4,1,i)
    plot(NL.tout, NL.x(:,i), 'k-',... % NL.tout, NL.x_hat(:,i), 'g-',...
        Lin.tout, Lin.x(:,i),'k--'); %, Lin.tout, Lin.x_hat(:,i), 'k--');
    xlabel('time [s]')
    ylabel(labels{i})
    legend(strcat('x_', num2str(i), ' Non-linear'),... % strcat('x_', num2str(i), ' hat Non-Linear'),...
        strcat('x_', num2str(i), ' Linearised')); %,... %strcat('x_', num2str(i), ' hat Linearised'))
    legend('Location','eastoutside');
end 
subplot(4,1,4)
plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
xlabel('Time [s]'); 
ylabel(labels{4});
legend("v Non Linear", "v Linear");
legend('Location','eastoutside');

%% Fig 5.7
% 
% % Commented out due to high computational strain
% 
% x0 = [0.8 0 0];
% 
% stoptime = 10000;
% Lin = sim('Linear_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
% NL = sim('nonLinearised_n9960210','solver','ode4','FixedStep','h','StopTime','stoptime');
% 
% figure(7)
% sgtitle(strcat('Maglev Model Output-Feedback Response to t = 10000'))
% %no hats plotted
% for i=1:3
%     subplot(4,1,i)
%     plot(NL.tout, NL.x(:,i), 'k-',... % NL.tout, NL.x_hat(:,i), 'g-',...
%         Lin.tout, Lin.x(:,i),'k--'); %, Lin.tout, Lin.x_hat(:,i), 'k--');
%     xlabel('time [s]')
%     ylabel(labels{i})
%     legend(strcat('x_', num2str(i), ' Non-linear'),... % strcat('x_', num2str(i), ' hat Non-Linear'),...
%         strcat('x_', num2str(i), ' Linearised')); %,... %strcat('x_', num2str(i), ' hat Linearised'))
%     legend('Location','eastoutside');
% end 
% subplot(4,1,4)
% plot(NL.tout, NL.u, 'k-', Lin.tout, Lin.u, 'k--')
% xlabel('Time [s]'); 
% ylabel(labels{4});
% legend("v Non Linear", "v Linear");
% legend('Location','eastoutside');