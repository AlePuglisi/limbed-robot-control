clear 
clc
close all 

% READ DYNAMIXEL WIZARD csv results, AFTER TUNING: 
B2C = csvread("B2C_edit.csv",1,0);
C2F = csvread("C2F_edit.csv",1,0);
F2T = csvread("F2T_edit.csv",1,0);
T2E = csvread("T2E_edit.csv",1,0);
wristH = csvread("wristH_edit.csv",1,0);
wristV = csvread("wristV_edit.csv",1,0);


F2T_offset = 1.364075222;
T2E_offset = 0.2067211047;

B2C = B2C';
B2C_time = B2C(1,:)*10e-3;
B2C_position = (B2C(4,:)-2048)*pi/2048;
B2C_reference = (B2C(3,:)-2048)*pi/2048;

C2F = C2F';
C2F_time = C2F(1,:)*10e-3;
C2F_position = (C2F(4,:)-2048)*pi/2048;
C2F_reference = (C2F(3,:)-2048)*pi/2048;

F2T = F2T';
F2T_time = F2T(1,:)*10e-3;
F2T_position = (F2T(4,:)-2048)*pi/2048 + F2T_offset;
F2T_reference = (F2T(3,:)-2048)*pi/2048 + F2T_offset;

T2E = T2E';
T2E_time = T2E(1,:)*10e-3;
T2E_position = (T2E(4,:)-2048)*pi/2048 +T2E_offset;
T2E_reference = (T2E(3,:)-2048)*pi/2048 + T2E_offset;

wristH = wristH';
wristH_time = wristH(1,:)*10e-3;
wristH_position = (wristH(4,:)-2048)*pi/2048;
wristH_reference = (wristH(3,:)-2048)*pi/2048;

wristV = wristV';
wristV_time = wristV(1,:)*10e-3;
wristV_position = (wristV(4,:)-2048)*pi/2048;
wristV_reference = (wristV(3,:)-2048)*pi/2048;

figure('Name', 'DYNAMIXEL B2C MOTORS TRACKING PERFORMANCE')
plot(B2C_time, B2C_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(B2C_time, B2C_position,'b', 'LineWidth',1.0)
plot(B2C_time, B2C_reference-B2C_position,'r-.')
title('B2C Tracking')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual', 'error')
grid on

figure('Name', 'DYNAMIXEL C2F MOTORS TRACKING PERFORMANCE')
plot(C2F_time, C2F_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(C2F_time, C2F_position,'b', 'LineWidth',1.0)
plot(C2F_time, C2F_reference-C2F_position,'r-.')
title('C2F Tracking')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual', 'error')
grid on

figure('Name', 'DYNAMIXEL F2T MOTORS TRACKING PERFORMANCE')
plot(F2T_time, F2T_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(F2T_time, F2T_position,'b', 'LineWidth',1.0)
plot(F2T_time, F2T_reference-F2T_position,'r-.')
title('F2T Tracking')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual', 'error')
grid on

figure('Name', 'DYNAMIXEL T2E MOTORS TRACKING PERFORMANCE')
plot(T2E_time, T2E_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(T2E_time, T2E_position,'b', 'LineWidth',1.0)
plot(T2E_time, T2E_reference-T2E_position,'r-.')
title('T2E Tracking')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual', 'error')
grid on

figure('Name', 'DYNAMIXEL wristH MOTORS TRACKING PERFORMANCE')
plot(wristH_time, wristH_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(wristH_time, wristH_position,'b', 'LineWidth',1.0)
plot(wristH_time, wristH_reference-wristH_position,'r-.')
title('WristH Tracking')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual', 'error')
grid on

figure('Name', 'DYNAMIXEL wristV MOTORS TRACKING PERFORMANCE')
plot(wristV_time, wristV_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(wristV_time, wristV_position,'b', 'LineWidth',1.0)
plot(wristV_time, wristV_reference-wristV_position,'r-.')
title('WristV Tracking')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on


figure('Name', 'DYNAMIXEL MOTORS TRACKING PERFORMANCE')

subplot(2,3,1)
plot(B2C_time, B2C_reference,'Color', '#0E6926', 'LineStyle','--', 'LineWidth',1.0)
hold on 
plot(B2C_time, B2C_position,'b', 'LineWidth',1.0)
plot(B2C_time, B2C_reference-B2C_position,'r-.')
title('B2C')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on

subplot(2,3,2)
plot(C2F_time, C2F_reference, 'b--')
hold on 
plot(C2F_time, C2F_position, 'r')
title('C2F')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on

subplot(2,3,3)
plot(F2T_time, F2T_reference, 'b--')
hold on 
plot(F2T_time, F2T_position, 'r')
title('F2T')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on

subplot(2,3,4)
plot(T2E_time, T2E_reference, 'b--')
hold on 
plot(T2E_time, T2E_position, 'r')
title('T2E')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on

subplot(2,3,5)
plot(wristH_time, wristH_reference, 'b--')
hold on 
plot(wristH_time, wristH_position, 'r')
title('wristH')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on

subplot(2,3,6)
plot(wristV_time, wristV_reference, 'b--')
hold on 
plot(wristV_time, wristV_position, 'r')
title('wristV')
xlabel('time[s]')
ylabel('joint angle [rad]')
hold off
legend('reference','actual')
grid on

