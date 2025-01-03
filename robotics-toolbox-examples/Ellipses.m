clc
clear
close all

%% ROBOTIXTOOLBOX by Peter Cork, APPENDIX E: 
%   Ellipses
E = [2 -1; -1 1]; 
figure('Name', 'Simple Ellipse')
plot_ellipse(E, 'b')
xlabel('x')
ylabel('y')
grid on

% eigenvectors/eigenvalues
[x,e] = eig(E); 
r = sqrt(diag(e)); % radii = sqrt(eigenvalue)

axis1_n = x(:,1)*r(1);
axis2_n = x(:,2)*r(2);

line([0 axis1_n(1)], [0 axis1_n(2)], 'Color', 'r'); 
line([0 axis2_n(1)], [0 axis2_n(2)], 'Color', 'r'); 

theta = atan2(x(2,2), x(1,2))*180/pi;

% show how the ellipsoid is plotted from the unit circle 
th = linspace(0, 2*pi, 50); 
y = [cos(th); sin(th)]; 

x = (sqrtm(E) * y)'; 
figure('Name', 'ELlipse plot from circle')
plot(x(:,1), x(:,2), 'b'); 
hold on 
plot(y(1,:), y(2,:), 'r--');
grid on 
