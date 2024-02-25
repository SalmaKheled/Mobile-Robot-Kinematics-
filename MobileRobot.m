clear all; clc; close all;

%% Simulation Parameter
  dt = 0.1;     %Step
  ts = 100;     %Simulation Time
   t = 0:dt:ts; %Simulation Running Time
   
 %% Wheeled Moble Robot Parameters
   a = 0.05;  % Wheel Radius
   d = 0.1;   % Distance Between Wheel frame and Robot frame along y-axis
   l = 0.3;   % Length of the Robot
   w = 0.2;   % Width of the Robot

  
  %% Inital Conditions
  x0 = 1;  %Initial Position in x-axis
  y0 = 2;  %Initial Position in y-axis
  Theta0 = pi/4;
  
  R0 = [x0;y0;Theta0]; %input values for position 
  R(:,1) = R0; %R is state matrix, and the first column is initial 
  
  
  %% Loop Over Time
 for i=1:length(t)
    Theta = R(3,i); % current orientation in rad
    %Jacobian matrix
    J_ = [cos(Theta),-sin(Theta),0;
             sin(Theta),cos(Theta),0;
             0,0,1];
    %% inputs
    omega_1 = 0.7; % left wheel angular velocity
    omega_2 = 0.5; % right wheel angular velocity
    
    omega = [omega_1;omega_2];
    %% wheel configuration matrix 
    W =[a/2,a/2; 
        0,0;
        -a/(2*d), a/(2*d)];
    % velocity input commands
    zeta(:,i) = W*omega;
      
    % time derivative of generalized coordinates
    R_dot(:,i) = J_ * zeta(:,i);
    
    %% position propagation using Euler method
    R(:,i+1) = R(:,i) + dt * R_dot(:,i); %state update
    % (generalized coordinates)
    
end
 %% Robot Motion Animation
   Robot = [-l/2,l/2,l/2,-l/2,-l/2;
           -w/2,-w/2,w/2,w/2,-w/2;];
           
   figure  
 for i = 1:5:length(t)
     Theta = R(3,i);
     R_Theta = [cos(Theta),-sin(Theta);
                sin(Theta),cos(Theta)];%Rotation Matrix    
     Robot_P = R_Theta *Robot ;  %Robot Position Updated with Time
     
     fill(Robot_P(1,:)+R(1,i),Robot_P(2,:)+R(2,i),'g'); %The Green Box
     hold on, grid on
     axis([0 4 0 4]), axis square
     plot(R(1,1:i),R(2,1:i),'b-');
     legend('Wheeled Robot', 'Path')
     xlabel('x,[M]'); ylabel('y,[M]');
     set(gca, 'fontsize',15)
     pause(0.01)
     hold off
      
 end
     
      
     
      
      