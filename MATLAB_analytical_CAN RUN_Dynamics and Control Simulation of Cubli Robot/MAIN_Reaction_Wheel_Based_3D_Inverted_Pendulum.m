clear all
close all
clc

%% Nonlinear Model

% Geometric Properties 
ls = 0.25;                  % housing side length
d = 0.087;                  % wheel offset
% Gravity
g0 = 1.62;                   % moon gravity
% Motor Properties
Km = 40.04;                  % torque constant
Cw = 0.0002694;              % damping coefficient
% Masses
mh = 5;                     % housing mass
mw = 0.75;                  % wheel mass
% Positions
rh = [ls/2; ls/2; ls/2];    % housing COM wrt fixed body frame
rw1 = [ls/2-d; ls/2; ls/2]; % wheel x COM wrt fixed body frame
rw2 = [ls/2; ls/2-d; ls/2]; % wheel y COM wrt fixed body frame
rw3 = [ls/2; ls/2; ls/2-d]; % wheel z COM wrt fixed body frame
% Inertias
Ih = [0.05, 0,    0;        % housing moment of inertia
      0,    0.05, 0;
      0,    0,    0.05];
Iw1 = [0.002, 0,     0;     % wheel moment inertia
       0,     0.001, 0;
       0,     0,     0.001];
Iw2 = [0.001, 0,     0;     % wheel moment inertia
       0,     0.002, 0;
       0,     0,     0.001];
Iw3 = [0.001, 0,     0;     % wheel moment inertia
       0,     0.001, 0;
       0,     0,     0.002]; 


M = mh*skew(rh)+mw*skew(rw1)+mw*skew(rw2)+mw*skew(rw3); % total mass
I = Ih-mh*skew(rh)*skew(rh)+Iw1-mw*skew(rw1)*skew(rw1)+Iw2-mw*skew(rw2)*skew(rw2)+Iw3-mw*skew(rw3)*skew(rw3); % total moment of inertia
Iw = diag([Iw1(1,1); Iw2(2,2); Iw3(3,3)]);      % wheel moment inertia
Ihat = I-Iw;

% Equilibrium State
wh0 = [0; 0; 0];            % housing angular velocity
ww0 = [0; 0; 0];            % wheel angular velocity
phi0 = double(phieq(M,g0)); % housing attitude zyx yaw pitch roll

%% Analysis of Nonlinear Model

% Nonlinear Step Response
figure('Position',[100 100 650 900])
outputNames = ["\alpha (rad)", "\beta (rad)", "\gamma (rad)", "\omega_{h1} (rad/s)", "\omega_{h2} (rad/s)", "\omega_{h3} (rad/s)"];
for input = 1:3
    x0 = [phi0; wh0; ww0];
    u = [0; 0; 0];
    u(input) = 1;
    [t,x] = ode45(@(t,x) Nonlinear_Model(I, Ihat, Iw, M, g0, Km, Cw, x, u),[0, 1],x0);
    for output = 1:6
        index = (output-1)*3+input;
        subplot(6,3,index)
        plot(t, x(:,output))
        if output == 1
            title("Input " + string(input),'FontWeight','normal','FontSize',12)
        end
        if input == 1
            ylabel(outputNames(output),'FontSize',12)
        end
        if output == 6 && input == 2
           xlabel('Time (s)','FontWeight','normal','FontSize',12)
        end
        
    end
end
%% Linear Model

% Inverted kinemetic equation about phi0
F = [0, sin(phi0(3))/cos(phi0(2)),              cos(phi0(3))/cos(phi0(2));
     0, cos(phi0(3)),                           -sin(phi0(3));
     1, sin(phi0(3))*sin(phi0(2))/cos(phi0(2)), cos(phi0(3))*sin(phi0(2))/cos(phi0(2))];
% Linearized gravity components aboiut phi0
dgdp = [0, cos(phi0(2)),              0;
        0, sin(phi0(2))*sin(phi0(3)), -cos(phi0(2))*cos(phi0(3));
        0, sin(phi0(2))*cos(phi0(3)), cos(phi0(2))*sin(phi0(3))];

% Linear State Space
A = [zeros(3,3),      F,          zeros(3,3);
     Ihat^-1*M*dgdp,  zeros(3,3), Cw*Ihat^-1;
     -1*Ihat^-1*M*dgdp, zeros(3,3), -1*Cw*(Ihat^-1+Iw^-1)];
% eliminate rounding errors 
A = (round(A*10^8))/10^8;
 
B = [zeros(3,3);
     (-1*Ihat^-1)*Km;
     (Ihat^-1+Iw^-1)*Km];
C = [eye(6), zeros(6,3)];
D = zeros(6,3);

%% Analysis of Linear Model
% Plant transfer function matrix
ABCD = [A, B; C, D];

% state space system
sys = ss(A,B,C,D);

% Plant transfer function
H = tf(sys);

% Bode plot
figure('Position',[850 100 650 900])
bodemag(H)

% Poles
pA = eig(A);

% Zeros
z = tzero(H);

% Stability
figure('Position',[1600 100 650 650])
pzmap(H)

% DC gain
dcg = dcgain(H);

% Controllability, stabilizability
P = ctrb(A,B);
rankP = rank(P);

% Observability, detectability
rankQ = rank(obsv(A,C));

%% Unobservable and Uncontrollable system
% Reduce A to remove unobservable yaw
A22 = A(2:9,2:9);   
B22 = B(2:9,:);
C22 = [eye(5), zeros(5,3)];
D22 = zeros(5,3);

% Transform A22 to remove uncontrollable state
[Ah,Bh,Ch,T,k] = ctrbf(A22,B22,C22);
Dh = D22;

% eliminate rounding errors
Ah = (round(Ah*10^8))/10^8;
Bh = (round(Bh*10^8))/10^8;
Ch = (round(Ch*10^8))/10^8;


%% Analysis of Linear Model
% Plant transfer function matrix
ABCDh = [Ah, Bh; Ch, Dh];

% state space system
sysh = ss(Ah, Bh, Ch, Dh);

% Plant transfer function
Hh = tf(sysh);

% Bode plot
figure('Position',[850 100 650 900])
bodemag(Hh)

% Poles
pAh = eig(Ah);

% Zeros
zh = tzero(Hh);

% Stability
figure('Position',[1600 100 650 650])
pzmap(Hh)

% DC gain
dcgh = dcgain(Hh);

% Controllability, stabilizability
Ph = ctrb(Ah,Bh);
rankPh = rank(Ph);

% Observability, detectability
rankQh = rank(obsv(Ah,Ch));


%% LQR Control

% LQR tuning parameters
Q = diag([10, 10, 1, 1, 1, 0.1, 0.1, 0.1]);
r = 1/100;
R = r*eye(3);

% LQR gains
[Glqr,Klqr,~] = lqr(Ah,Bh,Q,R);

% Transfer functions
s = tf('s');
Llqr = Glqr*(s*eye(8)-Ah)^-1*Bh;
Slqr = 1/(1+Llqr);
Tlqr = Llqr/(1+Llqr);

%% LQR Plotting

% Singular value plots of transfer functions
figure('Position',[100 100 650 650])
sigma(Llqr, 0.001:0.1:1000)
title("Llqr Singular Values")
figure('Position',[850 100 650 650])
sigma(Slqr, 0.001:0.1:1000)
title("Slqr Singular Values")
figure('Position',[1600 100 650 650])
sigma(Tlqr, 0.001:0.1:1000)
title("Tlqr Singular Values")

% Transformed initial angle offset
x0 = [0.1, 0.1, 0, 0, 0, 0, 0, 0]*T^-1;

% Simulate LQR response 
[tlqr,xlqr] = ode45(@(tlqr,xlqr) Linear_LQR_Model(Ah, Bh, Ch, Dh, Glqr, xlqr),[0, 5],x0);

% Calculate outputs
for i = 1:length(tlqr)
   ylqr(i,1:5) = Ch*(xlqr(i,:))';
end

% Plot response
figure('Position',[100 100 650 900])
lqrtitles = ["0.1 initial pitch angle", "0.1 initial roll angle", "x-axis housing angular velocity", "y-axis housing angular velocity", "z-axis housing angular velocity"];
lqrylabels = ["rad", "rad", "rad/s", "rad/s", "rad/s"];
for i = 1:5    
    subplot(5,1,i)
    plot(tlqr, ylqr(:,i))
    title(lqrtitles(i))
    ylabel(lqrylabels(i))
end
xlabel("t (s)")
 
