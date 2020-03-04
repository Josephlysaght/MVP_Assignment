%Vehicle parameters
Mass                  = 650;    %Vehicle Mass, kg
Weight_dist_f         = 0.47;   %Weight distribution 47% front, 53% rear
Wheelbase             = 2970;   %Wheelbase length, mm
CoG                   = 285;    %Centre of mass height, mm
Tyre_r                = 340;    %Rear tyre rolling radius, mm
Final_Drive           = 8/31;   %9:31 final drive ratio
GearRatio             = 21/29;  %27/27 gear ratio
Trans_eff             = 0.93;   %93% Transmission efficiency
mu_long               = 1.45;   %Coefficient of friction under braking & acceleration
mu_lat                = 1.38;   %Coefficient of friction under cornering
Aero_balance          = 0.47;   %Aero balance co-located with CoG
CoP_height            = 250;    %Centre of Pressure height, mm
ClA                   = 0.495;  %Downforce coefficient, m^2
CdA                   = 0.34;   %Drag coefficient, m^2
Motor_torque_below_5k = 440;    %Motor torque in region 0-5000 rpm. Can be used to get started, but use lookup for final solution
Motor_torque_lookup   = [0 2000 4000 6000 8000 10000 12000 14000 16000; 440 440 440 440 440 440 440 380 300]; %Motor torque lookup


%Circuit model
Corner_radius   = [125 82 32];      %Radius of each corner,m
Corner_angle    = [90 37.99 52.01]; %Included angle of each corner, degrees
Straight_length = [425 696.31 425]; %Length of each straight, m

%Simulation parameters
Delta_S         = 0.01; %Calculation step size interval, m