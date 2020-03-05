%%%%%%% Seppie's Laptime %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Load in template
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

%-------------------Lets calculate the track parameters---------------%
%%Corner arc lenghts - arc lenght is the radius multiplied by the angle in radians 
Corner_anglerad(1) = degtorad(Corner_angle(1)); %convert deg to radians T1
Corner_anglerad(2) = degtorad(Corner_angle(2)); %convert deg to radians T2
Corner_anglerad(3) = degtorad(Corner_angle(3)); %convert deg to radians T3

%calculates corner lenght (Arc lenght)
Corner_length(1) = Corner_anglerad(1)*Corner_radius(1);
Corner_length(2) = Corner_anglerad(2)*Corner_radius(2);
Corner_length(3) = Corner_anglerad(3)*Corner_radius(3);

%calculating calculation points for straights
Straight_calcpoints(1) = Straight_length(1)/Delta_S;
Straight_calcpoints(1) = Straight_length(1)/Delta_S;
Straight_calcpoints(1) = Straight_length(1)/Delta_S;

%Calculationg overall track length
Track_length = Corner_length(1) + Corner_length(2) + Corner_length(3) + Straight_length(1) + Straight_length(2) + Straight_length(3);

%-----------------------------Start Simulation---------------------------%

%Calculating steady state corner speed
Corner_speed(1) = sqrt((mu_lat*Corner_radius(1)*Mass*9.81)/(Mass-(ClA*mu_lat)));
Corner_speed(2) = sqrt((mu_lat*Corner_radius(2)*Mass*9.81)/(Mass-(ClA*mu_lat))); 
Corner_speed(3) = sqrt((mu_lat*Corner_radius(3)*Mass*9.81)/(Mass-(ClA*mu_lat))); 

%calculating time taken in the corner
Corner_time(1) = Corner_length(1)/Corner_speed(1); 
Corner_time(2) = Corner_length(2)/Corner_speed(2); 
Corner_time(3) = Corner_length(3)/Corner_speed(3); 

%Calculating aero drag during corner
Corner_drag(1) = CdA*Corner_speed(1)^2; 
Corner_drag(2) = CdA*Corner_speed(2)^2; 
Corner_drag(3) = CdA*Corner_speed(3)^2; 

%Calculating aero lift during corner
Corner_lift(1) = ClA*Corner_speed(1)^2; 
Corner_lift(2) = ClA*Corner_speed(2)^2; 
Corner_lift(3) = ClA*Corner_speed(3)^2; 

%Wheel speed out for turn (these values are RPM) 
FWH_EntrySpeed(1) = Corner_speed(1)/(2*pi()*(Tyre_r/1000))*60;
RWH_EntrySpeed(1) = Corner_speed(1)/(2*pi()*(Tyre_r/1000))*60;
FWH_EntrySpeed(2) = Corner_speed(2)/(2*pi()*(Tyre_r/1000))*60;
RWH_EntrySpeed(2) = Corner_speed(2)/(2*pi()*(Tyre_r/1000))*60;
FWH_EntrySpeed(3) = Corner_speed(3)/(2*pi()*(Tyre_r/1000))*60;
RWH_EntrySpeed(3) = Corner_speed(3)/(2*pi()*(Tyre_r/1000))*60;

%Calculations for straight 1
Corner1_speed(1) = Corner_speed(1);
Acceleration(1) = 0;
for n=2:Straight_calcpoints(1)+1
    Acceleration(n) = mu_long((1-Weight_dist_f)+((ClA*Corner1_speed(n-1))*(1-Aero_balance)/(Mass*9.81))+(Acceleration(n-1)*CoG/Wheelbase))-((CdA*Corner_speed(n-1)^2)/(Mass*9.81)); %traction limited acceleration
end



















