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
Straight_calcpoints(1) = (Straight_length(1)/Delta_S)+1;
Straight_calcpoints(2) = (Straight_length(2)/Delta_S)+1;
Straight_calcpoints(3) = (Straight_length(3)/Delta_S)+1;

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
Straight1_speed = zeros(Straight_calcpoints(1),1); %initilize the matrix
Straight1_A_T = zeros(Straight_calcpoints(1),1); %initilize the matrix
Straight1_A_P = zeros(Straight_calcpoints(1),1); %initilize the matrix
Straight1_A_C = zeros(Straight_calcpoints(1),1); %initilize the matrix
Straight1_speed(1) = Corner_speed(1);

%Iteration loop for straight line speed
for n=2:Straight_calcpoints(1)
    
    %Calculate Traction Limit
    Wr = (1-Weight_dist_f)*Mass*9.81;
    Aero_downforce = ClA*(Straight1_speed(n-1)^2)*(1-Aero_balance);
    weight_transfer_term = ((Mass*Straight1_A_T(n-1)*(CoG/1000))/(Wheelbase/1000));
    Aero_drag = CdA*(Straight1_speed(n-1)^2);
    Straight1_A_T(n) = (mu_long*(Wr+Aero_downforce+weight_transfer_term) - Aero_drag)/Mass; %F=ma
    
    %Calculating power limited acceleration
    F_tractive = (Motor_torque_below_5k*(1/GearRatio)*(1/Final_Drive))/(Tyre_r/1000);
    Straight1_A_P(n) = (F_tractive-Aero_drag)/Mass; %F=ma
    
    if Straight1_A_T(n) > Straight1_A_P(n)
        Straight1_A_C(n) = Straight1_A_P(n);
    else
        Straight1_A_C(n) = Straight1_A_T(n);
    end 
    
    Straight1_speed(n) = sqrt((Straight1_speed(n-1)^2)+(2*Straight1_A_C(n)*Delta_S)); %using SUVAT equation to calculate new speed for given acceleration value. 
end


%Calculations for straight 2
Straight2_speed = zeros(Straight_calcpoints(2),1); %initilize the matrix
Straight2_A_T = zeros(Straight_calcpoints(2),1); %initilize the matrix
Straight2_A_P = zeros(Straight_calcpoints(2),1); %initilize the matrix
Straight2_A_C = zeros(Straight_calcpoints(2),1); %initilize the matrix
Straight2_speed(2) = Corner_speed(2);

%Iteration loop for straight line speed
for n=2:Straight_calcpoints(2)
    
    %Calculate Traction Limit
    Wr = (1-Weight_dist_f)*Mass*9.81;
    Aero_downforce = ClA*(Straight2_speed(n-1)^2)*(1-Aero_balance);
    weight_transfer_term = ((Mass*Straight2_A_T(n-1)*(CoG/1000))/(Wheelbase/1000));
    Aero_drag = CdA*(Straight2_speed(n-1)^2);
    Straight2_A_T(n) = (mu_long*(Wr+Aero_downforce+weight_transfer_term) - Aero_drag)/Mass; %F=ma
    
    %Calculating power limited acceleration
    F_tractive = (Motor_torque_below_5k*(1/GearRatio)*(1/Final_Drive))/(Tyre_r/1000);
    Straight2_A_P(n) = (F_tractive-Aero_drag)/Mass; %F=ma
    
    if Straight2_A_T(n) > Straight2_A_P(n)
        Straight2_A_C(n) = Straight2_A_P(n);
    else
        Straight2_A_C(n) = Straight2_A_T(n);
    end 
    
    Straight2_speed(n) = sqrt((Straight2_speed(n-1)^2)+(2*Straight2_A_C(n)*Delta_S)); %using SUVAT equation to calculate new speed for given acceleration value. 
end


%Calculations for straight 3
Straight3_speed = zeros(Straight_calcpoints(3),1); %initilize the matrix
Straight3_A_T = zeros(Straight_calcpoints(3),1); %initilize the matrix
Straight3_A_P = zeros(Straight_calcpoints(3),1); %initilize the matrix
Straight3_A_C = zeros(Straight_calcpoints(3),1); %initilize the matrix
Straight3_speed(3) = Corner_speed(3);

%Iteration loop for straight line speed
for n=2:Straight_calcpoints(3)
    
    %Calculate Traction Limit
    Wr = (1-Weight_dist_f)*Mass*9.81;
    Aero_downforce = ClA*(Straight3_speed(n-1)^2)*(1-Aero_balance);
    weight_transfer_term = ((Mass*Straight3_A_T(n-1)*(CoG/1000))/(Wheelbase/1000));
    Aero_drag = CdA*(Straight3_speed(n-1)^2);
    Straight3_A_T(n) = (mu_long*(Wr+Aero_downforce+weight_transfer_term) - Aero_drag)/Mass; %F=ma
    
    %Calculating power limited acceleration
    F_tractive = (Motor_torque_below_5k*(1/GearRatio)*(1/Final_Drive))/(Tyre_r/1000);
    Straight3_A_P(n) = (F_tractive-Aero_drag)/Mass; %F=ma
    
    if Straight3_A_T(n) > Straight3_A_P(n)
        Straight3_A_C(n) = Straight3_A_P(n);
    else
        Straight3_A_C(n) = Straight3_A_T(n);
    end 
    
    Straight3_speed(n) = sqrt((Straight3_speed(n-1)^2)+(2*Straight3_A_C(n)*Delta_S)); %using SUVAT equation to calculate new speed for given acceleration value. 
end














