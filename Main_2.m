%%%%%%% Seppie's Laptime %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Load in template
clear
run('C:\MVP_Assignment\template.m');

%---------------------------------------------------Lets calculate the track parameters--------------------------------------------------------%
%%Corner arc lenghts - arc lenght is the radius multiplied by the angle in radians 
Corner_anglerad(1) = degtorad(Corner_angle(1)); %convert deg to radians T1
Corner_anglerad(2) = degtorad(Corner_angle(2)); %convert deg to radians T2
Corner_anglerad(3) = degtorad(Corner_angle(3)); %convert deg to radians T3

%calculates corner lenght (Arc lenght)
Corner_length(1) = Corner_anglerad(1)*Corner_radius(1);
Corner_length(2) = Corner_anglerad(2)*Corner_radius(2);
Corner_length(3) = Corner_anglerad(3)*Corner_radius(3);

%Points in each section
C1points = round(Corner_length(1)/Delta_S);
S1points = round(Straight_length(1)/Delta_S) + C1points;
C2points = round(Corner_length(2)/Delta_S) + S1points;
S2points = round(Straight_length(2)/Delta_S) + C2points;
C3points = round(Corner_length(2)/Delta_S) + S2points;

%Calculationg overall track length
Track.Length = Corner_length(1) + Corner_length(2) + Corner_length(3) + Straight_length(1) + Straight_length(2) + Straight_length(3);

%Calculating calculation points for track
Track.Calc_points = round(Track.Length/Delta_S)+1;

%Set up full track parameters
Track.Distance = zeros(Track.Calc_points,1);
Track.Section = zeros(Track.Calc_points,1);
Track.Type = zeros(Track.Calc_points,1);

%filling in track data
for n=1:Track.Calc_points
    if n < C1points
        %Sector C1
        Track.Section(n) = 1;
        Track.Type(n) = 'c';
        Track.Radius(n) = Corner_radius(1);
    elseif n < S1points
        %Sector S1
        Track.Section(n) = 1;
        Track.Type(n) = 's';
        Track.Radius(n) = 0;
    elseif n < C2points
        %Sector C2
        Track.Section(n) = 2;
        Track.Type(n) = 'c';
        Track.Radius(n) = Corner_radius(2);
    elseif n < S2points
        %Sector S2
        Track.Section(n) = 2;
        Track.Type(n) = 's';
        Track.Radius(n) = 0;
    elseif n < C3points
        %Sector C3
        Track.Section(n) = 3;
        Track.Type(n) = 'c';
        Track.Radius(n) = Corner_radius(3);
    else 
        %Sector S3
        Track.Section(n) = 3;
        Track.Type(n) = 's';
        Track.Radius(n) = 0;
    end
end

%------------------------------------ Start Simulation Steps ------------------------------------------------%

%Initize output struct
Results.Speed = zeros(Track.Calc_points,1);
Results.Distance = zeros(Track.Calc_points,1);
Results.Time = zeros(Track.Calc_points,1);
Results.Accel = zeros(Track.Calc_points,1);

%Calculate Cornering speeds and straight line acceleration
for n=2:Track.Calc_points 
    if Track.Type(n) == 'c'
        %calculate corner speed 
        Results.Speed(n) = sqrt((mu_lat*Track.Radius(n)*Mass*9.81)/(Mass-(ClA*mu_lat)));
        Results.Distance(n) = Results.Distance(n-1)+Delta_S;
    else 
        %calculate acceleration
        %Traction Limit
        Wr = (1-Weight_dist_f)*Mass*9.81;
        Aero_downforce = ClA*(Results.Speed(n-1)^2)*(1-Aero_balance);
        weight_transfer_term = ((Mass*Results.Accel(n-1)*(CoG/1000))/(Wheelbase/1000));
        Aero_drag = CdA*(Results.Speed(n-1)^2);
        TractionLimitedA = (mu_long*(Wr+Aero_downforce+weight_transfer_term) - Aero_drag)/Mass; 
        
        %Power Limit 
        F_tractive = (Motor_torque_below_5k*(1/GearRatio)*(1/Final_Drive))/(Tyre_r/1000);
        PowerLimitedA = (F_tractive-Aero_drag)/Mass; 
        
        %Limit arbitration
        if TractionLimitedA > PowerLimitedA
            Results.Accel(n) = PowerLimitedA;
        else
            Results.Accel(n) = TractionLimitedA;
        end
        
        %Speed calculation
        Results.Speed(n) = sqrt((Results.Speed(n-1)^2)+(2*Results.Accel(n)*Delta_S));
        Results.Distance(n) = Results.Distance(n-1)+Delta_S;
        
    end
end




%Plot the outputs
plot(Results.Distance,Results.Speed)
title('Speed Trace Simulation Output')
xlabel('Distance (m)') 
ylabel('Speed (m/s)') 





%Removing unnessesary varibles
clear C1points C2points C3points S1points S2points S3points

