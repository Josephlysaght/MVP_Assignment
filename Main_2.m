%%%%%%% Seppie's Laptime %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Load in template
clear
clc
run('\\brookesf1\s59\17031059\GitHub\MVP_Assignment\template.m');

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

%Calculate corner speeds
Results.C1Speed = sqrt((mu_lat*Corner_radius(1)*Mass*9.81)/(Mass-(ClA*mu_lat)));
Results.C2Speed = sqrt((mu_lat*Corner_radius(2)*Mass*9.81)/(Mass-(ClA*mu_lat)));
Results.C3Speed = sqrt((mu_lat*Corner_radius(3)*Mass*9.81)/(Mass-(ClA*mu_lat)));

%Calculate Aerodynamic drag during corner
Results.C1Drag = CdA*(Results.C1Speed^2);
Results.C2Drag = CdA*(Results.C2Speed^2);
Results.C3Drag = CdA*(Results.C3Speed^2);

%Calculate Aerodynamic downforce during corner
Results.C1Downforce = ClA*(Results.C1Speed^2);
Results.C2Downforce = ClA*(Results.C2Speed^2);
Results.C3Downforce = ClA*(Results.C3Speed^2);

%Elapsed time in each turn 
Results.C1Time = Corner_length(1)/Results.C1Speed;
Results.C2Time = Corner_length(2)/Results.C2Speed;
Results.C3Time = Corner_length(3)/Results.C3Speed;

%Calculate Cornering speeds and straight line acceleration
for n=2:Track.Calc_points 
    if Track.Type(n) == 'c'
        %calculate corner speed 
        if Track.Section(n) == 1
            Results.Speed(n) = Results.C1Speed;
        elseif Track.Section(n) == 2
            Results.Speed(n) = Results.C2Speed;
        else 
            Results.Speed(n) = Results.C3Speed;
        end
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

%Because track is closed the last point has to equal the first point.
Results.Speed(Track.Calc_points) = Results.Speed(2);

%Calculate braking
for n=Track.Calc_points-1:-1:1
    if Track.Type(n) == 's'
        
        %calculate braking acceleration
        StaticForce = Mass*9.81;
        Aero_downforce = ClA*(Results.Speed(n+1)^2);
        Aero_drag = CdA*(Results.Speed(n+1)^2);
        Braking_A = (mu_long*(StaticForce+Aero_downforce) + Aero_drag)/Mass;
        
        %calculate braking speed
        BSpeed = sqrt((Results.Speed(n+1)^2)+(2*Braking_A*Delta_S));
        
        %Decide if braking or previous speed wins
        if BSpeed < Results.Speed(n)
            Results.Speed(n) = BSpeed;
        end
    end
end


%Plot the outputs
plot(Results.Distance,Results.Speed)
title('Speed Trace Simulation Output')
xlabel('Distance (m)') 
ylabel('Speed (m/s)') 

%------------------- Outputs ---------------------------------------
%Corner Speeds
fprintf('----- Corner Speeds ----- \n');
fprintf('C1 - %5.2f m/s \n',Results.C1Speed);
fprintf('C2 - %5.2f m/s \n',Results.C2Speed);
fprintf('C3 - %5.2f m/s \n\n',Results.C3Speed);
%Cornering Drag 
fprintf('----- Aerodynamic Drag during corners----- \n');
fprintf('C1 - %5.2f N \n',Results.C1Drag);
fprintf('C2 - %5.2f N \n',Results.C2Drag);
fprintf('C3 - %5.2f N \n\n',Results.C3Drag);
%Cornering downforce
fprintf('----- Aerodynamic Downforce during corners ----- \n');
fprintf('C1 - %5.2f N \n',Results.C1Downforce);
fprintf('C2 - %5.2f N \n',Results.C2Downforce);
fprintf('C3 - %5.2f N \n\n',Results.C3Downforce);
%Corner Distances
fprintf('----- Distance traveled in each corner ----- \n');
fprintf('C1 - %5.2f m \n',Corner_length(1));
fprintf('C2 - %5.2f m \n',Corner_length(2));
fprintf('C3 - %5.2f m \n\n',Corner_length(3));
%Corner Times
fprintf('----- Time elapsed in each corner ----- \n');
fprintf('C1 - %5.2f s \n',Results.C1Time);
fprintf('C2 - %5.2f s \n',Results.C2Time);
fprintf('C3 - %5.2f s \n\n',Results.C2Time);

%Removing unnessesary varibles
clear C1points C2points C3points S1points S2points S3points

