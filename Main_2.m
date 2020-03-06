%%%%%%% Seppie's Laptime %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Load in template
clear
clc
run('\\brookesf1\s59\17031059\GitHub\MVP_Assignment\template.m');

%---------------------------------------------------Lets calculate the track parameters--------------------------------------------------------%
%how many corners?
NumCorners = length(Corner_radius);
NumStraights = length(Straight_length);

%Initize output struct
Results.CSpeed = zeros(NumCorners,1);
Results.CDrag = zeros(NumCorners,1);
Results.CDownforce = zeros(NumCorners,1);
Results.CTime = zeros(NumCorners,1);
Corner_anglerad = zeros(NumCorners,1);
Results.CDistance = zeros(NumCorners,1);

%%Corner arc lenghts - arc lenght is the radius multiplied by the angle in radians 
for n=1:NumCorners
    Corner_anglerad(n) = degtorad(Corner_angle(n)); %convert deg to radians
end

%calculates corner lenght (Arc lenght)
for n=1:NumCorners
    Results.CDistance(n) = Corner_anglerad(n)*Corner_radius(n);
end

%Points in each section
C1points = round(Results.CDistance(1)/Delta_S);
S1points = round(Straight_length(1)/Delta_S) + C1points;
C2points = round(Results.CDistance(2)/Delta_S) + S1points;
S2points = round(Straight_length(2)/Delta_S) + C2points;
C3points = round(Results.CDistance(2)/Delta_S) + S2points;

%Calculationg overall track length
Track.Length = Results.CDistance(1) + Results.CDistance(2) + Results.CDistance(3) + Straight_length(1) + Straight_length(2) + Straight_length(3);

%Calculating calculation points for track
Track.Calc_points = round(Track.Length/Delta_S)+1;

%Set up full track parameters
Track.Distance = zeros(Track.Calc_points,1);
Track.Section = zeros(Track.Calc_points,1);
Track.Type = zeros(Track.Calc_points,1);
Results.Speed = zeros(Track.Calc_points,1);
Results.Distance = zeros(Track.Calc_points,1);
Results.Time = zeros(Track.Calc_points,1);
Results.Accel = zeros(Track.Calc_points,1);

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

%Calculate corner speeds
for n=1:NumCorners
    Results.CSpeed(n) = sqrt((mu_lat*Corner_radius(n)*Mass*9.81)/(Mass-(ClA*mu_lat)));
end

%Calculate Aerodynamic drag during corner
for n=1:NumCorners
    Results.CDrag(n) = CdA*(Results.CSpeed(n)^2);
end

%Calculate Aerodynamic downforce during corner
for n=1:NumCorners
    Results.CDownforce(n) = ClA*(Results.CSpeed(n)^2);
end

%Elapsed time in each turn 
for n=1:NumCorners
    Results.CTime(n) = Results.CDistance(n)/Results.CSpeed(n);
end

%Calculate Cornering speeds and straight line acceleration
for n=2:Track.Calc_points 
    if Track.Type(n) == 'c'
        %calculate corner speed 
        if Track.Section(n) == 1
            Results.Speed(n) = Results.CSpeed(1);
        elseif Track.Section(n) == 2
            Results.Speed(n) = Results.CSpeed(2);
        else 
            Results.Speed(n) = Results.CSpeed(3);
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
for n=1:NumCorners
    fprintf('C%d - %5.2f m/s \n',n,Results.CSpeed(n));
end
%Cornering Drag 
fprintf('\n----- Aerodynamic Drag during corners----- \n');
for n=1:NumCorners
    fprintf('C%d - %5.2f N \n',n,Results.CDrag(n));
end
%Cornering downforce
fprintf('\n----- Aerodynamic Downforce during corners ----- \n');
for n=1:NumCorners
    fprintf('C%d - %5.2f N \n',n,Results.CDownforce(n));
end
%Corner Distances
fprintf('\n----- Distance traveled in each corner ----- \n');
for n=1:NumCorners
    fprintf('C%d - %5.2f m \n',n,Results.CDistance(n));
end
%Corner Times
fprintf('\n----- Time elapsed in each corner ----- \n');
for n=1:NumCorners
    fprintf('C%d - %5.2f s \n',n,Results.CTime(n));
end

%Removing unnessesary varibles
clear C1points C2points C3points S1points S2points S3points Corner_anglerad

