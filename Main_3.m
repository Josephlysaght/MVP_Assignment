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
Cpoints = zeros(NumCorners,1);
Spoints = zeros(NumStraights,1);
Results.CExitWheelSpeed = zeros(NumStraights,1);
Results.CExitTorque = zeros(NumStraights,1);
Results.CExitDownforceR = zeros(NumStraights,1);
Results.CExitVertLoadR = zeros(NumStraights,1);
Results.FirstA = zeros(NumStraights,1);
Results.PlimitedA = zeros(NumStraights,1);
Results.TlimitedA = zeros(NumStraights,1);

%%Corner arc lenghts - arc lenght is the radius multiplied by the angle in radians 
for n=1:NumCorners
    Corner_anglerad(n) = degtorad(Corner_angle(n)); %convert deg to radians
end

%calculates corner lenght (Arc lenght)
for n=1:NumCorners
    Results.CDistance(n) = Corner_anglerad(n)*Corner_radius(n);
end

%Calculation Points in each section
for n=1:NumCorners
    Cpoints(n) = round(Results.CDistance(1)/Delta_S);
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

%Wheel speed at the beginning of first calculation for straight
for n=1:NumStraights
    Results.CExitWheelSpeed(n) = Results.CSpeed(n)/(2*pi()*(Tyre_r/1000))*60;
end

%Motor torque at the beginning of first calculation 
RPM = Motor_torque_lookup(1,:); %split the array in two 
Torque = Motor_torque_lookup(2,:);
for n=1:NumStraights
    EngineRPM = Results.CExitWheelSpeed(n)/((1/GearRatio)*(1/Final_Drive)); %Calculate the engine RPM from wheel speed
    Results.CExitTorque(n) = interp1(RPM,Torque,EngineRPM,'linear'); %Linear interpolation to lookup motor torque for given Engine RPM 
end

%Drag force at the beginning of first calculation
    %this is the same as corner drag (can just use that when needed)

%Downforce on rear wheels at the beginning of first calculation
for n=1:NumStraights
   Results.CExitDownforceR(n) = Results.CDownforce(n)*(1-Aero_balance);
end

%Total vertical load on rear wheels at the beginning of first calculation
for n=1:NumStraights
    Results.CExitVertLoadR(n) = Mass*9.81*(1-Weight_dist_f)+ Results.CExitDownforceR(n);
end

%Traction limited acceleration between 0-0.01m after corner exit
for n=1:NumStraights
    Results.TlimitedA(n) = (mu_long*Results.CExitVertLoadR(n)-Results.CDrag(n))/Mass;
end

%Power limited acceleration between 0-0.01 after corner exit
for n=1:NumStraights
    Results.PlimitedA(n) = (((Results.CExitTorque(n)*(1/GearRatio)*(1/Final_Drive))/(Tyre_r/1000))-Results.CDrag(n))/Mass;
end

%Actual acceleration @0.01m after corner exit
for n=1:NumStraights
    if Results.PlimitedA(n) > Results.TlimitedA(n)
        Results.FirstA(n) = Results.TlimitedA(n);
    else
        Results.FirstA(n) = Results.PlimitedA(n);
    end
end

%Time taken to cover first 0-0.01m of straight
for n=1:NumStraights
    Results.TimefirstStep(n) = (Delta_S/(Results.CSpeed(n)+0.5*Results.FirstA(n)))^(1/3);
end

%How many calculation points for each straight? 
for n=1:NumStraights
    Spoints(n) = round(Straight_length(n)/Delta_S);
end

%Initilize arrays for full straight lenght
for n=1:NumStraights
    StraightSpeed. = zeros(Spoints(n),1);
end

%Speed along the entire length of first straight (in 0.01m increments)
for n=1:NumStraights
    for i=1:Spoints(n)
        A = 7.64;
        %using SUVAT calculate the speed 
        Results.StraightSpeed(n:i) = sqrt((Results.StraightSpeed(n:i-1))^2+2*Delta_S*A);
    end
end

%Speed 200m from the beginning of the first straight

%Time taken to complete the length of the straight

%------------------- Outputs ---------------------------------------
fprintf('----------------------- Part 1 - Cornering  ---------------------\n');
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

fprintf('----------------------- Part 2 - Acceleration at S1  ---------------------\n');
fprintf('Wheel speed at the beginning of first calculation = %5.0f RPM\n',Results.CExitWheelSpeed(1));
fprintf('Motor torque at the beginning of first calculation = %3.0f N/m\n',Results.CExitTorque(1));
fprintf('Drag force at the beginning of first calculation = %5.2f N\n',Results.CDrag(1));
fprintf('Downforce on rear wheels at the beginning of first calculation = %5.2f N\n',Results.CExitDownforceR(1));
fprintf('Total vertical load on rear wheels at the beginning of first calculation = %5.2f N\n',Results.CExitVertLoadR(1));
fprintf('Traction limited acceleration between 0-0.01m after corner exit = %5.2f m/s\n',Results.TlimitedA(1));
fprintf('Power limited acceleration between 0-0.01 after corner exit = %5.2f m/s\n',Results.PlimitedA(1)');
fprintf('Actual acceleration @0.01m after corner exit = %5.2f m/s\n',Results.FirstA(1)');
fprintf('Time taken to cover first 0-0.01m of straight = %2.4f s\n',Results.TimefirstStep(1));

%Removing unnessesary varibles
clear Corner_anglerad Cpoints Spoints

