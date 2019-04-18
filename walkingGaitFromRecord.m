motor1 = [];
motor2 = [];
motor3 = [];

for i = 1:length(data)
    motor1(end+1) = data(i,1);
    motor2(end+1) = data(i,2);
    motor3(end+1) = data(i,3);
end

index = round(linspace(30,146,30));
motor1 = motor1(index);
motor2 = motor2(index);
motor3 = motor3(index);

center = [(motor1(1)+motor1(end))/2 (motor2(end)+motor2(1))/2];
startAng = atan2(motor2(end)-center(2),motor1(end)-center(1));
endAng = atan2(motor2(1)-center(2),motor1(1)-center(1));
radius = sqrt((motor2(end)-center(2))^2+(motor1(end)-center(1))^2);
angles = linspace(startAng,endAng+2*pi,12);
angles(1) = []; angles(end) = [];
motor1Add = center(1)+radius*cos(angles);
motor2Add = center(2)-radius*sin(angles);

motor1 = [motor1 motor1Add];
motor2 = [motor2 motor2Add];
motor3 = [motor3 linspace(motor3(end),motor3(1),10)];

m1String = "";
m2String = "";
m3String = "";

for i = 1:length(motor1)
    m1String = m1String + num2str(motor1(i)) + ',';
     m2String = m2String + num2str(motor2(i)) + ',';
     m3String = m3String + num2str(motor3(i)) + ',';
end

