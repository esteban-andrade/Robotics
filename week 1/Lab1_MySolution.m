function lab1MySolution()
close all
%% Question 2 
%2.1 Download the image from UTSOnline to file “W1LabEx1_CircularRaceTrack.jpg”, then open
%Matlab and Load the image with:
%imshow('W1LabEx1_CircularRaceTrack.jpg');
%axis on
%2.2 Plot a transform representing the car at [300,550], use the ‘X’ axis of the transform to
%represent the heading of the car (note you will need to use “hold on” to show both)
%2.3 Make the leg lengths of the transform 50 units long so it is visible on the track.
%2.4 Now incrementally multiply the transform of the car by an offset transform and update the
%transform of the car.
%2.5 Display the transform of the car using the Matlab “text” command e.g

set(gcf,'Name',['Question #',num2str(2)])
imshow('W1LabEx1_CircularRaceTrack.jpg' );
axis on;
hold on;

car1_tr = se2(300,550,0);
car1_tr_h = trplot2(car1_tr,'frame','1','color','b','length',50);

% for checking the track diameter the other point of the circle diamater in
% Y will be around 55 (550 - 50 )= 500
% Hence the Circumference is pi*500;
circumference = pi*500;

%Loops increments in a circle will be equal to 360 (circle)
forLoopsIncrement = 360;


car1MoveTr = se2(circumference/forLoopsIncrement,0,0);
car1TurnTr = se2(0,0,-2*pi/forLoopsIncrement);

for i=1:forLoopsIncrement
    car1_tr=car1_tr*car1MoveTr*car1TurnTr;
    try 
        delete(car1_tr_h);
    end;
    try 
        delete (text_h);
    end;
    
    car1_tr_h = trplot2(car1_tr,'frame','1','color','b','length',50);
    message = sprintf([num2str(round(car1_tr(1,:),3,'significant')),'\n' ...
                     ,num2str(round(car1_tr(2,:),3,'significant')),'\n' ...
                     ,num2str(round(car1_tr(3,:),3,'significant'))]);
    text_h = text(10,50,message,'FontSize',10,'Color',[0 0 0]);
    drawnow();
    
end
hold off;

%% Consider a second car driving in the opposite direction
% Plot a transform representing the second car at [300,125], use the ‘X’ axis of the transform to
% represent the heading of the car, make transform red with length of 50.
% 3.2 Make the second car incrementally drives in the opposite direction to the first car
% 3.3 Determine the relative transform between the two cars at each time step
set(gcf,'Name',['Question #',num2str(3)])
imshow('W1LabEx1_CircularRaceTrack.jpg' );
axis on;
hold on;

car1_tr = se2(300,550,0);
car2_tr = se2(310,115,0);
car1_tr_h = trplot2(car1_tr,'frame','1','color','b','length',50);
car2_tr_h = trplot2(car2_tr,'frame','2','color','r','length',50);

%Loops increments in a circle will be equal to 360 (circle)
forLoopsIncrement = 360;

% Hence the Circumference is pi*500; refer to previous example
circumference_car_1 = pi*500;

%for car 2 will be (500 - 115)=
circumference_car_2= pi*(500-115);

%Movement Car 1
car1MoveTr = se2(circumference_car_1/forLoopsIncrement,0,0);
car1TurnTr = se2(0,0,-2*pi/forLoopsIncrement);

%movement Car 2
car2MoveTr = se2(circumference_car_2/forLoopsIncrement, 0, 0);
car2TurnTr = se2(0, 0, 2*pi/forLoopsIncrement);

for i=1:forLoopsIncrement
    car1_tr=car1_tr*car1MoveTr*car1TurnTr;
    car2_tr=car2_tr*car2MoveTr*car2TurnTr;
    try 
        delete(car1_tr_h);
    end;
    try 
        delete(car2_tr_h);
    end;
    try 
        delete (text_h_car1);
    end;
     try 
        delete (text_h_car2);
    end;
    
    %Car 1
    car1_tr_h = trplot2(car1_tr,'frame','1','color','b','length',50);
    message_car1 = sprintf(['CAR 1','\n',num2str(round(car1_tr(1,:),3,'significant')),'\n' ...
                     ,num2str(round(car1_tr(2,:),3,'significant')),'\n' ...
                     ,num2str(round(car1_tr(3,:),3,'significant'))]);
    text_h_car1 = text(10,50,message_car1,'FontSize',10,'Color',[0 0 0]);
    
    %Car 2
    car2_tr_h = trplot2(car2_tr,'frame','2','color','r','length',50);
    message_car2 = sprintf(['CAR 2\n',num2str(round(car2_tr(1,:),3,'significant')),'\n' ...
                     ,num2str(round(car2_tr(2,:),3,'significant')),'\n' ...
                     ,num2str(round(car2_tr(3,:),3,'significant'))]);
    text_h_car2 = text(450,50,message_car2,'FontSize',10,'Color',[0 0 0]);
    
    
    drawnow();
    
end
hold off;
%% 4 Bonus: Consider a distance sensor mounted to the first car (blue) that is tracking the distance to the second car
%4.1 Plot a graph of the distances between the two cars as they drive around the track distance

set(gcf,'Name',['Question #',num2str(4)])
subplot(1,2,1);
imshow('W1LabEx1_CircularRaceTrack.jpg' );
axis on;
hold on;

car1_tr = se2(300,550,0);
car2_tr = se2(310,115,0);
car1_tr_h = trplot2(car1_tr,'frame','1','color','b','length',50);
car2_tr_h = trplot2(car2_tr,'frame','2','color','r','length',50);

%Distance plot
subplot(1,2,2);
xlabel('TimeStep');
ylabel('SensorReading')'
hold on;

%Loops increments in a circle will be equal to 360 (circle)
forLoopsIncrement = 360;

% Hence the Circumference is pi*500; refer to previous example
circumference_car_1 = pi*500;

%for car 2 will be (500 - 115)=
circumference_car_2= pi*(500-115);

%Movement Car 1
car1MoveTr = se2(circumference_car_1/forLoopsIncrement,0,0);
car1TurnTr = se2(0,0,-2*pi/forLoopsIncrement);

%movement Car 2
car2MoveTr = se2(circumference_car_2/forLoopsIncrement, 0, 0);
car2TurnTr = se2(0, 0, 2*pi/forLoopsIncrement);

%Allocate memory
dist = zeros(1,forLoopsIncrement);


for i=1:forLoopsIncrement
    car1_tr=car1_tr*car1MoveTr*car1TurnTr;
    car2_tr=car2_tr*car2MoveTr*car2TurnTr;
    subplot(1,2,1);
    try
        delete(car1_tr_h);
    end;
    try
        delete(car2_tr_h);
    end;
    try
        delete (text_h_car1);
    end;
    try
        delete (text_h_car2);
    end;
    
    
    
    %Car 1
    car1_tr_h = trplot2(car1_tr,'frame','1','color','b','length',50);
    message_car1 = sprintf(['CAR 1','\n',num2str(round(car1_tr(1,:),3,'significant')),'\n' ...
        ,num2str(round(car1_tr(2,:),3,'significant')),'\n' ...
        ,num2str(round(car1_tr(3,:),3,'significant'))]);
    text_h_car1 = text(10,50,message_car1,'FontSize',10,'Color',[0 0 0]);
    
    %Car 2
    car2_tr_h = trplot2(car2_tr,'frame','2','color','r','length',50);
    message_car2 = sprintf(['CAR 2\n',num2str(round(car2_tr(1,:),3,'significant')),'\n' ...
        ,num2str(round(car2_tr(2,:),3,'significant')),'\n' ...
        ,num2str(round(car2_tr(3,:),3,'significant'))]);
    text_h_car2 = text(450,50,message_car2,'FontSize',10,'Color',[0 0 0]);
    
    subplot(1,2,2);
    try
        delete( distPlot_h);
    end;
    dist(i)=sqrt(sum((car1_tr(1:2,3)-car2_tr(1:2,3)).^2));
    distPlot_h = plot(1:i,dist(1:i),'r*');
    drawnow();
    
end
hold off;
hold off;
%%CHECK GIVEN SOLUTIONS
