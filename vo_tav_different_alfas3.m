function safetyValue = vo_tav_different_alfas3(VO_x, VO_y, investigate_velocities_x, investigate_velocities_y, MAXrobotvelocity, Topt)
% This function will define how far is the closest VO area from the
% velocities and it is normalized

% Input:
% VO_x : boundary points of the VOs (x coordinate), MATRIX: 4*nObstacles (1-4, 2-3,  3==4)
% VO_y : boundary points of the VOs (y coordinate), MATRIX: 4*nObstacles (1-4, 2-3,  3==4)
% investigate_velocities_x: velocities on the grid (x coordinate)
% investigate_velocities_y:  velocities on the grid (y coordinate)
% MAXrobotvelocity: the maximum speed of the robot
% samplingTime: how long will be used one selected velocity component

% Output:
%   safetyValue: the normalised minimum distance from the VO-s

Number_Obstacles = size(VO_x,2);

% for i=1:Number_Obstacles
%     for j=1:2
%         if(j==1)
%             p=4;
%             z=1;
%         else
%             p=3;
%             z=2;
%         end
%
%         t=((x-VO_x(p,i))*(VO_x(z,i)-VO_x(p,i))+(y-VO_y(p,i))*(VO_y(z,i)-VO_y(p,i)))/((VO_x(z,i)-VO_x(p,i))^2+(VO_y(z,i)-VO_y(p,i))^2);
%         % ide jön még, ha kisebb t, mint 0 akkor 3,4, ha nagyobb mint 1
%         % akkor vo_2,1

x1_ertek = zeros(size(investigate_velocities_x,1),Number_Obstacles);
y1_ertek = zeros(size(investigate_velocities_x,1),Number_Obstacles);

x2_ertek = zeros(size(investigate_velocities_x,1),Number_Obstacles);
y2_ertek = zeros(size(investigate_velocities_x,1),Number_Obstacles);
for i = 1: size(investigate_velocities_x,1)
    
    t1=((investigate_velocities_x(i)-VO_x(4,:)).*(VO_x(1,:)-VO_x(4,:))+(investigate_velocities_y(i)-VO_y(4,:)).*(VO_y(1,:)-VO_y(4,:)))./((VO_x(1,:)-VO_x(4,:)).^2+(VO_y(1,:)-VO_y(4,:)).^2);
    t2 =((investigate_velocities_x(i)-VO_x(3,:)).*(VO_x(2,:)-VO_x(3,:))+(investigate_velocities_y(i)-VO_y(3,:)).*(VO_y(2,:)-VO_y(3,:)))./((VO_x(2,:)-VO_x(3,:)).^2+(VO_y(2,:)-VO_y(3,:)).^2);
    
    x1_ertek(i,t1<0) = VO_x(4,t1<0);
    x1_ertek(i,t1>1) = VO_x(1,t1>1);
    x1_ertek(i,(t1>=0 & t1<=1)) = t1(t1>=0 & t1<=1).*VO_x(1,(t1>=0 & t1<=1))+(1-t1(t1>=0 & t1<=1)).*VO_x(4,(t1>=0 & t1<=1));
    y1_ertek(i,t1<0) = VO_y(4,t1<0);
    y1_ertek(i,t1>1) = VO_y(1,t1>1);
    y1_ertek(i,(t1>=0 & t1<=1)) = t1(t1>=0 & t1<=1).*VO_y(1,(t1>=0 & t1<=1))+(1-t1(t1>=0 & t1<=1)).*VO_y(4,(t1>=0 & t1<=1));
    
    
    x2_ertek(i,t2<0) = VO_x(3,t2<0);
    x2_ertek(i,t2>1) = VO_x(2,t2>1);
    x2_ertek(i,(t2>=0 & t2<=1)) = t2(t2>=0 & t2<=1).*VO_x(2,(t2>=0 & t2<=1))+(1-t2(t2>=0 & t2<=1)).*VO_x(3,(t2>=0 & t2<=1));
    y2_ertek(i,t2<0) = VO_y(3,t2<0);
    y2_ertek(i,t2>1) = VO_y(2,t2>1);
    y2_ertek(i,(t2>=0 & t2<=1)) = t2(t2>=0 & t2<=1).*VO_y(2,(t2>=0 & t2<=1))+(1-t2(t2>=0 & t2<=1)).*VO_y(3,(t2>=0 & t2<=1));
    
    
    % Csak a teszteléshez:
%     A = plot(x1_ertek(i,:),y1_ertek(i,:),'bx');
%     hold on
%     B = plot(x2_ertek(i,:),y2_ertek(i,:),'bo')
%     
%     delete(A)
%     delete(B)
    % ugyanezt kell y1-gyel is, akadályonként távolságot számolni,
    % min-at elmenteni
    % megcsinálni a másik egyenesre is u.e-et és amelyik kisebb táv az
    % (max távra figyelve
    % kerül ahhoz a seb-hez
    
end


    % the min. distance from the different VO-s
vo_tavolsag = min(distance(investigate_velocities_x*ones(1,Number_Obstacles),investigate_velocities_y*ones(1,Number_Obstacles),x1_ertek,y1_ertek), distance(investigate_velocities_x*ones(1,Number_Obstacles),investigate_velocities_y*ones(1,Number_Obstacles),x2_ertek,y2_ertek));


% vo_tavolsag = min(vo_tavolsag,[],2);
vo_tavolsag(vo_tavolsag>MAXrobotvelocity*Topt) = MAXrobotvelocity*Topt;
        
safetyValue =  vo_tavolsag;
safetyValue = min(safetyValue,[],2);
%safetyValue = safetyValue./max(safetyValue);
safetyValue = safetyValue./(MAXrobotvelocity*Topt);

        
end


