%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern 2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

camera_pos = [];
p =  [6.5; 4; 1];

%%
camera_pos(1:51, 3) = 0;
camera_pos(1:51, 1:2) = 0;

%%
th = linspace(0, -25, 90);
camera_pos(51:140, 3) = th;

for i=1:90
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0; 0];
    p2 = p2 - p;
    camera_pos(50+i, 1:2) = p2(1:2);
end

%%
camera_pos(140:227, 3) = -25;
camera_pos(140:227, 1) = 1.0815 + linspace(0, 4.5, 88);
camera_pos(140:227, 2) = -3.1218 + linspace(0, -2.5, 88);

%%

th = linspace(-25, 0, 47);
camera_pos(227:273, 3) = th;

for i=1:47
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [4.5; -2.5; 0];
    p2 = p2 - p;
    camera_pos(226+i, 1:2) = p2(1:2);
end

%%

camera_pos(273:355, 3) = 0;
camera_pos(273:355, 1) = 4.5 + linspace(0, 4.5, 83);
camera_pos(273:355, 2) = -2.5 + linspace(0, 0, 83);

%%

th = linspace(0, 25, 71);
camera_pos(355:425, 3) = th;

for i=1:71
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [9; -2.5; 0];
    p2 = p2 - p;
    camera_pos(354+i, 1:2) = p2(1:2);
end

%%

camera_pos(425:672, 3) = 25;
camera_pos(425:672, 1) = 6.7005 + linspace(0, 10, 248);
camera_pos(425:672, 2) = -0.1278 + linspace(0, 5.5, 248);

%%

th = linspace(25, 0, 53);
camera_pos(672:724, 3) = th;

for i=1:53
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [19; 3; 0];
    p2 = p2 - p;
    camera_pos(671+i, 1:2) = p2(1:2);
end
    
%%

camera_pos(724:899, 3) = 0;
camera_pos(724:899, 1) = 19 + linspace(0, 5.5, 176);
camera_pos(724:899, 2) = 3 + linspace(0, 0, 176);

%%

th = linspace(0, -25, 31);
camera_pos(899:929, 3) = th;

for i=1:31
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [24.5; 3; 0];
    p2 = p2 - p;
    camera_pos(898+i, 1:2) = p2(1:2);
end
    
%%

camera_pos(929:1036, 3) = -25;
camera_pos(929:1036, 1) = 25.5815 + linspace(0, 5, 108);
camera_pos(929:1036, 2) = -0.1218 + linspace(0, -3, 108);

%%

th = linspace(-25, 0, 76);
camera_pos(1036:1111, 3) = th;

for i=1:76
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [29.5; 0; 0];
    p2 = p2 - p;
    camera_pos(1035+i, 1:2) = p2(1:2);
end
    
%%

camera_pos(1111:1176, 3) = 0;
camera_pos(1111:1176, 1) = 29.5;
camera_pos(1111:1176, 2) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
camera_pos = [];
p =  [6.5; 4; 1];

%%

camera_pos(1:103, 3) = 0;
camera_pos(1:103, 1:2) = 0;

%%

camera_pos(103:203, 3) = 0;
camera_pos(103:203, 1) = linspace(0, 14, 101);
camera_pos(103:203, 2) = linspace(0, 0, 101);

%%
    
th = linspace(0, -22, 37);
camera_pos(203:239, 3) = th;

for i=1:37
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [14; 0; 0];
    p2 = p2 - p;
    camera_pos(202+i, 1:2) = p2(1:2);
end

camera_pos(240:250, 3) = -22;
camera_pos(240:250, 1) = camera_pos(239, 1);
camera_pos(240:250, 2) = camera_pos(239, 2);


%%

th = linspace(-22, 22, 41);
camera_pos(250:290, 3) = th;

for i=1:41
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [14; 0; 0];
    p2 = p2 - p;
    camera_pos(249+i, 1:2) = p2(1:2);
end

%% 

th = linspace(22, 0, 17);
camera_pos(290:306, 3) = th;

for i=1:17
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [14; 0; 0];
    p2 = p2 - p;
    camera_pos(289+i, 1:2) = p2(1:2);
end

camera_pos(307:316, 3) = 0;
camera_pos(307:316, 1) = camera_pos(306, 1);
camera_pos(307:316, 2) = camera_pos(306, 2);


%% 

camera_pos(316:416, 3) = 0;
camera_pos(316:416, 1) = linspace(14, 29.5, 101);
camera_pos(316:416, 2) = linspace(0, 0, 101);

%%

th = linspace(0, -22, 16);
camera_pos(416:431, 3) = th;

for i=1:16
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [29.5; 0; 0];
    p2 = p2 - p;
    camera_pos(415+i, 1:2) = p2(1:2);
end

camera_pos(432:440, 3) = -22;
camera_pos(432:440, 1) = camera_pos(431, 1);
camera_pos(432:440, 2) = camera_pos(431, 2);

%%

th = linspace(-22, 22, 45);
camera_pos(440:484, 3) = th;

for i=1:45
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [29.5; 0; 0];
    p2 = p2 - p;
    camera_pos(439+i, 1:2) = p2(1:2);
end

%% 

th = linspace(22, 0, 27);
camera_pos(485:511, 3) = th;

for i=1:27
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [29.5; 0; 0];
    p2 = p2 - p;
    camera_pos(484+i, 1:2) = p2(1:2);
end


%%

camera_pos(511:665, 3) = 0;
camera_pos(511:665, 1) = 29.5;
camera_pos(511:665, 2) = 0;
