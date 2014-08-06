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
camera_pos = zeros(n, 3);
p =  [-.1016; .1524; 1];

%%

camera_pos(1:103, 3) = 0;
camera_pos(1:103, 1:2) = 0;

%%

camera_pos(104:182, 3) = 0;
camera_pos(104:182, 1) = linspace(0, 0, 79);
camera_pos(104:182, 2) = linspace(0, 0.3556, 79);

%%
    
th = linspace(0, -45, 141);
camera_pos(265:405, 3) = th;

for i=1:141
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0.3048; 0];
    p2 = p2 - p;
    camera_pos(264+i, 1:2) = p2(1:2);
end

%%

th = linspace(-45, 45, 308);
camera_pos(406:713, 3) = th;

for i=1:308
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0.3048; 0];
    p2 = p2 - p;
    camera_pos(405+i, 1:2) = p2(1:2);
end

%% 

th = linspace(45, 0, 151);
camera_pos(714:864, 3) = th;

for i=1:151
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0.3048; 0];
    p2 = p2 - p;
    camera_pos(713+i, 1:2) = p2(1:2);
end

%% 

camera_pos(865:1088, 3) = 0;
camera_pos(865:1088, 1) = linspace(0, 0, 224);
camera_pos(865:1088, 2) = linspace(.3048, 0.6223, 224);

%%

th = linspace(0, -45, 114);
camera_pos(1089:1202, 3) = th;

for i=1:114
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0.6223; 0];
    p2 = p2 - p;
    camera_pos(1088+i, 1:2) = p2(1:2);
end

%%

th = linspace(-45, 45, 164);
camera_pos(1203:1366, 3) = th;

for i=1:164
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0.6223; 0];
    p2 = p2 - p;
    camera_pos(1202+i, 1:2) = p2(1:2);
end

%% 

th = linspace(45, 0, 98);
camera_pos(1367:1464, 3) = th;

for i=1:98
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0.6223; 0];
    p2 = p2 - p;
    camera_pos(1366+i, 1:2) = p2(1:2);
end


%%

camera_pos(1464:end, 3) = 0;
camera_pos(1464:end, 1) = 0;
camera_pos(1464:end, 2) = 0.6223;
