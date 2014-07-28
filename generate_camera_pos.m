camera_pos = zeros(n, 3);
p =  [-.1016; .1524; 1];

%%
camera_pos(1:44, 3) = 0;
camera_pos(1:44, 1:2) = 0;

%%
th = linspace(0, -45, 56);
camera_pos(45:100, 3) = th;

for i=1:56
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0; 0; 0];
    p2 = p2 - p;
    camera_pos(44+i, 1:2) = p2(1:2);
end

%%
camera_pos(100:313, 3) = -45;
camera_pos(100:313, 1) = linspace(.1375, .3788, 214);
camera_pos(100:313, 2) = linspace(.02721, .2431, 214);

%%

th = linspace(-45, 0, 150);
camera_pos(314:463, 3) = th;

for i=1:150
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0.2413; 0.2159; 0];
    p2 = p2 - p;
    camera_pos(313+i, 1:2) = p2(1:2);
end

%%

camera_pos(464:545, 3) = 0;
camera_pos(464:545, 1) = linspace(0.2413, 0.2413, 82);
camera_pos(464:545, 2) = linspace(0.2159, 0.3175, 82);

%%

th = linspace(0, 45, 107);
camera_pos(546:652, 3) = th;

for i=1:107
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [0.2413; 0.3175; 0];
    p2 = p2 - p;
    camera_pos(545+i, 1:2) = p2(1:2);
end

%%

camera_pos(653:831, 3) = 45;
camera_pos(653:831, 1) = linspace(0.1633, -.1796, 179);
camera_pos(653:831, 2) = linspace(0.2010, .5185, 179);

%%

th = linspace(45, 0, 68);
camera_pos(832:899, 3) = th;

for i=1:68
    R = [cosd(th(i)) -sind(th(i)) 0; sind(th(i)) cosd(th(i)) 0; 0 0 1];
    p2 = R * p + [-0.1016; 0.635; 0];
    p2 = p2 - p;
    camera_pos(831+i, 1:2) = p2(1:2);
end
    
%%

camera_pos(900:end, 3) = 0;
camera_pos(900:end, 1) = linspace(-0.1016, -0.1016, 216);
camera_pos(900:end, 2) = linspace(0.6350, .7366, 216);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
camera_pos = zeros(n, 3);
p =  [-.1016; .1524; 1];

%%

camera_pos(1:5, 3) = 0;
camera_pos(1:5, 1:2) = 0;

%%

camera_pos(6:264, 3) = 0;
camera_pos(6:264, 1) = linspace(0, 0, 259);
camera_pos(6:264, 2) = linspace(0, .3048, 259);

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
