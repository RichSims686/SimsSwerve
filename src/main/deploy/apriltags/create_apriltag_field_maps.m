clear all;
clc;

fieldLength = 651.25 *2.54/100;
fieldWidth =  315.5 *2.54/100;

tag(1).translationInches =   [+85.25, +24.00, 30.25]; % x, y, z
tag(1).rotationYPRRadians   =   [0.0, 0.0, pi];     % roll, pitch, yaw

tag(2).translationInches =   [+85.25, -24.00, 30.25]; % x, y, z
tag(2).rotationYPRRadians   =   [0.0, 0.0, pi];     % roll, pitch, yaw

tag(3).translationInches =   [-85.25, -23.00, 28.25]; % x, y, z
tag(3).rotationYPRRadians   =   [0.0, 0.0, 0.0];     % roll, pitch, yaw

tag(4).translationInches =   [-85.25, +25.00, 29.00]; % x, y, z
tag(4).rotationYPRRadians   =   [0.0, 0.0, 0.0];     % roll, pitch, yaw

tag(5).translationInches =   [0,0,0]; % x, y, z
tag(5).rotationYPRRadians   =   [0.0, 0.0, 0.0];     % roll, pitch, yaw

for k=1:numel(tag)
    tag(k).translationMeters = tag(k).translationInches * 2.54/100;
end


%% photonvision json

for k=1:numel(tag)
    roll =  tag(k).rotationYPRRadians(1);
    pitch = tag(k).rotationYPRRadians(2);
    yaw =   tag(k).rotationYPRRadians(3);
   
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    
    W = cr * cp * cy + sr * sp * sy;
    X = sr * cp * cy - cr * sp * sy;
    Y = cr * sp * cy + sr * cp * sy;
    Z = cr * cp * sy - sr * sp * cy;

    tags(k).ID = k;
    tags(k).pose.translation.x = tag(k).translationMeters(1) + fieldLength/2;
    tags(k).pose.translation.y = tag(k).translationMeters(2) + fieldWidth/2;
    tags(k).pose.translation.z = tag(k).translationMeters(3);
    tags(k).pose.rotation.quaternion.W = W;
    tags(k).pose.rotation.quaternion.X = X;
    tags(k).pose.rotation.quaternion.Y = Y;
    tags(k).pose.rotation.quaternion.Z = Z;
end
s.tags = tags;

s.field.length = fieldLength;
s.field.width  = fieldWidth;

photonvision = jsonencode(s, 'PrettyPrint', true);
fid = fopen('sims-basement.json', 'wt');
fprintf(fid, '%s', photonvision);
fclose(fid);

%% limelight fmap


tagSizeMm = 6.5 * 25.4;
for k=1:numel(tag)
    T = eye(4);
    T(4,1:3) = tag(k).translationMeters;
    
    theta = tag(k).rotationYPRRadians(3);
    Rz = [cos(theta), -sin(theta), 0, 0;
          sin(theta),  cos(theta), 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
    
    X = Rz * T;

    
    fiducials(k).family = "apriltag3_36h11_classic";
    fiducials(k).id = k;
    fiducials(k).size = tagSizeMm;
    fiducials(k).transform = X(:);
    fiducials(k).unique = 1;
end

q.fiducials = fiducials;
limelight = jsonencode(q, 'PrettyPrint', true);
fid = fopen('sims-basement.fmap', 'wt');
fprintf(fid, '%s', limelight);
fclose(fid);