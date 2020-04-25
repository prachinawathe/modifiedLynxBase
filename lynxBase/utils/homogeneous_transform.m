function A = homogeneous_transform(a, d, alpha, theta)

    cos_rad = @(angle) cosd(angle/pi*180);
    sin_rad = @(angle) sind(angle/pi*180);

    A = [cos_rad(theta) -1*sin_rad(theta)*cos_rad(alpha) sin_rad(theta)*sin_rad(alpha) a*cos_rad(theta);
        sin_rad(theta) cos_rad(theta)*cos_rad(alpha) -1*cos_rad(theta)*sin_rad(alpha) a*sin_rad(theta);
        0 sin_rad(alpha) cos_rad(alpha) d;
        0 0 0 1
        ];
end


