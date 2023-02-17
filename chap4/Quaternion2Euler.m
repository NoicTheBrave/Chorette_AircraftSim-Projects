function [phi, theta, psi] = Quaternion2Euler(e0, e1, e2, e3)
%Quaternion2Eular returns Eular angles [phi, theta, psi] given the 4
% quaternion vector (e0, e1, e2, e3)
    % roll (x-axis rotation)
    sinr_cosp = 2 * (e0 * e1 + e2 * e3);
    cosr_cosp = 1 - 2 * (e1 * e1 + e2 * e2);
    phi = atan2(sinr_cosp, cosr_cosp);

    % pitch (y-axis rotation)
    sinp = sqrt(1 + 2 * (e0 * e2 - e1 * e3));
    cosp = sqrt(1 - 2 * (e0 * e2 - e1 * e3));
    theta = 2 * atan2(sinp, cosp) - pi / 2;

    % yaw (z-axis rotation)
    siny_cosp = 2 * (e0 * e3 + e1 * e2);
    cosy_cosp = 1 - 2 * (e2 * e2 + e3 * e3);
    psi = atan2(siny_cosp, cosy_cosp);
end