function e = Euler2Quaternion(phi, theta, psi)
%Eular2Quaternion returns the 4 element quaternion vector (e0, e1, e2, e3)
% given the Eular angles (phi, theta, psi)
    cr = cos(phi * 0.5);
    sr = sin(phi * 0.5);
    cp = cos(theta * 0.5);
    sp = sin(theta * 0.5);
    cy = cos(psi * 0.5);
    sy = sin(psi * 0.5);

    e = [cr * cp * cy + sr * sp * sy;
        sr * cp * cy - cr * sp * sy;
        cr * sp * cy + sr * cp * sy;
        cr * cp * sy - sr * sp * cy;]; 
end