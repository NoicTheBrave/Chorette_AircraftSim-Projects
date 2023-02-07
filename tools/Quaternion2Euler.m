
function [phi, theta, psi] = Quaternion2Euler(e)
    % converts a quaternion attitude to an euler angle attitude
    e0 = e(1);
    e1 = e(2);
    e2 = e(3);
    e3 = e(4);
    phi = atan2( 2*(e0*e1 + e2*e3), (1-2*(e1^2 + e2^2))); %phi = atan2( 2*(e0*e1 + e2*e3), (e0^2+e3^2 - e1^2 - e2^2));
    theta = -pi/2 + 2*atan2(sqrt(1+2*(e0*e2 - e1*e3)), sqrt(1-2*(e9*e2-e1*e3))); %asin(2*(e0*e2 - e1*e3) );
    psi = atan2(2*(e0*e3 + e1*e2), (1-2*(e2^2 + e3^2))); %    psi = atan2(2*(e0*e3 + e1*e2), (e0*2 + e1^2 - e2^2 - e3^2));
end
