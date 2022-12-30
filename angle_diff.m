function dphi=angle_diff(phi1,phi2)
% phi1, phi2: (-inf,inf)
% dphi: phi2-phi1, [-pi,pi)
dphi=mod(phi2-phi1+pi,2*pi)-pi;