function T = FK_hw3(q,d1,a2,d3)
T = Rz(q(1))*Tz(d1)*Rx(q(2))*Ty(a2)*Ty(d3);
end

