function QMR = Q_mult(Q0, Q1)
w0=Q0(1); x0=Q0(2); y0=Q0(3); z0=Q0(4); 
w1=Q1(1); x1=Q1(2); y1=Q1(3); z1=Q1(4); 
wr=(w0.*w1 - x0.*x1 - y0.*y1 - z0.*z1);
xr=(w0.*x1 + x0.*w1 + y0.*z1 - z0.*y1);
yr=(w0.*y1 - x0.*z1 + y0.*w1 + z0.*x1);
zr=(w0.*z1 + x0.*y1 - y0.*x1 + z0.*w1);
QMR=[wr xr yr zr]
end