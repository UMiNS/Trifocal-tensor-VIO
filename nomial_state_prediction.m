function x = nomial_state_prediction( x_init, IMUData )
% nomial state prediction
global Tic idt;

Ric = Tic(1:3,1:3);
pic = Tic(1:3,4);
omxDt = IMUData(1:3);
axDt = IMUData(4:6);

% x' = f(x) is the differential equation
% Use 4-order Runge-Kutta method to integrate, the solution is:
% x(n+1) = x(n)+idt/6*(k1+2*k2+2*k3+k4)
% k1 = f(x); 
% k2 = f(x+idt/2*k1)
% k3 = f(x+idt/2*k2)
% k4 = f(x+idt*k3)
% reference: http://en.wikipedia.org/wiki/Runge-Kutta_methods

b = [ 1/6, 1/3, 1/3, 1/6 ]; 
% b = [ 1, 0, 0, 0 ];       % euler integration
d = [ 0, 0.5, 0.5, 1 ];

k = zeros( size( x_init, 1 ), 4 );
k(:,1) = idt*nomial_state_model( x_init, omxDt, axDt );
for i = 2:4
    k(:,i) = idt*nomial_state_model( x_init+d(i)*k(:,i-1), omxDt, axDt );
end
x = x_init + (b*k')';

end

