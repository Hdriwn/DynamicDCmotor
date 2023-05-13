function f = objfun(x, t_obj, exp_pos, exp_voltage,lambda)
    s = tf('s');
    P_motor = @(x) x(1)/((x(2)*s + x(3))*(x(4)*s + x(5)) + x(1)*x(6));
    y = lsim(P_motor(x), exp_voltage', t_obj);
    pos = y*(180/(pi)); % rad to degrees
    f = sum((exp_pos - pos).^2) + lambda*sum(x.^2);
end