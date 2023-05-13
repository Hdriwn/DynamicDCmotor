function [c, ceq] = constraints(x)
    % Inequality constraints
    c = [];
    if x(1) <= 0
        c = [c, -x(1)]; % Constraint for Km to be non-negative
    end
    if x(2) <= 0
        c = [c, -x(2)]; % Constraint for J to be non-negative
    end
    if x(3) <= 0
        c = [c, -x(3)]; % Constraint for B to be non-negative
    end
    if x(4) <= 0
        c = [c, -x(4)]; % Constraint for L to be non-negative
    end
    if x(5) <= 0
        c = [c, -x(5)]; % Constraint for r to be non-negative
    end
    if x(6) <= 0
        c = [c, -x(6)]; % Constraint for kB to be non-negative
    end
    % Equality constraints
    ceq = [];
end
