function [y] = low_pass(x, dt, RC)
  y = zeros(size(x));
  alpha = dt / (RC + dt);
  y(1) = alpha * x(1);
  for i = 2:length(x)
    y(i) =  y(i-1) + alpha * (x(i) - y(i-1));
  end
end
