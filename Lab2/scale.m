function [vs] = scale(v)
  if v > 0.3
      vs = 0.3;
  elseif v < -0.3
      vs = -0.3;
  else
      vs = v;
  end
end