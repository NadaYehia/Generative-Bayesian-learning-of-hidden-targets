
function curvature = calculate_curvature(x, y)
  % Calculate the curvature of a 2D curve defined by points (x, y)
  
  % Calculate first derivative (tangent vector)
  dx = diff(x);
  dy = diff(y);
  
  % Normalize tangent vector
  tangent_mag = sqrt(dx.^2 + dy.^2);
  dx = dx ./ tangent_mag;
  dy = dy ./ tangent_mag;
  
  % Calculate second derivative (curvature vector)
  d2x = diff(dx);
  d2y = diff(dy);
  
  % Curvature is the magnitude of the second derivative
  curvature = sqrt(d2x.^2 + d2y.^2);
end