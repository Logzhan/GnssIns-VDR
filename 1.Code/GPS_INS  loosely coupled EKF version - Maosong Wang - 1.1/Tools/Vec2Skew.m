function [ M ] = Vec2Skew( vec3)
   M=[      0,   -vec3(3),      vec3(2);
       vec3(3),         0,     -vec3(1);
      -vec3(2),    vec3(1),      0];
end

