classdef environment
   properties
      intercept
      blocks 
      targets_centers
      targets_dimensions
      arena_dimensions
      obstacle
      
   end
   methods 
      % create an obstacle?
      %argument check; throw error if the arguments are wrong.
      function targets=setup_targets_coord(obj)

          for i=1:size(obj.targets_dimensions,1)
              px=[obj.targets_centers(i,1)-(obj.targets_dimensions(i,1)/2),...
                  obj.targets_centers(i,1)+(obj.targets_dimensions(i,1)/2),...
                  obj.targets_centers(i,1)+(obj.targets_dimensions(i,1)/2),...
                  obj.targets_centers(i,1)-(obj.targets_dimensions(i,1)/2)];
    
               py=[obj.targets_centers(i,2)-(obj.targets_dimensions(i,2)/2),...
                   obj.targets_centers(i,2)-(obj.targets_dimensions(i,2)/2),...
                   obj.targets_centers(i,2)+(obj.targets_dimensions(i,2)/2),...
                   obj.targets_centers(i,2)+(obj.targets_dimensions(i,2)/2)];
    
               targets(i).x=px; 
               targets(i).y=py;
          end

          

      end

     
      function obstacle_=setup_obstacle_coord(obj)
          for k=1:size(obj.obstacle,1)
              %*cos(obj.obstacle(k,5)) *sin( obj.obstacle(k,5)+(pi/2) )

              px_o=[obj.obstacle(k,1)-( (obj.obstacle(k,3)/2) ),...
                    obj.obstacle(k,1)+( (obj.obstacle(k,3)/2) ),...
                    obj.obstacle(k,1)+( (obj.obstacle(k,3)/2) ),...
                    obj.obstacle(k,1)-( (obj.obstacle(k,3)/2) )];


              py_o=[obj.obstacle(k,2)-( (obj.obstacle(k,4)/2) ),...
                    obj.obstacle(k,2)-( (obj.obstacle(k,4)/2) ),...
                    obj.obstacle(k,2)+( (obj.obstacle(k,4)/2) ),...
                    obj.obstacle(k,2)+( (obj.obstacle(k,4)/2) )];




              px_o_rotated=[ (px_o.*cos(obj.obstacle(k,5))) - (py_o.*sin(obj.obstacle(k,5))) ];
              py_o_rotated=[ (px_o.*sin(obj.obstacle(k,5))) + (py_o.*cos(obj.obstacle(k,5))) ];



              obstacle_(k).x=px_o_rotated;
              obstacle_(k).y=py_o_rotated;


          end

      end



   end
end