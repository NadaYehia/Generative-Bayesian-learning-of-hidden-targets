classdef environment
   properties
      intercept
      blocks 
      targets_centers
      targets_dimensions
      arena_dimensions
   end
   methods 
      % create an obstacle?
      %argument check; throw error if the arguments are wrong.
      function targets=setup_targets_coord(obj)

          for i=1:size(obj.targets_dimensions,1)
              px=[obj.targets_centers(i,1)-obj.targets_dimensions(i,1),...
                  obj.targets_centers(i,1)+obj.targets_dimensions(i,1),...
                  obj.targets_centers(i,1)+obj.targets_dimensions(i,1),...
                  obj.targets_centers(i,1)-obj.targets_dimensions(i,1)];
    
               py=[obj.targets_centers(i,2)-obj.targets_dimensions(i,2),...
                   obj.targets_centers(i,2)-obj.targets_dimensions(i,2),...
                   obj.targets_centers(i,2)+obj.targets_dimensions(i,2),...
                   obj.targets_centers(i,2)+obj.targets_dimensions(i,2)];
    
               targets(i).x=px; 
               targets(i).y=py;
          end


      end
   end
end