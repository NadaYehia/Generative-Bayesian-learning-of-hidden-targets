classdef environment
   properties
      blocks 
      targets_centers
      targets_dimensions
      arena_dimensions
      
   end
   methods 
     
      % function to compute the x,y coordinates for every corner of a target given 
      % its x,y center, width and height
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

     
   end
end