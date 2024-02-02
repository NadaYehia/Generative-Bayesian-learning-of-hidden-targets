function dist=pixel_dist_to_normal_eucl_dist(k,sigma_ridge,res_x,res_y,range_x,range_y)


increment_x= res_x*k*sigma_ridge;
% increment_y= res_y*k*sigma_ridge;

dist= increment_x/range_x;






end