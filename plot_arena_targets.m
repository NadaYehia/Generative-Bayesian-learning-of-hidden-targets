c1=[0.12156862745098039 0.4666666666666667 0.7058823529411765];
c2=[1 0.4980392156862745 0.054901960784313725];
c3=[0.17254901960784313 0.6274509803921569 0.17254901960784313];
c4=[0.5019607843137255 0 0.5019607843137255];
c5=[0.6627450980392157 0.6627450980392157 0.6627450980392157];
colors=[c1;c2;c3;c4;c5];
f=figure(3); 
hold on;
patch(arena.x,arena.y,[1 1 1]); hold on;
for i=1:5

    xy_w_h=FiveLocData.mouse(3).loc(i).day.target_coords;
    x_shift= FiveLocData.mouse(3).loc(i).day.x_shift;
    y_shift= FiveLocData.mouse(3).loc(i).day.y_shift;
    corners={};
    corners.x=[xy_w_h(1)-(xy_w_h(3)/2),xy_w_h(1)+(xy_w_h(3)/2),xy_w_h(1)+(xy_w_h(3)/2),xy_w_h(1)-(xy_w_h(3)/2) ];
    corners.y= [xy_w_h(2)-(xy_w_h(4)/2), xy_w_h(2)-(xy_w_h(4)/2) ,xy_w_h(2)+(xy_w_h(4)/2) ,xy_w_h(2)+(xy_w_h(4)/2)];
    
    patch(corners.x-x_shift,corners.y-y_shift,colors(i,:));
    hold on;
end