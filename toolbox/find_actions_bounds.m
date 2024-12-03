% workout the arena boundaries in x,y to mu and omega actions

function [r_boundary,omega_boundary]=find_actions_bounds(arena,spc)

corner=[];
corner(1,:)=[arena(1) arena(3)];
corner(2,:)=[arena(2) arena(3)];

corner(4,:)=[arena(1) arena(4)];
corner(3,:)=[arena(2) arena(4)];


x_op=[];
y_op=[];

for cor=1:3

    delta=abs(corner(cor,1)-corner(cor+1,1));
    if (delta~=0)

        xq=linspace(corner(cor,1),corner(cor+1,1),spc);
        vq=interp1( [corner(cor,1) corner(cor+1,1)],[corner(cor,2) corner(cor+1,2)],xq);
    else
        delta=abs(corner(cor,2)-corner(cor+1,2));
        xq=repmat(corner(cor,1),1,spc);
        vq=linspace(corner(cor,2),corner(cor+1,2),spc);
    end

   
    x_op=[x_op,xq];
    y_op=[y_op,vq];

end

% last corner to the first one
delta=abs(corner(4,2)-corner(1,2));
xq=repmat(corner(4,1),1,spc);
vq=linspace(corner(4,2),corner(1,2),spc);
x_op=[x_op,xq];
y_op=[y_op,vq];


[r_boundary,omega_boundary]= convert_xy_r_angle(x_op,y_op);

end