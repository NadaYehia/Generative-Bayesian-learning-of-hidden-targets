function kappa=Mycurvature(x_op,y_op)

    for t=2:numel(x_op)-1
        x1=x_op(t-1);
        x2=x_op(t);
        x3=x_op(t+1);
         
        y1=y_op(t-1);
        y2=y_op(t);
        y3=y_op(t+1);
    
        a=norm([x1,y1]-[x2 y2]);
        b=norm([x2 y2]-[x3 y3]);
        c=norm([x3 y3]-[x1 y1]);
    
        s=(a+b+c)/2;
        A=sqrt(s*(s-a)*(s-b)*(s-c));
        K(t)=(4*A)/(a*b*c);
    
    end
kappa=sum(K);
end
