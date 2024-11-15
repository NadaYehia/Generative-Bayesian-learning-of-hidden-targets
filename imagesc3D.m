function imagesc3D(im,Os,Rs)

    h=imagesc(Rs,Os,makeimagestack(squeeze(im),[],[],[],[])); 
    set(gca,'YDir','normal')
    yticks([-pi/2 -pi/3 -pi/4 -pi/8 0 pi/8 pi/4 pi/3 pi/2])
    yticklabels({'-\pi/2','-\pi/3','-\pi/4','-\pi/8','0','\pi/8','\pi/4','\pi/3','\pi/2'})
    set(gca,'TickDir','out')
     fontsize(gca,16,'points');
     fontname(gca,'Times New Roman');
     xlabel('R');
     ylabel('θ');
     zlabel('trials');
     set(h, 'AlphaData', ~isnan(im));


drawnow;



