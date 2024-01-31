function entro=my_entropy(pdf)

pdf=reshape(pdf,size(pdf,1)*size(pdf,2),1);
pdf(pdf==0)=[];

entro= -sum(...
    (pdf(:).*log2(pdf(:))) );


end