function [convM]=convert(M)
len=length(M(:,1));
convM=zeros(len,3);
for i=1:len
    convM(i,3)=-[M(i,1)+61.8];
    convM(i,2)=M(i,2)-40;
    convM(i,1)=M(i,3)+1974;
end
