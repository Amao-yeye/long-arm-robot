function [SortG1]=sortdata(G1,start)
%G1=xlsread('C:\Users\YING\Desktop\数据整理\real_000.xls');
cnt=1;
flag=1;
j=1;
SortG1=[];
format long;
for i=start:length(G1(:,1))
    if flag==1
        SortG1(j,1)=G1(i,1);
        SortG1(j,2)=G1(i,2);
        SortG1(j,3)=G1(i,3);
        SortG1(j,4)=G1(i,4);
        SortG1(j,5)=G1(i,5);
        SortG1(j,6)=G1(i,6);
        SortG1(j,7)=G1(i,7);
        j=j+1;
        flag=0;
        
    end
    cnt=cnt+1;
    if cnt==11
        flag=1;
        cnt=1;
    end
end
end

