%clear;clc
%数据处理
%M =sread('C:\Users\YING\Desktop\data\pure290.xls');
M=L;
Tx=M(:,1);Ty=M(:,2);Tz=M(:,3);
cnt=1;%用来记录点点
valuex=[];
valuey=[];
valuez=[];
sum=Tx(1);
sumy=Ty(1);
sumz=Tz(1);
samenum=1;
for i=2:1059
    
    cha=abs(Tx(i-1)-Tx(i))
    chay=abs(Ty(i-1)-Ty(i))
    chaz=abs(Tz(i-1)-Tz(i))
        if cha<0.1 
            sum=sum+Tx(i)
            sumy=sumy+Ty(i);
            sumz=sumz+Tz(i);
            samenum=samenum+1
            b='same point'
        end
        if cha>0.1 
        if samenum>=3
            valuex(cnt)=sum/samenum
            valuey(cnt)=sumy/samenum
            valuez(cnt)=sumz/samenum
            cnt=cnt+1
            samenum=1
            sum=Tx(i)
            sumy=Ty(i)
            sumz=Tz(i)
            b='new point'
        else
            sum=Tx(i);
            sumy=Ty(i);
            sumz=Tz(i);
        end
        
        end
        
end

 result(:,1)=valuex';
 result(:,2)=valuey';
 result(:,3)=valuez';