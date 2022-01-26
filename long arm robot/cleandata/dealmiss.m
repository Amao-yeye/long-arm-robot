function [ok]=dealmiss(G)
ok=G;
for i=1:length(G(:,1))
    if ok(i,1)<-100000000
        
        ok(i,1)=ok(i-1,1);
        ok(i,2)=ok(i-1,2);
        ok(i,3)=ok(i-1,3);
        ok(i,4)=ok(i-1,4);
        ok(i,5)=ok(i-1,5);
        ok(i,6)=ok(i-1,6);
        ok(i,7)=ok(i-1,7);
    end
end
