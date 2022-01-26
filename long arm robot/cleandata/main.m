G1=xlsread('C:\Users\YING\Desktop\数据整理\real_000.xls');
start1=24;
s1=sortdata(G1,start1);
ok1=dealmiss(s1);
okk1=convert(ok1(:,5:7));

G2=xlsread('C:\Users\YING\Desktop\数据整理\real_001.xls');
start2=20;
s2=sortdata(G2,start2);
ok2=dealmiss(s2);
okk2=convert(ok2(:,5:7));

G3=xlsread('C:\Users\YING\Desktop\数据整理\real_002.xls');
start3=20;
s3=sortdata(G3,start3);
ok3=dealmiss(s3);
okk3=convert(ok3(:,5:7));

G4=xlsread('C:\Users\YING\Desktop\数据整理\real_003.xls');
start4=20;
s4=sortdata(G4,start4);
ok4=dealmiss(s4);
okk4=convert(ok4(:,5:7));

G5=xlsread('C:\Users\YING\Desktop\数据整理\real_004.xls');
start5=23;
s5=sortdata(G5,start5);
ok5=dealmiss(s5);
okk5=convert(ok5(:,5:7));

G6=xlsread('C:\Users\YING\Desktop\数据整理\real_005.xls');
start6=24;
s6=sortdata(G6,start6);
ok6=dealmiss(s6);
okk6=convert(ok6(:,5:7));

G7=xlsread('C:\Users\YING\Desktop\数据整理\real_006.xls');
start7=16;
s7=sortdata(G7,start7);
ok7=dealmiss(s7);
okk7=convert(ok7(:,5:7));