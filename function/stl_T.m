function fv = stl_T(T,fv_i)
fv = fv_i;
R = T(1:3,1:3);
tx = T(1,4);
ty = T(2,4);
tz = T(3,4);

fv.vertices(:,1:3) = (R*fv_i.vertices(:,1:3)')'+repmat([tx,ty,tz],length(fv_i.vertices(:,1)),1);
end