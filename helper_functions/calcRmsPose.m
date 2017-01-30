function [ ret,stdv ] = calcRmsPose( ref,odom)
    %Calculate sum of root-mean-squared euclidient distance in R^3.

    odom.p_intp(1,:) = interp1(odom.t,odom.p(1,:),ref.t,'spline');
    odom.p_intp(2,:) = interp1(odom.t,odom.p(2,:),ref.t,'spline');
    odom.p_intp(3,:) = interp1(odom.t,odom.p(3,:),ref.t,'spline');
    sz=round(size(odom.p_intp,2));
    %Interpolation to match the size of ref and odom
    errDistSumSq=0;
    err=zeros(sz,1);
    for i=1:sz
            errDist=sqrt((ref.p(1,i)-odom.p_intp(1,i))^2+(ref.p(2,i)-odom.p_intp(2,i))^2+...
                        (ref.p(3,i)-odom.p_intp(3,i))^2);
            err(i)=errDist;
            errDistSumSq =errDistSumSq +errDist^2;
    end
    errDistSumSq=errDistSumSq/sz;
    ret=sqrt(errDistSumSq);
    stdv=std(err);
end

