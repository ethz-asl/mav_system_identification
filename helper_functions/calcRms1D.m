function [ ret,stdv ] = calcRms1D( ref_t,ref_data,esti_t,esti_data )
    %Calculate sum of root-mean-squared
    esti_intp = interp1(esti_t,esti_data,ref_t,'spline');
    sz=size(esti_intp,2);
    errSumSq=0;
    err=zeros(sz,1);
    for i=1:sz
            errSumSq =errSumSq +(ref_data(i)-esti_intp(i))^2;
        	err(i)=ref_data(i)-esti_intp(i);
    end
    errSumSq=errSumSq/sz;
    ret=sqrt(errSumSq);
    stdv=std(err);
end

