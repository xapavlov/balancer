function [children, axis] = partition(box, force_axis, dJdx)
    if force_axis
      axis=force_axis;
      mn = mean([box.lb box.ub],2);      
      children{1}.lb=box.lb;
      children{1}.ub=box.ub;
      children{1}.ub(axis)=mn(axis);

      children{2}.lb=box.lb;
      children{2}.ub=box.ub;
      children{2}.lb(axis)=mn(axis);
    else
      % Compute vertices of the hyppercube
      dim=length(box.lb);
      V=box.lb*ones(1, 2^dim);
      combinations=(dec2bin(0:(2^dim-1)) - '0')';
      edges=(box.ub-box.lb)*ones(1, 2^dim);
      V=V+combinations.*edges;
      
      mn = mean([box.lb box.ub],2); 
      mask_gt=bsxfun(@gt,V,mn);
      mask_lt=bsxfun(@lt,V,mn);
      H=mask_gt.*dJdx-mask_lt.*dJdx;
      H=sum(H,2).*(box.ub-box.lb);
      [~,axis]=max(H);

      children{1}.lb=box.lb;
      children{1}.ub=box.ub;
      children{1}.ub(axis)=mn(axis);

      children{2}.lb=box.lb;
      children{2}.ub=box.ub;
      children{2}.lb(axis)=mn(axis);
    end 
end

