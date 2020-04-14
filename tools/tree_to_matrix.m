%these are the data structures where all required information is saved
location_matrix=[]; %this spans point location problem
control_matrix=zeros(1,16); %keeps only unique 8-bit discretisations of abs(control imput)
cost_matrix=zeros(1,16); %keeps only unique 8-bit discretisations of cost


cont_idx=0;
cost_idx=0;
iterator = stab_tree.breadthfirstiterator;
    
for i =1:numel(iterator)
    axis=axis_tree.get(iterator(i));

    if (axis>0)
        children=stab_tree.getchildren(iterator(i));
        if (numel(children)~=2) 
            error('only 1 children'); 
        end
        children_idx=find(iterator==children(1),1);
        if (iterator(children_idx+1)~=children(2)) 
            error('wrong children order'); 
        end
        location_matrix(i)=bitor(bitshift(axis, 28), children_idx);
    else
        control=round(255*(control_tree.get(iterator(i))/70),0);
        stability = stab_tree.get(iterator(i));
        
        controlsign=0;
        if all(control<0) 
           controlsign=1;
           control=-control;
        end
        
        if stability==1
           control=0*ones(1,16); %use LQR, no need to store controls
           cost=0*ones(1,16);
        elseif stability==2
           cost=0*ones(1,16);  %worstcase stable, no need to store costs
        elseif stability==3
           cost=cost_tree.get(iterator(i))';
           cost=cost-min(cost);
           cost=round(255*(cost./max(cost)),0);
        else  %saturated inputs
           control=255*ones(1,16); 
           cost=0*ones(1,16);
        end

        
        I=ismember(cost_matrix,cost,'rows');
        Iflip=ismember(cost_matrix,fliplr(cost),'rows');
        
        flip=0;
        if any(I) %if such vector exists we point to it
          cost_idx=find(I,1); 
        elseif any(Iflip) %if reversed vector exists we point to it
          flip=1;
          cost = fliplr(cost);
          cost_idx=find(Iflip,1); 
        else %otherwise we store this new vector
          cost_idx=numel(I)+1;
        end

        if (flip==0)
          I=ismember(control_matrix,control,'rows');
          if any(I)
            cont_idx=find(I,1); 
          else
            cont_idx=numel(I)+1;
          end
        else
          control=fliplr(control);
          I=ismember(control_matrix,control,'rows');
          if any(I)
            cont_idx=find(I,1); 
          else
            cont_idx=numel(I)+1;
          end  
        end
        
        if cont_idx>=8192 || cost_idx>=8192
            warning('size exceed 13bits limit');
        end

        cost_matrix(cost_idx,:)=cost; %save cost vector
        control_matrix(cont_idx,:)=control; %save control vector
        
        %format is as follows: 
        %the first 14 bits 
        %   1 bit (flip flag) and
        %   13 bits of cost_idx address 
        %last 14 bits are 
        %   1 bit (sign flag) and
        %   13 bits of cont_idx address 
        location_matrix(i)=bitor(...
        bitshift(bitor(bitshift(flip, 13),cost_idx), 14),... 
                 bitor(bitshift(controlsign, 13),cont_idx));
    end
end