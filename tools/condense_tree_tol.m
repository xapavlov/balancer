
max_depth=30;
dt = stab_tree.depthtree;
indices_to_remove=[];
tol=0.1; %tolerance on control std. deviation. If less than tol assume all are equal.
for depth=max_depth:-1:0
    indices = find( dt == depth );
    if isempty(indices) continue; end
    disp("Depth " + num2str(depth));
    for i = indices
        if isnan(stab_tree.get(i)) %if it has splitting
            children=stab_tree.getchildren(i); %get list of children
            if isempty(children)
                error('Error: it was splitted, but have no children');
            end

            stability=cell2mat(stab_tree.Node(children));

            if any(not(stability)) continue; end %can't merge

            if all(stability<=-1) %merging, as all children are infesible
                indices_to_remove=[indices_to_remove, children];
                stab_tree=stab_tree.set(i, -1);
                axis_tree=axis_tree.set(i, 0);
                continue;
            end

            control=cell2mat(control_tree.Node(children));

            if any(stability<=-1) %one child is infesible
                if (std(control)<tol) %merge if another child saturates
                    indices_to_remove=[indices_to_remove, children];
                    stab_tree=stab_tree.set(i, 4);
                    control_tree=control_tree.set(i, control);
                    cost_tree=cost_tree.set(i, 0*control');
                    axis_tree=axis_tree.set(i, 0);
                end
                continue;
            end

            if all(std(control,0,2)<tol)
                 indices_to_remove=[indices_to_remove, children];
                 stab_tree=stab_tree.set(i, 4);
                 control_tree=control_tree.set(i, control(1,:));
                 cost_tree=cost_tree.set(i, 0*control(1,:)');
                 axis_tree=axis_tree.set(i, 0);  
                 continue;
            end

            if isempty(control)
                error('something weird');
            end
        end
    end 
    indices_to_remove=sort(indices_to_remove,'descend');
    for j=indices_to_remove
      stab_tree=stab_tree.removenode(j);
      part_tree=part_tree.removenode(j);
      control_tree=control_tree.removenode(j);
      axis_tree=axis_tree.removenode(j);
      cost_tree=cost_tree.removenode(j);
      dt=dt.removenode(j);
    end
    indices_to_remove=[];
end
