function controller = approximate(data, controller)
%Performs stability checks and partition
    %do not partition hypercubes if depth exceeded
    max_tree_depth=30;
    %do not partition partially infeasible hypercubes if depth exceeded
    max_infeas_depth=16;
    
    %make a backup
    part_tree=controller.part_tree;
    stab_tree=controller.stab_tree;
    ctrl_tree=controller.ctrl_tree;
    cost_tree=controller.cost_tree;
    axis_tree=controller.axis_tree;
    
    %create/update depth tree 
    need_update=1;
    
    for depth=0:max_tree_depth-1
        if need_update %(not needed until trees get modified)
            dt = stab_tree.depthtree;
            need_update=0;
        end
        
        indices = find ( dt == depth );
        disp("Processing depth: " + num2str(depth));
        for i = indices
            if and(stab_tree.get(i)==0, isempty(axis_tree.getchildren(i)))
                need_update=1;
                box=part_tree.get(i);
                [stability, control, J, dJdx]=certify(data, box);
                
                if stability==0
                    [children, axis]= partition(box, 0, dJdx);
                elseif stability==-1
                    if depth < max_infeas_depth
                        axis=axis_tree.get(axis_tree.getparent(i))+1;
                        if axis>data.dim 
                            axis=1;
                        end
                        [children, ~]= partition(box, axis, []);
                    else
                        axis=[];
                        children={};
                    end
                else
                    axis=[];
                    children={};
                end
                
                stab_tree=stab_tree.set(i, stability);
                ctrl_tree=ctrl_tree.set(i, control);
                cost_tree=cost_tree.set(i, J);
                axis_tree=axis_tree.set(i, axis);
                
                for j=1:numel(children)
                   part_tree=part_tree.addnode(i, children{j});
                   stab_tree=stab_tree.addnode(i, 0);
                   ctrl_tree=ctrl_tree.addnode(i,[]);
                   cost_tree=ctrl_tree.addnode(i,[]);
                   axis_tree=axis_tree.addnode(i,[]);
                end
            end
        end
    end
    %return tree
    controller.part_tree=part_tree;
    controller.stab_tree=stab_tree;
    controller.ctrl_tree=ctrl_tree;
    controller.cost_tree=cost_tree;
    controller.axis_tree=axis_tree;
end

