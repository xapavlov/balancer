%Initialisation
[data, controller] = initialise();

%build approximate explicit controller
controller=approximate(data, controller);

%plot the stability tree
figure 
controller.stab_tree.plot;
