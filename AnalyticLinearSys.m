clear;
cmap = colormap(hsv); close;
cmap1 = colormap(spring); close;
cmap2 = colormap(summer); close;
cmap3 = colormap(winter); close;
N_states = 4;
N_inputs = 2;

%% Fixed u. Varying Z
nominal_u = 2;
N = 100;
Zk_start = 0.5; Zk_final = 5;
Zk = linspace(Zk_start,Zk_final,N);

fhandle = figure;
color_idx = 1;
for i = 1:N
    nominal_x0 = [0; Zk(i); nominal_u; 0]; % matters to X
    nominal_data_setting = [Zk(i); nominal_u]; % Set Z, u.
    [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
    r = eig(A(2:end,2:end));
    
    if (color_idx > size(cmap,1))
        color_idx = 1;
    end
    plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'Color',cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
    color_idx = color_idx+1;
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying Zk: ', num2str(Zk_start), ' to ', num2str(Zk_final)],['u = ', num2str(nominal_u)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);

%% Fixed u. Varying Z. (see the change with three different u)
nominal_u = [2 5 7];
cmaps = {cmap1, cmap2, cmap3};
N = 100;
Zk_start = 0.5; Zk_final = 5;
Zk = linspace(Zk_start,Zk_final,N);

fhandle = figure;
for j = 1:3
    current_cmap = cmaps{j};
    color_idx = 1;
    for i = 1:N
        nominal_x0 = [0; Zk(i); nominal_u(j); 0]; % matters to X
        nominal_data_setting = [Zk(i); nominal_u(j)]; % Set Z, u.
        [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
        r = eig(A(2:end,2:end));
        
        if (color_idx > size(current_cmap,1))
            color_idx = 1;
        end
        plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'Color',current_cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
        color_idx = color_idx+1;
    end
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying Z: ', num2str(Zk_start), ' to ', num2str(Zk_final)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);

%% Fixed Z. Varying u
nominal_Z = 0.5;
N = 100;
uk_start = 1; uk_final = 10;
uk = linspace(uk_start,uk_final,N);

fhandle = figure;
color_idx = 1;
for i = 1:N
    nominal_x0 = [0; nominal_Z; uk(i); 0]; % matters to X
    nominal_data_setting = [nominal_Z; uk(i)]; % Set Z, u.
    [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
    r = eig(A(2:end,2:end));
    
    if (color_idx > size(cmap,1))
        color_idx = 1;
    end
    plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'Color',cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
    color_idx = color_idx+1;
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying u: ', num2str(uk_start), ' to ', num2str(uk_final)],['Z = ', num2str(nominal_Z)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);

%% Fixed Z. Varying u. (see the change with three different Z)
nominal_Z = [0.5 1 1.5];
cmaps = {cmap1, cmap2, cmap3};
N = 100;
uk_start = 1; uk_final = 10;
uk = linspace(uk_start,uk_final,N);

fhandle = figure;
for j = 1:3
    current_cmap = cmaps{j};
    color_idx = 1;
    for i = 1:N
        nominal_x0 = [0; nominal_Z(j); uk(i); 0]; % matters to X
        nominal_data_setting = [nominal_Z(j); uk(i)]; % Set Z, u.
        [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
        r = eig(A(2:end,2:end));
        
        if (color_idx > size(current_cmap,1))
            color_idx = 1;
        end
        plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'Color',current_cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
        color_idx = color_idx+1;
    end
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying u: ', num2str(uk_start), ' to ', num2str(uk_final)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);

%% System controllability and initial consition analysis
nominal_Z = 1.909; nominal_u = 2;
nominal_x0 = [0; nominal_Z; nominal_u; 0]; % matters to X
nominal_data_setting = [nominal_Z; nominal_u]; % Set Z, u.

[A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
[T,D] = eig(A(2:end,2:end));
[T_tilde,D_tilde] = cdf2rdf(T,D);
B_z = eye(N_states-1)/T_tilde*B(2:end,:); % check controllability

%%%% see the result of T (T_tilde) and D (D_tilde) to analyze the trend of x(t)


