% Clean up workspace
clear all;
close all;

% Number of trials
num_trials = 50;
num_steps = 50;
if_plot = 0;

% Simulation variables
SF = 5;

for parameter = 1:3

X = [];

step_noise_mpgs = [];
step_noise_mpgs_lower = [];
step_noise_mpgs_upper = [];

step_noise_collisions = [];
step_noise_collisions_lower = [];
step_noise_collisions_upper = [];

sigma_a = 0.00;
sigma_c = 0.00;
sigma_m = 0.00;

    for step = 1:num_steps
        car_mpgs = [];
        car_collisions = [];
        mean_mpg = [];
        mean_collision = [];
    
        for idx=1:num_trials
            car_results = centralized_platoon(sigma_a, sigma_c, sigma_m, SF, if_plot);
            car_mpgs = vertcat(car_mpgs, car_results(1,:));
            mean_mpg = vertcat(mean_mpg, mean(car_results(1,:)));
            car_collisions = vertcat(car_collisions, car_results(2,:));
            mean_collision = vertcat(mean_collision, mean(car_results(2,:)));
        end
    
        X = [X; step*0.01];
    
        step_noise_mpgs = vertcat(step_noise_mpgs,mean(mean_mpg));
        pd_mean_mpg = fitdist(mean_mpg,'Normal');
        ci_mean_mpg = paramci(pd_mean_mpg);
        step_noise_mpgs_lower = vertcat(step_noise_mpgs_lower,ci_mean_mpg(1,2));
        step_noise_mpgs_upper = vertcat(step_noise_mpgs_upper,ci_mean_mpg(2,2));
    
    
        step_noise_collisions = vertcat(step_noise_collisions,mean(mean_collision));
        pd_mean_collision = fitdist(mean_collision,'Normal');
        ci_mean_collision = paramci(pd_mean_collision);
        step_noise_collisions_lower = vertcat(step_noise_collisions_lower,ci_mean_collision(1,2));
        step_noise_collisions_upper = vertcat(step_noise_collisions_upper,ci_mean_collision(2,2));
    
    
        if parameter == 1
            sigma_a = sigma_a + 0.01;
        elseif parameter == 2
            sigma_c = sigma_c + 0.01;
        elseif parameter == 3
            sigma_m = sigma_m + 0.01;
        end
    end

    if parameter == 1
        file_name=sprintf('cen_sigma_a');
        x_label = 'Actuator Noises Sigma';
    elseif parameter == 2
        file_name=sprintf('cen_sigma_c');
        x_label = 'Control Noises Sigma';
    elseif parameter == 3
        file_name=sprintf('cen_sigma_m');
        x_label = 'Sensor Noises Sigma';
    end

f1 = figure;
set(f1,'Visible','off');
subplot(2,1,1);
errorbar(X,step_noise_mpgs,step_noise_mpgs_lower,step_noise_mpgs_upper,'-', 'linewidth',0.2);
%axis([0 700 -1.2 1.2]);
%set(gca, 'FontSize', 14,'linewidth',0.2,'fontname','times');
xlabel(x_label,'FontSize',14);
ylabel('Fuel Economy (MPG)','FontSize',14);
%legend('Car 0','Car 1','Car 2', 'Car 3', 'Car 4', 'Car 5','location','eastoutside');
subplot(2,1,2);
hold on;
errorbar(X,step_noise_collisions,step_noise_collisions_lower,step_noise_collisions_upper,'-', 'linewidth',0.2);
hold off;
xlabel(x_label,'FontSize',14);
ylabel('Number of Collisions','FontSize',14);

f1.PaperPositionMode = 'auto';
fig_pos = f1.PaperPosition;
f1.PaperSize = [fig_pos(3) fig_pos(4)];

path = 'figures/';
%savefig([path, file_name, '.fig']);
saveas(gcf,[path, file_name, '.pdf']);

close(f1);
end
%car_mpgs
% Stats!
% mean(mean(car_mpgs));
% var(mean(car_mpgs,1));