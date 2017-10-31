clc;
clear all;
step = 700;
time = zeros(step,1);

% Delete the folder with last results
if exist('figure','dir')
    delete(strcat(pwd,'/figure/*.fig'));
    rmdir(strcat(pwd,'/figure'));
    mkdir('figure')    
else
    mkdir('figure') 
end

color = {'k','r','b','g','y','m','c', [1,0.5,0], [0.7,0.5,1], [0, 0.5, 0], [0, 0.5, 1], [0.5, 0, 1], [0.5, 0, 0]}; 
%color = ['k','r','b','g','y','m','c', 'rg','by']; 
for it = 3:2:9
    number = it;
    % load result
    load(fullfile(strcat(pwd, '/result'), strcat('output_a_behind_',num2str(number), '.mat')));
    load(fullfile(strcat(pwd, '/result'), strcat('output_s_behind_',num2str(number), '.mat')));
    load(fullfile(strcat(pwd, '/result'), strcat('output_v_behind_',num2str(number), '.mat')));
    load(fullfile(strcat(pwd, '/result'), strcat('output_fuel_',num2str(number), '.mat')));
    load(fullfile(strcat(pwd, '/result'), strcat('output_cost_',num2str(number), '.mat')));
    dist = zeros(step, number-1);
    for j = 1:(number-1)
        for i=1:step
            time(i) = 1*(i-1);
            dist(i,j) = output_s_behind(i,j) - output_s_behind(i,j+1);
        end
    end
    % Figure 1: acceleration
    figure
    subplot(3,2,1)
    %plot(time(1:step-1),output_a_behind(:,1), time(1:step-1),output_a_behind(:,2), time(1:step-1),output_a_behind(:,3), time(1:step-1),output_a_behind(:,4), time(1:step-1),output_a_behind(:,5), time(1:step-1),output_a_behind(:,6), time(1:step-1),output_a_behind(:,7),time(1:step-1),output_a_behind(:,8), time(1:step-1),output_a_behind(:,9));
    plot(time(1:step),output_a_behind(:,1), color{1});     
    hold on   
    ind = 2;
    while ind ~= number+1
        plot(time(1:step),output_a_behind(:,ind), 'Color', color{ind});
        ind = ind+1;
    end
    hold off
    xlabel('time [s]')
    ylabel('acceleration [m/s^2]')
    if number == 3
        legend('car1', 'car2', 'car3');
    elseif number == 5
        legend('car1', 'car2', 'car3', 'car4', 'car5')
    elseif number == 7
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7')
    elseif number == 9
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    elseif number == 11
         legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11')
    else
         legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11', 'car12', 'car13')  
    end
    %legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    title('Acceleration');
    xlim([0,700]);
%     xlabh = get(gca,'XLabel');
%     y_a = get(xlabh,'Position') + [0 0.65 0]
%     set(xlabh,'Position',get(xlabh,'Position') + [0 0.65 0])
 
    % Figure 2: veloity
    subplot(3,2,2)
    %plot(time,output_v_behind(:,1), time,output_v_behind(:,2), time,output_v_behind(:,3), time,output_v_behind(:,4), time,output_v_behind(:,5), time,output_v_behind(:,6),time,output_v_behind(:,7),time,output_v_behind(:,8), time,output_v_behind(:,9));
    plot(time,output_v_behind(:,1), color{1});
    hold on
    ind = 2;
    while ind ~= number+1
        plot(time,output_v_behind(:,ind), 'Color', color{ind});
        ind = ind+1;
    end
    hold off    
    xlabel('time [s]')
    ylabel('velocity [mile/hour]')
    if number == 3
        legend('car1', 'car2', 'car3');
    elseif number == 5
        legend('car1', 'car2', 'car3', 'car4', 'car5')
    elseif number == 7
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7')
    elseif number == 9
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    elseif number == 11
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11')
    else
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11', 'car12', 'car13')
    end
    %legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    title('Velocity');
%     xlabh = get(gca,'XLabel');
%     y_a = get(xlabh,'Position') + [-50 5 0]
%     set(xlabh,'Position',get(xlabh,'Position') + [-50 5 0])

    % Figure 3: positions
    subplot(3,2,3)
    %plot(time,output_s_behind(:,1), time,output_s_behind(:,2), time,output_s_behind(:,3), time,output_s_behind(:,4), time,output_s_behind(:,5), time,output_s_behind(:,6), time,output_s_behind(:,7), time,output_s_behind(:,8), time,output_s_behind(:,9));
    plot(time,output_s_behind(:,1), color{1});
    hold on
    ind = 2;
    while ind ~= number+1
        plot(time,output_s_behind(:,ind), 'Color', color{ind});
        ind = ind+1;
    end
    hold off     
    xlabel('time [s]')
    ylabel('position [mile]')
    if number == 3
        legend('car1', 'car2', 'car3');
    elseif number == 5
        legend('car1', 'car2', 'car3', 'car4', 'car5')
    elseif number == 7
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7')
    elseif number == 9
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    elseif number == 11
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11')
    else
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11', 'car12', 'car13')    
    end
    %legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    title('Position');
%     xlabh = get(gca,'XLabel');
%     last_y = get(xlabh,'Position')+ [-50 1.25 0]
%     set(xlabh,'Position',get(xlabh,'Position') + [-50 1.25 0])

    % Figure 4: distance to the front car
    subplot(3,2,4)
    %plot(time,dist12, time,dist23, time,dist34, time,dist45, time,dist56, time,dist67, time,dist78, time,dist89);
    plot(time,dist(:,1), color{1});
    hold on
    ind = 2;
    while ind ~= number
        plot(time,dist(:,ind), 'Color', color{ind});
        ind = ind+1;
    end
    hold off 
    xlabel('time [s]')
    ylabel('distance to the front car [mile]')
    if number == 3
        legend('between car1 and car2', 'between car2 and car3');
    elseif number == 5
        legend('between car1 and car2', 'between car2 and car3', 'between car3 and car4','between car4 and car5')
    elseif number == 7
        legend('between car1 and car2', 'between car2 and car3', 'between car3 and car4','between car4 and car5', 'between car5 and car6', 'between car6 and car7')
    elseif number == 9
        legend('between car1 and car2', 'between car2 and car3', 'between car3 and car4','between car4 and car5', 'between car5 and car6', 'between car6 and car7', 'between car7 and car8', 'between car8 and car9')
    elseif number == 11
        legend('between car1 and car2', 'between car2 and car3', 'between car3 and car4','between car4 and car5', 'between car5 and car6', 'between car6 and car7', 'between car7 and car8', 'between car8 and car9', 'between car9 and car10', 'between car10 and car11')
    else
        legend('between car1 and car2', 'between car2 and car3', 'between car3 and car4','between car4 and car5', 'between car5 and car6', 'between car6 and car7', 'between car7 and car8', 'between car8 and car9', 'between car9 and car10', 'between car10 and car11', 'between car11 and car12', 'between car12 and car13')
    end
    %legend('between car1 and car2', 'between car2 and car3', 'between car3 and car4','between car4 and car5', 'between car5 and car6', 'between car6 and car7', 'between car7 and car8', 'between car8 and car9')
    title('Distance to the front car');


    % Figure 5: Fuel consumption
    subplot(3,2,5)
    %plot(time(1:step-1),output_fuel(:,1), time(1:step-1),output_fuel(:,2), time(1:step-1),output_fuel(:,3), time(1:step-1),output_fuel(:,4), time(1:step-1),output_fuel(:,5), time(1:step-1),output_fuel(:,6),time(1:step-1),output_fuel(:,7),time(1:step-1),output_fuel(:,8));
    plot(time(1:step-1),output_fuel(:,1), color{1});
    hold on
    ind = 2;
    while ind ~= number+1
        plot(time(1:step-1),output_fuel(:,ind), 'Color', color{ind});
        ind = ind+1;
    end
    hold off 
    xlabel('time [s]')
    %ylabel('Fuel Consumption [gallon/hour]')
    ylabel('Fuel Consumption [mile/gallon]')
    if number == 3
        legend('car1', 'car2', 'car3');
    elseif number == 5
        legend('car1', 'car2', 'car3', 'car4', 'car5')
    elseif number == 7
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7')
    elseif number == 9
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    elseif number == 11
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11')
    else
        legend('car1', 'car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9', 'car10', 'car11', 'car12', 'car13')
    end
    %legend('car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
    title('Fuel Consumption');

    % Figure 6: value of cost functions
%     subplot(3,2,6) 
%     %plot(time(1:step-1),output_cost(:,1), time(1:step-1),output_cost(:,2), time(1:step-1),output_cost(:,3), time(1:step-1),output_cost(:,4), time(1:step-1),output_cost(:,5), time(1:step-1),output_cost(:,6), time(1:step-1),output_cost(:,7), time(1:step-1),output_cost(:,8));
%     plot(time(1:step-1),output_cost(:,1), color{1});
%     hold on
%     ind = 2;
%     while ind ~= number
%         plot(time(1:step-1),output_cost(:,ind), 'Color', color{ind});
%         ind = ind+1;
%     end
%     hold off     
%     xlabel('time [s]')
%     ylabel('Cost function value')
%     if number == 3
%         legend('car2', 'car3');
%     elseif number == 5
%         legend('car2', 'car3', 'car4', 'car5')
%     elseif number == 7
%         legend('car2', 'car3', 'car4', 'car5', 'car6', 'car7')
%     else
%         legend('car2', 'car3', 'car4', 'car5', 'car6', 'car7', 'car8', 'car9')
%     end
%     title('Cost function value');
    suptitle(['Results of the '  num2str(number)  '-car model in the HWFET driving cycle (Decentralized Control)'])

    saveas(gca, fullfile(strcat(pwd,'/figure/result_', num2str(number),'_d.fig')), 'fig')     
    figure
    if number == 3
        subplot(2,1,1)
        plot(time(1:step-1), read_cost(output_cost, 1, 1), color{1},time(1:step-1), read_cost(output_cost, 1, 2), color{2}, time(1:step-1), read_cost(output_cost, 1, 3), color{3},time(1:step-1), read_cost(output_cost, 1, 4), color{4},time(1:step-1), read_cost(output_cost, 1, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car2 cost function value')
        subplot(2,1,2)
        plot(time(1:step-1), read_cost(output_cost, 2, 1), color{1},time(1:step-1), read_cost(output_cost, 2, 2), color{2}, time(1:step-1), read_cost(output_cost, 2, 3), color{3},time(1:step-1), read_cost(output_cost, 2, 4), color{4},time(1:step-1), read_cost(output_cost, 2, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car3 cost function value')        
    elseif number == 5
        subplot(2,2,1)
        plot(time(1:step-1), read_cost(output_cost, 1, 1), color{1},time(1:step-1), read_cost(output_cost, 1, 2), color{2}, time(1:step-1), read_cost(output_cost, 1, 3), color{3},time(1:step-1), read_cost(output_cost, 1, 4), color{4},time(1:step-1), read_cost(output_cost, 1, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car2 cost function value')
        subplot(2,2,2)
        plot(time(1:step-1), read_cost(output_cost, 2, 1), color{1},time(1:step-1), read_cost(output_cost, 2, 2), color{2}, time(1:step-1), read_cost(output_cost, 2, 3), color{3},time(1:step-1), read_cost(output_cost, 2, 4), color{4},time(1:step-1), read_cost(output_cost, 2, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car3 cost function value')    
        subplot(2,2,3)
        plot(time(1:step-1), read_cost(output_cost, 3, 1), color{1},time(1:step-1), read_cost(output_cost, 3, 2), color{2}, time(1:step-1), read_cost(output_cost, 3, 3), color{3},time(1:step-1), read_cost(output_cost, 3, 4), color{4},time(1:step-1), read_cost(output_cost, 3, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car4 cost function value')
        subplot(2,2,4)
        plot(time(1:step-1), read_cost(output_cost, 4, 1), color{1},time(1:step-1), read_cost(output_cost, 4, 2), color{2}, time(1:step-1), read_cost(output_cost, 4, 3), color{3},time(1:step-1), read_cost(output_cost, 4, 4), color{4},time(1:step-1), read_cost(output_cost, 4, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car5 cost function value')    
    elseif number == 7
        subplot(3,2,1)
        plot(time(1:step-1), read_cost(output_cost, 1, 1), color{1},time(1:step-1), read_cost(output_cost, 1, 2), color{2}, time(1:step-1), read_cost(output_cost, 1, 3), color{3},time(1:step-1), read_cost(output_cost, 1, 4), color{4},time(1:step-1), read_cost(output_cost, 1, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car2 cost function value')
        subplot(3,2,2)
        plot(time(1:step-1), read_cost(output_cost, 2, 1), color{1},time(1:step-1), read_cost(output_cost, 2, 2), color{2}, time(1:step-1), read_cost(output_cost, 2, 3), color{3},time(1:step-1), read_cost(output_cost, 2, 4), color{4},time(1:step-1), read_cost(output_cost, 2, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car3 cost function value')    
        subplot(3,2,3)
        plot(time(1:step-1), read_cost(output_cost, 3, 1), color{1},time(1:step-1), read_cost(output_cost, 3, 2), color{2}, time(1:step-1), read_cost(output_cost, 3, 3), color{3},time(1:step-1), read_cost(output_cost, 3, 4), color{4},time(1:step-1), read_cost(output_cost, 3, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car4 cost function value')
        subplot(3,2,4)
        plot(time(1:step-1), read_cost(output_cost, 4, 1), color{1},time(1:step-1), read_cost(output_cost, 4, 2), color{2}, time(1:step-1), read_cost(output_cost, 4, 3), color{3},time(1:step-1), read_cost(output_cost, 4, 4), color{4},time(1:step-1), read_cost(output_cost, 4, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car5 cost function value')   
        subplot(3,2,5)
        plot(time(1:step-1), read_cost(output_cost, 5, 1), color{1},time(1:step-1), read_cost(output_cost, 5, 2), color{2}, time(1:step-1), read_cost(output_cost, 5, 3), color{3},time(1:step-1), read_cost(output_cost, 5, 4), color{4},time(1:step-1), read_cost(output_cost, 5, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car6 cost function value')
        subplot(3,2,6)
        plot(time(1:step-1), read_cost(output_cost, 6, 1), color{1},time(1:step-1), read_cost(output_cost, 6, 2), color{2}, time(1:step-1), read_cost(output_cost, 6, 3), color{3},time(1:step-1), read_cost(output_cost, 6, 4), color{4},time(1:step-1), read_cost(output_cost, 6, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car7 cost function value')   
    elseif number == 9
        subplot(4,2,1)
        plot(time(1:step-1), read_cost(output_cost, 1, 1), color{1},time(1:step-1), read_cost(output_cost, 1, 2), color{2}, time(1:step-1), read_cost(output_cost, 1, 3), color{3},time(1:step-1), read_cost(output_cost, 1, 4), color{4},time(1:step-1), read_cost(output_cost, 1, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car2 cost function value')
        subplot(4,2,2)
        plot(time(1:step-1), read_cost(output_cost, 2, 1), color{1},time(1:step-1), read_cost(output_cost, 2, 2), color{2}, time(1:step-1), read_cost(output_cost, 2, 3), color{3},time(1:step-1), read_cost(output_cost, 2, 4), color{4},time(1:step-1), read_cost(output_cost, 2, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car3 cost function value')    
        subplot(4,2,3)
        plot(time(1:step-1), read_cost(output_cost, 3, 1), color{1},time(1:step-1), read_cost(output_cost, 3, 2), color{2}, time(1:step-1), read_cost(output_cost, 3, 3), color{3},time(1:step-1), read_cost(output_cost, 3, 4), color{4},time(1:step-1), read_cost(output_cost, 3, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car4 cost function value')
        subplot(4,2,4)
        plot(time(1:step-1), read_cost(output_cost, 4, 1), color{1},time(1:step-1), read_cost(output_cost, 4, 2), color{2}, time(1:step-1), read_cost(output_cost, 4, 3), color{3},time(1:step-1), read_cost(output_cost, 4, 4), color{4},time(1:step-1), read_cost(output_cost, 4, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car5 cost function value')   
        subplot(4,2,5)
        plot(time(1:step-1), read_cost(output_cost, 5, 1), color{1},time(1:step-1), read_cost(output_cost, 5, 2), color{2}, time(1:step-1), read_cost(output_cost, 5, 3), color{3},time(1:step-1), read_cost(output_cost, 5, 4), color{4},time(1:step-1), read_cost(output_cost, 5, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car6 cost function value')
        subplot(4,2,6)
        plot(time(1:step-1), read_cost(output_cost, 6, 1), color{1},time(1:step-1), read_cost(output_cost, 6, 2), color{2}, time(1:step-1), read_cost(output_cost, 6, 3), color{3},time(1:step-1), read_cost(output_cost, 6, 4), color{4},time(1:step-1), read_cost(output_cost, 6, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car7 cost function value')  
        subplot(4,2,7)
        plot(time(1:step-1), read_cost(output_cost, 7, 1), color{1},time(1:step-1), read_cost(output_cost, 7, 2), color{2}, time(1:step-1), read_cost(output_cost, 7, 3), color{3},time(1:step-1), read_cost(output_cost, 7, 4), color{4},time(1:step-1), read_cost(output_cost, 7, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car8 cost function value')
        subplot(4,2,8)
        plot(time(1:step-1), read_cost(output_cost, 8, 1), color{1},time(1:step-1), read_cost(output_cost, 8, 2), color{2}, time(1:step-1), read_cost(output_cost, 8, 3), color{3},time(1:step-1), read_cost(output_cost, 8, 4), color{4},time(1:step-1), read_cost(output_cost, 8, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car9 cost function value')  
    elseif number == 11
        subplot(5,2,1)
        plot(time(1:step-1), read_cost(output_cost, 1, 1), color{1},time(1:step-1), read_cost(output_cost, 1, 2), color{2}, time(1:step-1), read_cost(output_cost, 1, 3), color{3},time(1:step-1), read_cost(output_cost, 1, 4), color{4},time(1:step-1), read_cost(output_cost, 1, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car2 cost function value')
        subplot(5,2,2)
        plot(time(1:step-1), read_cost(output_cost, 2, 1), color{1},time(1:step-1), read_cost(output_cost, 2, 2), color{2}, time(1:step-1), read_cost(output_cost, 2, 3), color{3},time(1:step-1), read_cost(output_cost, 2, 4), color{4},time(1:step-1), read_cost(output_cost, 2, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car3 cost function value')    
        subplot(5,2,3)
        plot(time(1:step-1), read_cost(output_cost, 3, 1), color{1},time(1:step-1), read_cost(output_cost, 3, 2), color{2}, time(1:step-1), read_cost(output_cost, 3, 3), color{3},time(1:step-1), read_cost(output_cost, 3, 4), color{4},time(1:step-1), read_cost(output_cost, 3, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car4 cost function value')
        subplot(5,2,4)
        plot(time(1:step-1), read_cost(output_cost, 4, 1), color{1},time(1:step-1), read_cost(output_cost, 4, 2), color{2}, time(1:step-1), read_cost(output_cost, 4, 3), color{3},time(1:step-1), read_cost(output_cost, 4, 4), color{4},time(1:step-1), read_cost(output_cost, 4, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car5 cost function value')   
        subplot(5,2,5)
        plot(time(1:step-1), read_cost(output_cost, 5, 1), color{1},time(1:step-1), read_cost(output_cost, 5, 2), color{2}, time(1:step-1), read_cost(output_cost, 5, 3), color{3},time(1:step-1), read_cost(output_cost, 5, 4), color{4},time(1:step-1), read_cost(output_cost, 5, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car6 cost function value')
        subplot(5,2,6)
        plot(time(1:step-1), read_cost(output_cost, 6, 1), color{1},time(1:step-1), read_cost(output_cost, 6, 2), color{2}, time(1:step-1), read_cost(output_cost, 6, 3), color{3},time(1:step-1), read_cost(output_cost, 6, 4), color{4},time(1:step-1), read_cost(output_cost, 6, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car7 cost function value')  
        subplot(5,2,7)
        plot(time(1:step-1), read_cost(output_cost, 7, 1), color{1},time(1:step-1), read_cost(output_cost, 7, 2), color{2}, time(1:step-1), read_cost(output_cost, 7, 3), color{3},time(1:step-1), read_cost(output_cost, 7, 4), color{4},time(1:step-1), read_cost(output_cost, 7, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car8 cost function value')
        subplot(5,2,8)
        plot(time(1:step-1), read_cost(output_cost, 8, 1), color{1},time(1:step-1), read_cost(output_cost, 8, 2), color{2}, time(1:step-1), read_cost(output_cost, 8, 3), color{3},time(1:step-1), read_cost(output_cost, 8, 4), color{4},time(1:step-1), read_cost(output_cost, 8, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car9 cost function value')  
        subplot(5,2,9)
        plot(time(1:step-1), read_cost(output_cost, 9, 1), color{1},time(1:step-1), read_cost(output_cost, 9, 2), color{2}, time(1:step-1), read_cost(output_cost, 9, 3), color{3},time(1:step-1), read_cost(output_cost, 9, 4), color{4},time(1:step-1), read_cost(output_cost, 9, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car10 cost function value')  
        subplot(5,2,10)
        plot(time(1:step-1), read_cost(output_cost, 10, 1), color{1},time(1:step-1), read_cost(output_cost, 10, 2), color{2}, time(1:step-1), read_cost(output_cost, 10, 3), color{3},time(1:step-1), read_cost(output_cost, 10, 4), color{4},time(1:step-1), read_cost(output_cost, 10, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car11 cost function value')  
    else
        subplot(6,2,1)
        plot(time(1:step-1), read_cost(output_cost, 1, 1), color{1},time(1:step-1), read_cost(output_cost, 1, 2), color{2}, time(1:step-1), read_cost(output_cost, 1, 3), color{3},time(1:step-1), read_cost(output_cost, 1, 4), color{4},time(1:step-1), read_cost(output_cost, 1, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car2 cost function value')
        subplot(6,2,2)
        plot(time(1:step-1), read_cost(output_cost, 2, 1), color{1},time(1:step-1), read_cost(output_cost, 2, 2), color{2}, time(1:step-1), read_cost(output_cost, 2, 3), color{3},time(1:step-1), read_cost(output_cost, 2, 4), color{4},time(1:step-1), read_cost(output_cost, 2, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car3 cost function value')    
        subplot(6,2,3)
        plot(time(1:step-1), read_cost(output_cost, 3, 1), color{1},time(1:step-1), read_cost(output_cost, 3, 2), color{2}, time(1:step-1), read_cost(output_cost, 3, 3), color{3},time(1:step-1), read_cost(output_cost, 3, 4), color{4},time(1:step-1), read_cost(output_cost, 3, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car4 cost function value')
        subplot(6,2,4)
        plot(time(1:step-1), read_cost(output_cost, 4, 1), color{1},time(1:step-1), read_cost(output_cost, 4, 2), color{2}, time(1:step-1), read_cost(output_cost, 4, 3), color{3},time(1:step-1), read_cost(output_cost, 4, 4), color{4},time(1:step-1), read_cost(output_cost, 4, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car5 cost function value')   
        subplot(6,2,5)
        plot(time(1:step-1), read_cost(output_cost, 5, 1), color{1},time(1:step-1), read_cost(output_cost, 5, 2), color{2}, time(1:step-1), read_cost(output_cost, 5, 3), color{3},time(1:step-1), read_cost(output_cost, 5, 4), color{4},time(1:step-1), read_cost(output_cost, 5, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car6 cost function value')
        subplot(6,2,6)
        plot(time(1:step-1), read_cost(output_cost, 6, 1), color{1},time(1:step-1), read_cost(output_cost, 6, 2), color{2}, time(1:step-1), read_cost(output_cost, 6, 3), color{3},time(1:step-1), read_cost(output_cost, 6, 4), color{4},time(1:step-1), read_cost(output_cost, 6, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car7 cost function value')  
        subplot(6,2,7)
        plot(time(1:step-1), read_cost(output_cost, 7, 1), color{1},time(1:step-1), read_cost(output_cost, 7, 2), color{2}, time(1:step-1), read_cost(output_cost, 7, 3), color{3},time(1:step-1), read_cost(output_cost, 7, 4), color{4},time(1:step-1), read_cost(output_cost, 7, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car8 cost function value')
        subplot(6,2,8)
        plot(time(1:step-1), read_cost(output_cost, 8, 1), color{1},time(1:step-1), read_cost(output_cost, 8, 2), color{2}, time(1:step-1), read_cost(output_cost, 8, 3), color{3},time(1:step-1), read_cost(output_cost, 8, 4), color{4},time(1:step-1), read_cost(output_cost, 8, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car9 cost function value')  
        subplot(6,2,9)
        plot(time(1:step-1), read_cost(output_cost, 9, 1), color{1},time(1:step-1), read_cost(output_cost, 9, 2), color{2}, time(1:step-1), read_cost(output_cost, 9, 3), color{3},time(1:step-1), read_cost(output_cost, 9, 4), color{4},time(1:step-1), read_cost(output_cost, 9, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car10 cost function value')  
        subplot(6,2,10)
        plot(time(1:step-1), read_cost(output_cost, 10, 1), color{1},time(1:step-1), read_cost(output_cost, 10, 2), color{2}, time(1:step-1), read_cost(output_cost, 10, 3), color{3},time(1:step-1), read_cost(output_cost, 10, 4), color{4},time(1:step-1), read_cost(output_cost, 10, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car11 cost function value')  
        subplot(6,2,11)
        plot(time(1:step-1), read_cost(output_cost, 11, 1), color{1},time(1:step-1), read_cost(output_cost, 11, 2), color{2}, time(1:step-1), read_cost(output_cost, 11, 3), color{3},time(1:step-1), read_cost(output_cost, 11, 4), color{4},time(1:step-1), read_cost(output_cost, 11, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car12 cost function value')  
        subplot(6,2,12)
        plot(time(1:step-1), read_cost(output_cost, 12, 1), color{1},time(1:step-1), read_cost(output_cost, 12, 2), color{2}, time(1:step-1), read_cost(output_cost, 12, 3), color{3},time(1:step-1), read_cost(output_cost, 12, 4), color{4},time(1:step-1), read_cost(output_cost, 12, 5), color{5}) 
        xlabel('time [s]')
        ylabel('Cost function value')
        ylim([0, 2*10^5]);
        legend('fuel consumption', 'distance punishment', 'velocity diff from the front car', 'acceleration', 'velocity diff from the following car');
        title('car13 cost function value')  
    end
    saveas(gca, fullfile(strcat(pwd,'/figure/cost_', num2str(number),'_d.fig')), 'fig')       
end

close all;






