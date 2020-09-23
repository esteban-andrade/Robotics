 function plot_robjointpos(history)
          
            if isempty(history)
                return
            end
            clf
            pos = [history.q]';
            plot(pos(:,1:6), '-')
            hold on
            ylabel('Joint angle')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6')
        end