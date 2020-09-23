  function plot_robjointvel(history)
          
            if isempty(history)
                return
            end
            clf
            vel = [history.qp]';
            plot(vel(:,1:6), '-')
            hold on
            ylabel('Joint velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_5', '\omega_6')
        end
