classdef control_viewer < handle
    %
    %    plot MAV states, estimates, and commands
    %
    %--------------------------------
    properties
    	p_handle
    	q_handle
        chi_handle
        phi_handle
        gamma_handle
        alt_handle
        va_handle
        beta_handle
        delta_a_handle
        delta_e_handle
        delta_t_handle
        delta_r_handle
        plot_initialized
        time
        plot_rate
        time_last_plot
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = control_viewer
            self.p_handle = [];
    	    self.q_handle = [];
            self.phi_handle = [];
            self.chi_handle = [];
            self.gamma_handle = [];
            self.alt_handle = [];
            self.va_handle = [];
            self.beta_handle = [];
            self.delta_a_handle = [];
            self.delta_e_handle = [];
            self.delta_t_handle = [];
            self.delta_r_handle = [];
            self.time = 0;
            self.plot_rate = .1;
            self.time_last_plot = self.time;
            self.plot_initialized = 0;    
        end
        %---------------------------
        function self=update(self, true, commanded, delta, Ts)
            
            if self.plot_initialized==0
                figure(3), clf
                subplot(4,3,1)
                    hold on
                    self.q_handle = self.graph_y_ycmd(self.time, true.q, commanded.q, [], 'q');
                subplot(4,3,4)
                    hold on
                    self.gamma_handle = self.graph_y_ycmd(self.time, true.gamma, commanded.gamma, [], '\gamma');
                subplot(4,3,7)
                    hold on
                    self.alt_handle = self.graph_y_ycmd(self.time, true.h, commanded.h, [], 'h');
                subplot(4,3,10)
                    hold on
                    self.va_handle = self.graph_y_ycmd(self.time, true.Va, commanded.Va, [], 'Va');
                subplot(4,3,2)
                    hold on
                    self.p_handle = self.graph_y_ycmd(self.time, true.p, commanded.p, [], 'p');
                subplot(4,3,5)
                    hold on
                    self.phi_handle = self.graph_y_ycmd(self.time, true.phi, commanded.phi, [], '\phi');
                subplot(4,3,8)
                    hold on
                    self.chi_handle = self.graph_y_ycmd(self.time, true.chi, commanded.chi, [], '\chi');
                subplot(4,3,11)
                    hold on
                    self.beta_handle = self.graph_y_ycmd(self.time, true.beta,0, [], '\beta');
                subplot(4,3,3)
                    hold on
                    self.delta_e_handle = self.graph_y(self.time, delta(1), [], '\delta_e');
                subplot(4,3,6)
                    hold on
                    self.delta_t_handle = self.graph_y(self.time, delta(2), [], '\delta_t');
                subplot(4,3,9)
                    hold on
                    self.delta_a_handle = self.graph_y(self.time, delta(3), [], '\delta_a');
                subplot(4,3,12)
                    hold on
                    self.delta_r_handle = self.graph_y(self.time, delta(4), [], '\delta_r');
                self.plot_initialized = 1;
            else
                if self.time >= self.time_last_plot + self.plot_rate
                    self.graph_y_ycmd(self.time, true.q, commanded.q, self.q_handle);
                    self.graph_y_ycmd(self.time, true.gamma, commanded.gamma, self.gamma_handle);
                    self.graph_y_ycmd(self.time, true.h, commanded.h, self.alt_handle);
                    self.graph_y_ycmd(self.time, true.Va, commanded.Va, self.va_handle);
                    self.graph_y_ycmd(self.time, true.p, commanded.p, self.p_handle);
                    self.graph_y_ycmd(self.time, true.phi, commanded.phi, self.phi_handle);
                    self.graph_y_ycmd(self.time, true.chi, commanded.chi, self.chi_handle);
                    self.graph_y_ycmd(self.time, true.beta, 0, self.beta_handle);
                    self.graph_y(self.time, delta(1), self.delta_e_handle);
                    self.graph_y(self.time, delta(2), self.delta_t_handle);
                    self.graph_y(self.time, delta(3), self.delta_a_handle);
                    self.graph_y(self.time, delta(4), self.delta_r_handle);
                    self.time_last_plot = self.time;
                end
                self.time = self.time + Ts;
            end
        end
        %---------------------------
%         function handle = graph_y_yhat_yd(self, t, y, yhat, yd, handle, lab)
%             if isempty(handle)
%                 handle(1)   = plot(t,y,'b');
%                 handle(2)   = plot(t,yhat,'g--');
%                 handle(3)   = plot(t,yd,'r-.');
%                 ylabel(lab)
%                 set(get(gca,'YLabel'),'Rotation',0.0);
%             else
%                 set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
%                 set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
%                 set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
%                 set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
%                 set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
%                 set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
%                 %drawnow
%             end
%         end
        %---------------------------

        function handle = graph_y(self, t, y, handle, lab)
            if isempty(handle)
                handle(1)   = plot(t,y,'b');
                ylabel(lab)
                set(get(gca,'YLabel'),'Rotation',0.0);
            else
                set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
                set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
                %drawnow
            end
        end
        
        function handle = graph_y_ycmd(self, t, y, ycmd, handle, lab)
            if isempty(handle)
                handle(1)   = plot(t,y,'b');
                handle(2)   = plot(t,ycmd,'g--');
                ylabel(lab)
                set(get(gca,'YLabel'),'Rotation',0.0);
            else
                set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
                set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
                set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
                set(handle(2),'Ydata',[get(handle(2),'Ydata'),ycmd]);
                %drawnow
            end
        end
        
    end
end