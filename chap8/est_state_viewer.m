classdef est_state_viewer < handle
    %
    %    plot MAV states, estimates, and commands
    %
    %--------------------------------
    properties
    	p_handle
    	q_handle
        r_handle
        h_handle
        va_handle
        phi_handle
        theta_handle
        pn_handle
        pe_handle
        chi_handle
        Vg_handle
        plot_initialized
        time
        plot_rate
        time_last_plot
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = est_state_viewer
            self.time = 0;
            self.plot_rate = .1;
            self.time_last_plot = self.time;
            self.plot_initialized = 0;    
        end
        %---------------------------
        function self=update(self, true, estimated_a, estimated_b, Ts)
       
            if self.plot_initialized==0
                figure(4), clf
                subplot(4,3,1)
                    hold on
                    self.p_handle = self.graph_y_yhata_yhatb_angle(self.time, true.p, estimated_a.p, estimated_b.p, [], 'p deg');
                subplot(4,3,2)
                    hold on
                    self.q_handle = self.graph_y_yhata_yhatb_angle(self.time, true.q, estimated_a.q, estimated_b.q, [], 'q deg');
                subplot(4,3,3)
                    hold on
                    self.r_handle = self.graph_y_yhata_yhatb_angle(self.time, true.r, estimated_a.r, estimated_b.r, [], 'r deg');
                subplot(4,3,4)
                    hold on
                    self.h_handle = self.graph_y_yhata_yhatb(self.time, true.h, estimated_a.h, estimated_b.h, [], 'h');
                subplot(4,3,5)
                    hold on
                    self.va_handle = self.graph_y_yhata_yhatb(self.time, true.Va, estimated_a.Va, estimated_b.Va, [], 'Va');
                subplot(4,3,6)
                    hold on
                    self.phi_handle = self.graph_y_yhata_yhatb_angle(self.time, true.phi, estimated_a.phi, estimated_b.phi, [], '\phi');
                subplot(4,3,7)
                    hold on
                    self.theta_handle = self.graph_y_yhata_yhatb_angle(self.time, true.theta, estimated_a.theta, estimated_b.theta, [], '\theta');
                
                subplot(4,3,8)
                    hold on
                    self.pn_handle = self.graph_y_yhata_yhatb_angle(self.time, true.pn, estimated_a.pn, estimated_b.pn, [], 'pn');
                subplot(4,3,9)
                    hold on
                    self.pe_handle = self.graph_y_yhata_yhatb_angle(self.time, true.pe, estimated_a.pe, estimated_b.pe, [], 'pe');
                subplot(4,3,10)
                    hold on
                    self.chi_handle = self.graph_y_yhata_yhatb_angle(self.time, true.chi, estimated_a.chi, estimated_b.chi, [], '\chi');
                subplot(4,3,11)
                    hold on
                    self.Vg_handle = self.graph_y_yhata_yhatb_angle(self.time, true.Vg, estimated_a.Vg, estimated_b.Vg, [], 'Vg');
%                 subplot(4,3,12)
%                     hold on
%                     self.theta_handle = self.graph_y_yhata_yhatb_angle(self.time, true.theta, estimated_a.theta, estimated_b.theta, [], '\theta');


                self.plot_initialized = 1;
            else
                if self.time >= self.time_last_plot + self.plot_rate
                    self.graph_y_yhata_yhatb_angle(self.time, true.p, estimated_a.p, estimated_b.p, self.p_handle);
                    self.graph_y_yhata_yhatb_angle(self.time, true.q, estimated_a.q, estimated_b.q, self.q_handle);
                    self.graph_y_yhata_yhatb_angle(self.time, true.r, estimated_a.r, estimated_b.r, self.r_handle);
                    self.graph_y_yhata_yhatb(self.time, true.h, estimated_a.h, estimated_b.h, self.h_handle);
                    self.graph_y_yhata_yhatb(self.time, true.Va, estimated_a.Va, estimated_b.Va, self.va_handle);
                    self.graph_y_yhata_yhatb_angle(self.time, true.phi, estimated_a.phi, estimated_b.phi, self.phi_handle);
                    self.graph_y_yhata_yhatb_angle(self.time, true.theta, estimated_a.theta, estimated_b.theta, self.theta_handle);
                    self.graph_y_yhata_yhatb(self.time, true.pn, estimated_a.pn, estimated_b.pn, self.pn_handle);
                    self.graph_y_yhata_yhatb(self.time, true.pe, estimated_a.pe, estimated_b.pe, self.pe_handle);
                    self.graph_y_yhata_yhatb_angle(self.time, true.chi, estimated_a.chi, estimated_b.chi, self.chi_handle);
                    self.graph_y_yhata_yhatb(self.time, true.Vg, estimated_a.Vg, estimated_b.Vg, self.Vg_handle);
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

        function handle = graph_y_yhata_yhatb(self, t, y, yhata, yhatb, handle, lab)
            if isempty(handle)
                handle(1)   = plot(t,y,'b');
                handle(2)   = plot(t, yhata, 'g--')
                handle(3)   = plot(t, yhatb, 'r--')
                ylabel(lab)
                set(get(gca,'YLabel'),'Rotation',0.0);
            else
                set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
                set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
                set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
                set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhata]);
                set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
                set(handle(3),'Ydata',[get(handle(3),'Ydata'),yhatb]);
                %drawnow
            end
        end

        function handle = graph_y_yhata_yhatb_angle(self, t, y, yhata, yhatb, handle, lab)
            if isempty(handle)
                handle(1)   = plot(t,rad2deg(y),'b');
                handle(2)   = plot(t, rad2deg(yhata), 'g--')
                handle(3)   = plot(t, rad2deg(yhatb), 'r--')
                ylabel(lab)
                %set(get(gca,'YLabel'),'Rotation',0.0);
            else
                set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
                set(handle(1),'Ydata',[get(handle(1),'Ydata'),rad2deg(y)]);
                set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
                set(handle(2),'Ydata',[get(handle(2),'Ydata'),rad2deg(yhata)]);
                set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
                set(handle(3),'Ydata',[get(handle(3),'Ydata'),rad2deg(yhatb)]);
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