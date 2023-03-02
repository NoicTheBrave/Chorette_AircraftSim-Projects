classdef mav_viewer < handle
    %
    %    Create Aircraft animation
    %
    %--------------------------------
    properties
        body_handle
    	Vertices
    	Faces
    	facecolors
        plot_initialized
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_viewer
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_MAV();
            self.plot_initialized = 0;           
        end
        function self=set_view(self, azmuth, elevation)
            figure(1);
            view(azmuth, elevation)
        end
        %---------------------------
        function self=update(self, state)
            if self.plot_initialized==0
                figure(1); clf;
                self=self.drawBody(state.pn, state.pe, -state.h,...
                                   state.phi, state.theta, state.psi);
                title('MAV')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                view(32,47)  % set the vieew angle for figure
                %axis([-10,10,-10,10,-10,10]);
                hold on
                grid on
                self.plot_initialized = 1;
            else
                self=self.drawBody(state.pn, state.pe, -state.h,... 
                                   state.phi, state.theta, state.psi);

            end
        end
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            Vertices = self.rotate(self.Vertices, phi, theta, psi);   % rotate rigid body  
            Vertices = self.translate(Vertices, pn, pe, pd);     % translate after rotation
            % transform vertices from NED to ENU (for matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            Vertices = R*Vertices;
            if isempty(self.body_handle)
                self.body_handle = patch('Vertices', Vertices', 'Faces', self.Faces,...
                                             'FaceVertexCData',self.facecolors,...
                                             'FaceColor','flat');
                set(self.body_handle.Parent, 'XLim',[pe-10,pe+10])
                set(self.body_handle.Parent, 'YLim',[pn-10,pn+10])
                set(self.body_handle.Parent, 'ZLim',[-pd-10,-pd+10])
            else
                set(self.body_handle,'Vertices',Vertices','Faces',self.Faces);
                set(self.body_handle.Parent, 'XLim',[pe-10,pe+10])
                set(self.body_handle.Parent, 'YLim',[pn-10,pn+10])
                set(self.body_handle.Parent, 'ZLim',[-pd-10,-pd+10])
                hold on
                drawnow
            end
        end 
        %---------------------------
        function pts=rotate(self, pts, phi, theta, psi)
            % define rotation matrix (right handed)
            R_roll = [...
                        1, 0, 0;...
                        0, cos(phi), sin(phi);...
                        0, -sin(phi), cos(phi)];
            R_pitch = [...
                        cos(theta), 0, -sin(theta);...
                        0, 1, 0;...
                        sin(theta), 0, cos(theta)];
            R_yaw = [...
                        cos(psi), sin(psi), 0;...
                        -sin(psi), cos(psi), 0;...
                        0, 0, 1];
            R = R_roll*R_pitch*R_yaw;   % inertial to body
            R = R';  % body to inertial
            % rotate vertices
            pts = R*pts;
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pn, pe, pd)
            pts = pts + repmat([pn;pe;pd],1,size(pts,2));
        end
        %---------------------------
        function [V, F, colors] = define_MAV(self)
            %parameters for drawing aircraft
            size = .01;% scale size
            %vertices
            fuse_l1 = 140;
            fuse_l2 = 80;
            fuse_l3 = 300;
            fuse_w  = 40;
            fuse_h  = 40;
            wing_l  = 120;
            wing_w  = 400;
            tail_h  = 80;
            tailwing_w = 200;
            tailwing_l = 60;
            
            V = [...
                fuse_l1+wing_l/2 0 0;...%1
                wing_l/2+fuse_l2 fuse_w/2 fuse_h/2;...%2
                wing_l/2+fuse_l2 -fuse_w/2 fuse_h/2;...%3
                wing_l/2+fuse_l2 -fuse_w/2 -fuse_h/2;...%4
                wing_l/2+fuse_l2 fuse_w/2 -fuse_h/2;...%5
                wing_l/2-fuse_l3 0 0;...%6
                wing_l/2 wing_w/2 0;...%7
                -wing_l/2 wing_w/2 0;...%8
                -wing_l/2 -wing_w/2 0;...%9
                wing_l/2 -wing_w/2 0;...%10
                wing_l/2-fuse_l3+tailwing_l tailwing_w/2 0;...%11
                wing_l/2-fuse_l3 tailwing_w/2 0;...%12
                wing_l/2-fuse_l3 -tailwing_w/2 0;...%13
                wing_l/2-fuse_l3+tailwing_l -tailwing_w/2 0;...%14
                wing_l/2-fuse_l3+tailwing_l 0 0;...%15
                wing_l/2-fuse_l3 0 -tail_h]';%16

            F = [...
                %fuse
                1 2 3 1;...%nose-up
                1 3 4 1;...%nose-east
                1 2 5 1;...%nose-west
                1 4 5 1;...%nose-down
                2 3 6 2;...%top
                3 4 6 3;...%left side
                4 5 6 4;...%bottom
                2 5 6 2;...%right side
                %wing
                7 8 9 10;...
                %rear Stab. & tail
                11 12 13 14;...%rear stab.
                15 16 6 15];%tail  

            % define colors for each face    
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];
            
            colors = [...
                myred;...%nose-up
                myred;...%nose-east
                myred;...%nose-west
                myred;...%nose-down
                mycyan;...%top
                mycyan;...%left side
                mycyan;...%bottom
                mycyan;...%right side
                myblue;...%wing
                myyellow;...%rear stab.
                myyellow];%tail
            V = size*V;   % rescale vertices
        end
    end
end