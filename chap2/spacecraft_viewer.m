classdef spacecraft_viewer < handle
    %
    %    Create spacecraft animation
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
        function self = spacecraft_viewer
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
            self.plot_initialized = 0;           
        end
        %---------------------------
        function self=update(self, state)
            if self.plot_initialized==0
                figure(1); clf;
                self=self.drawBody(state.pn, state.pe, -state.h,...
                                   state.phi, state.theta, state.psi);
                title('Spacecraft')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                view(32,47)  % set the vieew angle for figure
                axis([-10,10,-10,10,-10,10]);
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
            else
                set(self.body_handle,'Vertices',Vertices','Faces',self.Faces);
                drawnow
            end
                set(self.body_handle.Parent, 'XLim',[pe-30,pe+30])

                set(self.body_handle.Parent, 'YLim',[pn-30,pn+30])

                set(self.body_handle.Parent, 'ZLim',[-pd-30,-pd+30])

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
        function [V, F, colors] = define_spacecraft(self)
            % Define the vertices (physical location of vertices)
            V = [...
                %The "fore" Verticies
                 2    0    0;...  % point 1
                 1    1    -1;... % point 2
                 1    -1   -1;...  % point 3
                 1    -1    1;...  % point 4
                 1    1    1;... % point 5

                %The Aft Verticie
                 -4    0    0;...  % point 6

                %The Wing Verticies
                 0    3    0;... % -1   -1   -2;... % point 7
                 -1.5    3   0 ;... % -1    1   -2;... % point 8
                 -1.5    -3    0;... %1.5  1.5  0;... % point 9 <----UPDATED AS OF 1/30/2023
                 0    -3      0;... % 1.5 -1.5  0;... % point 10

                %The Horizontal Stabilizer Verticies
                 -3    1.5    0;... % -1.5 -1.5  0;... % point 11
                 -4    1.5    0;... % -1.5  1.5  0;... % point 12
                -4    -1.5    0;...% point 13
                -3    -1.5    0;...% point 14

                %The Vertical Stabilizer Verticies
                -3    0    0;...% point 15
                -4    0    -1.5;... % point 16
            ]';

            % define faces as a list of vertices numbered above
            F = [...
                    1, 2, 3, NaN;... % 1
                    1, 3, 4, NaN;... % 2
                    1, 5, 4, NaN;... % 3
                    1, 2, 5, NaN;... % 4
                    
                    3, 4, 6, NaN;... % 5
                    5, 4, 6, NaN;... % 6
                    2, 5, 6, NaN;... % 7
                    2, 3, 6, NaN;... % 8

                    7, 8, 9, 10;... %9 <----UPDATED AS OF 1/30/2023
                    11, 12, 13, 14;... %10
                    6, 15, 16, NaN;... %11

                    ];

            % define colors for each face    
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];

            colors = [...
                myyellow;... % noes_1
                myblue;...   % noes_2
                myred;...   % noes_3
                mycyan;...   % noes_4

                myyellow;... % body_1
                myblue;...   % body_2
                myred;...%myblue;...   % body_3
                mycyan;...%myblue;...   % body_4

                myyellow;... % Wing_1
                myblue;...   % Wing_2
                myred;...%myblue;...   % Wing_3


                %myblue;...   % top
                %mygreen;...  % bottom
                ];
        end
    end
end