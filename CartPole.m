classdef CartPole < handle
    properties
        x             % x coordinate
        y             %  y coordinate
        theta       % pole theta
        dx           % x velocity
        dtheta      % pole dtheta
        satLevel    %satuation Level
        
        cart_w      % cart width    1
        cart_h      % cart height   0.7
        cart_weight % cart weight   0.5
        
        pole_w      % pole width    0.15
        pole_h      % pole height   1.5
        pole_weight % pole weight   0.2

        pole_I         % pole Inertia   1/3*0.2*1.5^2
       
        pole_l        % com of pole to the joint    0.75
        
        friction   % friction coefficient   0.1
        
        linear   % determine if use linear model
        
        
        As; % system matrix
        Bs;
        Cs;
        g;  %9.8
    end
    
    properties (Access = protected)
        action
        lastState% last state;
        
        cartHandle;
        poleHandle;
        lwHandle;
        rwHandle;
    end
    
    methods
        
        function self = CartPole(startPos,theta,dx,dtheta,cart_weight,pole_weight)
            self.setAgent(startPos,theta,dx,dtheta);
            self.size(1,0.7,0.15,1.5);
            self.weight(cart_weight,pole_weight);
            self.friction=0.1;
            self.g=9.8;
            self.inertia();
            self.systemMatrix();
        end
        
        function systemMatrix(self)
            p=self.pole_I*(self.cart_weight+self.pole_weight)+self.cart_weight*self.pole_weight*self.pole_l^2;
            self.As=[0 1 0 0;
                0 -(self.pole_I+self.pole_weight*self.pole_l^2)*self.friction/p  (self.pole_weight^2*self.g*self.pole_l^2)/p 0;
                0 0 0 1;
                0 -(self.pole_weight*self.pole_l*self.friction)/p self.pole_weight*self.g*self.pole_l*(self.pole_weight+self.cart_weight)/p 0];
            self.Bs=[0; (self.pole_I+self.pole_weight*self.pole_l^2)/p;0;self.pole_weight*self.pole_l/p];
            self.Cs= [1 0 0 0;0 0 1 0];
        end
        
        function setAgent(self,startPos,theta,dx,dtheta)
            self.x = startPos.x;
            self.y = startPos.y;
            self.theta = theta;
            self.dx = dx;
            self.dtheta=dtheta;
        end
        
        
        function inertia(self)
            self.pole_I=1/3*self.pole_weight*self.pole_h^2;
        end

        function weight(self,cart_weight,pole_weight)
            self.cart_weight=cart_weight;
            self.pole_weight=pole_weight;
            
        end
        
        function size(self,cart_w,cart_h,pole_w,pole_h)
            self.cart_w=cart_w;
            self.cart_h=cart_h;
            self.pole_w=pole_w;
            self.pole_h=pole_h;
            self.pole_l=self.pole_h/2;
        end
        
        
        function step(self,tspan,action)
            self.lastState=[self.x,self.dx,self.theta,self.dtheta];
            sat=utils('sat');
            self.action=sat(action,self.satLevel);
            [t, state] = ode45(@(t, state)self.dynamics(t, state), tspan, self.lastState);
            self.x=state(end,1);
            self.dx=state(end,2);
            self.theta=self.roundTheta(state(end,3));
            self.dtheta=state(end,4);
            %action is for applied force
        end
        
        function setSatLevel(self,level)
            self.satLevel=level;
        end
        function setModel(self,linear)
            self.linear=linear;
        end
        
        function theta=roundTheta(self,t)
            idx=-floor(t/(2*pi));
            theta=t+idx*2*pi;
        end
        % refer to the article https://zhuanlan.zhihu.com/p/54071212
        function ds = dynamics(self,t,state)
            xl=state(1);
            dxl=state(2);
            thetal=self.roundTheta(state(3));
            dthetal=state(4);
       
            
            %nonlinear model
            if ~self.linear
                A=[self.cart_weight+self.pole_weight self.pole_weight*self.pole_l*cos(thetal);
                    self.pole_weight*self.pole_l*cos(thetal)  self.pole_I+self.pole_weight*self.pole_l^2];
                
                B=[self.action+self.pole_weight*self.pole_l*dthetal^2*sin(thetal)-self.friction*dxl;
                    -self.pole_weight*self.g*self.pole_l*sin(thetal)];
            else
                %   %linear model
                phi=thetal-pi;
                A=[self.cart_weight+self.pole_weight -self.pole_weight*self.pole_l;
                    -self.pole_weight*self.pole_l self.pole_I+self.pole_weight*self.pole_l^2];
                B=[self.action-self.friction*dxl;
                    self.pole_weight*self.g*self.pole_l*phi];
            end
            
            dd=inv(A)*B;
            ds=[dxl;dd(1);dthetal;dd(2)];
        end
        
        
        
        function plot(self,handle)
            cart_corners=[-self.cart_w/2,0;
                self.cart_w/2,0;
                self.cart_w/2,self.cart_h;
                -self.cart_w/2,self.cart_h; ];
            cart_z_org=self.y;
            cart_corners=[self.x;cart_z_org]+cart_corners';
            if isempty(self.cartHandle)
                self.cartHandle=fill(handle,cart_corners(1,:),cart_corners(2,:),'g');
            else
                set(self.cartHandle,'XData',cart_corners(1,:),'YData',cart_corners(2,:));
            end
            
            rotMat2=utils('rotMat2');
            rm=rotMat2(self.theta);
            
            
            pole_corners=[self.pole_w/2,0;
                -self.pole_w/2,0;
                -self.pole_w/2,-self.pole_h;
                self.pole_w/2,-self.pole_h;];
            
            
            
            pole_corners=[self.x;self.cart_h+cart_z_org]+rm*pole_corners';
            
            if isempty(self.poleHandle)
                self.poleHandle=fill(handle,pole_corners(1,:),pole_corners(2,:),'r');
            else
                set(self.poleHandle,'XData',pole_corners(1,:),'YData',pole_corners(2,:));
            end
            
            
        end
        
        
    end
    
end