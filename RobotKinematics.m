classdef RobotKinematics
    % Kinematics and trajectory planning functions for manipulator arms.
    % 
    % Usage: Call RobotKinematics.FunctionName(args)

    methods(Static)
        %% 6-DOF Manipulator: Forward Kinematics
        function [P]=forKin(qdes,joint_num,A)
            % Forward Kinematics for Manipulator Arm (up to 6 DOF) in terms
            % of CF0.
            %
            % qdes: Vector of desired joint positions (radians and meters)
            % joint_num: Joint of Interest (integer)
            % A: 3D array of 2D Frame Transformation A-matrices (DH)

            syms theta1 theta2 theta3 theta4 theta5 theta6;
            theta=[theta1 theta2 theta3 theta4 theta5 theta6];
            theta=theta(1:joint_num);

            % Calculate T-matrix (Composite Frame Transformation Matrix
            % (DH))
            T_local=computeT(A,joint_num);
            T_local=subs(T_local,theta(1:joint_num),qdes(1:joint_num));
            % Substitute value for pi in case symbol was used
            T_local=subs(T_local,pi,3.14159265359);
            P=vpa(T_local(1:3,4),3);
        end
        %% Denavit-Hartenberg (DH) Matrices
        function [T] = computeT(A,n)
            % Calculate Composite Frame Transformation Matrix (DH)
            %
            % A: 3D array of 2D Frame Transformation A-matrices (DH)
            n=size(A,3);
            for ii=1:n
                %Calculate Composite Transformation Matrix
                if ii==1
                    T=A(:,:,ii);
                else
                    T=T*A(:,:,ii);
                end
            end
        end
        
        function [A] = computeA(theta,d,a,alpha)
            % Calculate Frame Transformation Matrix (DH)
            %
            % DH Parameters:
            % theta: Rotation about z-axis (radians)
            % d: Translation about z-axis (meters)
            % a: Translation about x-axis (meters)
            % alpha: Rotation about x-axis (radians)
            A=computeRzh(theta)*computeTzh(d)*computeTxh(a)*computeRxh(alpha);
        end
        %% Trajectory Generation: Cubic Polynomial
        function [a]=a_coeffs(t0,tf,q0,qf,dq0,dqf)
            % Solve for a-coefficients required for cubic polynomial 
            % for defining position (p.197, Spong)
            %
            % t0: Start time (seconds)
            % tf: End time(seconds)
            % q0: Start joint position (radians)
            % qf: End joint position (radians)
            % q0: Start joint velocity (radians/second)
            % qf: End joint velocity (radians/second)
            a=[q0;                                          %a0
               dq0;                                         %a1
               (3*(qf-q0)-(2*dq0+dqf)*(tf-t0))/((tf-t0)^2); %a2
               (2*(q0-qf)+(dq0+dqf)*(tf-t0))/((tf-t0)^3)];  %a3
        end

        function [q]=joint_pos(a,t)
            % Calculate position from cubic polynomial
            %
            % a: Vector of a-coefficients required for cubic polynomial 
            % t: time (seconds)
            a_flip=flip(a); %Put in descending order (highest order term first)
            q=polyval(a_flip,t);
        end

        function [dq]=joint_vel(a,t)
            % Calculate velocity from cubic polynomial
            %
            % a: Vector of a-coefficients required for cubic polynomial 
            % t: time (seconds)
            a_flip=flip(a); %Put in descending order (highest order term first)
            a3=a_flip(1); a2=a_flip(2); a1=a_flip(3);
            dq=a1+2*a2*t+3*a3*(t.^2);
        end

        function [ddq]=joint_accel(a,t)
            % Calculate acceleration from cubic polynomial
            %
            % a: Vector of a-coefficients required for cubic polynomial 
            % t: time (seconds)
            a_flip=flip(a); %Put in descending order (highest order term first)
            a3=a_flip(1); a2=a_flip(2);
            ddq=2*a2+6*a3*t;
        end
        %% Rotation and Transformation Matrices
        function [Rxh] = computeRxh(theta)
            % Compute Basic Homogeneous Transform Matrix for
            % rotation of theta (radians) about x-axis
            Rxh=[1 0 0 0; ...
                 0 cos(theta) -sin(theta) 0; ...
                 0 sin(theta) cos(theta) 0; ...
                 0 0 0 1;];
        end

        function [Ryh] = computeRyh(theta)
            % Compute Basic Homogeneous Transform Matrix for
            % rotation of theta (radians) about y-axis
            Ryh=[cos(theta) 0 sin(theta) 0; ...
                 0 1 0 0; ...
                 -sin(theta) 0 cos(theta) 0; ...
                 0 0 0 1];
        end

        function [Rzh] = computeRzh(theta)
            % Compute Basic Homogeneous Transform Matrix for
            % rotation of theta (radians) about z-axis
            Rzh=[cos(theta) -sin(theta) 0 0; ...
                 sin(theta) cos(theta) 0 0; ...
                 0 0 1 0; ...
                 0 0 0 1];
        end

        function [Txh] = computeTxh(d)
            % Calculate Basic Homogeneous Transform Matrix for
            % translation of d (meters) along x-axis
            Txh=[1 0 0 d; ...
                 0 1 0 0; ...
                 0 0 1 0; ...
                 0 0 0 1];
        end

        function [Tyh] = computeTyh(d)
            % Calculate Basic Homogeneous Transform Matrix for
            % translation of d (meters) along y-axis
            Tyh=[1 0 0 0; ...
                 0 1 0 d; ...
                 0 0 1 0; ...
                 0 0 0 1];
        end

        function [Tzh] = computeTzh(d)
            % Calculate Basic Homogeneous Transform Matrix for
            % translation of d (meters) along z-axis
            Tzh=[1 0 0 0; ...
                 0 1 0 0; ...
                 0 0 1 d; ...
                 0 0 0 1];
        end
        %% Planar Arm: Forward & Inverse Kinematics
        function [x,y]=planar_forKin(q,l)
            % Calculate forward kinematics for Planar Arm 
            % robot geometrically.
            %
            % q: Vector of Joint Positions (radians)
            % l: Vector of Link Lengths (meters)
            q1=q(1);q2=q(2);l1=l(1);l2=l(2);
            x=l1*cos(q1)+l2*cos(q1+q2);
            y=l1*sin(q1)+l2*sin(q1+q2);
        end

        function [q1,q2]=planar_invKin(x,y,l,elbow_up)
            % Calculate inverse kinematics for Planar Arm robot
            % geometrically.
            %
            % x: End Effector x-Position (meters)
            % y: End Effector y-Position (meters)
            % l: Vector of Link Lengths (meters)
            % elbow_up: 0 or 1
            l1=l(1);l2=l(2);
            D=(x^2+y^2-l1^2-l2^2)/(2*l1*l2);
            if elbow_up==1
                q2=atan2(-sqrt(1-D^2),D);
            else %elbow is down
                q2=atan2(sqrt(1-D^2),D);
            end
            q1=atan2(y,x)-atan2((l2*sin(q2)),(l1+l2*cos(q2)));
        end
    end
end

